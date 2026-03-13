/*
 * la_test.ino
 *
 * Logic analyser — continuous binary capture, 1–8 channels
 * Edge-triggered ISR capture → ping-pong buffer → USB flash drive (.bin)
 *
 *  Capture pins: 14, 15, 16, 17, 20, 21, 22, 23  (use first num_channels)
 *  ENC_A: 3  |  ENC_B: 4  |  ENC_SW: 2  |  BLK: 7
 *  TFT pins defined in TFT_eSPI User_Setup.h
 *
 * Encoder (disarmed): rotate = select channel count 1–8
 * Button:             arm → start capture + open file
 *                     press again → stop + close file
 * No flash drive:     capture runs in RAM only, nothing written
 *
 * TFT:
 *   Disarmed — settings screen (channel count, pin list, next file)
 *   Armed    — event counter, updated 1 Hz
 *
 * Binary file format (la01.bin … la99.bin):
 *   Header  24 bytes: 'L','A', version=2, channels=N, pins[8], init_states[8],
 *                     uint32_t cpu_hz
 *   Events   6 bytes: uint32_t timestamp (DWT cycles), uint8_t pin, uint8_t rising
 *   All little-endian (ARM native)
 *   timestamp / (cpu_hz / 1e6) = microseconds
 *
 * File numbering: on drive mount, scans for highest la*.bin, continues from there.
 *
 * Memory:
 *   pp_buf  2 × 32768 events × 6 B = 384 KB  (OCRAM via DMAMEM)
 *
 * EEPROM byte 0: log file counter  |  byte 1: num_channels
 */

#include <TFT_eSPI.h>      // MUST be before USBHost_t36
#include <Encoder.h>
#include <USBHost_t36.h>
#include <EEPROM.h>

// #define DEBUG

#define TFT_BLK  7
#define ENC_SW   2
#define ENC_A    3
#define ENC_B    4

static const uint8_t  LA_PIN[8] = {14, 15, 16, 17, 20, 21, 22, 23};
static const uint16_t CH_COL[8] = {
  TFT_GREEN, TFT_CYAN, TFT_YELLOW, TFT_MAGENTA,
  TFT_ORANGE, 0x567F, TFT_GREENYELLOW, TFT_PINK
};

#define SCR_W    160
#define SCR_H    128
#define STATUS_H 8

// ── Event ─────────────────────────────────────────────────────────
struct __attribute__((packed)) Event {
  uint32_t timestamp;  // DWT cycles; divide by (cpu_hz/1e6) for µs
  uint8_t  pin;        // channel index 0–(num_channels-1)
  uint8_t  rising;     // 1 = rising, 0 = falling
};

// ── Ping-pong buffer (OCRAM — 384 KB) ────────────────────────────
#define BUF_HALF  32768

DMAMEM static volatile Event pp_buf[2][BUF_HALF];
static volatile uint8_t  pp_active      = 0;
static volatile uint32_t pp_pos         = 0;
static volatile bool     pp_ready[2]    = {false, false};
static volatile uint32_t cap_total      = 0;
static volatile uint32_t overflow_count = 0;

// ── App state ─────────────────────────────────────────────────────
static uint8_t num_channels = 8;
static bool    armed        = false;
static uint8_t init_state[8];
static volatile uint8_t ch_state[8];

// ── Encoder ───────────────────────────────────────────────────────
Encoder        myEnc(ENC_A, ENC_B);
static int32_t enc_last = 0;

// ── USB + file ────────────────────────────────────────────────────
USBHost        myusb;
USBDrive       myDrive(myusb);
USBFilesystem  myFS(myusb);

static bool     drive_ready  = false;
static uint8_t  log_file_num = 0;
static char     last_log[14] = "";
static File     log_file;
static uint32_t bytes_written = 0;

TFT_eSPI tft;

// ── ISRs ──────────────────────────────────────────────────────────
static FASTRUN void isr_capture(uint8_t pin_idx) {
  uint32_t t  = ARM_DWT_CYCCNT;
  uint8_t  val = ch_state[pin_idx] ^ 1;
  ch_state[pin_idx] = val;

  uint8_t  h   = pp_active;
  uint32_t pos = pp_pos;
  pp_buf[h][pos].timestamp = t;
  pp_buf[h][pos].pin       = pin_idx;
  pp_buf[h][pos].rising    = val;
  pos++;
  cap_total++;

  if (pos >= BUF_HALF) {
    uint8_t next = h ^ 1;
    if (!pp_ready[next]) {
      pp_ready[h] = true;
      pp_active   = next;
      pp_pos      = 0;
    } else {
      overflow_count++;
      pp_pos = 0;
    }
  } else {
    pp_pos = pos;
  }
}

void isr0() { isr_capture(0); }
void isr1() { isr_capture(1); }
void isr2() { isr_capture(2); }
void isr3() { isr_capture(3); }
void isr4() { isr_capture(4); }
void isr5() { isr_capture(5); }
void isr6() { isr_capture(6); }
void isr7() { isr_capture(7); }

static void (*const ISR_TABLE[8])() = {isr0,isr1,isr2,isr3,isr4,isr5,isr6,isr7};

static void attach_isrs() {
  for (uint8_t i = 0; i < num_channels; i++)
    attachInterrupt(digitalPinToInterrupt(LA_PIN[i]), ISR_TABLE[i], CHANGE);
}

static void detach_isrs() {
  for (uint8_t i = 0; i < num_channels; i++)
    detachInterrupt(digitalPinToInterrupt(LA_PIN[i]));
}

// ── USB file helpers ──────────────────────────────────────────────
static uint8_t scan_log_files() {
  char fname[14];
  uint8_t highest = 0;
  for (uint8_t n = 1; n <= 99; n++) {
    snprintf(fname, sizeof(fname), "la%02d.bin", n);
    if (myFS.exists(fname)) highest = n;
  }
  return highest;
}

struct __attribute__((packed)) FileHeader {
  uint8_t  magic[2];       // 'L', 'A'
  uint8_t  version;        // 2
  uint8_t  channels;       // num_channels at arm time
  uint8_t  pins[8];        // LA_PIN[0..7]
  uint8_t  init_states[8]; // pin levels at arm time
  uint32_t cpu_hz;         // F_CPU
};  // 24 bytes

static bool log_open() {
  log_file_num = (log_file_num >= 99) ? 1 : log_file_num + 1;
  EEPROM.write(0, log_file_num);

  snprintf(last_log, sizeof(last_log), "la%02d.bin", log_file_num);
  log_file = myFS.open(last_log, FILE_WRITE);
  if (!log_file) {
    snprintf(last_log, sizeof(last_log), "OPEN FAIL");
    return false;
  }

  FileHeader hdr;
  hdr.magic[0] = 'L'; hdr.magic[1] = 'A';
  hdr.version  = 2;   hdr.channels = num_channels;
  for (uint8_t i = 0; i < 8; i++) hdr.pins[i]       = LA_PIN[i];
  for (uint8_t i = 0; i < 8; i++) hdr.init_states[i] = init_state[i];
  hdr.cpu_hz = (uint32_t)F_CPU;
  log_file.write((const uint8_t*)&hdr, sizeof(FileHeader));
  bytes_written = sizeof(FileHeader);
  return true;
}

static void log_flush_ready() {
  for (uint8_t h = 0; h < 2; h++) {
    if (pp_ready[h] && log_file) {
      log_file.write((const uint8_t*)pp_buf[h], (uint32_t)BUF_HALF * sizeof(Event));
      bytes_written += (uint32_t)BUF_HALF * sizeof(Event);
      pp_ready[h] = false;
    }
  }
}

static void log_close_final() {
  log_flush_ready();
  noInterrupts();
  uint8_t  fh  = pp_active;
  uint32_t fpo = pp_pos;
  interrupts();
  if (fpo > 0 && log_file) {
    log_file.write((const uint8_t*)pp_buf[fh], (uint32_t)fpo * sizeof(Event));
    bytes_written += (uint32_t)fpo * sizeof(Event);
  }
  if (log_file) { log_file.close(); log_file = File(); }
}

// ── TFT ───────────────────────────────────────────────────────────
static void draw_status() {
  tft.fillRect(0, 0, SCR_W, STATUS_H, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);

  if (armed) {
    tft.setTextColor(overflow_count > 0 ? TFT_RED : TFT_GREEN, TFT_BLACK);
    tft.printf("ARM CH:%d %luev %luK OVF:%lu",
               num_channels, cap_total, bytes_written >> 10, overflow_count);
  } else if (cap_total > 0) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (last_log[0] && last_log[0] != 'O')
      tft.printf("DONE CH:%d %luev %-8s", num_channels, cap_total, last_log);
    else
      tft.printf("DONE CH:%d %luev (no file)", num_channels, cap_total);
  } else {
    tft.setTextColor(drive_ready ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.printf("READY CH:%d %s",
               num_channels, drive_ready ? "LOG:RDY" : "NO DRIVE");
  }
}

static void draw_settings() {
  tft.fillScreen(TFT_BLACK);
  draw_status();

  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(2, STATUS_H + 4);
  tft.printf("CH:%d  enc=sel  btn=arm", num_channels);

  // Active pin labels
  tft.setCursor(2, STATUS_H + 16);
  for (uint8_t i = 0; i < num_channels; i++) {
    tft.setTextColor(CH_COL[i], TFT_BLACK);
    tft.printf("%d ", LA_PIN[i]);
  }
  // Dimmed inactive pins
  if (num_channels < 8) {
    tft.setTextColor(0x2104, TFT_BLACK);
    for (uint8_t i = num_channels; i < 8; i++)
      tft.printf("%d ", LA_PIN[i]);
  }

  // Drive / next file
  tft.setCursor(2, STATUS_H + 28);
  if (drive_ready) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    uint8_t next = (log_file_num >= 99) ? 1 : log_file_num + 1;
    tft.printf("Next: la%02d.bin", next);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("No drive — RAM only");
  }
}

static void draw_armed_counter() {
  tft.setTextSize(2);
  tft.setTextColor(overflow_count > 0 ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.setCursor(0, STATUS_H + 20);
  tft.printf("%-10lu", cap_total);
  draw_status();
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  pinMode(ENC_SW, INPUT_PULLUP);

  for (uint8_t i = 0; i < 8; i++) {
    pinMode(LA_PIN[i], INPUT);
    init_state[i] = 0;
    ch_state[i]   = 0;
  }

  num_channels = EEPROM.read(1);
  if (num_channels < 1 || num_channels > 8) num_channels = 8;

  log_file_num = EEPROM.read(0);
  if (log_file_num > 99) log_file_num = 0;

  enc_last = myEnc.read();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  draw_settings();

  myusb.begin();
}

// ── Main loop ─────────────────────────────────────────────────────
void loop() {
  myusb.Task();

  // Drive mount
  if (myFS && !drive_ready) {
    drive_ready  = true;
    log_file_num = scan_log_files();
    draw_settings();
  }
  // Drive unmount
  if (!myFS && drive_ready) {
    drive_ready = false;
    if (armed) {
      detach_isrs();
      armed    = false;
      if (log_file) { log_file.close(); log_file = File(); }
    }
    draw_settings();
  }

  // Flush ping-pong halves while armed
  if (armed) log_flush_ready();

  // Encoder — channel select (disarmed only)
  if (!armed) {
    int32_t enc_pos = myEnc.read();
    int32_t delta   = (enc_pos - enc_last) / 4;
    if (delta != 0) {
      enc_last += delta * 4;
      int8_t nc = (int8_t)num_channels + (int8_t)delta;
      if (nc < 1) nc = 1;
      if (nc > 8) nc = 8;
      if ((uint8_t)nc != num_channels) {
        num_channels = (uint8_t)nc;
        EEPROM.write(1, num_channels);
        draw_settings();
      }
    }
  }

  // Button: arm / disarm
  static uint32_t btn_last = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - btn_last > 250) {
    btn_last = millis();
    armed = !armed;

    if (armed) {
      for (uint8_t i = 0; i < num_channels; i++) {
        init_state[i] = digitalReadFast(LA_PIN[i]);
        ch_state[i]   = init_state[i];
      }
      noInterrupts();
      pp_active      = 0;
      pp_pos         = 0;
      pp_ready[0]    = false;
      pp_ready[1]    = false;
      cap_total      = 0;
      overflow_count = 0;
      interrupts();
      bytes_written = 0;
      last_log[0]   = '\0';

      if (drive_ready) log_open();

      attach_isrs();
      tft.fillScreen(TFT_BLACK);
      draw_armed_counter();

#ifdef DEBUG
      if (Serial) Serial.printf("ARMED — %d channels\n", num_channels);
#endif
    } else {
      detach_isrs();
      if (drive_ready && log_file) log_close_final();
      draw_settings();

#ifdef DEBUG
      if (Serial) Serial.printf("DISARMED — %lu events, %lu bytes\n",
                                cap_total, bytes_written);
#endif
    }
  }

  // Counter update — 1 Hz while armed
  static uint32_t disp_last = 0;
  if (armed && millis() - disp_last >= 1000) {
    disp_last = millis();
    draw_armed_counter();
  }
}
