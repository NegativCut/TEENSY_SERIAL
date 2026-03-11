/*
 * la_test.ino
 *
 * Logic analyser — continuous binary capture + TFT waveform display
 * 8-channel interrupt edge capture, ping-pong buffer → flash drive (.bin)
 *
 *  Capture pins: 14, 15, 16, 17, 20, 21, 22, 23
 *  BLK: 7  |  ENC_SW: 2
 *  TFT pins defined in TFT_eSPI User_Setup.h
 *
 * Button: arm → start capture + open file; press again → stop + close file
 * If no flash drive: capture runs, nothing written (TFT shows waveform only)
 *
 * Binary file format (la01.bin … la99.bin):
 *   Header  24 bytes: 'L','A', version=2, channels=8, pins[8], init_states[8],
 *                     uint32_t cpu_hz
 *   Events   6 bytes each: uint32_t timestamp (CPU cycles), uint8_t pin,
 *                          uint8_t rising
 *   All little-endian (ARM native byte order)
 *   timestamp / (cpu_hz / 1e6) = microseconds
 *
 * Memory:
 *   disp_buf  256 events × 6 B =  1.5 KB  — circular, TFT display
 *   pp_buf  2×4096 events × 6 B = 48.0 KB — ping-pong, file write
 *
 * EEPROM byte 0: log file counter (1–99)
 */

#include <TFT_eSPI.h>      // MUST be before USBHost_t36
#include <USBHost_t36.h>
#include <EEPROM.h>

// #define DEBUG   // uncomment to enable Serial output

#define TFT_BLK   7
#define ENC_SW    2

static const uint8_t  LA_PIN[8]  = {14, 15, 16, 17, 20, 21, 22, 23};
static const uint16_t CH_COL[8]  = {
  TFT_GREEN, TFT_CYAN, TFT_YELLOW, TFT_MAGENTA,
  TFT_ORANGE, 0x567F, TFT_GREENYELLOW, TFT_PINK
};

// ── Layout (landscape 160×128) ────────────────────────────────────
#define SCR_W      160
#define SCR_H      128
#define STATUS_H   8
#define WAVE_X    18
#define WAVE_W    (SCR_W - WAVE_X)
#define LANES      8
#define LANE_H    ((SCR_H - STATUS_H) / LANES)  // = 15 px
#define LANE_Y(i) (STATUS_H + (i) * LANE_H)
#define HIGH_Y(i) (LANE_Y(i) + 2)
#define LOW_Y(i)  (LANE_Y(i) + LANE_H - 3)

// ── Event struct ──────────────────────────────────────────────────
struct __attribute__((packed)) Event {
  uint32_t timestamp;  // ARM_DWT_CYCCNT at edge time (divide by cpu_hz/1e6 for µs)
  uint8_t  pin;        // 0–7
  uint8_t  rising;     // 1 = rising, 0 = falling
};

// ── Display buffer (circular, for TFT waveform) ───────────────────
#define DISP_SIZE  256

static volatile Event    disp_buf[DISP_SIZE];
static volatile uint8_t  disp_write = 0;

// ── Ping-pong buffer (for continuous file writing) ────────────────
#define BUF_HALF   4096

static volatile Event    pp_buf[2][BUF_HALF];
static volatile uint8_t  pp_active       = 0;
static volatile uint16_t pp_pos          = 0;
static volatile bool     pp_ready[2]     = {false, false};
static volatile uint32_t cap_total       = 0;
static volatile uint32_t overflow_count  = 0;

// ── App state ─────────────────────────────────────────────────────
static bool     armed      = false;
static uint8_t  init_state[8];
static volatile uint8_t ch_state[8];
static uint32_t disp_shown = 0;

// ── USB + file ────────────────────────────────────────────────────
USBHost        myusb;
USBDrive       myDrive(myusb);
USBFilesystem  myFS(myusb);

static bool     drive_ready   = false;
static uint8_t  log_file_num  = 0;
static char     last_log[14]  = "";
static File     log_file;
static uint32_t bytes_written = 0;

TFT_eSPI tft;

// ── ISRs ──────────────────────────────────────────────────────────
// Toggle ch_state — no digitalRead race condition
static FASTRUN void isr_capture(uint8_t pin_idx) {
  uint32_t t   = ARM_DWT_CYCCNT;   // cycle-accurate timestamp (~1.67 ns @ 600 MHz)
  uint8_t  val = ch_state[pin_idx] ^ 1;
  ch_state[pin_idx] = val;

  // Display circular buffer (always updated)
  uint8_t dw = disp_write;
  disp_buf[dw].timestamp = t;
  disp_buf[dw].pin       = pin_idx;
  disp_buf[dw].rising    = val;
  disp_write = (dw + 1) % DISP_SIZE;

  // Ping-pong storage buffer
  uint8_t  h   = pp_active;
  uint16_t pos = pp_pos;
  pp_buf[h][pos].timestamp = t;
  pp_buf[h][pos].pin       = pin_idx;
  pp_buf[h][pos].rising    = val;
  pos++;
  cap_total++;

  if (pos >= BUF_HALF) {
    uint8_t next = h ^ 1;
    if (!pp_ready[next]) {
      // Normal swap
      pp_ready[h] = true;
      pp_active   = next;
      pp_pos      = 0;
    } else {
      // Overflow: write side hasn't caught up — wrap current half
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

// ── File helpers ──────────────────────────────────────────────────
struct __attribute__((packed)) FileHeader {
  uint8_t  magic[2];        // 'L', 'A'
  uint8_t  version;         // 2
  uint8_t  channels;        // 8
  uint8_t  pins[8];         // LA_PIN[0..7] physical pin numbers
  uint8_t  init_states[8];  // pin levels at arm time
  uint32_t cpu_hz;          // F_CPU — divides timestamps back to µs
};  // = 24 bytes

static bool log_open() {
  log_file_num++;
  if (log_file_num > 99) log_file_num = 1;
  EEPROM.write(0, log_file_num);

  snprintf(last_log, sizeof(last_log), "la%02d.bin", log_file_num);
  log_file = myFS.open(last_log, FILE_WRITE);
  if (!log_file) {
    snprintf(last_log, sizeof(last_log), "OPEN FAIL");
    return false;
  }

  FileHeader hdr;
  hdr.magic[0] = 'L';  hdr.magic[1] = 'A';
  hdr.version  = 2;    hdr.channels = 8;
  for (uint8_t i = 0; i < 8; i++) hdr.pins[i]       = LA_PIN[i];
  for (uint8_t i = 0; i < 8; i++) hdr.init_states[i] = init_state[i];
  hdr.cpu_hz   = (uint32_t)F_CPU;
  log_file.write((const uint8_t*)&hdr, sizeof(FileHeader));
  bytes_written = sizeof(FileHeader);
  return true;
}

// Called from main loop while armed — writes any ready halves
static void log_flush_ready() {
  for (uint8_t h = 0; h < 2; h++) {
    if (pp_ready[h] && log_file) {
      log_file.write((const uint8_t*)pp_buf[h],
                     (uint32_t)BUF_HALF * sizeof(Event));
      bytes_written += (uint32_t)BUF_HALF * sizeof(Event);
      pp_ready[h] = false;
    }
  }
}

// Called on disarm — flushes remaining data and closes
static void log_close_final() {
  log_flush_ready();  // flush any complete halves first

  // Flush partial active half
  noInterrupts();
  uint8_t  fh  = pp_active;
  uint16_t fpo = pp_pos;
  interrupts();

  if (fpo > 0 && log_file) {
    log_file.write((const uint8_t*)pp_buf[fh],
                   (uint32_t)fpo * sizeof(Event));
    bytes_written += (uint32_t)fpo * sizeof(Event);
  }

  if (log_file) {
    log_file.close();
    log_file = File();
  }
}

// ── Status bar ────────────────────────────────────────────────────
static void draw_status() {
  tft.fillRect(0, 0, SCR_W, STATUS_H, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);

  if (armed) {
    tft.setTextColor(overflow_count > 0 ? TFT_RED : TFT_GREEN, TFT_BLACK);
    uint32_t kb = bytes_written >> 10;  // / 1024
    tft.printf("ARM %luev %luK OVF:%lu", cap_total, kb, overflow_count);
  } else if (cap_total > 0) {
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (last_log[0] && last_log[0] != 'O')
      tft.printf("FRZ %luev  %-10s", cap_total, last_log);
    else
      tft.printf("FRZ %luev  (no file) ", cap_total);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    if (drive_ready)
      tft.printf("DISARMD LOG:RDY        ");
    else
      tft.printf("DISARMD  btn=arm       ");
  }
}

// ── Waveform draw (display buffer) ────────────────────────────────
static void draw_waveform() {
  noInterrupts();
  uint8_t  dw    = disp_write;
  uint32_t total = cap_total;
  Event    snap[DISP_SIZE];
  for (uint16_t i = 0; i < DISP_SIZE; i++) {
    snap[i].timestamp = disp_buf[i].timestamp;
    snap[i].pin     = disp_buf[i].pin;
    snap[i].rising  = disp_buf[i].rising;
  }
  interrupts();

  uint16_t count = (total < DISP_SIZE) ? (uint16_t)total : (uint16_t)DISP_SIZE;
  uint16_t head  = (total < DISP_SIZE) ? 0 : (uint16_t)dw;

  uint32_t t_start = 0, span = 0;
  if (count > 0) {
    t_start = snap[head].timestamp;
    uint32_t t_end = snap[(head + count - 1) % DISP_SIZE].timestamp;
    span = (t_end > t_start) ? (t_end - t_start) : 1;
  }

  tft.fillRect(0, STATUS_H, SCR_W, SCR_H - STATUS_H, TFT_BLACK);

  tft.setTextSize(1);
  for (uint8_t ch = 0; ch < LANES; ch++) {
    tft.drawFastHLine(0, LANE_Y(ch), SCR_W, 0x1082);
    tft.setTextColor(CH_COL[ch], TFT_BLACK);
    tft.setCursor(1, LANE_Y(ch) + (LANE_H - 8) / 2);
    tft.printf("P%d", ch);
  }

  if (count == 0) {
    for (uint8_t ch = 0; ch < LANES; ch++) {
      int16_t y = init_state[ch] ? HIGH_Y(ch) : LOW_Y(ch);
      tft.drawFastHLine(WAVE_X, y, WAVE_W, CH_COL[ch]);
    }
    draw_status();
    return;
  }

  for (uint8_t ch = 0; ch < LANES; ch++) {
    uint8_t  state  = init_state[ch];
    int16_t  prev_x = WAVE_X;

    for (uint16_t j = 0; j < count; j++) {
      uint16_t idx = (head + j) % DISP_SIZE;
      if (snap[idx].pin != ch) continue;

      int16_t ex = WAVE_X + (int16_t)((uint64_t)(snap[idx].timestamp - t_start)
                              * (WAVE_W - 1) / span);
      if (ex < WAVE_X)           ex = WAVE_X;
      if (ex >= WAVE_X + WAVE_W) ex = WAVE_X + WAVE_W - 1;

      int16_t y_cur = state ? HIGH_Y(ch) : LOW_Y(ch);
      if (ex >= prev_x)
        tft.drawFastHLine(prev_x, y_cur, ex - prev_x + 1, CH_COL[ch]);

      int16_t y_new = snap[idx].rising ? HIGH_Y(ch) : LOW_Y(ch);
      if (y_new != y_cur) {
        int16_t y_top = (y_cur < y_new) ? y_cur : y_new;
        tft.drawFastVLine(ex, y_top, abs(y_new - y_cur) + 1, CH_COL[ch]);
      }

      state  = snap[idx].rising;
      prev_x = ex;
    }

    int16_t y_fin = state ? HIGH_Y(ch) : LOW_Y(ch);
    int16_t trail = WAVE_X + WAVE_W - prev_x;
    if (trail > 0)
      tft.drawFastHLine(prev_x, y_fin, trail, CH_COL[ch]);
  }

  draw_status();
}

// ── Attach / detach helpers ───────────────────────────────────────
static void attach_isrs() {
  attachInterrupt(digitalPinToInterrupt(LA_PIN[0]), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[1]), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[2]), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[3]), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[4]), isr4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[5]), isr5, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[6]), isr6, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LA_PIN[7]), isr7, CHANGE);
}

static void detach_isrs() {
  for (uint8_t i = 0; i < 8; i++)
    detachInterrupt(digitalPinToInterrupt(LA_PIN[i]));
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Enable ARM cycle counter (DWT) for sub-µs timestamps
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  pinMode(ENC_SW, INPUT_PULLUP);

  for (uint8_t i = 0; i < 8; i++) {
    pinMode(LA_PIN[i], INPUT);
    init_state[i] = 0;
    ch_state[i]   = 0;
  }

  log_file_num = EEPROM.read(0);
  if (log_file_num > 99) log_file_num = 0;

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  draw_status();
  tft.setTextSize(1);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(0, STATUS_H + 4);
  tft.print("btn to arm");
  tft.setCursor(0, STATUS_H + 14);
  tft.printf("%d %d %d %d %d %d %d %d",
             LA_PIN[0], LA_PIN[1], LA_PIN[2], LA_PIN[3],
             LA_PIN[4], LA_PIN[5], LA_PIN[6], LA_PIN[7]);

  myusb.begin();
}

// ── Main loop ─────────────────────────────────────────────────────
void loop() {
  myusb.Task();

  // Drive mount / unmount
  if (myFS && !drive_ready) {
    drive_ready = true;
    draw_status();
  }
  if (!myFS && drive_ready) {
    drive_ready = false;
    if (armed) {
      // Drive pulled during capture — stop and discard file handle
      detach_isrs();
      armed     = false;
      log_file  = File();
    }
    draw_status();
  }

  // Flush ready ping-pong halves while armed
  if (armed) log_flush_ready();

  // Button: arm / disarm
  static uint32_t btn_last = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - btn_last > 250) {
    btn_last = millis();
    armed = !armed;

    if (armed) {
      for (uint8_t i = 0; i < 8; i++) {
        init_state[i] = digitalReadFast(LA_PIN[i]);
        ch_state[i]   = init_state[i];
      }
      noInterrupts();
      disp_write      = 0;
      pp_active       = 0;
      pp_pos          = 0;
      pp_ready[0]     = false;
      pp_ready[1]     = false;
      cap_total       = 0;
      overflow_count  = 0;
      interrupts();
      disp_shown    = 0;
      bytes_written = 0;
      last_log[0]   = '\0';

      if (drive_ready) log_open();

      attach_isrs();
      tft.fillScreen(TFT_BLACK);
      draw_waveform();

#ifdef DEBUG
      if (Serial) Serial.println("ARMED");
#endif
    } else {
      detach_isrs();
      if (drive_ready && log_file) log_close_final();
      draw_waveform();

#ifdef DEBUG
      if (Serial) Serial.printf("DISARMED — %lu events, %lu bytes\n",
                                cap_total, bytes_written);
#endif
    }
  }

  // Throttled TFT refresh ~10 Hz while armed
  static uint32_t disp_last = 0;
  if (armed && millis() - disp_last >= 100) {
    disp_last = millis();
    uint32_t t = cap_total;
    if (t != disp_shown) {
      disp_shown = t;
      draw_waveform();
    }
  }
}
