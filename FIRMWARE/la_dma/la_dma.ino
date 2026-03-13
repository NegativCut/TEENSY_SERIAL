/*
 * la_dma.ino
 *
 * Logic analyser — fixed-rate GPIO sampling + main-loop event detection
 *
 * Architecture (key difference from la_test):
 *   la_test: per-edge CHANGE ISR → event detection + buffer write in ISR
 *            → fails at high rates because ISR can't keep up with edge density
 *
 *   la_dma:  IntervalTimer ISR at SAMPLE_RATE → single GPIO read → circ_buf
 *            → event detection in main loop (no ISR contention)
 *            → ISR body is 3 instructions, works at 5 MHz+
 *
 *  Capture pins: 14,15,16,17,20,21,22,23  (all GPIO1/GPIO6)
 *  ENC_A: 3  |  ENC_B: 4  |  ENC_SW: 2  |  BLK: 7
 *
 * Output: v2 binary format, compatible with Logic_Analyser_Viewer
 *   Header 24 bytes: 'L','A', version=2, channels, pins[8], init_states[8], uint32_t cpu_hz
 *   Events  6 bytes: uint32_t timestamp (DWT cycles), uint8_t channel, uint8_t rising
 *
 * Timestamp source: DWT cycle counter captured in ISR alongside GPIO read.
 *   timestamp / (cpu_hz / 1e6) = microseconds
 *
 * EEPROM byte 0: log file counter  |  byte 1: num_channels
 */

#include <TFT_eSPI.h>
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

// ── GPIO bit extraction ────────────────────────────────────────────
// Pins 14,15,16,17,20,21,22,23 = GPIO1/GPIO6 bits 18,19,23,22,26,27,24,25
// ch0..7 map to those bit positions respectively.
static inline uint8_t extract_state(uint32_t g) {
  uint8_t s  =  (g >> 18) & 0x03;           // ch0 (bit18), ch1 (bit19)
  s |= (((g >> 23) & 1) << 2);              // ch2 (bit23)
  s |= (((g >> 22) & 1) << 3);              // ch3 (bit22)
  s |= (((g >> 26) & 1) << 4);              // ch4 (bit26)
  s |= (((g >> 27) & 1) << 5);              // ch5 (bit27)
  s |= (((g >> 24) & 1) << 6);              // ch6 (bit24)
  s |= (((g >> 25) & 1) << 7);              // ch7 (bit25)
  return s;
}

// ── Sample rate ────────────────────────────────────────────────────
// Reduce if IntervalTimer cannot achieve the period (try 0.5 for 2 MHz).
#define SAMPLE_PERIOD_US  0.2f    // 0.2 µs = 5 MHz

// ── Sample circular buffer ────────────────────────────────────────
// Holds raw GPIO reads + DWT timestamps. Power-of-2 for fast masking.
// At 5 MHz, CIRC_HALF=2048 fills in 410 µs — plenty of time for main loop.
#define CIRC_HALF   2048
#define CIRC_TOTAL  (CIRC_HALF * 2)
#define CIRC_MASK   (CIRC_TOTAL - 1)

struct Sample {
  uint32_t gpio;   // raw GPIO6_DR value
  uint32_t cycle;  // DWT cycle counter at sample time
};

DMAMEM static volatile Sample circ_buf[CIRC_TOTAL];
static volatile uint32_t wr_ptr = 0;   // written by ISR
static          uint32_t rd_ptr = 0;   // read by main loop

// ── Event struct ──────────────────────────────────────────────────
struct __attribute__((packed)) Event {
  uint32_t timestamp;
  uint8_t  pin;
  uint8_t  rising;
};

// ── Display buffer ────────────────────────────────────────────────
#define DISP_SIZE  256
static volatile Event   disp_buf[DISP_SIZE];
static volatile uint8_t disp_write = 0;

// ── USB ping-pong write buffer ────────────────────────────────────
#define BUF_HALF  32768
DMAMEM static volatile Event pp_buf[2][BUF_HALF];
static volatile uint8_t  pp_active     = 0;
static volatile uint32_t pp_pos        = 0;
static volatile bool     pp_ready[2]   = {false, false};
static volatile uint32_t cap_total     = 0;
static volatile uint32_t overflow_count = 0;

// ── App state ─────────────────────────────────────────────────────
static uint8_t  num_channels = 8;
static bool     armed        = false;
static uint8_t  init_state[8];
static uint8_t  prev_state_byte = 0;
static uint32_t disp_shown  = 0;

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
IntervalTimer sampleTimer;

// ── Layout ────────────────────────────────────────────────────────
#define SCR_W    160
#define SCR_H    128
#define STATUS_H 8
#define WAVE_X   18
#define WAVE_W   (SCR_W - WAVE_X)

// ── Sampling ISR — 3 instructions, runs at SAMPLE_RATE ───────────
FASTRUN void sample_isr() {
  uint32_t idx = wr_ptr & CIRC_MASK;
  circ_buf[idx].cycle = ARM_DWT_CYCCNT;
  circ_buf[idx].gpio  = GPIO6_DR;
  wr_ptr++;
}

// ── Process one CIRC_HALF worth of samples ────────────────────────
static void process_samples() {
  uint8_t nc   = num_channels;
  uint8_t mask = (nc == 8) ? 0xFF : (uint8_t)((1 << nc) - 1);

  while ((wr_ptr - rd_ptr) >= CIRC_HALF) {
    uint32_t end = rd_ptr + CIRC_HALF;
    while (rd_ptr != end) {
      uint32_t idx = rd_ptr & CIRC_MASK;
      uint8_t  cur = extract_state(circ_buf[idx].gpio) & mask;
      uint8_t  changed = cur ^ prev_state_byte;
      if (changed) {
        uint32_t t = circ_buf[idx].cycle;
        for (uint8_t ch = 0; ch < nc; ch++) {
          if (!(changed & (1 << ch))) continue;
          uint8_t rising = (cur >> ch) & 1;

          // Display buffer
          uint8_t dw = disp_write;
          disp_buf[dw] = {t, ch, rising};
          disp_write   = (dw + 1) % DISP_SIZE;

          // USB ping-pong
          uint8_t  h   = pp_active;
          uint32_t pos = pp_pos;
          pp_buf[h][pos] = {t, ch, rising};
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
        prev_state_byte = cur;
      }
      rd_ptr++;
    }
  }
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
  uint8_t  magic[2];
  uint8_t  version;
  uint8_t  channels;
  uint8_t  pins[8];
  uint8_t  init_states[8];
  uint32_t cpu_hz;
};

static bool log_open() {
  log_file_num = (log_file_num >= 99) ? 1 : log_file_num + 1;
  EEPROM.write(0, log_file_num);
  snprintf(last_log, sizeof(last_log), "la%02d.bin", log_file_num);
  log_file = myFS.open(last_log, FILE_WRITE);
  if (!log_file) { snprintf(last_log, sizeof(last_log), "OPEN FAIL"); return false; }

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
  uint8_t  fh  = pp_active;
  uint32_t fpo = pp_pos;
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
      tft.printf("FRZ CH:%d %luev %-8s", num_channels, cap_total, last_log);
    else
      tft.printf("FRZ CH:%d %luev (no file)", num_channels, cap_total);
  } else {
    tft.setTextColor(drive_ready ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.printf("DISARMD CH:%d %s",
               num_channels, drive_ready ? "LOG:RDY" : "NO DRIVE");
  }
}

static void draw_armed_counter() {
  tft.setTextSize(2);
  tft.setTextColor(overflow_count > 0 ? TFT_RED : TFT_GREEN, TFT_BLACK);
  tft.setCursor(0, STATUS_H + 20);
  tft.printf("%-10lu", cap_total);
  draw_status();
}

static void draw_idle() {
  tft.fillScreen(TFT_BLACK);
  draw_status();
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(2, STATUS_H + 4);
  tft.printf("CH:%d  enc=sel  btn=arm", num_channels);
  tft.setCursor(2, STATUS_H + 16);
  for (uint8_t i = 0; i < num_channels; i++) {
    tft.setTextColor(CH_COL[i], TFT_BLACK);
    tft.printf("%d ", LA_PIN[i]);
  }
  if (num_channels < 8) {
    tft.setTextColor(0x2104, TFT_BLACK);
    for (uint8_t i = num_channels; i < 8; i++)
      tft.printf("%d ", LA_PIN[i]);
  }
  tft.setCursor(2, STATUS_H + 28);
  if (drive_ready) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    uint8_t next = (log_file_num >= 99) ? 1 : log_file_num + 1;
    tft.printf("Next: la%02d.bin", next);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("No drive — TFT only");
  }
}

static void draw_waveform() {
  uint8_t  dw    = disp_write;
  uint32_t total = cap_total;
  Event snap[DISP_SIZE];
  for (uint16_t i = 0; i < DISP_SIZE; i++) {
    snap[i].timestamp = disp_buf[i].timestamp;
    snap[i].pin       = disp_buf[i].pin;
    snap[i].rising    = disp_buf[i].rising;
  }

  uint8_t  nc     = num_channels;
  uint8_t  lane_h = (SCR_H - STATUS_H) / nc;
  uint16_t count  = (total < DISP_SIZE) ? (uint16_t)total : DISP_SIZE;
  uint16_t head   = (total < DISP_SIZE) ? 0 : (uint16_t)dw;

  uint32_t t_start = 0, span = 0;
  if (count > 0) {
    t_start = snap[head].timestamp;
    uint32_t t_end = snap[(head + count - 1) % DISP_SIZE].timestamp;
    span = (t_end > t_start) ? (t_end - t_start) : 1;
  }

  tft.fillRect(0, STATUS_H, SCR_W, SCR_H - STATUS_H, TFT_BLACK);
  tft.setTextSize(1);

  for (uint8_t ch = 0; ch < nc; ch++) {
    uint8_t lane_y = STATUS_H + ch * lane_h;
    uint8_t high_y = lane_y + 2;
    uint8_t low_y  = lane_y + lane_h - 3;

    tft.drawFastHLine(0, lane_y, SCR_W, 0x1082);
    tft.setTextColor(CH_COL[ch], TFT_BLACK);
    tft.setCursor(1, lane_y + (lane_h - 8) / 2);
    tft.printf("P%d", ch);

    if (count == 0) {
      tft.drawFastHLine(WAVE_X, init_state[ch] ? high_y : low_y, WAVE_W, CH_COL[ch]);
      continue;
    }

    uint8_t  state  = init_state[ch];
    int16_t  prev_x = WAVE_X;

    for (uint16_t j = 0; j < count; j++) {
      uint16_t idx = (head + j) % DISP_SIZE;
      if (snap[idx].pin != ch) continue;
      int16_t ex = WAVE_X + (int16_t)((uint64_t)(snap[idx].timestamp - t_start)
                              * (WAVE_W - 1) / span);
      if (ex < WAVE_X)           ex = WAVE_X;
      if (ex >= WAVE_X + WAVE_W) ex = WAVE_X + WAVE_W - 1;
      int16_t y_cur = state ? high_y : low_y;
      if (ex >= prev_x)
        tft.drawFastHLine(prev_x, y_cur, ex - prev_x + 1, CH_COL[ch]);
      int16_t y_new = snap[idx].rising ? high_y : low_y;
      if (y_new != y_cur) {
        int16_t y_top = min(y_cur, y_new);
        tft.drawFastVLine(ex, y_top, abs(y_new - y_cur) + 1, CH_COL[ch]);
      }
      state  = snap[idx].rising;
      prev_x = ex;
    }
    int16_t trail = WAVE_X + WAVE_W - prev_x;
    if (trail > 0)
      tft.drawFastHLine(prev_x, state ? high_y : low_y, trail, CH_COL[ch]);
  }
  draw_status();
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.printf("la_dma: SAMPLE_PERIOD_US=%.2f\n", SAMPLE_PERIOD_US);
#endif

  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  pinMode(TFT_BLK, OUTPUT); digitalWrite(TFT_BLK, HIGH);
  pinMode(ENC_SW, INPUT_PULLUP);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(LA_PIN[i], INPUT);
    init_state[i] = 0;
  }

  num_channels = EEPROM.read(1);
  if (num_channels < 1 || num_channels > 8) num_channels = 8;
  log_file_num = EEPROM.read(0);
  if (log_file_num > 99) log_file_num = 0;
  enc_last = myEnc.read();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  draw_idle();

  myusb.begin();
}

// ── Main loop ─────────────────────────────────────────────────────
void loop() {
  myusb.Task();

  // Drive mount / unmount
  if (myFS && !drive_ready) {
    drive_ready  = true;
    log_file_num = scan_log_files();
    if (!armed) draw_idle(); else draw_status();
  }
  if (!myFS && drive_ready) {
    drive_ready = false;
    if (armed) {
      sampleTimer.end();
      armed    = false;
      log_file = File();
    }
    draw_idle();
  }

  // Process sample buffer + flush USB while armed
  if (armed) {
    process_samples();
    log_flush_ready();
  }

  // Encoder — channel select (disarmed only)
  if (!armed) {
    int32_t enc_pos = myEnc.read();
    int32_t delta   = (enc_pos - enc_last) / 4;
    if (delta != 0) {
      enc_last += delta * 4;
      int8_t nc = (int8_t)num_channels + (int8_t)delta;
      if (nc < 1) nc = 1; if (nc > 8) nc = 8;
      if ((uint8_t)nc != num_channels) {
        num_channels = (uint8_t)nc;
        EEPROM.write(1, num_channels);
        draw_idle();
      }
    }
  }

  // Button: arm / disarm
  static uint32_t btn_last = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - btn_last > 250) {
    btn_last = millis();
    armed = !armed;

    if (armed) {
      // Snapshot initial states
      uint8_t s = extract_state(GPIO6_DR);
      uint8_t mask = (num_channels == 8) ? 0xFF : (uint8_t)((1 << num_channels) - 1);
      prev_state_byte = s & mask;
      for (uint8_t i = 0; i < num_channels; i++)
        init_state[i] = (s >> i) & 1;

      // Reset all counters
      pp_active      = 0; pp_pos = 0;
      pp_ready[0]    = false; pp_ready[1] = false;
      cap_total      = 0; overflow_count = 0;
      disp_write     = 0; disp_shown = 0;
      bytes_written  = 0; last_log[0] = '\0';
      wr_ptr = 0; rd_ptr = 0;

      if (drive_ready) log_open();

      // Start sampling ISR
      if (!sampleTimer.begin(sample_isr, SAMPLE_PERIOD_US)) {
        // Period too short — fall back to 1 MHz
        sampleTimer.begin(sample_isr, 1.0f);
#ifdef DEBUG
        Serial.println("IntervalTimer: fell back to 1 MHz");
#endif
      }

      tft.fillScreen(TFT_BLACK);
      draw_armed_counter();

#ifdef DEBUG
      Serial.printf("ARMED — %d ch, init=0x%02X\n", num_channels, prev_state_byte);
#endif
    } else {
      sampleTimer.end();
      process_samples();   // drain remaining samples
      if (drive_ready && log_file) log_close_final();
      disp_shown = 0;
      tft.fillScreen(TFT_BLACK);
      draw_waveform();

#ifdef DEBUG
      Serial.printf("DISARMED — %lu events, %lu bytes\n", cap_total, bytes_written);
#endif
    }
  }

  // TFT: counter 1 Hz while armed, waveform 10 Hz while disarmed
  static uint32_t disp_last = 0;
  if (armed) {
    if (millis() - disp_last >= 1000) {
      disp_last = millis();
      draw_armed_counter();
    }
  } else if (cap_total != disp_shown && millis() - disp_last >= 100) {
    disp_last  = millis();
    disp_shown = cap_total;
    draw_waveform();
  }
}
