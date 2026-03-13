/*
 * la_dma.ino
 *
 * Logic analyser — QTimer1-triggered DMA parallel GPIO capture
 * No per-edge ISRs. CPU-free during sampling. Detects state changes in
 * main loop and outputs event stream compatible with la_test v2 file format.
 *
 *  Capture pins: 14,15,16,17,20,21,22,23  (GPIO1/GPIO6 bits 18,19,23,22,26,27,24,25)
 *  ENC_A: 3  |  ENC_B: 4  |  ENC_SW: 2  |  BLK: 7
 *
 * Architecture:
 *   QTimer1 ch0 fires at SAMPLE_RATE → triggers DMA → GPIO6_DR → dma_buf[] (circular)
 *   DMA ISR at half/complete: records timestamp, sets half_ready flag
 *   Main loop: processes ready DMA halves, extracts 8-ch state byte per sample,
 *              detects state changes, writes 6-byte events to ping-pong USB buffer
 *
 * File format: same as la_test v2 — compatible with Logic_Analyser_Viewer
 *   Header 24 bytes: 'L','A', version=2, channels, pins[8], init_states[8], uint32_t cpu_hz
 *   Events  6 bytes: uint32_t timestamp (DWT cycles), uint8_t channel, uint8_t rising
 *
 * QTimer IPG clock: F_BUS/2 ≈ 66 MHz (AHB=132 MHz, IPG=66 MHz at 600 MHz CPU)
 *   Actual rate = 66 MHz / (TMR_COMP + 1)
 *   If rate is wrong, adjust TMR_IPG_HZ below.
 *
 * EEPROM byte 0: log file counter fallback
 * EEPROM byte 1: num_channels (1–8)
 */

#include <TFT_eSPI.h>
#include <Encoder.h>
#include <USBHost_t36.h>
#include <EEPROM.h>
#include <DMAChannel.h>

// #define DEBUG   // uncomment to enable Serial output

#define TFT_BLK   7
#define ENC_SW    2
#define ENC_A     3
#define ENC_B     4

static const uint8_t  LA_PIN[8]  = {14, 15, 16, 17, 20, 21, 22, 23};
static const uint16_t CH_COL[8]  = {
  TFT_GREEN, TFT_CYAN, TFT_YELLOW, TFT_MAGENTA,
  TFT_ORANGE, 0x567F, TFT_GREENYELLOW, TFT_PINK
};

// ── GPIO bit positions within GPIO1/GPIO6 for each capture pin ────
// Pin 14=bit18, 15=bit19, 16=bit23, 17=bit22, 20=bit26, 21=bit27, 22=bit24, 23=bit25
static const uint8_t GPIO_BIT[8] = {18, 19, 23, 22, 26, 27, 24, 25};

static inline uint8_t extract_state(uint32_t g) {
  uint8_t s  =  (g >> 18) & 0x03;          // ch0 (pin14, bit18), ch1 (pin15, bit19)
  s |= (((g >> 23) & 0x01) << 2);          // ch2 (pin16, bit23)
  s |= (((g >> 22) & 0x01) << 3);          // ch3 (pin17, bit22)
  s |= (((g >> 26) & 0x01) << 4);          // ch4 (pin20, bit26)
  s |= (((g >> 27) & 0x01) << 5);          // ch5 (pin21, bit27)
  s |= (((g >> 24) & 0x01) << 6);          // ch6 (pin22, bit24)
  s |= (((g >> 25) & 0x01) << 7);          // ch7 (pin23, bit25)
  return s;
}

// ── Sample rate ────────────────────────────────────────────────────
// IPG clock drives QTimer. At 600 MHz CPU: AHB=132 MHz, IPG=66 MHz.
// Adjust TMR_IPG_HZ if measured rate differs (measure with DEBUG mode).
#define TMR_IPG_HZ    66000000UL
#define SAMPLE_RATE   5000000UL   // Hz — must divide evenly into TMR_IPG_HZ
// Actual: 66 MHz / (TMR_COMP+1) — nearest achievable to SAMPLE_RATE
#define TMR_COMP      ((uint16_t)(TMR_IPG_HZ / SAMPLE_RATE - 1))
#define CYCLES_PER_SAMPLE  ((uint32_t)(F_CPU_ACTUAL / SAMPLE_RATE))

// ── DMA circular buffer (power-of-2 size + alignment for DMOD) ────
// Buffer holds DMA_TOTAL uint32_t GPIO snapshots, split into two halves.
// At 5 MHz, DMA_HALF=2048: each half fills in 2048/5M = 409 µs.
#define DMA_HALF   2048
#define DMA_TOTAL  (DMA_HALF * 2)

DMAMEM static __attribute__((aligned(DMA_TOTAL * sizeof(uint32_t))))
  volatile uint32_t dma_buf[DMA_TOTAL];

DMAChannel dma;

static volatile bool     half_ready[2]  = {false, false};
static volatile uint32_t half_ts[2]     = {0, 0};  // DWT at end of each half

// ── Event struct (same as la_test) ───────────────────────────────
struct __attribute__((packed)) Event {
  uint32_t timestamp;
  uint8_t  pin;
  uint8_t  rising;
};

// ── Display buffer ────────────────────────────────────────────────
#define DISP_SIZE  256
static volatile Event   disp_buf[DISP_SIZE];
static volatile uint8_t disp_write = 0;

// ── Ping-pong USB write buffer ────────────────────────────────────
#define BUF_HALF   32768
DMAMEM static volatile Event pp_buf[2][BUF_HALF];
static volatile uint8_t  pp_active      = 0;
static volatile uint32_t pp_pos         = 0;
static volatile bool     pp_ready[2]    = {false, false};
static volatile uint32_t cap_total      = 0;
static volatile uint32_t overflow_count = 0;

// ── App state ─────────────────────────────────────────────────────
static uint8_t  num_channels = 8;
static bool     armed        = false;
static uint8_t  init_state[8];
static uint8_t  ch_state_byte = 0;   // last known 8-ch state byte
static uint32_t disp_shown   = 0;

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

// ── Layout ────────────────────────────────────────────────────────
#define SCR_W    160
#define SCR_H    128
#define STATUS_H 8
#define WAVE_X   18
#define WAVE_W   (SCR_W - WAVE_X)

// ── DMA ISR ───────────────────────────────────────────────────────
// Fires at half and complete of each major DMA loop.
// Alternates: half0 done, half1 done, half0 done, ...
static volatile uint8_t dma_isr_half = 0;

void dma_isr() {
  dma.clearInterrupt();
  uint8_t h = dma_isr_half;
  half_ts[h]    = ARM_DWT_CYCCNT;
  half_ready[h] = true;
  dma_isr_half  = h ^ 1;
}

// ── QTimer1 setup (triggers DMA at SAMPLE_RATE) ───────────────────
static void qtimer_dma_init() {
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);

  TMR1_CTRL0  = 0;       // stop while configuring
  TMR1_CNTR0  = 0;
  TMR1_LOAD0  = 0;       // reload to 0 on compare
  TMR1_COMP10 = TMR_COMP;
  TMR1_CMPLD10 = TMR_COMP;

  // CSCTRL: CL1=1 — reload COMP1 from CMPLD1 on each compare event (periodic)
  TMR1_CSCTRL0 = TMR_CSCTRL_CL1(1);

  // DMA: trigger on CMPLD1 load event
  TMR1_DMA0 = TMR_DMA_CMPLD1DE;

  // CTRL: count rising edges of IPG/1, count up to COMP1 then reload
  TMR1_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(8) | TMR_CTRL_LENGTH;

  // Enable channel 0
  TMR1_ENBL |= 1;
}

// ── DMA channel setup ─────────────────────────────────────────────
static void dma_init() {
  dma.begin(true);
  dma.source(GPIO6_DR);
  dma.destinationCircular(dma_buf, DMA_TOTAL * sizeof(uint32_t));
  dma.transferSize(4);
  dma.transferCount(DMA_TOTAL);
  dma.triggerAtHardwareEvent(DMAMUX_SOURCE_QTIMER1_READ0);
  dma.interruptAtHalf();
  dma.interruptAtCompletion();
  dma.attachInterrupt(dma_isr);
  dma.enable();
}

// ── Start / stop DMA sampling ─────────────────────────────────────
static void sampling_start() {
  dma_isr_half  = 0;
  half_ready[0] = false;
  half_ready[1] = false;
  // Enable QTimer to begin triggering DMA
  TMR1_CTRL0 |= TMR_CTRL_CM(1);
  dma.enable();
}

static void sampling_stop() {
  TMR1_CTRL0 &= ~TMR_CTRL_CM(7);  // CM=0: stop counting
  dma.disable();
}

// ── Write one event to both display and USB ping-pong buffers ─────
static void write_event(uint32_t t, uint8_t ch, uint8_t rising) {
  // Display buffer (circular, for TFT waveform)
  uint8_t dw = disp_write;
  disp_buf[dw] = {t, ch, rising};
  disp_write   = (dw + 1) % DISP_SIZE;

  // USB ping-pong buffer
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

// ── Process one DMA half-buffer ───────────────────────────────────
static void process_half(uint8_t h) {
  uint32_t t_end = half_ts[h];
  // t of sample i = t_end - (DMA_HALF - 1 - i) * CYCLES_PER_SAMPLE
  uint32_t t_base = t_end - (uint32_t)(DMA_HALF - 1) * CYCLES_PER_SAMPLE;

  volatile uint32_t *buf = dma_buf + h * DMA_HALF;
  uint8_t nc = num_channels;
  uint8_t mask = (nc == 8) ? 0xFF : (uint8_t)((1 << nc) - 1);

  for (uint32_t i = 0; i < DMA_HALF; i++) {
    uint8_t cur = extract_state(buf[i]) & mask;
    uint8_t changed = cur ^ ch_state_byte;
    if (changed) {
      uint32_t t = t_base + i * CYCLES_PER_SAMPLE;
      for (uint8_t ch = 0; ch < nc; ch++) {
        if (changed & (1 << ch)) {
          write_event(t, ch, (cur >> ch) & 1);
        }
      }
      ch_state_byte = cur;
    }
  }

  half_ready[h] = false;
}

// ── USB file helpers (same as la_test) ────────────────────────────
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
  Event    snap[DISP_SIZE];
  for (uint16_t i = 0; i < DISP_SIZE; i++) {
    snap[i].timestamp = disp_buf[i].timestamp;
    snap[i].pin       = disp_buf[i].pin;
    snap[i].rising    = disp_buf[i].rising;
  }

  uint8_t nc     = num_channels;
  uint8_t lane_h = (SCR_H - STATUS_H) / nc;
  uint16_t count = (total < DISP_SIZE) ? (uint16_t)total : DISP_SIZE;
  uint16_t head  = (total < DISP_SIZE) ? 0 : (uint16_t)dw;

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
        int16_t y_top = (y_cur < y_new) ? y_cur : y_new;
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
  Serial.printf("la_dma: SAMPLE_RATE=%lu TMR_COMP=%u CYCLES_PER_SAMPLE=%lu\n",
                SAMPLE_RATE, TMR_COMP, CYCLES_PER_SAMPLE);
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

  qtimer_dma_init();
  // DMA channel initialised but not triggered until armed
  dma_init();
  dma.disable();   // keep idle until armed

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
      sampling_stop();
      armed    = false;
      log_file = File();
    }
    draw_idle();
  }

  // Flush USB ping-pong while armed
  if (armed) log_flush_ready();

  // Process completed DMA halves (main capture work, no ISR overhead)
  if (armed) {
    for (uint8_t h = 0; h < 2; h++) {
      if (half_ready[h]) process_half(h);
    }
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
      // Snapshot initial pin states from live GPIO
      uint32_t g = GPIO6_DR;
      uint8_t  s = extract_state(g);
      uint8_t  mask = (num_channels == 8) ? 0xFF : (uint8_t)((1 << num_channels) - 1);
      ch_state_byte = s & mask;
      for (uint8_t i = 0; i < num_channels; i++)
        init_state[i] = (s >> i) & 1;

      // Reset capture state
      pp_active       = 0;
      pp_pos          = 0;
      pp_ready[0]     = false;
      pp_ready[1]     = false;
      cap_total       = 0;
      overflow_count  = 0;
      disp_write      = 0;
      disp_shown      = 0;
      bytes_written   = 0;
      last_log[0]     = '\0';

      if (drive_ready) log_open();

      sampling_start();
      tft.fillScreen(TFT_BLACK);
      draw_armed_counter();

#ifdef DEBUG
      Serial.printf("ARMED — %d channels, init_state=0x%02X\n", num_channels, ch_state_byte);
#endif
    } else {
      sampling_stop();
      // Process any remaining DMA data
      for (uint8_t h = 0; h < 2; h++)
        if (half_ready[h]) process_half(h);
      if (drive_ready && log_file) log_close_final();
      disp_shown = 0;
      tft.fillScreen(TFT_BLACK);
      draw_waveform();

#ifdef DEBUG
      Serial.printf("DISARMED — %lu events, %lu bytes\n", cap_total, bytes_written);
#endif
    }
  }

  // TFT: counter at 1 Hz while armed, waveform at 10 Hz while disarmed
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
