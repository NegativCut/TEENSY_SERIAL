/*
 * teensy_usb_tft_enc_profiles.ino
 *
 * Teensy 4.0 — Multi-profile USB serial instrument display
 * Native USB Host (direct solder to bottom D+/D- pads) → FT232R
 *
 *  TFT    | Teensy Pin     Encoder | Teensy Pin     USB-A socket (direct solder)
 *  -------|-----------     --------|-----------     ---------------------------
 *  MOSI   | 11            SW       | 2              D+      | bottom D+ pad
 *  SCK    | 13            A        | 3              D-      | bottom D- pad
 *  RST    | 8             B        | 4              VBUS    | VIN pad
 *  DC     | 9                                   GND     | any GND pad
 *  CS     | 10
 *  BLK    | 7
 */

#include <TFT_eSPI.h>      // ← MUST be before USBHost_t36
#include <USBHost_t36.h>
#include <Encoder.h>
#include <EEPROM.h>

// ── Pins (only the ones we control manually) ──────────────────────
#define TFT_BLK   7
#define ENC_SW    2
#define ENC_A     3
#define ENC_B     4

// ── TFT layout ────────────────────────────────────────────────────
#define SCROLL_LINES  20
#define LINE_H        8
#define LINE_W        22

// ── Profile definition (add new instruments here) ─────────────────
struct Profile {
  const char* name;
  uint32_t    baud;
  const char* init_cmds[6];
  const char* poll_cmd;
};

const Profile profiles[] = {
  {"GW Instek GDM-8251A",      9600, {"syst:rem", "*rst", "*cls", "*idn?", nullptr}, "val1?"},
  {"Keithley 2000",            9600, {"*rst", "*cls", "*idn?", nullptr}, "READ?"},
  {"Terminal Mode (raw)",      9600, {nullptr}, nullptr}
};

const int numProfiles = sizeof(profiles) / sizeof(Profile);

// ── Display & state ───────────────────────────────────────────────
static char    scr_lines[SCROLL_LINES][LINE_W];
static uint8_t scr_write = 0;
static uint8_t scr_count = 0;

TFT_eSPI    tft;
USBHost     myusb;
USBSerial   userial(myusb);
Encoder     myEnc(ENC_A, ENC_B);

int     selectedProfile = 0;
bool    meter_ready     = false;
bool    inMenu          = false;
int     menuSelection   = 0;
char    rx_line[256];
uint16_t rx_line_pos    = 0;

// ── TFT helpers ───────────────────────────────────────────────────
static void tft_scroll_add(const char* line) {
  snprintf(scr_lines[scr_write], LINE_W, "%-21s", line);
  scr_write = (scr_write + 1) % SCROLL_LINES;
  if (scr_count < SCROLL_LINES) scr_count++;

  uint8_t head = (scr_count < SCROLL_LINES) ? 0 : scr_write;
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  for (uint8_t i = 0; i < scr_count; i++) {
    tft.setCursor(0, i * LINE_H);
    tft.print(scr_lines[(head + i) % SCROLL_LINES]);
  }
}

static void tft_log(const char* line) {
  if (Serial) Serial.println(line);
  tft_scroll_add(line);
}

static void tft_logf(const char* fmt, ...) {
  char buf[32];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  tft_log(buf);
}

static void redrawScroll() {
  uint8_t head = (scr_count < SCROLL_LINES) ? 0 : scr_write;
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  for (uint8_t i = 0; i < scr_count; i++) {
    tft.setCursor(0, i * LINE_H);
    tft.print(scr_lines[(head + i) % SCROLL_LINES]);
  }
}

static void tft_drawMenu() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, 0);
  tft.print("=== SELECT PROFILE ===");

  for (int i = 0; i < numProfiles; i++) {
    tft.setCursor(0, (i + 2) * LINE_H);
    if (i == menuSelection) tft.setTextColor(TFT_GREEN, TFT_BLACK);
    else tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.print(i == menuSelection ? "> " : "  ");
    tft.print(profiles[i].name);
  }
}

// ── SCPI send ─────────────────────────────────────────────────────
static void sendScpi(const char* cmd, bool log_it = true) {
  if (log_it) tft_logf("[>] %s", cmd);
  if (meter_ready) {
    userial.print(cmd);
    userial.write("\r\n");
  }
}

// ── Profile activation ────────────────────────────────────────────
static void applyProfile(int idx) {
  const Profile& p = profiles[idx];
  selectedProfile = idx;
  EEPROM.write(0, (uint8_t)idx);

  if (meter_ready) userial.begin(p.baud);

  tft_logf("Switched to: %s", p.name);

  if (p.init_cmds[0] != nullptr) {
    tft_log("Running init...");
    for (int i = 0; p.init_cmds[i] != nullptr; i++) {
      sendScpi(p.init_cmds[i], false);
      if (strstr(p.init_cmds[i], "*rst")) delay(6000);
      else delay(200);
    }
  }
  tft_log("Profile ready");
}

// ===================================================================
// Setup
// ===================================================================
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  pinMode(ENC_SW, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  selectedProfile = EEPROM.read(0);
  if (selectedProfile >= numProfiles) selectedProfile = 0;

  tft_log("Teensy 4.0 multi-profile ready");
  tft_log("USB-A socket soldered to bottom D+/D- pads");
  tft_logf("Last profile: %s", profiles[selectedProfile].name);

  myusb.begin();
}

// ===================================================================
// Main loop
// ===================================================================
void loop() {
  static uint32_t hb_last = 0;
  if (millis() - hb_last >= 5000) {
    hb_last = millis();
    if (Serial) Serial.printf("[HB] %lu\n", hb_last / 1000);
  }

  myusb.Task();

  // Button (debounced)
  static uint32_t btn_last = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - btn_last > 250) {
    btn_last = millis();
    if (inMenu) {
      applyProfile(menuSelection);
      inMenu = false;
      redrawScroll();
    } else {
      inMenu = true;
      menuSelection = selectedProfile;
      tft_drawMenu();
    }
  }

  // Encoder (menu only)
  static long oldPosition = myEnc.read();
  long newPos = myEnc.read();
  if (inMenu && newPos != oldPosition) {
    int delta = (newPos - oldPosition) / 4;
    menuSelection += delta;
    if (menuSelection < 0) menuSelection = 0;
    if (menuSelection >= numProfiles) menuSelection = numProfiles - 1;
    tft_drawMenu();
    oldPosition = newPos;
  }

  // Connect / disconnect
  if (userial && !meter_ready) {
    meter_ready = true;
    userial.begin(profiles[selectedProfile].baud);
    tft_log("FT232R connected");
    applyProfile(selectedProfile);
  }
  if (!userial && meter_ready) {
    meter_ready = false;
    tft_log("--- disconnected ---");
  }

  // Serial monitor passthrough
  static char cmd_buf[128];
  static uint8_t cmd_pos = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_pos > 0) {
        cmd_buf[cmd_pos] = '\0';
        char buf[LINE_W];
        snprintf(buf, LINE_W, "[>] %-17s", cmd_buf);
        tft_log(buf);
        if (meter_ready) sendScpi(cmd_buf, false);
        cmd_pos = 0;
      }
    } else if (cmd_pos < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_pos++] = c;
    }
  }

  // SCPI RX + auto-poll
  if (meter_ready && userial.available()) {
    while (userial.available()) {
      char c = (char)userial.read();
      if (c == '\n' || c == '\r') {
        if (rx_line_pos > 0) {
          rx_line[rx_line_pos] = '\0';
          if (!inMenu) {
            char buf[LINE_W];
            snprintf(buf, LINE_W, "[RX] %-16s", rx_line);
            tft_log(buf);
          }
          if (profiles[selectedProfile].poll_cmd != nullptr) {
            sendScpi(profiles[selectedProfile].poll_cmd, false);
          }
          rx_line_pos = 0;
        }
      } else if (rx_line_pos < sizeof(rx_line) - 1) {
        rx_line[rx_line_pos++] = c;
      }
    }
  }
}
