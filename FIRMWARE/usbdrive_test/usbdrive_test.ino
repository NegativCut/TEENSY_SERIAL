/*
 * usbdrive_test.ino
 *
 * Minimal USB flash drive test for Teensy 4.0
 * Tests: mount, file create, write rows, close, verify read-back
 *
 *  TFT signal | Silkscreen | Physical pin
 *  -----------|------------|-------------
 *  MOSI       |     11     |     13
 *  SCK        |     13     |     15
 *  CS         |     10     |     12
 *  DC         |      9     |     11
 *  RST        |      8     |     10
 *  BLK        |      7     |      9
 *
 * USB-A socket soldered to bottom D+/D- pads
 * TFT pins defined in TFT_eSPI User_Setup.h
 */

#include <TFT_eSPI.h>
#include <USBHost_t36.h>

#define TFT_BLK     7
#define TEST_FILE   "drivetest.csv"
#define TEST_ROWS   10

TFT_eSPI       tft;
USBHost        myusb;
USBDrive       myDrive(myusb);
USBFilesystem  myFS(myusb);

static bool tested = false;

static void log(const char* msg) {
  static uint8_t line = 0;
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, line * 8);
  tft.printf("%-21s", msg);
  if (line < 19) line++;
  if (Serial) Serial.println(msg);
}

static void logf(const char* fmt, ...) {
  char buf[32];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  log(buf);
}

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  log("USB drive test");
  log("plug in flash drive...");

  myusb.begin();
}

void loop() {
  myusb.Task();

  if (myFS && !tested) {
    tested = true;

    log("drive mounted");

    // ── Write test ──────────────────────────────────────────────
    File f = myFS.open(TEST_FILE, FILE_WRITE);
    if (!f) {
      log("FAIL: open for write");
      return;
    }
    log("writing rows...");
    f.println("row,value");
    for (int i = 0; i < TEST_ROWS; i++) {
      f.printf("%d,%d\n", i, i * i);
    }
    f.close();
    logf("wrote %d rows", TEST_ROWS);

    // ── Read-back verify ────────────────────────────────────────
    f = myFS.open(TEST_FILE, FILE_READ);
    if (!f) {
      log("FAIL: open for read");
      return;
    }
    int rowCount = 0;
    while (f.available()) {
      String line = f.readStringUntil('\n');
      rowCount++;
      if (Serial) Serial.println(line);
    }
    f.close();
    logf("readback: %d lines", rowCount);

    // ── Result ──────────────────────────────────────────────────
    if (rowCount == TEST_ROWS + 1) {  // +1 for header
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      log("PASS");
    } else {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      logf("FAIL: got %d lines", rowCount);
    }
  }

  if (!myFS && tested) {
    tested = false;
    tft.fillScreen(TFT_BLACK);
    log("drive removed");
    log("plug in flash drive...");
  }
}
