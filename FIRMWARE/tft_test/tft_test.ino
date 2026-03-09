/*
 * tft_test.ino
 *
 * Minimal TFT_eSPI test for ST7735S 128x160 on Teensy 4.0
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
 * Pins defined in TFT_eSPI User_Setup.h
 */

#include <TFT_eSPI.h>

#define TFT_BLK 7

TFT_eSPI tft;

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Teensy 4.0 TFT test");
  tft.println("ST7735S 128x160");

  if (Serial) Serial.println("TFT init done");
}

void loop() {
  static uint32_t count = 0;
  static uint32_t last = 0;

  if (millis() - last >= 1000) {
    last = millis();
    tft.setCursor(0, 24);
    tft.printf("count: %lu   ", count++);
    if (Serial) Serial.printf("count: %lu\n", count);
  }
}
