/*
 * tft_test.ino
 *
 * TFT_eSPI + encoder test for ST7735S 128x160 on Teensy 4.0
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
 *  Encoder    | Silkscreen | Physical pin
 *  -----------|------------|-------------
 *  SW         |      2     |      4
 *  A          |      3     |      5
 *  B          |      4     |      6
 *
 * TFT pins defined in TFT_eSPI User_Setup.h
 */

#include <TFT_eSPI.h>
#include <Encoder.h>

#define TFT_BLK 7
#define ENC_SW  2
#define ENC_A   3
#define ENC_B   4

TFT_eSPI tft;
Encoder  enc(ENC_A, ENC_B);

void setup() {
  Serial.begin(115200);

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);
  pinMode(ENC_SW, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Teensy 4.0 TFT+ENC");
  tft.println("ST7735S 128x160");

  if (Serial) Serial.println("init done");
}

void loop() {
  static long lastEncPos  = 0;
  static bool lastBtn     = HIGH;
  static int  btnCount    = 0;

  // Encoder position
  long pos = enc.read() / 4;
  if (pos != lastEncPos) {
    lastEncPos = pos;
    tft.setCursor(0, 24);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.printf("enc: %ld      ", pos);
    if (Serial) Serial.printf("enc: %ld\n", pos);
  }

  // Button press (debounced)
  bool btn = digitalRead(ENC_SW);
  if (btn == LOW && lastBtn == HIGH) {
    delay(20);
    if (digitalRead(ENC_SW) == LOW) {
      btnCount++;
      tft.setCursor(0, 32);
      tft.printf("btn: %d      ", btnCount);
      if (Serial) Serial.printf("btn: %d\n", btnCount);
    }
  }
  lastBtn = btn;
}
