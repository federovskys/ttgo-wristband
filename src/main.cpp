
#include <Arduino.h>
#include <TFT_eSPI.h>

// src: https://github.com/Xinyuan-LilyGO/LilyGO-T-Wristband/blob/master/LilyGO-T-Wristband.ino
#define TP_PIN_PIN 33
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define IMU_INT_PIN 38
#define RTC_INT_PIN 34
#define BATT_ADC_PIN 35
#define VBUS_PIN 36
#define TP_PWR_PIN 25
#define LED_PIN 4
#define CHARGE_PIN 32

TFT_eSPI tft = TFT_eSPI();

void setup() {
  pinMode(LED_PIN, OUTPUT);
  // pinMode(TFT_BL, OUTPUT);
  // digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  // tft.loadFont(FreeMono12pt7b);
  tft.fillScreen(TFT_BLACK);
}

void loop() {
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Hello World ", 0, 0);
  tft.setTextColor(TFT_RED);
  tft.drawString("Red", 0, 12);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("GREEN", 0, 24);
  tft.setTextColor(TFT_BLUE);
  tft.drawString("Blue", 0, 36);

  delay(1000);
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}