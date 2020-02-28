
#include <Arduino.h>
#include "util.h"

enum MODE { SCREEN_TIME = 0, SCREEN_COMPASS = 1 };
uint8_t mode = SCREEN_TIME;
#define NUM_MODES 2

int screenTimeout = 3000;

void setup() {
  setupWatch();
  clearScreen();
}

void loop() {
  drawDateRow();
  drawTime();
  drawInfoRow();
  // setMinCPUSpeed();

  long started = millis();
  while (millis() - started < screenTimeout) {
    // digitalWrite(LED_PIN, digitalRead(TP_PIN_PIN));
    if (isButtonDown()) {
      mode++;
      mode = mode > NUM_MODES ? 0 : mode;
      clearScreen();
      started = millis();
    }
    switch (mode) {
      case SCREEN_TIME:
        screenTimeout = 3 * 1000;
        drawInfoRow();
        break;
      case SCREEN_COMPASS:
        screenTimeout = 60 * 1000;
        drawCompass();
    }
    delay(250);
  }

  deepSleep();
}