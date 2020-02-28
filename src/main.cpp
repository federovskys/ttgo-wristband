
#include <Arduino.h>
#include "util.h"

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
  while (millis() - started < 3000 || digitalRead(TP_PIN_PIN) == HIGH) {
    digitalWrite(LED_PIN, digitalRead(TP_PIN_PIN));
    // keep screen on for 3s or as long as the finger is on the touch button
    delay(250);
    drawInfoRow();
  }

  deepSleep();
}