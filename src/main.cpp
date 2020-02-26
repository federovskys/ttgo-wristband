
#include <Arduino.h>
#include "util.h"

void setup() {
  disableRadios();
  setLowCPUSpeed();
  initGPIOs();
  initRTC();
  initSensor();
  initScreen();
}

void loop() {
  drawTestScreen();

  delay(250);

  if (digitalRead(TP_PIN_PIN) == HIGH) {
    delay(1000);
    deepSleep();
  }
}