
#include <Arduino.h>
#include "util.h"

void setup() {
  initRTC();
  disableRadios();
  setLowCPUSpeed();
  initGPIOs();
  initSensor();
  initScreen();
}

void loop() {
  drawTestScreen();
  delay(4000);
  deepSleep();

  // if (digitalRead(TP_PIN_PIN) == HIGH) {
  //   delay(1000);
  //   deepSleep();
  // }
}