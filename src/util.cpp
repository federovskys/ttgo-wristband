#include "util.h"
#include <MPU9250.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp32-hal-cpu.h>
#include <esp_adc_cal.h>
#include <pcf8563.h>

TFT_eSPI tft = TFT_eSPI();
MPU9250 imu;
PCF8563_Class rtc;

int vref = 1100;

RTC_DATA_ATTR int wakeUpCount = 0;
RTC_DATA_ATTR bool initStartTime = true;
RTC_DATA_ATTR uint8_t startDay = 0;
RTC_DATA_ATTR uint8_t startHour = 0;
RTC_DATA_ATTR uint8_t startMinute = 0;
RTC_DATA_ATTR uint8_t startSecond = 0;

void setupADC() {
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6,
                                                          (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  // Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    // Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    // Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    // Serial.println("Default Vref: 1100mV");
  }
}

void initGPIOs() {
  setupADC();

  pinMode(TP_PIN_PIN, INPUT);

  pinMode(CHARGE_PIN, INPUT_PULLUP);
  attachInterrupt(CHARGE_PIN, [] { /** nop **/ }, CHANGE);

  //! Must be set to pull-up output mode in order to wake up in deep sleep mode
  pinMode(TP_PWR_PIN, PULLUP);
  digitalWrite(TP_PWR_PIN, HIGH);

  pinMode(RTC_INT_PIN, INPUT_PULLUP);  // need change to rtc_pin
  attachInterrupt(RTC_INT_PIN, [] { /** nop **/ }, FALLING);

  pinMode(LED_PIN, OUTPUT);
}

void initScreen() {
  wakeUpCount++;
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
}

void initSensor() { imu.initMPU9250(); }

void initRTC() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  rtc.begin(Wire);

  if (initStartTime) {
    initStartTime = false;
    rtc.setDateTime(RTC_Date(__DATE__, __TIME__));
    RTC_Date now = rtc.getDateTime();
    startDay = now.day;
    startHour = now.hour;
    startMinute = now.minute;
    startSecond = now.second;
    //   startTime = start.second //
    //   +start.minute * 60 //
    //   +start.hour * 60 * 60
    //   +start.day
  }
  // RTC_Date datetime = rtc.getDateTime();
  // hh = datetime.hour;
  // mm = datetime.minute;
  // ss = datetime.second;
}

void disableRadios() {
  WiFi.mode(WIFI_OFF);
  btStop();
}

void setLowCPUSpeed() {
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-cpu.h
  // ESP32 PICO D4 -> https://docs.espressif.com/projects/esp-idf/en/latest/hw-reference/get-started-pico-kit.html
  // -> 40MHz Oscillator
  // //  240, 160, 80, 40, 20, 10  <<< For 40MHz XTAL
  setCpuFrequencyMhz(10);
}

void deepSleep() {
  imu.setSleepEnabled(true);
  tft.writecommand(ST7735_SLPIN);
  tft.writecommand(ST7735_DISPOFF);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void drawTestScreen() {
  RTC_Date now = rtc.getDateTime();
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(String("Started: ") + String(startDay) + "d" + String(startHour) + "h" + String(startMinute) + "m" +
                     String(startSecond) + "s",
                 0, 0);
  tft.drawString(String("Time: ") + String(now.day) + "d  " + String(now.hour) + ":" + String(now.minute) + ":" +
                     String(now.second),
                 0, 12);
  tft.drawString(String("Uptime: ") + String(now.day - startDay) + "d" + String(now.hour - startHour) + "h" +
                     String(now.minute - startMinute) + "m" + String(now.second - startSecond) + "s",
                 0, 24);
  tft.drawString(String("Wakeups: ") + String(wakeUpCount), 0, 36);

  uint16_t v = analogRead(BATT_ADC_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  tft.drawString(String("Voltage: ") + String(battery_voltage), 0, 48);

  if (digitalRead(CHARGE_PIN) == LOW) {
    tft.drawString(String("Charging..."), 0, 60);
  }
}