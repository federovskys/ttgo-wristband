#include "util.h"
#include <MPU9250.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_adc_cal.h>
#include <pcf8563.h>

TFT_eSPI tft = TFT_eSPI();
MPU9250 imu;
PCF8563_Class rtc;

int vref = 1100;

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
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
}

void initSensor() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  imu.initMPU9250();
}

void initRTC() {
  rtc.begin(Wire);
  rtc.check();  // Check if the RTC clock matches, if not, use compile time

  // RTC_Date datetime = rtc.getDateTime();
  // hh = datetime.hour;
  // mm = datetime.minute;
  // ss = datetime.second;
}

void disableRadios() {
  WiFi.mode(WIFI_OFF);
  btStop();
}

void deepSleep() {
  imu.setSleepEnabled(true);
  tft.writecommand(ST7735_SLPIN);
  tft.writecommand(ST7735_DISPOFF);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}

void drawTestScreen() {
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Hello World ", 0, 0);
  tft.setTextColor(TFT_RED);
  tft.drawString("Red", 0, 12);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("GREEN", 0, 24);
  tft.setTextColor(TFT_BLUE);
  tft.drawString("Blue", 0, 36);
}