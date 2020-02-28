#include "util.h"
#include <MPU9250.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp32-hal-cpu.h>
#include <esp_adc_cal.h>
#include <pcf8563.h>
#include "sensor.h"

#define FONT_TIME "SourceCodePro-Regular-48"
#define FONT_TIME_PATH "/SourceCodePro-Regular-48.vlw"
bool hasFontTime = false;

#define FONT_SIZE_16 "SourceCodePro-Regular-16"
#define FONT_SIZE_16_PATH "/SourceCodePro-Regular-16.vlw"
bool hasFontSize16 = false;

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

void setupWatch() {
  Serial.begin(115200);
  SPIFFS.begin();
  initRTC();
  disableRadios();
  initGPIOs();
  initSensor();
  initScreen();
}

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

void initSensor() {
  imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
  imu.initMPU9250();
  imu.initAK8963(imu.magCalibration);
}

void initScreen() {
  wakeUpCount++;
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);

  if (SPIFFS.exists(FONT_TIME_PATH)) {
    hasFontTime = true;
  }
  if (SPIFFS.exists(FONT_SIZE_16_PATH)) {
    hasFontSize16 = true;
  }
}

void disableRadios() {
  WiFi.mode(WIFI_OFF);
  btStop();
}

void setMaxCPUSpeed() { setCpuFrequencyMhz(240); }
void setMinCPUSpeed() {
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

void clearScreen() { tft.fillScreen(TFT_BLACK); }

void drawTime() {
  RTC_Date now = rtc.getDateTime();

  if (hasFontTime) {
    tft.loadFont(FONT_TIME);
  }

  String hh = now.hour < 10 ? "0" + String(now.hour) : String(now.hour);
  String mm = now.minute < 10 ? "0" + String(now.minute) : String(now.minute);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(hh + ":" + mm, 9, 20);

  if (hasFontTime) {
    tft.unloadFont();
  }
}

void drawDateRow() {
  RTC_Date now = rtc.getDateTime();
  if (hasFontSize16) {
    tft.loadFont(FONT_SIZE_16);
  }

  String yyyy = String(now.year);
  String mm = now.month < 10 ? "0" + String(now.month) : String(now.month);
  String dd = now.day < 10 ? "0" + String(now.day) : String(now.day);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(yyyy + "-" + mm + "-" + dd, 31, 2);

  if (hasFontSize16) {
    tft.unloadFont();
  }
}

void drawInfoRow() {
  if (hasFontSize16) {
    tft.loadFont(FONT_SIZE_16);
  }

  uint16_t v = analogRead(BATT_ADC_PIN);
  float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
  String voltage = String(battery_voltage, 2) + "V";

  if (isCharging()) {
    voltage = voltage + " CHG";
  } else {
    voltage = voltage + "    ";
  }

  String wakeups = (wakeUpCount < 100 ? (wakeUpCount < 10 ? "00" : "0") : "") + String(wakeUpCount);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString(voltage, 0, 64);
  tft.drawString(wakeups, 128, 64);

  if (hasFontSize16) {
    tft.unloadFont();
  }
}

void drawCompass() {
  readMPU9250(&imu);

  if (hasFontSize16) {
    tft.loadFont(FONT_SIZE_16);
  }
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  String x = String(imu.mx, 1);
  String y = String(imu.my, 1);
  String z = String(imu.mz, 1);
  tft.drawString(x, 64, 16);
  tft.drawString(y, 64, 40);
  tft.drawString(z, 64, 64);

  if (hasFontSize16) {
    tft.unloadFont();
  }
}

bool isCharging() {
  return digitalRead(CHARGE_PIN) == LOW;
  wakeUpCount = 0;
}

bool isButtonDown() { return digitalRead(TP_PIN_PIN) == HIGH; }