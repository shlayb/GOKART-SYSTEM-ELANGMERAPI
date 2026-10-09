#pragma once
/*
DATA ACQUISITION RTC DS3231
INTERFACE : I2C
PIN
SDA : 21
SCL : 22
ADDRESS : 0x68
*/

#include <Arduino.h>
#include <Wire.h>

// ---------------- Konfigurasi ----------------
#define RTC_SDA_PIN     21
#define RTC_SCL_PIN     22
#define RTC_I2C_ADDR    0x68
#define RTC_I2C_FREQ    400000UL

// ---------------- Register DS3231 ----------------
#define DS3231_REG_TIME     0x00
#define DS3231_REG_STATUS   0x0F
#define DS3231_REG_TEMP_MSB 0x11
#define DS3231_OSF_BIT      0x80

// ---------------- Data ----------------
struct RtcData {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
  uint8_t  dow;
  float    temp_c;
  bool     valid;
  bool     oscStopped;
  uint32_t lastReadMs;
};

static RtcData           g_rtc = {2000, 1, 1, 0, 0, 0, 1, 0.0f, false, false, 0};
static SemaphoreHandle_t g_rtcMutex = nullptr;
static bool              g_rtcPresent = false;

// ---------------- Helper ----------------
static inline uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static inline uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

static bool rtcReadRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)RTC_I2C_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool rtcWriteRegs(uint8_t reg, const uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) Wire.write(buf[i]);
  return Wire.endTransmission() == 0;
}

// ---------------- INIT ----------------
static bool rtcInit() {
  if (g_rtcMutex == nullptr) g_rtcMutex = xSemaphoreCreateMutex();

  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  Wire.setClock(RTC_I2C_FREQ);
  Wire.setTimeOut(50);

  Wire.beginTransmission(RTC_I2C_ADDR);
  g_rtcPresent = (Wire.endTransmission() == 0);
  if (!g_rtcPresent) {
    Serial.println("[RTC] DS3231 tidak terdeteksi di 0x68");
    return false;
  }

  uint8_t status = 0;
  if (rtcReadRegs(DS3231_REG_STATUS, &status, 1)) {
    bool osf = status & DS3231_OSF_BIT;
    if (xSemaphoreTake(g_rtcMutex, portMAX_DELAY)) {
      g_rtc.oscStopped = osf;
      xSemaphoreGive(g_rtcMutex);
    }
    if (osf) Serial.println("[RTC] Peringatan: oscillator pernah berhenti, waktu perlu di-set ulang");
  }

  Serial.println("[RTC] DS3231 siap");
  return true;
}

// ---------------- SET WAKTU ----------------
// dow: 1-7. Panggil sekali saja untuk set waktu awal.
static bool rtcSetTime(uint16_t year, uint8_t month, uint8_t day,
                       uint8_t hour, uint8_t minute, uint8_t second, uint8_t dow = 1) {
  uint8_t buf[7] = {
    dec2bcd(second), dec2bcd(minute), dec2bcd(hour),
    dec2bcd(dow), dec2bcd(day), dec2bcd(month),
    dec2bcd((uint8_t)(year - 2000))
  };

  bool ok = false;
  if (xSemaphoreTake(g_rtcMutex, pdMS_TO_TICKS(100))) {
    ok = rtcWriteRegs(DS3231_REG_TIME, buf, 7);
    if (ok) {
      uint8_t status = 0;
      if (rtcReadRegs(DS3231_REG_STATUS, &status, 1)) {
        status &= ~DS3231_OSF_BIT;
        rtcWriteRegs(DS3231_REG_STATUS, &status, 1);
        g_rtc.oscStopped = false;
      }
    }
    xSemaphoreGive(g_rtcMutex);
  }
  return ok;
}

// ---------------- UPDATE ----------------
static bool rtcUpdate() {
  if (!g_rtcPresent) return false;

  uint8_t t[7];
  uint8_t tmp[2];
  bool okTime = rtcReadRegs(DS3231_REG_TIME, t, 7);
  bool okTemp = rtcReadRegs(DS3231_REG_TEMP_MSB, tmp, 2);

  if (!okTime) {
    if (xSemaphoreTake(g_rtcMutex, pdMS_TO_TICKS(20))) {
      g_rtc.valid = false;
      xSemaphoreGive(g_rtcMutex);
    }
    return false;
  }

  RtcData d = g_rtc;
  d.second = bcd2dec(t[0] & 0x7F);
  d.minute = bcd2dec(t[1] & 0x7F);
  d.hour   = bcd2dec(t[2] & 0x3F);
  d.dow    = bcd2dec(t[3] & 0x07);
  d.day    = bcd2dec(t[4] & 0x3F);
  d.month  = bcd2dec(t[5] & 0x1F);
  d.year   = 2000 + bcd2dec(t[6]);

  if (okTemp) {
    int16_t raw = ((int16_t)(int8_t)tmp[0] << 2) | (tmp[1] >> 6);
    d.temp_c = raw * 0.25f;
  }

  d.valid = (d.second < 60 && d.minute < 60 && d.hour < 24 &&
             d.month >= 1 && d.month <= 12 && d.day >= 1 && d.day <= 31);
  d.lastReadMs = millis();

  if (xSemaphoreTake(g_rtcMutex, pdMS_TO_TICKS(20))) {
    g_rtc = d;
    xSemaphoreGive(g_rtcMutex);
  }
  return d.valid;
}

// ---------------- GETTER (thread-safe) ----------------
static bool rtcGet(RtcData& out) {
  if (g_rtcMutex == nullptr) return false;
  if (xSemaphoreTake(g_rtcMutex, pdMS_TO_TICKS(20))) {
    out = g_rtc;
    xSemaphoreGive(g_rtcMutex);
    return true;
  }
  return false;
}

// Format "YYYY-MM-DD HH:MM:SS" ke buffer (min 20 byte)
static void rtcFormat(char* buf, size_t len) {
  RtcData d;
  if (!rtcGet(d)) { snprintf(buf, len, "----"); return; }
  snprintf(buf, len, "%04u-%02u-%02u %02u:%02u:%02u",
           d.year, d.month, d.day, d.hour, d.minute, d.second);
}
