#pragma once
/*
DATA ACQUISITION GPS
INTERFACE : UART
PIN
TX : 25
RX : 35   (input-only GPIO, aman untuk RX)
BAUDRATE : 9600
FORMAT : 8N1, NMEA
*/

#include <Arduino.h>
#include <TinyGPSPlus.h>   // library: TinyGPSPlus (Mikal Hart)

// ---------------- CONFIG ----------------
#define GPS_PIN_TX      25
#define GPS_PIN_RX      35
#define GPS_BAUD        9600
#define GPS_UART_NUM    4          // UART2
#define GPS_RX_BUF      1024

// ---------------- DATA ----------------
struct GpsData {
  bool     valid;        // fix valid & fresh
  double   lat;
  double   lon;
  double   alt_m;
  double   speed_kmph;
  double   course_deg;
  uint8_t  sat;
  double   hdop;
  uint16_t year;
  uint8_t  month, day;
  uint8_t  hour, minute, second;   // UTC
  uint32_t last_update_ms;
};

static HardwareSerial   gpsSerial(GPS_UART_NUM);
static TinyGPSPlus      gps;
static GpsData          gpsData = {};
static SemaphoreHandle_t gpsMutex = nullptr;
static TaskHandle_t     gpsTaskHandle = nullptr;

// ---------------- INIT ----------------
inline void gpsInit() {
  gpsMutex = xSemaphoreCreateMutex();
  gpsSerial.setRxBufferSize(GPS_RX_BUF);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_PIN_RX, GPS_PIN_TX);
}

// ---------------- UPDATE ----------------
// Baca byte NMEA dari UART, parse, lalu simpan ke gpsData (thread-safe).
inline void gpsUpdate(){
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  GpsData d = {};
  d.valid      = gps.location.isValid() && gps.location.age() < 2000;
  d.lat        = gps.location.lat();
  d.lon        = gps.location.lng();
  d.alt_m      = gps.altitude.meters();
  d.speed_kmph = gps.speed.kmph();
  d.course_deg = gps.course.deg();
  d.sat        = gps.satellites.isValid() ? gps.satellites.value() : 0;
  d.hdop       = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
  d.year       = gps.date.year();
  d.month      = gps.date.month();
  d.day        = gps.date.day();
  d.hour       = gps.time.hour();
  d.minute     = gps.time.minute();
  d.second     = gps.time.second();
  d.last_update_ms = millis();

  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    gpsData = d;
    xSemaphoreGive(gpsMutex);
  }
}

// ---------------- GETTER (dipanggil task lain) ----------------
inline bool gps_get(GpsData &out) {
  if (xSemaphoreTake(gpsMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
  out = gpsData;
  xSemaphoreGive(gpsMutex);
  return true;
}

