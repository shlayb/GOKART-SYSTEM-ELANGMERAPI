#include <Arduino.h>

#include "comp_data.h"
#include "acq_rtc.h"
#include "acq_gps.h"  
#include "acq_bms.h"
#include "acq_motcon.h"
#include "comp_nextion.h"
#include "telem_e34.h"
#include "com_mircosd.h"

// ---------------- Config timing (ms) ----------------
#define RTC_PERIOD_MS      1000
#define GPS_PERIOD_MS      50
#define BMS_PERIOD_MS      1000
#define MOTCON_PERIOD_MS   5
#define NEXTION_PERIOD_MS  500
#define TELEM_PERIOD_MS    1000
#define SD_PERIOD_MS       1000

// ---------------- Tasks ----------------
static void taskRtc(void*) {
  rtcInit();
  for (;;) { rtcUpdate(); vTaskDelay(pdMS_TO_TICKS(RTC_PERIOD_MS)); }
}

static void taskGps(void*) {
  gpsInit();
  for (;;) { gpsUpdate(); vTaskDelay(pdMS_TO_TICKS(GPS_PERIOD_MS)); }
}

static void taskBms(void*) {
  bmsInit();
  for (;;) { bmsUpdate(); vTaskDelay(pdMS_TO_TICKS(BMS_PERIOD_MS)); }
}

static void taskMotcon(void*) {
  motconInit();
  for (;;) { motconUpdate(); vTaskDelay(pdMS_TO_TICKS(MOTCON_PERIOD_MS)); }
}

static void taskNextion(void*) {
  nextionInit();
  for (;;) {
    NextionData d;
    if (nextionGetData(d)) nextionUpdate(d);
    vTaskDelay(pdMS_TO_TICKS(NEXTION_PERIOD_MS));
  }
}
static void taskTelemetry(void*) {
  telemInit();
  for (;;) { telemUpdate(); vTaskDelay(pdMS_TO_TICKS(TELEM_PERIOD_MS)); }
}

static void taskSd(void*) {
  sdInit();
  for (;;) { sdUpdate(); vTaskDelay(pdMS_TO_TICKS(SD_PERIOD_MS)); }
}

// ---------------- Setup ----------------
void setup() {
  // dataInit();  // shared data harus siap sebelum task jalan

  //                 fungsi         nama        stack  arg   prio  handle core
  xTaskCreatePinnedToCore(taskMotcon,    "motcon",   3072, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(taskBms,       "bms",      4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskGps,       "gps",      4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskRtc,       "rtc",      3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskNextion,   "nextion",  3072, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskTelemetry, "telem",    3072, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSd,        "sd",       6144, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}