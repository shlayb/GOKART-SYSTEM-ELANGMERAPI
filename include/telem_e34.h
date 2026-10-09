#pragma once
/*
TELEMETRY E34 (EBYTE wireless module)
INTERFACE : UART2
PIN
RX : 16   (UART2 default RX)
TX : 17   (UART2 default TX)
BAUDRATE : 115200
FORMAT : 8N1
TODO : pin M0 / M1 / AUX (mode config) belum didefinisikan

PAKET : ASCII, 1 baris per kiriman
  $TLM,seq,ms,speed,rpm,batt,vbat,cbat,tcon,tbat,boost*XX\n
  XX = XOR semua karakter antara '$' dan '*' (2 digit hex)
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ---------------- Konfigurasi ----------------
#define TELEM_RX_PIN      16
#define TELEM_TX_PIN      17
#define TELEM_BAUD        115200
#define TELEM_UART_NUM    2
#define TELEM_PKT_MAX     96
#define TELEM_AUX_WAIT_MS 50

// TODO: isi nomor GPIO setelah wiring ditentukan. -1 = tidak dipakai.
// Kalau M0/M1 diisi, init set mode normal (M0 = LOW, M1 = LOW).
// Kalau AUX diisi, kirim hanya dilakukan saat AUX HIGH (modul siap).
#define TELEM_M0_PIN      -1
#define TELEM_M1_PIN      -1
#define TELEM_AUX_PIN     -1

// ---------------- Data ----------------
struct TelemData {
  int   speed;
  int   rpm;
  int   batt;
  float vbat;
  float cbat;
  int   tcon;
  int   tbat;
  bool  boost;
};

struct TelemStatus {
  uint32_t seq;        // jumlah paket terkirim
  uint32_t failed;     // gagal kirim (AUX timeout / buffer TX penuh)
  uint32_t lastTxMs;
  bool     ready;
};

static HardwareSerial    g_telem(TELEM_UART_NUM);
static TelemData         g_telemData = {0, 0, 0, 0.0f, 0.0f, 0, 0, false};
static TelemStatus       g_telemSt   = {0, 0, 0, false};
static SemaphoreHandle_t g_telemMutex = nullptr;

// ---------------- Helper internal ----------------
static uint8_t telemChecksum(const char* s, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) c ^= (uint8_t)s[i];
  return c;
}

static bool telemWaitAux() {
#if TELEM_AUX_PIN >= 0
  uint32_t t0 = millis();
  while (digitalRead(TELEM_AUX_PIN) == LOW) {
    if (millis() - t0 > TELEM_AUX_WAIT_MS) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
#endif
  return true;
}

// ---------------- INIT ----------------
static bool telemInit() {
  if (g_telemMutex == nullptr) g_telemMutex = xSemaphoreCreateMutex();

#if TELEM_M0_PIN >= 0
  pinMode(TELEM_M0_PIN, OUTPUT);
  digitalWrite(TELEM_M0_PIN, LOW);
#endif
#if TELEM_M1_PIN >= 0
  pinMode(TELEM_M1_PIN, OUTPUT);
  digitalWrite(TELEM_M1_PIN, LOW);
#endif
#if TELEM_AUX_PIN >= 0
  pinMode(TELEM_AUX_PIN, INPUT);
#endif

  g_telem.begin(TELEM_BAUD, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);
  vTaskDelay(pdMS_TO_TICKS(100));   // beri waktu modul selesai boot

  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    g_telemSt.ready = true;
    xSemaphoreGive(g_telemMutex);
  }

  Serial.println("[TELEM] E34 siap @ 115200");
  return true;
}

// ---------------- UPDATE ----------------
// Semua data dikirim lewat parameter. Return true jika paket terkirim.
static bool telemUpdate(int speed, int rpm, int batt,
                        float vbat, float cbat,
                        int tcon, int tbat, bool boost) {
  if (!g_telemSt.ready) return false;

  char body[TELEM_PKT_MAX - 8];
  int n = snprintf(body, sizeof(body), "TLM,%lu,%lu,%d,%d,%d,%.1f,%.1f,%d,%d,%d",
                   (unsigned long)g_telemSt.seq, (unsigned long)millis(),
                   speed, rpm, batt, vbat, cbat, tcon, tbat, boost ? 1 : 0);
  if (n <= 0 || n >= (int)sizeof(body)) return false;

  char pkt[TELEM_PKT_MAX];
  int len = snprintf(pkt, sizeof(pkt), "$%s*%02X\n", body, telemChecksum(body, n));

  bool ok = telemWaitAux() && (g_telem.availableForWrite() >= len);
  if (ok) g_telem.write((const uint8_t*)pkt, len);

  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    if (ok) { g_telemSt.seq++; g_telemSt.lastTxMs = millis(); }
    else    { g_telemSt.failed++; }
    xSemaphoreGive(g_telemMutex);
  }
  return ok;
}

// Overload: kirim dari struct
static bool telemUpdate(const TelemData& d) {
  return telemUpdate(d.speed, d.rpm, d.batt, d.vbat, d.cbat, d.tcon, d.tbat, d.boost);
}

// ---------------- SETTER / GETTER (thread-safe) ----------------
static bool telemSetData(const TelemData& in) {
  if (g_telemMutex == nullptr) return false;
  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    g_telemData = in;
    xSemaphoreGive(g_telemMutex);
    return true;
  }
  return false;
}

static bool telemGetData(TelemData& out) {
  if (g_telemMutex == nullptr) return false;
  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    out = g_telemData;
    xSemaphoreGive(g_telemMutex);
    return true;
  }
  return false;
}

static bool telemGetStatus(TelemStatus& out) {
  if (g_telemMutex == nullptr) return false;
  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    out = g_telemSt;
    xSemaphoreGive(g_telemMutex);
    return true;
  }
  return false;
}

// ---------------- TASK RTOS ----------------
// Task ini mengirim data terakhir dari telemSetData().
// taskTelemetry kamu tidak diubah, jadi telemUpdate() versi tanpa
// parameter dibutuhkan sebagai wrapper:
static bool telemUpdate() {
  TelemData d;
  if (!telemGetData(d)) return false;
  return telemUpdate(d);
}