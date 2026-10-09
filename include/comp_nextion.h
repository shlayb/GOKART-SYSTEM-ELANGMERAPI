#pragma once
/*
NEXTION DISPLAY (HMI)
INTERFACE : UART
PIN
TX : 27   (ESP32 TX -> Nextion RX)
RX : 26   (ESP32 RX <- Nextion TX)
BAUDRATE : 9600
FORMAT : 8N1, command diakhiri 0xFF 0xFF 0xFF

KOMPONEN : speed, rpm, rrpm, batt, v_bat, c_bat, t_con, t_bat, bt0
*/

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <limits.h>

// ---------------- Konfigurasi ----------------
#define NEXTION_TX_PIN     27
#define NEXTION_RX_PIN     26
#define NEXTION_BAUD       9600
#define NEXTION_UART_NUM   3
#define NEXTION_CMD_MAX    64

// ---------------- Data ----------------
struct NextionData {
  int   speed;     // km/h
  int   rpm;
  int   batt;      // persen
  float vbat;      // volt
  float cbat;      // ampere
  int   tcon;      // suhu controller (C)
  int   tbat;      // suhu baterai (C)
  bool  boost;
};

static HardwareSerial    g_nextion(NEXTION_UART_NUM);
static NextionData       g_nxData = {0, 0, 0, 0.0f, 0.0f, 0, 0, false};
static SemaphoreHandle_t g_nxMutex = nullptr;
static bool              g_nxForce = true;   // true = kirim semua komponen (refresh penuh)

// ---------------- Helper internal ----------------
static void nextionSendCmd(const char* cmd) {
  g_nextion.print(cmd);
  g_nextion.write(0xFF);
  g_nextion.write(0xFF);
  g_nextion.write(0xFF);
}

static void nextionSendCmdf(const char* fmt, ...) {
  char buf[NEXTION_CMD_MAX];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  nextionSendCmd(buf);
}

// ---------------- INIT ----------------
static bool nextionInit() {
  if (g_nxMutex == nullptr) g_nxMutex = xSemaphoreCreateMutex();

  g_nextion.begin(NEXTION_BAUD, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN);
  vTaskDelay(pdMS_TO_TICKS(150));

  // Flush buffer perintah di sisi Nextion
  g_nextion.write(0xFF);
  g_nextion.write(0xFF);
  g_nextion.write(0xFF);
  vTaskDelay(pdMS_TO_TICKS(50));

  nextionSendCmd("bt0.val=0");
  g_nxForce = true;   // update pertama mengirim semua komponen

  Serial.println("[NEXTION] Siap @ 9600");
  return true;
}

// ---------------- UPDATE ----------------
// Semua data dikirim lewat parameter. Hanya nilai yang berubah yang dikirim
// (hemat bandwidth di 9600 baud). Return jumlah perintah yang dikirim.
static uint8_t nextionUpdate(int speed, int rpm, int batt,
                             float vbat, float cbat,
                             int tcon, int tbat, bool boost) {
  // Cache nilai terakhir yang terkirim. Sentinel membuat kiriman pertama pasti lolos.
  static int  lSpeed = INT_MIN, lRpm = INT_MIN, lBatt = INT_MIN;
  static int  lVbat = INT_MIN, lCbat = INT_MIN;   // dalam 0.1 satuan
  static int  lTcon = INT_MIN, lTbat = INT_MIN;
  static int  lBoost = -1;

  if (g_nxForce) {
    lSpeed = lRpm = lBatt = lVbat = lCbat = lTcon = lTbat = INT_MIN;
    lBoost = -1;
    g_nxForce = false;
  }

  uint8_t sent = 0;

  if (speed != lSpeed) {
    lSpeed = speed;
    nextionSendCmdf("speed.val=%d", speed);
    nextionSendCmdf("speed.txt=\"%d\"", speed);
    sent += 2;
  }

  if (rpm != lRpm) {
    lRpm = rpm;
    nextionSendCmdf("rpm.val=%d", rpm);
    nextionSendCmdf("rpm.txt=\"%d\"", rpm);
    nextionSendCmdf("rrpm.val=%d", rpm);
    sent += 3;
  }

  if (batt != lBatt) {
    lBatt = batt;
    nextionSendCmdf("batt.val=%d", batt);
    nextionSendCmdf("batt.txt=\"%d\"", batt);
    sent += 2;
  }

  int vbat10 = (int)lroundf(vbat * 10.0f);
  if (vbat10 != lVbat) {
    lVbat = vbat10;
    nextionSendCmdf("v_bat.txt=\"%.1f V\"", vbat);
    nextionSendCmdf("v_bat.val=%d", (int)vbat);
    sent += 2;
  }

  int cbat10 = (int)lroundf(cbat * 10.0f);
  if (cbat10 != lCbat) {
    lCbat = cbat10;
    nextionSendCmdf("c_bat.txt=\"%.1f A\"", cbat);
    nextionSendCmdf("c_bat.val=%d", (int)cbat);
    sent += 2;
  }

  if (tcon != lTcon) {
    lTcon = tcon;
    nextionSendCmdf("t_con.txt=\"%d C\"", tcon);
    nextionSendCmdf("t_con.val=%d", tcon);
    sent += 2;
  }

  if (tbat != lTbat) {
    lTbat = tbat;
    nextionSendCmdf("t_bat.txt=\"%d C\"", tbat);
    nextionSendCmdf("t_bat.val=%d", tbat);
    sent += 2;
  }

  if ((int)boost != lBoost) {
    lBoost = (int)boost;
    nextionSendCmdf("bt0.val=%d", boost ? 1 : 0);
    sent += 1;
  }

  return sent;
}

// Overload: kirim dari struct
static uint8_t nextionUpdate(const NextionData& d) {
  return nextionUpdate(d.speed, d.rpm, d.batt, d.vbat, d.cbat, d.tcon, d.tbat, d.boost);
}

// ---------------- SETTER / GETTER (thread-safe) ----------------
// Dipanggil dari task lain untuk mengisi data yang akan ditampilkan.
static bool nextionSetData(const NextionData& in) {
  if (g_nxMutex == nullptr) return false;
  if (xSemaphoreTake(g_nxMutex, pdMS_TO_TICKS(20))) {
    g_nxData = in;
    xSemaphoreGive(g_nxMutex);
    return true;
  }
  return false;
}

static bool nextionGetData(NextionData& out) {
  if (g_nxMutex == nullptr) return false;
  if (xSemaphoreTake(g_nxMutex, pdMS_TO_TICKS(20))) {
    out = g_nxData;
    xSemaphoreGive(g_nxMutex);
    return true;
  }
  return false;
}

// Paksa refresh penuh pada update berikutnya (misal setelah ganti page Nextion)
static void nextionRefresh() { g_nxForce = true; }
