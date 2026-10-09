#pragma once
/*
COMPUTATION DATA STORAGE (MICROSD)
INTERFACE : SPI (VSPI default)
PIN
CS : 5
SCK : 18
MISO : 19
MOSI : 23
FILESYSTEM : FAT32

BUFFER : SN74HC125 (quad bus buffer, OE aktif LOW)
  Buffer 1 : MOSI  ESP32 -> SD   (1A = GPIO23, 1Y = SD DI,  1OE = GND)
  Buffer 2 : SCK   ESP32 -> SD   (2A = GPIO18, 2Y = SD CLK, 2OE = GND)
  Buffer 3 : CS    ESP32 -> SD   (3A = GPIO5,  3Y = SD CS,   3OE = GND)
  Buffer 4 : MISO  SD -> ESP32   (4A = SD DO,  4Y = GPIO19,  4OE = GPIO5/CS)
  VCC = 3.3V, GND = GND
  4OE disambung ke CS supaya MISO hanya aktif saat SD dipilih,
  sehingga SD tidak mengganggu perangkat lain di bus VSPI yang sama.

  DATA YANG INGIN DI LOGGING
  1. BMS
  2. GPS
  3. MOTOR CONTROLLER
  4. RTC
  5. COMP DATA
  6. LAP TIME
  7. NEXTION DATA
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ---------------- Konfigurasi ----------------
#define SD_CS_PIN        5
#define SD_SCK_PIN       18
#define SD_MISO_PIN      19
#define SD_MOSI_PIN      23
#define SD_FREQ_HZ       20000000UL   // frekuensi utama
#define SD_FREQ_SAFE_HZ  4000000UL    // fallback jika mount gagal
#define SD_LOG_PATH      "/log.csv"
#define SD_LINE_MAX      128          // panjang maksimum 1 baris log
#define SD_QUEUE_LEN     16
#define SD_FLUSH_MS      2000         // interval flush ke kartu
#define SD_RETRY_MS      5000         // interval coba mount ulang

// ---------------- Data ----------------
struct SdStatus {
  bool     mounted;
  uint64_t totalBytes;
  uint64_t usedBytes;
  uint32_t linesWritten;
  uint32_t linesDropped;   // antrian penuh
  uint32_t writeErrors;
  uint32_t lastWriteMs;
};

struct SdLine {
  char text[SD_LINE_MAX];
};

static SPIClass          g_sdSpi(VSPI);
static SdStatus          g_sd = {false, 0, 0, 0, 0, 0, 0};
static SemaphoreHandle_t g_sdMutex = nullptr;
static QueueHandle_t     g_sdQueue = nullptr;
static File              g_sdFile;

// ---------------- Helper internal ----------------
static bool sdMount() {
  g_sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  const uint32_t freqs[2] = {SD_FREQ_HZ, SD_FREQ_SAFE_HZ};
  for (uint8_t i = 0; i < 2; i++) {
    SD.end();
    if (SD.begin(SD_CS_PIN, g_sdSpi, freqs[i])) {
      if (SD.cardType() == CARD_NONE) continue;
      Serial.printf("[SD] Mount OK @ %lu Hz\n", (unsigned long)freqs[i]);
      return true;
    }
  }
  return false;
}

static bool sdOpenLog() {
  if (g_sdFile) g_sdFile.close();
  g_sdFile = SD.open(SD_LOG_PATH, FILE_APPEND);
  return (bool)g_sdFile;
}

static void sdSetMounted(bool m) {
  if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(20))) {
    g_sd.mounted = m;
    if (m) {
      g_sd.totalBytes = SD.totalBytes();
      g_sd.usedBytes  = SD.usedBytes();
    }
    xSemaphoreGive(g_sdMutex);
  }
}

// ---------------- INIT ----------------
static bool sdInit() {
  if (g_sdMutex == nullptr) g_sdMutex = xSemaphoreCreateMutex();
  if (g_sdQueue == nullptr) g_sdQueue = xQueueCreate(SD_QUEUE_LEN, sizeof(SdLine));

  if (!sdMount()) {
    Serial.println("[SD] Kartu tidak terdeteksi / gagal mount (cek FAT32 & wiring HC125)");
    sdSetMounted(false);
    return false;
  }
  if (!sdOpenLog()) {
    Serial.println("[SD] Gagal membuka file log");
    sdSetMounted(false);
    return false;
  }

  sdSetMounted(true);
  Serial.println("[SD] Siap");
  return true;
}

// ---------------- API LOGGING (aman dipanggil dari task manapun) ----------------
// Mengirim 1 baris ke antrian. Tidak memblokir; return false jika antrian penuh.
static bool sdLog(const char* line) {
  if (g_sdQueue == nullptr) return false;
  SdLine l;
  strncpy(l.text, line, SD_LINE_MAX - 1);
  l.text[SD_LINE_MAX - 1] = '\0';
  if (xQueueSend(g_sdQueue, &l, 0) != pdTRUE) {
    if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(5))) {
      g_sd.linesDropped++;
      xSemaphoreGive(g_sdMutex);
    }
    return false;
  }
  return true;
}

static bool sdLogf(const char* fmt, ...) {
  char buf[SD_LINE_MAX];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return sdLog(buf);
}

// ---------------- UPDATE ----------------
// Ambil satu baris dari antrian lalu tulis ke kartu. Flush berkala.
static bool sdUpdate() {
  static uint32_t lastFlush = 0;
  static uint32_t lastRetry = 0;

  // Belum mount: coba ulang secara berkala
  if (!g_sd.mounted) {
    if (millis() - lastRetry >= SD_RETRY_MS) {
      lastRetry = millis();
      if (sdMount() && sdOpenLog()) sdSetMounted(true);
    }
    return false;
  }

  SdLine l;
  bool wrote = false;
  while (xQueueReceive(g_sdQueue, &l, 0) == pdTRUE) {
    size_t n = g_sdFile.println(l.text);
    if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(20))) {
      if (n == 0) {
        g_sd.writeErrors++;
      } else {
        g_sd.linesWritten++;
        g_sd.lastWriteMs = millis();
        wrote = true;
      }
      xSemaphoreGive(g_sdMutex);
    }
    if (n == 0) {              // tulis gagal -> anggap kartu lepas
      g_sdFile.close();
      SD.end();
      sdSetMounted(false);
      lastRetry = millis();
      return false;
    }
  }

  if (millis() - lastFlush >= SD_FLUSH_MS) {
    lastFlush = millis();
    g_sdFile.flush();
    if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(20))) {
      g_sd.usedBytes = SD.usedBytes();
      xSemaphoreGive(g_sdMutex);
    }
  }
  return wrote;
}

// ---------------- GETTER (thread-safe) ----------------
static bool sdGet(SdStatus& out) {
  if (g_sdMutex == nullptr) return false;
  if (xSemaphoreTake(g_sdMutex, pdMS_TO_TICKS(20))) {
    out = g_sd;
    xSemaphoreGive(g_sdMutex);
    return true;
  }
  return false;
}
