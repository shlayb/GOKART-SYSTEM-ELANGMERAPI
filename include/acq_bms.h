#pragma once
/*
JK BMS BD6A24S10P  (24S, RS485)  -- header-only, cukup #include "bms.h"
PROTOKOL : JK "RS485 Modbus V1.0"  (frame 4E 57 ... 68), master-poll
UART     : 115200 8N1 (default JK), pakai Serial2 ESP32
MODUL    : transceiver RS485 (MAX485 / auto-direction). Kalau modul butuh pin DE/RE, isi BMS_DE_PIN.

WIRING
- A/B modul RS485 ke port RS485 BMS (A->A, B->B). Kalau tidak ada balasan, tukar A/B.
- GND modul dan GND ESP32 harus sama.
- Di app JK, pastikan protokol port RS485 = "JK RS485 Modbus V1.0" dan alamat device = 0.

PAKAI
  static void taskBms(void*) {
    bmsInit();
    for (;;) { bmsUpdate(); vTaskDelay(pdMS_TO_TICKS(BMS_PERIOD_MS)); }
  }
  // di task lain:
  BmsData b;
  if (bmsGet(&b)) { ... b.totalV, b.soc, b.cellMv[] ... }
*/

#include <Arduino.h>

// ---------------- konfigurasi ----------------
#ifndef BMS_PERIOD_MS
#define BMS_PERIOD_MS 10          // periode polling
#endif

#define BMS_SERIAL          Serial2
#define BMS_BAUD            115200
#define BMS_RX_PIN          32      // TODO: sesuaikan wiring lama RS485 BMS (ESP32 RX <- modul RO)
#define BMS_TX_PIN          4       // TODO: sesuaikan wiring lama RS485 BMS (ESP32 TX -> modul DI)
#define BMS_DE_PIN          -1      // pin DE/RE (tied together); -1 kalau modul auto-direction

#define BMS_RESP_TIMEOUT_MS 300     // tunggu balasan BMS
#define BMS_FAIL_LIMIT      5       // gagal berturut-turut -> data dianggap tidak valid
#define BMS_CURRENT_INVERT  0       // set 1 kalau tanda arus terbalik (cek saat charge/discharge)
#define BMS_MAX_CELLS       24

// #define BMS_DEBUG                // aktifkan untuk log ke Serial

// ---------------- data ----------------
struct BmsData {
  bool     valid;                   // true kalau data terakhir masih segar
  uint32_t lastOkMs;                // millis() saat frame valid terakhir
  uint32_t okCount;
  uint32_t failCount;

  float    totalV;                  // V
  float    currentA;                // A, + = charge, - = discharge
  float    powerW;                  // W (V*I, tanda sama dengan arus)
  uint8_t  soc;                     // %
  uint16_t cycles;

  uint8_t  cellCount;
  uint16_t cellMv[BMS_MAX_CELLS];   // mV per cell
  uint16_t cellMinMv, cellMaxMv, cellDeltaMv, cellAvgMv;
  uint8_t  cellMinIdx, cellMaxIdx;  // 1-based

  float    tempMosC;                // power tube / MOSFET
  float    tempBat1C;               // sensor baterai 1
  float    tempBat2C;               // sensor baterai 2

  bool     chargeMosOn;
  bool     dischargeMosOn;
  bool     balancing;
  uint16_t warnBits;                // bitmask warning dari BMS (0x8B)
};

#ifdef BMS_DEBUG
#define BMS_LOG(...) Serial.printf(__VA_ARGS__)
#else
#define BMS_LOG(...) do {} while (0)
#endif

// ---------------- state internal (function-local static, aman kalau di-include >1 file) ----------------
struct BmsState {
  BmsData      data;
  portMUX_TYPE mux;
  uint8_t      failStreak;
};

inline BmsState& bmsState() {
  static BmsState s = { {}, portMUX_INITIALIZER_UNLOCKED, 0 };
  return s;
}

// ---------------- helper internal ----------------
inline uint16_t bmsBe16(const uint8_t* p) { return ((uint16_t)p[0] << 8) | p[1]; }

// Suhu JK: <=100 -> 0..100 C, >100 -> negatif (100 - raw)
inline float bmsDecodeTemp(uint16_t raw) {
  return (raw <= 100) ? (float)raw : -(float)(raw - 100);
}

// Panjang field (byte) per data ID, selain 0x79 (variabel). -1 = tidak dikenal.
inline int8_t bmsFieldLen(uint8_t id) {
  if (id >= 0x80 && id <= 0x84) return 2;
  if (id == 0x85 || id == 0x86) return 1;
  if (id == 0x87) return 2;
  if (id == 0x89) return 4;
  if (id >= 0x8A && id <= 0x8C) return 2;
  if (id >= 0x8E && id <= 0x9C) return 2;
  if (id == 0x9D) return 1;
  if (id >= 0x9E && id <= 0xA8) return 2;
  switch (id) {
    case 0xA9: return 1;
    case 0xAA: return 4;
    case 0xAB: case 0xAC: return 1;
    case 0xAD: return 2;
    case 0xAE: case 0xAF: return 1;
    case 0xB0: return 2;
    case 0xB1: return 1;
    case 0xB2: return 10;
    case 0xB3: return 1;
    case 0xB4: return 8;
    case 0xB5: case 0xB6: return 4;
    case 0xB7: return 15;
    case 0xB8: return 1;
    case 0xB9: return 4;
    case 0xBA: return 24;
    case 0xC0: return 1;
    default:   return -1;
  }
}

// Baca satu frame lengkap (4E 57 + length) dengan timeout.
inline bool bmsReadFrame(uint8_t* buf, size_t cap, size_t* outLen, uint32_t timeoutMs) {
  size_t n = 0, expect = 0;
  uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    while (BMS_SERIAL.available()) {
      uint8_t b = (uint8_t)BMS_SERIAL.read();
      if (n == 0) { if (b == 0x4E) buf[n++] = b; continue; }
      if (n == 1) {
        if (b == 0x57) { buf[n++] = b; }
        else if (b == 0x4E) { buf[0] = 0x4E; n = 1; }
        else { n = 0; }
        continue;
      }
      if (n >= cap) return false;
      buf[n++] = b;
      if (n == 4) {
        expect = (size_t)bmsBe16(buf + 2) + 2;       // length tidak termasuk 2 byte header
        if (expect < 21 || expect > cap) { n = 0; expect = 0; }
      }
      if (expect && n == expect) { *outLen = n; return true; }
    }
    vTaskDelay(1);
  }
  return false;
}

// Parse frame balasan -> d. Return true kalau minimal total voltage + cell terbaca.
inline bool bmsParseFrame(const uint8_t* f, size_t len, BmsData* d) {
  if (len < 21 || f[8] != 0x06 || f[len - 5] != 0x68) return false;

  // checksum: jumlah semua byte dari 4E sampai 0x68, dibanding 2 byte terakhir
  uint32_t sum = 0;
  for (size_t k = 0; k < len - 4; k++) sum += f[k];
  if ((uint16_t)(sum & 0xFFFF) != bmsBe16(f + len - 2)) return false;

  bool gotV = false;
  size_t i = 11;                // awal blok data
  const size_t end = len - 9;   // sisakan record number(4) + 0x68 + checksum(4)

  while (i < end) {
    uint8_t id = f[i++];

    if (id == 0x79) {           // tegangan cell: [len][idx, mV_hi, mV_lo]...
      if (i >= end) break;
      uint8_t L = f[i++];
      if (i + L > end) return false;
      for (uint8_t k = 0; k + 2 < L; k += 3) {
        uint8_t idx = f[i + k];
        uint16_t mv = bmsBe16(f + i + k + 1);
        if (idx >= 1 && idx <= BMS_MAX_CELLS) {
          d->cellMv[idx - 1] = mv;
          if (idx > d->cellCount) d->cellCount = idx;
        }
      }
      i += L;
      continue;
    }

    int8_t fl = bmsFieldLen(id);
    if (fl < 0 || i + fl > end) break;   // ID tidak dikenal: berhenti, data penting sudah di depan
    const uint8_t* p = f + i;

    switch (id) {
      case 0x80: d->tempMosC  = bmsDecodeTemp(bmsBe16(p)); break;
      case 0x81: d->tempBat1C = bmsDecodeTemp(bmsBe16(p)); break;
      case 0x82: d->tempBat2C = bmsDecodeTemp(bmsBe16(p)); break;
      case 0x83: d->totalV = bmsBe16(p) * 0.01f; gotV = true; break;
      case 0x84: {                       // bit15 = 1 -> charge, 0 -> discharge; satuan 0.01 A
        uint16_t raw = bmsBe16(p);
        float a = (raw & 0x7FFF) * 0.01f;
        d->currentA = (raw & 0x8000) ? a : -a;
#if BMS_CURRENT_INVERT
        d->currentA = -d->currentA;
#endif
        break;
      }
      case 0x85: d->soc = p[0]; break;
      case 0x87: d->cycles = bmsBe16(p); break;
      case 0x8B: d->warnBits = bmsBe16(p); break;
      case 0x8C: {
        uint16_t st = bmsBe16(p);
        d->chargeMosOn    = st & 0x0001;
        d->dischargeMosOn = st & 0x0002;
        d->balancing      = st & 0x0004;
        break;
      }
      default: break;                    // field dikenal tapi tidak dipakai: lewati
    }
    i += fl;
  }

  if (!gotV || d->cellCount == 0) return false;

  // statistik cell (abaikan cell 0 mV)
  uint32_t acc = 0; uint8_t n = 0;
  d->cellMinMv = 0xFFFF; d->cellMaxMv = 0;
  d->cellMinIdx = d->cellMaxIdx = 0;
  for (uint8_t k = 0; k < d->cellCount; k++) {
    uint16_t mv = d->cellMv[k];
    if (mv == 0) continue;
    acc += mv; n++;
    if (mv < d->cellMinMv) { d->cellMinMv = mv; d->cellMinIdx = k + 1; }
    if (mv > d->cellMaxMv) { d->cellMaxMv = mv; d->cellMaxIdx = k + 1; }
  }
  if (n == 0) return false;
  d->cellAvgMv   = acc / n;
  d->cellDeltaMv = d->cellMaxMv - d->cellMinMv;
  d->powerW      = d->totalV * d->currentA;
  return true;
}

inline void bmsRs485Tx(bool on) {
#if BMS_DE_PIN >= 0
  digitalWrite(BMS_DE_PIN, on ? HIGH : LOW);
#else
  (void)on;
#endif
}

// ---------------- API publik ----------------
inline void bmsInit() {
  BmsState& s = bmsState();
  memset(&s.data, 0, sizeof(s.data));
  s.failStreak = 0;
#if BMS_DE_PIN >= 0
  pinMode(BMS_DE_PIN, OUTPUT);
  digitalWrite(BMS_DE_PIN, LOW);
#endif
  BMS_SERIAL.setRxBufferSize(1024);      // harus sebelum begin()
  BMS_SERIAL.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  BMS_LOG("[BMS] init UART %d baud, RX=%d TX=%d\n", BMS_BAUD, BMS_RX_PIN, BMS_TX_PIN);
}

inline void bmsUpdate() {
  // Request "read all data" (cmd 0x06) ke BMS alamat 0.
  // 4E 57 | len=0x0013 | terminal=0 | cmd=06 | src=03 (PC) | type=00 | id=00 | record=0 | 68 | checksum
  static const uint8_t REQ_READ_ALL[21] = {
    0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 0x06, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x00, 0x01, 0x29
  };
  static uint8_t rx[512];

  BmsState& s = bmsState();
  size_t len = 0;
  BmsData d;
  memset(&d, 0, sizeof(d));

  // buang sisa data lama, kirim request
  while (BMS_SERIAL.available()) BMS_SERIAL.read();
  bmsRs485Tx(true);
  BMS_SERIAL.write(REQ_READ_ALL, sizeof(REQ_READ_ALL));
  BMS_SERIAL.flush();                    // tunggu TX selesai sebelum balik ke mode terima
  bmsRs485Tx(false);

  bool ok = bmsReadFrame(rx, sizeof(rx), &len, BMS_RESP_TIMEOUT_MS) && bmsParseFrame(rx, len, &d);

  portENTER_CRITICAL(&s.mux);
  if (ok) {
    s.failStreak = 0;
    d.valid     = true;
    d.lastOkMs  = millis();
    d.okCount   = s.data.okCount + 1;
    d.failCount = s.data.failCount;
    s.data = d;
  } else {
    s.data.failCount++;
    if (s.failStreak < 255) s.failStreak++;
    if (s.failStreak >= BMS_FAIL_LIMIT) s.data.valid = false;
  }
  portEXIT_CRITICAL(&s.mux);

#ifdef BMS_DEBUG
  if (ok) {
    BMS_LOG("[BMS] %.2fV %.2fA %uS SOC=%u%% cell %u..%u mV (d=%u) Tmos=%.0f MOS c=%d d=%d bal=%d\n",
            d.totalV, d.currentA, d.cellCount, d.soc, d.cellMinMv, d.cellMaxMv,
            d.cellDeltaMv, d.tempMosC, d.chargeMosOn, d.dischargeMosOn, d.balancing);
  } else {
    BMS_LOG("[BMS] gagal baca (rx=%u byte, streak=%u)\n", (unsigned)len, s.failStreak);
  }
#endif
}

// Salin snapshot terakhir (thread-safe). Return true kalau data valid/segar.
inline bool bmsGet(BmsData* out) {
  BmsState& s = bmsState();
  portENTER_CRITICAL(&s.mux);
  *out = s.data;
  portEXIT_CRITICAL(&s.mux);
  return out->valid;
}