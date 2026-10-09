#pragma once
/*
  ============================================================
  TELEMETRY E34 (EBYTE wireless module) - PAKET BINER
  ============================================================

  INTERFACE : UART2
  PIN
    RX       : 16   (UART2 default RX)
    TX       : 17   (UART2 default TX)
  BAUDRATE : 115200
  FORMAT   : 8N1
  TODO     : pin M0 / M1 / AUX (mode config) belum didefinisikan

  SUMBER DATA : struct elmer::Telemetry (telemetry.h)

  PAKET TELEMETRI (uplink, little-endian, 101 byte):
    off  tipe   field                        skala     N/A
    0    u8[2]  sync 0xAA 0x55
    2    u32    seq (diisi modul ini)
    6    u32    ms (uptime ESP32, millis())
    10   u16    speed_kmh                    x100      0xFFFF
    12   u16    rpm                          x1        0xFFFF
    14   i16    current_a                    x10       0x8000
    16   u16    throttle_pct                 x100      0xFFFF
    18   u16    brake_pct                    x100      0xFFFF
    20   u16    battery_pct                  x100      0xFFFF
    22   u16    voltage_v                    x10       0xFFFF
    24   i16    temp_batt_c                  x100      0x8000
    26   i16    temp_motor_c                 x100      0x8000
    28   i16    temp_controller_c            x100      0x8000
    30   i16    temp_board_c                 x100      0x8000
    32   u32    fault_code                   x1        0xFFFFFFFF
    36   u16    bms_voltage_v                x10       0xFFFF
    38   i16    bms_current_a                x10       0x8000
    40   u32    bms_discharge_time_s         x1        0xFFFFFFFF
    44   u16    bms_cell_voltage_max_v       x1000     0xFFFF
    46   u16    bms_cell_voltage_min_v       x1000     0xFFFF
    48   u16    bms_cell_voltage_max_index   x1        0xFFFF
    50   u16    bms_cell_voltage_min_index   x1        0xFFFF
    52   i16    bms_cell_temp_max_c          x100      0x8000
    54   i16    bms_cell_temp_min_c          x100      0x8000
    56   i16    bms_cell_temp_avg_c          x100      0x8000
    58   u16    bms_cell_temp_max_index      x1        0xFFFF
    60   u16    bms_cell_temp_min_index      x1        0xFFFF
    62   u32    bms_frame_count              x1        0xFFFFFFFF
    66   u32    bms_updated_at_raw           x1        0xFFFFFFFF
    70   u16    controller_voltage_v         x10       0xFFFF
    72   i16    controller_current_a         x10       0x8000
    74   i16    temp_external_c              x100      0x8000
    76   f32    temp_coeff                                NaN
    80   u32    controller_updated_at_raw    x1        0xFFFFFFFF
    84   u32    device_updated_at_raw        x1        0xFFFFFFFF
    88   i32    latitude_deg                 x1e7      0x80000000
    92   i32    longitude_deg                x1e7      0x80000000
    96   u8     gear_status (kode)                     0xFF
    97   u8     controller_status (kode)               0xFF
    98   u8     bools: bit0..3 = valid, bit4..7 = nilai
                  bit0/4 boost, bit1/5 controller_enabled,
                  bit2/6 bms_has_alarm, bit3/7 controller_data_valid
    99   u8     lap_state (0=IDLE,1=ARMED,2=RUNNING)      0xFF
    100  u16    crc CRC-16/CCITT-FALSE atas offset 2..99

  Python: "<2sIIHHhHHHHhhhhIHhIHHHHhhhHHIIHhhfIIiiBBBBH"

  ATURAN KUANTISASI
    - Opt kosong atau NaN/inf -> N/A (sesuai tabel)
    - Nilai di luar rentang tipe di-clamp (u16 maks 0xFFFE,
      i16 +-32767, u32 maks 0xFFFFFFFE, i32 +-2147483647),
      bukan dijadikan N/A
    - gear_status > 253 -> 0xFE
    - controller_status: 0=OK, 1=WARN, 2=FAULT, lainnya -> 0xFE
    - bool: bit valid = 1 jika Opt terisi, bit nilai = isinya

  CATATAN
    - Layout mengikuti program referensi (dummy sender). Jika
      layout berubah, ubah struct TelemPacket dan telemPack().
    - static_assert menjaga ukuran paket (101) dan offset CRC (99).
    - taskTelemetry tidak diubah: tetap memakai telemSetData() dan
      telemUpdate() tanpa parameter.
  ============================================================
*/

#include <Arduino.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "telemetry.h"   // berisi elmer::Telemetry & elmer::wire

// ---------------- Konfigurasi ----------------
#define TELEM_RX_PIN      16
#define TELEM_TX_PIN      17
#define TELEM_BAUD        115200
#define TELEM_UART_NUM    2
#define TELEM_AUX_WAIT_MS 50

// TODO: isi nomor GPIO setelah wiring ditentukan. -1 = tidak dipakai.
// Kalau M0/M1 diisi, init set mode normal (M0 = LOW, M1 = LOW).
// Kalau AUX diisi, kirim hanya dilakukan saat AUX HIGH (modul siap).
#define TELEM_M0_PIN      -1
#define TELEM_M1_PIN      -1
#define TELEM_AUX_PIN     -1

// ---------------- Sentinel N/A ----------------
static constexpr uint16_t kTelemNaU16 = 0xFFFF;
static constexpr int16_t  kTelemNaI16 = INT16_MIN;
static constexpr uint32_t kTelemNaU32 = 0xFFFFFFFFu;
static constexpr int32_t  kTelemNaI32 = INT32_MIN;
static constexpr uint8_t  kTelemNaU8  = 0xFF;

// ---------------- Paket ----------------
#pragma pack(push, 1)
struct TelemPacket {
  uint8_t  sync[2];
  uint32_t seq;
  uint32_t ms;
  uint16_t speedKmh;
  uint16_t rpm;
  int16_t  currentA;
  uint16_t throttlePct;
  uint16_t brakePct;
  uint16_t batteryPct;
  uint16_t voltageV;
  int16_t  tempBattC;
  int16_t  tempMotorC;
  int16_t  tempControllerC;
  int16_t  tempBoardC;
  uint32_t faultCode;
  uint16_t bmsVoltageV;
  int16_t  bmsCurrentA;
  uint32_t bmsDischargeTimeS;
  uint16_t bmsCellVMax;
  uint16_t bmsCellVMin;
  uint16_t bmsCellVMaxIdx;
  uint16_t bmsCellVMinIdx;
  int16_t  bmsCellTMaxC;
  int16_t  bmsCellTMinC;
  int16_t  bmsCellTAvgC;
  uint16_t bmsCellTMaxIdx;
  uint16_t bmsCellTMinIdx;
  uint32_t bmsFrameCount;
  uint32_t bmsUpdatedAt;
  uint16_t ctrlVoltageV;
  int16_t  ctrlCurrentA;
  int16_t  tempExternalC;
  float    tempCoeff;
  uint32_t ctrlUpdatedAt;
  uint32_t devUpdatedAt;
  int32_t  lat;
  int32_t  lon;
  uint8_t  gear;
  uint8_t  ctrlStatus;
  uint8_t  bools;      // bit0..3 = valid, bit4..7 = nilai
  uint8_t  lapState;   // 0=IDLE, 1=ARMED, 2=RUNNING; 0xFF = N/A
  uint16_t crc;        // CRC-16/CCITT-FALSE atas offset 2..99
};
#pragma pack(pop)

static_assert(sizeof(TelemPacket) == elmer::wire::kPacketSize, "ukuran paket berubah");
static_assert(offsetof(TelemPacket, crc) == elmer::wire::kCrcOffset, "offset CRC berubah");

// ---------------- Data ----------------
using TelemData = elmer::Telemetry;

struct TelemStatus {
  uint32_t seq;        // jumlah paket terkirim
  uint32_t failed;     // gagal kirim (AUX timeout / buffer TX penuh)
  uint32_t lastTxMs;
  bool     ready;
};

static HardwareSerial    g_telem(TELEM_UART_NUM);
static TelemData         g_telemData;
static TelemStatus       g_telemSt   = {0, 0, 0, false};
static SemaphoreHandle_t g_telemMutex = nullptr;

// ---------------- Helper kuantisasi ----------------
typedef elmer::Opt<double> TelemOptD;

static uint16_t telemQU16(const TelemOptD& v, double scale) {
  if (!v || !isfinite(*v)) return kTelemNaU16;
  double x = *v * scale;
  if (x < 0.0) x = 0.0;
  if (x > 65534.0) x = 65534.0;   // 65535 dicadangkan untuk N/A
  return (uint16_t)lround(x);
}

static int16_t telemQI16(const TelemOptD& v, double scale) {
  if (!v || !isfinite(*v)) return kTelemNaI16;
  double x = *v * scale;
  if (x < -32767.0) x = -32767.0;
  if (x > 32767.0) x = 32767.0;
  return (int16_t)lround(x);
}

static uint32_t telemQU32(const TelemOptD& v, double scale) {
  if (!v || !isfinite(*v)) return kTelemNaU32;
  double x = *v * scale;
  if (x < 0.0) x = 0.0;
  if (x > 4294967294.0) x = 4294967294.0;
  return (uint32_t)llround(x);
}

static int32_t telemQI32(const TelemOptD& v, double scale) {
  if (!v || !isfinite(*v)) return kTelemNaI32;
  double x = *v * scale;
  if (x < -2147483647.0) x = -2147483647.0;
  if (x > 2147483647.0) x = 2147483647.0;
  return (int32_t)llround(x);
}

static uint32_t telemCU32(const elmer::Opt<uint64_t>& v) {
  if (!v) return kTelemNaU32;
  return (*v > 0xFFFFFFFEull) ? 0xFFFFFFFEu : (uint32_t)*v;
}

static uint16_t telemCU16(const elmer::Opt<uint64_t>& v) {
  if (!v) return kTelemNaU16;
  return (*v > 0xFFFEull) ? 0xFFFEu : (uint16_t)*v;
}

static uint32_t telemRawU32(const elmer::Opt<uint32_t>& v) {
  return v.value_or(kTelemNaU32);
}

// Kode angka -> u8; di atas 253 menjadi 0xFE
static uint8_t telemGearCode(const elmer::Opt<uint8_t>& g) {
  if (!g) return kTelemNaU8;
  return (*g > 253) ? elmer::wire::kOther : *g;
}

static uint8_t telemLapCode(const elmer::Opt<uint8_t>& s) {
  if (!s) return kTelemNaU8;
  return (*s <= elmer::wire::kLapRunning) ? *s : elmer::wire::kOther;
}

static uint8_t telemStatusCode(const elmer::Opt<uint8_t>& s) {
  if (!s) return kTelemNaU8;
  return (*s <= elmer::wire::kStatusFault) ? *s : elmer::wire::kOther;
}

static uint8_t telemPackBools(const elmer::Opt<bool>& b0, const elmer::Opt<bool>& b1,
                              const elmer::Opt<bool>& b2, const elmer::Opt<bool>& b3) {
  const elmer::Opt<bool>* arr[4] = {&b0, &b1, &b2, &b3};
  uint8_t out = 0;
  for (int i = 0; i < 4; ++i) {
    if (arr[i]->has_value()) {
      out |= (uint8_t)(1u << i);
      if (**arr[i]) out |= (uint8_t)(1u << (i + 4));
    }
  }
  return out;
}

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF
static uint16_t telemCrc16(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ---------------- Telemetry -> paket ----------------
static void telemPack(const TelemData& t, uint32_t now, TelemPacket& p) {
  p.sync[0] = elmer::wire::kSync0;
  p.sync[1] = elmer::wire::kSync1;
  p.seq = t.seq;
  p.ms = now;

  p.speedKmh        = telemQU16(t.speed_kmh, 100.0);
  p.rpm             = telemQU16(t.rpm, 1.0);
  p.currentA        = telemQI16(t.current_a, 10.0);
  p.throttlePct     = telemQU16(t.throttle_pct, 100.0);
  p.brakePct        = telemQU16(t.brake_pct, 100.0);
  p.batteryPct      = telemQU16(t.battery_pct, 100.0);
  p.voltageV        = telemQU16(t.voltage_v, 10.0);
  p.tempBattC       = telemQI16(t.temp_batt_c, 100.0);
  p.tempMotorC      = telemQI16(t.temp_motor_c, 100.0);
  p.tempControllerC = telemQI16(t.temp_controller_c, 100.0);
  p.tempBoardC      = telemQI16(t.temp_board_c, 100.0);
  p.faultCode       = telemCU32(t.fault_code);

  p.bmsVoltageV       = telemQU16(t.bms_voltage_v, 10.0);
  p.bmsCurrentA       = telemQI16(t.bms_current_a, 10.0);
  p.bmsDischargeTimeS = telemQU32(t.bms_discharge_time_s, 1.0);
  p.bmsCellVMax       = telemQU16(t.bms_cell_voltage_max_v, 1000.0);
  p.bmsCellVMin       = telemQU16(t.bms_cell_voltage_min_v, 1000.0);
  p.bmsCellVMaxIdx    = telemCU16(t.bms_cell_voltage_max_index);
  p.bmsCellVMinIdx    = telemCU16(t.bms_cell_voltage_min_index);
  p.bmsCellTMaxC      = telemQI16(t.bms_cell_temp_max_c, 100.0);
  p.bmsCellTMinC      = telemQI16(t.bms_cell_temp_min_c, 100.0);
  p.bmsCellTAvgC      = telemQI16(t.bms_cell_temp_avg_c, 100.0);
  p.bmsCellTMaxIdx    = telemCU16(t.bms_cell_temp_max_index);
  p.bmsCellTMinIdx    = telemCU16(t.bms_cell_temp_min_index);
  p.bmsFrameCount     = telemCU32(t.bms_frame_count);
  p.bmsUpdatedAt      = telemRawU32(t.bms_updated_at_raw);

  p.ctrlVoltageV  = telemQU16(t.controller_voltage_v, 10.0);
  p.ctrlCurrentA  = telemQI16(t.controller_current_a, 10.0);
  p.tempExternalC = telemQI16(t.temp_external_c, 100.0);
  p.tempCoeff     = (t.temp_coeff && isfinite(*t.temp_coeff)) ? (float)*t.temp_coeff : NAN;
  p.ctrlUpdatedAt = telemRawU32(t.controller_updated_at_raw);

  p.devUpdatedAt = telemRawU32(t.device_updated_at_raw);
  p.lat = telemQI32(t.latitude_deg, 1e7);
  p.lon = telemQI32(t.longitude_deg, 1e7);

  p.gear       = telemGearCode(t.gear_status);
  p.ctrlStatus = telemStatusCode(t.controller_status);
  p.bools      = telemPackBools(t.boost, t.controller_enabled,
                                t.bms_has_alarm, t.controller_data_valid);
  p.lapState   = telemLapCode(t.lap_state);

  const uint8_t* base = reinterpret_cast<const uint8_t*>(&p);
  p.crc = telemCrc16(base + offsetof(TelemPacket, seq),
                     offsetof(TelemPacket, crc) - offsetof(TelemPacket, seq));
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

  Serial.println("[TELEM] E34 siap @ 115200 (paket biner 101 byte)");
  return true;
}

// ---------------- UPDATE ----------------
// Kirim data dari struct. seq diisi modul ini, ms = millis().
// Return true jika paket terkirim.
static bool telemUpdate(const TelemData& d) {
  if (!g_telemSt.ready) return false;

  TelemData t = d;
  uint32_t seq = 0;
  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    seq = g_telemSt.seq;
    xSemaphoreGive(g_telemMutex);
  }
  t.seq = seq;

  TelemPacket p;
  telemPack(t, millis(), p);

  bool ok = telemWaitAux() && (g_telem.availableForWrite() >= (int)sizeof(p));
  if (ok) g_telem.write(reinterpret_cast<const uint8_t*>(&p), sizeof(p));

  if (xSemaphoreTake(g_telemMutex, pdMS_TO_TICKS(20))) {
    if (ok) { g_telemSt.seq++; g_telemSt.lastTxMs = millis(); }
    else    { g_telemSt.failed++; }
    xSemaphoreGive(g_telemMutex);
  }
  return ok;
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