#pragma once
/*
  ============================================================
  DATA COMPUTATION
  ============================================================

  DESKRIPSI
  Modul ini melakukan komputasi dan pengolahan data dari seluruh
  data hasil akuisisi, lalu mendistribusikannya ke masing-masing
  tujuan keluaran sesuai format yang dibutuhkan.

  SUMBER DATA (./include/)
  1. acq_bms.h     : data akuisisi BMS
  2. acq_gps.h     : data akuisisi GPS
  3. acq_motcon.h  : data akuisisi motor controller
  4. acq_rtc.h     : data akuisisi RTC

  TUJUAN DATA
  1. LoRa     : telemetri nirkabel      -> telem_e34.h  (elmer::Telemetry)
  2. SD Card  : pencatatan data         -> com_mircosd.h (CSV, 1 baris per sumber)
  3. Nextion  : tampilan layar (HMI)    -> comp_nextion.h (NextionData)

  CATATAN
  - Format data untuk setiap tujuan (LoRa, SD Card, Nextion)
    mengikuti format yang sudah didefinisikan pada masing-masing
    header file akuisisi di atas.
  - Modul ini tidak melakukan akuisisi data langsung; seluruh
    data mentah diambil dari header file akuisisi.

  HASIL KOMPUTASI (struct CompData)
  - speed (GPS, fallback dari RPM), rpm, tegangan, arus, daya
  - throttle %, brake %, SOC, suhu baterai/motor/controller/board
  - energi (Wh), muatan (Ah), jarak (km), Wh/km, waktu discharge
  - lap time (garis start/finish via GPS atau tombol manual)

  KONVENSI
  - Nilai tidak tersedia = NaN (CompData) / Opt kosong (Telemetry).
  - Semua arus di modul ini: POSITIF = DISCHARGE (BMS dibalik tandanya
    lewat COMP_BMS_I_SIGN karena BMS JK: + = charge).

  PEMAKAIAN (paling sederhana)
    #include "comp_data.h"
    void setup() { Serial.begin(115200); compStartTasks(); }
    void loop()  { vTaskDelay(portMAX_DELAY); }

  Kalau task sudah kamu buat sendiri, cukup:
    compInit();                       // sekali
    for (;;) { compUpdate(); vTaskDelay(pdMS_TO_TICKS(COMP_PERIOD_MS)); }
  lalu task LoRa/Nextion/SD memanggil telemUpdate() / compNextionUpdate() / sdUpdate().

  Opsi (define SEBELUM #include "comp_data.h"):
    COMP_PERIOD_MS, COMP_NEXTION_PERIOD_MS, COMP_LORA_PERIOD_MS, COMP_SD_PERIOD_MS,
    COMP_WHEEL_CIRC_M, COMP_GEAR_RATIO, COMP_RPM_SCALE, COMP_THROTTLE_V_MIN/MAX,
    COMP_BOOST_GEAR, COMP_BMS_I_SIGN, COMP_LAP_*, COMP_NO_HW_WARN
  ============================================================
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "acq_rtc.h"
#include "acq_gps.h"
#include "acq_bms.h"
#include "acq_motcon.h"
#include "comp_nextion.h"
#include "com_mircosd.h"
#include "telem_e34.h"

// ================= Konfigurasi =================
#ifndef COMP_PERIOD_MS
#define COMP_PERIOD_MS          50      // periode komputasi
#endif
#ifndef COMP_NEXTION_PERIOD_MS
#define COMP_NEXTION_PERIOD_MS  50     // refresh layar
#endif
#ifndef COMP_LORA_PERIOD_MS
#define COMP_LORA_PERIOD_MS     50    // kirim paket telemetri
#endif
#ifndef COMP_SD_PERIOD_MS
#define COMP_SD_PERIOD_MS       100     // 1 set baris log per periode ini
#endif
#ifndef COMP_MOTCON_PERIOD_MS
#define COMP_MOTCON_PERIOD_MS   10
#endif
#ifndef COMP_GPS_PERIOD_MS
#define COMP_GPS_PERIOD_MS      50
#endif
#ifndef COMP_RTC_PERIOD_MS
#define COMP_RTC_PERIOD_MS      10
#endif
#ifndef COMP_TASK_STACK
#define COMP_TASK_STACK         4096
#endif

// Kalibrasi kecepatan dari RPM (dipakai jika GPS tidak valid).
// KALIBRASI DULU: nilai default hanya placeholder.
#ifndef COMP_WHEEL_CIRC_M
#define COMP_WHEEL_CIRC_M       1.60f   // keliling roda (m)
#endif
#ifndef COMP_GEAR_RATIO
#define COMP_GEAR_RATIO         1.0f    // rpm motor : rpm roda (1.0 = rpm sudah rpm roda)
#endif
#ifndef COMP_RPM_SCALE
#define COMP_RPM_SCALE          1.0f    // pengali MC_SPEED_RAW -> rpm motor
#endif

// Throttle: tegangan sinyal throttle (V) -> 0..100 %
#ifndef COMP_THROTTLE_V_MIN
#define COMP_THROTTLE_V_MIN     0.80f
#endif
#ifndef COMP_THROTTLE_V_MAX
#define COMP_THROTTLE_V_MAX     4.20f
#endif

#ifndef COMP_BOOST_GEAR
#define COMP_BOOST_GEAR         3       // gear mentah yang dianggap boost; -1 = boost selalu false
#endif
#ifndef COMP_BMS_I_SIGN
#define COMP_BMS_I_SIGN         (-1.0f) // BMS: + = charge -> dibalik jadi + = discharge
#endif
#ifndef COMP_DISCHARGE_I_MIN
#define COMP_DISCHARGE_I_MIN    1.0f    // A, di atas ini dihitung "sedang discharge"
#endif
#ifndef COMP_GPS_MIN_SAT
#define COMP_GPS_MIN_SAT        4
#endif
#ifndef COMP_RTC_STALE_MS
#define COMP_RTC_STALE_MS       3000
#endif

// Lap time (garis start/finish)
#ifndef COMP_LAP_MIN_MS
#define COMP_LAP_MIN_MS         15000   // lap minimum, anti double-trigger
#endif
#ifndef COMP_LAP_RADIUS_M
#define COMP_LAP_RADIUS_M       20.0f   // radius gerbang default
#endif

// ---- Peringatan konfigurasi hardware (bisa dimatikan: #define COMP_NO_HW_WARN) ----
#ifndef COMP_NO_HW_WARN
#if (GPS_UART_NUM == 2) || (NEXTION_UART_NUM == 2) || (TELEM_UART_NUM == 2)
#warning "BMS (Serial2), GPS, Nextion, dan E34 memakai UART2 yang sama: hanya satu yang bisa aktif. Pindahkan ke UART/pin lain."
#endif
#if MOTCON_ENABLE_REQUESTS && (MOTCON_TX_PIN == NEXTION_TX_PIN)
#warning "Pin TX motcon dan TX Nextion sama (GPIO27). Set MOTCON_ENABLE_REQUESTS=0 atau ganti pin."
#endif
#endif

// ================= Tipe data =================
struct CompData {
  uint32_t ms;

  // status sumber
  bool rtcOk, gpsOk, bmsOk, mcOk;
  uint32_t mcAgeMs;          // umur data motcon terbaru (ms); 0xFFFFFFFF = belum ada

  // data mentah terakhir (untuk logging & telemetri)
  RtcData         rtc;
  GpsData         gps;
  BmsData         bms;
  MotconSnapshot  mc;

  // hasil komputasi
  float speedKmh;            // km/h
  uint8_t speedSrc;          // 0 = tidak ada, 1 = GPS, 2 = RPM
  float rpm;
  float voltageV;            // V pack (controller, fallback BMS)
  float currentA;            // A, + = discharge
  float bmsCurrentA;         // A, + = discharge (dari BMS)
  float powerW;              // W, + = konsumsi
  float throttlePct;
  float brakePct;
  float battPct;             // SOC
  float tempBattC, tempMotorC, tempCtrlC, tempBoardC;
  bool  boost;
  uint8_t  gear;             // 0xFF = tidak tersedia
  uint16_t faults;           // bitmask fault controller

  // integrasi sejak boot
  float    energyWh;
  float    chargeAh;
  float    distKm;
  float    whPerKm;
  uint32_t dischargeS;

  // lap
  uint8_t  lapState; 
  bool     lapRunning;
  uint16_t lapCount;         // jumlah lap selesai
  uint32_t lapCurMs;
  uint32_t lapLastMs;
  uint32_t lapBestMs;
  

  // data yang dikirim ke Nextion
  NextionData nx;
};

// ================= State internal =================
static SemaphoreHandle_t g_compMutex = nullptr;
static CompData          g_comp;

static float    g_cdEnergyWh = 0, g_cdChargeAh = 0, g_cdDistKm = 0;
static float    g_cdDischargeAcc = 0;      // detik (float agar dt kecil tidak hilang)

struct CdLap {
  bool     enabled;
  float    lat, lon, radius;
  bool     armed;
  bool     running;
  uint32_t startMs, lastCross, lastMs, bestMs;
  uint16_t count;
  volatile bool manual;
};
static CdLap g_cdLap = {false, 0, 0, COMP_LAP_RADIUS_M, true, false, 0, 0, 0, 0, 0, false};

// ================= Helper =================
static inline float cdMcVal(const MotconSnapshot& m, MotconField f) {
  return m.fresh[f] ? m.val[f] : NAN;
}

static inline float cdClamp(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static inline void cdSet(elmer::Opt<double>& o, float v) {
  if (isfinite(v)) o = (double)v;
}

// Epoch UNIX (detik) dari tanggal/jam (algoritma days-from-civil).
static uint32_t cdEpoch(uint16_t y, uint8_t m, uint8_t d, uint8_t hh, uint8_t mm, uint8_t ss) {
  long yy  = (long)y - (m <= 2 ? 1 : 0);
  long era = yy / 400;
  long yoe = yy - era * 400;
  long doy = (153L * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + doe - 719468L;
  return (uint32_t)(days * 86400L + hh * 3600L + mm * 60L + ss);
}

// Jarak datar (m) antar dua koordinat, cukup akurat untuk jarak pendek.
static float cdDistM(float lat1, float lon1, float lat2, float lon2) {
  float dy = (lat2 - lat1) * 111320.0f;
  float dx = (lon2 - lon1) * 111320.0f * cosf(lat1 * 0.01745329f);
  return sqrtf(dx * dx + dy * dy);
}

// ---- builder baris CSV (maks SD_LINE_MAX) ----
struct CdLine { char b[SD_LINE_MAX]; size_t n; };

static void cdAppend(CdLine& l, const char* fmt, ...) {
  if (l.n >= sizeof(l.b) - 1) return;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(l.b + l.n, sizeof(l.b) - l.n, fmt, ap);
  va_end(ap);
  if (w > 0) {
    l.n += (size_t)w;
    if (l.n > sizeof(l.b) - 1) l.n = sizeof(l.b) - 1;
  }
}

static void cdBegin(CdLine& l, char tag) {
  l.n = 0;
  l.b[0] = '\0';
  cdAppend(l, "%lu,%c", (unsigned long)millis(), tag);
}

// Tambah kolom angka; kosong jika tidak tersedia.
static void cdV(CdLine& l, bool ok, double v, int dec) {
  if (!ok || !isfinite(v)) cdAppend(l, ",");
  else                     cdAppend(l, ",%.*f", dec, v);
}

static void cdEnd(CdLine& l) { sdLog(l.b); }

// ================= Lap time =================
static void cdLapCross(uint32_t now) {
  if (!g_cdLap.running) {
    g_cdLap.running = true;
    g_cdLap.startMs = now;
  } else {
    g_cdLap.lastMs = now - g_cdLap.startMs;
    if (g_cdLap.bestMs == 0 || g_cdLap.lastMs < g_cdLap.bestMs) g_cdLap.bestMs = g_cdLap.lastMs;
    g_cdLap.count++;
    g_cdLap.startMs = now;
  }
  g_cdLap.lastCross = now;
}

static void cdLapUpdate(uint32_t now, bool gpsUsable, float lat, float lon) {
  bool minOk = !g_cdLap.running || (now - g_cdLap.lastCross) >= COMP_LAP_MIN_MS;

  if (g_cdLap.manual) {                       // tombol/trigger manual
    g_cdLap.manual = false;
    if (minOk) cdLapCross(now);
    return;
  }
  if (!g_cdLap.enabled || !gpsUsable) return;

  float d = cdDistM(g_cdLap.lat, g_cdLap.lon, lat, lon);
  if (d <= g_cdLap.radius) {
    if (g_cdLap.armed && minOk) { cdLapCross(now); g_cdLap.armed = false; }
  } else if (d > g_cdLap.radius * 1.5f) {
    g_cdLap.armed = true;                     // sudah keluar gerbang, siap trigger lagi
  }
}

// API lap (aman dipanggil dari task lain)
static void compSetStartLine(double lat, double lon, float radiusM = COMP_LAP_RADIUS_M) {
  g_cdLap.lat = (float)lat;
  g_cdLap.lon = (float)lon;
  g_cdLap.radius = radiusM;
  g_cdLap.armed = true;
  g_cdLap.enabled = true;
}
static void compMarkLap() { g_cdLap.manual = true; }
static void compResetLaps() {
  g_cdLap.running = false;
  g_cdLap.count = 0;
  g_cdLap.lastMs = g_cdLap.bestMs = 0;
  g_cdLap.lastCross = g_cdLap.startMs = 0;
  g_cdLap.armed = true;
}

// ================= Hasil -> tujuan =================
// CompData -> Nextion
static void compBuildNextion(const CompData& c, NextionData& n) {
  n.speed = isfinite(c.speedKmh) ? (int)lroundf(c.speedKmh) : 0;
  n.rpm   = isfinite(c.rpm)      ? (int)lroundf(c.rpm)      : 0;
  n.batt  = isfinite(c.battPct)  ? (int)lroundf(c.battPct)  : 0;
  n.vbat  = isfinite(c.voltageV) ? c.voltageV : 0.0f;
  n.cbat  = isfinite(c.currentA) ? c.currentA : 0.0f;
  n.tcon  = isfinite(c.tempCtrlC) ? (int)lroundf(c.tempCtrlC) : 0;
  n.tbat  = isfinite(c.tempBattC) ? (int)lroundf(c.tempBattC) : 0;
  n.boost = c.boost;
}

// CompData -> elmer::Telemetry (seq diisi oleh telemUpdate)
static void compBuildTelemetry(const CompData& c, elmer::Telemetry& t) {
  t = elmer::Telemetry();
  const MotconSnapshot& m = c.mc;
  const BmsData& b = c.bms;

  // Drive
  cdSet(t.speed_kmh,         c.speedKmh);
  cdSet(t.rpm,               c.rpm);
  cdSet(t.current_a,         c.currentA);
  cdSet(t.throttle_pct,      c.throttlePct);
  cdSet(t.brake_pct,         c.brakePct);
  cdSet(t.battery_pct,       c.battPct);
  cdSet(t.voltage_v,         c.voltageV);
  cdSet(t.temp_batt_c,       c.tempBattC);
  cdSet(t.temp_motor_c,      c.tempMotorC);
  cdSet(t.temp_controller_c, c.tempCtrlC);
  cdSet(t.temp_board_c,      c.tempBoardC);
  if (m.statusValid) {
    t.boost      = c.boost;
    t.fault_code = (uint64_t)c.faults;
  }
  t.lap_state = c.lapState;

  // BMS
  if (c.bmsOk) {
    float t1 = b.tempBat1C, t2 = b.tempBat2C;
    t.bms_voltage_v              = (double)b.totalV;
    t.bms_current_a              = (double)c.bmsCurrentA;
    t.bms_discharge_time_s       = (double)c.dischargeS;
    t.bms_has_alarm              = (b.warnBits != 0);
    t.bms_cell_voltage_max_v     = b.cellMaxMv / 1000.0;
    t.bms_cell_voltage_min_v     = b.cellMinMv / 1000.0;
    t.bms_cell_voltage_max_index = (uint64_t)b.cellMaxIdx;
    t.bms_cell_voltage_min_index = (uint64_t)b.cellMinIdx;
    t.bms_cell_temp_max_c        = (double)fmaxf(t1, t2);
    t.bms_cell_temp_min_c        = (double)fminf(t1, t2);
    t.bms_cell_temp_avg_c        = (double)((t1 + t2) * 0.5f);
    t.bms_cell_temp_max_index    = (uint64_t)(t1 >= t2 ? 1 : 2);   // indeks sensor
    t.bms_cell_temp_min_index    = (uint64_t)(t1 <= t2 ? 1 : 2);
    t.bms_frame_count            = (uint64_t)b.okCount;
    t.bms_updated_at_raw         = b.lastOkMs;                     // uptime ms
  }

  // Motor controller
  cdSet(t.controller_voltage_v, cdMcVal(m, MC_VOLTAGE));
  cdSet(t.controller_current_a, cdMcVal(m, MC_CURRENT));
  if (m.statusValid) {
    t.gear_status       = (uint8_t)m.gearRaw;
    t.controller_status = (uint8_t)(c.faults ? elmer::wire::kStatusFault : elmer::wire::kStatusOk);
  }
  t.controller_data_valid = m.statusValid;
  if (c.mcAgeMs != 0xFFFFFFFFu) t.controller_updated_at_raw = (uint32_t)(c.ms - c.mcAgeMs);

  // Device & GPS
  if (c.rtcOk) {
    t.device_updated_at_raw = cdEpoch(c.rtc.year, c.rtc.month, c.rtc.day,
                                      c.rtc.hour, c.rtc.minute, c.rtc.second);   // epoch UTC/zona RTC
  }
  if (c.gpsOk) {
    t.latitude_deg  = c.gps.lat;
    t.longitude_deg = c.gps.lon;
  }
}

// ================= Logging SD =================
// Satu baris per sumber, diawali "millis,TAG". Kolom kosong = tidak tersedia.
static void compSdHeader() {
  sdLog("# ELMER LOG BOOT - kolom per tag:");
  sdLog("# R: ms,R,datetime,rtc_temp_c,valid");
  sdLog("# B: ms,B,valid,V,A(+dis),W,soc,cyc,cmin_mV,cmax_mV,cdelta,cavg,imin,imax,tmos,tb1,tb2,chg,dis,bal,warn");
  sdLog("# G: ms,G,valid,lat,lon,alt_m,kmh,course,sat,hdop,utc");
  sdLog("# M: ms,M,V,A,spdraw,thr_v,tmot,tctl,pha,phc,brake,phbits,gear,faults,crc_err");
  sdLog("# C: ms,C,kmh,rpm,V,A,W,soc,Wh,Ah,km,dischg_s,boost,thr_pct,Wh_per_km");
  sdLog("# L: ms,L,laps,cur_ms,last_ms,best_ms");
  sdLog("# N: ms,N,speed,rpm,batt,vbat,cbat,tcon,tbat,boost");
}

static void cdLogSd(const CompData& c) {
  CdLine l;

  // R - RTC
  cdBegin(l, 'R');
  cdAppend(l, ",%04u-%02u-%02u %02u:%02u:%02u", (unsigned)c.rtc.year, (unsigned)c.rtc.month,
           (unsigned)c.rtc.day, (unsigned)c.rtc.hour, (unsigned)c.rtc.minute, (unsigned)c.rtc.second);
  cdV(l, true, c.rtc.temp_c, 2);
  cdV(l, true, c.rtcOk ? 1 : 0, 0);
  cdEnd(l);

  // B - BMS
  {
    const BmsData& b = c.bms; bool ok = c.bmsOk;
    cdBegin(l, 'B');
    cdV(l, true, ok ? 1 : 0, 0);
    cdV(l, ok, b.totalV, 2);
    cdV(l, ok, c.bmsCurrentA, 2);
    cdV(l, ok, b.powerW * COMP_BMS_I_SIGN, 0);
    cdV(l, ok, b.soc, 0);
    cdV(l, ok, b.cycles, 0);
    cdV(l, ok, b.cellMinMv, 0);
    cdV(l, ok, b.cellMaxMv, 0);
    cdV(l, ok, b.cellDeltaMv, 0);
    cdV(l, ok, b.cellAvgMv, 0);
    cdV(l, ok, b.cellMinIdx, 0);
    cdV(l, ok, b.cellMaxIdx, 0);
    cdV(l, ok, b.tempMosC, 1);
    cdV(l, ok, b.tempBat1C, 1);
    cdV(l, ok, b.tempBat2C, 1);
    cdV(l, ok, b.chargeMosOn ? 1 : 0, 0);
    cdV(l, ok, b.dischargeMosOn ? 1 : 0, 0);
    cdV(l, ok, b.balancing ? 1 : 0, 0);
    if (ok) cdAppend(l, ",%04X", (unsigned)b.warnBits); else cdAppend(l, ",");
    cdEnd(l);
  }

  // G - GPS
  {
    const GpsData& g = c.gps; bool ok = c.gpsOk;
    cdBegin(l, 'G');
    cdV(l, true, ok ? 1 : 0, 0);
    cdV(l, ok, g.lat, 7);
    cdV(l, ok, g.lon, 7);
    cdV(l, ok, g.alt_m, 1);
    cdV(l, ok, g.speed_kmph, 2);
    cdV(l, ok, g.course_deg, 1);
    cdV(l, true, g.sat, 0);
    cdV(l, true, g.hdop, 1);
    if (ok) cdAppend(l, ",%02u:%02u:%02u", (unsigned)g.hour, (unsigned)g.minute, (unsigned)g.second);
    else    cdAppend(l, ",");
    cdEnd(l);
  }

  // M - motor controller (kolom kosong jika stale)
  {
    const MotconSnapshot& m = c.mc; bool st = m.statusValid;
    cdBegin(l, 'M');
    cdV(l, true, cdMcVal(m, MC_VOLTAGE) , 2);
    cdV(l, true, cdMcVal(m, MC_CURRENT) , 2);
    cdV(l, true, cdMcVal(m, MC_SPEED_RAW), 0);
    cdV(l, true, cdMcVal(m, MC_THROTTLE), 2);
    cdV(l, true, cdMcVal(m, MC_MOTOR_TEMP), 1);
    cdV(l, true, cdMcVal(m, MC_CONTROLLER_TEMP), 1);
    cdV(l, true, cdMcVal(m, MC_PHASE_A), 1);
    cdV(l, true, cdMcVal(m, MC_PHASE_C), 1);
    cdV(l, st, m.brake, 0);
    cdV(l, st, m.phaseBits, 0);
    cdV(l, st, m.gearRaw, 0);
    if (st) cdAppend(l, ",%04X", (unsigned)m.faults); else cdAppend(l, ",");
    cdV(l, true, m.crcErrors, 0);
    cdEnd(l);
  }

  // C - hasil komputasi
  cdBegin(l, 'C');
  cdV(l, true, c.speedKmh, 1);
  cdV(l, true, c.rpm, 0);
  cdV(l, true, c.voltageV, 2);
  cdV(l, true, c.currentA, 2);
  cdV(l, true, c.powerW, 0);
  cdV(l, true, c.battPct, 0);
  cdV(l, true, c.energyWh, 1);
  cdV(l, true, c.chargeAh, 2);
  cdV(l, true, c.distKm, 3);
  cdV(l, true, c.dischargeS, 0);
  cdV(l, true, c.boost ? 1 : 0, 0);
  cdV(l, true, c.throttlePct, 1);
  cdV(l, true, c.whPerKm, 1);
  cdEnd(l);

  // L - lap time
  cdBegin(l, 'L');
  cdV(l, true, c.lapCount, 0);
  cdV(l, c.lapRunning, c.lapCurMs, 0);
  cdV(l, c.lapLastMs > 0, c.lapLastMs, 0);
  cdV(l, c.lapBestMs > 0, c.lapBestMs, 0);
  cdEnd(l);

  // N - data yang dikirim ke Nextion
  cdBegin(l, 'N');
  cdV(l, true, c.nx.speed, 0);
  cdV(l, true, c.nx.rpm, 0);
  cdV(l, true, c.nx.batt, 0);
  cdV(l, true, c.nx.vbat, 1);
  cdV(l, true, c.nx.cbat, 1);
  cdV(l, true, c.nx.tcon, 0);
  cdV(l, true, c.nx.tbat, 0);
  cdV(l, true, c.nx.boost ? 1 : 0, 0);
  cdEnd(l);
}

// ================= API utama =================
static void compInit() {
  if (g_compMutex == nullptr) g_compMutex = xSemaphoreCreateMutex();
  g_cdEnergyWh = g_cdChargeAh = g_cdDistKm = 0;
  g_cdDischargeAcc = 0;
  compResetLaps();
}

// Ambil satu snapshot hasil komputasi (thread-safe).
static bool compGet(CompData& out) {
  if (g_compMutex == nullptr) return false;
  if (xSemaphoreTake(g_compMutex, pdMS_TO_TICKS(20))) {
    out = g_comp;
    xSemaphoreGive(g_compMutex);
    return true;
  }
  return false;
}

// Titik start/finish = posisi GPS saat ini (mis. dipanggil dari tombol).
static bool compMarkStartHere(float radiusM = COMP_LAP_RADIUS_M) {
  CompData c;
  if (!compGet(c) || !c.gpsOk) return false;
  compSetStartLine(c.gps.lat, c.gps.lon, radiusM);
  return true;
}

// Hitung semua turunan, lalu distribusikan ke Nextion / LoRa / SD.
static void compUpdate() {
  static uint32_t lastT = 0, lastSd = 0;
  uint32_t now = millis();
  float dt = lastT ? (now - lastT) / 1000.0f : 0.0f;
  if (dt > 1.0f) dt = 1.0f;
  lastT = now;

  CompData c = {};
  c.ms = now;
  c.speedKmh = c.rpm = c.voltageV = c.currentA = c.bmsCurrentA = c.powerW = NAN;
  c.throttlePct = c.brakePct = c.battPct = NAN;
  c.tempBattC = c.tempMotorC = c.tempCtrlC = c.tempBoardC = c.whPerKm = NAN;
  c.gear = 0xFF;

  // ---- 1. Ambil data akuisisi ----
  RtcData r = {};
  c.rtcOk = rtcGet(r) && r.valid && (uint32_t)(now - r.lastReadMs) <= COMP_RTC_STALE_MS;
  c.rtc = r;

  GpsData g = {};
  bool gOk = false;
  if (gpsMutex) gOk = gps_get(g) && g.valid;
  c.gpsOk = gOk;
  c.gps = g;

  BmsData b = {};
  c.bmsOk = bmsGet(&b);
  c.bms = b;

  motconSnapshot(c.mc);
  const MotconSnapshot& m = c.mc;
  c.mcAgeMs = 0xFFFFFFFFu;
  bool mcAny = false;
  for (uint8_t i = 0; i < MC_FIELD_COUNT; ++i) {
    if (m.fresh[i]) {
      mcAny = true;
      if (m.ageMs[i] < c.mcAgeMs) c.mcAgeMs = m.ageMs[i];
    }
  }
  c.mcOk = mcAny || m.statusValid;

  // ---- 2. Tegangan, arus, daya ----
  float vMc = cdMcVal(m, MC_VOLTAGE);
  float iMc = cdMcVal(m, MC_CURRENT);
  float vBms = c.bmsOk ? b.totalV : NAN;
  c.bmsCurrentA = c.bmsOk ? b.currentA * COMP_BMS_I_SIGN : NAN;
  float iBms = c.bmsCurrentA;

  c.voltageV = isfinite(vMc) ? vMc : vBms;
  c.currentA = isfinite(iMc) ? iMc : iBms;
  if (isfinite(c.voltageV) && isfinite(c.currentA)) c.powerW = c.voltageV * c.currentA;

  // ---- 3. RPM & kecepatan ----
  float raw = cdMcVal(m, MC_SPEED_RAW);
  if (isfinite(raw)) c.rpm = raw * COMP_RPM_SCALE;

  if (c.gpsOk && g.sat >= COMP_GPS_MIN_SAT) {
    c.speedKmh = (float)g.speed_kmph;
    c.speedSrc = 1;
  } else if (isfinite(c.rpm)) {
    float wheelRpm = c.rpm / COMP_GEAR_RATIO;
    c.speedKmh = wheelRpm * COMP_WHEEL_CIRC_M * 60.0f / 1000.0f;
    c.speedSrc = 2;
  }

  // ---- 4. Throttle, brake, gear, fault, boost ----
  float thrV = cdMcVal(m, MC_THROTTLE);
  if (isfinite(thrV)) {
    c.throttlePct = cdClamp((thrV - COMP_THROTTLE_V_MIN) /
                            (COMP_THROTTLE_V_MAX - COMP_THROTTLE_V_MIN) * 100.0f, 0.0f, 100.0f);
  }
  if (m.statusValid) {
    c.brakePct = m.brake ? 100.0f : 0.0f;
    c.gear     = m.gearRaw;
    c.faults   = m.faults;
    c.boost    = (COMP_BOOST_GEAR >= 0) && (m.gearRaw == (uint8_t)COMP_BOOST_GEAR);
  }

  // ---- 5. SOC & suhu ----
  if (c.bmsOk) {
    c.battPct   = b.soc;
    c.tempBattC = fmaxf(b.tempBat1C, b.tempBat2C);
  }
  c.tempMotorC = cdMcVal(m, MC_MOTOR_TEMP);
  c.tempCtrlC  = cdMcVal(m, MC_CONTROLLER_TEMP);
  if (c.rtcOk) c.tempBoardC = c.rtc.temp_c;      // sensor DS3231 di board

  // ---- 6. Integrasi (energi, muatan, jarak, waktu discharge) ----
  if (dt > 0.0f) {
    if (isfinite(c.powerW))  g_cdEnergyWh += c.powerW * dt / 3600.0f;
    if (isfinite(c.currentA)) {
      g_cdChargeAh += c.currentA * dt / 3600.0f;
      if (c.currentA > COMP_DISCHARGE_I_MIN) g_cdDischargeAcc += dt;
    }
    if (isfinite(c.speedKmh) && c.speedKmh > 0.0f) g_cdDistKm += c.speedKmh * dt / 3600.0f;
  }
  c.energyWh   = g_cdEnergyWh;
  c.chargeAh   = g_cdChargeAh;
  c.distKm     = g_cdDistKm;
  c.dischargeS = (uint32_t)g_cdDischargeAcc;
  if (g_cdDistKm > 0.05f) c.whPerKm = g_cdEnergyWh / g_cdDistKm;

  // ---- 7. Lap time ----
  cdLapUpdate(now, c.gpsOk && g.sat >= COMP_GPS_MIN_SAT, (float)g.lat, (float)g.lon);
  c.lapRunning = g_cdLap.running;
  c.lapCount   = g_cdLap.count;
  c.lapLastMs  = g_cdLap.lastMs;
  c.lapBestMs  = g_cdLap.bestMs;
  c.lapCurMs   = g_cdLap.running ? (now - g_cdLap.startMs) : 0;
  c.lapState   = !g_cdLap.enabled ? elmer::wire::kLapIdle
               : (g_cdLap.running ? elmer::wire::kLapRunning : elmer::wire::kLapArmed);

  // ---- 8. Distribusi ----
  compBuildNextion(c, c.nx);
  nextionSetData(c.nx);

  elmer::Telemetry t;
  compBuildTelemetry(c, t);
  telemSetData(t);

  if ((uint32_t)(now - lastSd) >= COMP_SD_PERIOD_MS) {
    lastSd = now;
    cdLogSd(c);
  }

  if (g_compMutex && xSemaphoreTake(g_compMutex, pdMS_TO_TICKS(10))) {
    g_comp = c;
    xSemaphoreGive(g_compMutex);
  }
}

// Dipanggil dari task Nextion: ambil data terakhir lalu tampilkan.
static uint8_t compNextionUpdate() {
  NextionData d;
  if (!nextionGetData(d)) return 0;
  return nextionUpdate(d);
}

// ================= Task RTOS (opsional) =================
static void cdTaskMotcon(void*) {
  motconInit();
  for (;;) { motconUpdate(); vTaskDelay(pdMS_TO_TICKS(COMP_MOTCON_PERIOD_MS)); }
}

static void cdTaskBms(void*) {
  bmsInit();
  for (;;) { bmsUpdate(); vTaskDelay(pdMS_TO_TICKS(BMS_PERIOD_MS)); }
}

static void cdTaskGps(void*) {
  gpsInit();
  for (;;) { gpsUpdate(); vTaskDelay(pdMS_TO_TICKS(COMP_GPS_PERIOD_MS)); }
}

static void cdTaskRtc(void*) {
  rtcInit();
  for (;;) {
    if (!g_rtcPresent) { rtcInit(); vTaskDelay(pdMS_TO_TICKS(5000)); continue; }
    rtcUpdate();
    vTaskDelay(pdMS_TO_TICKS(COMP_RTC_PERIOD_MS));
  }
}

static void cdTaskNextion(void*) {
  nextionInit();
  for (;;) { compNextionUpdate(); vTaskDelay(pdMS_TO_TICKS(COMP_NEXTION_PERIOD_MS)); }
}

static void cdTaskLora(void*) {
  telemInit();
  for (;;) { telemUpdate(); vTaskDelay(pdMS_TO_TICKS(COMP_LORA_PERIOD_MS)); }
}

static void cdTaskSd(void*) {
  sdInit();
  compSdHeader();
  for (;;) { sdUpdate(); vTaskDelay(pdMS_TO_TICKS(20)); }
}

static void cdTaskComp(void*) {
  vTaskDelay(pdMS_TO_TICKS(500));   // beri waktu modul lain init
  for (;;) { compUpdate(); vTaskDelay(pdMS_TO_TICKS(COMP_PERIOD_MS)); }
}

// Init semua modul dan jalankan semua task. Panggil sekali dari setup().
static void compStartTasks() {
  compInit();
  xTaskCreate(cdTaskMotcon,  "motcon",  COMP_TASK_STACK, nullptr, 3, nullptr);
  xTaskCreate(cdTaskBms,     "bms",     COMP_TASK_STACK, nullptr, 2, nullptr);
  xTaskCreate(cdTaskGps,     "gps",     COMP_TASK_STACK, nullptr, 2, nullptr);
  xTaskCreate(cdTaskRtc,     "rtc",     COMP_TASK_STACK, nullptr, 1, nullptr);
  xTaskCreate(cdTaskComp,    "comp",    COMP_TASK_STACK, nullptr, 2, nullptr);
  xTaskCreate(cdTaskNextion, "nextion", COMP_TASK_STACK, nullptr, 1, nullptr);
  xTaskCreate(cdTaskLora,    "lora",    COMP_TASK_STACK, nullptr, 1, nullptr);
  xTaskCreate(cdTaskSd,      "sd",      COMP_TASK_STACK, nullptr, 1, nullptr);
}