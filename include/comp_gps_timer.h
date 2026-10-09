#pragma once
/*
  ============================================================
  LAP TIMING (GPS) - GOKART
  Referensi: FLOWCHART 2 (revisi-2)
  ============================================================

  DESKRIPSI
  Modul penghitung waktu lap berbasis GPS. Dipanggil setiap ada
  sampel GPS baru dari loop utama (bukan dipanggil terus-menerus).
  Hasil lap dikirim pada siklus data Halaman 1 (LoRa, SD, Nextion).

  STATE
  - IDLE    : belum ada perintah START, semua sampel diabaikan
  - ARMED   : START LAP diterima, menunggu lintasan pertama
              di garis finish/start
  - RUNNING : lap sedang berjalan, tiap lintasan dihitung

  PERINTAH LoRa (diproses di Halaman 1)
  - START LAP -> STATE = ARMED      (gpsTimerStart())
  - STOP LAP  -> STATE = IDLE       (gpsTimerStop())

  ALUR PER SAMPEL GPS (urutan pengecekan)
  1. GPS fix valid?          (satelit, HDOP)       -> tidak: selesai
  2. STATE = IDLE?                                  -> ya   : selesai
  3. Melewati garis finish/start?                   -> tidak: selesai
     (segmen posisi lama -> baru memotong garis)
  4. Lockout aktif?                                 -> ya   : selesai
     (selang sejak lintasan terakhir < waktu lap minimum)
  5. Hitung t_cross dengan interpolasi
     (dari timestamp GPS, bukan millis)
  6. STATE = ARMED? (lap pertama)
       ya    : t_start = t_cross, STATE = RUNNING, lap_no = 1
       tidak : lap_time = t_cross - t_start
               simpan lap_no & lap_time, update best lap
               t_start = t_cross, lap_no + 1
  7. Set lockout (t_cross) & tandai hasil lap baru
     -> dikirim di siklus data Halaman 1 (LoRa, SD, Nextion)

  OUTPUT
  - Selesai tanpa aksi        : kembali ke loop utama
  - Selesai + hasil lap baru  : kembali ke loop utama,
                                flag hasil lap baru aktif

  PARAMETER YANG PERLU DIKONFIGURASI
  - Dua titik koordinat garis finish/start   -> gpsTimerSetGate()
  - Waktu lap minimum (lockout)              -> GPSTIMER_MIN_LAP_MS
  - Batas HDOP dan jumlah satelit minimum    -> GPSTIMER_HDOP_MAX, GPSTIMER_MIN_SAT

  CATATAN PENTING
  - Waktu lintasan memakai timestamp GPS hasil interpolasi,
    bukan millis(), agar akurasi tidak bergantung pada laju
    sampel GPS.
  - Lap pertama (ARMED -> RUNNING) hanya menyetel t_start,
    belum menghasilkan lap_time.
  - Lockout mencegah deteksi ganda akibat noise/jitter posisi
    di sekitar garis.
  - Waktu lap minimum harus lebih kecil dari lap tercepat yang
    realistis di lintasan.

  PEMAKAIAN
    #include "comp_gps_timer.h"

    void setup() {
      gpsTimerInit();
      gpsTimerSetGate(lat1, lon1, lat2, lon2);   // garis finish/start
    }

    // Tiap ada sampel GPS baru valid (mis. dari gpsUpdate() di acq_gps.h):
    GpsData g;
    if (gps_get(g)) gpsTimerUpdate(g);

    // Perintah dari LoRa / tombol:
    gpsTimerStart();   // START LAP
    gpsTimerStop();    // STOP LAP

    // Baca hasil (siklus Halaman 1 - LoRa/SD/Nextion):
    GpsTimerSnapshot s;
    if (gpsTimerGet(s)) {
      // s.lapNo, s.lapCurMs, s.lastLapMs, s.bestLapMs, s.newResult
      if (s.newResult) gpsTimerConsumeNewLap();
    }
  ============================================================
*/

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "acq_gps.h"

// ---------------- Konfigurasi (define SEBELUM #include kalau mau override) ----------------
#ifndef GPSTIMER_MIN_LAP_MS
#define GPSTIMER_MIN_LAP_MS   15000   // lockout: lap minimum realistis (ms)
#endif
#ifndef GPSTIMER_HDOP_MAX
#define GPSTIMER_HDOP_MAX     5.0     // sampel dgn HDOP di atas ini diabaikan
#endif
#ifndef GPSTIMER_MIN_SAT
#define GPSTIMER_MIN_SAT      4       // minimum satelit supaya sampel dipakai
#endif

// ---------------- Tipe ----------------
enum GpsTimerState : uint8_t {
  GT_IDLE = 0,
  GT_ARMED,
  GT_RUNNING
};

// Snapshot hasil, aman dibaca dari task lain (mis. task Halaman 1 / LoRa / SD).
struct GpsTimerSnapshot {
  GpsTimerState state;
  bool     gateSet;
  uint16_t lapNo;         // jumlah lap selesai (lap pertama bernilai 1)
  uint32_t lapCurMs;      // durasi lap yang sedang berjalan (perkiraan, berbasis millis)
  uint32_t lastLapMs;     // waktu lap terakhir yang selesai (ms)
  uint32_t bestLapMs;     // lap tercepat sejauh ini (ms); 0 = belum ada
  bool     newResult;     // true setelah ada lap baru, sampai di-consume
};

// ---------------- State internal ----------------
struct GtGate {
  bool   set;
  double lat1, lon1;
  double lat2, lon2;
};

struct GtInternal {
  GpsTimerState state;
  GtGate   gate;

  bool     prevValid;      // ada posisi sampel sebelumnya utk dibandingkan
  double   prevLat, prevLon;
  uint64_t prevEpochMs;    // timestamp GPS (epoch, ms) sampel sebelumnya

  uint64_t lastCrossEpochMs;  // 0 = belum pernah ada lintasan (lockout nonaktif)
  uint64_t lapStartEpochMs;   // t_start lap berjalan (epoch GPS, ms)
  uint32_t lapStartSysMs;     // millis() saat t_start (utk estimasi lapCurMs)

  uint16_t lapNo;
  uint32_t lastLapMs;
  uint32_t bestLapMs;
  bool     newResult;
};

static GtInternal        g_gt = {};
static SemaphoreHandle_t g_gtMutex = nullptr;

// ---------------- Helper ----------------

// Epoch (ms, UTC) dari tanggal/jam GPS. Resolusi dasar 1 detik (NMEA umumnya
// tidak memberi sub-detik); presisi sub-detik didapat lewat interpolasi posisi
// terhadap garis (lihat gtSegIntersect & pemakaiannya di gpsTimerUpdate).
static uint64_t gtEpochMs(uint16_t y, uint8_t mo, uint8_t d, uint8_t hh, uint8_t mm, uint8_t ss) {
  long yy  = (long)y - (mo <= 2 ? 1 : 0);
  long era = yy / 400;
  long yoe = yy - era * 400;
  long doy = (153L * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097L + doe - 719468L;
  uint64_t secs = (uint64_t)days * 86400ULL + (uint64_t)hh * 3600ULL + (uint64_t)mm * 60ULL + ss;
  return secs * 1000ULL;
}

// Proyeksi datar lat/lon -> meter lokal relatif terhadap titik referensi.
// Cukup akurat untuk jarak pendek (skala lintasan gokart).
static void gtToXY(double refLat, double refLon, double lat, double lon, float &x, float &y) {
  x = (float)((lon - refLon) * 111320.0 * cos(refLat * 0.017453292519943295));
  y = (float)((lat - refLat) * 110540.0);
}

// Interseksi segmen AB (garis gerbang) dengan segmen PQ (lintasan posisi
// lama -> baru). tOnPQ = posisi potong sepanjang PQ (0..1), dipakai untuk
// interpolasi waktu crossing.
static bool gtSegIntersect(float ax, float ay, float bx, float by,
                           float px, float py, float qx, float qy,
                           float &tOnPQ) {
  float rx = bx - ax, ry = by - ay;
  float sx = qx - px, sy = qy - py;
  float denom = rx * sy - ry * sx;
  if (fabsf(denom) < 1e-9f) return false;   // sejajar / segmen nol panjang
  float t = ((px - ax) * sy - (py - ay) * sx) / denom;   // param sepanjang AB
  float u = ((px - ax) * ry - (py - ay) * rx) / denom;   // param sepanjang PQ
  if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f) return false;
  tOnPQ = u;
  return true;
}

// ---------------- API ----------------

static void gpsTimerInit() {
  if (g_gtMutex == nullptr) g_gtMutex = xSemaphoreCreateMutex();
  g_gt = GtInternal{};
  g_gt.state = GT_IDLE;
}

// Set dua titik koordinat garis finish/start (derajat, WGS84).
static void gpsTimerSetGate(double lat1, double lon1, double lat2, double lon2) {
  if (g_gtMutex && xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20))) {
    g_gt.gate.set  = true;
    g_gt.gate.lat1 = lat1; g_gt.gate.lon1 = lon1;
    g_gt.gate.lat2 = lat2; g_gt.gate.lon2 = lon2;
    xSemaphoreGive(g_gtMutex);
  }
}

// Perintah "START LAP" -> STATE = ARMED.
// Reset tracking posisi supaya segmen lama (sebelum start) tidak ikut terhitung.
static void gpsTimerStart() {
  if (g_gtMutex && xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20))) {
    g_gt.state     = GT_ARMED;
    g_gt.prevValid = false;
    xSemaphoreGive(g_gtMutex);
  }
}

// Perintah "STOP LAP" -> STATE = IDLE.
static void gpsTimerStop() {
  if (g_gtMutex && xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20))) {
    g_gt.state     = GT_IDLE;
    g_gt.prevValid = false;
    xSemaphoreGive(g_gtMutex);
  }
}

// Reset total: lap count, last/best lap, state -> IDLE. Gate TIDAK direset.
static void gpsTimerReset() {
  if (g_gtMutex && xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20))) {
    GtGate keep = g_gt.gate;
    g_gt = GtInternal{};
    g_gt.gate  = keep;
    g_gt.state = GT_IDLE;
    xSemaphoreGive(g_gtMutex);
  }
}

// Dipanggil tiap ada sampel GPS baru (bukan di-poll terus-menerus).
// Mengikuti urutan pengecekan FLOWCHART 2 di atas, langkah 1-7.
static void gpsTimerUpdate(const GpsData &g) {
  if (g_gtMutex == nullptr) return;

  // 1. GPS fix valid?
  bool gpsOk = g.valid && g.sat >= GPSTIMER_MIN_SAT && g.hdop <= GPSTIMER_HDOP_MAX;
  if (!gpsOk) return;

  uint64_t nowEpochMs = gtEpochMs(g.year, g.month, g.day, g.hour, g.minute, g.second);

  if (xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  // 2. STATE = IDLE? -> abaikan sampel sepenuhnya (jangan lacak posisi)
  if (g_gt.state == GT_IDLE) {
    xSemaphoreGive(g_gtMutex);
    return;
  }

  // Belum ada posisi sebelumnya (baru saja ARMED) atau gate belum di-set:
  // simpan posisi ini sebagai acuan, tunggu sampel berikutnya.
  if (!g_gt.prevValid || !g_gt.gate.set) {
    g_gt.prevValid   = true;
    g_gt.prevLat     = g.lat;
    g_gt.prevLon     = g.lon;
    g_gt.prevEpochMs = nowEpochMs;
    xSemaphoreGive(g_gtMutex);
    return;
  }

  // 3. Melewati garis finish/start? (segmen posisi lama -> baru memotong garis)
  double refLat = g_gt.gate.lat1, refLon = g_gt.gate.lon1;
  float ax, ay, bx, by, px, py, qx, qy, u;
  gtToXY(refLat, refLon, g_gt.gate.lat1, g_gt.gate.lon1, ax, ay);
  gtToXY(refLat, refLon, g_gt.gate.lat2, g_gt.gate.lon2, bx, by);
  gtToXY(refLat, refLon, g_gt.prevLat,   g_gt.prevLon,   px, py);
  gtToXY(refLat, refLon, g.lat,          g.lon,          qx, qy);

  bool crossed = gtSegIntersect(ax, ay, bx, by, px, py, qx, qy, u);

  if (!crossed) {
    g_gt.prevLat = g.lat; g_gt.prevLon = g.lon; g_gt.prevEpochMs = nowEpochMs;
    xSemaphoreGive(g_gtMutex);
    return;
  }

  // 5. Hitung t_cross dengan interpolasi (dari timestamp GPS, bukan millis)
  uint64_t tCrossMs = g_gt.prevEpochMs +
      (uint64_t)llroundf((float)(nowEpochMs - g_gt.prevEpochMs) * u);

  // 4. Lockout aktif? (selang sejak lintasan terakhir < waktu lap minimum)
  bool lockout = (g_gt.lastCrossEpochMs != 0) &&
                 (tCrossMs - g_gt.lastCrossEpochMs) < (uint64_t)GPSTIMER_MIN_LAP_MS;
  if (lockout) {
    g_gt.prevLat = g.lat; g_gt.prevLon = g.lon; g_gt.prevEpochMs = nowEpochMs;
    xSemaphoreGive(g_gtMutex);
    return;
  }

  // 6. STATE = ARMED? (lap pertama)
  if (g_gt.state == GT_ARMED) {
    g_gt.lapStartEpochMs = tCrossMs;
    g_gt.lapStartSysMs   = millis();
    g_gt.state = GT_RUNNING;
    g_gt.lapNo = 1;
  } else {
    uint32_t lapMs = (uint32_t)(tCrossMs - g_gt.lapStartEpochMs);
    g_gt.lastLapMs = lapMs;
    if (g_gt.bestLapMs == 0 || lapMs < g_gt.bestLapMs) g_gt.bestLapMs = lapMs;
    g_gt.lapNo++;
    g_gt.lapStartEpochMs = tCrossMs;
    g_gt.lapStartSysMs   = millis();
    g_gt.newResult = true;
  }

  // 7. Set lockout (t_cross) & update posisi terakhir -> hasil dikirim
  //    di siklus data Halaman 1 (LoRa, SD, Nextion) lewat gpsTimerGet().
  g_gt.lastCrossEpochMs = tCrossMs;
  g_gt.prevLat = g.lat; g_gt.prevLon = g.lon; g_gt.prevEpochMs = nowEpochMs;

  xSemaphoreGive(g_gtMutex);
}

// Ambil snapshot hasil (thread-safe). Dipanggil dari siklus data Halaman 1.
static bool gpsTimerGet(GpsTimerSnapshot &out) {
  if (g_gtMutex == nullptr) return false;
  if (xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;

  out.state     = g_gt.state;
  out.gateSet   = g_gt.gate.set;
  out.lapNo     = g_gt.lapNo;
  out.lapCurMs  = (g_gt.state == GT_RUNNING) ? (millis() - g_gt.lapStartSysMs) : 0;
  out.lastLapMs = g_gt.lastLapMs;
  out.bestLapMs = g_gt.bestLapMs;
  out.newResult = g_gt.newResult;

  xSemaphoreGive(g_gtMutex);
  return true;
}

// Konsumsi flag "hasil lap baru" (panggil setelah data dikirim ke LoRa/SD/Nextion).
static void gpsTimerConsumeNewLap() {
  if (g_gtMutex && xSemaphoreTake(g_gtMutex, pdMS_TO_TICKS(20))) {
    g_gt.newResult = false;
    xSemaphoreGive(g_gtMutex);
  }
}