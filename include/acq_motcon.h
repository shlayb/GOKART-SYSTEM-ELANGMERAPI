#pragma once
/*
DATA ACQUISITION MOTOR CONTROLLER (FARDRIVER)
INTERFACE : UART TTL (bukan RS485/CAN)
PIN
RX : 14   (ESP32 RX <- controller TX)
TX : 27   (ESP32 TX -> controller RX, hanya jika ENABLE_REQUESTS=true)
BAUDRATE : 19200
FORMAT : 8N1

PROTOCOL
Reference : https://github.com/jackhumbert/fardriver-controllers
CRC/framing/map based on commit: 7cbc0c95054a9f53e062d32b3621862e5f172a22
Open/keepalive commands hasil reverse-engineering.

WIRING / SAFETY
- UART TTL only. Konfirmasi pinout dan level tegangan controller dulu.
- Lepas modul Bluetooth eksternal sebelum menyambung ESP32 TX ke controller RX.
- Passive listening bareng Bluetooth: sambung ESP32 RX + GND saja,
  set ENABLE_REQUESTS=false.

SCOPE
- Tidak ada parameter write, AutoLearn command, atau firmware update.
- Tidak ada logging ke SD/flash.
*/

/*
PEMAKAIAN
  #include "motcon.h"

  static void taskMotcon(void*) {
    motconInit();
    for (;;) { motconUpdate(); vTaskDelay(pdMS_TO_TICKS(MOTCON_PERIOD_MS)); }
  }

  // Dari task lain (LoRa / logger / dll):
  MotconSnapshot s;
  motconSnapshot(s);
  if (s.fresh[MC_VOLTAGE]) { float v = s.val[MC_VOLTAGE]; ... }

Opsi (define SEBELUM #include "motcon.h"):
  MOTCON_RX_PIN, MOTCON_TX_PIN, MOTCON_BAUD, MOTCON_UART_NUM,
  MOTCON_ENABLE_REQUESTS (1/0; ini yang disebut ENABLE_REQUESTS di atas),
  MOTCON_AUTO_OPEN (1 = kirim Open + keepalive otomatis di motconInit),
  MOTCON_STALE_MS, MOTCON_PERIOD_MS.
*/

#include <Arduino.h>
#include <math.h>
#include <string.h>

// ================= Konfigurasi =================

#ifndef MOTCON_RX_PIN
#define MOTCON_RX_PIN 14
#endif
#ifndef MOTCON_TX_PIN
#define MOTCON_TX_PIN 27
#endif
#ifndef MOTCON_BAUD
#define MOTCON_BAUD 19200
#endif
#ifndef MOTCON_UART_NUM
#define MOTCON_UART_NUM 1          // UART1; hindari bentrok dengan UART lain (mis. LoRa di UART2)
#endif
#ifndef MOTCON_ENABLE_REQUESTS
#define MOTCON_ENABLE_REQUESTS 1   // 0 = pasif RX-only, pin TX tidak dipakai
#endif
#ifndef MOTCON_AUTO_OPEN
#define MOTCON_AUTO_OPEN 0         // referensi: tidak ada request saat boot
#endif
#ifndef MOTCON_STALE_MS
#define MOTCON_STALE_MS 5000
#endif
#ifndef MOTCON_MAX_BYTES_PER_UPDATE
#define MOTCON_MAX_BYTES_PER_UPDATE 256
#endif

#define MOTCON_FRAME_LEN        16
#define MOTCON_INTERBYTE_RESET  250    // ms; buang frame parsial jika jeda antar byte lebih lama
#define MOTCON_KEEPALIVE_MS     1000

// ================= Tipe data =================

enum MotconField : uint8_t {
  MC_VOLTAGE,          // Battery V
  MC_CURRENT,          // Battery A
  MC_SPEED_RAW,        // nilai mentah (belum diverifikasi RPM)
  MC_THROTTLE,         // throttle V dari controller
  MC_MOTOR_TEMP,       // C* (mapping perlu dibandingkan dengan app)
  MC_CONTROLLER_TEMP,  // C*
  MC_PHASE_A,          // A*
  MC_PHASE_C,          // A*
  MC_FIELD_COUNT
};

// Salinan konsisten dari data terakhir, aman dipanggil dari task lain.
struct MotconSnapshot {
  float    val[MC_FIELD_COUNT];
  bool     fresh[MC_FIELD_COUNT];   // true = pernah terbaca, finite, dan umur <= MOTCON_STALE_MS
  uint32_t ageMs[MC_FIELD_COUNT];   // umur data (ms); hanya bermakna jika pernah terbaca
  bool     statusValid;             // status di bawah masih segar
  uint8_t  brake;
  uint8_t  phaseBits;
  uint8_t  gearRaw;
  uint16_t faults;                  // bitmask, nama lewat motconFaultName()
  uint32_t rxBytes;
  uint32_t validFrames;
  uint32_t crcErrors;
  uint32_t otherFrames;
  uint32_t ms;                      // millis() saat snapshot
};

// ================= State internal =================

struct MotconState {
  float    val[MC_FIELD_COUNT];
  uint32_t time[MC_FIELD_COUNT];
  bool     seen[MC_FIELD_COUNT];
  uint16_t faults;
  uint8_t  brake;
  uint8_t  phases;
  uint8_t  gear;
  bool     statusSeen;
  uint32_t statusTime;
  uint32_t rxBytes;
  uint32_t validFrames;
  uint32_t crcErrors;
  uint32_t otherFrames;
};

static HardwareSerial mcSerial(MOTCON_UART_NUM);
static portMUX_TYPE   mcMux = portMUX_INITIALIZER_UNLOCKED;
static MotconState    mcS;

// parser (hanya disentuh dari task motconUpdate)
static uint8_t  mcFrame[MOTCON_FRAME_LEN];
static uint8_t  mcUsed = 0;
static uint32_t mcLastByteTime = 0;

// keepalive (boleh diubah dari task lain)
static volatile bool mcKeepAlive = false;
static uint32_t      mcLastKeepAlive = 0;

static const uint8_t mcAddressMap[55] = {
  0xE2,0xE8,0xEE,0x00,0x06,0x0C,0x12,
  0xE2,0xE8,0xEE,0x18,0x1E,0x24,0x2A,
  0xE2,0xE8,0xEE,0x30,0x5D,0x63,0x69,
  0xE2,0xE8,0xEE,0x7C,0x82,0x88,0x8E,
  0xE2,0xE8,0xEE,0x94,0x9A,0xA0,0xA6,
  0xE2,0xE8,0xEE,0xAC,0xB2,0xB8,0xBE,
  0xE2,0xE8,0xEE,0xC4,0xCA,0xD0,
  0xE2,0xE8,0xEE,0xD6,0xDC,0xF4,0xFA
};

static const char *mcFaultNames[15] = {
  "Hall", "Throttle", "CurrentRestart", "PhaseSurge", "Voltage",
  "Alarm", "MotorTemp", "ControllerTemp", "PhaseOverflow", "PhaseZero",
  "PhaseShort?", "LineZero", "MOSHigh", "MOSLow", "MOE"
};

// ================= Decoding =================

static inline uint16_t mcCRC(const uint8_t *data, size_t length) {
  uint16_t crc = 0x7F3C;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? uint16_t((crc >> 1) ^ 0xA001) : uint16_t(crc >> 1);
    }
  }
  return crc;
}

static inline uint16_t mcU16le(const uint8_t *p) {
  return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

static inline int32_t mcS16le(const uint8_t *p) {
  uint16_t u = mcU16le(p);
  return (u & 0x8000) ? int32_t(u) - 65536 : int32_t(u);
}

static inline float mcPhaseCurrent(const uint8_t *p) {
  uint32_t raw = (uint32_t(p[0]) << 16) | (uint32_t(p[1]) << 8) | p[2];
  return 1.953125f * sqrtf(float(raw));
}

// Dipanggil di dalam critical section.
static inline void mcSet(MotconField f, float value, uint32_t now) {
  mcS.val[f]  = value;
  mcS.time[f] = now;
  mcS.seen[f] = true;
}

static void mcDecodeFrame(uint8_t address, const uint8_t *p) {
  uint32_t now = millis();
  portENTER_CRITICAL(&mcMux);
  switch (address) {
    case 0xE8:
      mcSet(MC_VOLTAGE, mcS16le(p) / 10.0f, now);
      mcSet(MC_CURRENT, mcS16le(p + 4) / 4.0f, now);
      break;
    case 0xE2:
      mcSet(MC_SPEED_RAW, mcU16le(p + 6), now);
      mcS.faults = uint16_t(p[2]) | (uint16_t(p[3] & 0x7F) << 8);
      mcS.brake  = (p[3] >> 7) & 1;
      mcS.phases = p[1] & 7;
      mcS.gear   = (p[0] >> 2) & 3;
      mcS.statusSeen = true;
      mcS.statusTime = now;
      break;
    case 0x82:
      mcSet(MC_THROTTLE, mcU16le(p) * 0.01f, now);
      break;
    case 0xF4:
      mcSet(MC_MOTOR_TEMP, mcS16le(p), now);
      break;
    case 0xD6:
      mcSet(MC_CONTROLLER_TEMP, mcS16le(p + 10), now);
      break;
    case 0xEE:
      mcSet(MC_PHASE_A, mcPhaseCurrent(p + 4), now);
      mcSet(MC_PHASE_C, mcPhaseCurrent(p + 7), now);
      break;
  }
  portEXIT_CRITICAL(&mcMux);
}

static void mcReceiveByte(uint8_t b) {
  uint32_t now = millis();
  if (mcUsed && uint32_t(now - mcLastByteTime) > MOTCON_INTERBYTE_RESET) {
    mcUsed = 0;
  }
  mcLastByteTime = now;
  ++mcS.rxBytes;

  mcFrame[mcUsed++] = b;
  if (mcUsed < MOTCON_FRAME_LEN) return;

  if (mcFrame[0] == 0xAA && (mcFrame[1] >> 6) == 2) {
    if (mcCRC(mcFrame, 14) == mcU16le(mcFrame + 14)) {
      uint8_t id = mcFrame[1] & 0x3F;
      if (id < sizeof(mcAddressMap)) {
        ++mcS.validFrames;
        mcDecodeFrame(mcAddressMap[id], mcFrame + 2);
      } else {
        ++mcS.otherFrames;
      }
      mcUsed = 0;
      return;
    }
    ++mcS.crcErrors;
  }

  // frame belum valid: geser 1 byte, cari sinkronisasi berikutnya
  memmove(mcFrame, mcFrame + 1, MOTCON_FRAME_LEN - 1);
  mcUsed = MOTCON_FRAME_LEN - 1;
}

// ================= Session request (reverse-engineered) =================

static void mcSendSessionRequest(bool opening) {
#if MOTCON_ENABLE_REQUESTS
  uint8_t p[8] = {
    0xAA, 0x13, 0xEC, 0x07,
    uint8_t(opening ? 0x01 : 0x5F),
    uint8_t(opening ? 0xF1 : 0x5F),
    0, 0
  };
  for (uint8_t i = 0; i < 6; ++i) p[6] += p[i];
  p[7] = uint8_t(~p[6]);
  mcSerial.write(p, sizeof(p));
#else
  (void)opening;
#endif
}

// Kirim Open + aktifkan keepalive 1 Hz. Tidak ada efek jika MOTCON_ENABLE_REQUESTS=0.
static inline void motconOpen() {
#if MOTCON_ENABLE_REQUESTS
  mcSendSessionRequest(true);
  mcLastKeepAlive = millis();
  mcKeepAlive = true;
#endif
}

// Hentikan keepalive (BUKAN perintah stop motor).
static inline void motconStopKeepAlive() {
  mcKeepAlive = false;
}

// ================= API utama =================

static void motconInit() {
  memset(&mcS, 0, sizeof(mcS));
  mcUsed = 0;
  mcLastByteTime = 0;
  mcKeepAlive = false;

  mcSerial.setRxBufferSize(2048);  // harus sebelum begin()
  mcSerial.begin(MOTCON_BAUD, SERIAL_8N1, MOTCON_RX_PIN,
                 MOTCON_ENABLE_REQUESTS ? MOTCON_TX_PIN : -1);

#if MOTCON_AUTO_OPEN
  motconOpen();
#endif
}

static void motconUpdate() {
  for (uint16_t i = 0; i < MOTCON_MAX_BYTES_PER_UPDATE && mcSerial.available(); ++i) {
    int b = mcSerial.read();
    if (b >= 0) mcReceiveByte(uint8_t(b));
  }

#if MOTCON_ENABLE_REQUESTS
  uint32_t now = millis();
  if (mcKeepAlive && uint32_t(now - mcLastKeepAlive) >= MOTCON_KEEPALIVE_MS) {
    mcLastKeepAlive = now;
    mcSendSessionRequest(false);
  }
#endif
}

static void motconSnapshot(MotconSnapshot &out) {
  MotconState s;
  portENTER_CRITICAL(&mcMux);
  s = mcS;
  portEXIT_CRITICAL(&mcMux);

  uint32_t now = millis();
  out.ms = now;
  for (uint8_t i = 0; i < MC_FIELD_COUNT; ++i) {
    uint32_t age = s.seen[i] ? uint32_t(now - s.time[i]) : 0;
    out.val[i]   = s.val[i];
    out.ageMs[i] = age;
    out.fresh[i] = s.seen[i] && age <= MOTCON_STALE_MS && isfinite(s.val[i]);
  }
  out.statusValid = s.statusSeen && uint32_t(now - s.statusTime) <= MOTCON_STALE_MS;
  out.brake       = out.statusValid ? s.brake  : 0;
  out.phaseBits   = out.statusValid ? s.phases : 0;
  out.gearRaw     = out.statusValid ? s.gear   : 0;
  out.faults      = out.statusValid ? s.faults : 0;
  out.rxBytes     = s.rxBytes;
  out.validFrames = s.validFrames;
  out.crcErrors   = s.crcErrors;
  out.otherFrames = s.otherFrames;
}

// Nama fault untuk bit 0..14 (mapping repo). nullptr jika di luar range.
static inline const char *motconFaultName(uint8_t bit) {
  return bit < 15 ? mcFaultNames[bit] : nullptr;
}

// Opsional untuk debug via Serial.
static void motconPrintDebug(Print &out = Serial) {
  static const char *labels[MC_FIELD_COUNT] = {
    "Battery V", "Battery A", "SpeedRaw", "Throttle V",
    "Motor C*", "Controller C*", "Phase A*", "Phase C*"
  };
  MotconSnapshot s;
  motconSnapshot(s);
  out.printf("[motcon t=%lu] RX=%lu valid=%lu badCRC=%lu other=%lu\n",
             (unsigned long)s.ms, (unsigned long)s.rxBytes,
             (unsigned long)s.validFrames, (unsigned long)s.crcErrors,
             (unsigned long)s.otherFrames);
  for (uint8_t i = 0; i < MC_FIELD_COUNT; ++i) {
    if (s.fresh[i]) out.printf("  %-14s: %.2f [age=%lu ms]\n", labels[i], s.val[i], (unsigned long)s.ageMs[i]);
    else            out.printf("  %-14s: N/A/STALE\n", labels[i]);
  }
  if (s.statusValid) {
    out.printf("  Status: brake=%u phaseBits=0x%X gearRaw=%u faults=0x%04X\n",
               unsigned(s.brake), unsigned(s.phaseBits), unsigned(s.gearRaw), unsigned(s.faults));
  } else {
    out.println("  Status: N/A/STALE");
  }
}