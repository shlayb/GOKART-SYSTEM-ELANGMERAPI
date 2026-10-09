#pragma once

#include <stddef.h>
#include <stdint.h>

namespace elmer {

// Pengganti std::optional: nilai + flag. Tanpa std, tanpa heap, C++11 ok.
template <class T>
struct Opt {
    T v;
    bool has;

    Opt() : v(), has(false) {}

    Opt &operator=(const T &x) { v = x; has = true; return *this; }
    void reset() { has = false; }

    bool has_value() const { return has; }
    explicit operator bool() const { return has; }
    const T &operator*() const { return v; }
    T &operator*() { return v; }
    T value_or(const T &def) const { return has ? v : def; }
};

// Nama field identik dengan domain/telemetry.py. Hanya seq yang wajib.
// Opt kosong = tidak tersedia (bukan nol/false), dikirim sebagai sentinel N/A.
// Struct ini TIDAK dikirim sebagai raw bytes; ia dikuantisasi ke paket biner
// little-endian 101 byte (lihat namespace wire).
struct Telemetry {
    uint32_t seq = 0;

    // Drive measurements.
    Opt<double> speed_kmh;          // 0..400 (x100)
    Opt<double> rpm;                // 0..50000 (x1)
    Opt<double> current_a;          // -1000..1000 (x10)
    Opt<double> throttle_pct;       // 0..100 (x100)
    Opt<double> brake_pct;          // 0..100 (x100)
    Opt<double> battery_pct;        // 0..100 (x100)
    Opt<double> voltage_v;          // 0..1000 (x10)
    Opt<double> temp_batt_c;        // -50..250 (x100)
    Opt<double> temp_motor_c;       // -50..250 (x100)
    Opt<double> temp_controller_c;  // -50..250 (x100)
    Opt<double> temp_board_c;       // -50..250 (x100)
    Opt<bool> boost;
    Opt<bool> controller_enabled;
    Opt<uint64_t> fault_code;       // 0 = tanpa fault (wire: u32)

    // Battery management system.
    Opt<double> bms_voltage_v;                  // 0..1000 (x10)
    Opt<double> bms_current_a;                  // -1000..1000 (x10)
    Opt<double> bms_discharge_time_s;           // >= 0 (wire: u32)
    Opt<bool> bms_has_alarm;
    Opt<double> bms_cell_voltage_max_v;         // >= 0 (x1000)
    Opt<double> bms_cell_voltage_min_v;         // >= 0 (x1000)
    Opt<uint64_t> bms_cell_voltage_max_index;   // wire: u16
    Opt<uint64_t> bms_cell_voltage_min_index;   // wire: u16
    Opt<double> bms_cell_temp_max_c;            // -50..250 (x100)
    Opt<double> bms_cell_temp_min_c;            // -50..250 (x100)
    Opt<double> bms_cell_temp_avg_c;            // -50..250 (x100)
    Opt<uint64_t> bms_cell_temp_max_index;      // wire: u16
    Opt<uint64_t> bms_cell_temp_min_index;      // wire: u16
    Opt<uint64_t> bms_frame_count;              // wire: u32
    Opt<uint32_t> bms_updated_at_raw;           // dulu string, kini angka u32

    // Motor controller.
    Opt<double> controller_voltage_v;           // 0..1000 (x10)
    Opt<double> controller_current_a;           // -1000..1000 (x10)
    Opt<double> temp_external_c;                // -50..250 (x100)
    Opt<double> temp_coeff;                     // angka finite (wire: float32)
    Opt<uint8_t> gear_status;                   // 0..253; 0xFE = lainnya
    Opt<uint8_t> controller_status;             // wire::kStatusOk/Warn/Fault; 0xFE = lainnya
    Opt<bool> controller_data_valid;
    Opt<uint32_t> controller_updated_at_raw;    // angka u32

    // Device metadata and GPS.
    Opt<uint32_t> device_updated_at_raw;        // angka u32
    Opt<double> latitude_deg;                   // -90..90 (x1e7, i32)
    Opt<double> longitude_deg;                  // -180..180 (x1e7, i32)
    // struct Telemetry — tambahkan field baru (mis. setelah blok "Drive measurements")
    Opt<uint8_t> lap_state;      // 0=IDLE, 1=ARMED, 2=RUNNING (wire: u8)
};

namespace wire {
constexpr uint8_t kSync0 = 0xAA;
constexpr uint8_t kSync1 = 0x55;
constexpr size_t  kPacketSize = 102;  // sync(2) + payload(98) + crc(2)
constexpr size_t  kCrcOffset = 100;   // CRC-16/CCITT-FALSE atas offset 2..99

constexpr uint8_t kStatusOk = 0;
constexpr uint8_t kStatusWarn = 1;
constexpr uint8_t kStatusFault = 2;
constexpr uint8_t kOther = 0xFE;
constexpr uint8_t kNaU8 = 0xFF;
constexpr uint8_t kLapIdle = 0;
constexpr uint8_t kLapArmed = 1;
constexpr uint8_t kLapRunning = 2;
}  // namespace wire

}  // namespace elmer