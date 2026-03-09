#pragma once

#include <cstdint>
#include <string>
#include <vector>


// Parsed from Space line field 13 (loadState bitmask)
struct LoadStatusFlags {
    bool load_disconnected = false;  // bit 0
    bool night_mode_active = false;  // bit 1
    bool lvd_active        = false;  // bit 2
    bool user_disconnect   = false;  // bit 3
    bool low_temp          = false;  // bit 4
    bool high_temp         = false;  // bit 5
    bool over_current      = false;  // bit 8

    static LoadStatusFlags parse(int v) {
        LoadStatusFlags f;
        f.load_disconnected = (v & 0x001) != 0;
        f.night_mode_active = (v & 0x002) != 0;
        f.lvd_active        = (v & 0x004) != 0;
        f.user_disconnect   = (v & 0x008) != 0;
        f.low_temp          = (v & 0x010) != 0;
        f.high_temp         = (v & 0x020) != 0;
        f.over_current      = (v & 0x100) != 0;
        return f;
    }

    bool load_on() const { return !load_disconnected; }
};


// Parsed from Space line field 14 (chargeState bitmask)
struct ChargeStatusFlags {
    bool boost_charge        = false;  // bit 0
    bool equalization_charge = false;  // bit 1
    bool is_night            = false;  // bit 3
    bool dimming_override    = false;  // bit 6  (V2 only)
    bool ssr_output          = false;  // bit 7

    static ChargeStatusFlags parse(int v) {
        ChargeStatusFlags f;
        f.boost_charge        = (v & 0x01) != 0;
        f.equalization_charge = (v & 0x02) != 0;
        f.is_night            = (v & 0x08) != 0;
        f.dimming_override    = (v & 0x40) != 0;
        f.ssr_output          = (v & 0x80) != 0;
        return f;
    }
};


// Parsed from Space line field 40 (V3 only)
struct FaultStatusFlags {
    bool pv_over_voltage     = false;  // bit 1
    bool charge_over_current = false;  // bit 3

    static FaultStatusFlags parse(int v) {
        FaultStatusFlags f;
        f.pv_over_voltage     = (v & 0x02) != 0;
        f.charge_over_current = (v & 0x08) != 0;
        return f;
    }
};


struct PhocosHeader {
    std::string type;
    std::string production_date;
    std::string serial_number;
    int         hw_version = 3;   // 2 or 3, inferred from type string
};


// One parsed Space Command response (a single poll from the controller).
struct PhocosTelemetry {
    // General
    uint8_t  firmware_version   = 0;
    int      internal_temp_C    = 0;
    int      external_temp_C    = 0;
    uint16_t op_days            = 0;

    // Battery
    uint32_t battery_voltage_mV   = 0;
    uint8_t  battery_soc_pct      = 0;
    uint32_t charge_current_mA    = 0;
    uint32_t battery_threshold_mV = 0;
    uint16_t bat_op_days          = 0;
    uint16_t energy_in_daily_Wh   = 0;
    uint16_t energy_out_daily_Wh  = 0;
    uint16_t energy_retained_Wh   = 0;
    uint16_t charge_power_W       = 0;
    uint8_t  battery_detected     = 0;

    // Load
    uint32_t load_current_mA = 0;
    uint16_t load_power_W    = 0;

    // PV
    uint32_t pv_voltage_mV = 0;
    uint32_t pv_target_mV  = 0;
    uint8_t  pv_detected   = 0;
    uint16_t pwm_counts    = 0;

    // Night
    uint16_t nightlength_min     = 0;
    uint16_t avg_nightlength_min = 0;

    // LED
    uint32_t led_voltage_mV = 0;
    uint32_t led_current_mA = 0;
    uint16_t led_power_W    = 0;
    uint8_t  led_status     = 0;
    uint8_t  dali_active    = 0;

    // Raw state fields
    int charge_state_raw = 0;   // field 14, source of charge mode and is_night
    int load_state_raw   = 0;   // field 13
    int load_state2_raw  = 0;   // field 24
    int mpp_state        = 0;   // field 21
    int hvd_state        = 0;   // field 22

    // Parsed flags
    LoadStatusFlags   load_flags   = {};
    ChargeStatusFlags charge_flags = {};
    FaultStatusFlags  fault_flags  = {};
    uint16_t          fault_status = 0;

    // Set by parser from semicolon count, not from a protocol field
    int hw_version = 3;
};


// One day of recorded data from the EEPROM circular buffer.
struct DailyLog {
    int      day_index       = 0;   // 1-based chronological day number
    uint16_t vbat_max_mV     = 0;
    uint16_t vbat_min_mV     = 0;
    uint32_t ah_charge_mAh   = 0;
    uint32_t ah_load_mAh     = 0;
    uint16_t vpv_max_mV      = 0;
    uint16_t vpv_min_mV      = 0;
    uint16_t il_max_mA       = 0;
    uint16_t ipv_max_mA      = 0;
    float    soc_pct         = 0.0f;
    int8_t   ext_temp_max_C  = 0;
    int8_t   ext_temp_min_C  = 0;
    uint16_t nightlength_min = 0;
    uint16_t state           = 0;   // 9-bit event bitmask

    // State flag accessors
    bool isLd()    const { return (state & 0x0001) != 0; }   // Load Disconnect
    bool isFcb()   const { return (state & 0x0002) != 0; }   // Full Charge Battery
    bool isPvoc()  const { return (state & 0x0004) != 0; }   // PV Over-Current
    bool isLoc()   const { return (state & 0x0008) != 0; }   // Load Over-Current
    bool isBov()   const { return (state & 0x0010) != 0; }   // Battery Over-Voltage
    bool isLsoc()  const { return (state & 0x0020) != 0; }   // Low SOC
    bool isTopvo() const { return (state & 0x0040) != 0; }   // Temp Over, PV Over
    bool isTopvl() const { return (state & 0x0080) != 0; }   // Temp Over, PV Low
    bool isTolo()  const { return (state & 0x0100) != 0; }   // Temp Over, Load Over
};


// One month of recorded data from the EEPROM circular buffer.
// Same 16-byte structure as DailyLog but Ah values are raw u16 (no /10 division).
struct MonthlyLog {
    int      month_index     = 0;
    uint16_t vbat_max_mV     = 0;
    uint16_t vbat_min_mV     = 0;
    uint32_t ah_charge_mAh   = 0;
    uint32_t ah_load_mAh     = 0;
    uint16_t vpv_max_mV      = 0;
    uint16_t vpv_min_mV      = 0;
    uint16_t il_max_mA       = 0;
    uint16_t ipv_max_mA      = 0;
    float    soc_pct         = 0.0f;
    int8_t   ext_temp_max_C  = 0;
    int8_t   ext_temp_min_C  = 0;
    uint16_t nightlength_min = 0;
    uint16_t state           = 0;
};


// All configuration fields read from the EEPROM dump.
// Parsed from the same '!' dump line as the datalogger data.
struct EepromConfig {
    // Identity
    std::string device_id;
    std::string serial_number;
    std::string production_date;
    int         hw_version = 3;    // 2 or 3, from EEPROM byte 120

    // Battery
    int         battery_type_index = 0;
    std::string battery_type;
    int         capacity_ah        = 0;
    int         battery_op_days    = 0;

    // LVD
    int  lvd_voltage_mV   = 0;
    bool lvd_mode_voltage = true;

    // Charge setpoints
    int    equalization_mV    = 0;
    int    boost_mV           = 0;
    int    float_mV           = 0;
    double temp_comp_mV_per_C = 0.0;

    // Night
    int night_threshold_mV = 0;
    int operation_days     = 0;

    // DALI / ALC
    bool dali_active = false;
    bool alc_dimming = false;

    // Night mode
    int         night_mode_index   = 0;
    std::string night_mode;
    int         evening_minutes_mn = 0;
    int         morning_minutes_mn = 0;

    // Dimming
    int         night_mode_dimming_index   = 0;
    std::string night_mode_dimming;
    int         evening_minutes_dimming_mn = 0;
    int         morning_minutes_dimming_mn = 0;
    int         dimming_pct                = 0;
    int         base_dimming_pct           = 0;
};


// Summary statistics from the EEPROM datalogger block (offset 128).
struct DataloggerSummary {
    uint16_t days_with_lvd              = 0;
    uint8_t  months_without_full_charge = 0;
    float    avg_morning_soc_pct        = 0.0f;
    float    total_ah_charge            = 0.0f;
    float    total_ah_load              = 0.0f;
    uint16_t num_days                   = 0;
};
