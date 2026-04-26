#pragma once

#include <cstdint>
#include <algorithm>
#include "constants.h"

// Parsed from Space line field 13 (loadState bitmask)
struct LoadStatusFlags {
    bool load_disconnected = false; // bit 0
    bool night_mode_active = false; // bit 1
    bool lvd_active        = false; // bit 2
    bool user_disconnect   = false; // bit 3
    bool low_temp          = false; // bit 4
    bool high_temp         = false; // bit 5
    bool over_current      = false; // bit 8

    static LoadStatusFlags parse(uint16_t v) {
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
    bool boost_charge        = false; // bit 0
    bool equalization_charge = false; // bit 1
    bool is_night            = false; // bit 3
    bool ssr_output          = false; // bit 7

    static ChargeStatusFlags parse(uint16_t v) {
        ChargeStatusFlags f;
        f.boost_charge        = (v & 0x01) != 0;
        f.equalization_charge = (v & 0x02) != 0;
        f.is_night            = (v & 0x08) != 0;
        f.ssr_output          = (v & 0x80) != 0;
        return f;
    }
};

// Parsed from Space line field 40 (V3 only)
struct FaultStatusFlags {
    bool battery_over_voltage   = false;
    bool pv_over_voltage        = false;
    bool controller_over_temp   = false;
    bool charge_over_current    = false;
    bool lvd_active             = false;
    bool over_discharge_current = false;
    bool battery_over_temp      = false;
    bool battery_under_temp     = false;

    static FaultStatusFlags parse(uint16_t v) {
        FaultStatusFlags f;
        f.battery_over_voltage   = (v & 0x01) != 0;
        f.pv_over_voltage        = (v & 0x02) != 0;
        f.controller_over_temp   = (v & 0x04) != 0;
        f.charge_over_current    = (v & 0x08) != 0;
        f.lvd_active             = (v & 0x10) != 0;
        f.over_discharge_current = (v & 0x20) != 0;
        f.battery_over_temp      = (v & 0x40) != 0;
        f.battery_under_temp     = (v & 0x80) != 0;
        return f;
    }

    uint32_t to_bitmask() const {
        uint32_t v = 0;
        if (battery_over_voltage)   v |= (1U << 0);
        if (pv_over_voltage)        v |= (1U << 1);
        if (controller_over_temp)   v |= (1U << 2);
        if (charge_over_current)    v |= (1U << 3);
        if (lvd_active)             v |= (1U << 4);
        if (over_discharge_current) v |= (1U << 5);
        if (battery_over_temp)      v |= (1U << 6);
        if (battery_under_temp)     v |= (1U << 7);
        return v;
    }
};

struct StateFlags {
    bool load_disconnect      = false;
    bool full_charge          = false;
    bool pv_over_current      = false;
    bool load_over_current    = false;
    bool battery_over_voltage = false;
    bool low_soc              = false;
    bool temp_over_pv_over    = false;
    bool temp_over_pv_low     = false;
    bool temp_over_load_over  = false;

    static StateFlags parse(uint16_t v) {
        StateFlags f;
        f.load_disconnect      = (v & 0x0001) != 0;
        f.full_charge          = (v & 0x0002) != 0;
        f.pv_over_current      = (v & 0x0004) != 0;
        f.load_over_current    = (v & 0x0008) != 0;
        f.battery_over_voltage = (v & 0x0010) != 0;
        f.low_soc              = (v & 0x0020) != 0;
        f.temp_over_pv_over    = (v & 0x0040) != 0;
        f.temp_over_pv_low     = (v & 0x0080) != 0;
        f.temp_over_load_over  = (v & 0x0100) != 0;
        return f;
    }

    uint32_t to_bitmask() const {
        uint32_t v = 0;
        if (load_disconnect)      v |= (1U << 0);
        if (full_charge)          v |= (1U << 1);
        if (pv_over_current)      v |= (1U << 2);
        if (load_over_current)    v |= (1U << 3);
        if (battery_over_voltage) v |= (1U << 4);
        if (low_soc)              v |= (1U << 5);
        if (temp_over_pv_over)    v |= (1U << 6);
        if (temp_over_pv_low)     v |= (1U << 7);
        if (temp_over_load_over)  v |= (1U << 8);
        return v;
    }
};

struct EepromLogEntry {
    uint8_t    index;
    uint8_t    vbat_max_mv;
    uint8_t    vbat_min_mv;
    uint16_t   ah_charge_mah;
    uint16_t   ah_load_mah;
    uint8_t    vpv_max_mv;
    uint8_t    vpv_min_mv;
    uint8_t    il_max_ma;
    uint8_t    ipv_max_ma;
    float      soc_pct;
    int8_t     ext_temp_max_c;
    int8_t     ext_temp_min_c;
    uint8_t    nightlen_min;
    StateFlags state_flags;
};

struct SpaceTelemetry {
    uint32_t firmware_version;
    int16_t  internal_temp_c;
    int16_t  external_temp_c;
    uint16_t op_days;
    uint32_t battery_voltage_mv;
    uint8_t  battery_soc_pct;
    uint32_t charge_current_ma10;
    uint32_t bat_threshold_mv;
    uint16_t bat_op_days;
    uint16_t energy_in_wh;
    uint16_t energy_out_wh;
    uint16_t energy_retained_wh;
    uint16_t charge_power_w;
    uint32_t load_current_ma10;
    uint16_t load_power_w;
    uint32_t pv_voltage_mv;
    uint32_t pv_target_mv;
    uint16_t pwm_counts;
    uint16_t nightlength_min;
    uint16_t avg_nightlength;
    uint32_t led_voltage_mv;
    uint32_t led_current_ma10;
    uint16_t led_power_w;
    uint8_t  led_status;
    uint8_t  dali_active;
    uint16_t charge_state_raw;
    uint16_t load_state_raw;
    uint16_t fault_status;
    uint8_t  pv_detected;
    uint8_t  battery_detected;
    uint8_t  hw_version;
    
    LoadStatusFlags   load_flags;
    ChargeStatusFlags charge_flags;
    FaultStatusFlags  fault_flags;
};

struct EepromData {
    uint8_t  night_mode;
    uint16_t evening_minutes;
    uint16_t morning_minutes;
    uint16_t lvd_current_mv;
    uint16_t lvd_voltage_mv;
    uint8_t  dim_mode;
    uint16_t dim_evening_min;
    uint16_t dim_morning_min;
    uint8_t  lvd_mode_voltage;
    uint8_t  battery_type_idx;
    int8_t   temp_comp;
    uint16_t night_threshold_mv;
    uint8_t  dimming_pct;
    uint8_t  base_dimming_pct;
    uint16_t equalization_mv;
    uint16_t boost_mv;
    uint16_t float_mv;
    uint16_t capacity_ah;
    uint8_t  dali_enable;
    uint8_t  alc_enable;
    uint8_t  sn_lo, sn_hi;
    uint8_t  prod_day, prod_month, prod_year_lo, prod_year_hi;
    uint16_t device_identifier;
    uint16_t days_with_lvd;
    uint8_t  months_without_full_charge;
    float    total_ah_charge_mah;
    float    total_ah_load_mah;
    uint16_t num_days;
    float    avg_morning_soc_pct;
    
    uint8_t        daily_count;
    EepromLogEntry daily_logs[EEPROM_DAILY_MAX_BLOCKS];
    uint8_t        monthly_count;
    EepromLogEntry monthly_logs[EEPROM_MONTHLY_MAX_BLOCKS];
    
    bool valid;
};

bool parse_space_line(const char *line, SpaceTelemetry &out);
bool parse_eeprom_line(const char *line, EepromData &out);

inline uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}
