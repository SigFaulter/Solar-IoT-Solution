#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "constants.h"

// Parsed from Space line field 13 (loadState bitmask)
typedef struct {
    bool load_disconnected; // bit 0
    bool night_mode_active; // bit 1
    bool lvd_active;        // bit 2
    bool user_disconnect;   // bit 3
    bool low_temp;          // bit 4
    bool high_temp;         // bit 5
    bool over_current;      // bit 8
} LoadStatusFlags;

LoadStatusFlags load_status_parse(uint16_t v);
bool load_status_is_on(const LoadStatusFlags *f);

// Parsed from Space line field 14 (chargeState bitmask)
typedef struct {
    bool boost_charge;        // bit 0
    bool equalization_charge; // bit 1
    bool is_night;            // bit 3
    bool ssr_output;          // bit 7
} ChargeStatusFlags;

ChargeStatusFlags charge_status_parse(uint16_t v);

// Parsed from Space line field 40 (V3 only)
typedef struct {
    bool battery_over_voltage;
    bool pv_over_voltage;
    bool controller_over_temp;
    bool charge_over_current;
    bool lvd_active;
    bool over_discharge_current;
    bool battery_over_temp;
    bool battery_under_temp;
} FaultStatusFlags;

FaultStatusFlags fault_status_parse(uint16_t v);
uint32_t fault_status_to_bitmask(const FaultStatusFlags *f);

typedef struct {
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
    uint16_t avg_nightlength_min;
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
} SpaceTelemetry;

bool parse_space_line(const char *line, SpaceTelemetry *out);

static inline uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}
