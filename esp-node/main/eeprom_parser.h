#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "constants.h"

typedef struct {
    bool load_disconnect;
    bool full_charge;
    bool pv_over_current;
    bool load_over_current;
    bool battery_over_voltage;
    bool low_soc;
    bool temp_over_pv_over;
    bool temp_over_pv_low;
    bool temp_over_load_over;
} StateFlags;

StateFlags state_flags_parse(uint16_t v);
uint32_t state_flags_to_bitmask(const StateFlags *f);

typedef struct {
    uint16_t   index;
    uint8_t    vbat_max_mv;
    uint8_t    vbat_min_mv;
    uint16_t   ah_charge_mah;
    uint16_t   ah_load_mah;
    uint8_t    vpv_max_mv;
    uint8_t    vpv_min_mv;
    uint8_t    il_max_ma;
    uint8_t    ipv_max_ma;
    uint8_t    soc_pct;
    int8_t     ext_temp_max_c;
    int8_t     ext_temp_min_c;
    uint32_t   nightlength_min;
    StateFlags state_flags;
} EepromLogEntry;

typedef struct {
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
    uint32_t total_ah_charge_mah;
    uint32_t total_ah_load_mah;
    uint16_t num_days;
    uint16_t avg_morning_soc_pct;
    
    uint8_t        daily_count;
    EepromLogEntry daily_logs[EEPROM_DAILY_MAX_BLOCKS];
    uint8_t        monthly_count;
    EepromLogEntry monthly_logs[EEPROM_MONTHLY_MAX_BLOCKS];
    
    bool valid;
} EepromData;

bool parse_eeprom_line(const char *line, EepromData *out);
