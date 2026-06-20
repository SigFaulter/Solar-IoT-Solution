#include "eeprom_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* ---------- Helpers ---------- */

static uint8_t byte_at(const uint8_t *data, int len, int offset) {
    if (offset < 0 || offset >= len) return 0;
    return data[offset];
}

static uint16_t u16be(const uint8_t *data, int len, int msb_off) {
    return (uint16_t)((uint16_t)byte_at(data, len, msb_off) << 8 | byte_at(data, len, msb_off + 1));
}

static uint32_t u32be(const uint8_t *data, int len, int b0) {
    return ((uint32_t)byte_at(data, len, b0) << 24) |
           ((uint32_t)byte_at(data, len, b0 + 1) << 16) |
           ((uint32_t)byte_at(data, len, b0 + 2) << 8) |
           (uint32_t)byte_at(data, len, b0 + 3);
}

static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int parse_hex_dump(const char *sv, uint8_t *out, int max_len) {
    int count = 0;
    const char *p = sv;
    while (*p && count < max_len) {
        while (*p && (isspace((unsigned char)*p) || *p == ';')) p++;
        if (!*p || *p == '\r' || *p == '\n') break;

        const char *next = p;
        while (*next && !isspace((unsigned char)*next) && *next != ';') next++;

        int field_len = (int)(next - p);
        if (field_len > 0 && field_len <= 3) {
            bool is_hex = true;
            for (int i = 0; i < field_len; i++) {
                if (!isxdigit((unsigned char)p[i])) {
                    is_hex = false;
                    break;
                }
            }
            out[count++] = (uint8_t)strtol(p, NULL, is_hex ? 16 : 10);
        }
        p = next;
    }
    return count;
}

/* ---------- State Flags Parser ---------- */

StateFlags state_flags_parse(uint16_t v) {
    StateFlags f = {0};
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

uint32_t state_flags_to_bitmask(const StateFlags *f) {
    uint32_t v = 0;
    if (f->load_disconnect)      v |= (1U << 0);
    if (f->full_charge)          v |= (1U << 1);
    if (f->pv_over_current)      v |= (1U << 2);
    if (f->load_over_current)    v |= (1U << 3);
    if (f->battery_over_voltage) v |= (1U << 4);
    if (f->low_soc)              v |= (1U << 5);
    if (f->temp_over_pv_over)    v |= (1U << 6);
    if (f->temp_over_pv_low)     v |= (1U << 7);
    if (f->temp_over_load_over)  v |= (1U << 8);
    return v;
}

/* ---------- EEPROM Line Parser ---------- */

bool parse_eeprom_line(const char *line, EepromData *out) {
    if (!line || !out) return false;
    memset(out, 0, sizeof(EepromData));

    const char *p = line;
    while (*p && (isspace((unsigned char)*p) || *p == '!')) p++;
    if (!*p) return false;

    static uint8_t data[2048];
    int len = parse_hex_dump(p, data, sizeof(data));
    if (len < 144) return false;

    out->device_identifier = u16be(data, len, EEPROM_DEVICE_ID_OFFSET);

    out->night_mode = byte_at(data, len, EEPROM_NIGHT_MODE_OFFSET);
    out->evening_minutes = u16be(data, len, EEPROM_EVENING_MINUTES_OFFSET);
    out->morning_minutes = u16be(data, len, EEPROM_MORNING_MINUTES_OFFSET);
    out->lvd_current_mv = u16be(data, len, EEPROM_LVD_CURRENT_MV_OFFSET);
    out->lvd_voltage_mv = u16be(data, len, EEPROM_LVD_VOLTAGE_MV_OFFSET);
    out->dim_mode = byte_at(data, len, EEPROM_DIM_MODE_OFFSET);
    out->dim_evening_min = u16be(data, len, EEPROM_DIM_EVENING_OFFSET);
    out->dim_morning_min = u16be(data, len, EEPROM_DIM_MORNING_OFFSET);
    out->lvd_mode_voltage = byte_at(data, len, EEPROM_LVD_MODE_OFFSET);
    out->battery_type_idx = byte_at(data, len, EEPROM_BATTERY_TYPE_OFFSET);
    out->temp_comp = (int8_t)byte_at(data, len, EEPROM_TEMP_COMP_OFFSET);
    out->night_threshold_mv = u16be(data, len, EEPROM_NIGHT_THRESH_MV_OFFSET);
    out->dimming_pct = byte_at(data, len, EEPROM_DIMMING_PCT_OFFSET);
    out->base_dimming_pct = byte_at(data, len, EEPROM_BASE_DIMMING_PCT_OFFSET);
    out->equalization_mv = u16be(data, len, EEPROM_EQUALIZATION_MV_OFFSET);
    out->boost_mv = u16be(data, len, EEPROM_BOOST_MV_OFFSET);
    out->float_mv = u16be(data, len, EEPROM_FLOAT_MV_OFFSET);
    out->capacity_ah = u16be(data, len, EEPROM_CAPACITY_AH_OFFSET);
    out->dali_enable = byte_at(data, len, EEPROM_DALI_FLAG_OFFSET);
    out->alc_enable = byte_at(data, len, EEPROM_ALC_FLAG_OFFSET);

    out->sn_lo = byte_at(data, len, EEPROM_SERIAL_LSB_OFFSET);
    out->sn_hi = byte_at(data, len, EEPROM_SERIAL_MSB_OFFSET);
    out->prod_day = byte_at(data, len, EEPROM_PROD_DAY_OFFSET);
    out->prod_month = byte_at(data, len, EEPROM_PROD_MONTH_OFFSET);
    out->prod_year_lo = byte_at(data, len, EEPROM_PROD_YEAR_LSB_OFFSET);
    out->prod_year_hi = byte_at(data, len, EEPROM_PROD_YEAR_MSB_OFFSET);

    // Datalogger summary
    if (len >= (EEPROM_DATALOG_SUMMARY_OFFSET + 16)) {
        const int S = EEPROM_DATALOG_SUMMARY_OFFSET;
        out->days_with_lvd = u16be(data, len, S);
        out->months_without_full_charge = byte_at(data, len, S + 2);
        out->avg_morning_soc_pct = u16be(data, len, S + 4);
        out->total_ah_charge_mah = u32be(data, len, S + 6);
        out->total_ah_load_mah = u32be(data, len, S + 10);
        out->num_days = u16be(data, len, S + 14);

        // Daily logs
        out->daily_count = 0;
        int num_daily = clamp(out->num_days, 0, EEPROM_DAILY_MAX_BLOCKS);
        int start_daily = (out->num_days - num_daily) % EEPROM_DAILY_MAX_BLOCKS;

        for (int i = 0; i < num_daily; i++) {
            int slot = (start_daily + i) % EEPROM_DAILY_MAX_BLOCKS;
            int off = EEPROM_DAILY_START_OFFSET + slot * EEPROM_DAILY_BLOCK_SIZE;
            if (off + EEPROM_DAILY_BLOCK_SIZE > len) break;

            // Skip blank
            if (byte_at(data, len, off) == 0 && byte_at(data, len, off + 1) == 0 &&
                u16be(data, len, off + 2) == 0 && u16be(data, len, off + 4) == 0)
                continue;

            EepromLogEntry *e = &out->daily_logs[out->daily_count++];
            e->index = (uint16_t)(i + 1);
            e->vbat_max_mv = byte_at(data, len, off);
            e->vbat_min_mv = byte_at(data, len, off + 1);
            e->ah_charge_mah = u16be(data, len, off + 2);
            e->ah_load_mah = u16be(data, len, off + 4);
            e->vpv_max_mv = byte_at(data, len, off + 6);
            e->vpv_min_mv = byte_at(data, len, off + 7);
            e->il_max_ma = byte_at(data, len, off + 8);
            e->ipv_max_ma = byte_at(data, len, off + 9);
            e->soc_pct = byte_at(data, len, off + 10);
            e->ext_temp_max_c = (int8_t)byte_at(data, len, off + 11);
            e->ext_temp_min_c = (int8_t)byte_at(data, len, off + 12);
            e->nightlength_min = byte_at(data, len, off + 13);
            e->state_flags = state_flags_parse(u16be(data, len, off + 14));
        }

        // Monthly logs
        out->monthly_count = 0;
        int total_months = out->num_days / 31;
        int num_monthly = clamp(total_months, 0, EEPROM_MONTHLY_MAX_BLOCKS);
        int start_monthly = (total_months - num_monthly) % EEPROM_MONTHLY_MAX_BLOCKS;

        for (int i = 0; i < num_monthly; i++) {
            int slot = (start_monthly + i) % EEPROM_MONTHLY_MAX_BLOCKS;
            int off = EEPROM_MONTHLY_START_OFFSET + slot * EEPROM_MONTHLY_BLOCK_SIZE;
            if (off + EEPROM_MONTHLY_BLOCK_SIZE > len) break;

            if (byte_at(data, len, off) == 0 && byte_at(data, len, off + 1) == 0 &&
                u16be(data, len, off + 2) == 0 && u16be(data, len, off + 4) == 0)
                continue;

            EepromLogEntry *e = &out->monthly_logs[out->monthly_count++];
            e->index = (uint16_t)(i + 1);
            e->vbat_max_mv = byte_at(data, len, off);
            e->vbat_min_mv = byte_at(data, len, off + 1);
            e->ah_charge_mah = u16be(data, len, off + 2);
            e->ah_load_mah = u16be(data, len, off + 4);
            e->vpv_max_mv = byte_at(data, len, off + 6);
            e->vpv_min_mv = byte_at(data, len, off + 7);
            e->il_max_ma = byte_at(data, len, off + 8);
            e->ipv_max_ma = byte_at(data, len, off + 9);
            e->soc_pct = byte_at(data, len, off + 10);
            e->ext_temp_max_c = (int8_t)byte_at(data, len, off + 11);
            e->ext_temp_min_c = (int8_t)byte_at(data, len, off + 12);
            e->nightlength_min = byte_at(data, len, off + 13);
            e->state_flags = state_flags_parse(u16be(data, len, off + 14));
        }
    }

    out->valid = true;
    return true;
}
