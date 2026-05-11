#include "space_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* ---------- Helpers ---------- */

static int fast_atoi(const char *s, int len) {
    if (len <= 0) return 0;
    int start = 0;
    bool neg = false;
    if (s[0] == '-') {
        neg = true;
        start = 1;
    } else if (s[0] == '+') {
        start = 1;
    }

    int val = 0;
    for (int i = start; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') break;
        val = val * 10 + (s[i] - '0');
    }
    return neg ? -val : val;
}

static int clamp(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ---------- Bitmask Parsers ---------- */

LoadStatusFlags load_status_parse(uint16_t v) {
    LoadStatusFlags f = {0};
    f.load_disconnected = (v & 0x001) != 0;
    f.night_mode_active = (v & 0x002) != 0;
    f.lvd_active        = (v & 0x004) != 0;
    f.user_disconnect   = (v & 0x008) != 0;
    f.low_temp          = (v & 0x010) != 0;
    f.high_temp         = (v & 0x020) != 0;
    f.over_current      = (v & 0x100) != 0;
    return f;
}

bool load_status_is_on(const LoadStatusFlags *f) {
    return !f->load_disconnected;
}

ChargeStatusFlags charge_status_parse(uint16_t v) {
    ChargeStatusFlags f = {0};
    f.boost_charge        = (v & 0x01) != 0;
    f.equalization_charge = (v & 0x02) != 0;
    f.is_night            = (v & 0x08) != 0;
    f.ssr_output          = (v & 0x80) != 0;
    return f;
}

FaultStatusFlags fault_status_parse(uint16_t v) {
    FaultStatusFlags f = {0};
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

uint32_t fault_status_to_bitmask(const FaultStatusFlags *f) {
    uint32_t v = 0;
    if (f->battery_over_voltage)   v |= (1U << 0);
    if (f->pv_over_voltage)        v |= (1U << 1);
    if (f->controller_over_temp)   v |= (1U << 2);
    if (f->charge_over_current)    v |= (1U << 3);
    if (f->lvd_active)             v |= (1U << 4);
    if (f->over_discharge_current) v |= (1U << 5);
    if (f->battery_over_temp)      v |= (1U << 6);
    if (f->battery_under_temp)     v |= (1U << 7);
    return v;
}

/* ---------- Space Line Parser ---------- */

bool parse_space_line(const char *line, SpaceTelemetry *out) {
    if (!line || !out) return false;
    memset(out, 0, sizeof(SpaceTelemetry));

    const char *start = line;
    while (*start && (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n')) {
        start++;
    }
    if (!*start) return false;

    int semis = 0;
    for (const char *c = start; *c; c++) {
        if (*c == ';') semis++;
    }

    if (semis == EXPECTED_FIELDS_V2) out->hw_version = 2;
    else if (semis == EXPECTED_FIELDS_V3) out->hw_version = 3;
    else return false;

    int expected = semis;
    int field_idx = 0;
    const char *pos = start;

    while (*pos && field_idx < expected) {
        const char *semi = strchr(pos, ';');
        if (!semi) semi = pos + strlen(pos);

        int field_len = (int)(semi - pos);
        if (field_len > 0) {
            int V = fast_atoi(pos, field_len);

            switch (field_idx + 1) {
                case FIELD_CHARGE_CURRENT_MA:  out->charge_current_ma10 = (uint32_t)V; break;
                case FIELD_LOAD_CURRENT_MA:    out->load_current_ma10   = (uint32_t)V; break;
                case FIELD_PV_VOLTAGE_MV:      out->pv_voltage_mv       = (uint32_t)V; break;
                case FIELD_PV_TARGET_MV:       out->pv_target_mv        = (uint32_t)V; break;
                case FIELD_PWM_COUNTS:         out->pwm_counts          = (uint16_t)V; break;
                case FIELD_FIRMWARE_VERSION:   out->firmware_version    = (uint32_t)V; break;
                case FIELD_LOAD_STATE_RAW:
                    out->load_state_raw = (uint16_t)V;
                    out->load_flags = load_status_parse(out->load_state_raw);
                    break;
                case FIELD_CHARGE_STATE_RAW:
                    out->charge_state_raw = (uint16_t)V;
                    out->charge_flags = charge_status_parse(out->charge_state_raw);
                    break;
                case FIELD_BATTERY_VOLTAGE_MV: out->battery_voltage_mv  = (uint32_t)V; break;
                case FIELD_BAT_THRESHOLD_MV:   out->bat_threshold_mv    = (uint32_t)V; break;
                case FIELD_BATTERY_SOC_PCT:    out->battery_soc_pct     = (uint8_t)clamp(V, 0, 100); break;
                case FIELD_INTERNAL_TEMP_C:    out->internal_temp_c     = (int16_t)V;  break;
                case FIELD_EXTERNAL_TEMP_C:    out->external_temp_c     = (int16_t)V;  break;
                case FIELD_NIGHTLENGTH_MIN:    out->nightlength_min     = (uint16_t)V; break;
                case FIELD_AVG_NIGHTLENGTH:    out->avg_nightlength_min = (uint16_t)V; break;
                case FIELD_LED_VOLTAGE_MV:     out->led_voltage_mv      = (uint32_t)V; break;
                case FIELD_LED_CURRENT_MA:     out->led_current_ma10    = (uint32_t)V; break;
                case FIELD_LED_STATUS:         out->led_status          = (uint8_t)V;  break;
                case FIELD_DALI_ACTIVE:        out->dali_active         = (uint8_t)V;  break;
                case FIELD_OP_DAYS:            out->op_days             = (uint16_t)V; break;
                case FIELD_BAT_OP_DAYS:        out->bat_op_days         = (uint16_t)V; break;
                case FIELD_ENERGY_IN_WH:       out->energy_in_wh        = (uint16_t)V; break;
                case FIELD_ENERGY_OUT_WH:      out->energy_out_wh       = (uint16_t)V; break;
                case FIELD_ENERGY_RETAINED_WH: out->energy_retained_wh  = (uint16_t)V; break;
                case FIELD_CHARGE_POWER_W:     out->charge_power_w      = (uint16_t)V; break;
                case FIELD_LOAD_POWER_W:       out->load_power_w        = (uint16_t)V; break;
                case FIELD_LED_POWER_W:        out->led_power_w         = (uint16_t)V; break;
                case FIELD_FAULT_STATUS:
                    if (out->hw_version == 3) {
                        out->fault_status = (uint16_t)V;
                        out->fault_flags = fault_status_parse(out->fault_status);
                    }
                    break;
                case FIELD_PV_DETECTED:
                    if (out->hw_version == 3) out->pv_detected = (uint8_t)V;
                    break;
                case FIELD_BATTERY_DETECTED:
                    if (out->hw_version == 3) out->battery_detected = (uint8_t)V;
                    break;
            }
        }

        if (*semi == '\0') break;
        pos = semi + 1;
        field_idx++;
    }

    return field_idx >= expected && out->battery_voltage_mv != 0;
}
