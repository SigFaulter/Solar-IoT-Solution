#include "settings_parser.h"
#include <stdio.h>
#include <string.h>

void build_write_commands(const mppt_DeviceSettings *s, uint8_t hw_version, char cmds[][16], int *count) {
    int c = 0;
    
    // Helper to add command
    #define ADD_CMD(fmt, ...) snprintf(cmds[c++], 16, fmt, ##__VA_ARGS__)

    ADD_CMD("&GAA3C00"); // 1. Unlock

    if (s->has_lvd_mode) {
        ADD_CMD("&H1B%02X00", (unsigned int)((s->lvd_mode == mppt_LvdMode_LVD_MODE_VOLTAGE) ? 1 : 0));
    }
    
    if (s->has_battery_type) {
        // V2: AGM=0, Liquid=1, LFP=2 -> Proto: 1, 2, 3
        // V3: LFP Hi=0, Med=1, Lo=2 -> Proto: 4, 5, 6
        int idx = (hw_version == 2) ? (s->battery_type - 1) : (s->battery_type - 4);
        if (idx < 0) idx = 0;
        ADD_CMD("&H1C%02X00", (unsigned int)(idx & 0xFF));
    }

    if (hw_version == 3 && s->has_advanced_flags) {
        ADD_CMD("&H3F%02X00", (unsigned int)((s->advanced_flags & 1) ? 1 : 0)); // DALI
        ADD_CMD("&H40%02X00", (unsigned int)((s->advanced_flags & 2) ? 1 : 0)); // ALC
    }

    if (s->has_night_mode) ADD_CMD("&H00%02X00", (unsigned int)(s->night_mode & 0xFF));
    if (s->has_dimming_mode) ADD_CMD("&H09%02X00", (unsigned int)(s->dimming_mode & 0xFF));
    if (s->has_dimming_pct) ADD_CMD("&H23%02X00", (unsigned int)(s->dimming_pct & 0xFF));
    if (s->has_base_dimming_pct) {
        int lo = (hw_version == 2) ? 15 : 10;
        int hi = (hw_version == 2) ? 70 : 87;
        int clamped = s->base_dimming_pct;
        if (clamped < lo) clamped = lo;
        if (clamped > hi) clamped = hi;
        ADD_CMD("&H24%02X00", (unsigned int)(clamped & 0xFF));
    }

    if (s->has_lvd_voltage_mv) {
        const char *reg = (s->has_lvd_mode && s->lvd_mode == mppt_LvdMode_LVD_MODE_VOLTAGE) ? "07" : "05";
        uint16_t v = s->lvd_voltage_mv;
        ADD_CMD("&M%s%02X%02X", reg, (unsigned int)(v & 0xFF), (unsigned int)((v >> 8) & 0xFF));
    }

    if (s->has_capacity_ah) ADD_CMD("&M3D%02X%02X", (unsigned int)(s->capacity_ah & 0xFF), (unsigned int)((s->capacity_ah >> 8) & 0xFF));
    if (s->has_night_threshold_mv) ADD_CMD("&M21%02X%02X", (unsigned int)(s->night_threshold_mv & 0xFF), (unsigned int)((s->night_threshold_mv >> 8) & 0xFF));

    if (s->has_evening_minutes) {
        uint16_t m = (s->has_night_mode && s->night_mode <= 1) ? 0 : s->evening_minutes;
        ADD_CMD("&M01%02X%02X", (unsigned int)(m & 0xFF), (unsigned int)((m >> 8) & 0xFF));
    }
    if (s->has_morning_minutes) {
        uint16_t m = (s->has_night_mode && s->night_mode <= 1) ? 0 : s->morning_minutes;
        ADD_CMD("&M03%02X%02X", (unsigned int)(m & 0xFF), (unsigned int)((m >> 8) & 0xFF));
    }
    if (s->has_evening_minutes_dimming) ADD_CMD("&M0A%02X%02X", (unsigned int)(s->evening_minutes_dimming & 0xFF), (unsigned int)((s->evening_minutes_dimming >> 8) & 0xFF));
    if (s->has_morning_minutes_dimming) ADD_CMD("&M0C%02X%02X", (unsigned int)(s->morning_minutes_dimming & 0xFF), (unsigned int)((s->morning_minutes_dimming >> 8) & 0xFF));

    *count = c;
    #undef ADD_CMD
}
