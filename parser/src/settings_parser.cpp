#include "settings_parser.h"
#include "constants.h"
#include "utils.h"
#include "lookups.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

static constexpr int SETTINGS_BLOB_MIN = 99;

DeviceSettings deviceSettingsFromEeprom(const EepromConfig& cfg, int load_state) {
    DeviceSettings s = cfg.settings;
    s.hw_version = cfg.hw_version;

    if (load_state >= 0)
        s.load_disconnect_mode = (load_state & 0x4) != 0;

    return s;
}


bool applySettingsPatch(const std::string& kv, DeviceSettings& s) {
    size_t eq = kv.find('=');
    if (eq == std::string::npos) {
        fprintf(stderr, "[settings] bad argument '%s' - expected key=value\n", kv.c_str());
        return false;
    }
    const std::string key = kv.substr(0, eq);
    int val = 0;
    try {
        val = std::stoi(kv.substr(eq + 1));
    } catch (...) {
        fprintf(stderr, "[settings] bad value for '%s'\n", key.c_str());
        return false;
    }

    if      (key == "battery_type")     s.battery_type_index         = std::clamp(val, 0, 2);
    else if (key == "capacity_ah")      s.capacity_ah                = std::clamp(val, 1, 500);
    else if (key == "lvd_mv") {
        if (s.lvd_mode_voltage) s.lvd_level_voltage_mv = std::clamp(val, 5000, 15000);
        else                    s.lvd_level_current_mv = std::clamp(val, 5000, 15000);
        s.lvd_voltage_mv = std::clamp(val, 5000, 15000);
    }
    else if (key == "night_thresh_mv")  s.night_threshold_mv         = std::clamp(val, 4000, 14000);
    else if (key == "night_mode")       s.night_mode_index           = std::clamp(val, 0, 3);
    else if (key == "evening_min")      s.evening_minutes_mn         = std::clamp(val, 0, 600);
    else if (key == "morning_min")      s.morning_minutes_mn         = std::clamp(val, 0, 600);
    else if (key == "dim_mode")         s.night_mode_dimming_index   = std::clamp(val, 0, 3);
    else if (key == "dim_evening_min")  s.evening_minutes_dimming_mn = std::clamp(val, 0, 600);
    else if (key == "dim_morning_min")  s.morning_minutes_dimming_mn = std::clamp(val, 0, 600);
    else if (key == "dimming_pct")      s.dimming_pct                = std::clamp(val, 0, 100);
    else if (key == "base_dimming_pct") s.base_dimming_pct           = std::clamp(val, 0, 100);
    else if (key == "dali")             s.dali_power_enable          = (val != 0);
    else if (key == "alc")              s.alc_dimming_enable         = (val != 0);
    else {
        fprintf(stderr, "[settings] unknown key '%s'\n", key.c_str());
        return false;
    }
    fprintf(stderr, "[settings] patch: %s = %d\n", key.c_str(), val);
    return true;
}



void patchSettingsBytes(const std::vector<uint8_t>& current_bytes,
                        const DeviceSettings&       s,
                        std::vector<uint8_t>&       out_bytes) {
    out_bytes = current_bytes;
    if (out_bytes.size() < static_cast<size_t>(SETTINGS_BLOB_MIN))
        out_bytes.resize(SETTINGS_BLOB_MIN, 0u);

    out_bytes[W(SR_BATTERY_TYPE_IDX)] = static_cast<uint8_t>(std::clamp(s.battery_type_index, 0, 2));
    put_u16(out_bytes, W(SR_CAPACITY_AH),      std::clamp(s.capacity_ah,           0, 0xFFFF));

    out_bytes[W(SR_LVD_MODE_VOLTAGE)] = s.lvd_mode_voltage ? 1u : 0u;
    put_u16(out_bytes, W(SR_LVD_CURRENT_MV),   std::clamp(s.lvd_level_current_mv,  0, 0xFFFF));
    put_u16(out_bytes, W(SR_LVD_VOLTAGE_MV),   std::clamp(s.lvd_level_voltage_mv,  0, 0xFFFF));

    out_bytes[W(SR_NIGHT_MODE_IDX)] = static_cast<uint8_t>(std::clamp(s.night_mode_index, 0, 3));
    put_u16(out_bytes, W(SR_EVENING_MIN),       std::clamp(s.evening_minutes_mn,    0, 0xFFFF));
    put_u16(out_bytes, W(SR_MORNING_MIN),       std::clamp(s.morning_minutes_mn,    0, 0xFFFF));
    put_u16(out_bytes, W(SR_NIGHT_THRESH_MV),   std::clamp(s.night_threshold_mv,    0, 0xFFFF));

    out_bytes[W(SR_DIM_MODE_IDX)] = static_cast<uint8_t>(std::clamp(s.night_mode_dimming_index, 0, 3));
    put_u16(out_bytes, W(SR_DIM_EVENING_MIN),   std::clamp(s.evening_minutes_dimming_mn, 0, 0xFFFF));
    put_u16(out_bytes, W(SR_DIM_MORNING_MIN),   std::clamp(s.morning_minutes_dimming_mn, 0, 0xFFFF));
    out_bytes[W(SR_DIMMING_PCT)]      = static_cast<uint8_t>(std::clamp(s.dimming_pct,      0, 100));
    out_bytes[W(SR_BASE_DIMMING_PCT)] = static_cast<uint8_t>(std::clamp(s.base_dimming_pct, 0, 100));

    out_bytes[W(SR_DALI_ENABLE)] = s.dali_power_enable  ? 1u : 0u;
    out_bytes[W(SR_ALC_DIMMING)] = s.alc_dimming_enable ? 1u : 0u;
}


void printSettings(const DeviceSettings& s) {
    static const char* night_modes[] = {"Off", "Dusk-to-Dawn", "Dusk-and-Dawn", "Mid-Night"};
    static const char* v2_batt[]     = {"AGM/Gel", "Liquid", "LiFePO4"};
    static const char* v3_batt[]     = {"LFP Hi-Temp", "LFP Med-Temp", "LFP Lo-Temp"};
    const char** batt_names = (s.hw_version == 2) ? v2_batt : v3_batt;

    printf("\n-- Device Settings (V%d) ----------------------------------\n", s.hw_version);
    printf("  Battery type     : %s (index %d)\n",
           batt_names[std::clamp(s.battery_type_index, 0, 2)], s.battery_type_index);
    printf("  Capacity         : %d Ah\n",     s.capacity_ah);
    printf("  LVD mode         : %s\n",         s.lvd_mode_voltage ? "voltage" : "current");
    printf("  LVD voltage      : %.3f V  (%d mV)\n",
           s.lvd_voltage_mv / 1000.0, s.lvd_voltage_mv);
    printf("  LVD current reg  : %.3f V  LVD voltage reg: %.3f V\n",
           s.lvd_level_current_mv / 1000.0, s.lvd_level_voltage_mv / 1000.0);
    printf("  Night threshold  : %.3f V\n",    s.night_threshold_mv / 1000.0);
    printf("  Night mode       : %s (index %d)\n",
           night_modes[std::clamp(s.night_mode_index, 0, 3)], s.night_mode_index);
    printf("  Evening mins     : %d min\n",    s.evening_minutes_mn);
    printf("  Morning mins     : %d min\n",    s.morning_minutes_mn);
    printf("  Dim mode         : %s (index %d)\n",
           night_modes[std::clamp(s.night_mode_dimming_index, 0, 3)], s.night_mode_dimming_index);
    printf("  Dim evening mins : %d min\n",    s.evening_minutes_dimming_mn);
    printf("  Dim morning mins : %d min\n",    s.morning_minutes_dimming_mn);
    printf("  Dimming          : %d%%\n",      s.dimming_pct);
    printf("  Base dimming     : %d%%\n",      s.base_dimming_pct);
    printf("  DALI             : %s\n",         s.dali_power_enable  ? "enabled" : "disabled");
    printf("  ALC dimming      : %s\n",         s.alc_dimming_enable ? "enabled" : "disabled");
    printf("  Load disconnect  : %s\n",         s.load_disconnect_mode ? "yes" : "no");
    printf("----------------------------------------------------------\n\n");
}
