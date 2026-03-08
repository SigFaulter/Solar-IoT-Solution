#include "json_builder.h"
#include "lookups.h"
#include "utils.h"

#include <cstdio>

using json = nlohmann::json;


nlohmann::json buildJSON(
    const PhocosHeader&            hdr,
    const PhocosTelemetry&         t,
    const EepromConfig&            cfg,
    const DataloggerSummary&       s,
    const std::vector<DailyLog>&   days,
    const std::vector<MonthlyLog>& months,
    const char*                    ts) {

    const std::string& serial    = !cfg.serial_number.empty()   ? cfg.serial_number
                                 : !hdr.serial_number.empty()   ? hdr.serial_number   : "";
    const std::string& prod_date = !cfg.production_date.empty() ? cfg.production_date
                                 : !hdr.production_date.empty() ? hdr.production_date : "";
    const std::string& dev_type  = !cfg.device_type.empty()     ? cfg.device_type
                                 : !hdr.type.empty()             ? hdr.type
                                 : "Unknown";

    json j;

    j["general"] = json{
        {"timestamp",          ts},
        {"type",               dev_type},
        {"production_date",    prod_date},
        {"serial_number",      serial},
        {"firmware_version",   t.firmware_version},
        {"internal_temp_c",    t.internal_temp_C},
        {"external_temp_c",    t.external_temp_C},
        {"controller_op_days", t.op_days},
        {"hw_version",         t.hw_version}
    };

    j["battery"] = json{
        {"type",                    cfg.battery_type},
        {"voltage_v",               r2(t.battery_voltage_mV   / 1000.0f)},
        {"soc_pct",                 t.battery_soc_pct},
        {"charge_current_a",        r2(t.charge_current_mA    / 1000.0f)},
        {"end_of_charge_voltage_v", r2(t.battery_threshold_mV / 1000.0f)},
        {"charge_mode",             chargeModeFromState(t.charge_state_raw)},
        {"is_night",                t.charge_flags.is_night},
        {"operation_days",          t.bat_op_days},
        {"energy_in_daily_wh",      t.energy_in_daily_Wh},
        {"energy_out_daily_wh",     t.energy_out_daily_Wh},
        {"energy_retained_wh",      t.energy_retained_Wh},
        {"charge_power_w",          t.charge_power_W},
        {"detected",                bool(t.battery_detected)}
    };

    j["load"] = json{
        {"load_on",         t.load_flags.load_on()},
        {"night_mode",      t.load_flags.night_mode_active},
        {"lvd_active",      t.load_flags.lvd_active},
        {"user_disconnect", t.load_flags.user_disconnect},
        {"over_current",    t.load_flags.over_current},
        {"current_a",       r2(t.load_current_mA / 1000.0f)},
        {"power_w",         t.load_power_W}
    };

    j["pv"] = json{
        {"voltage_v",        r2(t.pv_voltage_mV / 1000.0f)},
        {"target_voltage_v", r2(t.pv_target_mV  / 1000.0f)},
        {"detected",         bool(t.pv_detected)}
    };

    j["night"] = json{
        {"time_since_dusk_min", t.nightlength_min},
        {"average_length_min",  t.avg_nightlength_min}
    };

    j["led"] = json{
        {"voltage_v",   r2(t.led_voltage_mV / 1000.0f)},
        {"current_a",   r2(t.led_current_mA / 1000.0f)},
        {"power_w",     t.led_power_W},
        {"status",      ledStatusName(t.led_status)},
        {"dali_active", bool(t.dali_active)}
    };

    j["charge_status"] = json{
        {"boost",            t.charge_flags.boost_charge},
        {"equalization",     t.charge_flags.equalization_charge},
        {"ssr_output",       t.charge_flags.ssr_output},
        {"dimming_override", t.charge_flags.dimming_override}
    };

    j["faults"] = json{
        {"pv_over_voltage",      t.fault_flags.pv_over_voltage},
        {"charge_over_current",  t.fault_flags.charge_over_current},
        {"lvd_active",           t.load_flags.lvd_active},
        {"load_over_current",    t.load_flags.over_current},
        {"high_temp_protection", t.load_flags.high_temp},
        {"low_temp_protection",  t.load_flags.low_temp}
    };

    j["eeprom"] = json{
        {"battery_type",     cfg.battery_type},
        {"capacity_ah",      cfg.capacity_ah},
        {"lvd_voltage_v",    r2(cfg.lvd_voltage_mV  / 1000.0f)},
        {"boost_voltage_v",  r2(cfg.boost_mV         / 1000.0f)},
        {"float_voltage_v",  r2(cfg.float_mV         / 1000.0f)},
        {"equalization_v",   r2(cfg.equalization_mV  / 1000.0f)},
        {"night_mode",       cfg.night_mode},
        {"dimming_pct",      cfg.dimming_pct},
        {"base_dimming_pct", cfg.base_dimming_pct},
        {"dali_active",      cfg.dali_active},
        {"alc_dimming",      cfg.alc_dimming}
    };

    json daily = json::array();
    for (const auto& d : days) {
        json entry;
        entry["day"]             = d.day_index;
        entry["vbat_min_v"]      = r2(d.vbat_min_mV   / 1000.0f);
        entry["vbat_max_v"]      = r2(d.vbat_max_mV   / 1000.0f);
        entry["ah_charge"]       = r1(d.ah_charge_mAh / 1000.0f);
        entry["ah_load"]         = r1(d.ah_load_mAh   / 1000.0f);
        entry["vpv_max_v"]       = r2(d.vpv_max_mV    / 1000.0f);
        entry["vpv_min_v"]       = r2(d.vpv_min_mV    / 1000.0f);
        entry["il_max_a"]        = r2(d.il_max_mA     / 1000.0f);
        entry["ipv_max_a"]       = r2(d.ipv_max_mA    / 1000.0f);
        entry["soc_pct"]         = r1(d.soc_pct);
        entry["ext_temp_max_c"]  = d.ext_temp_max_C;
        entry["ext_temp_min_c"]  = d.ext_temp_min_C;
        entry["nightlength_min"] = d.nightlength_min;
        entry["state"]           = d.state;
        entry["flags"]           = json{
            {"ld",    d.isLd()},
            {"fcb",   d.isFcb()},
            {"pvoc",  d.isPvoc()},
            {"loc",   d.isLoc()},
            {"bov",   d.isBov()},
            {"lsoc",  d.isLsoc()},
            {"topvo", d.isTopvo()},
            {"topvl", d.isTopvl()},
            {"tolo",  d.isTolo()}
        };
        daily.push_back(entry);
    }

    json monthly = json::array();
    for (const auto& m : months) {
        json entry;
        entry["month"]           = m.month_index;
        entry["vbat_min_v"]      = r2(m.vbat_min_mV   / 1000.0f);
        entry["vbat_max_v"]      = r2(m.vbat_max_mV   / 1000.0f);
        entry["ah_charge"]       = r1(m.ah_charge_mAh / 1000.0f);
        entry["ah_load"]         = r1(m.ah_load_mAh   / 1000.0f);
        entry["vpv_max_v"]       = r2(m.vpv_max_mV    / 1000.0f);
        entry["vpv_min_v"]       = r2(m.vpv_min_mV    / 1000.0f);
        entry["il_max_a"]        = r2(m.il_max_mA     / 1000.0f);
        entry["ipv_max_a"]       = r2(m.ipv_max_mA    / 1000.0f);
        entry["soc_pct"]         = r1(m.soc_pct);
        entry["ext_temp_max_c"]  = m.ext_temp_max_C;
        entry["ext_temp_min_c"]  = m.ext_temp_min_C;
        entry["nightlength_min"] = m.nightlength_min;
        monthly.push_back(entry);
    }

    j["datalogger"] = json{
        {"recorded_days",              s.num_days},
        {"days_with_lvd",              s.days_with_lvd},
        {"months_without_full_charge", s.months_without_full_charge},
        {"avg_morning_soc_pct",        r1(s.avg_morning_soc_pct)},
        {"total_ah_charge",            r1(s.total_ah_charge)},
        {"total_ah_load",              r1(s.total_ah_load)},
        {"daily",                      daily},
        {"monthly",                    monthly}
    };

    return j;
}


// TODO: simple LoRa size check, remove in prod.
// The air link will use binary packing (fport 1-4), not raw JSON
void publishJSON(const nlohmann::json& j, const char* serial_number) {
    std::string s = j.dump(2);
    printf("\n[JSON]  (%zu bytes", s.size());

    if      (s.size() <=  51) { printf("  * fits SF12 / 51B max"); }
    else if (s.size() <= 115) { printf("  * fits SF10 / 115B max"); }
    else if (s.size() <= 222) { printf("  * fits SF7  / 222B max"); }
    else                      { printf("  ! too large for LoRa — use binary packing for the air link"); }

    printf(")\n");
    printf("  MQTT topic: mppt/%s/state\n\n", (serial_number && serial_number[0]) ? serial_number : "<serial>");
    printf("%s\n", s.c_str());
}
