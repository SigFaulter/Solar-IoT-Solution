#include "json_builder.h"
#include "lookups.h"
#include "utils.h"

#include <cstdio>

using json = nlohmann::json;


nlohmann::json buildTelemetryJSON(
    const PhocosTelemetry& t,
    const EepromConfig&    cfg,
    const char*            ts) {
    const std::string& serial    = !cfg.serial_number.empty()   ? cfg.serial_number   : "Unknown";
    const std::string& prod_date = !cfg.production_date.empty() ? cfg.production_date : "";
    const std::string& dev_type  = !cfg.device_id.empty()     ? cfg.device_id     : "Unknown";

    json j;

    j["general"] = {
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

    j["battery"] = {
        {"type",                    cfg.battery_type},
        {"voltage_v",               r2(t.battery_voltage_mV   / 1000.0f)},
        {"soc_pct",                 t.battery_soc_pct},
        {"charge_current_a",        r2(t.charge_current_mA    / 1000.0f)},
        {"charge_power_w",          t.charge_power_W},
        {"end_of_charge_voltage_v", r2(t.battery_threshold_mV / 1000.0f)},
        {"charge_mode",             chargeModeFromState(t.charge_state_raw)},
        {"is_night",                t.charge_flags.is_night},
        {"operation_days",          t.bat_op_days},
        {"energy_in_daily_wh",      t.energy_in_daily_Wh},
        {"energy_out_daily_wh",     t.energy_out_daily_Wh},
        {"energy_retained_wh",      t.energy_retained_Wh},
        {"detected",                bool(t.battery_detected)}
    };

    j["load"] = {
        {"load_on",         t.load_flags.load_on()},
        {"night_mode",      t.load_flags.night_mode_active},
        {"lvd_active",      t.load_flags.lvd_active},
        {"user_disconnect", t.load_flags.user_disconnect},
        {"over_current",    t.load_flags.over_current},
        {"current_a",       r2(t.load_current_mA / 1000.0f)},
        {"power_w",         t.load_power_W}
    };

    j["pv"] = {
        {"voltage_v",        r2(t.pv_voltage_mV / 1000.0f)},
        {"target_voltage_v", r2(t.pv_target_mV  / 1000.0f)},
        {"detected",         bool(t.pv_detected)}
    };

    j["night"] = {
        {"time_since_dusk_min", t.nightlength_min},
        {"average_length_min",  t.avg_nightlength_min}
    };

    j["led"] = {
        {"voltage_v",   r2(t.led_voltage_mV / 1000.0f)},
        {"current_a",   r2(t.led_current_mA / 1000.0f)},
        {"power_w",     t.led_power_W},
        {"status",      ledStatusName(t.led_status)},
        {"dali_active", bool(t.dali_active)}
    };

    j["faults"] = {
        {"battery_over_voltage",   t.fault_flags.battery_over_voltage},
        {"pv_over_voltage",        t.fault_flags.pv_over_voltage},
        {"controller_over_temp",   t.fault_flags.controller_over_temp},
        {"charge_over_current",    t.fault_flags.charge_over_current},
        {"lvd_active",             t.load_flags.lvd_active || t.fault_flags.lvd_active},
        {"over_discharge_current", t.fault_flags.over_discharge_current},
        {"load_over_current",      t.load_flags.over_current},
        {"battery_over_temp",      t.fault_flags.battery_over_temp || t.load_flags.high_temp},
        {"battery_under_temp",     t.fault_flags.battery_under_temp || t.load_flags.low_temp}
    };

    return j;
}


nlohmann::json buildDataloggerJSON(
    const EepromConfig&             cfg,
    const DataloggerSummary&        s,
    const std::vector<DailyLog>&    days,
    const std::vector<MonthlyLog>&  months,
    const char*                     ts) {
    const std::string& serial = !cfg.serial_number.empty() ? cfg.serial_number : "Unknown";

    json daily = json::array();
    for (const auto& d : days) {
        daily.push_back({
            {"day",             d.day_index},
            {"vbat_min_v",      r2(d.vbat_min_mV   / 1000.0f)},
            {"vbat_max_v",      r2(d.vbat_max_mV   / 1000.0f)},
            {"ah_charge",       r1(d.ah_charge_mAh / 1000.0f)},
            {"ah_load",         r1(d.ah_load_mAh   / 1000.0f)},
            {"vpv_max_v",       r2(d.vpv_max_mV    / 1000.0f)},
            {"vpv_min_v",       r2(d.vpv_min_mV    / 1000.0f)},
            {"il_max_a",        r2(d.il_max_mA     / 1000.0f)},
            {"ipv_max_a",       r2(d.ipv_max_mA    / 1000.0f)},
            {"soc_pct",         r1(d.soc_pct)},
            {"ext_temp_max_c",  d.ext_temp_max_C},
            {"ext_temp_min_c",  d.ext_temp_min_C},
            {"nightlength_min", d.nightlength_min},
            {"state",           d.state},
            {"flags", {
                {"ld",    d.isLd()},
                {"fcb",   d.isFcb()},
                {"pvoc",  d.isPvoc()},
                {"loc",   d.isLoc()},
                {"bov",   d.isBov()},
                {"lsoc",  d.isLsoc()},
                {"topvo", d.isTopvo()},
                {"topvl", d.isTopvl()},
                {"tolo",  d.isTolo()}
            }}
        });
    }

    json monthly = json::array();
    for (const auto& m : months) {
        monthly.push_back({
            {"month",           m.month_index},
            {"vbat_min_v",      r2(m.vbat_min_mV   / 1000.0f)},
            {"vbat_max_v",      r2(m.vbat_max_mV   / 1000.0f)},
            {"ah_charge",       r1(m.ah_charge_mAh / 1000.0f)},
            {"ah_load",         r1(m.ah_load_mAh   / 1000.0f)},
            {"vpv_max_v",       r2(m.vpv_max_mV    / 1000.0f)},
            {"vpv_min_v",       r2(m.vpv_min_mV    / 1000.0f)},
            {"il_max_a",        r2(m.il_max_mA     / 1000.0f)},
            {"ipv_max_a",       r2(m.ipv_max_mA    / 1000.0f)},
            {"soc_pct",         r1(m.soc_pct)},
            {"ext_temp_max_c",  m.ext_temp_max_C},
            {"ext_temp_min_c",  m.ext_temp_min_C},
            {"nightlength_min", m.nightlength_min}
        });
    }

    return {
        {"timestamp",                  ts},
        {"serial_number",              serial},
        {"eeprom", {
            {"battery_type",     cfg.battery_type},
            {"capacity_ah",      cfg.settings.capacity_ah},
            {"lvd_voltage_v",    r2(cfg.settings.lvd_voltage_mv / 1000.0f)},
            {"boost_voltage_v",  r2(cfg.boost_mV        / 1000.0f)},
            {"float_voltage_v",  r2(cfg.float_mV        / 1000.0f)},
            {"equalization_v",   r2(cfg.equalization_mV / 1000.0f)},
            {"night_mode",       cfg.night_mode},
            {"dimming_pct",      cfg.settings.dimming_pct},
            {"base_dimming_pct", cfg.settings.base_dimming_pct},
            {"dali_active",      cfg.settings.dali_power_enable},
            {"alc_dimming",      cfg.settings.alc_dimming_enable}
        }},
        {"datalogger", {
            {"recorded_days",              s.num_days},
            {"days_with_lvd",              s.days_with_lvd},
            {"months_without_full_charge", s.months_without_full_charge},
            {"avg_morning_soc_pct",        r1(s.avg_morning_soc_pct)},
            {"total_ah_charge",            r1(s.total_ah_charge)},
            {"total_ah_load",              r1(s.total_ah_load)},
            {"daily",                      daily},
            {"monthly",                    monthly}
        }}
    };
}


nlohmann::json buildSettingsJSON(const DeviceSettings& s, const char* ts) {
    return {
        {"timestamp",               ts},
        {"battery_type_index",      s.battery_type_index},
        {"capacity_ah",             s.capacity_ah},
        {"lvd_voltage_mv",          s.lvd_voltage_mv},
        {"lvd_mode_voltage",        s.lvd_mode_voltage},
        {"night_mode_index",        s.night_mode_index},
        {"evening_minutes_mn",      s.evening_minutes_mn},
        {"morning_minutes_mn",      s.morning_minutes_mn},
        {"night_threshold_mv",      s.night_threshold_mv},
        {"night_mode_dimming_index",s.night_mode_dimming_index},
        {"evening_minutes_dimming", s.evening_minutes_dimming_mn},
        {"morning_minutes_dimming", s.morning_minutes_dimming_mn},
        {"dimming_pct",             s.dimming_pct},
        {"base_dimming_pct",        s.base_dimming_pct},
        {"dali_power_enable",       s.dali_power_enable},
        {"alc_dimming_enable",      s.alc_dimming_enable}
    };
}
