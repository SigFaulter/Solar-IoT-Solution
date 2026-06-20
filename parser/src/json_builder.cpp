#include "json_builder.h"

#include "lookups.h"
#include "utils.h"

using json = nlohmann::json;

auto build_info_json(const EepromSettings &settings, JsonScalingFormat /*scaling_format*/)
    -> nlohmann::json {
    return {
        {"serial_number", !settings.serial_number.empty() ? settings.serial_number : "Unknown"},
        {"production_date", !settings.production_date.empty() ? settings.production_date : ""},
        {"type", !settings.device_id.empty() ? settings.device_id : "Unknown"},
        {"hw_version", settings.hw_version},
    };
}

auto build_telemetry_json(const PhocosTelemetry &t,
                          const EepromSettings  &settings,
                          std::time_t            ts,
                          JsonScalingFormat      scaling_format) -> nlohmann::json {
    nlohmann::json j;

    j["general"] = {
        {"timestamp", static_cast<long long>(ts)},
        {"serial_number", settings.serial_number},
        {"firmware_version", t.firmware_version},
        {"internal_temp_c", t.internal_temp_c},
        {"external_temp_c", t.external_temp_c},
        {"controller_op_days", t.op_days},
        {"hw_version", t.hw_version},
    };

    j["battery"] = {
        {"voltage_mv", static_cast<double>(t.battery_voltage_mv)},
        {"soc_pct", t.battery_soc_pct},
        {"charge_current_ma",
         (scaling_format == JsonScalingFormat::SCALED)
             ? static_cast<double>(t.charge_current_ma10 * 10)
             : static_cast<double>(t.charge_current_ma10)},
        {"charge_power_w", t.charge_power_w},
        {"end_of_charge_voltage_mv", static_cast<double>(t.battery_threshold_mv)},
        {"charge_mode", charge_mode_to_string(charge_mode_from_state(t.charge_state_raw))},
        {"is_night", t.charge_flags.is_night},
        {"operation_days", t.bat_op_days},
        {"energy_in_daily_wh", t.energy_in_daily_wh},
        {"energy_out_daily_wh", t.energy_out_daily_wh},
        {"energy_retained_wh", t.energy_retained_wh},
        {"detected", static_cast<bool>(t.battery_detected)},
    };

    j["load"] = {
        {"load_on", t.load_flags.load_on()},
        {"night_mode", t.load_flags.night_mode_active},
        {"lvd_active", t.load_flags.lvd_active},
        {"user_disconnect", t.load_flags.user_disconnect},
        {"over_current", t.load_flags.over_current},
        {"current_ma",
         (scaling_format == JsonScalingFormat::SCALED)
             ? static_cast<double>(t.load_current_ma10 * 10)
             : static_cast<double>(t.load_current_ma10)},
        {"power_w", t.load_power_w},
    };

    j["pv"] = {
        {"voltage_mv", static_cast<double>(t.pv_voltage_mv)},
        {"target_voltage_mv", static_cast<double>(t.pv_target_mv)},
        {"detected", static_cast<bool>(t.pv_detected)},
    };

    j["night"] = {
        {"time_since_dusk_min", t.nightlength_min},
        {"average_length_min", t.avg_nightlength_min},
    };

    j["led"] = {
        {"voltage_mv", static_cast<double>(t.led_voltage_mv)},
        {"current_ma",
         (scaling_format == JsonScalingFormat::SCALED)
             ? static_cast<double>(t.led_current_ma10 * 10)
             : static_cast<double>(t.led_current_ma10)},
        {"power_w", t.led_power_w},
        {"status", led_status_name(t.led_status)},
        {"dali_active", static_cast<bool>(t.dali_active)},
    };

    // lvd_active and temperature faults are surfaced from both flag sources;
    // either flag being set is sufficient to raise the fault.
    j["faults"] = {
        {"battery_over_voltage", t.fault_flags.battery_over_voltage},
        {"pv_over_voltage", t.fault_flags.pv_over_voltage},
        {"controller_over_temp", t.fault_flags.controller_over_temp},
        {"charge_over_current", t.fault_flags.charge_over_current},
        {"lvd_active", t.load_flags.lvd_active || t.fault_flags.lvd_active},
        {"over_discharge_current", t.fault_flags.over_discharge_current},
        {"load_over_current", t.load_flags.over_current},
        {"battery_over_temp", t.fault_flags.battery_over_temp || t.load_flags.high_temp},
        {"battery_under_temp", t.fault_flags.battery_under_temp || t.load_flags.low_temp},
    };

    return j;
}

static auto log_entries_to_json(const LogEntry   *entries,
                                std::size_t       count,
                                const char       *index_key,
                                bool              is_monthly,
                                JsonScalingFormat scaling_format) -> nlohmann::json {
    nlohmann::json arr = nlohmann::json::array();

    for (std::size_t i = 0; i < count; ++i) {
        const auto &e   = entries[i];
        float       soc = is_monthly ? (static_cast<float>(e.soc_pct) * 6.6F >= 99.0F
                                            ? 100.0F
                                            : static_cast<float>(e.soc_pct) * 6.6F)
                                     : static_cast<float>(e.soc_pct);

        arr.emplace_back(nlohmann::json{
            {index_key, e.index},
            {"vbat_min_mv",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.vbat_min_mv) * 100.0
                 : static_cast<double>(e.vbat_min_mv)},
            {"vbat_max_mv",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.vbat_max_mv) * 100.0
                 : static_cast<double>(e.vbat_max_mv)},
            {"ah_charge_mah",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.ah_charge_mah) * 100.0
                 : static_cast<double>(e.ah_charge_mah)},
            {"ah_load_mah",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.ah_load_mah) * 100.0
                 : static_cast<double>(e.ah_load_mah)},
            {"vpv_max_mv",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.vpv_max_mv) * 500.0
                 : static_cast<double>(e.vpv_max_mv)},
            {"vpv_min_mv",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.vpv_min_mv) * 500.0
                 : static_cast<double>(e.vpv_min_mv)},
            {"il_max_ma",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.il_max_ma) * 500.0
                 : static_cast<double>(e.il_max_ma)},
            {"ipv_max_ma",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<double>(e.ipv_max_ma) * 500.0
                 : static_cast<double>(e.ipv_max_ma)},
            {"soc_pct",
             (scaling_format == JsonScalingFormat::SCALED) ? round1(static_cast<double>(soc))
                                                           : static_cast<double>(e.soc_pct)},
            {"ext_temp_max_c", e.ext_temp_max_c},
            {"ext_temp_min_c", e.ext_temp_min_c},
            {"nightlength_min",
             (scaling_format == JsonScalingFormat::SCALED)
                 ? static_cast<uint16_t>(e.nightlength_min) * 10
                 : static_cast<uint16_t>(e.nightlength_min)},
            {"flags",
             {
                 {"ld", e.state.load_disconnect},
                 {"fcb", e.state.full_charge},
                 {"pvoc", e.state.pv_over_current},
                 {"loc", e.state.load_over_current},
                 {"bov", e.state.battery_over_voltage},
                 {"lsoc", e.state.low_soc},
                 {"topvo", e.state.temp_over_pv_over},
                 {"topvl", e.state.temp_over_pv_low},
                 {"tolo", e.state.temp_over_load_over},
             }},
        });
    }

    return arr;
}

auto build_datalogger_json(const EepromSettings    &settings,
                           const DataloggerSummary &s,
                           const DailyLogBuffer    &days,
                           const MonthlyLogBuffer  &months,
                           std::time_t              ts,
                           JsonScalingFormat        scaling_format) -> nlohmann::json {
    return {
        {"timestamp", static_cast<long long>(ts)},
        {"eeprom",
         {
             {"battery_type", settings.battery_type},
             {"capacity_ah", settings.settings.capacity_ah},
             {"lvd_voltage_mv", static_cast<double>(settings.settings.lvd_voltage_mv)},
             {"boost_voltage_mv", static_cast<double>(settings.boost_mv)},
             {"float_voltage_mv", static_cast<double>(settings.float_mv)},
             {"equalization_voltage_mv", static_cast<double>(settings.equalization_mv)},
             {"night_mode", settings.night_mode},
             {"dimming_pct", settings.settings.dimming_pct},
             {"base_dimming_pct", settings.settings.base_dimming_pct},
             {"dali_active", settings.settings.dali_power_enable},
             {"alc_dimming", settings.settings.alc_dimming_enable},
         }},
        {"datalogger",
         {
             {"recorded_days", s.num_days},
             {"days_with_lvd", s.days_with_lvd},
             {"months_without_full_charge", s.months_without_full_charge},
             {"avg_morning_soc_pct",
              (scaling_format == JsonScalingFormat::SCALED)
                  ? (round1((s.num_days > 0) ? (static_cast<double>(s.avg_morning_soc_pct) * 6.6 /
                                                static_cast<double>(s.num_days))
                                             : 0.0))
                  : static_cast<double>(s.avg_morning_soc_pct)},
             {"total_ah_charge_ah", static_cast<double>(s.total_ah_charge)},
             {"total_ah_load_ah", static_cast<double>(s.total_ah_load)},
             {"daily",
              log_entries_to_json(days.entries.data(), days.count, "day", false, scaling_format)},
             {"monthly",
              log_entries_to_json(
                  months.entries.data(), months.count, "month", true, scaling_format)},
         }},
    };
}

auto build_settings_json(const DeviceSettings &s, std::time_t ts) -> nlohmann::json {
    return {
        {"timestamp", static_cast<long long>(ts)},
        {"battery_type_index", s.battery_type ? static_cast<int>(*s.battery_type) : -1},
        {"capacity_ah", s.capacity_ah},
        {"lvd_voltage_mv", s.lvd_voltage_mv},
        {"lvd_mode_voltage", s.lvd_mode_voltage},
        {"night_mode_index", s.night_mode_index ? static_cast<int>(*s.night_mode_index) : -1},
        {"evening_minutes", s.evening_minutes},
        {"morning_minutes", s.morning_minutes},
        {"night_threshold_mv", s.night_threshold_mv},
        {"night_mode_dimming_index",
         s.night_mode_dimming_index ? static_cast<int>(*s.night_mode_dimming_index) : -1},
        {"evening_minutes_dimming", s.evening_minutes_dimming},
        {"morning_minutes_dimming", s.morning_minutes_dimming},
        {"dimming_pct", s.dimming_pct},
        {"base_dimming_pct", s.base_dimming_pct},
        {"dali_power_enable", s.dali_power_enable},
        {"alc_dimming_enable", s.alc_dimming_enable},
    };
}
