#include "printer.h"
#include "lookups.h"

#include <cstdio>
#include <cstring>
#include <string>


void printSystemState(
    const PhocosTelemetry& t,
    const EepromConfig&    cfg,
    const char*            ts) {
    const std::string& dev_type  = !cfg.device_id.empty()     ? cfg.device_id  : "Unknown";
    const std::string& serial    = !cfg.serial_number.empty()   ? cfg.serial_number : "N/A";
    const std::string& prod_date = !cfg.production_date.empty() ? cfg.production_date : "N/A";
    const std::string& bat_type  = !cfg.battery_type.empty()    ? cfg.battery_type : "Unknown";

    printf("\n╔══════════════════════════════════════════╗\n");
    printf(  "║           System State                   ║\n");
    printf(  "╚══════════════════════════════════════════╝\n");

    printf("\n[General]\n");
    printf("  %-36s %s\n",     "Timestamp",                  ts);
    printf("  %-36s %s\n",     "Type",                       dev_type.c_str());
    printf("  %-36s V%d\n",    "Hardware Version",           t.hw_version);
    printf("  %-36s %s\n",     "Production Date",            prod_date.c_str());
    printf("  %-36s %s\n",     "Serial Number",              serial.c_str());
    printf("  %-36s %u\n",     "Firmware Version",           t.firmware_version);
    printf("  %-36s %+d °C\n", "Internal Temp",              t.internal_temp_C);
    printf("  %-36s %+d °C\n", "External Temp",              t.external_temp_C);
    printf("  %-36s %u Days\n","Controller Operation Time",  t.op_days);

    printf("\n[Battery]\n");
    printf("  %-36s %s\n",     "Type",                          bat_type.c_str());
    printf("  %-36s %.3f V\n", "Voltage",                       t.battery_voltage_mV   / 1000.0f);
    printf("  %-36s %u %%\n",  "SOC",                           t.battery_soc_pct);
    printf("  %-36s %.2f A\n", "Charge Current",                t.charge_current_mA    / 1000.0f);
    printf("  %-36s %.3f V\n", "End of Charge Voltage",         t.battery_threshold_mV / 1000.0f);
    printf("  %-36s %s\n",     "Charge Mode",                   chargeModeFromState(t.charge_state_raw));
    printf("  %-36s %s\n",     "Is Night",                      t.charge_flags.is_night ? "Yes" : "No");
    printf("  %-36s %u Days\n","Operation Time",                t.bat_op_days);
    printf("  %-36s %u Wh\n",  "Yesterday Total Input Energy",  t.energy_in_daily_Wh);
    printf("  %-36s %u Wh\n",  "Yesterday Total Output Energy", t.energy_out_daily_Wh);
    printf("  %-36s %u Wh\n",  "Retained Energy at Dawn",       t.energy_retained_Wh);
    printf("  %-36s %u W\n",   "Charge Power",                  t.charge_power_W);
    printf("  %-36s %s\n",     "Detected",                      t.battery_detected ? "Yes" : "No");

    printf("\n[Load]\n");
    printf("  %-36s %s\n",    "Load On",         t.load_flags.load_on()         ? "Yes" : "No");
    printf("  %-36s %s\n",    "Night Mode",      t.load_flags.night_mode_active ? "Yes" : "No");
    printf("  %-36s %s\n",    "LVD Active",      t.load_flags.lvd_active        ? "Yes" : "No");
    printf("  %-36s %s\n",    "User Disconnect", t.load_flags.user_disconnect   ? "Yes" : "No");
    printf("  %-36s %s\n",    "Over Current",    t.load_flags.over_current      ? "Yes" : "No");
    printf("  %-36s %.2f A\n","Current",         t.load_current_mA / 1000.0f);
    printf("  %-36s %u W\n",  "Power",           t.load_power_W);

    printf("\n[PV]\n");
    printf("  %-36s %.3f V\n","Voltage",   t.pv_voltage_mV / 1000.0f);
    printf("  %-36s %s\n",    "Detected",  t.pv_detected ? "Yes" : "No");

    printf("\n[Night]\n");
    printf("  %-36s %u min\n","Time Since Dusk", t.nightlength_min);
    printf("  %-36s %u min\n","Average Length",  t.avg_nightlength_min);

    printf("\n[LED]\n");
    printf("  %-36s %.3f V\n","Voltage",     t.led_voltage_mV / 1000.0f);
    printf("  %-36s %.2f A\n","Current",     t.led_current_mA / 1000.0f);
    printf("  %-36s %u W\n",  "Power",       t.led_power_W);
    printf("  %-36s %s\n",    "Status",      ledStatusName(t.led_status));
    printf("  %-36s %s\n",    "DALI Active", t.dali_active ? "Yes" : "No");

    if (t.hw_version == 2) {
        printf("  (LED, Fault, Energy fields not available on V2 firmware)\n");
    }

    printf("\n[Charge Status]\n");
    printf("  %-36s %s\n", "SSR Output", t.charge_flags.ssr_output ? "Yes" : "No");

    if (t.hw_version == 2) {
        printf("  %-36s %s\n", "Dimming Override", t.charge_flags.dimming_override ? "Yes" : "No");
    }

    printf("\n[Faults / Warnings]\n");

    bool any_fault = t.load_flags.lvd_active
                  || t.load_flags.over_current
                  || t.load_flags.high_temp
                  || t.load_flags.low_temp
                  || t.fault_flags.battery_over_voltage
                  || t.fault_flags.pv_over_voltage
                  || t.fault_flags.controller_over_temp
                  || t.fault_flags.charge_over_current
                  || t.fault_flags.over_discharge_current
                  || t.fault_flags.battery_over_temp
                  || t.fault_flags.battery_under_temp;

    if (!any_fault) {
        printf("  None\n");
    }
    else {
        if (t.fault_flags.battery_over_voltage)                         { printf("  ! Battery Over-Voltage\n"); }
        if (t.fault_flags.pv_over_voltage)                              { printf("  ! PV Over-Voltage\n"); }
        if (t.fault_flags.controller_over_temp)                         { printf("  ! Controller Over-Temperature\n"); }
        if (t.fault_flags.charge_over_current)                          { printf("  ! Charge Over-Current\n"); }
        if (t.load_flags.lvd_active)                                    { printf("  ! Low Voltage Disconnect\n"); }
        if (t.fault_flags.over_discharge_current)                       { printf("  ! Battery Over-Discharge Current\n"); }
        if (t.load_flags.over_current)                                  { printf("  ! Load Over-Current\n"); }
        if (t.fault_flags.battery_over_temp || t.load_flags.high_temp)  { printf("  ! Battery Over-Temperature\n"); }
        if (t.fault_flags.battery_under_temp || t.load_flags.low_temp)  { printf("  ! Battery Under-Temperature\n"); }
    }
}


void printEepromConfig(const EepromConfig& cfg) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf(  "║           Device Configuration           ║\n");
    printf(  "╚══════════════════════════════════════════╝\n");

    printf("\n[Identity]\n");
    printf("  %-36s %s\n",  "Device ID",            cfg.device_id.c_str());
    printf("  %-36s %s\n",  "Serial Number",   cfg.serial_number.c_str());
    printf("  %-36s %s\n",  "Production Date", cfg.production_date.c_str());

    printf("\n[Battery Configuration]\n");
    printf("  %-36s %s\n",     "Type",                  cfg.battery_type.c_str());
    printf("  %-36s %u Ah\n",  "Capacity",              cfg.settings.capacity_ah);
    printf("  %-36s %u Days\n","Battery Op. Time",      cfg.battery_op_days);
    printf("  %-36s %.3f V\n", "LVD Voltage",           cfg.settings.lvd_voltage_mv / 1000.0f);
    printf("  %-36s %s\n",     "LVD Mode",              cfg.settings.lvd_mode_voltage ? "Voltage" : "Current");
    printf("  %-36s %.3f V\n", "Equalization Voltage",  cfg.equalization_mV / 1000.0f);
    printf("  %-36s %.3f V\n", "Boost Voltage",         cfg.boost_mV        / 1000.0f);
    printf("  %-36s %.3f V\n", "Float Voltage",         cfg.float_mV        / 1000.0f);
    printf("  %-36s %.1f mV/°C\n", "Temp Compensation", cfg.temp_comp_mV_per_C);

    printf("\n[Night Mode]\n");
    printf("  %-36s %.3f V\n", "Night Threshold",   cfg.settings.night_threshold_mv / 1000.0f);
    printf("  %-36s %s\n",     "Mode",              cfg.night_mode.c_str());
    printf("  %-36s %u min\n", "Evening Duration",  cfg.settings.evening_minutes_mn);
    printf("  %-36s %u min\n", "Morning Duration",  cfg.settings.morning_minutes_mn);

    printf("\n[Dimming]\n");
    printf("  %-36s %s\n",     "Dimming Mode",          cfg.night_mode_dimming.c_str());
    printf("  %-36s %u min\n", "Evening Dim Duration",  cfg.settings.evening_minutes_dimming_mn);
    printf("  %-36s %u min\n", "Morning Dim Duration",  cfg.settings.morning_minutes_dimming_mn);
    printf("  %-36s %u %%\n",  "Dimming Level",         cfg.settings.dimming_pct);
    printf("  %-36s %u %%\n",  "Base Dimming Level",    cfg.settings.base_dimming_pct);

    printf("\n[Features]\n");
    printf("  %-36s %s\n",     "DALI Active",    cfg.settings.dali_power_enable  ? "Yes" : "No");
    printf("  %-36s %s\n",     "ALC Dimming",    cfg.settings.alc_dimming_enable ? "Yes" : "No");
    printf("  %-36s %u Days\n","Operation Time", cfg.operation_days);
}


void printDataLogger(
    const DataloggerSummary&       s,
    const std::vector<DailyLog>&   days,
    const std::vector<MonthlyLog>& months) {
    printf("\n╔══════════════════════════════════════════╗\n");
    printf(  "║           Data Logger                    ║\n");
    printf(  "╚══════════════════════════════════════════╝\n");

    printf("\n[General]\n");
    printf("  %-36s %u\n", "Recorded Days", s.num_days);

    printf("\n[Charge]\n");
    printf("  %-36s %.1f %%\n", "SOC in the Morning", s.avg_morning_soc_pct);
    printf("  %-36s %.1f Ah\n", "Charge",             s.total_ah_charge);

    printf("\n[Discharge]\n");
    printf("  %-36s %u\n",      "Days with Load Disconnect",  s.days_with_lvd);
    printf("  %-36s %u\n",      "Months Without Full Charge", s.months_without_full_charge);
    printf("  %-36s %.1f Ah\n", "Discharge",                  s.total_ah_load);

    if (!days.empty()) {
        printf("\n[Daily Data]  (%zu entries)\n", days.size());
        printf("  %-4s  %-9s %-9s %-9s %-9s %-9s %-9s %-8s %-8s %-7s  %s\n",
               "Day", "VBat Min", "VBat Max", "AhChg", "AhLoad",
               "VPv Max", "VPv Min", "IL Max", "IPv Max", "SOC%", "Flags");
        printf("  %s\n", std::string(100, '-').c_str());

        for (const auto& d : days) {
            std::string flags;
            if (d.isLd())    flags += "LD ";
            if (d.isFcb())   flags += "FCB ";
            if (d.isPvoc())  flags += "PVOC ";
            if (d.isLoc())   flags += "LOC ";
            if (d.isBov())   flags += "BOV ";
            if (d.isLsoc())  flags += "LSOC ";
            if (d.isTopvo()) flags += "TOPVO ";
            if (d.isTopvl()) flags += "TOPVL ";
            if (d.isTolo())  flags += "TOLO ";

            printf("  %-4d  %-9.1f %-9.1f %-9.1f %-9.1f %-9.1f %-9.1f %-8.1f %-8.1f %-7.1f  %s\n",
                   d.day_index,
                   d.vbat_min_mV   / 1000.0f,  d.vbat_max_mV   / 1000.0f,
                   d.ah_charge_mAh / 1000.0f,  d.ah_load_mAh   / 1000.0f,
                   d.vpv_max_mV    / 1000.0f,  d.vpv_min_mV    / 1000.0f,
                   d.il_max_mA     / 1000.0f,  d.ipv_max_mA    / 1000.0f,
                   d.soc_pct,
                   flags.c_str());
        }
    }

    if (!months.empty()) {
        printf("\n[Monthly Data]  (%zu entries)\n", months.size());
        printf("  %-6s  %-9s %-9s %-9s %-9s %-9s %-9s %-8s %-8s %-7s\n",
               "Month", "VBat Min", "VBat Max", "AhChg", "AhLoad",
               "VPv Max", "VPv Min", "IL Max", "IPv Max", "SOC%");
        printf("  %s\n", std::string(100, '-').c_str());

        for (const auto& m : months) {
            printf("  %-6d  %-9.1f %-9.1f %-9.1f %-9.1f %-9.1f %-9.1f %-8.1f %-8.1f %-7.1f\n",
                   m.month_index,
                   m.vbat_min_mV   / 1000.0f,  m.vbat_max_mV   / 1000.0f,
                   m.ah_charge_mAh / 1000.0f,  m.ah_load_mAh   / 1000.0f,
                   m.vpv_max_mV    / 1000.0f,  m.vpv_min_mV    / 1000.0f,
                   m.il_max_mA     / 1000.0f,  m.ipv_max_mA    / 1000.0f,
                   m.soc_pct);
        }
    }
}
