#include "eeprom_parser.h"
#include "constants.h"
#include "utils.h"
#include "lookups.h"

#include <algorithm>
#include <cstring>
#include <cstdio>


// Reads all configuration fields from a parsed byte vector into an EepromConfig.
static bool parseEepromConfig(const std::vector<uint8_t>& data, EepromConfig& cfg) {
    if (static_cast<int> (data.size()) < 144) {
        return false;
    }

    // Device type, byte 120 is the source for V2 vs V3
    uint8_t type_byte = b(data, EEPROM_DEVICE_ID_OFFSET);
    cfg.hw_version = (type_byte == 0x52) ? 2 : 3;

    const char* di = (type_byte == 0x52) ? "Solar Smart Controller V2"
                   : (type_byte == 0x56) ? "Solar Smart Controller V3"
                   : "Unknown";
    cfg.device_id = di;

    // Serial number: bytes 113 (MSB), 112 (LSB), 118 (manuf. month), all BCD
    // Format: MMSS-MM  (e.g. "0166-18")
    {
        int sm = bcd2dec(b(data, EEPROM_SERIAL_MSB_OFFSET));
        int sl = bcd2dec(b(data, EEPROM_SERIAL_LSB_OFFSET));
        int mm = bcd2dec(b(data, EEPROM_MANUF_MONTH_OFFSET));
        char serial_buf[16];
        snprintf(serial_buf, sizeof(serial_buf), "%02d%02d-%02d", sm, sl, mm);
        cfg.serial_number = serial_buf;
    }

    // Production date: bytes 117, 116, 115, 114 (year MSB, year LSB, month, day), all BCD
    {
        int day = bcd2dec(b(data, EEPROM_PROD_DAY_OFFSET));
        int mon = bcd2dec(b(data, EEPROM_PROD_MONTH_OFFSET));
        int yl  = bcd2dec(b(data, EEPROM_PROD_YEAR_LSB_OFFSET));
        int ym  = bcd2dec(b(data, EEPROM_PROD_YEAR_MSB_OFFSET));
        char date_buf[16];
        snprintf(date_buf, sizeof(date_buf), "%02d%02d-%02d-%02d", ym, yl, mon, day);
        cfg.production_date = date_buf;
    }

    // Battery type, names are controller-generation dependent (V2 vs V3 have different battery types)
    cfg.settings.battery_type_index = std::clamp((int)b(data, EEPROM_BATTERY_TYPE_OFFSET), 0, 2);
    cfg.battery_type = batteryTypeName(cfg.settings.battery_type_index, cfg.hw_version);

    cfg.settings.capacity_ah = get_u16(data, EEPROM_CAPACITY_AH_OFFSET);
    cfg.battery_op_days      = get_u16(data, EEPROM_BAT_OP_DAYS_OFFSET);

    // LVD mode:
    bool lvd_mode_raw             = (b(data, EEPROM_LVD_MODE_OFFSET) == 1);
    cfg.settings.lvd_mode_voltage = isVoltageLvdMode(cfg.hw_version, cfg.settings.battery_type_index, lvd_mode_raw);

    int lvd_cur = get_u16(data, EEPROM_LVD_CURRENT_MV_OFFSET);
    int lvd_vol = get_u16(data, EEPROM_LVD_VOLTAGE_MV_OFFSET);
    cfg.settings.lvd_level_current_mv = lvd_cur;
    cfg.settings.lvd_level_voltage_mv = lvd_vol;
    cfg.settings.lvd_voltage_mv       = cfg.settings.lvd_mode_voltage ? lvd_vol : lvd_cur;

    cfg.equalization_mV    = get_u16(data, EEPROM_EQUALIZATION_MV_OFFSET);
    cfg.boost_mV           = get_u16(data, EEPROM_BOOST_MV_OFFSET);
    cfg.float_mV           = get_u16(data, EEPROM_FLOAT_MV_OFFSET);
    cfg.temp_comp_mV_per_C = static_cast<int8_t>(b(data, EEPROM_TEMP_COMP_OFFSET));
    cfg.settings.night_threshold_mv = get_u16(data, EEPROM_NIGHT_THRESH_MV_OFFSET);
    cfg.operation_days     = get_u16(data, EEPROM_OPERATION_DAYS_OFFSET);

    cfg.settings.dali_power_enable  = (b(data, EEPROM_DALI_FLAG_OFFSET) == 1);
    cfg.settings.alc_dimming_enable = (b(data, EEPROM_ALC_FLAG_OFFSET)  == 1);

    cfg.settings.night_mode_index   = std::clamp((int)b(data, EEPROM_NIGHT_MODE_OFFSET), 0, 3);
    cfg.night_mode = nightModeName(cfg.settings.night_mode_index);
    cfg.settings.evening_minutes_mn = get_u16(data, EEPROM_EVENING_MINUTES_OFFSET);
    cfg.settings.morning_minutes_mn = get_u16(data, EEPROM_MORNING_MINUTES_OFFSET);

    cfg.settings.night_mode_dimming_index = std::clamp((int)b(data, EEPROM_DIM_MODE_OFFSET), 0, 3);
    cfg.night_mode_dimming = nightModeName(cfg.settings.night_mode_dimming_index);
    cfg.settings.evening_minutes_dimming_mn = get_u16(data, EEPROM_DIM_EVENING_OFFSET);
    cfg.settings.morning_minutes_dimming_mn = get_u16(data, EEPROM_DIM_MORNING_OFFSET);
    cfg.settings.dimming_pct      = b(data, EEPROM_DIMMING_PCT_OFFSET);
    cfg.settings.base_dimming_pct = b(data, EEPROM_BASE_DIMMING_PCT_OFFSET);

    return true;
}


// Reads the daily circular buffer into daily_logs.
//
// The buffer holds up to 30 entries
// The correct chronological order requires computing the start index from the recorded day count
static void parseDailyLogs(
    const std::vector<uint8_t>& data,
    uint16_t                    num_days,
    std::vector<DailyLog>&      daily_logs) {
    daily_logs.clear();

    const int num_entries = std::min((int)num_days, EEPROM_DAILY_MAX_BLOCKS);
    if (num_entries <= 0) {
        return;
    }

    const int start_buf = ((int)num_days - num_entries) % EEPROM_DAILY_MAX_BLOCKS;

    for (int i = 0; i < num_entries; ++i) {
        int buf_idx = (start_buf + i) % EEPROM_DAILY_MAX_BLOCKS;
        int o       = EEPROM_DAILY_START_OFFSET + buf_idx * EEPROM_DAILY_BLOCK_SIZE;

        if (static_cast<size_t>(o + EEPROM_DAILY_BLOCK_SIZE) > data.size()) {
            continue;
        }

        // Skip blocks with no meaningful data
        bool blank = (b(data, o) == 0 && b(data, o + 1) == 0 && get_u16(data, o+2) == 0 && get_u16(data, o+4) == 0);
        if (blank) {
            continue;
        }

        float soc_raw = b(data, o + 10) * 6.6f;

        DailyLog day;
        day.day_index       = i + 1;
        day.vbat_max_mV     = static_cast<uint16_t>(b(data, o)     * 100);
        day.vbat_min_mV     = static_cast<uint16_t>(b(data, o + 1) * 100);
        day.ah_charge_mAh   = static_cast<uint32_t>(get_u16(data, o + 2) * 100);
        day.ah_load_mAh     = static_cast<uint32_t>(get_u16(data, o + 4) * 100);
        day.vpv_max_mV      = static_cast<uint16_t>(b(data, o + 6) * 500);
        day.vpv_min_mV      = static_cast<uint16_t>(b(data, o + 7) * 500);
        day.il_max_mA       = static_cast<uint16_t>(b(data, o + 8) * 500);
        day.ipv_max_mA      = static_cast<uint16_t>(b(data, o + 9) * 500);
        day.soc_pct         = (soc_raw >= 99.0f) ? 100.0f : soc_raw;
        day.ext_temp_max_C  = static_cast<int8_t>(b(data, o + 11));
        day.ext_temp_min_C  = static_cast<int8_t>(b(data, o + 12));
        day.nightlength_min = static_cast<uint16_t>(b(data, o + 13) * 10);
        day.state           = static_cast<uint16_t>(get_u16(data, o + 14));

        daily_logs.push_back(day);
    }
}


// Reads the monthly circular buffer into monthly_logs.
//
// Offset 624, up to 24 entries, same 16-byte block structure as daily
// Monthly Ah values are raw get_u16 (no /10 division unlike daily)
static void parseMonthlyLogs(
    const std::vector<uint8_t>& data,
    uint16_t num_days,
    std::vector<MonthlyLog>& monthly_logs) {
    monthly_logs.clear();

    if (data.size() < static_cast<size_t>(EEPROM_MONTHLY_START_OFFSET + EEPROM_MONTHLY_BLOCK_SIZE)) {
        return;
    }

    const int total_months = num_days / 30;
    const int num_monthly  = std::min(total_months, EEPROM_MONTHLY_MAX_BLOCKS);

    if (num_monthly <= 0) {
        return;
    }

    const int start_mbuf = (total_months - num_monthly) % EEPROM_MONTHLY_MAX_BLOCKS;

    for (int i = 0; i < num_monthly; ++i) {
        int buf_idx = (start_mbuf + i) % EEPROM_MONTHLY_MAX_BLOCKS;
        int o       = EEPROM_MONTHLY_START_OFFSET + buf_idx * EEPROM_MONTHLY_BLOCK_SIZE;

        if (static_cast<size_t>(o + EEPROM_MONTHLY_BLOCK_SIZE) > data.size()) {
            continue;
        }

        if (b(data, o) == 0 && get_u16(data, o + 2) == 0) {
            continue;
        }

        float soc_raw = b(data, o + 10) * 6.6f;

        MonthlyLog mo;
        mo.month_index      = i + 1;
        mo.vbat_max_mV      = static_cast<uint16_t>(b(data, o)     * 100);
        mo.vbat_min_mV      = static_cast<uint16_t>(b(data, o + 1) * 100);
        mo.ah_charge_mAh    = static_cast<uint32_t>(get_u16(data, o + 2) * 1000);  // raw Ah, no /10
        mo.ah_load_mAh      = static_cast<uint32_t>(get_u16(data, o + 4) * 1000);
        mo.vpv_max_mV       = static_cast<uint16_t>(b(data, o + 6) * 500);
        mo.vpv_min_mV       = static_cast<uint16_t>(b(data, o + 7) * 500);
        mo.il_max_mA        = static_cast<uint16_t>(b(data, o + 8) * 500);
        mo.ipv_max_mA       = static_cast<uint16_t>(b(data, o + 9) * 500);
        mo.soc_pct          = (soc_raw >= 99.0f) ? 100.0f : soc_raw;
        mo.ext_temp_max_C   = static_cast<int8_t>(b(data, o + 11));
        mo.ext_temp_min_C   = static_cast<int8_t>(b(data, o + 12));
        mo.nightlength_min  = static_cast<uint16_t>(b(data, o + 13) * 10);
        mo.state            = static_cast<uint16_t>(get_u16(data, o + 14));

        monthly_logs.push_back(mo);
    }
}


static bool parseEepromBytes(
    std::vector<uint8_t>&    data,
    EepromConfig&            cfg,
    DataloggerSummary&       summary,
    std::vector<DailyLog>&   daily_logs,
    std::vector<MonthlyLog>& monthly_logs) {

    if (!parseEepromConfig(data, cfg)) {
        return false;
    }

    auto u32 = [&](int b0, int b1, int b2, int b3) -> uint32_t {
        return  ((uint32_t)b(data, b0) << 24)
              | ((uint32_t)b(data, b1) << 16)
              | ((uint32_t)b(data, b2) <<  8)
              |  (uint32_t)b(data, b3);
    };

    const int s = EEPROM_DATALOG_SUMMARY_OFFSET;
    if (static_cast<size_t>(s + 16) > data.size()) {
        return true;   // config parsed, no datalogger section present
    }

    summary.days_with_lvd              = static_cast<uint16_t>(get_u16(data, s));
    summary.months_without_full_charge = b(data, s + 2);

    uint16_t morning_soc_sum = static_cast<uint16_t>(get_u16(data, s + 4));
    summary.total_ah_charge  = u32(s + 6, s + 7, s + 8, s + 9)    / 10.0f;
    summary.total_ah_load    = u32(s + 10, s + 11, s + 12, s + 13) / 10.0f;
    summary.num_days         = static_cast<uint16_t>(get_u16(data, s + 14));

    summary.avg_morning_soc_pct = (summary.num_days > 0)
        ? (morning_soc_sum * 6.6f) / summary.num_days
        : 0.0f;

    parseDailyLogs(data,   summary.num_days, daily_logs);
    parseMonthlyLogs(data, summary.num_days, monthly_logs);

    return true;
}


// Shared parse entry point: strip leading whitespace/!, decode hex, validate length.
static bool decodeEepromLine(const char* raw, size_t len, std::vector<uint8_t>& out) {
    std::string_view sv(raw, len);
    size_t start = sv.find_first_not_of(" \t\r\n!");
    if (start == std::string_view::npos) return false;
    out = parseHexDump(sv.substr(start));
    return static_cast<int> (out.size()) >= 144;
}

bool parseEepromDump(
    const char*              raw,
    size_t                   len,
    EepromConfig&            cfg,
    DataloggerSummary&       summary,
    std::vector<DailyLog>&   daily_logs,
    std::vector<MonthlyLog>& monthly_logs) {

    std::vector<uint8_t> data;
    if (!decodeEepromLine(raw, len, data)) return false;
    return parseEepromBytes(data, cfg, summary, daily_logs, monthly_logs);
}

bool parseEepromDumpRaw(
    const char*              raw,
    size_t                   len,
    EepromConfig&            cfg,
    DataloggerSummary&       summary,
    std::vector<DailyLog>&   daily_logs,
    std::vector<MonthlyLog>& monthly_logs,
    std::vector<uint8_t>&    out_bytes) {

    if (!decodeEepromLine(raw, len, out_bytes)) return false;
    return parseEepromBytes(out_bytes, cfg, summary, daily_logs, monthly_logs);
}
