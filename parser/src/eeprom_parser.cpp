#include "eeprom_parser.h"
#include "constants.h"
#include "utils.h"
#include "lookups.h"

#include <algorithm>
#include <cstring>
#include <cstdio>


// Reads all configuration fields from a parsed byte vector into an EepromConfig.
static bool parseEepromConfig(const std::vector<uint8_t>& data, EepromConfig& cfg) {
    if ((int)data.size() < 144) {
        return false;
    }

    auto b = [&](int off) -> uint8_t {
        return (off < (int)data.size()) ? data[off] : 0u;
    };

    auto u16 = [&](int msb, int lsb) -> int {
        return ((int)b(msb) << 8) | b(lsb);
    };

    auto s16 = [&](int msb, int lsb) -> int {
        int v = u16(msb, lsb);
        return (v & 0x8000) ? v - 0x10000 : v;
    };

    // Device type, byte 120 is the source for V2 vs V3
    uint8_t type_byte = b(EEPROM_DEVICE_TYPE_OFFSET);
    cfg.hw_version = (type_byte == 0x52) ? 2 : 3;

    const char* dt = (type_byte == 0x52) ? "Solar Smart Controller V2"
                   : (type_byte == 0x56) ? "Solar Smart Controller V3"
                   : "Unknown";
    cfg.device_type = dt;

    // Serial number: bytes 113 (MSB), 112 (LSB), 118 (manuf. month), all BCD
    // Format: MMSS-MM  (e.g. "0166-18")
    {
        int sm = bcd2dec(b(EEPROM_SERIAL_MSB_OFFSET));
        int sl = bcd2dec(b(EEPROM_SERIAL_LSB_OFFSET));
        int mm = bcd2dec(b(EEPROM_MANUF_MONTH_OFFSET));
        char serial_buf[16];
        snprintf(serial_buf, sizeof(serial_buf), "%02d%02d-%02d", sm, sl, mm);
        cfg.serial_number = serial_buf;
    }

    // Production date: bytes 117, 116, 115, 114 (year MSB, year LSB, month, day), all BCD
    {
        int day = bcd2dec(b(EEPROM_PROD_DAY_OFFSET));
        int mon = bcd2dec(b(EEPROM_PROD_MONTH_OFFSET));
        int yl  = bcd2dec(b(EEPROM_PROD_YEAR_LSB_OFFSET));
        int ym  = bcd2dec(b(EEPROM_PROD_YEAR_MSB_OFFSET));
        char date_buf[16];
        snprintf(date_buf, sizeof(date_buf), "%02d%02d-%02d-%02d", ym, yl, mon, day);
        cfg.production_date = date_buf;
    }

    // Battery type, names are controller-generation dependent (V2 vs V3 have different batter types)
    // TODO confirm this is the case
    cfg.battery_type_index = std::clamp((int)b(EEPROM_BATTERY_TYPE_OFFSET), 0, 2);
    cfg.battery_type = batteryTypeName(cfg.battery_type_index, cfg.hw_version);

    cfg.capacity_ah     = u16(EEPROM_CAPACITY_AH_OFFSET,  EEPROM_CAPACITY_AH_OFFSET  + 1);
    cfg.battery_op_days = u16(EEPROM_BAT_OP_DAYS_OFFSET,  EEPROM_BAT_OP_DAYS_OFFSET  + 1);

    // LVD mode:
    //   V3 always uses voltage mode.
    //   V2 + LiFePO4 (index 2) also uses voltage mode.
    //   All other V2 configs use the stored lvd_mode flag.
    bool lvd_mode_raw   = (b(EEPROM_LVD_MODE_OFFSET) == 1);
    cfg.lvd_mode_voltage = (cfg.hw_version == 3)
                        || (cfg.hw_version == 2 && cfg.battery_type_index == 2)
                        || lvd_mode_raw;

    int lvd_cur = u16(EEPROM_LVD_CURRENT_MV_OFFSET, EEPROM_LVD_CURRENT_MV_OFFSET + 1);
    int lvd_vol = u16(EEPROM_LVD_VOLTAGE_MV_OFFSET, EEPROM_LVD_VOLTAGE_MV_OFFSET + 1);
    cfg.lvd_voltage_mV = cfg.lvd_mode_voltage ? lvd_vol : lvd_cur;

    cfg.equalization_mV    = u16(EEPROM_EQUALIZATION_MV_OFFSET, EEPROM_EQUALIZATION_MV_OFFSET + 1);
    cfg.boost_mV           = u16(EEPROM_BOOST_MV_OFFSET,        EEPROM_BOOST_MV_OFFSET        + 1);
    cfg.float_mV           = u16(EEPROM_FLOAT_MV_OFFSET,        EEPROM_FLOAT_MV_OFFSET        + 1);
    cfg.temp_comp_mV_per_C = s16(EEPROM_TEMP_COMP_OFFSET,       EEPROM_TEMP_COMP_OFFSET       + 1) / 10.0;
    cfg.night_threshold_mV = u16(EEPROM_NIGHT_THRESH_MV_OFFSET, EEPROM_NIGHT_THRESH_MV_OFFSET + 1);
    cfg.operation_days     = u16(EEPROM_OPERATION_DAYS_OFFSET,  EEPROM_OPERATION_DAYS_OFFSET  + 1);

    cfg.dali_active = (b(EEPROM_DALI_FLAG_OFFSET) == 1);
    cfg.alc_dimming = (b(EEPROM_ALC_FLAG_OFFSET)  == 1);

    cfg.night_mode_index   = std::clamp((int)b(EEPROM_NIGHT_MODE_OFFSET), 0, 3);
    cfg.night_mode = nightModeName(cfg.night_mode_index);
    cfg.evening_minutes_mn = u16(EEPROM_EVENING_MINUTES_OFFSET, EEPROM_EVENING_MINUTES_OFFSET + 1);
    cfg.morning_minutes_mn = u16(EEPROM_MORNING_MINUTES_OFFSET, EEPROM_MORNING_MINUTES_OFFSET + 1);

    cfg.night_mode_dimming_index = std::clamp((int)b(EEPROM_DIM_MODE_OFFSET), 0, 3);
    cfg.night_mode_dimming = nightModeName(cfg.night_mode_dimming_index);
    cfg.evening_minutes_dimming_mn = u16(EEPROM_DIM_EVENING_OFFSET, EEPROM_DIM_EVENING_OFFSET + 1);
    cfg.morning_minutes_dimming_mn = u16(EEPROM_DIM_MORNING_OFFSET, EEPROM_DIM_MORNING_OFFSET + 1);
    cfg.dimming_pct      = b(EEPROM_DIMMING_PCT_OFFSET);
    cfg.base_dimming_pct = b(EEPROM_BASE_DIMMING_PCT_OFFSET);

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

    auto b   = [&](int off) -> uint8_t { return (off < (int)data.size()) ? data[off] : 0u; };
    auto u16 = [&](int msb, int lsb) -> int { return ((int)b(msb) << 8) | b(lsb); };

    const int start_buf = ((int)num_days - num_entries) % EEPROM_DAILY_MAX_BLOCKS;

    for (int i = 0; i < num_entries; ++i) {
        int buf_idx = (start_buf + i) % EEPROM_DAILY_MAX_BLOCKS;
        int o       = EEPROM_DAILY_START_OFFSET + buf_idx * EEPROM_DAILY_BLOCK_SIZE;

        if (o + EEPROM_DAILY_BLOCK_SIZE > (int)data.size()) {
            continue;
        }

        // Skip blocks with no meaningful data
        bool blank = (b(o) == 0 && b(o + 1) == 0 && u16(o+2, o+3) == 0 && u16(o+4, o+5) == 0);
        if (blank) {
            continue;
        }

        float soc_raw = b(o + 10) * 6.6f;

        DailyLog day;
        day.day_index       = i + 1;
        day.vbat_max_mV     = static_cast<uint16_t>(b(o)     * 100);
        day.vbat_min_mV     = static_cast<uint16_t>(b(o + 1) * 100);
        day.ah_charge_mAh   = static_cast<uint32_t>(u16(o + 2, o + 3) * 100);
        day.ah_load_mAh     = static_cast<uint32_t>(u16(o + 4, o + 5) * 100);
        day.vpv_max_mV      = static_cast<uint16_t>(b(o + 6) * 500);
        day.vpv_min_mV      = static_cast<uint16_t>(b(o + 7) * 500);
        day.il_max_mA       = static_cast<uint16_t>(b(o + 8) * 500);
        day.ipv_max_mA      = static_cast<uint16_t>(b(o + 9) * 500);
        day.soc_pct         = (soc_raw >= 99.0f) ? 100.0f : soc_raw;
        day.ext_temp_max_C  = static_cast<int8_t>(b(o + 11));
        day.ext_temp_min_C  = static_cast<int8_t>(b(o + 12));
        day.nightlength_min = static_cast<uint16_t>(b(o + 13) * 10);
        day.state           = static_cast<uint16_t>(u16(o + 14, o + 15));

        daily_logs.push_back(day);
    }
}


// Reads the monthly circular buffer into monthly_logs.
//
// Offset 624, up to 24 entries, same 16-byte block structure as daily
// Monthly Ah values are raw u16 (no /10 division unlike daily)
static void parseMonthlyLogs(
    const std::vector<uint8_t>& data,
    uint16_t num_days,
    std::vector<MonthlyLog>& monthly_logs) {
    monthly_logs.clear();

    if ((int)data.size() < EEPROM_MONTHLY_START_OFFSET + EEPROM_MONTHLY_BLOCK_SIZE) {
        return;
    }

    auto b   = [&](int off) -> uint8_t { return (off < (int)data.size()) ? data[off] : 0u; };
    auto u16 = [&](int msb, int lsb) -> int { return ((int)b(msb) << 8) | b(lsb); };

    const int total_months = num_days / 31;
    const int num_monthly  = std::min(total_months, EEPROM_MONTHLY_MAX_BLOCKS);

    if (num_monthly <= 0) {
        return;
    }

    const int start_mbuf = (total_months - num_monthly) % EEPROM_MONTHLY_MAX_BLOCKS;

    for (int i = 0; i < num_monthly; ++i) {
        int buf_idx = (start_mbuf + i) % EEPROM_MONTHLY_MAX_BLOCKS;
        int o       = EEPROM_MONTHLY_START_OFFSET + buf_idx * EEPROM_MONTHLY_BLOCK_SIZE;

        if (o + EEPROM_MONTHLY_BLOCK_SIZE > (int)data.size()) {
            continue;
        }

        if (b(o) == 0 && u16(o + 2, o + 3) == 0) {
            continue;
        }

        float soc_raw = b(o + 10) * 6.6f;

        MonthlyLog mo;
        mo.month_index      = i + 1;
        mo.vbat_max_mV      = static_cast<uint16_t>(b(o)     * 100);
        mo.vbat_min_mV      = static_cast<uint16_t>(b(o + 1) * 100);
        mo.ah_charge_mAh    = static_cast<uint32_t>(u16(o + 2, o + 3) * 1000);  // raw Ah, no /10
        mo.ah_load_mAh      = static_cast<uint32_t>(u16(o + 4, o + 5) * 1000);
        mo.vpv_max_mV       = static_cast<uint16_t>(b(o + 6) * 500);
        mo.vpv_min_mV       = static_cast<uint16_t>(b(o + 7) * 500);
        mo.il_max_mA        = static_cast<uint16_t>(b(o + 8) * 500);
        mo.ipv_max_mA       = static_cast<uint16_t>(b(o + 9) * 500);
        mo.soc_pct          = (soc_raw >= 99.0f) ? 100.0f : soc_raw;
        mo.ext_temp_max_C   = static_cast<int8_t>(b(o + 11));
        mo.ext_temp_min_C   = static_cast<int8_t>(b(o + 12));
        mo.nightlength_min  = static_cast<uint16_t>(b(o + 13) * 10);
        mo.state            = static_cast<uint16_t>(u16(o + 14, o + 15));

        monthly_logs.push_back(mo);
    }
}


bool parseEepromDump(
    const char*              raw,
    size_t                   len,
    EepromConfig&            cfg,
    DataloggerSummary&       summary,
    std::vector<DailyLog>&   daily_logs,
    std::vector<MonthlyLog>& monthly_logs) {
    std::string_view sv(raw, len);

    size_t start = sv.find_first_not_of(" \t\r\n!");
    if (start == std::string_view::npos) {
        return false;
    }
    sv = sv.substr(start);

    std::vector<uint8_t> data = parseHexDump(sv);

    if ((int)data.size() < 144) {
        return false;
    }

    // Config fields, always parsed if we have enough bytes
    if (!parseEepromConfig(data, cfg)) {
        return false;
    }

    auto b   = [&](int off) -> uint8_t { return (off < (int)data.size()) ? data[off] : 0u; };
    auto u16 = [&](int msb, int lsb) -> int { return ((int)b(msb) << 8) | b(lsb); };
    auto u32 = [&](int b0, int b1, int b2, int b3) -> uint32_t {
        return  ((uint32_t)b(b0) << 24)
              | ((uint32_t)b(b1) << 16)
              | ((uint32_t)b(b2) <<  8)
              |  (uint32_t)b(b3);
    };

    // Datalogger summary block (offset 128)
    const int s = EEPROM_DATALOG_SUMMARY_OFFSET;
    if (s + 16 > (int)data.size()) {
        return true;   // config parsed, no datalogger section present
    }

    summary.days_with_lvd              = static_cast<uint16_t>(u16(s, s + 1));
    summary.months_without_full_charge = b(s + 2);

    uint16_t morning_soc_sum = static_cast<uint16_t>(u16(s + 4, s + 5));
    summary.total_ah_charge  = u32(s + 6, s + 7, s + 8, s + 9)    / 10.0f;
    summary.total_ah_load    = u32(s + 10, s + 11, s + 12, s + 13) / 10.0f;
    summary.num_days         = static_cast<uint16_t>(u16(s + 14, s + 15));

    summary.avg_morning_soc_pct = (summary.num_days > 0)
        ? (morning_soc_sum * 6.6f) / summary.num_days
        : 0.0f;

    parseDailyLogs(data,   summary.num_days, daily_logs);
    parseMonthlyLogs(data, summary.num_days, monthly_logs);

    return true;
}
