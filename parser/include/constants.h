#pragma once


//
// Protocol field counts
//
// Version is detected by counting semicolons before parsing
// V2 firmware produces 27 fields, V3 produces 42
//
constexpr int EXPECTED_FIELDS_V2 = 27;
constexpr int EXPECTED_FIELDS_V3 = 42;

constexpr size_t FIELD_BUF_SIZE   = 24;
constexpr size_t MIN_RESPONSE_LEN = 80;   // V2 lines ~142 chars, V3 ~205


//
// EEPROM byte offsets
//
// All offsets are from the start of the dump, which begins at device address 0x1000
// Derived from Kotlin EepromParser and DataLoggerParser
//
constexpr int EEPROM_DEVICE_ID_OFFSET          = 120;   // 0x52 = V2, 0x56 = V3

constexpr int EEPROM_BATTERY_TYPE_OFFSET         = 62;
constexpr int EEPROM_CAPACITY_AH_OFFSET          = 95;    // u16 @ 95, 96
constexpr int EEPROM_BAT_OP_DAYS_OFFSET          = 110;   // u16 @ 110, 111

constexpr int EEPROM_LVD_MODE_OFFSET             = 61;    // 0 = current, 1 = voltage
constexpr int EEPROM_LVD_CURRENT_MV_OFFSET       = 39;    // u16 @ 39, 40
constexpr int EEPROM_LVD_VOLTAGE_MV_OFFSET       = 41;    // u16 @ 41, 42

constexpr int EEPROM_EQUALIZATION_MV_OFFSET      = 75;    // u16 @ 75, 76
constexpr int EEPROM_BOOST_MV_OFFSET             = 77;    // u16 @ 77, 78
constexpr int EEPROM_FLOAT_MV_OFFSET             = 81;    // u16 @ 81, 82
constexpr int EEPROM_TEMP_COMP_OFFSET            = 66;    // s16 @ 66, 67  /10.0 = mV/°C
constexpr int EEPROM_NIGHT_THRESH_MV_OFFSET      = 67;    // u16 @ 67, 68

constexpr int EEPROM_NIGHT_MODE_OFFSET           = 34;    // 0–3
constexpr int EEPROM_EVENING_MINUTES_OFFSET      = 35;    // u16 @ 35, 36
constexpr int EEPROM_MORNING_MINUTES_OFFSET      = 37;    // u16 @ 37, 38
constexpr int EEPROM_DIM_MODE_OFFSET             = 43;    // 0–3
constexpr int EEPROM_DIM_EVENING_OFFSET          = 44;    // u16 @ 44, 45
constexpr int EEPROM_DIM_MORNING_OFFSET          = 46;    // u16 @ 46, 47
constexpr int EEPROM_DIMMING_PCT_OFFSET          = 69;
constexpr int EEPROM_BASE_DIMMING_PCT_OFFSET     = 70;

constexpr int EEPROM_DALI_FLAG_OFFSET            = 97;
constexpr int EEPROM_ALC_FLAG_OFFSET             = 98;

constexpr int EEPROM_SERIAL_LSB_OFFSET           = 112;   // BCD
constexpr int EEPROM_SERIAL_MSB_OFFSET           = 113;   // BCD
constexpr int EEPROM_PROD_DAY_OFFSET             = 114;   // BCD
constexpr int EEPROM_PROD_MONTH_OFFSET           = 115;   // BCD
constexpr int EEPROM_PROD_YEAR_LSB_OFFSET        = 116;   // BCD
constexpr int EEPROM_PROD_YEAR_MSB_OFFSET        = 117;   // BCD
constexpr int EEPROM_MANUF_MONTH_OFFSET          = 118;   // BCD

constexpr int EEPROM_OPERATION_DAYS_OFFSET       = 142;   // u16 @ 142, 143

constexpr int EEPROM_DATALOG_SUMMARY_OFFSET      = 128;
constexpr int EEPROM_DAILY_START_OFFSET          = 144;
constexpr int EEPROM_DAILY_BLOCK_SIZE            = 16;
constexpr int EEPROM_DAILY_MAX_BLOCKS            = 30;

constexpr int EEPROM_MONTHLY_START_OFFSET        = 624;
constexpr int EEPROM_MONTHLY_BLOCK_SIZE          = 16;
constexpr int EEPROM_MONTHLY_MAX_BLOCKS          = 24;
