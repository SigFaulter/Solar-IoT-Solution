#pragma once

#include "types.h"

#include <vector>


// Parses the full '!' EEPROM dump line into config, datalogger summary,
// daily logs and monthly logs.
//
// The dump starts at device address 0x1000
// All offsets in the EEPROM constants are relative to the start of the dump
//
// Returns true if at least the config section was successfully parsed
// Daily and monthly logs are only populated if enough bytes are present
bool parseEepromDump(
    const char*                  raw,
    size_t                       len,
    EepromConfig&                cfg,
    DataloggerSummary&           summary,
    std::vector<DailyLog>&       daily_logs,
    std::vector<MonthlyLog>&     monthly_logs
);
