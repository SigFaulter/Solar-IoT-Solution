#pragma once

#include "types.h"

#include <vector>


// Prints the parsed telemetry to stdout in a human-readable format.
void printSystemState(
    const PhocosHeader&     hdr,
    const PhocosTelemetry&  t,
    const EepromConfig&     cfg,
    const char*             timestamp
);


// Prints all configuration fields read from the EEPROM dump.
void printEepromConfig(const EepromConfig& cfg);


// Prints the datalogger summary, daily table and monthly table.
void printDataLogger(
    const DataloggerSummary&        summary,
    const std::vector<DailyLog>&    daily_logs,
    const std::vector<MonthlyLog>&  monthly_logs
);
