#pragma once

#include <ctime>

#include "types.h"

// Prints the parsed telemetry to stdout in a human-readable format.
void print_system_state(const PhocosTelemetry &t, const EepromSettings &settings, std::time_t ts);

// Prints all settings fields read from the EEPROM dump.
void print_eeprom_config(const EepromSettings &settings);

// Prints the datalogger summary, daily table and monthly table.
void print_data_logger(const DataloggerSummary &summary,
                       const DailyLogBuffer    &days,
                       const MonthlyLogBuffer  &months);
