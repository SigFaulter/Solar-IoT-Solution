#pragma once

#include "types.h"
#include "settings_parser.h"
#include "json.hpp"

#include <string>
#include <vector>


// Telemetry snapshot
// Topic: mppt/{zone}/{gw}/{serial}/state
nlohmann::json buildTelemetryJSON(
    const PhocosTelemetry& t,
    const EepromConfig&    cfg,
    const char*            timestamp
);

// Datalogger, published once in a while
// Topic: mppt/{zone}/{gw}/{serial}/datalog
nlohmann::json buildDataloggerJSON(
    const EepromConfig&             cfg,
    const DataloggerSummary&        summary,
    const std::vector<DailyLog>&    days,
    const std::vector<MonthlyLog>&  months,
    const char*                     timestamp
);

// Settings published when settings have been parsed/changed
// Topic: mppt/{zone}/{gw}/{serial}/settings
nlohmann::json buildSettingsJSON(
    const DeviceSettings& s,
    const char*           timestamp
);
