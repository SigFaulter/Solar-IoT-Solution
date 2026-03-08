#pragma once

#include "types.h"
#include "json.hpp"

#include <string>
#include <vector>


// Serialises all parsed data into a single JSON string for MQTT publishing.
// MQTT topic: mppt/{serial_number}/state
nlohmann::json buildJSON(
    const PhocosHeader&             hdr,
    const PhocosTelemetry&          t,
    const EepromConfig&             cfg,
    const DataloggerSummary&        summary,
    const std::vector<DailyLog>&    daily_logs,
    const std::vector<MonthlyLog>&  monthly_logs,
    const char*                     timestamp
);

void publishJSON(const nlohmann::json& j, const char* serial_number);
