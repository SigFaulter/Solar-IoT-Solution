#pragma once

#include "types.h"

#include <vector>


bool parseEepromDump(
    const char*                  raw,
    size_t                       len,
    EepromConfig&                cfg,
    DataloggerSummary&           summary,
    std::vector<DailyLog>&       daily_logs,
    std::vector<MonthlyLog>&     monthly_logs
);


bool parseEepromDumpRaw(
    const char*              raw,
    size_t                   len,
    EepromConfig&            cfg,
    DataloggerSummary&       summary,
    std::vector<DailyLog>&   daily_logs,
    std::vector<MonthlyLog>& monthly_logs,
    std::vector<uint8_t>&    out_bytes
);
