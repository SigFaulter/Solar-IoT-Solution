#pragma once
#include "space_parser.h"
#include "mppt.pb.h"
#include <pb_encode.h>

struct LogArg {
    const EepromLogEntry *entries;
    uint8_t start;
    uint8_t count;
};

bool encode_log_entries(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
bool write_string_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);

uint32_t make_tele_flags(const SpaceTelemetry &t);
mppt_Telemetry build_telemetry(const SpaceTelemetry &t, uint32_t ts);
mppt_FaultStatus build_fault_status(const SpaceTelemetry &t, uint32_t ts);
mppt_DeviceInfo build_device_info(const SpaceTelemetry &t, const EepromData &e, uint32_t ts);
mppt_DeviceSettings build_device_settings(const EepromData &e, const mppt_DeviceSettings *override, uint32_t ts);
