#pragma once
#include <ctime>
#include <string>
#include <string_view>
#include "../proto_gen/mppt.pb.h"
#include "lookups.h"
#include "types.h"

std::string proto_to_string(const ::google::protobuf::MessageLite &msg);
mppt::FaultStatus build_fault_status_proto(const PhocosTelemetry &t, std::time_t ts);
mppt::Telemetry build_telemetry_proto(const PhocosTelemetry &t, std::time_t ts);
mppt::LogEntry build_log_entry_proto(const LogEntry &e);
mppt::DataloggerPayload build_datalogger_summary_proto(const DataloggerSummary &summary, std::time_t ts);
mppt::DataloggerPayload build_datalogger_daily_proto(const DailyLogBuffer &days, std::time_t ts);
mppt::DataloggerPayload build_datalogger_monthly_proto(const MonthlyLogBuffer &months, std::time_t ts);
mppt::DeviceInfo build_device_info_proto(const EepromSettings &cfg, uint32_t fw_version, std::time_t ts);
mppt::DeviceSettings build_device_settings_proto(const DeviceSettings &s, std::time_t ts);
bool parse_control_command(const std::string &payload_bytes, mppt::ControlCommand &cmd_out);
std::string build_command_ack(std::string_view request_id, bool ok, std::string_view reason, std::time_t ts);
DeviceSettings device_settings_from_proto(const mppt::DeviceSettings &p);
