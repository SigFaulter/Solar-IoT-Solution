#pragma once

#include "types.h"

#include <cstdint>
#include <vector>
#include <string>

static constexpr int SETTINGS_WINDOW_START       = 34;
static constexpr int SR_NIGHT_MODE_IDX           =  0;
static constexpr int SR_EVENING_MIN              =  1;
static constexpr int SR_MORNING_MIN              =  3;
static constexpr int SR_LVD_CURRENT_MV           =  5;
static constexpr int SR_LVD_VOLTAGE_MV           =  7;
static constexpr int SR_DIM_MODE_IDX             =  9;
static constexpr int SR_DIM_EVENING_MIN          = 10;
static constexpr int SR_DIM_MORNING_MIN          = 12;
static constexpr int SR_LVD_MODE_VOLTAGE         = 27;
static constexpr int SR_BATTERY_TYPE_IDX         = 28;
static constexpr int SR_NIGHT_THRESH_MV          = 33;
static constexpr int SR_DIMMING_PCT              = 35;
static constexpr int SR_BASE_DIMMING_PCT         = 36;
static constexpr int SR_CAPACITY_AH              = 61;
static constexpr int SR_DALI_ENABLE              = 63;
static constexpr int SR_ALC_DIMMING              = 64;
inline constexpr int W(int rel) { return SETTINGS_WINDOW_START + rel; }


// Build DeviceSettings from a parsed EepromConfig (live mode).
DeviceSettings deviceSettingsFromEeprom(const EepromConfig& cfg, int load_state = -1);


// Patch a copy of the 99-byte settings blob with the fields from s.
// Produces new_bytes for writeSettingsToDevice. Actual writes use &G/&H/&M.
void patchSettingsBytes(const std::vector<uint8_t>& current_bytes,
                        const DeviceSettings&       s,
                        std::vector<uint8_t>&       out_bytes);


// Apply a single "key=value" patch string to a DeviceSettings.
// Returns false and prints an error on bad key or value.
bool applySettingsPatch(const std::string& kv, DeviceSettings& s);


void printSettings(const DeviceSettings& s);
