#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "mppt.pb.h"


// Builds the ordered list of &G / &H / &M / &K write commands for a
// ControlCommand.set_settings payload
void build_write_commands(const mppt_DeviceSettings *s, uint8_t hw_version, char cmds[][16], int *count);
