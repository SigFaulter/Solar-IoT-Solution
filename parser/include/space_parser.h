#pragma once

#include "types.h"


// Parses one Space Command response line into a PhocosTelemetry struct.
// Returns true on success, false if the line is malformed or unrecognised.
bool parsePhocosLine(const char* raw, size_t len, PhocosTelemetry& out);
