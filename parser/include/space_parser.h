#pragma once

#include "types.h"


// Parses one Space Command response line into a PhocosTelemetry struct.
// Returns true on success, false if the line is malformed or unrecognised.
bool parsePhocosLine(const char* raw, size_t len, PhocosTelemetry& out);


// Extracts device identity fields from a banner comment line (lines starting with '*').
// Only present in log files produced by PhocosLink PC software.
// Handles both V3 (English) and V2 (French / Latin-1) header keys.
void parseHeaderLine(const std::string& line, PhocosHeader& out);
