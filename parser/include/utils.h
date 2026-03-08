#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string_view>
#include <vector>


inline float r1(float v) { return std::round(v * 10.0f)    / 10.0f; }
inline float r2(float v) { return std::round(v * 100.0f)   / 100.0f; }

//
// fast_atoi
//
// Converts a null-terminated ASCII string to int
// Handles leading +/- signs. Stops at the first non-digit character
//
inline int fast_atoi(const char* s) {
    if (!s || *s == '\0') {
        return 0;
    }

    bool neg = false;

    if (*s == '-') {
        neg = true;
        ++s;
    }
    else if (*s == '+') {
        ++s;
    }

    int v = 0;
    while (*s >= '0' && *s <= '9') {
        v = v * 10 + (*s - '0');
        ++s;
    }

    return neg ? -v : v;
}


// Converts a BCD-encoded byte to its decimal value.
// Example: 0x25 → 25
//
inline int bcd2dec(uint8_t b) {
    return ((b >> 4) & 0xF) * 10 + (b & 0xF);
}


inline void currentTimestamp(char* buf, size_t sz) {
    time_t now = time(nullptr);
    strftime(buf, sz, "%H:%M:%S", localtime(&now));
}


// Line type detectors
// Used by main() to route each line to the correct parser.
//
inline bool isSpaceLine(std::string_view line) {
    if (line.empty()) {
        return false;
    }

    size_t s = line.find_first_not_of(" \t\r\n");

    if (s == std::string_view::npos) {
        return false;
    }

    char c = line[s];

    if (c == '!' || c == '"' || c == '*' || c == '-') {
        return false;
    }

    return (c >= '0' && c <= '9');
}

inline bool isEepromLine(std::string_view line) {
    if (line.empty()) {
        return false;
    }

    size_t s = line.find_first_not_of(" \t\r\n");

    if (s == std::string_view::npos) {
        return false;
    }

    return line[s] == '!';
}

inline bool isSettingsLine(std::string_view line) {
    if (line.empty()) {
        return false;
    }

    size_t s = line.find_first_not_of(" \t\r\n");

    if (s == std::string_view::npos) {
        return false;
    }

    return line[s] == '"';
}


// converts a hex/decimal byte string into a byte vector
inline std::vector<uint8_t> parseHexDump(std::string_view sv) {
    std::vector<uint8_t> data;
    data.reserve(1024);

    size_t pos = 0;
    char   hex_buf[4];

    while (pos < sv.size()) {
        size_t semi = sv.find(';', pos);

        if (semi == std::string_view::npos) {
            semi = sv.size();
        }

        size_t hlen = semi - pos;

        if (hlen == 0 || hlen > 3) {
            pos = semi + 1;
            continue;
        }

        std::memcpy(hex_buf, sv.data() + pos, hlen);
        hex_buf[hlen] = '\0';

        bool is_hex = true;
        for (size_t i = 0; i < hlen; ++i) {
            if (!std::isxdigit((unsigned char)hex_buf[i])) {
                is_hex = false;
                break;
            }
        }

        int val = is_hex ? (int)strtol(hex_buf, nullptr, 16) : fast_atoi(hex_buf);
        data.push_back(static_cast<uint8_t>(val & 0xFF));

        pos = semi + 1;
    }

    return data;
}
