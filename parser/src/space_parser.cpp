#include "space_parser.h"
#include "constants.h"
#include "utils.h"
#include "lookups.h"

#include <algorithm>
#include <cstring>


bool parsePhocosLine(const char* raw, size_t len, PhocosTelemetry& t) {
    if (len < MIN_RESPONSE_LEN) {
        return false;
    }

    std::string_view resp(raw, len);

    size_t start = resp.find_first_not_of(" \t\r\n");
    if (start == std::string_view::npos) {
        return false;
    }
    resp = resp.substr(start);

    // Count semicolons to detect firmware version before touching any fields
    const int total_semis = static_cast<int>(std::count(resp.begin(), resp.end(), ';'));
    const int expected =
        (total_semis == EXPECTED_FIELDS_V2) ? EXPECTED_FIELDS_V2 :
        (total_semis == EXPECTED_FIELDS_V3) ? EXPECTED_FIELDS_V3 : -1;

    if (expected == -1) {
        return false;
    }

    t.hw_version = (expected == EXPECTED_FIELDS_V2) ? 2 : 3;

    int    field_idx = 0;
    size_t pos       = 0;
    char   buf[FIELD_BUF_SIZE];

    while (pos < resp.size() && field_idx < expected) {
        size_t semi = resp.find(';', pos);
        if (semi == std::string_view::npos) {
            semi = resp.size();
        }

        size_t flen = semi - pos;

        if (flen == 0) {
            pos = semi + 1;
            ++field_idx;
            continue;
        }

        if (flen >= FIELD_BUF_SIZE) {
            return false;
        }

        std::memcpy(buf, resp.data() + pos, flen);
        buf[flen] = '\0';

        const int v = fast_atoi(buf);

        switch (field_idx + 1) {
            case  1: t.charge_current_mA    = static_cast<uint32_t>(v * 10);  break;
            case  2: t.load_current_mA      = static_cast<uint32_t>(v * 10);  break;
            case  6: t.pv_voltage_mV        = static_cast<uint32_t>(v);       break;
            case  7: t.pv_target_mV         = static_cast<uint32_t>(v);       break;
            case  8: t.pwm_counts           = static_cast<uint16_t>(v);       break;
            case 12: t.firmware_version     = static_cast<uint8_t>(v);        break;

            // Field 13: loadState — parse full bitmask into LoadStatusFlags
            case 13:
                t.load_state_raw = v;
                t.load_flags     = LoadStatusFlags::parse(v);
                break;

            // Field 14: chargeState — source of charge mode and is_night
            case 14:
                t.charge_state_raw = v;
                t.charge_flags     = ChargeStatusFlags::parse(v);
                break;

            case 15: t.battery_voltage_mV   = static_cast<uint32_t>(v);       break;
            case 16: t.battery_threshold_mV = static_cast<uint32_t>(v);       break;

            case 17:
                t.battery_soc_pct = static_cast<uint8_t>(std::clamp(v, 0, 100));
                break;

            // Fields 18–19: temp values come through as ASCII strings in the protocol
            case 18: t.internal_temp_C  = v;                                   break;
            case 19: t.external_temp_C  = v;                                   break;

            case 21: t.mpp_state        = v;                                   break;
            case 22: t.hvd_state        = v;                                   break;
            case 24: t.load_state2_raw  = v;                                   break;
            case 25: t.nightlength_min  = static_cast<uint16_t>(v);           break;
            case 26: t.avg_nightlength_min = static_cast<uint16_t>(v);        break;
            case 28: t.led_voltage_mV   = static_cast<uint32_t>(v);           break;
            case 29: t.led_current_mA   = static_cast<uint32_t>(v * 10);      break;
            case 30: t.led_status       = static_cast<uint8_t>(v);            break;
            case 31: t.dali_active      = static_cast<uint8_t>(v);            break;
            case 32: t.op_days          = static_cast<uint16_t>(v);           break;
            case 33: t.bat_op_days      = static_cast<uint16_t>(v);           break;
            case 34: t.energy_in_daily_Wh  = static_cast<uint16_t>(v);        break;
            case 35: t.energy_out_daily_Wh = static_cast<uint16_t>(v);        break;
            case 36: t.energy_retained_Wh  = static_cast<uint16_t>(v);        break;
            case 37: t.charge_power_W      = static_cast<uint16_t>(v);        break;
            case 38: t.load_power_W        = static_cast<uint16_t>(v);        break;
            case 39: t.led_power_W         = static_cast<uint16_t>(v);        break;

            // Field 40: fault mask — V3 only, parse into FaultStatusFlags
            case 40:
                t.fault_status = static_cast<uint16_t>(v);
                t.fault_flags  = FaultStatusFlags::parse(v);
                break;

            case 41: t.pv_detected      = static_cast<uint8_t>(v);            break;
            case 42: t.battery_detected = static_cast<uint8_t>(v);            break;

            default: break;
        }

        pos = semi + 1;
        ++field_idx;
    }

    if (field_idx < expected || t.battery_voltage_mV == 0) {
        return false;
    }

    return true;
}


void parseHeaderLine(const std::string& line, PhocosHeader& hdr) {
    size_t s = line.find_first_not_of(" *\t");
    if (s == std::string::npos) {
        return;
    }

    std::string_view sv(line.c_str() + s, line.size() - s);

    auto extract = [&](const char* key, std::string& dest) {
        size_t klen = strlen(key);
        if (sv.substr(0, klen) != key) {
            return;
        }

        size_t v = sv.find_first_not_of(" \t:", klen);
        if (v == std::string_view::npos) {
            return;
        }

        std::string_view val = sv.substr(v);

        while (!val.empty() && (val.back() == ' ' || val.back() == '\r' || val.back() == '\n')) {
            val.remove_suffix(1);
        }

        dest = std::string(val);
    };

    // TODO remove in prod
    // for local log parsing only
    // V3 English header keys
    extract("Charge Controller", hdr.type);
    extract("Production date",   hdr.production_date);
    extract("Serial number",     hdr.serial_number);

    // V2 French header keys (log file is Latin-1 encoded)
    extract("R\xe9gulateur de charge", hdr.type);
    extract("Date de production",      hdr.production_date);
    extract("Num\xe9ro de s\xe9rie",  hdr.serial_number);

    if (!hdr.type.empty() && hdr.type.find("V2") != std::string::npos) {
        hdr.hw_version = 2;
    }
    else if (!hdr.type.empty()) {
        hdr.hw_version = 3;
    }
}
