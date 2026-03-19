#pragma once

#include <cstdint>


// Derives the charge mode string from the chargeState bitmask (Space line field 14)
inline const char* chargeModeFromState(int charge_state) {
    if ((charge_state & 0x20) == 0x20) { return "Disabled"; }
    if ((charge_state & 0x01) == 0x01) { return "Boost"; }
    if ((charge_state & 0x02) == 0x02) { return "Equalization"; }
    return "Float";
}


inline const char* ledStatusName(uint8_t s) {
    switch (s) {
        case 0:  return "Normal";
        case 1:  return "Short";
        case 2:  return "Open";
        default: return "Unknown";
    }
}


// Battery type names are controller-generation dependent.
//   V2 (0x52):  0 = AGM/Gel,              1 = Liquid,              2 = LiFePO4
//   V3 (0x56):  0 = LiFePO4 - High Temp,  1 = LiFePO4 - Medium Temp,  2 = LiFePO4 - Low Temp
inline const char* batteryTypeName(int idx, int hw_version) {
    if (hw_version == 2) {
        switch (idx) {
            case 0:  return "AGM/Gel";
            case 1:  return "Liquid";
            case 2:  return "LiFePO4";
            default: return "Unknown";
        }
    }
    else {
        switch (idx) {
            case 0:  return "LiFePO4 - High Temp";
            case 1:  return "LiFePO4 - Medium Temp";
            case 2:  return "LiFePO4 - Low Temp";
            default: return "Unknown";
        }
    }
}


// Returns true if the device is in voltage-based LVD mode.
// V3 always uses voltage mode.
// V2 + LiFePO4 (index 2) also uses voltage mode.
// Otherwise, it depends on the raw flag from the device.
inline bool isVoltageLvdMode(int hw_version, int battery_type_index, bool raw_flag) {
    if (hw_version == 3) return true;
    if (hw_version == 2 && battery_type_index == 2) return true;
    return raw_flag;
}


// Resolves the hardware version (2 or 3) based on available data.
// EEPROM version is authoritative; fall back to the Space poll count.
inline int resolveHwVersion(bool have_eeprom, int eeprom_hw, bool have_tele, int tele_hw) {
    if (have_eeprom && eeprom_hw != 0) return eeprom_hw;
    if (have_tele) return tele_hw;
    return 3; // Default to V3
}


inline const char* nightModeName(int idx) {
    switch (idx) {
        case 0:  return "Off";
        case 1:  return "Dusk to Dawn (D2D)";
        case 2:  return "Dusk and Dawn (DD)";
        case 3:  return "Middle of the Night (MN)";
        default: return "Unknown";
    }
}
