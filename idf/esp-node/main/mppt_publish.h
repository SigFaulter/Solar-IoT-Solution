#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "space_parser.h"  /* SpaceTelemetry, EepromData */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * All publish functions read from state set by publish_init().
 * They are no-ops if the relevant data is not ready.
 */
void publish_device_info(void);
void publish_device_settings(void);
void publish_telemetry(void);
void publish_fault_status(bool force);
void publish_datalog(bool send_all);

/* Must be called once before any publish_* call */
void publish_init(SpaceTelemetry *tele, EepromData *eeprom,
                  bool *tele_ok, bool *eeprom_ok,
                  uint8_t *hw_version);

/* Decode and handle an inbound ControlCommand proto, send ACK */
void handle_command(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif
