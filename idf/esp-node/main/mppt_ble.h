#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Message type tags - must match rpi_ble_gateway.py */
typedef enum {
    MSG_TELEMETRY    = 0x01,
    MSG_FAULT_STATUS = 0x02,
    MSG_DEVICE_INFO  = 0x03,
    MSG_SETTINGS     = 0x04,
    MSG_DATALOG      = 0x05,
    MSG_ACK          = 0x11,
    MSG_CMD          = 0x10,
} mppt_msg_type_t;

/*
 * Wire format:
 *   byte 0:   msg_type
 *   bytes 1-2: payload_len (uint16 LE)
 *   bytes 3+:  payload
 *
 * Chunked on the wire with 1-byte tag prefix:
 *   0x00 CHUNK_FIRST  (or only chunk)
 *   0x01 CHUNK_CONT
 *   0x80 CHUNK_LAST
 */

/* Send a framed proto payload over BLE; handles chunking to MTU */
void mppt_ble_send(mppt_msg_type_t msg_type, const uint8_t *payload, uint16_t len);

/* ---------- Reassembler ---------- */
#define MPPT_REASSEMBLER_BUF 4096

typedef struct {
    uint8_t  buf[MPPT_REASSEMBLER_BUF];
    uint16_t len;
    bool     busy;
    mppt_msg_type_t msg_type;
    uint16_t payload_len;
} mppt_reassembler_t;

void mppt_reassembler_reset(mppt_reassembler_t *r);

/*
 * Feed one BLE write chunk.
 * Returns true when a complete frame is ready.
 * Caller reads r->msg_type, r->buf+3, r->payload_len.
 */
bool mppt_reassembler_feed(mppt_reassembler_t *r, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif
