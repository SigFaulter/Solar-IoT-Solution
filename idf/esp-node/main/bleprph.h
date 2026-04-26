/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef H_BLEPRPH_
#define H_BLEPRPH_

#include <stdbool.h>
#include "nimble/ble.h"
#include "modlog/modlog.h"
#include "esp_peripheral.h"
#ifdef __cplusplus
extern "C" {
#endif

struct ble_hs_cfg;
struct ble_gatt_register_ctxt;

/** NUS Service UUID: 6e400001-b5a3-f393-e0a9-e50e24dcca9e */
#define GATT_SVR_SVC_NUS_UUID_128 \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e

/** NUS TX Characteristic UUID: 6e400003-b5a3-f393-e0a9-e50e24dcca9e */
#define GATT_SVR_CHR_NUS_TX_UUID_128 \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e

/** NUS RX Characteristic UUID: 6e400002-b5a3-f393-e0a9-e50e24dcca9e */
#define GATT_SVR_CHR_NUS_RX_UUID_128 \
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, \
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e

extern uint16_t g_nus_tx_char_handle;
extern uint16_t g_nus_rx_char_handle;

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
int gatt_svr_init(void);

// Callback for RX data
typedef void (*nus_rx_cb_t)(const uint8_t *data, uint16_t len);
void gatt_svr_set_rx_cb(nus_rx_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif
