#pragma once

#include <cstdint>
#include "host/ble_hs.h"

typedef void (*gatt_svr_rx_cb_t)(const uint8_t *data, uint16_t len);

int gatt_svr_init(void);
void gatt_svr_set_rx_cb(gatt_svr_rx_cb_t cb);

extern uint16_t g_nus_tx_char_handle;
extern uint16_t g_nus_rx_char_handle;
