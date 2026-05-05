#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Called after nimble_port_init(); registers GATT services */
void ble_gatt_init(uint8_t own_addr_type);

/* Start advertising; safe to call repeatedly (stops first) */
void ble_gatt_advertise(void);

/* Current connection handle; BLE_HS_CONN_HANDLE_NONE if disconnected */
uint16_t ble_gatt_conn_handle(void);

/* True when the peer has subscribed to TX notifications */
bool ble_gatt_subscribed(void);

/* TX characteristic handle (needed by ble_gatts_notify_custom) */
uint16_t ble_gatt_tx_handle(void);

/* Callback registered by the application to receive RX writes */
typedef void (*ble_rx_cb_t)(const uint8_t *data, uint16_t len);
void ble_gatt_set_rx_cb(ble_rx_cb_t cb);

#ifdef __cplusplus
}
#endif
