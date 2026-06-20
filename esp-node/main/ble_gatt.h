#pragma once
#include <stdint.h>
#include <stdbool.h>


/* Initialize BLE stack and GATT services */
void ble_gatt_init(void);

/* Start NimBLE host task */
void ble_gatt_start(void);

/* Set device name for advertising (must call before ble_gatt_start) */
void ble_gatt_set_name(const char *name);

/* Start advertising */
void ble_gatt_advertise(uint8_t own_addr_type);

/* Current connection handle; BLE_HS_CONN_HANDLE_NONE if disconnected */
uint16_t ble_gatt_conn_handle(void);

/* True when the peer has subscribed to TX notifications */
bool ble_gatt_subscribed(void);

/* TX characteristic handle (needed by ble_gatts_notify_custom) */
uint16_t ble_gatt_tx_handle(void);

/* Callback registered by the application to receive RX writes */
typedef void (*ble_rx_cb_t)(const uint8_t *data, uint16_t len);
void ble_gatt_set_rx_cb(ble_rx_cb_t cb);
