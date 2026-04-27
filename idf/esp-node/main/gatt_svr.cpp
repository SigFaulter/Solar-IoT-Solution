#include "gatt_svr.h"
#include <cstring>
#include "host/ble_hs.h"
#include "services/gatt/ble_svc_gatt.h"

uint16_t g_nus_tx_char_handle;
uint16_t g_nus_rx_char_handle;

static gatt_svr_rx_cb_t g_rx_cb = nullptr;

// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t gatt_svr_svc_nus_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

// 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (RX)
static const ble_uuid128_t gatt_svr_char_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

// 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (TX)
static const ble_uuid128_t gatt_svr_char_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static int
gatt_svr_chr_access_nus_rx(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);

static int
gatt_svr_chr_access_nus_tx(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg);

static const struct ble_gatt_chr_def gatt_svr_nus_chars[] = {
    {
        /* Characteristic: RX */
        .uuid = &gatt_svr_char_rx_uuid.u,
        .access_cb = gatt_svr_chr_access_nus_rx,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = &g_nus_rx_char_handle,
        .cpfd = NULL,
    },
    {
        /* Characteristic: TX */
        .uuid = &gatt_svr_char_tx_uuid.u,
        .access_cb = gatt_svr_chr_access_nus_tx,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
        .min_key_size = 0,
        .val_handle = &g_nus_tx_char_handle,
        .cpfd = NULL,
    },
    {
        .uuid = NULL,
        .access_cb = NULL,
        .arg = NULL,
        .descriptors = NULL,
        .flags = 0,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL,
    }
};

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: Nordic UART Service (NUS) */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_nus_uuid.u,
        .includes = NULL,
        .characteristics = gatt_svr_nus_chars,
    },
    {
        .type = 0,
        .uuid = NULL,
        .includes = NULL,
        .characteristics = NULL,
    },
};

static int
gatt_svr_chr_access_nus_rx(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (g_rx_cb) {
            // Flatten mbuf to call callback
            uint8_t buf[512];
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > sizeof(buf)) len = sizeof(buf);
            os_mbuf_copydata(ctxt->om, 0, len, buf);
            g_rx_cb(buf, len);
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_access_nus_tx(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt,
                           void *arg)
{
    return 0;
}

void
gatt_svr_set_rx_cb(gatt_svr_rx_cb_t cb)
{
    g_rx_cb = cb;
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
