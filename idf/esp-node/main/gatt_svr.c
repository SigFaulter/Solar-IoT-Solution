/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"

uint16_t g_nus_tx_char_handle;
uint16_t g_nus_rx_char_handle;

static nus_rx_cb_t g_nus_rx_cb = NULL;

static const ble_uuid128_t gatt_svr_svc_nus_uuid =
    BLE_UUID128_INIT(GATT_SVR_SVC_NUS_UUID_128);

static const ble_uuid128_t gatt_svr_chr_nus_tx_uuid =
    BLE_UUID128_INIT(GATT_SVR_CHR_NUS_TX_UUID_128);

static const ble_uuid128_t gatt_svr_chr_nus_rx_uuid =
    BLE_UUID128_INIT(GATT_SVR_CHR_NUS_RX_UUID_128);

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt,
                void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** NUS Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_nus_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /*** TX Characteristic ***/
                .uuid = &gatt_svr_chr_nus_tx_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
                .val_handle = &g_nus_tx_char_handle,
            }, {
                /*** RX Characteristic ***/
                .uuid = &gatt_svr_chr_nus_rx_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &g_nus_rx_char_handle,
            }, {
                0, /* No more characteristics in this service. */
            }
        },
    },

    {
        0, /* No more services. */
    },
};

static int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    if (attr_handle == g_nus_tx_char_handle) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            // NUS TX is usually just for notify, but we support read too
            return 0;
        }
    } else if (attr_handle == g_nus_rx_char_handle) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if (g_nus_rx_cb) {
                uint8_t buf[256]; // Adjust size as needed
                uint16_t len;
                rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
                if (rc == 0) {
                    g_nus_rx_cb(buf, len);
                }
            }
            return 0;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

void gatt_svr_set_rx_cb(nus_rx_cb_t cb) {
    g_nus_rx_cb = cb;
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
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
