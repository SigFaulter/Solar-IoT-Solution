#include "ble_gatt.h"
#include <string.h>
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_GATT";

/* Nordic UART Service UUIDs */
static const ble_uuid128_t s_svc_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,
                     0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e);

static uint16_t  s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t  s_tx_handle;
static bool      s_subscribed;
static ble_rx_cb_t s_rx_cb;
static uint8_t   s_own_addr_type;

/* ---------- GATT access callback ---------- */

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;
    if (s_rx_cb) s_rx_cb(ctxt->om->om_data, ctxt->om->om_len);
    return 0;
}

/* ---------- GATT service table ---------- */

static const struct ble_gatt_chr_def s_chrs[] = {
    {
        .uuid       = &s_rx_uuid.u,
        .access_cb  = gatt_access_cb,
        .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = NULL,
    },
    {
        .uuid       = &s_tx_uuid.u,
        .access_cb  = NULL,
        .flags      = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_handle,
    },
    { .uuid = NULL }
};

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = s_chrs,
    },
    { .type = 0 }
};

/* ---------- GAP event handler ---------- */

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_subscribed  = false;
            ESP_LOGI(TAG, "Connected handle=%d", s_conn_handle);

            /* Request power-saving connection interval (600 ms) */
            struct ble_gap_upd_params upd = {
                .itvl_min          = 480,
                .itvl_max          = 480,
                .latency           = 0,
                .supervision_timeout = 300,
                .min_ce_len        = 0,
                .max_ce_len        = 0,
            };
            ble_gap_update_params(s_conn_handle, &upd);
        } else {
            ble_gatt_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed  = false;
        ble_gatt_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_handle) {
            s_subscribed = (event->subscribe.cur_notify != 0);
            ESP_LOGI(TAG, "TX subscribed=%d", s_subscribed);
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated=%d", event->mtu.value);
        break;

    default:
        break;
    }
    return 0;
}

/* Called by NimBLE host when stack is synced - forward decl */
static void on_sync(void);

/* ---------- Public API ---------- */

void ble_gatt_init(void)
{
    int rc;

    /* These are often initialized by nimble_port_init or not needed to be called twice */
    // ble_svc_gap_init();
    // ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed; rc=%d", rc);
    }
    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed; rc=%d", rc);
    }

    ble_svc_gap_device_name_set("MPPT-Gateway");
    ble_hs_cfg.sync_cb = on_sync;
}

static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    ble_gatt_advertise();
}

void ble_gatt_advertise(void)
{
    ble_gap_adv_stop();

    struct ble_hs_adv_fields fields = {0};
    fields.flags             = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl        = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    const char *name         = "MPPT-Gateway";
    fields.name              = (const uint8_t *)name;
    fields.name_len          = strlen(name);
    fields.name_is_complete  = 1;
    ble_gap_adv_set_fields(&fields);

    struct ble_gap_adv_params adv = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min  = 1600,   /* 1000 ms */
        .itvl_max  = 1600,
    };
    ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv, gap_event_cb, NULL);
}

uint16_t ble_gatt_conn_handle(void) { return s_conn_handle; }
bool     ble_gatt_subscribed(void)  { return s_subscribed; }
uint16_t ble_gatt_tx_handle(void)   { return s_tx_handle; }

void ble_gatt_set_rx_cb(ble_rx_cb_t cb) { s_rx_cb = cb; }
