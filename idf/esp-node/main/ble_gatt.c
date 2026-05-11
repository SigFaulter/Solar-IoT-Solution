#include "ble_gatt.h"
#include <string.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/ram/ble_store_ram.h"

static const char *TAG = "BLE_GATT";

/* Nordic UART Service UUIDs */
static const ble_uuid128_t s_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_tx_handle;
static bool s_subscribed;
static ble_rx_cb_t s_rx_cb;
static uint8_t s_own_addr_type;
static char s_device_name[32];

/* ---------- Forward Declarations ---------- */
static void ble_on_sync(void);
static void ble_on_reset(int reason);
static void ble_host_task(void *arg);

/* ---------- GATT access callback ---------- */

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (s_rx_cb) {
            uint8_t buf[512];
            uint16_t len = sizeof(buf);
            int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len);
            if (rc == 0) {
                s_rx_cb(buf, len);
            }
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* ---------- GATT service table ---------- */

static const struct ble_gatt_chr_def s_chrs[] = {
    {
        /* Characteristic: RX (Write) */
        .uuid = &s_rx_uuid.u,
        .access_cb = gatt_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        /* Characteristic: TX (Notify) */
        .uuid = &s_tx_uuid.u,
        .access_cb = gatt_access_cb,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &s_tx_handle,
    },
    { 0 }
};

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        /* Service: Nordic UART */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = s_chrs,
    },
    { 0 },
};

/* ---------- GAP event handler ---------- */

static void
ble_gatt_on_register(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "registered characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle,
                 ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGI(TAG, "registered descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    default:
        break;
    }
}

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_subscribed = false;
            ESP_LOGI(TAG, "Connected handle=%d", s_conn_handle);

            struct ble_gap_upd_params upd = {
                .itvl_min = 480, /* 600ms */
                .itvl_max = 480,
                .latency = 0,
                .supervision_timeout = 300,
            };
            ble_gap_update_params(s_conn_handle, &upd);
        } else {
            ble_gatt_advertise(s_own_addr_type);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_subscribed = false;
        ble_gatt_advertise(s_own_addr_type);
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_handle) {
            s_subscribed = (event->subscribe.cur_notify != 0);
            ESP_LOGI(TAG, "TX subscribed=%d", s_subscribed);
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated=%d", event->mtu.value);
        return 0;
    }
    return 0;
}

/* ---------- NimBLE callbacks ---------- */

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(s_own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);

    ble_gatt_advertise(s_own_addr_type);
}

static void ble_on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting BLE state; reason=%d", reason);
}

static void ble_host_task(void *arg)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------- Public API ---------- */

void ble_gatt_advertise(uint8_t own_addr_type)
{
    s_own_addr_type = own_addr_type;
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = 1600, /* 1000ms */
        .itvl_max = 1600,
    };

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)s_device_name;
    fields.name_len = strlen(s_device_name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error enabling advertisement; rc=%d", rc);
    }
}

void ble_gatt_init(void)
{
    int rc;

    rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.gatts_register_cb = ble_gatt_on_register;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(s_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    /* Initialize the NimBLE store. */
    ble_store_ram_init();
}

void ble_gatt_start(void)
{
    nimble_port_freertos_init(ble_host_task);
}

void ble_gatt_set_name(const char *name)
{
    strncpy(s_device_name, name, sizeof(s_device_name) - 1);
    s_device_name[sizeof(s_device_name) - 1] = '\0';
    ble_svc_gap_device_name_set(s_device_name);
}

uint16_t ble_gatt_conn_handle(void) { return s_conn_handle; }
bool ble_gatt_subscribed(void) { return s_subscribed; }
uint16_t ble_gatt_tx_handle(void) { return s_tx_handle; }
void ble_gatt_set_rx_cb(ble_rx_cb_t cb) { s_rx_cb = cb; }
