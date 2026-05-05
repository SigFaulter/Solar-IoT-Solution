#include <string.h>
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"

#include "ble_gatt.h"
#include "mppt_ble.h"
#include "mppt_publish.h"
#include "space_parser.h"

static const char *TAG = "MAIN";

/* ---------- Poll intervals ---------- */
#define TELE_INTERVAL_MS    30000U
#define EEPROM_INTERVAL_MS  300000U

/* ---------- Device state ---------- */
static SpaceTelemetry g_tele   = {0};
static EepromData     g_eeprom = {0};
static bool           g_tele_ok   = false;
static bool           g_eeprom_ok = false;
static uint8_t        g_hw_version = 0;
static uint8_t        g_own_addr_type;

/* ---------- NimBLE callbacks ---------- */

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining address type; rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x",
             addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);

    ble_gatt_init(g_own_addr_type);
    ble_gatt_advertise();
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

/* ---------- Data Polling (Stubs) ---------- */

static void repoll_telemetry(void)
{
    static const char SPACE_LINE[] =
        "00000;00000;00000;00000;00000;000060;018500;014500;00000;003;000;00017;"
        "00001;000;13800;14000;100;+21;+22;+25;0;0;00002;00001;00000;00720;00000;"
        "00233;00000;0;0;00049;00049;0000;0000;0000;000;000;000;000;0;1;\r\n";
    g_tele_ok = parse_space_line(SPACE_LINE, g_tele);
    g_tele.hw_version = g_hw_version;
}

static bool repoll_eeprom(void)
{
    static const char EEPROM_LINE[] = "!00;F2;10;E8;A1;52;41;D0;10;DE;03;E8;00;C8;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;18;00;01;00;00;00;00;2E;7C;2A;F8;02;00;78;00;3C;2E;7C;2A;F8;00;00;00;00;00;2E;7C;2A;F8;01;01;1E;78;1E;18;09;C4;1E;46;3C;8C;39;D0;39;D0;38;40;37;78;35;E8;32;C8;32;00;29;04;30;0C;2F;44;00;02;00;80;00;01;00;00;00;00;00;00;00;00;00;00;00;00;31;03;05;13;06;25;20;18;00;56;00;00;00;00;00;00;00;00;17;00;00;00;54;00;00;2A;68;00;00;2E;43;00;31;74;74;00;00;00;00;13;00;00;00;00;1B;14;4C;00;21;74;74;00;00;00;00;13;00;00;00;00;1B;12;4C;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;0F;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1C;10;4C;00;21;74;73;00;00;00;00;14;00;00;00;00;1C;11;4C;00;21;73;73;00;00;00;00;14;00;00;00;00;1C;11;4B;00;21;73;72;00;00;00;00;13;00;00;00;00;21;0F;4C;00;21;73;72;00;00;00;00;13;00;00;00;00;1D;15;4C;00;21;72;72;00;00;00;00;14;00;00;00;00;18;11;4C;00;21;72;71;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;17;0C;4C;00;21;70;6F;00;00;00;00;13;00;00;00;00;1A;0C;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;20;0B;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;1F;10;4D;00;21;6E;6D;00;00;00;00;12;00;00;00;00;17;11;4E;00;21;6D;6B;00;00;00;00;13;00;00;00;00;17;0E;4F;00;21;6C;6A;00;00;00;00;13;00;00;00;00;1A;10;4E;00;21;6A;68;00;00;00;00;14;00;00;00;00;17;11;4E;00;21;8C;7E;02;2D;01;39;13;00;15;0D;04;20;11;49;00;02;8C;7B;02;4C;02;46;13;00;16;0E;02;20;12;48;00;02;84;7B;00;00;02;44;13;00;16;00;01;1D;13;49;00;00;83;80;00;00;00;85;13;00;04;00;00;1D;12;49;00;00;83;80;00;00;00;89;13;00;04;00;00;1E;15;49;00;00;82;7F;00;00;00;89;13;00;04;00;00;1F;13;4A;00;00;80;7D;00;00;00;89;13;00;04;00;00;1E;13;4A;00;00;7E;6D;00;00;00;77;14;00;05;00;00;1B;12;4A;00;21;74;72;00;00;00;00;13;00;00;00;00;1C;12;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;11;4B;00;21;74;74;00;00;00;00;13;00;00;00;00;20;12;4B;00;21;8C;6D;04;3D;04;A0;14;00;17;13;02;2E;10;43;00;23;\r\n";
    g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);
    return g_eeprom_ok;
}

/* ---------- BLE RX Callback ---------- */

static mppt_reassembler_t s_rx_asm;

static void on_ble_rx(const uint8_t *data, uint16_t len)
{
    if (mppt_reassembler_feed(&s_rx_asm, data, len)) {
        if (s_rx_asm.msg_type == MSG_CMD) {
            handle_command(s_rx_asm.buf + 3, s_rx_asm.payload_len);
        }
    }
}

/* ---------- Main Task ---------- */

static void main_loop_task(void *arg)
{
    esp_task_wdt_add(NULL);

    bool burst_sent = false;
    uint32_t last_tele_ms   = 0;
    uint32_t last_eeprom_ms = 0;

    for (;;) {
        esp_task_wdt_reset();

        if (ble_gatt_conn_handle() == BLE_HS_CONN_HANDLE_NONE || !ble_gatt_subscribed()) {
            burst_sent = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        if (!burst_sent) {
            ESP_LOGI(TAG, "New connection, sending initial data burst");
            publish_device_info();
            vTaskDelay(pdMS_TO_TICKS(50));
            publish_device_settings();
            vTaskDelay(pdMS_TO_TICKS(50));
            publish_fault_status(true);
            vTaskDelay(pdMS_TO_TICKS(50));
            publish_datalog(true);
            burst_sent      = true;
            last_tele_ms    = now_ms;
            last_eeprom_ms  = now_ms;
            continue;
        }

        if (now_ms - last_tele_ms >= TELE_INTERVAL_MS) {
            last_tele_ms = now_ms;
            repoll_telemetry();
            if (g_tele_ok) {
                publish_telemetry();
                publish_fault_status(false);
            }
        }

        if (now_ms - last_eeprom_ms >= EEPROM_INTERVAL_MS) {
            last_eeprom_ms = now_ms;
            if (repoll_eeprom()) {
                publish_datalog(false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main(void)
{
    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Initialize NimBLE */
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb = ble_on_reset;
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Power management & WDT */
    esp_pm_config_t pm = {
        .max_freq_mhz      = 80,
        .min_freq_mhz      = 10,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm));

    esp_task_wdt_config_t wdt = {
        .timeout_ms    = 10000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_task_wdt_reconfigure(&wdt);

    /* Initial Data */
    repoll_telemetry();
    repoll_eeprom();

    if (g_eeprom_ok && g_eeprom.device_identifier != 0) {
        g_hw_version = ((g_eeprom.device_identifier & 0xFF) == 0x52) ? 2 : 3;
    } else if (g_tele_ok) {
        g_hw_version = g_tele.hw_version;
    } else {
        g_hw_version = 3;
    }
    g_tele.hw_version = g_hw_version;

    /* Initialize Modules */
    publish_init(&g_tele, &g_eeprom, &g_tele_ok, &g_eeprom_ok, &g_hw_version);
    mppt_reassembler_reset(&s_rx_asm);
    ble_gatt_set_rx_cb(on_ble_rx);

    /* Start Tasks */
    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(main_loop_task, "mppt_loop", 4096, NULL, 5, NULL);
}
