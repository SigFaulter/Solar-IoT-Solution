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

#include "ble_gatt.h"
#include "mppt_ble.h"
#include "mppt_publish.h"
#include "space_parser.h"   /* SpaceTelemetry, EepromData, parse_* */

static const char *TAG = "MAIN";

/* ---------- Poll intervals ---------- */
#define TELE_INTERVAL_MS    30000U   /* production: 30 s */
#define EEPROM_INTERVAL_MS  300000U  /* production: 5 min */

/* ---------- Device state ---------- */
static SpaceTelemetry g_tele   = {0};
static EepromData     g_eeprom = {0};
static bool           g_tele_ok   = false;
static bool           g_eeprom_ok = false;
static uint8_t        g_hw_version = 0;

/* ---------- Test stubs (replace with real UART calls) ---------- */

/* TODO: send ' ' over UART2, parse response into g_tele */
static void repoll_telemetry(void)
{
    static const char SPACE_LINE[] =
        "00000;00000;00000;00000;00000;000060;018500;014500;00000;003;000;00017;"
        "00001;000;13800;14000;100;+21;+22;+25;0;0;00002;00001;00000;00720;00000;"
        "00233;00000;0;0;00049;00049;0000;0000;0000;000;000;000;000;0;1;\r\n";
    g_tele_ok = parse_space_line(SPACE_LINE, g_tele);
    g_tele.hw_version = g_hw_version;
}

/* TODO: send '!' over UART2, parse response into g_eeprom; return true on new data */
static bool repoll_eeprom(void)
{
    static const char EEPROM_LINE[] = "!00;F2;10;E8;A1;52;41;D0;10;DE;03;E8;00;C8;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;18;00;01;00;00;00;00;2E;7C;2A;F8;02;00;78;00;3C;2E;7C;2A;F8;00;00;00;00;00;2E;7C;2A;F8;01;01;1E;78;1E;18;09;C4;1E;46;3C;8C;39;D0;39;D0;38;40;37;78;35;E8;32;C8;32;00;29;04;30;0C;2F;44;00;02;00;80;00;01;00;00;00;00;00;00;00;00;00;00;00;00;31;03;05;13;06;25;20;18;00;56;00;00;00;00;00;00;00;00;17;00;00;00;54;00;00;2A;68;00;00;2E;43;00;31;74;74;00;00;00;00;13;00;00;00;00;1B;14;4C;00;21;74;74;00;00;00;00;13;00;00;00;00;1B;12;4C;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;0F;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1C;10;4C;00;21;74;73;00;00;00;00;14;00;00;00;00;1C;11;4C;00;21;73;73;00;00;00;00;14;00;00;00;00;1C;11;4B;00;21;73;72;00;00;00;00;13;00;00;00;00;21;0F;4C;00;21;73;72;00;00;00;00;13;00;00;00;00;1D;15;4C;00;21;72;72;00;00;00;00;14;00;00;00;00;18;11;4C;00;21;72;71;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;17;0C;4C;00;21;70;6F;00;00;00;00;13;00;00;00;00;1A;0C;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;20;0B;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;1F;10;4D;00;21;6E;6D;00;00;00;00;12;00;00;00;00;17;11;4E;00;21;6D;6B;00;00;00;00;13;00;00;00;00;17;0E;4F;00;21;6C;6A;00;00;00;00;13;00;00;00;00;1A;10;4E;00;21;6A;68;00;00;00;00;14;00;00;00;00;17;11;4E;00;21;8C;7E;02;2D;01;39;13;00;15;0D;04;20;11;49;00;02;8C;7B;02;4C;02;46;13;00;16;0E;02;20;12;48;00;02;84;7B;00;00;02;44;13;00;16;00;01;1D;13;49;00;00;83;80;00;00;00;85;13;00;04;00;00;1D;12;49;00;00;83;80;00;00;00;89;13;00;04;00;00;1E;15;49;00;00;82;7F;00;00;00;89;13;00;04;00;00;1F;13;4A;00;00;80;7D;00;00;00;89;13;00;04;00;00;1E;13;4A;00;00;7E;6D;00;00;00;77;14;00;05;00;00;1B;12;4A;00;21;74;72;00;00;00;00;13;00;00;00;00;1C;12;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;11;4B;00;21;74;74;00;00;00;00;13;00;00;00;00;20;12;4B;00;21;8C;6D;04;3D;04;A0;14;00;17;13;02;2E;10;43;00;23;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;54;47;08;00;FE;FE;90;FE;07;00;FF;FF;FF;FF;00;00;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;39;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;03;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;51;FF;FF;FF;FF;00;00;3B;0D;00;00;3C;9C;00;3E;D7;80;02;02;00;00;00;00;00;00;0C;00;00;11;00;00;11;00;20;53;52;00;20;53;52;07;05;05;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;";
    g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);
    return g_eeprom_ok;
}

/* ---------- RX callback (called from BLE context) ---------- */

static mppt_reassembler_t s_rx_asm;

static void on_rx(const uint8_t *data, uint16_t len)
{
    if (mppt_reassembler_feed(&s_rx_asm, data, len)) {
        if (s_rx_asm.msg_type == MSG_CMD) {
            handle_command(s_rx_asm.buf + 3, (uint16_t)s_rx_asm.payload_len);
        }
    }
}

/* ---------- NimBLE host task ---------- */

static void ble_host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ---------- Main polling loop ---------- */

static void main_loop_task(void *arg)
{
    esp_task_wdt_add(NULL);

    /* Initial burst: do a full poll before entering the loop */
    repoll_telemetry();
    repoll_eeprom();

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

/* ---------- app_main ---------- */

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) ESP_LOGE(TAG, "NVS erase failed: %d", ret);
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Operation failed with error %d", ret);
    }

    /* Power management: 80 MHz max, 10 MHz min, light sleep enabled */
    /*
    esp_pm_config_t pm = {
        .max_freq_mhz      = 80,
        .min_freq_mhz      = 10,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm));
    */

    /* WDT: 10 s timeout, panic on trigger */
    esp_task_wdt_config_t wdt = {
        .timeout_ms    = 10000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    /* Ignore error - WDT may not be enabled in sdkconfig */
    esp_task_wdt_reconfigure(&wdt);

    /* Initial data parse (stub - replace with real UART) */
    repoll_telemetry();
    repoll_eeprom();

    /* Determine hardware version from EEPROM (authoritative), fall back to tele */
    if (g_eeprom_ok && g_eeprom.device_identifier != 0) {
        g_hw_version = ((g_eeprom.device_identifier & 0xFF) == 0x52) ? 2 : 3;
    } else if (g_tele_ok) {
        g_hw_version = g_tele.hw_version;
    } else {
        g_hw_version = 3;
    }
    g_tele.hw_version = g_hw_version;

    /* Wire up publish module */
    publish_init(&g_tele, &g_eeprom, &g_tele_ok, &g_eeprom_ok, &g_hw_version);

    /* Wire up BLE RX -> reassembler -> command handler */
    mppt_reassembler_reset(&s_rx_asm);
    ble_gatt_set_rx_cb(on_rx);

    /* Init GATT service and start NimBLE host */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
    }
    ble_gatt_init();
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(main_loop_task, "mppt_loop", 4096, NULL, 5, NULL);
}
