#include "mppt_publish.h"
#include "mppt_ble.h"
#include "proto_builder.h"
#include "mppt.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "PUBLISH";

/* Shared encode buffer - avoids 4 KB stack hit per send_proto call */
static uint8_t s_enc_buf[4096];

/* Pointers set by publish_init() */
static SpaceTelemetry *s_tele;
static EepromData      *s_eeprom;
static bool            *s_tele_ok;
static bool            *s_eeprom_ok;
static uint8_t         *s_hw_version;

/* Pending settings override (from CMD_SET_SETTINGS) */
static mppt_DeviceSettings s_override     = mppt_DeviceSettings_init_zero;
static bool                s_has_override = false;

/* Delta tracking for incremental datalog sends */
static uint8_t  s_last_sent_days   = 0;
static uint8_t  s_last_sent_months = 0;
static uint32_t s_last_fault_mask  = 0xFFFFFFFFu;

static uint32_t now_sec(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

static bool encode_and_send(mppt_msg_type_t type,
                            const pb_msgdesc_t *fields,
                            const void *msg)
{
    pb_ostream_t stream = pb_ostream_from_buffer(s_enc_buf, sizeof(s_enc_buf));
    if (!pb_encode(&stream, fields, msg)) {
        ESP_LOGE(TAG, "encode failed type=0x%02x: %s", type, PB_GET_ERROR(&stream));
        return false;
    }
    mppt_ble_send(type, s_enc_buf, (uint16_t)stream.bytes_written);
    return true;
}

/* Nanopb string decode callback */
static bool read_str_cb(pb_istream_t *stream, const pb_field_t *field, void **arg)
{
    char *buf = (char *)*arg;
    size_t n  = stream->bytes_left < 63 ? stream->bytes_left : 63;
    if (!pb_read(stream, (pb_byte_t *)buf, n)) return false;
    buf[n] = '\0';
    return true;
}

/* ---------- Public init ---------- */

void publish_init(SpaceTelemetry *tele, EepromData *eeprom,
                  bool *tele_ok, bool *eeprom_ok,
                  uint8_t *hw_version)
{
    s_tele       = tele;
    s_eeprom     = eeprom;
    s_tele_ok    = tele_ok;
    s_eeprom_ok  = eeprom_ok;
    s_hw_version = hw_version;
}

/* ---------- Publish functions ---------- */

void publish_telemetry(void)
{
    if (!*s_tele_ok) return;
    mppt_Telemetry msg = build_telemetry(*s_tele, now_sec());
    encode_and_send(MSG_TELEMETRY, mppt_Telemetry_fields, &msg);
}

void publish_fault_status(bool force)
{
    if (!*s_tele_ok || *s_hw_version != 3) return;
    uint32_t cur = s_tele->fault_status;
    if (!force && cur == s_last_fault_mask) return;
    s_last_fault_mask = cur;

    mppt_FaultStatus msg = build_fault_status(*s_tele, now_sec());
    encode_and_send(MSG_FAULT_STATUS, mppt_FaultStatus_fields, &msg);
}

void publish_device_info(void)
{
    if (!*s_tele_ok || !*s_eeprom_ok) return;
    mppt_DeviceInfo msg = build_device_info(*s_tele, *s_eeprom, now_sec());
    encode_and_send(MSG_DEVICE_INFO, mppt_DeviceInfo_fields, &msg);
}

void publish_device_settings(void)
{
    if (!*s_eeprom_ok) return;
    mppt_DeviceSettings msg = build_device_settings(*s_eeprom, s_has_override ? &s_override : NULL, now_sec());
    encode_and_send(MSG_SETTINGS, mppt_DeviceSettings_fields, &msg);
}

void publish_datalog(bool send_all)
{
    if (!*s_eeprom_ok) return;

    // Daily
    uint8_t d_count = s_eeprom->daily_count;
    if (d_count > 0 && (send_all || d_count != s_last_sent_days)) {
        LogArg arg = { s_eeprom->daily_logs, 0, d_count };
        mppt_DataloggerPayload msg = mppt_DataloggerPayload_init_zero;
        msg.timestamp = now_sec();
        msg.recorded_days = s_eeprom->num_days;
        msg.days_with_lvd = s_eeprom->days_with_lvd;
        msg.months_without_full_charge = s_eeprom->months_without_full_charge;
        msg.avg_morning_soc_pct = s_eeprom->avg_morning_soc_pct;
        msg.total_ah_charge_mah = s_eeprom->total_ah_charge_mah;
        msg.total_ah_load_mah = s_eeprom->total_ah_load_mah;

        msg.daily_logs.funcs.encode = encode_log_entries;
        msg.daily_logs.arg = &arg;
        encode_and_send(MSG_DATALOG, mppt_DataloggerPayload_fields, &msg);
        s_last_sent_days = d_count;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Monthly
    uint8_t m_count = s_eeprom->monthly_count;
    if (m_count > 0 && (send_all || m_count != s_last_sent_months)) {
        LogArg arg = { s_eeprom->monthly_logs, 0, m_count };
        mppt_DataloggerPayload msg = mppt_DataloggerPayload_init_zero;
        msg.timestamp = now_sec();
        msg.recorded_days = s_eeprom->num_days;
        msg.days_with_lvd = s_eeprom->days_with_lvd;
        msg.months_without_full_charge = s_eeprom->months_without_full_charge;
        msg.avg_morning_soc_pct = s_eeprom->avg_morning_soc_pct;
        msg.total_ah_charge_mah = s_eeprom->total_ah_charge_mah;
        msg.total_ah_load_mah = s_eeprom->total_ah_load_mah;

        msg.monthly_logs.funcs.encode = encode_log_entries;
        msg.monthly_logs.arg = &arg;
        encode_and_send(MSG_DATALOG, mppt_DataloggerPayload_fields, &msg);
        s_last_sent_months = m_count;
    }
}

/* ---------- Command Handling ---------- */

void handle_command(const uint8_t *data, uint16_t len)
{
    static char req_id[64];
    req_id[0] = '\0';

    mppt_ControlCommand cmd = mppt_ControlCommand_init_zero;
    cmd.request_id.funcs.decode = read_str_cb;
    cmd.request_id.arg          = req_id;

    pb_istream_t stream = pb_istream_from_buffer(data, len);
    if (!pb_decode(&stream, mppt_ControlCommand_fields, &cmd)) {
        ESP_LOGE(TAG, "CMD decode failed");
        return;
    }

    bool        ok     = false;
    const char *reason = "unknown payload";

    switch (cmd.which_payload) {
    case mppt_ControlCommand_set_settings_tag: {
        if (!s_has_override) {
            s_override     = build_device_settings(*s_eeprom, NULL, 0);
            s_has_override = true;
        }
        const mppt_DeviceSettings *ps = &cmd.payload.set_settings;
#define APPLY(field) if (ps->has_##field) s_override.field = ps->field
        APPLY(battery_type);
        APPLY(capacity_ah);
        APPLY(lvd_voltage_mv);
        APPLY(lvd_mode);
        APPLY(night_mode);
        APPLY(evening_minutes);
        APPLY(morning_minutes);
        APPLY(night_threshold_mv);
        APPLY(dimming_mode);
        APPLY(evening_minutes_dimming);
        APPLY(morning_minutes_dimming);
        APPLY(dimming_pct);
        APPLY(base_dimming_pct);
        APPLY(advanced_flags);
#undef APPLY
        ok     = true;
        reason = "settings applied";
        break;
    }
    case mppt_ControlCommand_switch_load_tag:
        ok     = true;
        reason = (cmd.payload.switch_load.flags & 1u) ? "load on" : "load off";
        break;
    }

    mppt_CommandAck ack          = mppt_CommandAck_init_zero;
    ack.ok                       = ok;
    ack.timestamp                = now_sec();
    ack.request_id.funcs.encode  = write_string_callback;
    ack.request_id.arg           = req_id;
    ack.reason.funcs.encode      = write_string_callback;
    ack.reason.arg               = (void *)reason;
    encode_and_send(MSG_ACK, mppt_CommandAck_fields, &ack);

    if (ok && cmd.which_payload == mppt_ControlCommand_set_settings_tag) {
        vTaskDelay(pdMS_TO_TICKS(50));
        publish_device_settings();
    }
}
