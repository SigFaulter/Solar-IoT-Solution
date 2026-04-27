#include <cstdio>
#include <cstring>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#include "ble_transport.h"
#include "mppt.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto_builder.h"
#include "space_parser.h"

static const char *tag = "MPPT-Gateway";

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
uint16_t g_nus_tx_char_handle;
static BleReassembler g_rx_asm;
static bool g_subscribed = false;

// NUS UUIDs: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t g_nus_svc_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t g_nus_rx_char_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

static const ble_uuid128_t g_nus_tx_char_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static int gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def g_gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &g_nus_svc_uuid.u,
        .includes = NULL,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &g_nus_rx_char_uuid.u,
                .access_cb = gatt_svr_access,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .min_key_size = 0,
                .val_handle = NULL,
                .cpfd = NULL,
            },
            {
                .uuid = &g_nus_tx_char_uuid.u,
                .access_cb = gatt_svr_access,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_NOTIFY,
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
        },
    },
    {
        .type = 0,
        .uuid = NULL,
        .includes = NULL,
        .characteristics = NULL,
    }
};

static void on_nus_rx(const uint8_t *data, uint16_t len);

static int gatt_svr_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ble_uuid_cmp(ctxt->chr->uuid, &g_nus_rx_char_uuid.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            on_nus_rx(ctxt->om->om_data, ctxt->om->om_len);
            return 0;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}
static bool g_burst_sent = false;

static SpaceTelemetry g_tele{};
static EepromData g_eeprom{};
static bool g_tele_ok = false;
static bool g_eeprom_ok = false;

static uint8_t g_hw_version = 0;

static mppt_DeviceSettings g_override = mppt_DeviceSettings_init_zero;
static bool g_has_override = false;

static uint32_t g_last_fault_mask = 0xFFFFFFFFu;
static uint8_t g_last_sent_days = 0;
static uint8_t g_last_sent_months = 0;

static uint32_t g_last_tele_ms = 0;
static uint32_t g_last_eeprom_ms = 0;

static constexpr uint32_t TELE_INTERVAL_MS = 5000;
static constexpr uint32_t EEPROM_INTERVAL_MS = 15000;

// Testing data
static const char SPACE_LINE[] =
    "00000;00000;00000;00000;00000;000060;018500;014500;00000;003;000;00017;"
    "00001;000;13800;14000;100;+21;+22;+25;0;0;00002;00001;00000;00720;00000;"
    "00233;00000;0;0;00049;00049;0000;0000;0000;000;000;000;000;0;1;\r\n";

static const char EEPROM_LINE[] = "!00;F2;10;E8;A1;52;41;D0;10;DE;03;E8;00;C8;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;18;00;01;00;00;00;00;2E;7C;2A;F8;02;00;78;00;3C;2E;7C;2A;F8;00;00;00;00;00;2E;7C;2A;F8;01;01;1E;78;1E;18;09;C4;1E;46;3C;8C;39;D0;39;D0;38;40;37;78;35;E8;32;C8;32;00;29;04;30;0C;2F;44;00;02;00;80;00;01;00;00;00;00;00;00;00;00;00;00;00;00;31;03;05;13;06;25;20;18;00;56;00;00;00;00;00;00;00;00;17;00;00;00;54;00;00;2A;68;00;00;2E;43;00;31;74;74;00;00;00;00;13;00;00;00;00;1B;14;4C;00;21;74;74;00;00;00;00;13;00;00;00;00;1B;12;4C;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;0F;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1C;10;4C;00;21;74;73;00;00;00;00;14;00;00;00;00;1C;11;4C;00;21;73;73;00;00;00;00;14;00;00;00;00;1C;11;4B;00;21;73;72;00;00;00;00;13;00;00;00;00;21;0F;4C;00;21;73;72;00;00;00;00;13;00;00;00;00;1D;15;4C;00;21;72;72;00;00;00;00;14;00;00;00;00;18;11;4C;00;21;72;71;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;19;0D;4C;00;21;71;70;00;00;00;00;13;00;00;00;00;17;0C;4C;00;21;70;6F;00;00;00;00;13;00;00;00;00;1A;0C;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;20;0B;4C;00;21;6F;6E;00;00;00;00;13;00;00;00;00;1F;10;4D;00;21;6E;6D;00;00;00;00;12;00;00;00;00;17;11;4E;00;21;6D;6B;00;00;00;00;13;00;00;00;00;17;0E;4F;00;21;6C;6A;00;00;00;00;13;00;00;00;00;1A;10;4E;00;21;6A;68;00;00;00;00;14;00;00;00;00;17;11;4E;00;21;8C;7E;02;2D;01;39;13;00;15;0D;04;20;11;49;00;02;8C;7B;02;4C;02;46;13;00;16;0E;02;20;12;48;00;02;84;7B;00;00;02;44;13;00;16;00;01;1D;13;49;00;00;83;80;00;00;00;85;13;00;04;00;00;1D;12;49;00;00;83;80;00;00;00;89;13;00;04;00;00;1E;15;49;00;00;82;7F;00;00;00;89;13;00;04;00;00;1F;13;4A;00;00;80;7D;00;00;00;89;13;00;04;00;00;1E;13;4A;00;00;7E;6D;00;00;00;77;14;00;05;00;00;1B;12;4A;00;21;74;72;00;00;00;00;13;00;00;00;00;1C;12;4B;00;21;74;73;00;00;00;00;13;00;00;00;00;1E;11;4B;00;21;74;74;00;00;00;00;13;00;00;00;00;20;12;4B;00;21;8C;6D;04;3D;04;A0;14;00;17;13;02;2E;10;43;00;23;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;54;47;08;00;FE;FE;90;FE;07;00;FF;FF;FF;FF;00;00;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;39;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;03;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;FF;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;51;FF;FF;FF;FF;00;00;3B;0D;00;00;3C;9C;00;3E;D7;80;02;02;00;00;00;00;00;00;0C;00;00;11;00;00;11;00;20;53;52;00;20;53;52;07;05;05;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;00;";

static uint32_t millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Proto helpers
static void send_proto(MpptMsgType msg_type, const pb_msgdesc_t *fields,
                       const void *proto_struct) {
  static uint8_t buf[4096];
  pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
  if (!pb_encode(&stream, fields, proto_struct)) {
    ESP_LOGE(tag, "[PROTO] encode error type=0x%02X: %s", msg_type,
           PB_GET_ERROR(&stream));
    return;
  }
  ble_send(g_conn_handle, msg_type, buf, stream.bytes_written);
}

static void publish_telemetry() {
  if (!g_tele_ok)
    return;
  auto msg = build_telemetry(g_tele, millis() / 1000u);
  send_proto(MSG_TELEMETRY, mppt_Telemetry_fields, &msg);
}

static void publish_fault_status(bool force = false) {
  if (!g_tele_ok || g_hw_version != 3)
    return;
  const uint32_t cur = g_tele.fault_status;
  if (!force && cur == g_last_fault_mask)
    return;
  g_last_fault_mask = cur;
  auto msg = build_fault_status(g_tele, millis() / 1000u);
  send_proto(MSG_FAULT_STATUS, mppt_FaultStatus_fields, &msg);
}

static void publish_device_info() {
  if (!g_tele_ok || !g_eeprom_ok) {
    ESP_LOGW(tag, "publish_device_info: data not ready");
    return;
  }
  auto msg = build_device_info(g_tele, g_eeprom, millis() / 1000u);
  send_proto(MSG_DEVICE_INFO, mppt_DeviceInfo_fields, &msg);
}

static void publish_device_settings() {
  if (!g_eeprom_ok)
    return;
  auto msg = build_device_settings(
      g_eeprom, g_has_override ? &g_override : nullptr, millis() / 1000u);
  send_proto(MSG_SETTINGS, mppt_DeviceSettings_fields, &msg);
}

static void publish_datalog(bool send_all) {
  if (!g_eeprom_ok)
    return;

  mppt_DataloggerPayload summary = mppt_DataloggerPayload_init_zero;
  summary.timestamp = millis() / 1000u;
  summary.recorded_days = g_eeprom.num_days;
  summary.days_with_lvd = g_eeprom.days_with_lvd;
  summary.months_without_full_charge = g_eeprom.months_without_full_charge;
  summary.avg_morning_soc_pct = g_eeprom.avg_morning_soc_pct;
  summary.total_ah_charge_mah = g_eeprom.total_ah_charge_mah;
  summary.total_ah_load_mah = g_eeprom.total_ah_load_mah;
  send_proto(MSG_DATALOG, mppt_DataloggerPayload_fields, &summary);

  uint8_t day_start = send_all ? 0 : g_last_sent_days;
  if (g_eeprom.daily_count > day_start) {
    mppt_DataloggerPayload dl = mppt_DataloggerPayload_init_zero;
    dl.timestamp = millis() / 1000u;
    LogArg d_arg = {g_eeprom.daily_logs, day_start,
                    (uint8_t)(g_eeprom.daily_count - day_start)};
    dl.daily_logs.funcs.encode = encode_log_entries;
    dl.daily_logs.arg = &d_arg;
    send_proto(MSG_DATALOG, mppt_DataloggerPayload_fields, &dl);
  }

  uint8_t mon_start = send_all ? 0 : g_last_sent_months;
  if (g_eeprom.monthly_count > mon_start) {
    mppt_DataloggerPayload ml = mppt_DataloggerPayload_init_zero;
    ml.timestamp = millis() / 1000u;
    LogArg m_arg = {g_eeprom.monthly_logs, mon_start,
                    (uint8_t)(g_eeprom.monthly_count - mon_start)};
    ml.monthly_logs.funcs.encode = encode_log_entries;
    ml.monthly_logs.arg = &m_arg;
    send_proto(MSG_DATALOG, mppt_DataloggerPayload_fields, &ml);
  }

  g_last_sent_days = g_eeprom.daily_count;
  g_last_sent_months = g_eeprom.monthly_count;
}

static bool decode_string_cb(pb_istream_t *stream, const pb_field_t *,
                             void **arg) {
  char *buf = (char *)*arg;
  size_t n = stream->bytes_left < 63 ? stream->bytes_left : 63;
  if (!pb_read(stream, (pb_byte_t *)buf, stream->bytes_left))
    return false;
  buf[n] = '\0';
  return true;
}

static void handle_command(const uint8_t *data, size_t len) {
  static char req_id_buf[64];
  req_id_buf[0] = '\0';

  mppt_ControlCommand cmd = mppt_ControlCommand_init_zero;
  cmd.request_id.funcs.decode = decode_string_cb;
  cmd.request_id.arg = req_id_buf;

  pb_istream_t stream = pb_istream_from_buffer(data, len);
  if (!pb_decode(&stream, mppt_ControlCommand_fields, &cmd))
    return;

  bool ok = false;
  const char *reason = "unknown payload";

  switch (cmd.which_payload) {
  case mppt_ControlCommand_set_settings_tag: {
    const mppt_DeviceSettings &ps = cmd.payload.set_settings;
    if (!g_has_override) {
      g_override = build_device_settings(g_eeprom, nullptr, 0);
      g_has_override = true;
    }
    if (ps.has_battery_type)
      g_override.battery_type = ps.battery_type;
    if (ps.has_capacity_ah)
      g_override.capacity_ah = ps.capacity_ah;
    if (ps.has_lvd_voltage_mv)
      g_override.lvd_voltage_mv = ps.lvd_voltage_mv;
    if (ps.has_lvd_mode)
      g_override.lvd_mode = ps.lvd_mode;
    if (ps.has_night_mode)
      g_override.night_mode = ps.night_mode;
    if (ps.has_evening_minutes)
      g_override.evening_minutes = ps.evening_minutes;
    if (ps.has_morning_minutes)
      g_override.morning_minutes = ps.morning_minutes;
    if (ps.has_night_threshold_mv)
      g_override.night_threshold_mv = ps.night_threshold_mv;
    if (ps.has_dimming_mode)
      g_override.dimming_mode = ps.dimming_mode;
    if (ps.has_evening_minutes_dimming)
      g_override.evening_minutes_dimming = ps.evening_minutes_dimming;
    if (ps.has_morning_minutes_dimming)
      g_override.morning_minutes_dimming = ps.morning_minutes_dimming;
    if (ps.has_dimming_pct)
      g_override.dimming_pct = ps.dimming_pct;
    if (ps.has_base_dimming_pct)
      g_override.base_dimming_pct = ps.base_dimming_pct;
    if (ps.has_advanced_flags)
      g_override.advanced_flags = ps.advanced_flags;
    ok = true;
    reason = "settings applied";
    break;
  }

  case mppt_ControlCommand_switch_load_tag:
    ok = true;
    reason = (cmd.payload.switch_load.flags & 1u) ? "load on" : "load off";
    break;
  }

  mppt_CommandAck ack = mppt_CommandAck_init_zero;
  ack.ok = ok;
  ack.timestamp = millis() / 1000u;
  ack.request_id.funcs.encode = write_string_callback;
  ack.request_id.arg = req_id_buf;
  ack.reason.funcs.encode = write_string_callback;
  ack.reason.arg = (void *)reason;
  send_proto(MSG_ACK, mppt_CommandAck_fields, &ack);

  if (ok && cmd.which_payload == mppt_ControlCommand_set_settings_tag) {
    vTaskDelay(pdMS_TO_TICKS(50));
    publish_device_settings();
  }
}

static void on_nus_rx(const uint8_t *data, uint16_t len) {
    if (g_rx_asm.feed(data, len)) {
      if (g_rx_asm.msg_type() == MSG_CMD) {
        handle_command(g_rx_asm.payload(), g_rx_asm.payload_len());
      }
    }
}

static uint8_t g_own_addr_type;


static int ble_app_gap_event(struct ble_gap_event *event, void *arg);

static void ble_app_advertise(void) {
    // Stop any existing advertising first to avoid BLE_HS_EALREADY
    ble_gap_adv_stop();

    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name = "MPPT-Gateway";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    
    if (rc != 0) {
        ESP_LOGE(tag, "error setting advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    // Match Arduino intervals: 1600 * 0.625ms = 1000ms
    adv_params.itvl_min = 1600;
    adv_params.itvl_max = 1600;

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "error enabling advertisement; rc=%d", rc);
        return;
    }
}

static int ble_app_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
      case BLE_GAP_EVENT_CONNECT:
          if (event->connect.status == 0) {
              g_conn_handle = event->connect.conn_handle;
              g_burst_sent = false;
              g_last_sent_days = 0;
              g_last_sent_months = 0;

              // Request longer connection interval for power saving
              struct ble_gap_upd_params upd = {
                  .itvl_min = 480,   // 600ms
                  .itvl_max = 480,
                  .latency = 0,
                  .supervision_timeout = 300,  // 3s
                  .min_ce_len = 0,
                  .max_ce_len = 0,
              };
              ble_gap_conn_param_update(g_conn_handle, &upd);
              ESP_LOGI(tag, "Connected");
          } else {
              ble_app_advertise();
          }
          break;

      case BLE_GAP_EVENT_DISCONNECT:
          g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
          g_subscribed = false;
          ESP_LOGI(tag, "Disconnected; reason=%d", event->disconnect.reason);
          ble_app_advertise();
          break;

      case BLE_GAP_EVENT_SUBSCRIBE:
          if (event->subscribe.attr_handle == g_nus_tx_char_handle) {
              g_subscribed = event->subscribe.cur_notify;
              ESP_LOGI(tag, "TX Subscribed: %d", g_subscribed);
          }
          break;
          
      case BLE_GAP_EVENT_MTU:
          ESP_LOGI(tag, "MTU updated: %d", event->mtu.value);
          break;
      }
    return 0;
}

static void ble_app_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(tag, "error determining address type; rc=%d", rc);
        return;
    }
    ble_app_advertise();
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void main_loop_task(void *pvParameters) {
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    for (;;) {
        esp_task_wdt_reset();
        if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || !g_subscribed) {
            if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE) {

                g_subscribed = false;
                g_burst_sent = false;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        const uint32_t now = millis();

        if (!g_burst_sent) {
            g_tele_ok = parse_space_line(SPACE_LINE, g_tele);
            g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);
            g_tele.hw_version = g_hw_version;

            publish_device_info();
            vTaskDelay(pdMS_TO_TICKS(50));
            publish_device_settings();
            vTaskDelay(pdMS_TO_TICKS(50));
            publish_fault_status(true);
            vTaskDelay(pdMS_TO_TICKS(50));
            publish_datalog(true);

            g_burst_sent = true;
            g_last_tele_ms = now;
            g_last_eeprom_ms = now;
            continue;
        }

        if (now - g_last_tele_ms >= TELE_INTERVAL_MS) {
            g_last_tele_ms = now;
            // Stub repoll
            g_tele_ok = parse_space_line(SPACE_LINE, g_tele);
            g_tele.hw_version = g_hw_version;
            if (g_tele_ok) {
                publish_telemetry();
                publish_fault_status();
            }
        }

        if (now - g_last_eeprom_ms >= EEPROM_INTERVAL_MS) {
            g_last_eeprom_ms = now;
            // Stub repoll (always true in this version for simplicity)
            g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);
            if (g_eeprom_ok) {
                publish_datalog(false);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Configure power management
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 80,
        .min_freq_mhz = 10,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));

    int rc;
    rc = ble_gatts_count_cfg(g_gatt_svr_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(g_gatt_svr_svcs);
    assert(rc == 0);

    ble_svc_gap_device_name_set("MPPT-Gateway");
    ble_hs_cfg.sync_cb = ble_app_on_sync;

    nimble_port_init();
    nimble_port_freertos_init(ble_host_task);

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));


    g_tele_ok = parse_space_line(SPACE_LINE, g_tele);
    g_eeprom_ok = parse_eeprom_line(EEPROM_LINE, g_eeprom);

    if (g_eeprom_ok && g_eeprom.device_identifier != 0) {
        g_hw_version = ((g_eeprom.device_identifier & 0xFF) == 0x52) ? 2 : 3;
    } else if (g_tele_ok) {
        g_hw_version = g_tele.hw_version;
    } else {
        g_hw_version = 3;
    }
    g_tele.hw_version = g_hw_version;

    xTaskCreate(main_loop_task, "main_loop", 4096, NULL, 5, NULL);
}
