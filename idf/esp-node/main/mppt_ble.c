#include "mppt_ble.h"
#include "ble_gatt.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/ble_gatt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPPT_BLE";

#define CHUNK_FIRST 0x00
#define CHUNK_CONT  0x01
#define CHUNK_LAST  0x80

void mppt_ble_send(mppt_msg_type_t msg_type, const uint8_t *payload, uint16_t len)
{
    uint16_t conn = ble_gatt_conn_handle();
    if (conn == BLE_HS_CONN_HANDLE_NONE || !ble_gatt_subscribed()) return;

    /* Build frame: [type(1)][len_lo(1)][len_hi(1)][payload] */
    uint16_t total = 3u + len;
    uint8_t *frame = (uint8_t *)malloc(total);
    if (!frame) {
        ESP_LOGE(TAG, "OOM frame %u", total);
        return;
    }
    frame[0] = (uint8_t)msg_type;
    frame[1] = (uint8_t)(len & 0xFF);
    frame[2] = (uint8_t)(len >> 8);
    memcpy(frame + 3, payload, len);

    uint16_t mtu = ble_att_mtu(conn);
    if (mtu < 23) mtu = 23;
    /* MTU - 3 bytes GATT overhead - 1 byte chunk tag */
    uint16_t chunk_max = mtu - 4u;

    uint16_t offset = 0;
    bool     first  = true;

    /* Stack buffer for one chunk: tag(1) + up to chunk_max bytes */
    uint8_t pkt[512];

    while (offset < total) {
        uint16_t take    = ((total - offset) < chunk_max) ? (total - offset) : chunk_max;
        bool     is_last = (offset + take >= total);

        if      (first && is_last) pkt[0] = CHUNK_FIRST;
        else if (first)            pkt[0] = CHUNK_FIRST;
        else if (is_last)          pkt[0] = CHUNK_LAST;
        else                       pkt[0] = CHUNK_CONT;

        memcpy(pkt + 1, frame + offset, take);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(pkt, 1u + take);
        if (om) {
            ble_gatts_notify_custom(conn, ble_gatt_tx_handle(), om);
        }

        offset += take;
        first   = false;

        if (!is_last) vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(frame);
}

/* ---------- Reassembler ---------- */

void mppt_reassembler_reset(mppt_reassembler_t *r)
{
    r->len         = 0;
    r->busy        = false;
    r->payload_len = 0;
}

bool mppt_reassembler_feed(mppt_reassembler_t *r, const uint8_t *data, uint16_t len)
{
    if (len == 0) return false;

    uint8_t        tag  = data[0];
    const uint8_t *body = data + 1;
    uint16_t       blen = len - 1;

    if (tag == CHUNK_FIRST) {
        r->len  = 0;
        r->busy = true;
    } else if (!r->busy) {
        return false;
    }

    uint16_t space = MPPT_REASSEMBLER_BUF - r->len;
    uint16_t copy  = (blen < space) ? blen : space;
    memcpy(r->buf + r->len, body, copy);
    r->len += copy;

    if (tag == CHUNK_FIRST || tag == CHUNK_LAST) {
        if (r->len < 3) { mppt_reassembler_reset(r); return false; }
        r->msg_type    = (mppt_msg_type_t)r->buf[0];
        r->payload_len = (uint16_t)r->buf[1] | ((uint16_t)r->buf[2] << 8);
        r->busy        = false;
        return (r->payload_len + 3 <= r->len);
    }

    return false;
}
