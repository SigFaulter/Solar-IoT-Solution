#include "ble_transport.h"
#include <cstring>
#include <cstdio>
#include "host/ble_hs.h"
#include "bleprph.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void ble_send(uint16_t conn_handle,
              MpptMsgType           msg_type,
              const uint8_t        *payload,
              size_t                len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    // Build framed packet: [msg_type(1)] [payload_len_le(2)] [payload(N)]
    const size_t total = 3 + len;
    uint8_t *frame = (uint8_t *)malloc(total);
    if (!frame) return;

    frame[0] = static_cast<uint8_t>(msg_type);
    frame[1] = static_cast<uint8_t>(len & 0xFF);
    frame[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    memcpy(frame + 3, payload, len);

    // Get actual MTU for this connection. GATT overhead is 3 bytes.
    uint16_t peer_mtu = ble_att_mtu(conn_handle);
    if (peer_mtu < 23) peer_mtu = 23; 
    
    const size_t chunk_max = peer_mtu - 4; // MTU - 3 (GATT) - 1 (Tag)

    size_t offset = 0;
    bool   first  = true;

    while (offset < total) {
        const size_t remaining = total - offset;
        const size_t take      = (remaining < chunk_max) ? remaining : chunk_max;
        const bool   is_last   = (offset + take >= total);

        struct os_mbuf *om = ble_hs_mbuf_from_flat(NULL, 0); // Start with empty mbuf
        if (!om) break;

        uint8_t tag;
        // Single-chunk messages use CHUNK_FIRST only (per spec).
        // Multi-chunk: first=CHUNK_FIRST, middle=CHUNK_CONT, last=CHUNK_LAST.
        if (first && is_last)  tag = CHUNK_FIRST; // single chunk
        else if (first)        tag = CHUNK_FIRST;
        else if (is_last)      tag = CHUNK_LAST;
        else                   tag = CHUNK_CONT;

        os_mbuf_append(om, &tag, 1);
        os_mbuf_append(om, frame + offset, take);

        int rc = ble_gattc_notify_custom(conn_handle, g_nus_tx_char_handle, om);
        if (rc != 0) {
            printf("[BLE] notify error: %d\n", rc);
            // os_mbuf_free_chain(om); // ble_gattc_notify_custom takes ownership or frees on error? 
            // Actually NimBLE usually frees mbuf on success/error in notify_custom.
            break; 
        }

        offset += take;
        first   = false;

        if (!is_last) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    free(frame);
}

void BleReassembler::reset()
{
    _len         = 0;
    _busy        = false;
    _payload_len = 0;
}

bool BleReassembler::feed(const uint8_t *data, size_t len)
{
    if (len == 0) return false;

    const uint8_t  tag  = data[0];
    const uint8_t *body = data + 1;
    const size_t   blen = len - 1;

    if (tag == CHUNK_FIRST) {
        _len  = 0;
        _busy = true;
    } else if (!_busy) {
        // Stray CONT or LAST without a preceding FIRST - ignore
        return false;
    }

    // Append body, guard against overflow
    const size_t space = BUF_SIZE - _len;
    const size_t copy  = (blen < space) ? blen : space;
    memcpy(_buf + _len, body, copy);
    _len += copy;

    // Frame is complete on CHUNK_FIRST (single-chunk) or CHUNK_LAST
    if (tag == CHUNK_FIRST || tag == CHUNK_LAST) {
        if (_len < 3) { reset(); return false; }
        _msg_type    = static_cast<MpptMsgType>(_buf[0]);
        _payload_len = static_cast<size_t>(_buf[1]) | (static_cast<size_t>(_buf[2]) << 8);
        _busy        = false;
        return (_payload_len + 3 <= _len);
    }

    return false; // CHUNK_CONT - waiting for more
}
