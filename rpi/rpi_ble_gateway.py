#!/usr/bin/env python3
"""
BLE-to-MQTT multi-gateway for MPPT ESP32 nodes.

Scans for BLE devices with 'MPPT-' prefix (e.g., MPPT-0503-18) and
maintains concurrent GATT connections for each. Serial numbers are
auto-extracted from the BLE device name.

Usage:
    python3 rpi_ble_gateway.py --broker 192.168.1.10 --zone site1
"""

import argparse
import asyncio
import logging
import struct
import time
import sys
import os
import socket
from typing import Optional, Dict

from bleak import BleakClient, BleakScanner
import paho.mqtt.client as mqtt

_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

try:
    from proto import mppt_pb2
except ImportError:
    sys.exit(
        f"Cannot import proto.mppt_pb2.\n"
        f"Run `make proto` inside parser/ to generate it.\n"
        f"Expected: {os.path.join(_REPO_ROOT, 'proto', 'mppt_pb2.py')}"
    )

# ── BLE constants ──────────────────────────────────────────────────────────────
SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 -> RPi (notify)
RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # RPi -> ESP32 (write)

MSG_TELEMETRY = 0x01
MSG_FAULT_STATUS = 0x02
MSG_DEVICE_INFO = 0x03
MSG_SETTINGS = 0x04
MSG_DATALOG = 0x05
MSG_ACK = 0x11
MSG_CMD = 0x10

CHUNK_FIRST = 0x00
CHUNK_CONT = 0x01
CHUNK_LAST = 0x80

BLE_SCAN_TIMEOUT = 10.0
BLE_TARGET_MTU = 247
CMD_WRITE_DELAY = 0.02

log = logging.getLogger("mppt-ble")

# ── BLE framing ────────────────────────────────────────────────────────────────

class FrameReassembler:
    def __init__(self, timeout: float = 10.0):
        self._buf: bytearray = bytearray()
        self._busy: bool = False
        self._last_chunk_time: float = 0.0
        self._timeout = timeout

    def feed(self, data: bytes) -> Optional[tuple[int, bytes]]:
        if not data: return None
        now = time.monotonic()
        tag = data[0]
        chunk = data[1:]

        if self._busy and (now - self._last_chunk_time > self._timeout):
            self._buf.clear()
            self._busy = False
        self._last_chunk_time = now

        if tag == CHUNK_FIRST:
            self._buf = bytearray(chunk)
            self._busy = True
        elif tag == CHUNK_CONT:
            if not self._busy: return None
            self._buf.extend(chunk)
            return None
        elif tag == CHUNK_LAST:
            if not self._busy: return None
            self._buf.extend(chunk)
            self._busy = False
        else: return None

        if len(self._buf) < 3: return None
        payload_len = struct.unpack_from("<H", self._buf, 1)[0]
        if len(self._buf) < 3 + payload_len: return None

        msg_type = self._buf[0]
        payload = bytes(self._buf[3 : 3 + payload_len])
        self._buf.clear()
        return (msg_type, payload)

def build_framed_packet(msg_type: int, payload: bytes, mtu: int = BLE_TARGET_MTU) -> list[bytes]:
    frame = bytes([msg_type]) + struct.pack("<H", len(payload)) + payload
    chunk_data_max = mtu - 4
    chunks = []
    offset = 0
    while offset < len(frame):
        take = min(len(frame) - offset, chunk_data_max)
        is_last = offset + take >= len(frame)
        is_first = offset == 0
        tag = CHUNK_FIRST if (is_first and is_last) else (CHUNK_FIRST if is_first else (CHUNK_LAST if is_last else CHUNK_CONT))
        chunks.append(bytes([tag]) + frame[offset : offset + take])
        offset += take
    return chunks

# ── MQTT Manager ──────────────────────────────────────────────────────────────

class MqttManager:
    def __init__(self, broker: str, port: int, zone: str, gateway_id: str):
        self.broker = broker
        self.port = port
        self.zone = zone
        self.gateway_id = gateway_id
        self._gateways: Dict[str, 'MpptBleGateway'] = {}
        self._client = self._setup_mqtt()

    def _setup_mqtt(self) -> mqtt.Client:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"ble_gw_{self.gateway_id}")
        client.on_connect = self._on_connect
        client.on_message = self._on_message
        client.reconnect_delay_set(min_delay=1, max_delay=30)
        client.connect_async(self.broker, self.port, keepalive=60)
        client.loop_start()
        return client

    def _on_connect(self, client, userdata, flags, reason, props):
        log.info("MQTT connected")
        client.subscribe(f"mppt/{self.zone}/{self.gateway_id}/+/cmd", qos=1)

    def _on_message(self, client, userdata, msg):
        parts = msg.topic.split("/")
        if len(parts) >= 5 and parts[4] == "cmd":
            serial = parts[3]
            if serial in self._gateways:
                self._gateways[serial].queue_command(msg.payload)

    def register_gateway(self, serial: str, gateway: 'MpptBleGateway'):
        self._gateways[serial] = gateway

    def unregister_gateway(self, serial: str):
        self._gateways.pop(serial, None)

    def publish(self, serial: str, subtopic: str, payload: bytes, retain: bool = False, qos: int = 0):
        topic = f"mppt/{self.zone}/{self.gateway_id}/{serial}/{subtopic}"
        self._client.publish(topic, payload, qos=qos, retain=retain)

    def publish_online(self, serial: str, online: bool):
        topic = f"mppt/{self.zone}/{self.gateway_id}/{serial}/online"
        self._client.publish(topic, "1" if online else "0", qos=1, retain=True)

    def cleanup(self):
        self._client.loop_stop()
        self._client.disconnect()

# ── BLE Gateway ───────────────────────────────────────────────────────────────

class MpptBleGateway:
    def __init__(self, manager: MqttManager, address: str, serial: str):
        self.manager = manager
        self.address = address
        self.serial = serial
        self._reassembler = FrameReassembler(timeout=10.0)
        self._cmd_queue: asyncio.Queue = asyncio.Queue()
        self._running = False

    def queue_command(self, payload: bytes):
        self._cmd_queue.put_nowait(payload)

    def _dispatch(self, msg_type: int, payload: bytes):
        try:
            if msg_type == MSG_TELEMETRY:
                self.manager.publish(self.serial, "state", payload)
            elif msg_type == MSG_FAULT_STATUS:
                self.manager.publish(self.serial, "faults", payload, retain=True, qos=1)
            elif msg_type == MSG_DEVICE_INFO:
                self.manager.publish(self.serial, "info", payload, retain=True, qos=1)
            elif msg_type == MSG_SETTINGS:
                self.manager.publish(self.serial, "settings", payload, retain=True, qos=1)
            elif msg_type == MSG_DATALOG:
                m = mppt_pb2.DataloggerPayload()
                m.ParseFromString(payload)
                self._split_and_publish_datalog(m)
            elif msg_type == MSG_ACK:
                self.manager.publish(self.serial, "ack", payload, qos=1)
        except Exception as e:
            log.error("[%s] Dispatch error: %s", self.serial, e)

    def _split_and_publish_datalog(self, m: "mppt_pb2.DataloggerPayload"):
        summary = mppt_pb2.DataloggerPayload()
        summary.CopyFrom(m)
        summary.ClearField("daily_logs")
        summary.ClearField("monthly_logs")
        if m.recorded_days > 0 or m.days_with_lvd > 0 or m.total_ah_charge_mah > 0:
            self.manager.publish(self.serial, "datalog/summary", summary.SerializeToString(), retain=True, qos=1)
        if m.daily_logs:
            daily = mppt_pb2.DataloggerPayload()
            daily.timestamp = m.timestamp
            daily.daily_logs.extend(m.daily_logs)
            self.manager.publish(self.serial, "datalog/daily", daily.SerializeToString(), retain=True, qos=1)
        if m.monthly_logs:
            monthly = mppt_pb2.DataloggerPayload()
            monthly.timestamp = m.timestamp
            monthly.monthly_logs.extend(m.monthly_logs)
            self.manager.publish(self.serial, "datalog/monthly", monthly.SerializeToString(), retain=True, qos=1)

    def _on_notify(self, handle, data: bytearray):
        result = self._reassembler.feed(bytes(data))
        if result:
            msg_type, payload = result
            self._dispatch(msg_type, payload)

    def _on_disconnected(self, client: BleakClient):
        log.warning("[%s] BLE disconnected", self.serial)
        self._running = False

    async def run(self):
        self._running = True
        log.info("[%s] Connecting to %s...", self.serial, self.address)
        try:
            async with BleakClient(self.address, disconnected_callback=self._on_disconnected) as client:
                try:
                    await client.get_services()
                    if hasattr(client, "_acquire_mtu"): await client._acquire_mtu()
                    log.info("[%s] Connected, MTU=%d", self.serial, client.mtu_size)
                except Exception as e:
                    log.warning("[%s] MTU sync failed: %s", self.serial, e)

                await client.start_notify(TX_UUID, self._on_notify)
                self.manager.publish_online(self.serial, True)
                
                while self._running and client.is_connected:
                    try:
                        cmd_bytes = await asyncio.wait_for(self._cmd_queue.get(), timeout=1.0)
                        log.info("[%s] Forwarding CMD (%d B)", self.serial, len(cmd_bytes))
                        for chunk in build_framed_packet(MSG_CMD, cmd_bytes, mtu=client.mtu_size):
                            await client.write_gatt_char(RX_UUID, chunk, response=False)
                            await asyncio.sleep(CMD_WRITE_DELAY)
                    except asyncio.TimeoutError:
                        continue
        except Exception as e:
            log.error("[%s] Connection error: %s", self.serial, e)
        finally:
            self._running = False
            self.manager.publish_online(self.serial, False)
            self.manager.unregister_gateway(self.serial)

# ── Main Loop ─────────────────────────────────────────────────────────────────

async def main_loop():
    parser = argparse.ArgumentParser(description="MPPT BLE-to-MQTT multi-gateway")
    parser.add_argument("--broker", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", default=1883, type=int, help="MQTT broker port")
    parser.add_argument("--zone", default="default", help="MQTT topic zone segment")
    parser.add_argument("--gateway", default="auto", help="Gateway ID")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s")
    
    gateway_id = args.gateway if args.gateway != "auto" else socket.gethostname()
    mqtt_mgr = MqttManager(args.broker, args.port, args.zone, gateway_id)
    
    active_tasks = {} # address -> Task
    
    log.info("Starting multi-device gateway (ID: %s)", gateway_id)
    
    try:
        while True:
            # Clean up finished tasks
            finished = [addr for addr, task in active_tasks.items() if task.done()]
            for addr in finished:
                task = active_tasks.pop(addr)
                try:
                    task.result()
                except Exception as e:
                    log.error("Task for %s failed: %s", addr, e)

            # Scan for devices advertising our service UUID
            try:
                # discover() returns (device, advertisement_data) tuples if return_adv=True
                scanner_results = await BleakScanner.discover(timeout=5.0, return_adv=True)
                for addr, (dev, adv) in scanner_results.items():
                    # Filter by service UUID (case-insensitive check)
                    if SVC_UUID.lower() in [s.lower() for s in adv.service_uuids]:
                        serial = dev.name or "Unknown-SN"
                        if addr not in active_tasks:
                            log.info("Found node: %s (%s)", serial, addr)
                            gw = MpptBleGateway(mqtt_mgr, addr, serial)
                            mqtt_mgr.register_gateway(serial, gw)
                            active_tasks[addr] = asyncio.create_task(gw.run())
            except Exception as e:
                log.error("Scan error: %s", e)
            
            await asyncio.sleep(10)
    finally:
        mqtt_mgr.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main_loop())
    except KeyboardInterrupt:
        pass
