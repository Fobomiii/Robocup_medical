#!/usr/bin/env python3
"""Receive ESP32-S3 vital signs over BLE and publish a ROS status topic."""

import asyncio
import json
import queue
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .health_ble_core import (
    HealthSample,
    ble_advertisement_matches,
    decode_health_packet,
)


DEFAULT_DEVICE_NAME = "MedicalVitals-S3"
DEFAULT_SERVICE_UUID = "7d2e1000-5f5b-4f4b-9c61-7f1e9d5a0001"
DEFAULT_CHARACTERISTIC_UUID = "7d2e1001-5f5b-4f4b-9c61-7f1e9d5a0001"


class HealthBleBridge(Node):
    def __init__(self) -> None:
        super().__init__("health_ble_bridge")
        self.device_name = str(
            self.declare_parameter("device_name", DEFAULT_DEVICE_NAME).value
        )
        self.device_address = str(
            self.declare_parameter("device_address", "").value
        ).strip()
        self.service_uuid = str(
            self.declare_parameter("service_uuid", DEFAULT_SERVICE_UUID).value
        ).lower()
        self.characteristic_uuid = str(
            self.declare_parameter(
                "characteristic_uuid", DEFAULT_CHARACTERISTIC_UUID
            ).value
        ).lower()
        self.scan_timeout_s = max(
            1.0, float(self.declare_parameter("scan_timeout_s", 5.0).value)
        )
        self.reconnect_delay_s = max(
            0.2, float(self.declare_parameter("reconnect_delay_s", 1.0).value)
        )
        self.connect_timeout_s = max(
            2.0, float(self.declare_parameter("connect_timeout_s", 10.0).value)
        )
        self.forget_after_failures = max(
            0, int(self.declare_parameter("forget_after_failures", 3).value)
        )
        self.clear_stale_device = bool(
            self.declare_parameter("clear_stale_device", True).value
        )
        self.stale_timeout_s = max(
            0.5, float(self.declare_parameter("stale_timeout_s", 2.0).value)
        )
        status_rate_hz = max(
            1.0, float(self.declare_parameter("status_rate_hz", 5.0).value)
        )

        self.publisher = self.create_publisher(
            String, "/medical_nav/health_status", 10
        )
        self.notifications = queue.SimpleQueue()
        self.state_lock = threading.Lock()
        self.connected = False
        self.connected_address = ""
        self.last_error = "starting"
        self.connection_phase = "starting"
        self.last_sample: Optional[HealthSample] = None
        self.last_sample_s = 0.0
        self.stop_event = threading.Event()
        self.worker = threading.Thread(
            target=self._ble_worker, name="health-ble", daemon=True
        )
        self.worker.start()
        self.create_timer(1.0 / status_rate_hz, self._publish_status)
        self.get_logger().info(
            f"BLE health receiver scanning for {self.device_name}"
        )

    def _set_connection(
        self,
        connected: bool,
        address: str = "",
        error: str = "",
        phase: str = "",
    ) -> None:
        with self.state_lock:
            self.connected = connected
            self.connected_address = address if connected else ""
            self.last_error = error
            if phase:
                self.connection_phase = phase

    def _ble_worker(self) -> None:
        try:
            asyncio.run(self._ble_loop())
        except Exception as error:  # pragma: no cover - defensive thread boundary
            self._set_connection(False, error=f"BLE worker stopped: {error}")

    async def _ble_loop(self) -> None:
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            self._set_connection(False, error="python3-bleak is not installed")
            return

        if self.clear_stale_device:
            await self._remove_stale_devices()

        connection_failures = 0
        while not self.stop_event.is_set():
            target = None
            client = None
            target_address = ""
            forget_target = False
            try:
                self._set_connection(False, phase="scanning")
                devices = await BleakScanner.discover(timeout=self.scan_timeout_s)
                matching = [device for device in devices if self._matches(device)]
                if matching:
                    matching.sort(
                        key=lambda device: getattr(device, "rssi", -999) or -999,
                        reverse=True,
                    )
                    target = matching[0]
                if target is None:
                    self._set_connection(
                        False,
                        error=f"{self.device_name} not found",
                        phase="scanning",
                    )
                else:
                    target_address = str(getattr(target, "address", ""))
                    disconnect_event = asyncio.Event()
                    event_loop = asyncio.get_running_loop()

                    def disconnected(_client) -> None:
                        event_loop.call_soon_threadsafe(disconnect_event.set)

                    self._set_connection(False, phase="connecting")
                    client = BleakClient(
                        target,
                        disconnected_callback=disconnected,
                        timeout=self.connect_timeout_s,
                    )
                    await client.connect()
                    if not client.is_connected:
                        raise RuntimeError("BLE connect returned without a link")
                    await client.start_notify(
                        self.characteristic_uuid, self._notification
                    )
                    connection_failures = 0
                    self._set_connection(
                        True, target_address, "", phase="connected"
                    )
                    self.get_logger().info(
                        f"BLE health connected to {target_address}"
                    )
                    while (
                        client.is_connected
                        and not disconnect_event.is_set()
                        and not self.stop_event.is_set()
                    ):
                        await asyncio.sleep(0.2)
                    self._set_connection(
                        False, error="BLE link disconnected", phase="reconnecting"
                    )
            except Exception as error:
                if target is not None:
                    connection_failures += 1
                    self.get_logger().warn(
                        "BLE health connection failed "
                        f"(attempt {connection_failures}): "
                        f"{error}"
                    )
                self._set_connection(
                    False, error=str(error), phase="reconnecting"
                )
                forget_target = bool(
                    target_address
                    and self.forget_after_failures > 0
                    and connection_failures >= self.forget_after_failures
                )
            finally:
                if client is not None:
                    try:
                        if client.is_connected:
                            await client.stop_notify(self.characteristic_uuid)
                            await client.disconnect()
                    except Exception:
                        pass
                if forget_target:
                    await self._remove_bluez_device(target_address)
                    connection_failures = 0
                if not self.stop_event.is_set():
                    await asyncio.sleep(self.reconnect_delay_s)

    def _matches(self, device) -> bool:
        address = str(getattr(device, "address", ""))
        if self.device_address:
            return address.casefold() == self.device_address.casefold()
        name = str(getattr(device, "name", "") or "")
        metadata = getattr(device, "metadata", {}) or {}
        return ble_advertisement_matches(
            self.device_name,
            self.device_address,
            self.service_uuid,
            name,
            address,
            metadata.get("uuids", []),
        )

    async def _bluetoothctl(self, *arguments: str) -> tuple[int, str]:
        process = None
        try:
            process = await asyncio.create_subprocess_exec(
                "bluetoothctl",
                *arguments,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
            output, _ = await asyncio.wait_for(
                process.communicate(), timeout=5.0
            )
            return process.returncode or 0, output.decode(errors="replace")
        except asyncio.TimeoutError as error:
            if process is not None and process.returncode is None:
                process.kill()
                await process.wait()
            return 1, str(error)
        except FileNotFoundError as error:
            return 1, str(error)

    async def _remove_stale_devices(self) -> None:
        return_code, output = await self._bluetoothctl("devices")
        if return_code != 0:
            self.get_logger().warn(
                f"Unable to inspect stale BlueZ devices: {output.strip()}"
            )
            return
        for line in output.splitlines():
            fields = line.strip().split(maxsplit=2)
            if len(fields) != 3 or fields[0] != "Device":
                continue
            address, name = fields[1], fields[2]
            if name == self.device_name or (
                self.device_address
                and address.casefold() == self.device_address.casefold()
            ):
                await self._remove_bluez_device(address)

    async def _remove_bluez_device(self, address: str) -> None:
        return_code, output = await self._bluetoothctl("remove", address)
        detail = output.strip()
        if return_code == 0:
            self.get_logger().info(
                f"Removed stale BlueZ record for {address}; rescanning"
            )
        else:
            self.get_logger().warn(
                f"Could not remove stale BlueZ record for {address}: {detail}"
            )

    def _notification(self, _sender, data: bytearray) -> None:
        self.notifications.put((time.monotonic(), bytes(data)))

    def _publish_status(self) -> None:
        while True:
            try:
                received_s, packet = self.notifications.get_nowait()
            except queue.Empty:
                break
            sample = decode_health_packet(packet)
            if sample is not None:
                self.last_sample = sample
                self.last_sample_s = received_s

        with self.state_lock:
            link_connected = self.connected
            address = self.connected_address
            error = self.last_error
            phase = self.connection_phase
        age_s = (
            time.monotonic() - self.last_sample_s
            if self.last_sample_s > 0.0
            else float("inf")
        )
        fresh = link_connected and self.last_sample is not None and age_s <= self.stale_timeout_s
        sample = self.last_sample if fresh else None
        payload = {
            "connected": fresh,
            "ble_link": link_connected,
            "device_name": self.device_name,
            "service_uuid": self.service_uuid,
            "address": address,
            "phase": phase,
            "heart_rate_bpm": sample.heart_rate_bpm if sample else None,
            "temperature_c": sample.temperature_c if sample else None,
            "sequence": sample.sequence if sample else None,
            "uptime_ms": sample.uptime_ms if sample else None,
            "age_s": round(age_s, 3) if age_s != float("inf") else None,
            "error": error,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthBleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
