"""Serial link to the STM32 with the A5 5A ping/pong handshake."""

import threading
import time
from typing import Callable

import serial
import serial.tools.list_ports

from .nav_protocol import Frame, FrameParser


PING_FRAME = bytes((0xA5, 0x5A, 0x01, 0x00))
PONG_FRAME = bytes((0xA5, 0x5A, 0x01, 0x01))


class SerialTransport:
    def __init__(
        self,
        port: str,
        baud: int,
        on_frame: Callable[[Frame], None],
        logger,
    ) -> None:
        self.port = port
        self.baud = baud
        self.on_frame = on_frame
        self.logger = logger
        self.parser = FrameParser()
        self.serial = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ping_index = 0
        self.connected = False
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        with self.lock:
            if self.serial is not None:
                try:
                    self.serial.close()
                except serial.SerialException:
                    pass
                self.serial = None

    def send(self, data: bytes) -> bool:
        try:
            with self.lock:
                if self.serial is None or not self.serial.is_open:
                    return False
                self.serial.write(data)
            return True
        except serial.SerialException as exc:
            self.logger.error(f"Serial write failed: {exc}")
            self._disconnect()
            return False

    def _disconnect(self) -> None:
        with self.lock:
            if self.serial is not None:
                try:
                    self.serial.close()
                except serial.SerialException:
                    pass
            self.serial = None
            self.connected = False

    def _open(self) -> bool:
        try:
            connection = serial.serial_for_url(
                self.port,
                self.baud,
                timeout=0.02,
                write_timeout=0.1,
            )
            with self.lock:
                self.serial = connection
                self.connected = True
            self.logger.info(f"Serial {self.port}@{self.baud} opened")
            return True
        except serial.SerialException:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.logger.warn(f"Serial {self.port} unavailable; detected ports: {ports}")
            return False

    def _feed_handshake(self, value: int) -> None:
        if value == PING_FRAME[self.ping_index]:
            self.ping_index += 1
            if self.ping_index == len(PING_FRAME):
                self.ping_index = 0
                self.send(PONG_FRAME)
        elif value == PING_FRAME[0]:
            self.ping_index = 1
        else:
            self.ping_index = 0

    def _run(self) -> None:
        retry_at = 0.0
        while not self.stop_event.is_set():
            if not self.connected:
                if time.monotonic() < retry_at:
                    time.sleep(0.05)
                    continue
                if not self._open():
                    retry_at = time.monotonic() + 2.0
                    continue

            try:
                with self.lock:
                    connection = self.serial
                data = connection.read(128) if connection is not None else b""
            except serial.SerialException as exc:
                self.logger.error(f"Serial read failed: {exc}")
                self._disconnect()
                retry_at = time.monotonic() + 1.0
                continue

            if not data:
                continue
            for value in data:
                self._feed_handshake(value)
            for frame in self.parser.feed(data):
                self.on_frame(frame)
