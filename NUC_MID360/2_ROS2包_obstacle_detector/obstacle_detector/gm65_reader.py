"""Bounded, nonblocking USB serial input for bedside barcode scanning."""

from typing import Callable, List, Optional

from .nav_protocol import SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3, SCAN_FORMAT_CODE128
from .scanner_core import value_is_allowed


DEFAULT_GM65_DEVICE = (
    "/dev/serial/by-id/usb-USBKey_Chip_USBKey_Module_202730041341-if00"
)


def open_gm65_serial(**kwargs):
    import serial

    return serial.Serial(**kwargs)


class Gm65Reader:
    def __init__(
        self,
        device: str = DEFAULT_GM65_DEVICE,
        baud: int = 9600,
        serial_factory: Callable = open_gm65_serial,
    ) -> None:
        self.device = device
        self.baud = baud
        self.serial_factory = serial_factory
        self.connection = None
        self.last_error = ""
        self._context = None
        self._buffer = bytearray()
        self._discarding = False
        self._last_byte_s = None
        self._last_poll_s = None
        self._retry_at_s = 0.0

    @property
    def connected(self) -> bool:
        return self.connection is not None

    def reset(self) -> None:
        self._context = None
        self._buffer.clear()
        self._discarding = False
        self._last_byte_s = None

    def close(self) -> None:
        connection, self.connection = self.connection, None
        self.reset()
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    def _finish_frame(self) -> Optional[str]:
        value = None
        if not self._discarding:
            try:
                candidate = self._buffer.decode("ascii")
            except UnicodeDecodeError:
                candidate = ""
            if value_is_allowed(SCAN_FORMAT_CODE128, candidate):
                value = candidate
        self._buffer.clear()
        self._discarding = False
        self._last_byte_s = None
        return value

    def poll(self, context: Optional[int], now_s: float) -> List[str]:
        if context not in (SCAN_CONTEXT_BED1, SCAN_CONTEXT_BED3):
            context = None
        delayed = (
            self._last_poll_s is not None and now_s - self._last_poll_s > 0.5
        )
        self._last_poll_s = now_s
        try:
            if self.connection is None:
                if now_s < self._retry_at_s:
                    return []
                self.connection = self.serial_factory(
                    port=self.device,
                    baudrate=self.baud,
                    timeout=0,
                    write_timeout=0,
                    exclusive=True,
                )
                self.reset()
                self.connection.reset_input_buffer()
                self._context = context
                self.last_error = ""
                return []
            if context is None or context != self._context or delayed:
                self.reset()
                self.connection.reset_input_buffer()
                self._context = context
                return []
            waiting = self.connection.in_waiting
            if waiting > 512:
                self.reset()
                self.connection.reset_input_buffer()
                self._context = context
                return []
            data = self.connection.read(waiting) if waiting else b""
        except (OSError, ValueError, ImportError) as exc:
            self.last_error = str(exc)
            self.close()
            self._retry_at_s = now_s + 2.0
            return []

        values = []
        if (
            not data
            and self._last_byte_s is not None
            and now_s - self._last_byte_s >= 0.1
        ):
            value = self._finish_frame()
            if value is not None:
                values.append(value)
        for byte in data:
            if byte in (10, 13):
                value = self._finish_frame()
                if value is not None:
                    values.append(value)
            else:
                self._last_byte_s = now_s
                if len(self._buffer) < 64 and not self._discarding:
                    self._buffer.append(byte)
                else:
                    self._discarding = True
                    self._buffer.clear()
        return values
