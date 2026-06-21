"""Transport interface and serial implementation.

`Transport` is the minimal interface the protocol layer depends on. Unit tests
inject a fake transport; real communication uses `SerialTransport` (pyserial).

pyserial is an optional dependency (the `serial` extra). It is imported only when
a `SerialTransport` is actually created, so this module and the protocol layer
import fine without pyserial installed.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

from .constants import DEFAULT_BAUDRATE, DEFAULT_TIMEOUT


@runtime_checkable
class Transport(Protocol):
    """Minimal interface a serial transport must provide."""

    def write(self, data: bytes) -> int:
        """Send all of data and return the number of bytes written."""
        ...

    def read(self, size: int) -> bytes:
        """Read up to size bytes; may return fewer."""
        ...

    def flush_input(self) -> None:
        """Discard any bytes left in the input buffer (call before a request)."""
        ...


class SerialTransport:
    """pyserial-based RS485 / Modbus RTU serial transport.

    Implements the `Transport` protocol. Defaults are 19200 8N1. On RS485
    half-duplex the bus must switch to receive right after transmit, so write
    flushes to wait for transmission to complete.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        timeout: float = DEFAULT_TIMEOUT,
        settle: float = 0.2,
        write_timeout: float = 1.0,
    ) -> None:
        import time

        import serial  # lazy import: pyserial is an optional dependency

        self.port = port
        self.baudrate = baudrate
        # write_timeout: keep write/flush from blocking forever if the port wedges
        # (prevents shutdown hangs).
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=write_timeout,
        )
        # USB-serial adapters (FTDI/CH340) can emit boot noise / leftover bytes
        # right after open (observed: the first 1-2 transactions can be 0xFF noise
        # or desync). Wait `settle`, then clear the RX/TX buffers to align the
        # first transaction.
        if settle > 0:
            time.sleep(settle)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    @classmethod
    def from_serial(cls, serial_port: Any) -> "SerialTransport":
        """Wrap an already-open pyserial-compatible object (tests / advanced use).

        Does not open a new pyserial.Serial; the object is injected. Used to pass a
        fake serial port in unit tests.
        """
        obj = cls.__new__(cls)
        obj.port = getattr(serial_port, "port", None)
        obj.baudrate = getattr(serial_port, "baudrate", None)
        obj._serial = serial_port
        return obj

    def write(self, data: bytes) -> int:
        """Send data, wait for transmission to complete, and return bytes written."""
        written = self._serial.write(data)
        self._serial.flush()
        return written if written is not None else len(data)

    def read(self, size: int) -> bytes:
        """Read up to size bytes; returns fewer (or empty) on timeout."""
        return self._serial.read(size)

    def flush_input(self) -> None:
        """Discard any bytes left in the receive buffer (call before a request)."""
        self._serial.reset_input_buffer()

    def close(self) -> None:
        """Close the serial port."""
        self._serial.close()

    @property
    def is_open(self) -> bool:
        """Whether the port is open."""
        return bool(getattr(self._serial, "is_open", False))

    def __enter__(self) -> "SerialTransport":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
