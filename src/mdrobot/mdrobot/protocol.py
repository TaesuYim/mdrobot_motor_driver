"""Protocol layer: read_registers / write_register / write_registers.

With just these three primitives, every byte/word/long/n-word PID can be built as
a higher-level helper. The transport is injected; the frame module builds requests
and validates responses.
"""

from __future__ import annotations

from typing import Sequence

from . import frame
from .codec import (
    join_i32_low_word_first,
    join_u32_low_word_first,
    split_i32_low_word_first,
)
from .constants import DEFAULT_SLAVE_ID, EXCEPTION_FLAG
from .exceptions import IncompleteResponseError
from .transport import Transport


class ModbusClient:
    """Modbus RTU client for a single MDROBOT controller on an RS485 bus."""

    def __init__(self, transport: Transport, slave_id: int = DEFAULT_SLAVE_ID) -> None:
        self.transport = transport
        self.slave_id = slave_id

    # --- low-level send/receive --------------------------------------------------------

    def _read_exact(self, size: int) -> bytes:
        """Read until exactly size bytes are collected; raise IncompleteResponseError otherwise."""
        buf = bytearray()
        while len(buf) < size:
            chunk = self.transport.read(size - len(buf))
            if not chunk:
                raise IncompleteResponseError(
                    f"short read: got {len(buf)} want {size}: {bytes(buf).hex()}"
                )
            buf += chunk
        return bytes(buf)

    def _transact(self, request: bytes, expected_len: int) -> bytes:
        """Send a request and assemble the response frame.

        Read the [ID, FUNC] header first; if the exception bit is set, read the
        rest as a 5-byte exception frame, otherwise read up to expected_len.
        """
        self.transport.flush_input()
        self.transport.write(request)
        header = self._read_exact(2)
        if header[1] & EXCEPTION_FLAG:
            return header + self._read_exact(3)  # CODE + CRC_L + CRC_H
        return header + self._read_exact(expected_len - 2)

    # --- shared raw primitives ---------------------------------------------------------

    def read_registers(self, pid: int, count: int) -> list[int]:
        """Read count words via 0x03 and return them in wire order."""
        request = frame.build_read_request(self.slave_id, pid, count)
        response = self._transact(request, frame.read_response_length(count))
        return frame.parse_read_response(response, self.slave_id, count)

    def read_register(self, pid: int) -> int:
        """Read a single word."""
        return self.read_registers(pid, 1)[0]

    def write_register(self, pid: int, word: int) -> None:
        """Write a single word via 0x06 and verify the echo."""
        request = frame.build_write_single_request(self.slave_id, pid, word)
        response = self._transact(request, frame.WRITE_SINGLE_RESPONSE_LENGTH)
        frame.parse_write_single_response(response, request)

    def write_registers(self, pid: int, words: Sequence[int]) -> None:
        """Write multiple words via 0x10 and verify the start-address/count echo."""
        request = frame.build_write_multiple_request(self.slave_id, pid, words)
        response = self._transact(request, frame.WRITE_MULTIPLE_RESPONSE_LENGTH)
        frame.parse_write_multiple_response(response, self.slave_id, pid, len(words))

    # --- long helpers ------------------------------------------------------------------

    def read_long(self, pid: int, *, signed: bool = True) -> int:
        """Read a 32-bit long (low word first). Signed by default."""
        low, high = self.read_registers(pid, 2)
        if signed:
            return join_i32_low_word_first(low, high)
        return join_u32_low_word_first(low, high)

    def write_long(self, pid: int, value: int) -> None:
        """Write a 32-bit long (low word first)."""
        low, high = split_i32_low_word_first(value)
        self.write_registers(pid, [low, high])

    # --- command helper ----------------------------------------------------------------

    def command(self, cmd: int) -> None:
        """Send a CMD number through the PID_COMMAND gateway (raw)."""
        from .registers import PID_COMMAND

        self.write_register(PID_COMMAND, cmd)
