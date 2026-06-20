"""프로토콜 계층: read_registers / write_register / write_registers.

이 세 primitive만 정확하면 byte/word/long/n-word PID는 전부 상위 helper로 구현할 수 있다
(doc 02 §11). transport는 주입받으며, frame 모듈로 요청을 만들고 응답을 검증한다.

Phase 1 범위는 인코딩/디코딩/검증 + 가짜 transport 단위 테스트다. 실제 직렬 timeout 누적
read는 Phase 2 SerialTransport에서 다룬다.
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
    """단일 RS485 버스 위의 MDROBOT 컨트롤러 한 대에 대한 Modbus RTU 클라이언트."""

    def __init__(self, transport: Transport, slave_id: int = DEFAULT_SLAVE_ID) -> None:
        self.transport = transport
        self.slave_id = slave_id

    # --- 저수준 송수신 -----------------------------------------------------------------

    def _read_exact(self, size: int) -> bytes:
        """정확히 size byte를 모을 때까지 read한다. 더 안 오면 IncompleteResponseError."""
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
        """요청을 보내고 응답 프레임을 조립해 반환한다.

        먼저 [ID, FUNC] 2 byte를 읽고, exception bit가 서 있으면 5 byte짜리 exception
        프레임으로 마저 읽는다. 아니면 expected_len까지 읽는다.
        """
        self.transport.flush_input()
        self.transport.write(request)
        header = self._read_exact(2)
        if header[1] & EXCEPTION_FLAG:
            return header + self._read_exact(3)  # CODE + CRC_L + CRC_H
        return header + self._read_exact(expected_len - 2)

    # --- 공통 raw primitive ------------------------------------------------------------

    def read_registers(self, pid: int, count: int) -> list[int]:
        """0x03으로 word를 count개 읽어 wire 순서 리스트로 반환한다."""
        request = frame.build_read_request(self.slave_id, pid, count)
        response = self._transact(request, frame.read_response_length(count))
        return frame.parse_read_response(response, self.slave_id, count)

    def read_register(self, pid: int) -> int:
        """word 한 개를 읽는다."""
        return self.read_registers(pid, 1)[0]

    def write_register(self, pid: int, word: int) -> None:
        """0x06으로 word 한 개를 쓰고 echo를 검증한다."""
        request = frame.build_write_single_request(self.slave_id, pid, word)
        response = self._transact(request, frame.WRITE_SINGLE_RESPONSE_LENGTH)
        frame.parse_write_single_response(response, request)

    def write_registers(self, pid: int, words: Sequence[int]) -> None:
        """0x10으로 word 여러 개를 쓰고 start address/count echo를 검증한다."""
        request = frame.build_write_multiple_request(self.slave_id, pid, words)
        response = self._transact(request, frame.WRITE_MULTIPLE_RESPONSE_LENGTH)
        frame.parse_write_multiple_response(response, self.slave_id, pid, len(words))

    # --- long helper -------------------------------------------------------------------

    def read_long(self, pid: int, *, signed: bool = True) -> int:
        """32비트 long을 읽는다(low word first). 기본 signed."""
        low, high = self.read_registers(pid, 2)
        if signed:
            return join_i32_low_word_first(low, high)
        return join_u32_low_word_first(low, high)

    def write_long(self, pid: int, value: int) -> None:
        """32비트 long을 쓴다(low word first)."""
        low, high = split_i32_low_word_first(value)
        self.write_registers(pid, [low, high])

    # --- command helper ----------------------------------------------------------------

    def command(self, cmd: int) -> None:
        """PID_COMMAND 게이트웨이로 CMD 번호를 보낸다(raw)."""
        from .registers import PID_COMMAND

        self.write_register(PID_COMMAND, cmd)
