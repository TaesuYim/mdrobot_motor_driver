"""Modbus RTU 프레임 builder / parser.

순수 함수 계층이다. 직렬 I/O는 transport(Phase 2), 송수신 흐름은 protocol에서 다룬다.
프레임 규칙은 doc 02, 검증 체크리스트는 doc 07 §4를 따른다.
"""

from __future__ import annotations

from typing import Sequence

from .constants import (
    EXCEPTION_FLAG,
    FUNC_READ,
    FUNC_WRITE_MULTIPLE,
    FUNC_WRITE_SINGLE,
)
from .crc import append_crc, check_crc
from .exceptions import CrcError, IncompleteResponseError, ProtocolError


def _hi(value: int) -> int:
    return (value >> 8) & 0xFF


def _lo(value: int) -> int:
    return value & 0xFF


# --- request builders ----------------------------------------------------------------

def build_read_request(slave_id: int, pid: int, count: int) -> bytes:
    """0x03 read holding registers 요청 프레임."""
    if count < 1:
        raise ValueError("count must be >= 1")
    body = bytes((slave_id, FUNC_READ, _hi(pid), _lo(pid), _hi(count), _lo(count)))
    return append_crc(body)


def build_write_single_request(slave_id: int, pid: int, word: int) -> bytes:
    """0x06 write single register 요청 프레임."""
    word &= 0xFFFF
    body = bytes((slave_id, FUNC_WRITE_SINGLE, _hi(pid), _lo(pid), _hi(word), _lo(word)))
    return append_crc(body)


def build_write_multiple_request(slave_id: int, pid: int, words: Sequence[int]) -> bytes:
    """0x10 write multiple registers 요청 프레임."""
    count = len(words)
    if count < 1:
        raise ValueError("words must not be empty")
    body = bytearray((slave_id, FUNC_WRITE_MULTIPLE, _hi(pid), _lo(pid), _hi(count), _lo(count), 2 * count))
    for word in words:
        word &= 0xFFFF
        body.append(_hi(word))
        body.append(_lo(word))
    return append_crc(bytes(body))


# --- response length 계산 (doc 02 §7) -------------------------------------------------

def read_response_length(count: int) -> int:
    """0x03 정상 응답 길이: [ID, FUNC, BYTE_COUNT] + 2*count + CRC."""
    return 5 + 2 * count


WRITE_SINGLE_RESPONSE_LENGTH = 8
WRITE_MULTIPLE_RESPONSE_LENGTH = 8


# --- response parsers ----------------------------------------------------------------

def raise_for_exception(frame: bytes, expected_func: int) -> None:
    """Modbus exception response면 ProtocolError를 올린다.

    CRC가 맞을 때만 exception으로 신뢰한다(doc 02 §9, CRC 오류 우선).
    """
    if len(frame) >= 5 and frame[1] == (expected_func | EXCEPTION_FLAG):
        if not check_crc(frame[:5]):
            raise CrcError(f"CRC mismatch on exception frame: {frame.hex()}")
        raise ProtocolError(
            f"modbus exception: func=0x{expected_func:02x} code=0x{frame[2]:02x}",
            function=expected_func,
            code=frame[2],
        )


def parse_read_response(frame: bytes, slave_id: int, count: int) -> list[int]:
    """0x03 응답을 검증하고 16비트 word 리스트(wire 순서)를 반환한다."""
    raise_for_exception(frame, FUNC_READ)
    expected = read_response_length(count)
    if len(frame) < expected:
        raise IncompleteResponseError(f"short read: got {len(frame)} want {expected}: {frame.hex()}")
    if len(frame) > expected:
        raise ProtocolError(f"oversized read: got {len(frame)} want {expected}: {frame.hex()}")
    if not check_crc(frame):
        raise CrcError(f"CRC mismatch: {frame.hex()}")
    if frame[0] != slave_id:
        raise ProtocolError(f"id mismatch: got {frame[0]} want {slave_id}")
    if frame[1] != FUNC_READ:
        raise ProtocolError(f"function mismatch: got 0x{frame[1]:02x} want 0x{FUNC_READ:02x}")
    byte_count = frame[2]
    if byte_count != 2 * count:
        raise ProtocolError(f"byte count mismatch: got {byte_count} want {2 * count}")
    data = frame[3:3 + byte_count]
    return [(data[2 * i] << 8) | data[2 * i + 1] for i in range(count)]


def parse_write_single_response(frame: bytes, request: bytes) -> None:
    """0x06 응답을 검증한다. CRC 제외 앞 6 byte가 요청 echo와 같아야 한다."""
    raise_for_exception(frame, FUNC_WRITE_SINGLE)
    if len(frame) != WRITE_SINGLE_RESPONSE_LENGTH:
        raise IncompleteResponseError(f"write-single response length {len(frame)} != 8: {frame.hex()}")
    if not check_crc(frame):
        raise CrcError(f"CRC mismatch: {frame.hex()}")
    if frame[:6] != request[:6]:
        raise ProtocolError(f"echo mismatch: req={request[:6].hex()} resp={frame[:6].hex()}")


def parse_write_multiple_response(frame: bytes, slave_id: int, pid: int, count: int) -> None:
    """0x10 응답을 검증한다. start address echo와 register count echo가 일치해야 한다."""
    raise_for_exception(frame, FUNC_WRITE_MULTIPLE)
    if len(frame) != WRITE_MULTIPLE_RESPONSE_LENGTH:
        raise IncompleteResponseError(f"write-multiple response length {len(frame)} != 8: {frame.hex()}")
    if not check_crc(frame):
        raise CrcError(f"CRC mismatch: {frame.hex()}")
    if frame[0] != slave_id:
        raise ProtocolError(f"id mismatch: got {frame[0]} want {slave_id}")
    if frame[1] != FUNC_WRITE_MULTIPLE:
        raise ProtocolError(f"function mismatch: got 0x{frame[1]:02x} want 0x{FUNC_WRITE_MULTIPLE:02x}")
    echoed_pid = (frame[2] << 8) | frame[3]
    if echoed_pid != pid:
        raise ProtocolError(f"start address echo mismatch: got {echoed_pid} want {pid}")
    echoed_count = (frame[4] << 8) | frame[5]
    if echoed_count != count:
        raise ProtocolError(f"register count echo mismatch: got {echoed_count} want {count}")
