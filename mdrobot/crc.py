"""Modbus CRC16.

규칙(CLAUDE.md §2.x, doc 02 §8):
- 초기값 0xFFFF
- reflected polynomial 0xA001 (x^16 + x^15 + x^2 + 1)
- wire append 순서: low byte 먼저, high byte 나중
- 검증 KAT: crc16_modbus(b"123456789") == 0x4B37
"""

from __future__ import annotations

_POLY = 0xA001


def crc16_modbus(data: bytes) -> int:
    """Modbus CRC16을 정수로 반환한다(0..0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ _POLY
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def append_crc(frame_without_crc: bytes) -> bytes:
    """CRC를 계산해 low byte, high byte 순서로 뒤에 붙인다."""
    crc = crc16_modbus(frame_without_crc)
    return frame_without_crc + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def check_crc(frame: bytes) -> bool:
    """프레임 마지막 두 byte를 CRC(low first)로 보고 나머지 본문과 일치하는지 검사한다."""
    if len(frame) < 3:
        return False
    body, crc_lo, crc_hi = frame[:-2], frame[-2], frame[-1]
    expected = crc16_modbus(body)
    return (expected & 0xFF) == crc_lo and ((expected >> 8) & 0xFF) == crc_hi
