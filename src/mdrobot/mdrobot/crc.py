"""Modbus CRC16.

Rules:
- initial value 0xFFFF
- reflected polynomial 0xA001 (x^16 + x^15 + x^2 + 1)
- wire append order: low byte first, then high byte
- known-answer test: crc16_modbus(b"123456789") == 0x4B37
"""

from __future__ import annotations

_POLY = 0xA001


def crc16_modbus(data: bytes) -> int:
    """Return the Modbus CRC16 as an integer (0..0xFFFF)."""
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
    """Compute the CRC and append it as low byte then high byte."""
    crc = crc16_modbus(frame_without_crc)
    return frame_without_crc + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def check_crc(frame: bytes) -> bool:
    """Treat the last two bytes as CRC (low first) and verify against the body."""
    if len(frame) < 3:
        return False
    body, crc_lo, crc_hi = frame[:-2], frame[-2], frame[-1]
    expected = crc16_modbus(body)
    return (expected & 0xFF) == crc_lo and ((expected >> 8) & 0xFF) == crc_hi
