"""CRC16 단위 테스트 (CLAUDE.md §7, doc 02 §8 / doc 07 §3.1)."""

from mdrobot.crc import append_crc, check_crc, crc16_modbus


def test_crc_kat():
    # 필수 KAT: "123456789" -> 0x4B37.
    assert crc16_modbus(b"123456789") == 0x4B37


def test_crc_wire_order_low_byte_first():
    # wire 순서는 low byte, high byte. KAT는 37 4B.
    framed = append_crc(b"123456789")
    assert framed[-2] == 0x37
    assert framed[-1] == 0x4B


def test_append_crc_matches_doc_read_version():
    # doc 07 §3.2: 01 03 00 01 00 01 -> CRC D5 CA.
    body = bytes.fromhex("01 03 00 01 00 01")
    assert append_crc(body) == bytes.fromhex("01 03 00 01 00 01 D5 CA")


def test_check_crc_roundtrip():
    framed = append_crc(b"hello modbus")
    assert check_crc(framed) is True


def test_check_crc_detects_corruption():
    framed = bytearray(append_crc(b"hello modbus"))
    framed[0] ^= 0xFF
    assert check_crc(bytes(framed)) is False


def test_check_crc_rejects_too_short():
    assert check_crc(b"\x01\x02") is False
