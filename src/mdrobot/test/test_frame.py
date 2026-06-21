"""Frame builder/parser unit tests.

The build side must match the golden vectors (full frame hex, CRC included)
exactly. The parse side follows the validation checklist.
"""

import pytest

from mdrobot import frame
from mdrobot.crc import append_crc, check_crc
from mdrobot.exceptions import CrcError, IncompleteResponseError, ProtocolError


# --- golden vectors ------------------------------------------------------------------

def test_build_read_version():
    # PID_VERSION(1) read
    assert frame.build_read_request(1, 1, 1) == bytes.fromhex("01 03 00 01 00 01 D5 CA")


def test_build_read_monitor_196():
    # PID_MONITOR(196=0xC4) count 6
    assert frame.build_read_request(1, 196, 6) == bytes.fromhex("01 03 00 C4 00 06 84 35")


def test_build_read_pnt_monitor_216():
    # PID_PNT_MONITOR(216=0xD8) count 7
    assert frame.build_read_request(1, 216, 7) == bytes.fromhex("01 03 00 D8 00 07 84 33")


def test_build_read_pnt_main_data_210():
    # PID_PNT_MAIN_DATA(210=0xD2) count 9
    assert frame.build_read_request(1, 210, 9) == bytes.fromhex("01 03 00 D2 00 09 25 F5")


def test_build_write_velocity_plus_100():
    # PID_VEL_CMD(130=0x82) = +100 rpm
    assert frame.build_write_single_request(1, 130, 100) == bytes.fromhex("01 06 00 82 00 64 28 09")


def test_build_write_velocity_minus_100():
    # PID_VEL_CMD(130) = -100 rpm -> 0xFF9C
    assert frame.build_write_single_request(1, 130, 0xFF9C) == bytes.fromhex("01 06 00 82 FF 9C 68 7B")


def test_build_write_long_pid197():
    # long 0x12345678 -> words [0x5678, 0x1234]
    assert frame.build_write_multiple_request(1, 197, [0x5678, 0x1234]) == bytes.fromhex(
        "01 10 00 C5 00 02 04 56 78 12 34 A3 26"
    )


def test_build_posi_vel_cmd_219_long_plus_word():
    # position 0x12345678 + max speed 0x9ABC
    assert frame.build_write_multiple_request(1, 219, [0x5678, 0x1234, 0x9ABC]) == bytes.fromhex(
        "01 10 00 DB 00 03 06 56 78 12 34 9A BC 10 57"
    )


def test_build_pnt_posi_vel_cmd_206_nword():
    # dual position + speed (12 bytes)
    words = [0x5678, 0x1234, 0x9ABC, 0x7890, 0x3456, 0x0123]
    assert frame.build_write_multiple_request(1, 206, words) == bytes.fromhex(
        "01 10 00 CE 00 06 0C 56 78 12 34 9A BC 78 90 34 56 01 23 C0 C0"
    )


def test_build_pnt_vel_cmd_independent_words_no_swap():
    # Two independent words are NOT word-swapped.
    built = frame.build_write_multiple_request(1, 207, [0x1234, 0x5678])
    assert built[:-2] == bytes.fromhex("01 10 00 CF 00 02 04 12 34 56 78")
    assert check_crc(built)


# --- read response validation --------------------------------------------------------

def test_parse_read_single_word():
    resp = append_crc(bytes.fromhex("01 03 02 00 0D"))  # expected response, version 1.3
    assert frame.parse_read_response(resp, 1, 1) == [0x000D]


def test_parse_read_multi_word():
    resp = append_crc(bytes.fromhex("01 03 04 00 0D 12 34"))
    assert frame.parse_read_response(resp, 1, 2) == [0x000D, 0x1234]


def test_parse_read_exception_response():
    resp = append_crc(bytes([1, 0x83, 0x02]))  # 0x03 | 0x80, illegal data address
    with pytest.raises(ProtocolError) as exc:
        frame.parse_read_response(resp, 1, 1)
    assert exc.value.function == 0x03
    assert exc.value.code == 0x02


def test_parse_read_crc_error():
    resp = bytearray(append_crc(bytes.fromhex("01 03 02 00 0D")))
    resp[-1] ^= 0xFF
    with pytest.raises(CrcError):
        frame.parse_read_response(bytes(resp), 1, 1)


def test_parse_read_byte_count_mismatch():
    resp = append_crc(bytes([1, 0x03, 0x03, 0x00, 0x0D, 0x00, 0x00]))  # byte_count=3, len fits count=2
    with pytest.raises(ProtocolError, match="byte count"):
        frame.parse_read_response(resp, 1, 2)


def test_parse_read_id_mismatch():
    resp = append_crc(bytes.fromhex("02 03 02 00 0D"))
    with pytest.raises(ProtocolError, match="id mismatch"):
        frame.parse_read_response(resp, 1, 1)


def test_parse_read_short():
    resp = append_crc(bytes.fromhex("01 03 02 00"))  # too short for count=1
    with pytest.raises(IncompleteResponseError):
        frame.parse_read_response(resp, 1, 1)


# --- write single validation ---------------------------------------------------------

def test_parse_write_single_echo_ok():
    request = frame.build_write_single_request(1, 130, 100)
    response = request  # the 0x06 response echoes the request
    frame.parse_write_single_response(response, request)  # must not raise


def test_parse_write_single_echo_mismatch():
    request = frame.build_write_single_request(1, 130, 100)
    bad = bytearray(request)
    bad[5] ^= 0x01  # corrupt the data low byte
    bad = append_crc(bytes(bad)[:6])
    with pytest.raises(ProtocolError, match="echo"):
        frame.parse_write_single_response(bad, request)


# --- write multiple validation -------------------------------------------------------

def test_parse_write_multiple_ok():
    resp = append_crc(bytes.fromhex("01 10 00 DB 00 03"))  # PID 219, count 3 echo
    frame.parse_write_multiple_response(resp, 1, 219, 3)


def test_parse_write_multiple_count_mismatch():
    resp = append_crc(bytes.fromhex("01 10 00 DB 00 02"))  # count echo 2 != 3
    with pytest.raises(ProtocolError, match="count echo"):
        frame.parse_write_multiple_response(resp, 1, 219, 3)


def test_parse_write_multiple_address_mismatch():
    resp = append_crc(bytes.fromhex("01 10 00 C5 00 03"))  # addr echo 197 != 219
    with pytest.raises(ProtocolError, match="address echo"):
        frame.parse_write_multiple_response(resp, 1, 219, 3)
