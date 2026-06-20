"""ModbusClient 단위 테스트.

실제 직렬 포트 없이 가짜 transport를 주입해 송신 프레임과 응답 파싱을 검증한다.
(transport 인터페이스는 mdrobot.transport.Transport, 실제 구현은 Phase 2)
"""

import pytest

from mdrobot import frame
from mdrobot.crc import append_crc
from mdrobot.exceptions import CrcError, IncompleteResponseError, ProtocolError
from mdrobot.protocol import ModbusClient


class FakeTransport:
    """버퍼에서 순차적으로 read를 제공하고, write/flush를 기록하는 가짜 transport."""

    def __init__(self, response: bytes = b"") -> None:
        self._rx = bytearray(response)
        self.written = bytearray()
        self.flush_count = 0

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def flush_input(self) -> None:
        self.flush_count += 1


def test_read_registers_sends_request_and_parses():
    response = append_crc(bytes.fromhex("01 03 02 00 0D"))
    transport = FakeTransport(response)
    client = ModbusClient(transport, slave_id=1)

    assert client.read_registers(1, 1) == [0x000D]
    assert transport.written == frame.build_read_request(1, 1, 1)
    assert transport.flush_count == 1


def test_read_register_convenience():
    response = append_crc(bytes.fromhex("01 03 02 12 34"))
    client = ModbusClient(FakeTransport(response), slave_id=1)
    assert client.read_register(196) == 0x1234


def test_write_register_echo_ok():
    client_id = 1
    request = frame.build_write_single_request(client_id, 130, 100)
    transport = FakeTransport(request)  # 0x06 응답은 요청 echo
    client = ModbusClient(transport, slave_id=client_id)

    client.write_register(130, 100)
    assert transport.written == request


def test_write_registers_echo_ok():
    response = append_crc(bytes.fromhex("01 10 00 DB 00 03"))
    transport = FakeTransport(response)
    client = ModbusClient(transport, slave_id=1)

    client.write_registers(219, [0x5678, 0x1234, 0x9ABC])
    assert transport.written == frame.build_write_multiple_request(1, 219, [0x5678, 0x1234, 0x9ABC])


def test_exception_response_raises_protocol_error():
    response = append_crc(bytes([1, 0x83, 0x02]))
    client = ModbusClient(FakeTransport(response), slave_id=1)
    with pytest.raises(ProtocolError) as exc:
        client.read_registers(1, 1)
    assert exc.value.code == 0x02


def test_crc_error_raises():
    bad = bytearray(append_crc(bytes.fromhex("01 03 02 00 0D")))
    bad[-1] ^= 0xFF
    client = ModbusClient(FakeTransport(bytes(bad)), slave_id=1)
    with pytest.raises(CrcError):
        client.read_registers(1, 1)


def test_short_response_raises_incomplete():
    client = ModbusClient(FakeTransport(b"\x01"), slave_id=1)
    with pytest.raises(IncompleteResponseError):
        client.read_registers(1, 1)


def test_read_long_signed():
    # value 0x12345678 -> words low 0x5678, high 0x1234
    response = append_crc(bytes.fromhex("01 03 04 56 78 12 34"))
    client = ModbusClient(FakeTransport(response), slave_id=1)
    assert client.read_long(197) == 0x12345678


def test_read_long_negative():
    # -2 -> 0xFFFFFFFE -> low 0xFFFE, high 0xFFFF
    response = append_crc(bytes.fromhex("01 03 04 FF FE FF FF"))
    client = ModbusClient(FakeTransport(response), slave_id=1)
    assert client.read_long(197) == -2


def test_read_long_unsigned():
    response = append_crc(bytes.fromhex("01 03 04 FF FE FF FF"))
    client = ModbusClient(FakeTransport(response), slave_id=1)
    assert client.read_long(197, signed=False) == 0xFFFFFFFE


def test_write_long_splits_low_word_first():
    response = append_crc(bytes.fromhex("01 10 00 C5 00 02"))
    transport = FakeTransport(response)
    client = ModbusClient(transport, slave_id=1)

    client.write_long(197, 0x12345678)
    assert transport.written == frame.build_write_multiple_request(1, 197, [0x5678, 0x1234])
