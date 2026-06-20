"""transport.py 단위 테스트: SerialTransport (가짜 직렬 포트 주입).

실제 pyserial 포트를 열지 않고 SerialTransport.from_serial로 fake를 주입한다.
read 누적/프레임 조립은 ModbusClient와 통합해 검증한다.
"""

import pytest

from mdrobot.crc import append_crc
from mdrobot.protocol import ModbusClient
from mdrobot.transport import SerialTransport, Transport


class FakeSerial:
    """SerialTransport.from_serial에 주입하는 최소 pyserial 호환 가짜 포트."""

    def __init__(self, to_read: bytes = b"", *, flush_clears: bool = True) -> None:
        self.port = "fake"
        self.baudrate = 19200
        self.is_open = True
        self.written = bytearray()
        self.flush_calls = 0
        self.reset_calls = 0
        self._rx = bytearray(to_read)
        self._flush_clears = flush_clears

    def write(self, data: bytes) -> int:
        self.written += data
        return len(data)

    def flush(self) -> None:
        self.flush_calls += 1

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def reset_input_buffer(self) -> None:
        self.reset_calls += 1
        if self._flush_clears:
            self._rx.clear()

    def close(self) -> None:
        self.is_open = False


def test_satisfies_transport_protocol():
    fake = FakeSerial()
    transport = SerialTransport.from_serial(fake)
    assert isinstance(transport, Transport)


def test_write_returns_count_and_flushes():
    fake = FakeSerial()
    transport = SerialTransport.from_serial(fake)
    n = transport.write(b"\x01\x02\x03")
    assert n == 3
    assert fake.written == b"\x01\x02\x03"
    assert fake.flush_calls == 1  # RS485 송신 완료 대기


def test_read_pulls_up_to_size():
    fake = FakeSerial(to_read=b"\xaa\xbb\xcc", flush_clears=False)
    transport = SerialTransport.from_serial(fake)
    assert transport.read(2) == b"\xaa\xbb"
    assert transport.read(2) == b"\xcc"
    assert transport.read(2) == b""


def test_flush_input_resets_buffer():
    fake = FakeSerial(to_read=b"\x01\x02")
    transport = SerialTransport.from_serial(fake)
    transport.flush_input()
    assert fake.reset_calls == 1
    assert transport.read(2) == b""  # flush가 버퍼를 비웠다


def test_close_and_is_open():
    fake = FakeSerial()
    transport = SerialTransport.from_serial(fake)
    assert transport.is_open is True
    transport.close()
    assert transport.is_open is False


def test_context_manager_closes():
    fake = FakeSerial()
    with SerialTransport.from_serial(fake) as transport:
        assert transport.is_open is True
    assert fake.is_open is False


def test_modbus_client_read_over_fake_serial():
    """SerialTransport + ModbusClient 통합: read_register가 응답 word를 디코딩한다."""
    # PID_VERSION(1) 1 word 읽기, 값 0x000D 응답.
    response = append_crc(bytes((1, 0x03, 2, 0x00, 0x0D)))
    fake = FakeSerial(to_read=response, flush_clears=False)
    client = ModbusClient(SerialTransport.from_serial(fake), slave_id=1)

    value = client.read_register(1)
    assert value == 0x000D
    # 요청 프레임이 실제로 wire로 나갔는지 확인.
    expected_request = append_crc(bytes((1, 0x03, 0x00, 0x01, 0x00, 0x01)))
    assert bytes(fake.written) == expected_request


def test_modbus_client_short_read_raises():
    """응답이 부족하면 IncompleteResponseError."""
    from mdrobot.exceptions import IncompleteResponseError

    fake = FakeSerial(to_read=b"\x01\x03", flush_clears=False)  # 헤더만, 본문 없음
    client = ModbusClient(SerialTransport.from_serial(fake), slave_id=1)
    with pytest.raises(IncompleteResponseError):
        client.read_register(1)
