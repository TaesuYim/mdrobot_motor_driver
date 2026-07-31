"""transport.py unit tests: SerialTransport (with a fake serial port injected).

No real pyserial port is opened; a fake is injected via SerialTransport.from_serial.
Read accumulation / frame assembly is verified together with ModbusClient.
"""

import pytest

from mdrobot.crc import append_crc
from mdrobot.protocol import ModbusClient
from mdrobot.transport import SerialTransport, Transport


class FakeSerial:
    """Minimal pyserial-compatible fake port to inject into SerialTransport.from_serial."""

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
    assert fake.flush_calls == 1  # wait for RS485 transmission to complete


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
    assert transport.read(2) == b""  # flush cleared the buffer


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
    """SerialTransport + ModbusClient integration: read_register decodes the response word."""
    # Read 1 word of PID_VERSION(1); response value 0x000D.
    response = append_crc(bytes((1, 0x03, 2, 0x00, 0x0D)))
    fake = FakeSerial(to_read=response, flush_clears=False)
    client = ModbusClient(SerialTransport.from_serial(fake), slave_id=1)

    value = client.read_register(1)
    assert value == 0x000D
    # Verify the request frame actually went out on the wire.
    expected_request = append_crc(bytes((1, 0x03, 0x00, 0x01, 0x00, 0x01)))
    assert bytes(fake.written) == expected_request


def test_modbus_client_short_read_raises():
    """A short response raises IncompleteResponseError."""
    from mdrobot.exceptions import IncompleteResponseError

    fake = FakeSerial(to_read=b"\x01\x03", flush_clears=False)  # header only, no body
    client = ModbusClient(SerialTransport.from_serial(fake), slave_id=1)
    with pytest.raises(IncompleteResponseError):
        client.read_register(1)


# --- resolve_port / MDROBOT_PORT --------------------------------------------------------


def test_resolve_port_explicit_wins(monkeypatch):
    monkeypatch.setenv("MDROBOT_PORT", "/dev/from_env")
    from mdrobot.transport import resolve_port

    assert resolve_port("/dev/explicit") == "/dev/explicit"


def test_resolve_port_falls_back_to_env(monkeypatch):
    monkeypatch.setenv("MDROBOT_PORT", "/dev/from_env")
    from mdrobot.transport import resolve_port

    assert resolve_port() == "/dev/from_env"
    assert resolve_port(None) == "/dev/from_env"
    assert resolve_port("") == "/dev/from_env"


def test_resolve_port_missing_raises_with_hint(monkeypatch):
    monkeypatch.delenv("MDROBOT_PORT", raising=False)
    from mdrobot.transport import resolve_port

    with pytest.raises(ValueError, match="MDROBOT_PORT"):
        resolve_port()


def test_driver_open_without_port_raises_before_serial(monkeypatch):
    """open() with no port and no env must fail with the config hint, not a pyserial error."""
    monkeypatch.delenv("MDROBOT_PORT", raising=False)
    from mdrobot.device import SingleMotorDriver

    with pytest.raises(ValueError, match="MDROBOT_PORT"):
        SingleMotorDriver.open()


def test_resolve_port_env_trimmed_and_whitespace_only_raises(monkeypatch):
    from mdrobot.transport import resolve_port

    monkeypatch.setenv("MDROBOT_PORT", "  /dev/padded \n")
    assert resolve_port() == "/dev/padded"
    monkeypatch.setenv("MDROBOT_PORT", "   ")
    with pytest.raises(ValueError, match="MDROBOT_PORT"):
        resolve_port()


# --- Modbus RTU inter-frame silence (t3.5) ------------------------------------------


def test_inter_frame_delay_scales_with_baud_and_is_fixed_above_19200():
    from mdrobot.constants import MODBUS_T3_5_FIXED, modbus_inter_frame_delay

    # At or below 19200: 3.5 characters of 11 bits.
    assert modbus_inter_frame_delay(19200) == pytest.approx(3.5 * 11 / 19200)
    assert modbus_inter_frame_delay(9600) == pytest.approx(3.5 * 11 / 9600)
    # Slower baud needs a longer gap.
    assert modbus_inter_frame_delay(9600) > modbus_inter_frame_delay(19200)
    # Above 19200 the spec fixes it at 1.750 ms.
    assert modbus_inter_frame_delay(38400) == MODBUS_T3_5_FIXED
    assert modbus_inter_frame_delay(115200) == MODBUS_T3_5_FIXED


def test_from_serial_disables_inter_frame_delay():
    """An injected fake port has no real line to keep idle — tests must not sleep."""
    t = SerialTransport.from_serial(FakeSerial())
    assert t.inter_frame_delay == 0.0


def test_write_waits_for_inter_frame_silence(monkeypatch):
    """write() holds off until the line has been idle for the configured gap."""
    t = SerialTransport.from_serial(FakeSerial())
    t.inter_frame_delay = 0.05

    now = [100.0]
    slept: list[float] = []
    monkeypatch.setattr("time.monotonic", lambda: now[0])
    monkeypatch.setattr("time.sleep", lambda s: slept.append(s))

    t._last_activity = now[0] - 0.02  # only 20 ms idle so far
    t.write(b"\x01")
    assert slept == [pytest.approx(0.03)]  # waits the remaining 30 ms

    # Already idle longer than the gap -> no wait.
    slept.clear()
    t._last_activity = now[0] - 1.0
    t.write(b"\x02")
    assert slept == []


def test_shared_transport_enforces_gap_across_slave_ids(monkeypatch):
    """The gap lives on the transport, so it also applies between different clients.

    Two ModbusClients on one bus is the twin topology; the id-to-id transition is
    exactly where a missing gap breaks framing on a fast adapter.
    """
    responses = append_crc(bytes([1, 0x03, 2, 0x00, 0x2A])) + append_crc(
        bytes([2, 0x03, 2, 0x00, 0x2B])
    )
    # flush_clears=False: transact() flushes before each request, which would drop the
    # queued second response from the fake.
    fake = FakeSerial(responses, flush_clears=False)
    t = SerialTransport.from_serial(fake)
    t.inter_frame_delay = 0.05

    now = [100.0]
    slept: list[float] = []
    monkeypatch.setattr("time.monotonic", lambda: now[0])
    monkeypatch.setattr("time.sleep", lambda s: slept.append(s))

    ModbusClient(t, 1).read_register(10)
    slept.clear()
    ModbusClient(t, 2).read_register(10)  # different slave id, same transport
    assert slept and slept[0] == pytest.approx(0.05)
