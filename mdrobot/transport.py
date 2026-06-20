"""Transport 인터페이스와 직렬 구현.

`Transport`는 protocol 계층이 의존하는 최소 인터페이스(Protocol)다. 단위 테스트는
가짜 transport를 주입하고, 실물 통신은 `SerialTransport`(pyserial 기반)를 쓴다.

pyserial은 optional dependency다(pyproject `serial` extra). `SerialTransport`를 실제로
생성할 때만 import하므로, pyserial이 없어도 이 모듈과 protocol 계층은 import된다.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

from .constants import DEFAULT_BAUDRATE, DEFAULT_TIMEOUT


@runtime_checkable
class Transport(Protocol):
    """직렬 통신 transport가 제공해야 하는 최소 인터페이스."""

    def write(self, data: bytes) -> int:
        """data를 모두 전송하고 보낸 byte 수를 반환한다."""
        ...

    def read(self, size: int) -> bytes:
        """최대 size byte를 읽어 반환한다. 더 적게 반환할 수 있다."""
        ...

    def flush_input(self) -> None:
        """입력 버퍼에 남은 byte를 버린다(요청 전 호출 권장, doc 02 §1)."""
        ...


class SerialTransport:
    """pyserial 기반 RS485 / Modbus RTU 직렬 transport.

    `Transport` 프로토콜을 구현한다. 기본값은 MD400/PNT50 검증 장비 기준 19200 8N1
    (doc 01 §2)이다. RS485 half-duplex에서 송신 후 바로 수신해야 하므로 write 뒤에
    flush로 송신 완료를 기다린다.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        timeout: float = DEFAULT_TIMEOUT,
        settle: float = 0.2,
        write_timeout: float = 1.0,
    ) -> None:
        import time

        import serial  # lazy import: pyserial은 optional dependency

        self.port = port
        self.baudrate = baudrate
        # write_timeout: 포트가 wedge돼도 write/flush가 무한 대기하지 않게 한다(종료 hang 방지).
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=write_timeout,
        )
        # USB-직렬 어댑터(FTDI/CH340)는 open 직후 라인이 안정화될 때까지 잠깐 부팅 노이즈/
        # 잔여 바이트가 있을 수 있다(실물: open 직후 첫 1~2 트랜잭션이 0xFF 노이즈/desync).
        # settle 동안 대기한 뒤 RX/TX 버퍼를 비워 첫 트랜잭션 정렬을 보장한다.
        if settle > 0:
            time.sleep(settle)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    @classmethod
    def from_serial(cls, serial_port: Any) -> "SerialTransport":
        """이미 열린 pyserial 호환 객체를 감싼다(테스트/고급 사용).

        실제 pyserial.Serial을 새로 열지 않고 주입받는다. 단위 테스트에서 가짜 직렬
        포트를 넣을 때 사용한다.
        """
        obj = cls.__new__(cls)
        obj.port = getattr(serial_port, "port", None)
        obj.baudrate = getattr(serial_port, "baudrate", None)
        obj._serial = serial_port
        return obj

    def write(self, data: bytes) -> int:
        """data를 전송하고 송신 완료까지 기다린 뒤 보낸 byte 수를 반환한다."""
        written = self._serial.write(data)
        self._serial.flush()
        return written if written is not None else len(data)

    def read(self, size: int) -> bytes:
        """최대 size byte를 읽는다. timeout이 지나면 더 적게(또는 빈 bytes) 반환한다."""
        return self._serial.read(size)

    def flush_input(self) -> None:
        """수신 버퍼에 남은 byte를 버린다(요청 전 호출, doc 02 §1)."""
        self._serial.reset_input_buffer()

    def close(self) -> None:
        """직렬 포트를 닫는다."""
        self._serial.close()

    @property
    def is_open(self) -> bool:
        """포트가 열려 있는지 여부."""
        return bool(getattr(self._serial, "is_open", False))

    def __enter__(self) -> "SerialTransport":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
