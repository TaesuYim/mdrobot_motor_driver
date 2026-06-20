"""고수준 장치 드라이버: SingleMotorDriver(MD400) / DualMotorDriver(PNT50).

이 계층은 `ModbusClient`(raw primitive) 위에 직관적인 모터 API를 올린다. raw 접근은
`self.client`로 항상 유지된다(CLAUDE.md §6).

실물 검증 기반 구동 모델 (2026-06-19, docs/dev/test-log.md Phase 3/6/7):

- serial 속도 명령은 `PID_UI_COM(78)=1`(serial 단독) + `PID_START_STOP(100)=1`(run-latch
  arm)이 선행돼야 동작한다. 이 두 가지가 없으면 명령 echo는 성공해도 모터 reference가 0에
  머문다. `enable()`이 이 두 쓰기를 수행한다.
- 모터 속도원: MD400/PNT50 모터1 = `PID_VEL_CMD(130)`, PNT50 모터2 = `PID_VEL_CMD2(131)`.
  signed rpm, `+` = position 증가(CCW), `-` = position 감소(CW).
- PNT50은 명령 후 회전까지 약 1초 지연이 있다. 명령 직후 곧바로 0을 보내면 회전을 놓친다.

안전: `set_velocity*`는 즉시 모터를 돌린다. 호출 전 `enable()`, 종료 시 `stop()` 후
`torque_off*()` 또는 `disable()`을 호출한다.
"""

from __future__ import annotations

import time

from . import registers as reg
from .codec import int16, split_i32_low_word_first, word_from_int16
from .constants import DEFAULT_BAUDRATE, DEFAULT_SLAVE_ID, DEFAULT_TIMEOUT
from .exceptions import MdrobotError
from .protocol import ModbusClient
from .status import (
    DualMonitor,
    Monitor,
    StatusBits,
    decode_monitor,
    decode_pnt_main_data,
    decode_pnt_monitor,
)

# PID_UI_COM(78) 값: serial 통신 단독 제어(CTRL I/O 무시).
UI_COM_SERIAL = 1
# PID_START_STOP(100) 값.
START = 1
STOP = 0


class _DriverBase:
    """싱글/듀얼 공통: 연결 관리, 버전/전압/상태, enable/disable, alarm reset."""

    def __init__(self, client: ModbusClient) -> None:
        self.client = client

    @classmethod
    def open(
        cls,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        slave_id: int = DEFAULT_SLAVE_ID,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        """포트를 열어 드라이버를 만든다(편의 생성자). `close()`로 닫는다."""
        from .transport import SerialTransport

        transport = SerialTransport(port, baudrate, timeout=timeout)
        return cls(ModbusClient(transport, slave_id=slave_id))

    def close(self) -> None:
        """하부 transport를 닫는다(있으면)."""
        close = getattr(self.client.transport, "close", None)
        if callable(close):
            close()

    def __enter__(self):
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # --- 공통 read -----------------------------------------------------------------
    def get_version(self) -> int:
        """firmware/protocol version 코드(DL byte)."""
        return self.client.read_register(reg.PID_VERSION) & 0xFF

    def get_voltage(self) -> float:
        """공급 전압(V). raw 0.1V 단위."""
        return self.client.read_register(reg.PID_VOLT_IN) / 10.0

    def get_status(self) -> StatusBits:
        """`PID_CTRL_STATUS(34)` status1 비트."""
        return StatusBits.from_byte(self.client.read_register(reg.PID_CTRL_STATUS) & 0xFF)

    def ping(self) -> bool:
        """버전 읽기로 통신 가능 여부를 확인한다."""
        try:
            self.get_version()
            return True
        except MdrobotError:
            return False

    # --- enable / safety -----------------------------------------------------------
    def enable(self) -> None:
        """serial 단독 제어 + run-latch arm. 속도 명령 전에 반드시 호출.

        실물 확인(test-log Phase 3): 이 두 쓰기(`PID_UI_COM=1`, `PID_START_STOP=1`)가
        없으면 `set_velocity`가 echo는 되지만 모터가 돌지 않는다. arm(START_STOP=1) 시
        velocity 모드가 잔여 `COM_TAR_SPEED`로 돌지 않도록 속도원을 먼저 0으로 둔다
        (속도 구동의 속도원은 VEL_CMD라 영향 없음). 위치 제어는 UI_COM=1만으로 동작하지만
        enable()을 호출해도 무방하다.
        """
        self.client.write_register(reg.PID_UI_COM, UI_COM_SERIAL)
        self.client.write_register(reg.PID_COM_TAR_SPEED, 0)
        self.client.write_register(reg.PID_START_STOP, START)

    def disable(self) -> None:
        """run-latch를 해제한다(START_STOP=0). 모터 reference가 끊긴다."""
        self.client.write_register(reg.PID_START_STOP, STOP)

    def reset_alarm(self) -> None:
        """alarm을 리셋한다(`CMD_ALARM_RESET`)."""
        self.client.command(reg.CMD_ALARM_RESET)


class SingleMotorDriver(_DriverBase):
    """싱글 채널(MD400) 모터 드라이버."""

    def set_velocity(self, rpm: int) -> None:
        """WARNING: 즉시 모터를 회전시킨다. `enable()` 선행 필요.

        rpm은 signed. 실물 확인(MD400): `+` = position 증가 방향(CCW), `-` = CW.
        """
        self.client.write_register(reg.PID_VEL_CMD, word_from_int16(rpm))

    def stop(self) -> None:
        """속도 0 명령으로 감속 정지한다(closed-loop)."""
        self.client.write_register(reg.PID_VEL_CMD, 0)

    def brake(self) -> None:
        """WARNING: electric brake를 건다(`PID_BRAKE`)."""
        self.client.write_register(reg.PID_BRAKE, 0)

    def torque_off(self) -> None:
        """모터를 free 상태로 둔다(`PID_TQ_OFF`). 출력 차단, 관성 정지."""
        self.client.write_register(reg.PID_TQ_OFF, 0)

    def reset_position(self) -> None:
        """position count를 0으로 리셋한다(`PID_POSI_RESET`). 위치 기준이 사라진다."""
        self.client.write_register(reg.PID_POSI_RESET, 0)

    def get_speed(self) -> int:
        """실측 속도(signed rpm)."""
        return int16(self.client.read_register(reg.PID_INT_RPM_DATA))

    def get_current(self) -> float:
        """전류(A). raw 0.1A 단위. 부호 해석은 부하 시 재확인 대상."""
        return self.client.read_register(reg.PID_TQ_DATA) / 10.0

    def get_position(self) -> int:
        """position count(signed long)."""
        return self.client.read_long(reg.PID_POSI_DATA)

    def read_monitor(self) -> Monitor:
        """`PID_MONITOR(196)` 한 번 읽어 speed/current/output/position/status 반환."""
        return decode_monitor(self.client.read_registers(reg.PID_MONITOR, 6))

    # --- 위치 제어 (실물 확인: UI_COM=1만 필요, START_STOP arm 불필요) -----------------
    def _write_posi_vel(self, pid: int, position: int, speed: int) -> None:
        """position(long, low word first) + max speed(word) 6-byte 명령(doc 07 §3.6)."""
        low, high = split_i32_low_word_first(position)
        self.client.write_registers(pid, [low, high, speed & 0xFFFF])

    def move_to(self, position: int, speed: int = 100) -> None:
        """WARNING: 즉시 절대 position으로 이동한다. `enable()` 선행 필요.

        position은 count, speed는 최대 속도(양수 rpm). 실물 확인(MD400): 목표 도달 시
        정지하고 `get_in_position()`이 True가 된다. `+` target = position 증가 방향.
        """
        self._write_posi_vel(reg.PID_POSI_VEL_CMD, position, speed)

    def move_by(self, delta: int, speed: int = 100) -> None:
        """WARNING: 현재 위치 기준 상대 이동(`PID_INC_POSI_VEL_CMD`). `enable()` 선행 필요."""
        self._write_posi_vel(reg.PID_INC_POSI_VEL_CMD, delta, speed)

    def get_in_position(self) -> bool:
        """위치 제어 목표 도달 여부(`PID_IN_POSITION_OK`)."""
        return bool(self.client.read_register(reg.PID_IN_POSITION_OK))

    def wait_in_position(self, timeout: float = 10.0, poll: float = 0.1) -> bool:
        """`get_in_position()`이 True가 될 때까지 기다린다. timeout 시 False."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.get_in_position():
                return True
            time.sleep(poll)
        return False


class DualMotorDriver(_DriverBase):
    """듀얼 채널(PNT50) 모터 드라이버. channel은 1 또는 2.

    실물 확인(test-log Phase 6/7): 모터1=`PID_VEL_CMD(130)`, 모터2=`PID_VEL_CMD2(131)`로
    개별 구동한다(두 PID는 독립이므로 한쪽만 바꿔도 반대쪽 target에 영향 없음). 명령 후
    회전까지 약 1초 지연이 있으니 명령 직후 곧바로 0을 보내지 않는다.
    """

    _CH_VEL_PID = {1: reg.PID_VEL_CMD, 2: reg.PID_VEL_CMD2}

    @staticmethod
    def _ch_flag_word(channel: int) -> int:
        """PNT brake/tq-off용 word. ch1 -> 0x0001(DL), ch2 -> 0x0100(DH)."""
        if channel == 1:
            return 0x0001
        if channel == 2:
            return 0x0100
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def _vel_pid(self, channel: int) -> int:
        try:
            return self._CH_VEL_PID[channel]
        except KeyError:
            raise ValueError(f"channel must be 1 or 2, got {channel}") from None

    def set_velocities(self, rpm1: int, rpm2: int) -> None:
        """WARNING: 즉시 두 모터를 회전시킨다. `enable()` 선행 + ~1s 지연 유의.

        signed rpm. `+` = position 증가 방향. 모터1/모터2 독립.
        """
        self.client.write_register(reg.PID_VEL_CMD, word_from_int16(rpm1))
        self.client.write_register(reg.PID_VEL_CMD2, word_from_int16(rpm2))

    def set_velocity(self, channel: int, rpm: int) -> None:
        """WARNING: 지정 channel(1/2)만 즉시 회전시킨다. 반대 channel은 그대로 둔다."""
        self.client.write_register(self._vel_pid(channel), word_from_int16(rpm))

    def stop(self) -> None:
        """두 모터 속도 0."""
        self.set_velocities(0, 0)

    def stop_channel(self, channel: int) -> None:
        """지정 channel 속도 0."""
        self.client.write_register(self._vel_pid(channel), 0)

    def brake_both(self) -> None:
        """WARNING: 두 모터 electric brake(`PID_PNT_BRAKE`=0x0101)."""
        self.client.write_register(reg.PID_PNT_BRAKE, 0x0101)

    def brake(self, channel: int) -> None:
        """WARNING: 지정 channel만 electric brake."""
        self.client.write_register(reg.PID_PNT_BRAKE, self._ch_flag_word(channel))

    def torque_off_both(self) -> None:
        """두 모터 free 상태(`PID_PNT_TQ_OFF`=0x0101)."""
        self.client.write_register(reg.PID_PNT_TQ_OFF, 0x0101)

    def torque_off(self, channel: int) -> None:
        """지정 channel만 free 상태."""
        self.client.write_register(reg.PID_PNT_TQ_OFF, self._ch_flag_word(channel))

    def read_monitor(self) -> DualMonitor:
        """`PID_PNT_MONITOR(216)`: 두 모터 speed/position/status(current 없음)."""
        return decode_pnt_monitor(self.client.read_registers(reg.PID_PNT_MONITOR, 7))

    def read_main_data(self) -> DualMonitor:
        """`PID_PNT_MAIN_DATA(210)`: 두 모터 speed/current/position/status."""
        return decode_pnt_main_data(self.client.read_registers(reg.PID_PNT_MAIN_DATA, 9))

    def get_speed(self, channel: int) -> int:
        """지정 channel 실측 속도(signed rpm), PNT monitor 기준."""
        mon = self.read_monitor()
        if channel == 1:
            return mon.motor1.speed_rpm
        if channel == 2:
            return mon.motor2.speed_rpm
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def get_positions(self) -> tuple[int, int]:
        """(motor1 position, motor2 position), PNT monitor 기준."""
        mon = self.read_monitor()
        return mon.motor1.position, mon.motor2.position

    def get_position(self, channel: int) -> int:
        """지정 channel position count."""
        p1, p2 = self.get_positions()
        if channel == 1:
            return p1
        if channel == 2:
            return p2
        raise ValueError(f"channel must be 1 or 2, got {channel}")

    def reset_position(self) -> None:
        """두 모터 position count를 0으로 리셋한다(`PID_POSI_RESET`, 실물: 양쪽 동시 리셋)."""
        self.client.write_register(reg.PID_POSI_RESET, 0)

    # --- 위치 제어 (실물 확인: UI_COM=1만 필요, START_STOP arm 불필요) -----------------
    def _write_pnt_posi_vel(self, pid: int, pos1: int, spd1: int, pos2: int, spd2: int) -> None:
        """[pos1 long, spd1 word, pos2 long, spd2 word] 12-byte 명령(doc 06 §5, 07 §3.7)."""
        l1, h1 = split_i32_low_word_first(pos1)
        l2, h2 = split_i32_low_word_first(pos2)
        self.client.write_registers(pid, [l1, h1, spd1 & 0xFFFF, l2, h2, spd2 & 0xFFFF])

    def move_to_both(self, pos1: int, pos2: int, speed1: int = 100, speed2: int | None = None) -> None:
        """WARNING: 즉시 두 모터를 절대 position으로 이동(`PID_PNT_POSI_VEL_CMD`). `enable()` 선행.

        speed2 생략 시 speed1 사용. 실물 확인(PNT50): 두 모터 독립 target·양방향 도달.
        """
        self._write_pnt_posi_vel(
            reg.PID_PNT_POSI_VEL_CMD, pos1, speed1, pos2, speed1 if speed2 is None else speed2
        )

    def move_by_both(self, delta1: int, delta2: int, speed1: int = 100, speed2: int | None = None) -> None:
        """WARNING: 두 모터 상대 이동(`PID_PNT_INC_POSI_VEL_CMD`). `enable()` 선행."""
        self._write_pnt_posi_vel(
            reg.PID_PNT_INC_POSI_VEL_CMD, delta1, speed1, delta2, speed1 if speed2 is None else speed2
        )
