"""status bit / monitor 디코딩 계층.

doc 05를 따르는 순수 함수 계층이다. 하드웨어가 필요 없고 단위 테스트로 검증한다.
모든 bit 의미는 문서 기반 가정이며, 실제 입력 변화와 bit 대응은 `PID_DI(48)` /
`PID_CTRL_STATUS(34)` 하드웨어 로그로 확정한다(CLAUDE.md §2.5).

byte 순서 주의: `PID_MONITOR`의 word 5는 wire상 `D6H D6L`이므로
`status2 = word >> 8`, `status1 = word & 0xFF`다(doc 05 §3).
"""

from __future__ import annotations

from dataclasses import dataclass

from .codec import int16, join_i32_low_word_first

# --- status1 bit: PID_CTRL_STATUS(34) 및 PID_MONITOR status1 (doc 05 §2) -------------
STATUS1_BIT_NAMES = {
    0: "ALARM",
    1: "CTRL_FAIL",
    2: "OVER_VOLT",
    3: "OVER_TEMP",
    4: "OVER_LOAD",
    5: "HALL_OR_ENC_FAIL",
    6: "INV_VEL",
    7: "STALL",
}

# --- status2 bit: PID_MONITOR status2 (doc 05 §4) ------------------------------------
STATUS2_BIT_NAMES = {
    0: "REGEN_OVER_TEMP",
    1: "ENC_FAIL",
}

# --- PID_DI(48) digital input bit (doc 05 §8) ---------------------------------------
DI_BIT_NAMES = {
    0: "INT_SPEED",
    1: "ALARM_RESET",
    2: "DIR",
    3: "RUN_BRAKE",
    4: "START_STOP",
    5: "ENC_B",
    6: "ENC_A",
}


def active_bits(value: int, names: dict[int, str]) -> list[str]:
    """value에서 set된 bit를 names 맵에 따라 이름 리스트로 반환한다(bit 오름차순)."""
    return [name for bit, name in sorted(names.items()) if value & (1 << bit)]


@dataclass(frozen=True)
class StatusBits:
    """status1 byte(8 bit) 해석 결과 (doc 05 §2)."""

    raw: int
    alarm: bool
    ctrl_fail: bool
    over_voltage: bool
    over_temperature: bool
    overload: bool
    hall_or_encoder_fail: bool
    inverse_velocity: bool
    stall: bool

    @classmethod
    def from_byte(cls, value: int) -> "StatusBits":
        v = value & 0xFF
        return cls(
            raw=v,
            alarm=bool(v & (1 << 0)),
            ctrl_fail=bool(v & (1 << 1)),
            over_voltage=bool(v & (1 << 2)),
            over_temperature=bool(v & (1 << 3)),
            overload=bool(v & (1 << 4)),
            hall_or_encoder_fail=bool(v & (1 << 5)),
            inverse_velocity=bool(v & (1 << 6)),
            stall=bool(v & (1 << 7)),
        )

    @property
    def active(self) -> list[str]:
        """set된 status1 bit 이름 리스트."""
        return active_bits(self.raw, STATUS1_BIT_NAMES)


@dataclass(frozen=True)
class Monitor:
    """`PID_MONITOR(196)` single monitor 해석 결과 (doc 05 §3, §11)."""

    speed_rpm: int
    current_a: float | None
    output_raw: int | None
    position: int
    status: StatusBits
    status2_raw: int = 0


def decode_monitor(words: list[int]) -> Monitor:
    """`PID_MONITOR(196)` 6-word 응답을 Monitor로 디코딩한다(doc 05 §3).

    current는 0.1A 단위, position은 INT32 long(low word first)이다. current의 부호
    해석(INT16/UINT16)은 문서상 후보이므로 unsigned 0.1A로 두고 실물로 확정한다.
    """
    if len(words) != 6:
        raise ValueError(f"PID_MONITOR expects 6 words, got {len(words)}")
    return Monitor(
        speed_rpm=int16(words[0]),
        current_a=words[1] / 10.0,
        output_raw=int16(words[2]),
        position=join_i32_low_word_first(words[3], words[4]),
        status=StatusBits.from_byte(words[5] & 0xFF),
        status2_raw=(words[5] >> 8) & 0xFF,
    )


@dataclass(frozen=True)
class DualMonitor:
    """PNT/MDTx 듀얼 monitor 해석 결과 (doc 05 §11).

    motor2가 없는 필드(`PID_PNT_MONITOR`의 current/output)는 `None`으로 둔다.
    """

    motor1: Monitor
    motor2: Monitor


def decode_pnt_monitor(words: list[int]) -> DualMonitor:
    """`PID_PNT_MONITOR(216)` 7-word 응답을 디코딩한다(doc 05 §6).

    구조: [speed1, pos1_low, pos1_high, speed2, pos2_low, pos2_high, status].
    current/output은 이 packet에 없어 `None`이다. 마지막 word는 wire상 status2(H)/status1(L).
    """
    if len(words) != 7:
        raise ValueError(f"PID_PNT_MONITOR expects 7 words, got {len(words)}")
    motor1 = Monitor(
        speed_rpm=int16(words[0]),
        current_a=None,
        output_raw=None,
        position=join_i32_low_word_first(words[1], words[2]),
        status=StatusBits.from_byte(words[6] & 0xFF),
    )
    motor2 = Monitor(
        speed_rpm=int16(words[3]),
        current_a=None,
        output_raw=None,
        position=join_i32_low_word_first(words[4], words[5]),
        status=StatusBits.from_byte((words[6] >> 8) & 0xFF),
    )
    return DualMonitor(motor1, motor2)


def decode_pnt_main_data(words: list[int]) -> DualMonitor:
    """`PID_PNT_MAIN_DATA(210)` 9-word 응답을 디코딩한다(doc 05 §5).

    구조: [speed1, cur1, pos1_low, pos1_high, speed2, cur2, pos2_low, pos2_high, status].
    current는 0.1A 단위. output은 이 packet에 없어 `None`이다.
    """
    if len(words) != 9:
        raise ValueError(f"PID_PNT_MAIN_DATA expects 9 words, got {len(words)}")
    motor1 = Monitor(
        speed_rpm=int16(words[0]),
        current_a=words[1] / 10.0,
        output_raw=None,
        position=join_i32_low_word_first(words[2], words[3]),
        status=StatusBits.from_byte(words[8] & 0xFF),
    )
    motor2 = Monitor(
        speed_rpm=int16(words[4]),
        current_a=words[5] / 10.0,
        output_raw=None,
        position=join_i32_low_word_first(words[6], words[7]),
        status=StatusBits.from_byte((words[8] >> 8) & 0xFF),
    )
    return DualMonitor(motor1, motor2)
