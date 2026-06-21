"""Status-bit / monitor decoding layer.

Pure-function layer; no hardware needed, verified by unit tests. Bit meanings
follow the protocol documentation and are confirmed against `PID_DI(48)` /
`PID_CTRL_STATUS(34)` hardware logs.

Byte-order note: word 5 of `PID_MONITOR` is `D6H D6L` on the wire, so
`status2 = word >> 8` and `status1 = word & 0xFF`.
"""

from __future__ import annotations

from dataclasses import dataclass

from .codec import int16, join_i32_low_word_first

# --- status1 bits: PID_CTRL_STATUS(34) and PID_MONITOR status1 ----------------------
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

# --- status2 bits: PID_MONITOR status2 ----------------------------------------------
STATUS2_BIT_NAMES = {
    0: "REGEN_OVER_TEMP",
    1: "ENC_FAIL",
}

# --- PID_DI(48) digital input bits --------------------------------------------------
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
    """Return the names of the set bits in value, per the names map (ascending bit order)."""
    return [name for bit, name in sorted(names.items()) if value & (1 << bit)]


@dataclass(frozen=True)
class StatusBits:
    """Decoded status1 byte (8 bits)."""

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
        """Names of the set status1 bits."""
        return active_bits(self.raw, STATUS1_BIT_NAMES)


@dataclass(frozen=True)
class Monitor:
    """Decoded `PID_MONITOR(196)` single-channel monitor."""

    speed_rpm: int
    current_a: float | None
    output_raw: int | None
    position: int
    status: StatusBits
    status2_raw: int = 0


def decode_monitor(words: list[int]) -> Monitor:
    """Decode a 6-word `PID_MONITOR(196)` response into a Monitor.

    current is in 0.1 A units; position is an INT32 long (low word first). The
    signedness of current (INT16/UINT16) is treated as unsigned 0.1 A pending
    hardware confirmation.
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
    """Decoded dual-channel monitor.

    Fields absent for motor 2 (current/output in `PID_PNT_MONITOR`) are left None.
    """

    motor1: Monitor
    motor2: Monitor


def decode_pnt_monitor(words: list[int]) -> DualMonitor:
    """Decode a 7-word `PID_PNT_MONITOR(216)` response.

    Layout: [speed1, pos1_low, pos1_high, speed2, pos2_low, pos2_high, status].
    current/output are not in this packet (None). The last word is status2(H)/status1(L).
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
    """Decode a 9-word `PID_PNT_MAIN_DATA(210)` response.

    Layout: [speed1, cur1, pos1_low, pos1_high, speed2, cur2, pos2_low, pos2_high, status].
    current is in 0.1 A units; output is not in this packet (None).
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
