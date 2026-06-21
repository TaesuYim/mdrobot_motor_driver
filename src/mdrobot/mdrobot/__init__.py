"""mdrobot: RS485 / Modbus RTU driver for MDROBOT MD-series motor controllers.

Public symbols of the communication library: CRC, framing/codec, the Modbus
protocol client, registers, status decoding, unit conversion, and the
single-/dual-channel motor driver classes.
"""

from __future__ import annotations

from .codec import (
    int16,
    join_i32_low_word_first,
    join_u32_low_word_first,
    split_i32_low_word_first,
    split_u32_low_word_first,
    u16,
    word_from_int16,
)
from .crc import append_crc, check_crc, crc16_modbus
from .device import DualMotorDriver, SingleMotorDriver
from .exceptions import (
    CrcError,
    IncompleteResponseError,
    MdrobotError,
    ProtocolError,
)
from .protocol import ModbusClient
from .status import (
    DI_BIT_NAMES,
    STATUS1_BIT_NAMES,
    STATUS2_BIT_NAMES,
    DualMonitor,
    Monitor,
    StatusBits,
    active_bits,
    decode_monitor,
    decode_pnt_main_data,
    decode_pnt_monitor,
)
from .transport import SerialTransport, Transport
from .units import (
    SLOW_DEFAULT_FULL_SCALE_S,
    counts_to_rad,
    rad_s_to_rpm,
    rad_to_counts,
    rpm_to_rad_s,
    slow_raw_to_seconds,
    slow_seconds_to_raw,
)

__all__ = [
    "crc16_modbus",
    "append_crc",
    "check_crc",
    "u16",
    "int16",
    "word_from_int16",
    "split_u32_low_word_first",
    "split_i32_low_word_first",
    "join_u32_low_word_first",
    "join_i32_low_word_first",
    "ModbusClient",
    "Transport",
    "SerialTransport",
    "SingleMotorDriver",
    "DualMotorDriver",
    "StatusBits",
    "Monitor",
    "DualMonitor",
    "decode_monitor",
    "decode_pnt_monitor",
    "decode_pnt_main_data",
    "counts_to_rad",
    "rad_to_counts",
    "rpm_to_rad_s",
    "rad_s_to_rpm",
    "slow_seconds_to_raw",
    "slow_raw_to_seconds",
    "SLOW_DEFAULT_FULL_SCALE_S",
    "active_bits",
    "STATUS1_BIT_NAMES",
    "STATUS2_BIT_NAMES",
    "DI_BIT_NAMES",
    "MdrobotError",
    "CrcError",
    "ProtocolError",
    "IncompleteResponseError",
]
