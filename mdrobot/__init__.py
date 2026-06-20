"""mdrobot: MDROBOT MD 시리즈 모터 컨트롤러용 RS485 / Modbus RTU 드라이버.

Phase 1(공통 프로토콜 코어)에서 제공하는 공개 심볼을 모은다.
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
from .units import counts_to_rad, rad_s_to_rpm, rad_to_counts, rpm_to_rad_s

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
    "active_bits",
    "STATUS1_BIT_NAMES",
    "STATUS2_BIT_NAMES",
    "DI_BIT_NAMES",
    "MdrobotError",
    "CrcError",
    "ProtocolError",
    "IncompleteResponseError",
]
