"""Modbus function codes and shared communication constants.

The PID/CMD tables live in registers.py. This module holds only the protocol
constants used by the frame/transport layers.
"""

from __future__ import annotations

# Modbus function codes — only these three are implemented.
FUNC_READ = 0x03            # read holding registers
FUNC_WRITE_SINGLE = 0x06    # write single register
FUNC_WRITE_MULTIPLE = 0x10  # write multiple registers

# This bit set in the response function marks a Modbus exception response.
EXCEPTION_FLAG = 0x80

# ID / write-check constants.
ID_ALL = 0xFE            # all/broadcast ID. Not used in the initial implementation.
ID_WRITE_CHK = 0xAA      # write-check value for some settings (ID/baudrate, ...).
ID_DEFAULT_CHK = 0x55    # default-setting check value.
ID_DEVELOPER_CHK = 0x77  # developer check value. Purpose unclear.

# Communication defaults. Default link settings: 19200 8N1.
DEFAULT_SLAVE_ID = 1
DEFAULT_BAUDRATE = 19200
DEFAULT_TIMEOUT = 0.3          # serial read timeout (s); enough for a max 23-byte frame.

# Modbus RTU frames are delimited by silence, not by a start byte: a receiver treats
# 3.5 character times of idle line as "frame over". Without that gap the tail of one
# response and the next request arrive as one continuous stream and the addressed
# slave discards both. The spec fixes the gap at 1.750 ms above 19200 baud and at
# 3.5 characters (11 bits each, counting the parity slot even for 8N1) at or below it.
MODBUS_T3_5_FIXED = 0.00175  # s; spec-mandated value above 19200 baud
MODBUS_BITS_PER_CHAR = 11


def modbus_inter_frame_delay(baudrate: int) -> float:
    """Return the Modbus RTU inter-frame silence (t3.5) in seconds for a baud rate.

    Measured need: at 19200 with two controllers on one bus, sending back-to-back
    transactions with no gap failed every time the addressed slave id changed; a 1 ms
    gap removed it entirely. Slow USB adapters (FTDI `latency_timer` 16 ms) used to
    hide this by accident.
    """
    if baudrate > 19200:
        return MODBUS_T3_5_FIXED
    return 3.5 * MODBUS_BITS_PER_CHAR / baudrate
