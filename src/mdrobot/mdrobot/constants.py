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
