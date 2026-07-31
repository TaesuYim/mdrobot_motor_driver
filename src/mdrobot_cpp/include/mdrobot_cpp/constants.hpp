// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file constants.hpp
/// Modbus function codes and shared communication constants.

#pragma once

#include <cstdint>

namespace mdrobot {

// Modbus function codes — only these three are implemented.
constexpr uint8_t FUNC_READ           = 0x03;  // read holding registers
constexpr uint8_t FUNC_WRITE_SINGLE   = 0x06;  // write single register
constexpr uint8_t FUNC_WRITE_MULTIPLE = 0x10;  // write multiple registers

// Exception flag in response function byte.
constexpr uint8_t EXCEPTION_FLAG = 0x80;

// ID / write-check constants.
constexpr uint8_t ID_ALL           = 0xFE;
constexpr uint8_t ID_WRITE_CHK     = 0xAA;
constexpr uint8_t ID_DEFAULT_CHK   = 0x55;
constexpr uint8_t ID_DEVELOPER_CHK = 0x77;

// Communication defaults: 19200 8N1.
constexpr uint8_t  DEFAULT_SLAVE_ID = 1;
constexpr int      DEFAULT_BAUDRATE = 19200;
constexpr double   DEFAULT_TIMEOUT  = 0.3;  // serial read timeout (seconds)

// Modbus RTU frames are delimited by silence, not by a start byte: a receiver treats
// 3.5 character times of idle line as "frame over". Without that gap the tail of one
// response and the next request arrive as one continuous stream and the addressed
// slave discards both. The spec fixes the gap at 1.750 ms above 19200 baud and at
// 3.5 characters (11 bits each, counting the parity slot even for 8N1) at or below it.
constexpr double MODBUS_T3_5_FIXED    = 0.00175;  // seconds, above 19200 baud
constexpr int    MODBUS_BITS_PER_CHAR = 11;

/// Modbus RTU inter-frame silence (t3.5) in seconds for a baud rate.
///
/// Measured need: at 19200 with two controllers on one bus, back-to-back
/// transactions with no gap failed every time the addressed slave id changed; a 1 ms
/// gap removed it entirely. Slow USB adapters (FTDI `latency_timer` 16 ms) used to
/// hide this by accident.
constexpr double modbus_inter_frame_delay(int baudrate) {
  return baudrate > 19200
             ? MODBUS_T3_5_FIXED
             : 3.5 * MODBUS_BITS_PER_CHAR / static_cast<double>(baudrate);
}

}  // namespace mdrobot
