// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file transport.hpp
/// Transport interface and POSIX termios serial implementation.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mdrobot {

/// Environment variable consulted when no port is given explicitly (see resolve_port).
inline constexpr const char* kPortEnvVar = "MDROBOT_PORT";

/// Return the serial port to use: the explicit argument, else $MDROBOT_PORT.
///
/// An explicit non-empty @p port always wins. With @p port empty, the
/// MDROBOT_PORT environment variable is used, so programs can omit the port
/// entirely (`export MDROBOT_PORT=/dev/ttyUSB0` once). Throws
/// std::invalid_argument with a how-to-fix message when neither is set.
std::string resolve_port(const std::string& port = "");

/// Abstract transport interface — the protocol layer depends on this.
class Transport {
 public:
  virtual ~Transport() = default;

  /// Send all of data and return the number of bytes written.
  virtual std::size_t write(const uint8_t* data, std::size_t len) = 0;

  /// Read up to @p size bytes; may return fewer.
  virtual std::vector<uint8_t> read(std::size_t size) = 0;

  /// Discard any bytes left in the input buffer.
  virtual void flush_input() = 0;
};

/// POSIX termios serial transport for RS485 / Modbus RTU.
///
/// The transport also owns the Modbus RTU **inter-frame silence** (t3.5): it holds
/// off each outgoing frame until the line has been idle long enough. This state
/// belongs here, not in ModbusClient, because several clients (one per slave id)
/// share one transport — the gap has to be enforced across the whole bus, and the
/// id-to-id transition is exactly where the missing gap breaks framing.
class SerialTransport : public Transport {
 public:
  /// Open a serial port. Throws on failure.
  ///
  /// @param inter_frame_delay Modbus t3.5 silence in seconds; negative (the
  ///        default) derives it from @p baudrate, 0 disables it.
  SerialTransport(const std::string& port, int baudrate = 19200,
                  double timeout = 0.3, double settle = 0.2,
                  double write_timeout = 1.0,
                  double inter_frame_delay = -1.0);

  ~SerialTransport() override;

  // Non-copyable.
  SerialTransport(const SerialTransport&) = delete;
  SerialTransport& operator=(const SerialTransport&) = delete;

  std::size_t write(const uint8_t* data, std::size_t len) override;
  std::vector<uint8_t> read(std::size_t size) override;
  void flush_input() override;

  void close();
  bool is_open() const;

  const std::string& port() const { return port_; }
  int baudrate() const { return baudrate_; }
  double inter_frame_delay() const { return inter_frame_delay_; }

 private:
  /// Block until the line has been idle for inter_frame_delay_.
  void await_inter_frame();
  void mark_activity();

  std::string port_;
  int baudrate_;
  double write_timeout_;
  double inter_frame_delay_;
  std::chrono::steady_clock::time_point last_activity_{};
  int fd_ = -1;
};

}  // namespace mdrobot
