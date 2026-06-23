// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include "mdrobot_cpp/transport.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <chrono>

// POSIX headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace mdrobot {

static speed_t to_speed(int baudrate) {
  switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:
      throw std::invalid_argument("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

SerialTransport::SerialTransport(const std::string& port, int baudrate,
                                 double timeout, double settle,
                                 double write_timeout)
    : port_(port), baudrate_(baudrate), write_timeout_(write_timeout) {
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open " + port + ": " + std::strerror(errno));
  }

  // Clear non-blocking after open.
  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tty {};
  if (::tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
  }

  // 8N1.
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Raw mode.
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;

  // Timeout: VMIN=0, VTIME in deciseconds.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(timeout * 10);

  speed_t spd = to_speed(baudrate);
  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
  }

  // Settle + flush (same as Python: USB-serial boot noise mitigation).
  if (settle > 0) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(settle * 1000)));
  }
  ::tcflush(fd_, TCIOFLUSH);
}

SerialTransport::~SerialTransport() {
  close();
}

std::size_t SerialTransport::write(const uint8_t* data, std::size_t len) {
  if (fd_ < 0) throw std::runtime_error("Port not open");
  // POSIX write() may transfer fewer bytes than requested — loop until the
  // whole frame is out (or write_timeout elapses), so frames are never split.
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(write_timeout_));
  std::size_t total = 0;
  while (total < len) {
    ssize_t n = ::write(fd_, data + total, len - total);
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (std::chrono::steady_clock::now() >= deadline) {
          throw std::runtime_error("write timeout");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      throw std::runtime_error("write failed: " + std::string(std::strerror(errno)));
    }
    total += static_cast<std::size_t>(n);
    if (total < len && std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("write timeout (partial)");
    }
  }
  ::tcdrain(fd_);  // wait for transmission to complete (like pyserial flush)
  return total;
}

std::vector<uint8_t> SerialTransport::read(std::size_t size) {
  if (fd_ < 0) throw std::runtime_error("Port not open");
  std::vector<uint8_t> buf(size);
  ssize_t n = ::read(fd_, buf.data(), size);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return {};
    throw std::runtime_error("read failed: " + std::string(std::strerror(errno)));
  }
  buf.resize(static_cast<std::size_t>(n));
  return buf;
}

void SerialTransport::flush_input() {
  if (fd_ >= 0) {
    ::tcflush(fd_, TCIFLUSH);
  }
}

void SerialTransport::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialTransport::is_open() const {
  return fd_ >= 0;
}

}  // namespace mdrobot
