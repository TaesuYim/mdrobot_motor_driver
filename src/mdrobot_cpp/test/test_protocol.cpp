// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
#include <gtest/gtest.h>

#include "mdrobot_cpp/constants.hpp"
#include "mdrobot_cpp/crc.hpp"
#include "mdrobot_cpp/exceptions.hpp"
#include "mdrobot_cpp/frame.hpp"
#include "mdrobot_cpp/protocol.hpp"

using namespace mdrobot;

static std::vector<uint8_t> with_crc(const std::vector<uint8_t>& body) {
  return append_crc(body.data(), body.size());
}

/// Fake transport that serves reads sequentially from a buffer and records writes.
class FakeTransport : public Transport {
 public:
  explicit FakeTransport(const std::vector<uint8_t>& response = {})
      : rx_(response) {}

  std::size_t write(const uint8_t* data, std::size_t len) override {
    written.insert(written.end(), data, data + len);
    return len;
  }

  std::vector<uint8_t> read(std::size_t size) override {
    std::size_t n = std::min(size, rx_.size());
    std::vector<uint8_t> chunk(rx_.begin(), rx_.begin() + n);
    rx_.erase(rx_.begin(), rx_.begin() + n);
    return chunk;
  }

  void flush_input() override { ++flush_count; }

  std::vector<uint8_t> written;
  int flush_count = 0;

 private:
  std::vector<uint8_t> rx_;
};

TEST(Protocol, ReadRegistersSendsRequestAndParses) {
  auto response = with_crc({0x01, 0x03, 0x02, 0x00, 0x0D});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  auto words = client.read_registers(1, 1);
  ASSERT_EQ(words.size(), 1u);
  EXPECT_EQ(words[0], 0x000D);
  EXPECT_EQ(transport.written, build_read_request(1, 1, 1));
  EXPECT_EQ(transport.flush_count, 1);
}

TEST(Protocol, ReadRegisterConvenience) {
  auto response = with_crc({0x01, 0x03, 0x02, 0x12, 0x34});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  EXPECT_EQ(client.read_register(196), 0x1234);
}

TEST(Protocol, WriteRegisterEchoOk) {
  auto request = build_write_single_request(1, 130, 100);
  FakeTransport transport(request);  // echo
  ModbusClient client(transport, 1);
  client.write_register(130, 100);
  EXPECT_EQ(transport.written, request);
}

TEST(Protocol, WriteRegistersEchoOk) {
  auto response = with_crc({0x01, 0x10, 0x00, 0xDB, 0x00, 0x03});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  client.write_registers(219, {0x5678, 0x1234, 0x9ABC});
  EXPECT_EQ(transport.written, build_write_multiple_request(1, 219, {0x5678, 0x1234, 0x9ABC}));
}

TEST(Protocol, ExceptionResponseRaisesProtocolError) {
  auto response = with_crc({1, 0x83, 0x02});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  EXPECT_THROW({
    try { client.read_registers(1, 1); }
    catch (const ProtocolError& e) {
      EXPECT_EQ(e.code(), 0x02);
      throw;
    }
  }, ProtocolError);
}

TEST(Protocol, CrcErrorRaises) {
  auto bad = with_crc({0x01, 0x03, 0x02, 0x00, 0x0D});
  bad.back() ^= 0xFF;
  FakeTransport transport(bad);
  ModbusClient client(transport, 1);
  EXPECT_THROW(client.read_registers(1, 1), CrcError);
}

TEST(Protocol, ShortResponseRaisesIncomplete) {
  FakeTransport transport({0x01});
  ModbusClient client(transport, 1);
  EXPECT_THROW(client.read_registers(1, 1), IncompleteResponseError);
}

TEST(Protocol, ReadLongSigned) {
  auto response = with_crc({0x01, 0x03, 0x04, 0x56, 0x78, 0x12, 0x34});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  EXPECT_EQ(client.read_long(197), 0x12345678);
}

TEST(Protocol, ReadLongNegative) {
  auto response = with_crc({0x01, 0x03, 0x04, 0xFF, 0xFE, 0xFF, 0xFF});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  EXPECT_EQ(client.read_long(197), -2);
}

TEST(Protocol, ReadLongUnsigned) {
  auto response = with_crc({0x01, 0x03, 0x04, 0xFF, 0xFE, 0xFF, 0xFF});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  EXPECT_EQ(static_cast<uint32_t>(client.read_long(197, false)), 0xFFFFFFFEu);
}

TEST(Protocol, WriteLongSplitsLowWordFirst) {
  auto response = with_crc({0x01, 0x10, 0x00, 0xC5, 0x00, 0x02});
  FakeTransport transport(response);
  ModbusClient client(transport, 1);
  client.write_long(197, 0x12345678);
  EXPECT_EQ(transport.written, build_write_multiple_request(1, 197, {0x5678, 0x1234}));
}

// --- Modbus RTU inter-frame silence (t3.5) ------------------------------------------
//
// Mirrors mdrobot/constants.py::modbus_inter_frame_delay. SerialTransport owns the
// gap (not ModbusClient) because several clients share one bus in twin mode.

TEST(InterFrameDelay, ScalesWithBaudAtOrBelow19200) {
  EXPECT_DOUBLE_EQ(modbus_inter_frame_delay(19200), 3.5 * 11 / 19200.0);
  EXPECT_DOUBLE_EQ(modbus_inter_frame_delay(9600), 3.5 * 11 / 9600.0);
  EXPECT_GT(modbus_inter_frame_delay(9600), modbus_inter_frame_delay(19200));
}

TEST(InterFrameDelay, FixedAbove19200) {
  EXPECT_DOUBLE_EQ(modbus_inter_frame_delay(38400), MODBUS_T3_5_FIXED);
  EXPECT_DOUBLE_EQ(modbus_inter_frame_delay(115200), MODBUS_T3_5_FIXED);
}

TEST(InterFrameDelay, IsConstexpr) {
  static_assert(modbus_inter_frame_delay(115200) == MODBUS_T3_5_FIXED,
                "must be usable in a constant expression");
  SUCCEED();
}
