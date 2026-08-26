#include <cstdint>

#include <gtest/gtest.h>

#include "bw_std_control/async_serial_transport.hpp"

namespace bw_std_control
{
namespace
{

TEST(AsyncSerialTransport, ClosedTransportRejectsWrites)
{
  AsyncSerialTransport transport;
  EXPECT_FALSE(transport.is_open());
  EXPECT_FALSE(transport.async_write(V3CommandFrame{}));
  transport.close();
  EXPECT_FALSE(transport.is_open());
}

TEST(AsyncSerialTransport, InvalidDeviceThrowsWithoutLeakingOpenState)
{
  AsyncSerialTransport transport;
  EXPECT_THROW(
    transport.open(
      "/definitely/not/a/serial/device", 2000000U,
      [](const std::uint8_t *, std::size_t) {}, [](const std::string &) {}),
    std::exception);
  EXPECT_FALSE(transport.is_open());
}

}  // namespace
}  // namespace bw_std_control
