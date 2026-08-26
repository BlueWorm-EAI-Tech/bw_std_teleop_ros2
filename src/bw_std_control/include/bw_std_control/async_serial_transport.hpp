#ifndef BW_STD_CONTROL__ASYNC_SERIAL_TRANSPORT_HPP_
#define BW_STD_CONTROL__ASYNC_SERIAL_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "bw_std_control/v3_protocol.hpp"

namespace bw_std_control
{

class AsyncSerialTransport
{
public:
  using DataCallback = std::function<void(const std::uint8_t *, std::size_t)>;
  using ErrorCallback = std::function<void(const std::string &)>;

  AsyncSerialTransport();
  ~AsyncSerialTransport();

  AsyncSerialTransport(const AsyncSerialTransport &) = delete;
  AsyncSerialTransport & operator=(const AsyncSerialTransport &) = delete;
  AsyncSerialTransport(AsyncSerialTransport &&) = delete;
  AsyncSerialTransport & operator=(AsyncSerialTransport &&) = delete;

  void open(
    const std::string & device, std::uint32_t baud_rate,
    DataCallback data_callback, ErrorCallback error_callback);
  void close() noexcept;
  bool is_open() const noexcept;
  bool async_write(const V3CommandFrame & frame) noexcept;
  bool async_write_power_off(const V3CommandFrame & frame) noexcept;
  bool wait_for_power_off(std::chrono::milliseconds timeout) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__ASYNC_SERIAL_TRANSPORT_HPP_
