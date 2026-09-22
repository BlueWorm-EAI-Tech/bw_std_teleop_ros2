#ifndef BW_STD_CONTROL__ASYNC_SERIAL_TRANSPORT_HPP_
#define BW_STD_CONTROL__ASYNC_SERIAL_TRANSPORT_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "bw_std_control/protocol.hpp"

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
  std::uint64_t async_write(const CommandFrame & frame) noexcept;
  [[nodiscard]] bool write_completed(std::uint64_t sequence) const noexcept;
  std::uint64_t async_write_power_off(const CommandFrame & frame) noexcept;
  bool wait_for_power_off(
    std::uint64_t sequence, std::chrono::milliseconds timeout) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__ASYNC_SERIAL_TRANSPORT_HPP_
