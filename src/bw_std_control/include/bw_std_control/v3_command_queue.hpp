#ifndef BW_STD_CONTROL__V3_COMMAND_QUEUE_HPP_
#define BW_STD_CONTROL__V3_COMMAND_QUEUE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <mutex>

#include "bw_std_control/v3_protocol.hpp"

namespace bw_std_control
{

class V3CommandQueue
{
public:
  static constexpr std::size_t capacity() noexcept {return kCapacity;}

  bool try_push(const V3CommandFrame & frame) noexcept;
  bool try_pop(V3CommandFrame & frame) noexcept;
  void clear_from_consumer() noexcept;
  bool is_lock_free() const noexcept;

private:
  static constexpr std::size_t kCapacity = 4U;
  static constexpr std::size_t kStorageSize = kCapacity + 1U;

  std::array<V3CommandFrame, kStorageSize> storage_{};
  alignas(64) std::atomic_size_t head_{0U};
  alignas(64) std::atomic_size_t tail_{0U};
};

class V3CommandMailbox
{
public:
  bool try_push_normal(const V3CommandFrame & frame) noexcept;
  bool try_push_power_off(const V3CommandFrame & frame) noexcept;
  bool try_pop_next(V3CommandFrame & frame, bool & is_power_off) noexcept;
  void complete_power_off_from_consumer() noexcept;
  bool wait_for_power_off(std::chrono::steady_clock::duration timeout) noexcept;
  void clear_from_consumer() noexcept;
  bool is_lock_free() const noexcept;

private:
  V3CommandQueue normal_queue_{};
  V3CommandQueue power_off_queue_{};
  alignas(64) std::atomic_bool emergency_latched_{false};
  std::mutex completion_mutex_;
  std::condition_variable completion_condition_;
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__V3_COMMAND_QUEUE_HPP_
