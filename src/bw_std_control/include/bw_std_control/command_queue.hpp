#ifndef BW_STD_CONTROL__COMMAND_QUEUE_HPP_
#define BW_STD_CONTROL__COMMAND_QUEUE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <mutex>

#include "bw_std_control/protocol.hpp"

namespace bw_std_control
{

class CommandQueue
{
public:
  static constexpr std::size_t capacity() noexcept {return kCapacity;}

  bool try_push(
    const CommandFrame & frame, std::uint64_t sequence = 0U,
    std::uint64_t session_generation = 0U) noexcept;
  bool try_pop(CommandFrame & frame) noexcept;
  bool try_pop(
    CommandFrame & frame, std::uint64_t & sequence,
    std::uint64_t & session_generation) noexcept;
  void clear_from_consumer() noexcept;
  bool is_lock_free() const noexcept;

private:
  struct QueuedCommand
  {
    CommandFrame frame{};
    std::uint64_t sequence{0U};
    std::uint64_t session_generation{0U};
  };

  static constexpr std::size_t kCapacity = 4U;
  static constexpr std::size_t kStorageSize = kCapacity + 1U;

  std::array<QueuedCommand, kStorageSize> storage_{};
  alignas(64) std::atomic_size_t head_{0U};
  alignas(64) std::atomic_size_t tail_{0U};
};

class CommandMailbox
{
public:
  CommandMailbox() noexcept;

  std::uint64_t try_push_normal(
    const CommandFrame & frame, std::uint64_t session_generation) noexcept;
  std::uint64_t try_push_power_off(
    const CommandFrame & frame, std::uint64_t session_generation) noexcept;
  bool try_pop_next(
    CommandFrame & frame, bool & is_power_off, std::uint64_t & sequence,
    std::uint64_t expected_session_generation) noexcept;
  void complete_normal_from_consumer(std::uint64_t sequence) noexcept;
  [[nodiscard]] bool sequence_completed(std::uint64_t sequence) const noexcept;
  void complete_power_off_from_consumer(
    std::uint64_t sequence, std::uint64_t session_generation) noexcept;
  bool wait_for_power_off(
    std::uint64_t sequence,
    std::chrono::steady_clock::duration timeout) noexcept;
  void clear_from_consumer() noexcept;
  bool is_lock_free() const noexcept;

private:
  static constexpr std::size_t kCompletionHistorySize = 64U;

  [[nodiscard]] std::uint64_t allocate_sequence() noexcept;
  void complete_sequence(std::uint64_t sequence) noexcept;

  CommandQueue normal_queue_{};
  CommandQueue power_off_queue_{};
  alignas(64) std::atomic_uint64_t next_sequence_{1U};
  alignas(64) std::array<std::atomic_uint64_t, kCompletionHistorySize>
  completed_sequences_{};
  alignas(64) std::atomic_uint64_t emergency_session_generation_{0U};
  std::mutex completion_mutex_;
  std::condition_variable completion_condition_;
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__COMMAND_QUEUE_HPP_
