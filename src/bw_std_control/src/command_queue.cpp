#include "bw_std_control/command_queue.hpp"

#include <algorithm>
#include <limits>

namespace bw_std_control
{

bool CommandQueue::try_push(
  const CommandFrame & frame, const std::uint64_t sequence,
  const std::uint64_t session_generation) noexcept
{
  const std::size_t head = head_.load(std::memory_order_relaxed);
  const std::size_t next = (head + 1U) % kStorageSize;
  if (next == tail_.load(std::memory_order_acquire)) {
    return false;
  }
  storage_[head] = QueuedCommand{frame, sequence, session_generation};
  head_.store(next, std::memory_order_release);
  return true;
}

bool CommandQueue::try_pop(CommandFrame & frame) noexcept
{
  std::uint64_t sequence = 0U;
  std::uint64_t session_generation = 0U;
  return try_pop(frame, sequence, session_generation);
}

bool CommandQueue::try_pop(
  CommandFrame & frame, std::uint64_t & sequence,
  std::uint64_t & session_generation) noexcept
{
  const std::size_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire)) {
    return false;
  }
  frame = storage_[tail].frame;
  sequence = storage_[tail].sequence;
  session_generation = storage_[tail].session_generation;
  tail_.store((tail + 1U) % kStorageSize, std::memory_order_release);
  return true;
}

void CommandQueue::clear_from_consumer() noexcept
{
  CommandFrame discarded{};
  while (try_pop(discarded)) {
  }
}

bool CommandQueue::is_lock_free() const noexcept
{
  return head_.is_lock_free() && tail_.is_lock_free();
}

CommandMailbox::CommandMailbox() noexcept
{
  for (std::atomic_uint64_t & sequence : completed_sequences_) {
    sequence.store(0U, std::memory_order_relaxed);
  }
}

std::uint64_t CommandMailbox::allocate_sequence() noexcept
{
  std::uint64_t sequence = next_sequence_.load(std::memory_order_relaxed);
  while (sequence != std::numeric_limits<std::uint64_t>::max() &&
    !next_sequence_.compare_exchange_weak(
      sequence, sequence + 1U, std::memory_order_relaxed,
      std::memory_order_relaxed))
  {
  }
  return sequence == std::numeric_limits<std::uint64_t>::max() ? 0U : sequence;
}

std::uint64_t CommandMailbox::try_push_normal(
  const CommandFrame & frame, const std::uint64_t session_generation) noexcept
{
  if (session_generation == 0U ||
    emergency_session_generation_.load(std::memory_order_acquire) == session_generation)
  {
    return 0U;
  }

  const std::uint64_t sequence = allocate_sequence();
  if (sequence == 0U || !normal_queue_.try_push(frame, sequence, session_generation)) {
    return 0U;
  }
  return sequence;
}

std::uint64_t CommandMailbox::try_push_power_off(
  const CommandFrame & frame, const std::uint64_t session_generation) noexcept
{
  if (session_generation == 0U) {
    return 0U;
  }
  emergency_session_generation_.store(session_generation, std::memory_order_release);
  const std::uint64_t sequence = allocate_sequence();
  if (sequence != 0U && power_off_queue_.try_push(frame, sequence, session_generation)) {
    return sequence;
  }
  return 0U;
}

bool CommandMailbox::try_pop_next(
  CommandFrame & frame, bool & is_power_off, std::uint64_t & sequence,
  const std::uint64_t expected_session_generation) noexcept
{
  std::uint64_t queued_session_generation = 0U;
  while (power_off_queue_.try_pop(frame, sequence, queued_session_generation)) {
    if (queued_session_generation == expected_session_generation) {
      is_power_off = true;
      return true;
    }
  }
  if (emergency_session_generation_.load(std::memory_order_acquire) ==
    expected_session_generation)
  {
    return false;
  }
  is_power_off = false;
  while (normal_queue_.try_pop(frame, sequence, queued_session_generation)) {
    if (queued_session_generation == expected_session_generation) {
      return true;
    }
  }
  return false;
}

void CommandMailbox::complete_normal_from_consumer(
  const std::uint64_t sequence) noexcept
{
  complete_sequence(sequence);
}

void CommandMailbox::complete_sequence(const std::uint64_t sequence) noexcept
{
  if (sequence != 0U) {
    completed_sequences_[sequence % kCompletionHistorySize].store(
      sequence, std::memory_order_release);
  }
}

bool CommandMailbox::sequence_completed(const std::uint64_t sequence) const noexcept
{
  return sequence != 0U && completed_sequences_[sequence % kCompletionHistorySize].load(
    std::memory_order_acquire) == sequence;
}

void CommandMailbox::complete_power_off_from_consumer(
  const std::uint64_t sequence, const std::uint64_t session_generation) noexcept
{
  const std::lock_guard<std::mutex> lock{completion_mutex_};
  complete_sequence(sequence);
  normal_queue_.clear_from_consumer();
  CommandFrame equivalent{};
  std::uint64_t equivalent_sequence = 0U;
  std::uint64_t equivalent_session_generation = 0U;
  while (power_off_queue_.try_pop(
      equivalent, equivalent_sequence, equivalent_session_generation))
  {
    if (equivalent_session_generation == session_generation) {
      complete_sequence(equivalent_sequence);
    }
  }
  std::uint64_t expected = session_generation;
  static_cast<void>(emergency_session_generation_.compare_exchange_strong(
    expected, 0U, std::memory_order_release, std::memory_order_relaxed));
  completion_condition_.notify_all();
}

bool CommandMailbox::wait_for_power_off(
  const std::uint64_t sequence,
  const std::chrono::steady_clock::duration timeout) noexcept
{
  if (sequence == 0U) {
    return false;
  }
  std::unique_lock<std::mutex> lock{completion_mutex_};
  return completion_condition_.wait_for(
    lock, timeout,
    [this, sequence]()
    {
      return sequence_completed(sequence);
    });
}

void CommandMailbox::clear_from_consumer() noexcept
{
  const std::lock_guard<std::mutex> lock{completion_mutex_};
  power_off_queue_.clear_from_consumer();
  normal_queue_.clear_from_consumer();
  emergency_session_generation_.store(0U, std::memory_order_release);
  completion_condition_.notify_all();
}

bool CommandMailbox::is_lock_free() const noexcept
{
  const bool completion_history_is_lock_free = std::all_of(
    completed_sequences_.begin(), completed_sequences_.end(),
    [](const std::atomic_uint64_t & sequence) {return sequence.is_lock_free();});
  return normal_queue_.is_lock_free() && power_off_queue_.is_lock_free() &&
         next_sequence_.is_lock_free() && completion_history_is_lock_free &&
         emergency_session_generation_.is_lock_free();
}

}  // namespace bw_std_control
