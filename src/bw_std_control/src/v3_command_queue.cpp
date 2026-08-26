#include "bw_std_control/v3_command_queue.hpp"

namespace bw_std_control
{

bool V3CommandQueue::try_push(const V3CommandFrame & frame) noexcept
{
  const std::size_t head = head_.load(std::memory_order_relaxed);
  const std::size_t next = (head + 1U) % kStorageSize;
  if (next == tail_.load(std::memory_order_acquire)) {
    return false;
  }
  storage_[head] = frame;
  head_.store(next, std::memory_order_release);
  return true;
}

bool V3CommandQueue::try_pop(V3CommandFrame & frame) noexcept
{
  const std::size_t tail = tail_.load(std::memory_order_relaxed);
  if (tail == head_.load(std::memory_order_acquire)) {
    return false;
  }
  frame = storage_[tail];
  tail_.store((tail + 1U) % kStorageSize, std::memory_order_release);
  return true;
}

void V3CommandQueue::clear_from_consumer() noexcept
{
  V3CommandFrame discarded{};
  while (try_pop(discarded)) {
  }
}

bool V3CommandQueue::is_lock_free() const noexcept
{
  return head_.is_lock_free() && tail_.is_lock_free();
}

bool V3CommandMailbox::try_push_normal(const V3CommandFrame & frame) noexcept
{
  if (emergency_latched_.load(std::memory_order_acquire)) {
    return false;
  }
  return normal_queue_.try_push(frame);
}

bool V3CommandMailbox::try_push_power_off(const V3CommandFrame & frame) noexcept
{
  emergency_latched_.store(true, std::memory_order_release);
  if (power_off_queue_.try_push(frame)) {
    return true;
  }
  // 该队列只保存软件掉电帧；已满表示至少一个等价安全请求正在等待发送。
  return true;
}

bool V3CommandMailbox::try_pop_next(
  V3CommandFrame & frame, bool & is_power_off) noexcept
{
  if (emergency_latched_.load(std::memory_order_acquire)) {
    is_power_off = power_off_queue_.try_pop(frame);
    return is_power_off;
  }
  is_power_off = false;
  return normal_queue_.try_pop(frame);
}

void V3CommandMailbox::complete_power_off_from_consumer() noexcept
{
  const std::lock_guard<std::mutex> lock{completion_mutex_};
  normal_queue_.clear_from_consumer();
  power_off_queue_.clear_from_consumer();
  emergency_latched_.store(false, std::memory_order_release);
  completion_condition_.notify_all();
}

bool V3CommandMailbox::wait_for_power_off(
  const std::chrono::steady_clock::duration timeout) noexcept
{
  std::unique_lock<std::mutex> lock{completion_mutex_};
  return completion_condition_.wait_for(
    lock, timeout,
    [this]() {return !emergency_latched_.load(std::memory_order_acquire);});
}

void V3CommandMailbox::clear_from_consumer() noexcept
{
  const std::lock_guard<std::mutex> lock{completion_mutex_};
  power_off_queue_.clear_from_consumer();
  normal_queue_.clear_from_consumer();
  emergency_latched_.store(false, std::memory_order_release);
  completion_condition_.notify_all();
}

bool V3CommandMailbox::is_lock_free() const noexcept
{
  return normal_queue_.is_lock_free() && power_off_queue_.is_lock_free() &&
         emergency_latched_.is_lock_free();
}

}  // namespace bw_std_control
