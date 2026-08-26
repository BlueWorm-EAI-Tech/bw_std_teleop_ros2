#include "bw_std_control/feedback_handoff.hpp"

namespace bw_std_control
{

void FeedbackHandoff::publish(const StampedFeedback & feedback)
{
  StampedFeedback published{feedback};
  published.sequence = published_sequence_.load(std::memory_order_relaxed) + 1U;
  buffer_.writeFromNonRT(published);
  published_sequence_.store(published.sequence, std::memory_order_release);
}

bool FeedbackHandoff::read_latest(StampedFeedback & feedback) noexcept
{
  const StampedFeedback * const latest = buffer_.readFromRT();
  if (latest == nullptr || !latest->valid) {
    return false;
  }
  feedback = *latest;
  return true;
}

void FeedbackHandoff::reset() noexcept
{
  buffer_.initRT(StampedFeedback{});
  published_sequence_.store(0U, std::memory_order_release);
}

std::uint64_t FeedbackHandoff::published_sequence() const noexcept
{
  return published_sequence_.load(std::memory_order_acquire);
}

}  // namespace bw_std_control
