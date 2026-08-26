#ifndef BW_STD_CONTROL__FEEDBACK_HANDOFF_HPP_
#define BW_STD_CONTROL__FEEDBACK_HANDOFF_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>

#include "realtime_tools/realtime_buffer.hpp"

#include "bw_std_control/v3_protocol.hpp"

namespace bw_std_control
{

struct StampedFeedback
{
  V3FeedbackPayload feedback{};
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t sequence{0U};
  bool valid{false};
};

class FeedbackHandoff
{
public:
  void publish(const StampedFeedback & feedback);
  bool read_latest(StampedFeedback & feedback) noexcept;
  void reset() noexcept;
  std::uint64_t published_sequence() const noexcept;

private:
  realtime_tools::RealtimeBuffer<StampedFeedback> buffer_{};
  std::atomic_uint64_t published_sequence_{0U};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__FEEDBACK_HANDOFF_HPP_
