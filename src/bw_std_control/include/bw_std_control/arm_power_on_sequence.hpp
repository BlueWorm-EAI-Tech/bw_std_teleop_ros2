#ifndef BW_STD_CONTROL__ARM_POWER_ON_SEQUENCE_HPP_
#define BW_STD_CONTROL__ARM_POWER_ON_SEQUENCE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>

#include "bw_std_control/protocol.hpp"

namespace bw_std_control
{

// 等待臂反馈就绪与等待预充电完成的上限, 超时即放弃本次使能。
constexpr auto kArmEnableReadyTimeout = std::chrono::milliseconds{3000};
constexpr auto kArmEnablePrechargeTimeout = std::chrono::milliseconds{500};
constexpr std::size_t kDefaultArmReadyFrames = 5U;
constexpr auto kDefaultArmPrechargeDuration = std::chrono::milliseconds{10};

// 就绪判据只看位置字段有限性: 未上电时右臂状态掩码为 0x00 但位置仍可信, 且系统上电后的
// 短窗口内位置会短暂归零; 因此不能先等状态掩码就绪再上电。
[[nodiscard]] bool arm_feedback_ready(const FeedbackPayload & feedback) noexcept;

enum class ArmPowerStage : std::uint8_t
{
  awaiting_ready_feedback,
  ready_for_precharge,
  awaiting_precharge_completion,
  enabled
};

enum class ArmPowerFrameKind : std::uint8_t
{
  power_off,
  precharge,
  enabled
};

struct ArmPowerFrameDecision
{
  ArmPowerFrameKind kind{ArmPowerFrameKind::power_off};
  std::uint8_t control_flag{0U};
};

// 按电源请求与使能阶段选择本周期发送的控制字。未就绪时仍发送关闭双臂位的系统上电帧
// (0x25): 固件需要收到系统上电帧后才会上报完整臂反馈, 但双臂使能位保持关闭直到反馈就绪。
[[nodiscard]] ArmPowerFrameDecision decide_arm_power_frame(
  bool power_requested, ArmPowerStage stage) noexcept;

class ArmPowerOnSequence final
{
public:
  explicit ArmPowerOnSequence(
    std::size_t required_ready_frames = kDefaultArmReadyFrames,
    std::chrono::steady_clock::duration precharge_duration =
    kDefaultArmPrechargeDuration) noexcept;

  void reset() noexcept;
  [[nodiscard]] ArmPowerStage stage() const noexcept;
  [[nodiscard]] std::size_t required_ready_frames() const noexcept;
  [[nodiscard]] std::size_t consecutive_ready_frames() const noexcept;
  [[nodiscard]] std::uint64_t pending_precharge_sequence() const noexcept;
  [[nodiscard]] bool latest_feedback_ready() const noexcept;

  // 返回 true 表示连续就绪帧数达到门禁(或已处于使能阶段)。
  bool observe_feedback(const FeedbackPayload & feedback) noexcept;
  bool note_precharge_queued(
    std::uint64_t sequence, std::chrono::steady_clock::time_point now) noexcept;
  [[nodiscard]] bool precharge_completed(
    std::chrono::steady_clock::time_point now,
    bool precharge_write_completed) const noexcept;
  void mark_enabled() noexcept;

private:
  std::size_t required_ready_frames_{kDefaultArmReadyFrames};
  std::chrono::steady_clock::duration precharge_duration_{kDefaultArmPrechargeDuration};
  std::size_t consecutive_ready_frames_{0U};
  ArmPowerStage stage_{ArmPowerStage::awaiting_ready_feedback};
  bool latest_feedback_ready_{false};
  std::uint64_t pending_precharge_sequence_{0U};
  std::chrono::steady_clock::time_point precharge_started_at_{};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__ARM_POWER_ON_SEQUENCE_HPP_
