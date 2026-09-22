#include "bw_std_control/arm_power_on_sequence.hpp"

#include <algorithm>
#include <cmath>

namespace bw_std_control
{

bool arm_feedback_ready(const FeedbackPayload & feedback) noexcept
{
  return std::all_of(
           feedback.left_joint_position.begin(), feedback.left_joint_position.end(),
           [](const float value) {return std::isfinite(value);}) &&
         std::all_of(
           feedback.right_joint_position.begin(), feedback.right_joint_position.end(),
           [](const float value) {return std::isfinite(value);});
}

ArmPowerFrameDecision decide_arm_power_frame(
  const bool power_requested, const ArmPowerStage stage) noexcept
{
  if (!power_requested) {
    return {};
  }
  switch (stage) {
    case ArmPowerStage::enabled:
      return {ArmPowerFrameKind::enabled, kActiveControlFlag};
    case ArmPowerStage::awaiting_ready_feedback:
    case ArmPowerStage::ready_for_precharge:
    case ArmPowerStage::awaiting_precharge_completion:
    default:
      return {ArmPowerFrameKind::precharge, kArmsDisabledControlFlag};
  }
}

ArmPowerOnSequence::ArmPowerOnSequence(
  const std::size_t required_ready_frames,
  const std::chrono::steady_clock::duration precharge_duration) noexcept
: required_ready_frames_(required_ready_frames == 0U ? 1U : required_ready_frames),
  precharge_duration_(std::max(
    precharge_duration, std::chrono::steady_clock::duration::zero()))
{
}

void ArmPowerOnSequence::reset() noexcept
{
  consecutive_ready_frames_ = 0U;
  stage_ = ArmPowerStage::awaiting_ready_feedback;
  latest_feedback_ready_ = false;
  pending_precharge_sequence_ = 0U;
  precharge_started_at_ = {};
}

ArmPowerStage ArmPowerOnSequence::stage() const noexcept
{
  return stage_;
}

std::size_t ArmPowerOnSequence::required_ready_frames() const noexcept
{
  return required_ready_frames_;
}

std::size_t ArmPowerOnSequence::consecutive_ready_frames() const noexcept
{
  return consecutive_ready_frames_;
}

std::uint64_t ArmPowerOnSequence::pending_precharge_sequence() const noexcept
{
  return pending_precharge_sequence_;
}

bool ArmPowerOnSequence::latest_feedback_ready() const noexcept
{
  return latest_feedback_ready_;
}

bool ArmPowerOnSequence::observe_feedback(const FeedbackPayload & feedback) noexcept
{
  latest_feedback_ready_ = arm_feedback_ready(feedback);
  if (stage_ == ArmPowerStage::enabled) {
    return latest_feedback_ready_;
  }
  if (!latest_feedback_ready_) {
    consecutive_ready_frames_ = 0U;
    stage_ = ArmPowerStage::awaiting_ready_feedback;
    pending_precharge_sequence_ = 0U;
    precharge_started_at_ = {};
    return false;
  }
  if (consecutive_ready_frames_ < required_ready_frames_) {
    ++consecutive_ready_frames_;
  }
  if (consecutive_ready_frames_ < required_ready_frames_) {
    return false;
  }
  if (stage_ == ArmPowerStage::awaiting_ready_feedback) {
    stage_ = ArmPowerStage::ready_for_precharge;
  }
  return true;
}

bool ArmPowerOnSequence::note_precharge_queued(
  const std::uint64_t sequence, const std::chrono::steady_clock::time_point now) noexcept
{
  if (sequence == 0U) {
    return false;
  }
  if (stage_ == ArmPowerStage::ready_for_precharge) {
    pending_precharge_sequence_ = sequence;
    precharge_started_at_ = now;
    stage_ = ArmPowerStage::awaiting_precharge_completion;
    return true;
  }
  if (stage_ == ArmPowerStage::awaiting_precharge_completion) {
    pending_precharge_sequence_ = sequence;
    return true;
  }
  return false;
}

bool ArmPowerOnSequence::precharge_completed(
  const std::chrono::steady_clock::time_point now,
  const bool precharge_write_completed) const noexcept
{
  return stage_ == ArmPowerStage::awaiting_precharge_completion &&
         latest_feedback_ready_ && pending_precharge_sequence_ != 0U &&
         precharge_write_completed &&
         now - precharge_started_at_ >= precharge_duration_;
}

void ArmPowerOnSequence::mark_enabled() noexcept
{
  if (stage_ == ArmPowerStage::awaiting_precharge_completion) {
    stage_ = ArmPowerStage::enabled;
  }
}

}  // namespace bw_std_control
