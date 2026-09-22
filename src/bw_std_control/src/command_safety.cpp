#include "bw_std_control/command_safety.hpp"

#include <algorithm>
#include <cmath>

namespace bw_std_control
{
namespace
{

CommandSafetyDiagnostic joint_diagnostic(
  const CommandSafetyFailure failure, const std::size_t joint,
  const double value, const double lower, const double upper) noexcept
{
  return CommandSafetyDiagnostic{failure, joint, value, lower, upper};
}

CommandSafetyDiagnostic check_base_velocity(const StandardCommand & command) noexcept
{
  for (std::size_t index = 0U; index < command.base_velocity.size(); ++index) {
    if (!std::isfinite(command.base_velocity[index])) {
      return CommandSafetyDiagnostic{
        CommandSafetyFailure::base_velocity, index,
        command.base_velocity[index], 0.0, 0.0};
    }
  }
  return {};
}

constexpr double kFloatLimitEpsilon = 1e-6;

CommandSafetyDiagnostic check_head_position(
  const StandardCommand & command, const CommandLimits & limits) noexcept
{
  for (std::size_t index = 0U; index < command.head_position.size(); ++index) {
    const double value = command.head_position[index];
    if (!std::isfinite(value) || !std::isfinite(limits.head_lower[index]) ||
      !std::isfinite(limits.head_upper[index]) ||
      limits.head_lower[index] > limits.head_upper[index] ||
      value < limits.head_lower[index] - kHeadBoundaryToleranceRad ||
      value > limits.head_upper[index] + kHeadBoundaryToleranceRad)
    {
      return joint_diagnostic(
        CommandSafetyFailure::head_position, index, value,
        limits.head_lower[index], limits.head_upper[index]);
    }
  }
  return {};
}

bool joint_configuration_is_valid(
  const double value, const double lower, const double upper) noexcept
{
  return std::isfinite(value) && std::isfinite(lower) && std::isfinite(upper) &&
         lower <= upper;
}

}  // namespace

bool CommandSafetyDiagnostic::ok() const noexcept
{
  return failure == CommandSafetyFailure::none;
}

CommandSafetyDiagnostic check_auxiliary_limits(
  const StandardCommand & command, const CommandLimits & limits) noexcept
{
  const CommandSafetyDiagnostic base_diagnostic = check_base_velocity(command);
  if (!base_diagnostic.ok()) {
    return base_diagnostic;
  }
  return check_head_position(command, limits);
}

CommandSafetyDiagnostic check_command_limits(
  const StandardCommand & command, const CommandLimits & limits) noexcept
{
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    const double value = command.position[joint];
    if (!joint_configuration_is_valid(value, limits.lower[joint], limits.upper[joint]) ||
      value < limits.lower[joint] || value > limits.upper[joint])
    {
      return joint_diagnostic(
        CommandSafetyFailure::joint_position, joint, value,
        limits.lower[joint], limits.upper[joint]);
    }
  }
  return check_auxiliary_limits(command, limits);
}

std::array<double, kStandardJointCount> make_standard_startup_limit_tolerance(
  const double arm_tolerance_rad, const double gripper_tolerance_m) noexcept
{
  std::array<double, kStandardJointCount> tolerance{};
  for (std::size_t joint = static_cast<std::size_t>(JointIndex::left_degree1);
    joint <= static_cast<std::size_t>(JointIndex::right_degree7); ++joint)
  {
    tolerance[joint] = arm_tolerance_rad;
  }
  tolerance[static_cast<std::size_t>(JointIndex::left_gripper)] = gripper_tolerance_m;
  tolerance[static_cast<std::size_t>(JointIndex::right_gripper)] = gripper_tolerance_m;
  return tolerance;
}

CommandSafetyDiagnostic StartupLimitRecovery::begin(
  const StandardCommand & measured, const CommandLimits & limits,
  const std::array<double, kStandardJointCount> & tolerance) noexcept
{
  reset();
  std::array<LimitRecoveryDirection, kStandardJointCount> planned{};
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    const double value = measured.position[joint];
    const double lower = limits.lower[joint];
    const double upper = limits.upper[joint];
    const double allowed = tolerance[joint];
    if (!std::isfinite(allowed) || allowed < 0.0) {
      return joint_diagnostic(
        CommandSafetyFailure::startup_tolerance, joint, allowed, 0.0, 0.0);
    }
    if (!joint_configuration_is_valid(value, lower, upper)) {
      return joint_diagnostic(
        CommandSafetyFailure::joint_position, joint, value, lower, upper);
    }
    if (value < lower) {
      if (value < lower - allowed) {
        return joint_diagnostic(
          CommandSafetyFailure::joint_position, joint, value,
          lower - allowed, upper + allowed);
      }
      planned[joint] = LimitRecoveryDirection::increasing;
    } else if (value > upper) {
      if (value > upper + allowed) {
        return joint_diagnostic(
          CommandSafetyFailure::joint_position, joint, value,
          lower - allowed, upper + allowed);
      }
      planned[joint] = LimitRecoveryDirection::decreasing;
    }
  }
  const CommandSafetyDiagnostic auxiliary_diagnostic = check_auxiliary_limits(measured, limits);
  if (!auxiliary_diagnostic.ok()) {
    return auxiliary_diagnostic;
  }
  directions_ = planned;
  tolerance_ = tolerance;
  return {};
}

CommandSafetyDiagnostic StartupLimitRecovery::validate(
  const StandardCommand & command, const StandardCommand & previous,
  const CommandLimits & limits) const noexcept
{
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    const double value = command.position[joint];
    const double prior = previous.position[joint];
    const double lower = limits.lower[joint];
    const double upper = limits.upper[joint];
    if (!joint_configuration_is_valid(value, lower, upper) || !std::isfinite(prior)) {
      return joint_diagnostic(
        CommandSafetyFailure::joint_position, joint, value, lower, upper);
    }

    double allowed_lower = lower;
    double allowed_upper = upper;
    if (directions_[joint] == LimitRecoveryDirection::none &&
      std::isfinite(tolerance_[joint]) && tolerance_[joint] >= 0.0)
    {
      allowed_lower = lower - tolerance_[joint];
      allowed_upper = upper + tolerance_[joint];
    }
    switch (directions_[joint]) {
      case LimitRecoveryDirection::decreasing:
        if (value < lower) {
          return joint_diagnostic(
            CommandSafetyFailure::joint_position, joint, value, lower, upper);
        }
        allowed_upper = prior;
        break;
      case LimitRecoveryDirection::increasing:
        if (value > upper) {
          return joint_diagnostic(
            CommandSafetyFailure::joint_position, joint, value, lower, upper);
        }
        allowed_lower = prior;
        break;
      case LimitRecoveryDirection::none:
        break;
    }
    if (value < allowed_lower || value > allowed_upper) {
      const CommandSafetyFailure failure =
        directions_[joint] == LimitRecoveryDirection::none ?
        CommandSafetyFailure::joint_position : CommandSafetyFailure::recovery_direction;
      return joint_diagnostic(failure, joint, value, allowed_lower, allowed_upper);
    }
  }
  return check_auxiliary_limits(command, limits);
}

void StartupLimitRecovery::note_command_queued(
  const StandardCommand & command, const StandardCommand & previous,
  const CommandLimits & limits, const std::uint64_t sequence) noexcept
{
  if (sequence == 0U) {
    return;
  }
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    if (entry_command_sequences_[joint] != 0U) {
      continue;
    }
    if (directions_[joint] == LimitRecoveryDirection::decreasing &&
      command.position[joint] < previous.position[joint])
    {
      if (command.position[joint] >= limits.lower[joint] &&
        command.position[joint] <= limits.upper[joint])
      {
        entry_command_sequences_[joint] = sequence;
        entry_commands_completed_[joint] = false;
      }
    } else if (directions_[joint] == LimitRecoveryDirection::increasing &&
      command.position[joint] > previous.position[joint])
    {
      if (command.position[joint] >= limits.lower[joint] &&
        command.position[joint] <= limits.upper[joint])
      {
        entry_command_sequences_[joint] = sequence;
        entry_commands_completed_[joint] = false;
      }
    }
  }
}

CommandSafetyDiagnostic StartupLimitRecovery::update_measured_state(
  const StandardState & measured, const CommandLimits & limits,
  const std::array<bool, kStandardJointCount> & entry_commands_completed,
  const std::uint64_t feedback_sequence) noexcept
{
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    const double value = measured.position[joint];
    const double lower = limits.lower[joint];
    const double upper = limits.upper[joint];
    if (!joint_configuration_is_valid(value, lower, upper)) {
      return joint_diagnostic(
        CommandSafetyFailure::joint_position, joint, value, lower, upper);
    }
    if (directions_[joint] == LimitRecoveryDirection::none) {
      const double tolerance = std::isfinite(tolerance_[joint]) && tolerance_[joint] >= 0.0 ?
        tolerance_[joint] : 0.0;
      if (value < lower - tolerance || value > upper + tolerance) {
        return joint_diagnostic(
          CommandSafetyFailure::joint_position, joint, value, lower, upper);
      }
      continue;
    }
    if (entry_command_sequences_[joint] == 0U) {
      continue;
    }
    if (!entry_commands_completed_[joint] && entry_commands_completed[joint] &&
      feedback_sequence != 0U)
    {
      entry_commands_completed_[joint] = true;
      entry_completion_feedback_sequences_[joint] = feedback_sequence;
    }
    if (!entry_commands_completed_[joint]) {
      continue;
    }
    if (feedback_sequence <= entry_completion_feedback_sequences_[joint]) {
      continue;
    }
    if (value >= lower && value <= upper) {
      directions_[joint] = LimitRecoveryDirection::none;
      entry_command_sequences_[joint] = 0U;
      entry_commands_completed_[joint] = false;
      entry_completion_feedback_sequences_[joint] = 0U;
    }
  }
  return {};
}

void StartupLimitRecovery::reset() noexcept
{
  directions_.fill(LimitRecoveryDirection::none);
  tolerance_.fill(0.0);
  entry_command_sequences_.fill(0U);
  entry_commands_completed_.fill(false);
  entry_completion_feedback_sequences_.fill(0U);
}

bool StartupLimitRecovery::has_active_recovery() const noexcept
{
  return std::any_of(
    directions_.begin(), directions_.end(),
    [](const LimitRecoveryDirection direction)
    {
      return direction != LimitRecoveryDirection::none;
    });
}

LimitRecoveryDirection StartupLimitRecovery::direction(const std::size_t joint) const noexcept
{
  return joint < directions_.size() ? directions_[joint] : LimitRecoveryDirection::none;
}

std::uint64_t StartupLimitRecovery::entry_command_sequence(const std::size_t joint) const noexcept
{
  return joint < entry_command_sequences_.size() ? entry_command_sequences_[joint] : 0U;
}

bool command_within_limits(
  const StandardCommand & command, const CommandLimits & limits,
  const double tolerance) noexcept
{
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    return false;
  }
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    const double value = command.position[joint];
    if (!std::isfinite(value) || !std::isfinite(limits.lower[joint]) ||
      !std::isfinite(limits.upper[joint]) || limits.lower[joint] > limits.upper[joint] ||
      value < limits.lower[joint] - tolerance || value > limits.upper[joint] + tolerance)
    {
      return false;
    }
  }
  if (!std::all_of(
      command.base_velocity.begin(), command.base_velocity.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return false;
  }
  const double head_tolerance =
    std::max(tolerance, std::max(kFloatLimitEpsilon, kHeadBoundaryToleranceRad));
  for (std::size_t index = 0U; index < kHeadInterfaceCount; ++index) {
    const double value = command.head_position[index];
    if (!std::isfinite(value) || !std::isfinite(limits.head_lower[index]) ||
      !std::isfinite(limits.head_upper[index]) || limits.head_lower[index] > limits.head_upper[index] ||
      value < limits.head_lower[index] - head_tolerance ||
      value > limits.head_upper[index] + head_tolerance)
    {
      return false;
    }
  }
  return true;
}

bool command_is_finite(const StandardCommand & command) noexcept
{
  return std::all_of(
           command.position.begin(), command.position.end(),
           [](const double value) {return std::isfinite(value);}) &&
         std::all_of(
           command.head_position.begin(), command.head_position.end(),
           [](const double value) {return std::isfinite(value);}) &&
         std::all_of(
           command.base_velocity.begin(), command.base_velocity.end(),
           [](const double value) {return std::isfinite(value);});
}

void clamp_command_to_limits(
  StandardCommand & command, const CommandLimits & limits) noexcept
{
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    if (!std::isfinite(limits.lower[joint]) || !std::isfinite(limits.upper[joint]) ||
      limits.lower[joint] > limits.upper[joint] || !std::isfinite(command.position[joint]))
    {
      continue;
    }
    command.position[joint] = std::clamp(
      command.position[joint], limits.lower[joint], limits.upper[joint]);
  }
  for (std::size_t index = 0U; index < kHeadInterfaceCount; ++index) {
    if (!std::isfinite(limits.head_lower[index]) || !std::isfinite(limits.head_upper[index]) ||
      limits.head_lower[index] > limits.head_upper[index] ||
      !std::isfinite(command.head_position[index]))
    {
      continue;
    }
    command.head_position[index] = std::clamp(
      command.head_position[index], limits.head_lower[index], limits.head_upper[index]);
  }
}

bool limit_command_step(
  StandardCommand & command, const StandardCommand & previous,
  const CommandLimits & limits, const double elapsed_sec) noexcept
{
  if (!std::isfinite(elapsed_sec) || elapsed_sec <= 0.0) {
    return false;
  }
  for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
    const double velocity = limits.velocity[joint];
    const double prior = previous.position[joint];
    if (!std::isfinite(velocity) || velocity <= 0.0 || !std::isfinite(prior) ||
      !std::isfinite(command.position[joint]))
    {
      return false;
    }
    const double max_delta = velocity * elapsed_sec;
    if (!std::isfinite(max_delta)) {
      return false;
    }
    command.position[joint] = std::clamp(
      command.position[joint], prior - max_delta, prior + max_delta);
  }
  for (std::size_t index = 0U; index < kHeadInterfaceCount; ++index) {
    const double velocity = limits.head_velocity[index];
    const double prior = previous.head_position[index];
    if (!std::isfinite(velocity) || velocity <= 0.0 || !std::isfinite(prior) ||
      !std::isfinite(command.head_position[index]))
    {
      return false;
    }
    const double max_delta = velocity * elapsed_sec;
    if (!std::isfinite(max_delta)) {
      return false;
    }
    command.head_position[index] = std::clamp(
      command.head_position[index], prior - max_delta, prior + max_delta);
  }
  return true;
}

}  // namespace bw_std_control
