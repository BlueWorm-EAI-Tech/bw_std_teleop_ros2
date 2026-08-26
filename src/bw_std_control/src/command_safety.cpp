#include "bw_std_control/command_safety.hpp"

#include <algorithm>
#include <cmath>

namespace bw_std_control
{

bool command_within_limits(
  const StandardCommand & command, const CommandLimits & limits) noexcept
{
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    const double value = command.position[joint];
    if (!std::isfinite(value) || !std::isfinite(limits.lower[joint]) ||
      !std::isfinite(limits.upper[joint]) || limits.lower[joint] > limits.upper[joint] ||
      value < limits.lower[joint] || value > limits.upper[joint])
    {
      return false;
    }
  }
  return std::all_of(
    command.base_velocity.begin(), command.base_velocity.end(),
    [](const double value) {return std::isfinite(value);});
}

bool limit_command_step(
  StandardCommand & command, const StandardCommand & previous,
  const CommandLimits & limits, const double elapsed_sec) noexcept
{
  if (!std::isfinite(elapsed_sec) || elapsed_sec <= 0.0) {
    return false;
  }
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
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
  return true;
}

}  // namespace bw_std_control
