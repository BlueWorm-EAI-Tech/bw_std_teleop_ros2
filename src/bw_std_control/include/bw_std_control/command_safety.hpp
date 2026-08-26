#ifndef BW_STD_CONTROL__COMMAND_SAFETY_HPP_
#define BW_STD_CONTROL__COMMAND_SAFETY_HPP_

#include <array>

#include "bw_std_control/standard_mapping.hpp"

namespace bw_std_control
{

struct CommandLimits
{
  std::array<double, kStandardJointCount> lower{};
  std::array<double, kStandardJointCount> upper{};
  std::array<double, kStandardJointCount> velocity{};
};

bool command_within_limits(
  const StandardCommand & command, const CommandLimits & limits) noexcept;
bool limit_command_step(
  StandardCommand & command, const StandardCommand & previous,
  const CommandLimits & limits, double elapsed_sec) noexcept;

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__COMMAND_SAFETY_HPP_
