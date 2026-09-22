#include "bw_std_control/hardware_contract.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <string>
#include <system_error>

namespace bw_std_control
{

bool has_exact_standard_joint_names(
  const std::vector<std::string_view> & names) noexcept
{
  if (names.size() != kStandardJointNames.size()) {
    return false;
  }
  return std::all_of(
    kStandardJointNames.begin(), kStandardJointNames.end(),
    [&names](const std::string_view expected) {
      return std::count(names.begin(), names.end(), expected) == 1;
    });
}

bool parse_arm_raw_zero_parameter(
  const std::string_view text, std::array<double, kArmJointCount> & values) noexcept
{
  std::array<double, kArmJointCount> parsed{};
  std::size_t begin = 0U;
  try {
    for (std::size_t index = 0U; index < parsed.size(); ++index) {
      const std::size_t delimiter = text.find(',', begin);
      const bool is_last = index + 1U == parsed.size();
      if ((is_last && delimiter != std::string_view::npos) ||
        (!is_last && delimiter == std::string_view::npos))
      {
        return false;
      }

      const std::size_t end = is_last ? text.size() : delimiter;
      const std::string token{text.substr(begin, end - begin)};
      if (token.empty()) {
        return false;
      }
      std::size_t consumed = 0U;
      const double value = std::stod(token, &consumed);
      if (consumed != token.size() || !std::isfinite(value)) {
        return false;
      }
      parsed[index] = value;
      begin = end + 1U;
    }
  } catch (const std::exception &) {
    return false;
  }
  values = parsed;
  return true;
}

bool parse_arm_motor_indices_parameter(
  const std::string_view text,
  std::array<std::size_t, kArmJointCount> & values) noexcept
{
  std::array<std::size_t, kArmJointCount> parsed{};
  std::array<bool, kArmJointCount> seen{};
  std::size_t begin = 0U;
  for (std::size_t joint = 0U; joint < parsed.size(); ++joint) {
    const std::size_t delimiter = text.find(',', begin);
    const bool is_last = joint + 1U == parsed.size();
    if ((is_last && delimiter != std::string_view::npos) ||
      (!is_last && delimiter == std::string_view::npos))
    {
      return false;
    }

    const std::size_t end = is_last ? text.size() : delimiter;
    const std::string_view token = text.substr(begin, end - begin);
    std::size_t motor_index = 0U;
    const auto result = std::from_chars(
      token.data(), token.data() + token.size(), motor_index);
    if (token.empty() || result.ec != std::errc{} ||
      result.ptr != token.data() + token.size() ||
      motor_index >= kArmJointCount || seen[motor_index])
    {
      return false;
    }
    parsed[joint] = motor_index;
    seen[motor_index] = true;
    begin = end + 1U;
  }
  values = parsed;
  return true;
}

bool parse_arm_direction_parameter(
  const std::string_view text, std::array<double, kArmJointCount> & values) noexcept
{
  std::array<double, kArmJointCount> parsed{};
  if (!parse_arm_raw_zero_parameter(text, parsed) ||
    !std::all_of(
      parsed.begin(), parsed.end(),
      [](const double direction) {return direction == -1.0 || direction == 1.0;}))
  {
    return false;
  }
  values = parsed;
  return true;
}

bool parse_bounded_nonnegative_parameter(
  const std::string_view text, const double maximum, double & value) noexcept
{
  if (!std::isfinite(maximum) || maximum < 0.0 || text.empty()) {
    return false;
  }
  try {
    const std::string token{text};
    std::size_t consumed = 0U;
    const double parsed = std::stod(token, &consumed);
    if (consumed != token.size() || !std::isfinite(parsed) ||
      parsed < 0.0 || parsed > maximum)
    {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool arm_mapping_enables_software_power(
  const bool power_on_requested, const bool arm_mapping_calibrated) noexcept
{
  return power_on_requested && arm_mapping_calibrated;
}

}  // namespace bw_std_control
