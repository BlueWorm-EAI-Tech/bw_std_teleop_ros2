#ifndef BW_STD_CONTROL__HARDWARE_CONTRACT_HPP_
#define BW_STD_CONTROL__HARDWARE_CONTRACT_HPP_

#include <array>
#include <string_view>
#include <vector>

#include "bw_std_control/standard_mapping.hpp"

namespace bw_std_control
{

bool has_exact_standard_joint_names(
  const std::vector<std::string_view> & names) noexcept;
bool parse_arm_raw_zero_parameter(
  std::string_view text, std::array<double, kArmJointCount> & values) noexcept;
bool parse_arm_motor_indices_parameter(
  std::string_view text,
  std::array<std::size_t, kArmJointCount> & values) noexcept;
bool parse_arm_direction_parameter(
  std::string_view text, std::array<double, kArmJointCount> & values) noexcept;
bool parse_bounded_nonnegative_parameter(
  std::string_view text, double maximum, double & value) noexcept;
bool arm_mapping_enables_software_power(
  bool power_on_requested, bool arm_mapping_calibrated) noexcept;

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__HARDWARE_CONTRACT_HPP_
