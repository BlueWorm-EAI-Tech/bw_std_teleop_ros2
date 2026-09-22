#ifndef BW_STD_CONTROL__STANDARD_MAPPING_HPP_
#define BW_STD_CONTROL__STANDARD_MAPPING_HPP_

#include <array>
#include <cstddef>
#include <string_view>

#include "bw_std_control/protocol.hpp"

namespace bw_std_control
{

constexpr std::size_t kStandardJointCount = 17U;
constexpr std::size_t kBaseInterfaceCount = 3U;
constexpr std::size_t kHeadInterfaceCount = 3U;
constexpr std::size_t kArmJointCount = 7U;

constexpr double kHeadPitchMin = -0.524;
constexpr double kHeadPitchMax = 0.785;
constexpr double kHeadYawMin = -1.570;
constexpr double kHeadYawMax = 1.570;
constexpr double kHeadRollMin = -0.349;
constexpr double kHeadRollMax = 0.349;

enum class JointIndex : std::size_t
{
  lift = 0,
  left_degree1,
  left_degree2,
  left_degree3,
  left_degree4,
  left_degree5,
  left_degree6,
  left_degree7,
  right_degree1,
  right_degree2,
  right_degree3,
  right_degree4,
  right_degree5,
  right_degree6,
  right_degree7,
  left_gripper,
  right_gripper
};

enum class HeadIndex : std::size_t
{
  pitch = 0,
  yaw,
  roll
};

extern const std::array<std::string_view, kStandardJointCount> kStandardJointNames;
extern const std::array<std::string_view, kBaseInterfaceCount> kBaseInterfaceNames;
extern const std::array<std::string_view, kHeadInterfaceCount> kHeadInterfaceNames;

struct MappingParameters
{
  std::array<std::size_t, kArmJointCount> left_arm_motor_indices{0U, 1U, 2U, 3U, 4U, 5U, 6U};
  std::array<std::size_t, kArmJointCount> right_arm_motor_indices{0U, 1U, 2U, 3U, 4U, 5U, 6U};
  std::array<double, kArmJointCount> left_arm_direction{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, kArmJointCount> right_arm_direction{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, kArmJointCount> left_arm_raw_zero_rad{};
  std::array<double, kArmJointCount> right_arm_raw_zero_rad{};
  double gripper_travel_m{0.04965};
  double pelvis_max_velocity_mm_s{200.0};
  double arm_max_velocity_rad_s{1.0};
  double gripper_max_velocity_normalized_s{1.0};
  double head_max_velocity_rad_s{1.0};
  double base_max_acceleration_x{1.5};
  double base_max_acceleration_y{1.5};
  double base_max_acceleration_omega{1.5};
};

struct StandardState
{
  std::array<double, kStandardJointCount> position{};
  std::array<double, kStandardJointCount> velocity{};
  std::array<double, kStandardJointCount> effort{};
};

struct StandardCommand
{
  std::array<double, kStandardJointCount> position{};
  std::array<double, kBaseInterfaceCount> base_velocity{};
  std::array<double, kHeadInterfaceCount> head_position{};
};

struct HeadHold
{
  float waist_position{0.0F};
  float head_yaw_position{0.0F};
  float head_pitch_position{0.0F};
  float head_roll_position{0.0F};
};

bool try_extract_head_hold(
  const FeedbackPayload & feedback, HeadHold & head_hold) noexcept;
bool decode_standard_state(
  const FeedbackPayload & feedback, const MappingParameters & parameters,
  StandardState & state) noexcept;
bool decode_complete_feedback(
  const FeedbackPayload & feedback, const MappingParameters & parameters,
  StandardState & state, HeadHold & head_hold) noexcept;
bool encode_standard_command(
  const StandardCommand & command, const MappingParameters & parameters,
  const HeadHold & head_hold, bool power_enabled, CommandPayload & payload,
  const std::array<double, kStandardJointCount> * joint_velocities = nullptr) noexcept;
bool command_payload_is_finite(const CommandPayload & payload) noexcept;

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__STANDARD_MAPPING_HPP_
