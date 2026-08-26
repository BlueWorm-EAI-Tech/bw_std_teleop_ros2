#include "bw_std_control/standard_mapping.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace bw_std_control
{

const std::array<std::string_view, kStandardJointCount> kStandardJointNames{
  "C_joint",
  "A_left_Degree1_joint", "A_left_Degree2_joint", "A_left_Degree3_joint",
  "A_left_Degree4_joint", "A_left_Degree5_joint", "A_left_Degree6_joint",
  "A_left_Degree7_joint",
  "A_right_Degree1_joint", "A_right_Degree2_joint", "A_right_Degree3_joint",
  "A_right_Degree4_joint", "A_right_Degree5_joint", "A_right_Degree6_joint",
  "A_right_Degree7_joint",
  "A_left_Degree8_joint", "A_right_Degree8_joint"};

const std::array<std::string_view, kBaseInterfaceCount> kBaseInterfaceNames{"vx", "vy", "wz"};

namespace
{

constexpr double kMillimetresPerMetre = 1000.0;

constexpr std::size_t index(const JointIndex joint) noexcept
{
  return static_cast<std::size_t>(joint);
}

template<typename ValuesT>
bool all_finite(const ValuesT & values) noexcept
{
  return std::all_of(
    values.begin(), values.end(), [](const auto value) {return std::isfinite(value);});
}

bool float_representable(const double value) noexcept
{
  return std::isfinite(value) &&
         std::abs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}

template<typename ValuesT>
bool all_float_representable(const ValuesT & values) noexcept
{
  return std::all_of(values.begin(), values.end(), float_representable);
}

bool to_finite_float(const double value, float & encoded) noexcept
{
  if (!float_representable(value)) {
    return false;
  }
  encoded = static_cast<float>(value);
  return std::isfinite(encoded);
}

bool valid_motor_indices(
  const std::array<std::size_t, kArmJointCount> & motor_indices) noexcept;
bool valid_directions(const std::array<double, kArmJointCount> & directions) noexcept;

bool valid_parameters(const MappingParameters & parameters) noexcept
{
  const std::array<double, 7> values{
    parameters.gripper_travel_m,
    parameters.pelvis_max_velocity_mm_s,
    parameters.arm_max_velocity_rad_s, parameters.gripper_max_velocity_normalized_s,
    parameters.base_max_acceleration_x, parameters.base_max_acceleration_y,
    parameters.base_max_acceleration_omega};
  return all_float_representable(values) &&
         valid_motor_indices(parameters.left_arm_motor_indices) &&
         valid_motor_indices(parameters.right_arm_motor_indices) &&
         valid_directions(parameters.left_arm_direction) &&
         valid_directions(parameters.right_arm_direction) &&
         all_float_representable(parameters.left_arm_raw_zero_rad) &&
         all_float_representable(parameters.right_arm_raw_zero_rad) &&
         parameters.gripper_travel_m > 0.0 &&
         parameters.pelvis_max_velocity_mm_s > 0.0 &&
         parameters.arm_max_velocity_rad_s > 0.0 &&
         parameters.gripper_max_velocity_normalized_s > 0.0 &&
         parameters.base_max_acceleration_x > 0.0 &&
         parameters.base_max_acceleration_y > 0.0 &&
         parameters.base_max_acceleration_omega > 0.0;
}

bool finite_command(const StandardCommand & command) noexcept
{
  return all_float_representable(command.position) &&
         all_float_representable(command.base_velocity);
}

bool finite_state(const StandardState & state) noexcept
{
  return all_finite(state.position) && all_finite(state.velocity) && all_finite(state.effort);
}

bool finite_head_hold(const HeadHold & head_hold) noexcept
{
  return std::isfinite(head_hold.waist_position) &&
         std::isfinite(head_hold.head_yaw_position) &&
         std::isfinite(head_hold.head_pitch_position);
}

bool valid_motor_indices(
  const std::array<std::size_t, kArmJointCount> & motor_indices) noexcept
{
  std::array<bool, kArmJointCount> seen{};
  for (const std::size_t motor_index : motor_indices) {
    if (motor_index >= kArmJointCount || seen[motor_index]) {
      return false;
    }
    seen[motor_index] = true;
  }
  return true;
}

bool valid_directions(const std::array<double, kArmJointCount> & directions) noexcept
{
  return std::all_of(
    directions.begin(), directions.end(),
    [](const double direction) {return direction == -1.0 || direction == 1.0;});
}

}  // namespace

bool try_extract_head_hold(
  const V3FeedbackPayload & feedback, HeadHold & head_hold) noexcept
{
  if (!std::isfinite(feedback.waist_position) || !std::isfinite(feedback.head_yaw_position) ||
    !std::isfinite(feedback.head_pitch_position))
  {
    return false;
  }
  head_hold = {
    feedback.waist_position, feedback.head_yaw_position, feedback.head_pitch_position};
  return true;
}

bool decode_complete_feedback(
  const V3FeedbackPayload & feedback, const MappingParameters & parameters,
  StandardState & state, HeadHold & head_hold) noexcept
{
  StandardState decoded{};
  HeadHold decoded_head{};
  if (!decode_standard_state(feedback, parameters, decoded) ||
    !try_extract_head_hold(feedback, decoded_head))
  {
    return false;
  }
  state = decoded;
  head_hold = decoded_head;
  return true;
}

bool decode_standard_state(
  const V3FeedbackPayload & feedback, const MappingParameters & parameters,
  StandardState & state) noexcept
{
  if (!valid_parameters(parameters)) {
    return false;
  }
  StandardState decoded{};
  decoded.position[index(JointIndex::lift)] =
    static_cast<double>(feedback.pelvis_height) / kMillimetresPerMetre;
  decoded.velocity[index(JointIndex::lift)] =
    static_cast<double>(feedback.pelvis_velocity) / kMillimetresPerMetre;

  for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
    const std::size_t left = index(JointIndex::left_degree1) + joint;
    const std::size_t right = index(JointIndex::right_degree1) + joint;
    const std::size_t left_motor = parameters.left_arm_motor_indices[joint];
    const std::size_t right_motor = parameters.right_arm_motor_indices[joint];
    decoded.position[left] =
      parameters.left_arm_direction[joint] *
      (feedback.left_joint_position[left_motor] - parameters.left_arm_raw_zero_rad[joint]);
    decoded.velocity[left] =
      parameters.left_arm_direction[joint] * feedback.left_joint_velocity[left_motor];
    decoded.effort[left] =
      parameters.left_arm_direction[joint] * feedback.left_joint_torque[left_motor];
    decoded.position[right] =
      parameters.right_arm_direction[joint] *
      (feedback.right_joint_position[right_motor] - parameters.right_arm_raw_zero_rad[joint]);
    decoded.velocity[right] =
      parameters.right_arm_direction[joint] * feedback.right_joint_velocity[right_motor];
    decoded.effort[right] =
      parameters.right_arm_direction[joint] * feedback.right_joint_torque[right_motor];
  }

  decoded.position[index(JointIndex::left_gripper)] =
    feedback.left_joint_position[7] * parameters.gripper_travel_m;
  decoded.velocity[index(JointIndex::left_gripper)] =
    feedback.left_joint_velocity[7] * parameters.gripper_travel_m;
  decoded.effort[index(JointIndex::left_gripper)] = feedback.left_joint_torque[7];
  decoded.position[index(JointIndex::right_gripper)] =
    feedback.right_joint_position[7] * parameters.gripper_travel_m;
  decoded.velocity[index(JointIndex::right_gripper)] =
    feedback.right_joint_velocity[7] * parameters.gripper_travel_m;
  decoded.effort[index(JointIndex::right_gripper)] = feedback.right_joint_torque[7];

  if (!finite_state(decoded)) {
    return false;
  }
  state = decoded;
  return true;
}

bool encode_standard_command(
  const StandardCommand & command, const MappingParameters & parameters,
  const HeadHold & head_hold, const bool power_enabled, V3CommandPayload & payload) noexcept
{
  if (!finite_command(command) || !valid_parameters(parameters) ||
    !finite_head_hold(head_hold))
  {
    return false;
  }

  V3CommandPayload encoded{};
  encoded.control_flag = power_enabled ? 0x01U : 0x00U;
  if (power_enabled) {
    if (!to_finite_float(command.base_velocity[0], encoded.vx) ||
      !to_finite_float(command.base_velocity[1], encoded.vy) ||
      !to_finite_float(command.base_velocity[2], encoded.omega) ||
      !to_finite_float(parameters.base_max_acceleration_x, encoded.max_acc_x) ||
      !to_finite_float(parameters.base_max_acceleration_y, encoded.max_acc_y) ||
      !to_finite_float(parameters.base_max_acceleration_omega, encoded.max_acc_omega))
    {
      return false;
    }
  }
  const double pelvis_height =
    command.position[index(JointIndex::lift)] * kMillimetresPerMetre;
  if (!to_finite_float(pelvis_height, encoded.pelvis_height) ||
    (power_enabled &&
    !to_finite_float(parameters.pelvis_max_velocity_mm_s, encoded.pelvis_velocity)))
  {
    return false;
  }

  float arm_max_velocity = 0.0F;
  float gripper_max_velocity = 0.0F;
  if (power_enabled &&
    (!to_finite_float(parameters.arm_max_velocity_rad_s, arm_max_velocity) ||
    !to_finite_float(
      parameters.gripper_max_velocity_normalized_s, gripper_max_velocity)))
  {
    return false;
  }

  for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
    const std::size_t left = index(JointIndex::left_degree1) + joint;
    const std::size_t right = index(JointIndex::right_degree1) + joint;
    const std::size_t left_motor = parameters.left_arm_motor_indices[joint];
    const std::size_t right_motor = parameters.right_arm_motor_indices[joint];
    if (!to_finite_float(
        parameters.left_arm_direction[joint] * command.position[left] +
        parameters.left_arm_raw_zero_rad[joint],
        encoded.left_joint_position[left_motor]) ||
      !to_finite_float(
        parameters.right_arm_direction[joint] * command.position[right] +
        parameters.right_arm_raw_zero_rad[joint],
        encoded.right_joint_position[right_motor]))
    {
      return false;
    }
    encoded.left_joint_max_velocity[left_motor] = arm_max_velocity;
    encoded.right_joint_max_velocity[right_motor] = arm_max_velocity;
  }

  const double left_gripper = std::clamp(
    command.position[index(JointIndex::left_gripper)] /
    parameters.gripper_travel_m, 0.0, 1.0);
  const double right_gripper = std::clamp(
    command.position[index(JointIndex::right_gripper)] /
    parameters.gripper_travel_m, 0.0, 1.0);
  if (!to_finite_float(left_gripper, encoded.left_joint_position[7]) ||
    !to_finite_float(right_gripper, encoded.right_joint_position[7]))
  {
    return false;
  }
  encoded.left_joint_max_velocity[7] = gripper_max_velocity;
  encoded.right_joint_max_velocity[7] = gripper_max_velocity;

  // 头部契约尚未稳定：只保持最近原始反馈位置，三个速度始终为零。
  encoded.waist_position = head_hold.waist_position;
  encoded.head_yaw_position = head_hold.head_yaw_position;
  encoded.head_pitch_position = head_hold.head_pitch_position;
  encoded.waist_max_velocity = 0.0F;
  encoded.head_yaw_max_velocity = 0.0F;
  encoded.head_pitch_max_velocity = 0.0F;
  if (!v3_command_payload_is_finite(encoded)) {
    return false;
  }
  payload = encoded;
  return true;
}

bool v3_command_payload_is_finite(const V3CommandPayload & payload) noexcept
{
  const std::array<float, 14> scalar_values{
    payload.vx, payload.vy, payload.omega,
    payload.max_acc_x, payload.max_acc_y, payload.max_acc_omega,
    payload.pelvis_height, payload.pelvis_velocity,
    payload.waist_position, payload.head_yaw_position, payload.head_pitch_position,
    payload.waist_max_velocity, payload.head_yaw_max_velocity,
    payload.head_pitch_max_velocity};
  return all_finite(scalar_values) &&
         all_finite(payload.left_joint_position) &&
         all_finite(payload.right_joint_position) &&
         all_finite(payload.left_joint_max_velocity) &&
         all_finite(payload.right_joint_max_velocity) &&
         all_finite(payload.left_joint_torque) &&
         all_finite(payload.right_joint_torque);
}

}  // namespace bw_std_control
