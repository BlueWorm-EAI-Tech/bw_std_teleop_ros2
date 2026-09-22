#include "bw_teleop/input/vr_input_mapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace bw_teleop::input
{
namespace
{

algorithm::Pose to_pose(const protocol::VrPose & pose) noexcept
{
  return algorithm::Pose{
    {pose.position[0], pose.position[1], pose.position[2]},
    {pose.orientation_xyzw[0], pose.orientation_xyzw[1],
      pose.orientation_xyzw[2], pose.orientation_xyzw[3]}};
}

}  // namespace

bool is_compatible_frame(
  const std::string & received_frame, const std::string & target_frame) noexcept
{
  return !target_frame.empty() &&
         (received_frame.empty() || received_frame == target_frame);
}

VrInputMapper::VrInputMapper(VrInputMapperConfig config)
: config_{std::move(config)}
{
  if (!std::isfinite(config_.joystick_deadzone) || config_.joystick_deadzone < 0.0 ||
    config_.joystick_deadzone >= 1.0)
  {
    throw std::invalid_argument("joystick_deadzone 必须位于 [0, 1)");
  }
}

VrInputMapResult VrInputMapper::map(const protocol::VrSample & sample) const
{
  VrInputMapResult result;
  if (!std::isfinite(sample.sender_timestamp)) {
    result.message = "VR 发送端时间戳非法";
    return result;
  }
  result.frame.sender_timestamp = sample.sender_timestamp;
  result.frame.head_pose = to_pose(sample.head_pose);
  result.frame.left_pose = to_pose(sample.left_pose);
  result.frame.right_pose = to_pose(sample.right_pose);
  result.frame.left_connected = sample.left_connected;
  result.frame.right_connected = sample.right_connected;

  JoyInput & left = result.frame.left_joy;
  left.x = apply_deadzone(sample.left_joystick_x);
  left.y = apply_deadzone(sample.left_joystick_y);
  left.trigger_value = std::clamp(sample.left_trigger_value, 0.0, 1.0);
  left.xa = sample.left_ax_button;
  left.yb = sample.left_by_button;
  left.grip = sample.left_grip;
  left.menu = sample.menu;
  left.trigger = sample.left_trigger;
  left.joystick = sample.left_joystick_clicked;

  JoyInput & right = result.frame.right_joy;
  right.x = apply_deadzone(sample.right_joystick_x);
  right.y = apply_deadzone(sample.right_joystick_y);
  right.trigger_value = std::clamp(sample.right_trigger_value, 0.0, 1.0);
  right.xa = sample.right_ax_button;
  right.yb = sample.right_by_button;
  right.grip = sample.right_grip;
  right.trigger = sample.right_trigger;
  right.joystick = sample.right_joystick_clicked;

  const std::array<double, 12> controls{
    sample.left_joystick_x, sample.left_joystick_y,
    sample.right_joystick_x, sample.right_joystick_y,
    sample.left_trigger_value, sample.right_trigger_value,
    sample.left_grip_value, sample.right_grip_value,
    result.frame.left_joy.x, result.frame.left_joy.y,
    result.frame.right_joy.x, result.frame.right_joy.y};
  if (!algorithm::is_valid_pose(result.frame.head_pose) ||
    (result.frame.left_connected && !algorithm::is_valid_pose(result.frame.left_pose)) ||
    (result.frame.right_connected && !algorithm::is_valid_pose(result.frame.right_pose)))
  {
    result.message = "VR 输入包含活动手柄或头显的非有限位姿";
    return result;
  }
  if (!std::all_of(controls.cbegin(), controls.cend(), [](const double value) {
      return std::isfinite(value);
    }) || sample.left_joystick_x < -1.0 || sample.left_joystick_x > 1.0 ||
    sample.left_joystick_y < -1.0 || sample.left_joystick_y > 1.0 ||
    sample.right_joystick_x < -1.0 || sample.right_joystick_x > 1.0 ||
    sample.right_joystick_y < -1.0 || sample.right_joystick_y > 1.0 ||
    sample.left_trigger_value < 0.0 || sample.left_trigger_value > 1.0 ||
    sample.right_trigger_value < 0.0 || sample.right_trigger_value > 1.0 ||
    sample.left_grip_value < 0.0 || sample.left_grip_value > 1.0 ||
    sample.right_grip_value < 0.0 || sample.right_grip_value > 1.0)
  {
    result.message = "VR 输入包含非法摇杆、trigger 或 grip 值";
    return result;
  }
  result.success = true;
  result.message = "VR 输入映射成功";
  return result;
}

double VrInputMapper::apply_deadzone(const double value) const noexcept
{
  const double bounded = std::clamp(value, -1.0, 1.0);
  const double magnitude = std::abs(bounded);
  if (magnitude <= config_.joystick_deadzone) {
    return 0.0;
  }
  const double scaled =
    (magnitude - config_.joystick_deadzone) / (1.0 - config_.joystick_deadzone);
  return std::copysign(scaled, bounded);
}

}  // namespace bw_teleop::input
