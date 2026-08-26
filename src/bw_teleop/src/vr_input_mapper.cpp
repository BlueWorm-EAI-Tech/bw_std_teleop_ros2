#include "bw_teleop/input/vr_input_mapper.hpp"

#include <algorithm>
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

  if (!algorithm::is_valid_pose(result.frame.head_pose) ||
    !algorithm::is_valid_pose(result.frame.left_pose) ||
    !algorithm::is_valid_pose(result.frame.right_pose))
  {
    result.message = "VR 输入包含非有限位姿";
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
