#include "bw_teleop/manager/teleop_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace bw_teleop::manager
{
TeleopManager::TeleopManager(TeleopConfig config)
: config_(normalize_config(std::move(config))),
  pose_mapper_(make_pose_mapper_config(config_)),
  base_speed_gear_(config_.base_initial_gear)
{
  output_.left_target = pose_mapper_.target(algorithm::Side::left);
  output_.right_target = pose_mapper_.target(algorithm::Side::right);
  output_.gripper_command = {config_.gripper_min, config_.gripper_min};
  output_.lift_command = config_.lift_initial_position;
  output_.base_speed_gear = base_speed_gear_;
}

void TeleopManager::update_vr_frame(
  const input::MappedVrFrame & frame, const double received_sec) noexcept
{
  if (!std::isfinite(received_sec) || !algorithm::is_valid_pose(frame.head_pose)) {
    return;
  }
  head_pose_.value = frame.head_pose;
  head_pose_.received_sec = received_sec;
  head_pose_.received = true;

  const std::array<bool, 2> connected{frame.left_connected, frame.right_connected};
  const std::array<algorithm::Pose, 2> poses{frame.left_pose, frame.right_pose};
  const std::array<JoyInput, 2> joys{frame.left_joy, frame.right_joy};
  for (std::size_t hand_index = 0U; hand_index < connected.size(); ++hand_index) {
    if (!connected[hand_index] || !algorithm::is_valid_pose(poses[hand_index]) ||
      !valid_joy(joys[hand_index]))
    {
      continue;
    }
    hand_poses_[hand_index] = TimedPose{poses[hand_index], received_sec, true};
    joys_[hand_index] = TimedJoy{joys[hand_index], received_sec, true};
  }
}

void TeleopManager::update_measured_state(
  const MeasuredState & state, const double received_sec) noexcept
{
  if (!std::isfinite(received_sec)) {
    return;
  }
  if (state.left_gripper_valid && std::isfinite(state.left_gripper)) {
    const bool first_feedback = !measured_state_.left_gripper_valid;
    measured_state_.left_gripper = state.left_gripper;
    measured_state_.left_gripper_valid = true;
    measured_received_sec_[0] = received_sec;
    if (first_feedback) {
      output_.gripper_command[0] = std::clamp(
        state.left_gripper, config_.gripper_min, config_.gripper_max);
    }
  }
  if (state.right_gripper_valid && std::isfinite(state.right_gripper)) {
    const bool first_feedback = !measured_state_.right_gripper_valid;
    measured_state_.right_gripper = state.right_gripper;
    measured_state_.right_gripper_valid = true;
    measured_received_sec_[1] = received_sec;
    if (first_feedback) {
      output_.gripper_command[1] = std::clamp(
        state.right_gripper, config_.gripper_min, config_.gripper_max);
    }
  }
  if (state.lift_valid && std::isfinite(state.lift)) {
    const bool first_lift_feedback = !measured_state_.lift_valid;
    measured_state_.lift = state.lift;
    measured_state_.lift_valid = true;
    measured_received_sec_[2] = received_sec;
    if (first_lift_feedback) {
      output_.lift_command = std::clamp(state.lift, config_.lift_min, config_.lift_max);
    }
  }
}

void TeleopManager::update_measured_pose(
  const Side side, const Pose & pose, const double received_sec) noexcept
{
  if (!algorithm::is_valid_pose(pose) || !std::isfinite(received_sec)) {
    return;
  }
  measured_arm_poses_[index(side)] = TimedPose{pose, received_sec, true};
}

TickResult TeleopManager::tick(double now_sec) noexcept
{
  ManagerEvents events;
  std::array<bool, 2> target_updated{false, false};
  double dt_sec = 0.0;
  if (tick_initialized_ && std::isfinite(now_sec) && now_sec >= last_tick_sec_) {
    dt_sec = std::clamp(now_sec - last_tick_sec_, 0.0, config_.watchdog_timeout_sec);
  }
  tick_initialized_ = true;
  last_tick_sec_ = now_sec;

  const std::array<bool, 2> arm_feedback_fresh{
    is_fresh(
      now_sec, measured_arm_poses_[0].received_sec, config_.arm_pose_timeout_sec,
      measured_arm_poses_[0].received),
    is_fresh(
      now_sec, measured_arm_poses_[1].received_sec, config_.arm_pose_timeout_sec,
      measured_arm_poses_[1].received)};
  const bool auxiliary_feedback_fresh =
    is_fresh(
      now_sec, measured_received_sec_[0], config_.feedback_timeout_sec,
      measured_state_.left_gripper_valid) &&
    is_fresh(
      now_sec, measured_received_sec_[1], config_.feedback_timeout_sec,
      measured_state_.right_gripper_valid) &&
    is_fresh(
      now_sec, measured_received_sec_[2], config_.feedback_timeout_sec,
      measured_state_.lift_valid);
  if (auxiliary_feedback_fresh && !previous_auxiliary_feedback_fresh_) {
    auxiliary_hold_pending_ = true;
  }

  const bool fresh = all_inputs_fresh(now_sec);
  if (!fresh) {
    recovery_release_required_ = true;
    output_.safe_hold = true;
    output_.base_vx = 0.0;
    output_.base_vy = 0.0;
    output_.base_wz = 0.0;
    pose_mapper_.release_clutch(algorithm::Side::left);
    pose_mapper_.release_clutch(algorithm::Side::right);
    apply_measured_hold();
    clear_interaction_states();
  } else {
    const JoyInput & left_joy = joys_[index(algorithm::Side::left)].value;
    const JoyInput & right_joy = joys_[index(algorithm::Side::right)].value;
    if (recovery_release_required_ && (left_joy.grip || right_joy.grip)) {
      output_.safe_hold = true;
      output_.base_vx = 0.0;
      output_.base_vy = 0.0;
      output_.base_wz = 0.0;
      pose_mapper_.release_clutch(algorithm::Side::left);
      pose_mapper_.release_clutch(algorithm::Side::right);
      apply_measured_hold();
      clear_interaction_states();
    } else {
      if (recovery_release_required_) {
        recovery_release_required_ = false;
        clear_interaction_states();
      }
      output_.safe_hold = false;
      process_button_events(now_sec, events);
      target_updated = update_active_commands(
        dt_sec, arm_feedback_fresh,
        auxiliary_feedback_fresh && !auxiliary_hold_pending_);
    }
  }

  if (auxiliary_feedback_fresh && auxiliary_hold_pending_) {
    apply_measured_hold();
    auxiliary_hold_pending_ = false;
  }

  output_.left_target = pose_mapper_.target(algorithm::Side::left);
  output_.right_target = pose_mapper_.target(algorithm::Side::right);
  output_.left_target_valid = !output_.safe_hold && arm_feedback_fresh[0] &&
    (target_updated[0] || events.left_reset || events.both_reset);
  output_.right_target_valid = !output_.safe_hold && arm_feedback_fresh[1] &&
    (target_updated[1] || events.right_reset || events.both_reset);
  output_.auxiliary_commands_valid = auxiliary_feedback_fresh;
  output_.control_mode = pose_mapper_.control_mode();
  output_.base_speed_gear = base_speed_gear_;

  events.entered_safe_hold = !previous_safe_hold_ && output_.safe_hold;
  events.exited_safe_hold = previous_safe_hold_ && !output_.safe_hold;
  previous_safe_hold_ = output_.safe_hold;
  previous_auxiliary_feedback_fresh_ = auxiliary_feedback_fresh;
  return TickResult{output_, events};
}

TeleopConfig TeleopManager::normalize_config(TeleopConfig config)
{
  const std::array<double, 17> values{
    config.relative_position_scale, config.absolute_position_scale,
    config.watchdog_timeout_sec, config.feedback_timeout_sec,
    config.arm_pose_timeout_sec, config.long_press_sec, config.joystick_deadzone,
    config.gripper_min, config.gripper_max, config.base_max_forward,
    config.base_max_lateral, config.base_max_yaw, config.lift_min,
    config.lift_max, config.lift_initial_position, config.lift_jog_speed,
    static_cast<double>(config.base_initial_gear)};
  if (!std::all_of(values.cbegin(), values.cend(), [](const double value) {
      return std::isfinite(value);
    }))
  {
    throw std::invalid_argument("遥操作运动参数必须全部为有限值");
  }
  if (config.watchdog_timeout_sec <= 0.0 || config.feedback_timeout_sec <= 0.0 ||
    config.arm_pose_timeout_sec <= 0.0 || config.long_press_sec <= 0.0)
  {
    throw std::invalid_argument("遥操作超时和长按时长必须大于 0");
  }
  if (config.joystick_deadzone < 0.0 || config.joystick_deadzone >= 1.0 ||
    config.gripper_min > config.gripper_max || config.lift_min > config.lift_max ||
    config.lift_initial_position < config.lift_min ||
    config.lift_initial_position > config.lift_max ||
    config.base_max_forward < 0.0 || config.base_max_lateral < 0.0 ||
    config.base_max_yaw < 0.0 || config.lift_jog_speed < 0.0 ||
    config.base_initial_gear < 0 || config.base_initial_gear > 2)
  {
    throw std::invalid_argument("遥操作运动参数范围或顺序非法");
  }
  for (const double scale : config.base_speed_scales) {
    if (!std::isfinite(scale) || scale < 0.0 || scale > 1.0) {
      throw std::invalid_argument("底盘速度档比例必须为 [0, 1] 内有限值");
    }
  }
  return config;
}

algorithm::PoseMapperConfig TeleopManager::make_pose_mapper_config(
  const TeleopConfig & config) noexcept
{
  algorithm::PoseMapperConfig mapper_config;
  mapper_config.reset_poses = config.reset_poses;
  mapper_config.workspace = config.workspace;
  mapper_config.relative_position_scale = config.relative_position_scale;
  mapper_config.absolute_position_scale = config.absolute_position_scale;
  return mapper_config;
}

std::size_t TeleopManager::index(algorithm::Side side) noexcept
{
  return static_cast<std::size_t>(side);
}

bool TeleopManager::valid_joy(const JoyInput & joy) noexcept
{
  return std::isfinite(joy.x) && std::isfinite(joy.y) &&
         std::isfinite(joy.trigger_value);
}

bool TeleopManager::is_fresh(
  double now_sec, double received_sec, double timeout_sec, bool received) noexcept
{
  if (!received || !std::isfinite(now_sec) || !std::isfinite(received_sec)) {
    return false;
  }
  const double age_sec = now_sec - received_sec;
  return age_sec >= 0.0 && age_sec <= timeout_sec;
}

bool TeleopManager::all_inputs_fresh(double now_sec) const noexcept
{
  for (const TimedPose & hand_pose : hand_poses_) {
    if (!is_fresh(
        now_sec, hand_pose.received_sec, config_.watchdog_timeout_sec,
        hand_pose.received))
    {
      return false;
    }
  }
  for (const TimedJoy & joy : joys_) {
    if (!is_fresh(now_sec, joy.received_sec, config_.watchdog_timeout_sec, joy.received)) {
      return false;
    }
  }
  return is_fresh(
    now_sec, head_pose_.received_sec, config_.watchdog_timeout_sec, head_pose_.received);
}

bool TeleopManager::update_long_press(
  bool condition, double now_sec, LongPressState & state) noexcept
{
  if (!condition) {
    state = LongPressState{};
    return false;
  }
  if (!state.active) {
    state.active = true;
    state.started_sec = now_sec;
    return false;
  }
  if (!state.fired && now_sec - state.started_sec >= config_.long_press_sec) {
    state.fired = true;
    return true;
  }
  return false;
}

bool TeleopManager::rising_edge(bool current, bool & previous) noexcept
{
  const bool rising = current && !previous;
  previous = current;
  return rising;
}

double TeleopManager::apply_deadzone(double value) const noexcept
{
  const double clamped = std::clamp(value, -1.0, 1.0);
  return std::abs(clamped) < config_.joystick_deadzone ? 0.0 : clamped;
}

void TeleopManager::clear_interaction_states() noexcept
{
  mode_press_ = LongPressState{};
  left_reset_press_ = LongPressState{};
  right_reset_press_ = LongPressState{};
  both_reset_press_ = LongPressState{};
  previous_right_joystick_ = joys_[index(algorithm::Side::right)].value.joystick;
}

void TeleopManager::apply_measured_hold() noexcept
{
  if (measured_state_.left_gripper_valid) {
    output_.gripper_command[0] = std::clamp(
      measured_state_.left_gripper, config_.gripper_min, config_.gripper_max);
  }
  if (measured_state_.right_gripper_valid) {
    output_.gripper_command[1] = std::clamp(
      measured_state_.right_gripper, config_.gripper_min, config_.gripper_max);
  }
  if (measured_state_.lift_valid) {
    output_.lift_command = std::clamp(
      measured_state_.lift, config_.lift_min, config_.lift_max);
  }
}

void TeleopManager::process_button_events(double now_sec, ManagerEvents & events) noexcept
{
  const JoyInput & left = joys_[index(algorithm::Side::left)].value;
  const JoyInput & right = joys_[index(algorithm::Side::right)].value;
  const bool left_combo = left.xa && left.yb;
  const bool right_combo = right.xa && right.yb;
  const bool both_combo = left_combo && right_combo;

  if (both_combo) {
    left_reset_press_ = LongPressState{};
    right_reset_press_ = LongPressState{};
    if (update_long_press(true, now_sec, mode_press_)) {
      const algorithm::ControlMode next_mode =
        pose_mapper_.control_mode() == algorithm::ControlMode::relative ?
        algorithm::ControlMode::absolute : algorithm::ControlMode::relative;
      pose_mapper_.set_control_mode(next_mode);
      events.mode_changed = true;
    }
  } else {
    update_long_press(false, now_sec, mode_press_);
    if (update_long_press(left_combo, now_sec, left_reset_press_)) {
      pose_mapper_.reset(algorithm::Side::left);
      events.left_reset = true;
    }
    if (update_long_press(right_combo, now_sec, right_reset_press_)) {
      pose_mapper_.reset(algorithm::Side::right);
      events.right_reset = true;
    }
  }

  if (update_long_press(left.menu || right.menu, now_sec, both_reset_press_)) {
    pose_mapper_.reset(algorithm::Side::left);
    pose_mapper_.reset(algorithm::Side::right);
    events.both_reset = true;
  }

  if (rising_edge(right.joystick, previous_right_joystick_)) {
    base_speed_gear_ = (base_speed_gear_ + 1) % 3;
    events.speed_gear_changed = true;
  }
}

std::array<bool, 2> TeleopManager::update_active_commands(
  const double dt_sec, const std::array<bool, 2> & arm_feedback_fresh,
  const bool auxiliary_updates_enabled) noexcept
{
  const JoyInput & left = joys_[index(algorithm::Side::left)].value;
  const JoyInput & right = joys_[index(algorithm::Side::right)].value;

  std::array<bool, 2> target_updated{false, false};
  const std::array<JoyInput, 2> joy_values{left, right};
  for (std::size_t hand_index = 0U; hand_index < target_updated.size(); ++hand_index) {
    const Side side = static_cast<Side>(hand_index);
    if (!arm_feedback_fresh[hand_index]) {
      pose_mapper_.release_clutch(side);
      continue;
    }
    target_updated[hand_index] = pose_mapper_.update(
      side, hand_poses_[hand_index].value, joy_values[hand_index].grip,
      measured_arm_poses_[hand_index].value, true);
  }

  if (auxiliary_updates_enabled) {
    const double gripper_range = config_.gripper_max - config_.gripper_min;
    output_.gripper_command[0] = config_.gripper_min +
      std::clamp(left.trigger_value, 0.0, 1.0) * gripper_range;
    output_.gripper_command[1] = config_.gripper_min +
      std::clamp(right.trigger_value, 0.0, 1.0) * gripper_range;

    const double lift_delta =
      apply_deadzone(right.y) * config_.lift_jog_speed * std::max(0.0, dt_sec);
    output_.lift_command = std::clamp(
      output_.lift_command + lift_delta, config_.lift_min, config_.lift_max);
  }

  const double speed_scale = config_.base_speed_scales[base_speed_gear_];
  output_.base_vx = apply_deadzone(left.y) * config_.base_max_forward * speed_scale;
  output_.base_vy = -apply_deadzone(left.x) * config_.base_max_lateral * speed_scale;
  output_.base_wz = -apply_deadzone(right.x) * config_.base_max_yaw * speed_scale;

  return target_updated;
}

}  // namespace bw_teleop::manager
