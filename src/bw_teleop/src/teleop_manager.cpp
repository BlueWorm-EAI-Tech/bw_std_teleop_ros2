#include "bw_teleop/manager/teleop_manager.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace bw_teleop::manager
{
namespace
{

constexpr double kPiHalf{1.57079632679489661923};
constexpr double kQuaternionEpsilonSquared{1.0e-24};

}  // namespace

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
  if (!std::isfinite(received_sec) || !std::isfinite(frame.sender_timestamp)) {
    clear_all_vr_input();
    return;
  }

  std::array<double, 3> head_pitch_yaw_roll{};
  if (!algorithm::is_valid_pose(frame.head_pose) ||
    !pose_to_head_euler(frame.head_pose, head_pitch_yaw_roll))
  {
    clear_all_vr_input();
    return;
  }
  if (head_pose_.received && received_sec > head_pose_.received_sec &&
    received_sec - head_pose_.received_sec > config_.watchdog_timeout_sec)
  {
    head_reanchor_required_ = true;
  }
  head_pose_.value = frame.head_pose;
  head_pose_.received_sec = received_sec;
  head_pose_.received = true;
  update_head_angles(head_pitch_yaw_roll);

  const std::array<bool, 2> connected{frame.left_connected, frame.right_connected};
  const std::array<algorithm::Pose, 2> poses{frame.left_pose, frame.right_pose};
  const std::array<JoyInput, 2> joys{frame.left_joy, frame.right_joy};
  for (std::size_t hand_index = 0U; hand_index < connected.size(); ++hand_index) {
    hand_connected_[hand_index] = connected[hand_index];
    if (!connected[hand_index] || !algorithm::is_valid_pose(poses[hand_index]) ||
      !valid_joy(joys[hand_index]))
    {
      clear_hand_input(hand_index);
      continue;
    }
    hand_poses_[hand_index] = TimedPose{poses[hand_index], received_sec, true};
    joys_[hand_index] = TimedJoy{joys[hand_index], received_sec, true};
  }
}

void TeleopManager::clear_vr_input() noexcept
{
  clear_all_vr_input();
}

void TeleopManager::update_measured_state(
  const MeasuredState & state, const double received_sec) noexcept
{
  if (!std::isfinite(received_sec)) {
    invalidate_measured_state();
    return;
  }
  if (state.left_gripper_valid && std::isfinite(state.left_gripper) &&
    state.left_gripper >= config_.gripper_min && state.left_gripper <= config_.gripper_max) {
    const bool first_feedback = !measured_state_.left_gripper_valid;
    measured_state_.left_gripper = state.left_gripper;
    measured_state_.left_gripper_valid = true;
    measured_received_sec_[0] = received_sec;
    if (first_feedback) {
      output_.gripper_command[0] = std::clamp(
        state.left_gripper, config_.gripper_min, config_.gripper_max);
    }
  } else if (state.left_gripper_valid) {
    invalidate_measured_gripper(Side::left);
  }
  if (state.right_gripper_valid && std::isfinite(state.right_gripper) &&
    state.right_gripper >= config_.gripper_min && state.right_gripper <= config_.gripper_max) {
    const bool first_feedback = !measured_state_.right_gripper_valid;
    measured_state_.right_gripper = state.right_gripper;
    measured_state_.right_gripper_valid = true;
    measured_received_sec_[1] = received_sec;
    if (first_feedback) {
      output_.gripper_command[1] = std::clamp(
        state.right_gripper, config_.gripper_min, config_.gripper_max);
    }
  } else if (state.right_gripper_valid) {
    invalidate_measured_gripper(Side::right);
  }
  if (state.lift_valid && std::isfinite(state.lift) &&
    state.lift >= config_.lift_min && state.lift <= config_.lift_max) {
    const bool first_lift_feedback = !measured_state_.lift_valid;
    measured_state_.lift = state.lift;
    measured_state_.lift_valid = true;
    measured_received_sec_[2] = received_sec;
    if (first_lift_feedback) {
      output_.lift_command = std::clamp(state.lift, config_.lift_min, config_.lift_max);
    }
  } else if (state.lift_valid) {
    invalidate_measured_lift();
  }
}

void TeleopManager::invalidate_measured_state() noexcept
{
  measured_state_.left_gripper_valid = false;
  measured_state_.right_gripper_valid = false;
  measured_state_.lift_valid = false;
  measured_received_sec_ = {0.0, 0.0, 0.0};
}

void TeleopManager::invalidate_measured_gripper(const Side side) noexcept
{
  if (side == Side::left) {
    measured_state_.left_gripper_valid = false;
    measured_received_sec_[0] = 0.0;
  } else if (side == Side::right) {
    measured_state_.right_gripper_valid = false;
    measured_received_sec_[1] = 0.0;
  }
}

void TeleopManager::invalidate_measured_lift() noexcept
{
  measured_state_.lift_valid = false;
  measured_received_sec_[2] = 0.0;
}

void TeleopManager::update_measured_pose(
  const Side side, const Pose & pose, const double received_sec) noexcept
{
  if (side != Side::left && side != Side::right) {
    return;
  }
  if (!algorithm::is_valid_pose(pose) || !std::isfinite(received_sec)) {
    invalidate_measured_pose(side);
    return;
  }
  measured_arm_poses_[index(side)] = TimedPose{pose, received_sec, true};
}

void TeleopManager::invalidate_measured_pose(const Side side) noexcept
{
  if (side == Side::left || side == Side::right) {
    measured_arm_poses_[index(side)].received = false;
    measured_arm_poses_[index(side)].received_sec = 0.0;
  }
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
    auto_reset_pending_ = config_.auto_reset_on_vr_connect;
    output_.safe_hold = true;
    output_.base_vx = 0.0;
    output_.base_vy = 0.0;
    output_.base_wz = 0.0;
    head_reanchor_required_ = true;
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
      head_reanchor_required_ = true;
      pose_mapper_.release_clutch(algorithm::Side::left);
      pose_mapper_.release_clutch(algorithm::Side::right);
      apply_measured_hold();
      clear_interaction_states();
    } else {
      if (recovery_release_required_) {
        recovery_release_required_ = false;
        clear_interaction_states();
      }
      if (auto_reset_pending_) {
        pose_mapper_.reset(algorithm::Side::left);
        pose_mapper_.reset(algorithm::Side::right);
        events.both_reset = true;
        events.reset_joint_request = true;
        auto_reset_pending_ = false;
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
  output_.left_target_valid = !output_.safe_hold && arm_feedback_fresh[0] && target_updated[0];
  output_.right_target_valid = !output_.safe_hold && arm_feedback_fresh[1] && target_updated[1];
  if (!output_.safe_hold && fresh) {
    update_head_command();
  } else {
    hold_head_command();
  }
  output_.head_position_valid = !output_.safe_hold && fresh && head_angles_initialized_;
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
  const std::array<double, 23> values{
    config.relative_position_scale, config.absolute_position_scale,
    config.watchdog_timeout_sec, config.feedback_timeout_sec,
    config.arm_pose_timeout_sec, config.long_press_sec, config.joystick_deadzone,
    config.gripper_min, config.gripper_max, config.base_max_forward,
    config.base_max_lateral, config.base_max_yaw, config.lift_min,
    config.lift_max, config.lift_initial_position, config.lift_jog_speed,
    config.head_min[0], config.head_min[1], config.head_min[2],
    config.head_max[0], config.head_max[1], config.head_max[2],
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
  for (std::size_t index = 0U; index < config.head_min.size(); ++index) {
    if (!std::isfinite(config.head_min[index]) ||
      !std::isfinite(config.head_max[index]) ||
      config.head_min[index] > config.head_max[index] ||
      config.head_min[index] > 0.0 || config.head_max[index] < 0.0)
    {
      throw std::invalid_argument("头部限位必须有限、有序且包含安全零位");
    }
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
  return std::isfinite(joy.x) && joy.x >= -1.0 && joy.x <= 1.0 &&
         std::isfinite(joy.y) && joy.y >= -1.0 && joy.y <= 1.0 &&
         std::isfinite(joy.trigger_value) && joy.trigger_value >= 0.0 &&
         joy.trigger_value <= 1.0;
}

bool TeleopManager::pose_to_head_euler(
  const Pose & pose, std::array<double, 3> & pitch_yaw_roll) noexcept
{
  if (!algorithm::is_valid_pose(pose)) {
    return false;
  }
  const double norm_squared =
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  if (!std::isfinite(norm_squared) || norm_squared < kQuaternionEpsilonSquared) {
    return false;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  const double x = pose.orientation.x * inverse_norm;
  const double y = pose.orientation.y * inverse_norm;
  const double z = pose.orientation.z * inverse_norm;
  const double w = pose.orientation.w * inverse_norm;
  const double squared_x = x * x;
  const double squared_y = y * y;
  const double squared_z = z * z;
  const double squared_w = w * w;
  const double sarg = std::clamp(
    -2.0 * (x * z - w * y), -1.0, 1.0);

  double yaw{0.0};
  double pitch{0.0};
  double roll{0.0};
  if (sarg <= -0.99999) {
    pitch = -0.5 * kPiHalf;
    roll = 0.0;
    yaw = -2.0 * std::atan2(y, x);
  } else if (sarg >= 0.99999) {
    pitch = 0.5 * kPiHalf;
    roll = 0.0;
    yaw = 2.0 * std::atan2(y, x);
  } else {
    pitch = std::asin(sarg);
    roll = std::atan2(
      2.0 * (y * z + w * x), squared_w - squared_x - squared_y + squared_z);
    yaw = std::atan2(
      2.0 * (x * y + w * z), squared_w + squared_x - squared_y - squared_z);
  }
  pitch_yaw_roll = {pitch, yaw, roll};
  return std::all_of(pitch_yaw_roll.cbegin(), pitch_yaw_roll.cend(),
    [](const double value) {return std::isfinite(value);});
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

void TeleopManager::update_head_angles(
  const std::array<double, 3> & pitch_yaw_roll) noexcept
{
  if (!std::all_of(pitch_yaw_roll.cbegin(), pitch_yaw_roll.cend(),
    [](const double value) {return std::isfinite(value);}))
  {
    return;
  }

  if (!head_angles_initialized_) {
    head_last_raw_angles_ = pitch_yaw_roll;
    head_unwrapped_angles_ = pitch_yaw_roll;
    // 首帧有效头显样本作为零参考, 避免 controller 重连时头部跳变。
    head_zero_offsets_ = pitch_yaw_roll;
    head_angles_initialized_ = true;
    head_reanchor_required_ = false;
    return;
  }

  if (head_reanchor_required_) {
    head_last_raw_angles_ = pitch_yaw_roll;
    head_unwrapped_angles_ = pitch_yaw_roll;
    for (std::size_t index = 0U; index < pitch_yaw_roll.size(); ++index) {
      const double held = std::isfinite(output_.head_position[index]) ?
        output_.head_position[index] : 0.0;
      head_zero_offsets_[index] = pitch_yaw_roll[index] - held;
    }
    head_reanchor_required_ = false;
    return;
  }

  for (std::size_t index = 0U; index < pitch_yaw_roll.size(); ++index) {
    const double delta = std::atan2(
      std::sin(pitch_yaw_roll[index] - head_last_raw_angles_[index]),
      std::cos(pitch_yaw_roll[index] - head_last_raw_angles_[index]));
    if (!std::isfinite(delta)) {
      return;
    }
    head_unwrapped_angles_[index] += delta;
    head_last_raw_angles_[index] = pitch_yaw_roll[index];
  }
}

void TeleopManager::update_head_command() noexcept
{
  if (!head_angles_initialized_) {
    hold_head_command();
    return;
  }
  for (std::size_t index = 0U; index < output_.head_position.size(); ++index) {
    const double position = head_unwrapped_angles_[index] - head_zero_offsets_[index];
    if (!std::isfinite(position)) {
      hold_head_command();
      return;
    }
    output_.head_position[index] = std::clamp(
      position, config_.head_min[index], config_.head_max[index]);
  }
}

void TeleopManager::hold_head_command() noexcept
{
  for (std::size_t index = 0U; index < output_.head_position.size(); ++index) {
    if (!std::isfinite(output_.head_position[index])) {
      output_.head_position[index] = 0.0;
    }
    output_.head_position[index] = std::clamp(
      output_.head_position[index], config_.head_min[index], config_.head_max[index]);
  }
}

bool TeleopManager::reset_head_reference() noexcept
{
  if (!head_angles_initialized_ ||
    !std::all_of(head_unwrapped_angles_.cbegin(), head_unwrapped_angles_.cend(),
    [](const double value) {return std::isfinite(value);}))
  {
    return false;
  }
  head_zero_offsets_ = head_unwrapped_angles_;
  head_reanchor_required_ = false;
  output_.head_position = {0.0, 0.0, 0.0};
  return true;
}

void TeleopManager::clear_hand_input(const std::size_t hand_index) noexcept
{
  if (hand_index >= hand_poses_.size()) {
    return;
  }
  hand_poses_[hand_index] = TimedPose{};
  joys_[hand_index] = TimedJoy{};
  pose_mapper_.release_clutch(static_cast<Side>(hand_index));
  if (hand_index == index(algorithm::Side::left)) {
    previous_left_joystick_ = false;
  } else {
    previous_right_joystick_ = false;
  }
}

void TeleopManager::clear_all_vr_input() noexcept
{
  hand_poses_ = {};
  joys_ = {};
  hand_connected_ = {false, false};
  head_pose_ = TimedPose{};
  pose_mapper_.release_clutch(algorithm::Side::left);
  pose_mapper_.release_clutch(algorithm::Side::right);
  head_reanchor_required_ = true;
  recovery_release_required_ = true;
  previous_left_joystick_ = false;
  previous_right_joystick_ = false;
}

bool TeleopManager::all_inputs_fresh(double now_sec) const noexcept
{
  for (std::size_t hand_index = 0U; hand_index < hand_poses_.size(); ++hand_index) {
    if (!hand_connected_[hand_index]) {
      // 该侧手柄未连接, 仅保持该侧构型。
      continue;
    }
    const TimedPose & hand_pose = hand_poses_[hand_index];
    if (!is_fresh(
        now_sec, hand_pose.received_sec, config_.watchdog_timeout_sec,
        hand_pose.received))
    {
      return false;
    }
  }
  for (std::size_t hand_index = 0U; hand_index < joys_.size(); ++hand_index) {
    if (!hand_connected_[hand_index]) {
      continue;
    }
    const TimedJoy & joy = joys_[hand_index];
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
  previous_left_joystick_ = joys_[index(algorithm::Side::left)].value.joystick;
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

void TeleopManager::begin_reset(const Side side) noexcept
{
  const std::size_t hand_index = index(side);
  // 从实测位姿出发插值, 避免从陈旧目标扫过工作空间。
  const TimedPose & measured = measured_arm_poses_[hand_index];
  reset_start_[hand_index] =
    (measured.received && algorithm::is_valid_pose(measured.value)) ?
    measured.value : config_.reset_poses[hand_index];
  reset_progress_[hand_index] = 0.0;
  reset_elapsed_[hand_index] = 0.0;
  reset_active_[hand_index] = true;
  pose_mapper_.release_clutch(side);
  pose_mapper_.set_target(side, reset_start_[hand_index]);
}

void TeleopManager::advance_reset(const Side side, const double dt_sec) noexcept
{
  const std::size_t hand_index = index(side);
  const Pose & reset_target = config_.reset_poses[hand_index];
  // 以当前实测位姿为起点闭环推进: 手臂跟不上时继续补发目标, 直到真正到位。
  const TimedPose & measured = measured_arm_poses_[hand_index];
  const Pose current =
    (measured.received && algorithm::is_valid_pose(measured.value)) ?
    measured.value : pose_mapper_.target(side);
  const double duration = std::max(0.1, config_.reset_duration_sec);
  const double step = (std::isfinite(dt_sec) && dt_sec > 0.0) ? dt_sec : 0.0;
  reset_elapsed_[hand_index] += step;
  const double fraction = std::clamp(step / duration, 0.0, 1.0);
  pose_mapper_.set_target(side, interpolate_pose(current, reset_target, fraction));

  const double dx = current.position.x - reset_target.position.x;
  const double dy = current.position.y - reset_target.position.y;
  const double dz = current.position.z - reset_target.position.z;
  const double position_gap = std::sqrt(dx * dx + dy * dy + dz * dz);
  double dot = current.orientation.x * reset_target.orientation.x +
    current.orientation.y * reset_target.orientation.y +
    current.orientation.z * reset_target.orientation.z +
    current.orientation.w * reset_target.orientation.w;
  dot = std::clamp(std::abs(dot), 0.0, 1.0);
  const double orientation_gap = 2.0 * std::acos(dot);
  if (position_gap < 0.005 && orientation_gap < 0.05) {
    pose_mapper_.set_target(side, reset_target);
    reset_active_[hand_index] = false;
    return;
  }
  const double timeout = std::max(duration, config_.reset_timeout_sec);
  if (reset_elapsed_[hand_index] >= timeout) {
    reset_active_[hand_index] = false;
  }
}

Pose TeleopManager::interpolate_pose(
  const Pose & start, const Pose & end, const double progress) noexcept
{
  const double t = std::clamp(progress, 0.0, 1.0);
  Pose result;
  result.position = algorithm::Vector3{
    start.position.x + (end.position.x - start.position.x) * t,
    start.position.y + (end.position.y - start.position.y) * t,
    start.position.z + (end.position.z - start.position.z) * t};
  algorithm::Quaternion a = start.orientation;
  algorithm::Quaternion b = end.orientation;
  double dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  if (dot < 0.0) {
    b.x = -b.x;
    b.y = -b.y;
    b.z = -b.z;
    b.w = -b.w;
    dot = -dot;
  }
  double s0 = 1.0 - t;
  double s1 = t;
  if (dot < 0.9995) {
    const double theta = std::acos(std::clamp(dot, -1.0, 1.0));
    const double sin_theta = std::sin(theta);
    if (std::isfinite(sin_theta) && std::abs(sin_theta) > 1.0e-9) {
      s0 = std::sin((1.0 - t) * theta) / sin_theta;
      s1 = std::sin(t * theta) / sin_theta;
    }
  }
  result.orientation = algorithm::Quaternion{
    a.x * s0 + b.x * s1, a.y * s0 + b.y * s1,
    a.z * s0 + b.z * s1, a.w * s0 + b.w * s1};
  return result;
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
      events.reset_joint_request = true;
    }
    if (update_long_press(right_combo, now_sec, right_reset_press_)) {
      pose_mapper_.reset(algorithm::Side::right);
      events.right_reset = true;
      events.reset_joint_request = true;
    }
  }

  if (update_long_press(left.menu || right.menu, now_sec, both_reset_press_)) {
    pose_mapper_.reset(algorithm::Side::left);
    pose_mapper_.reset(algorithm::Side::right);
    events.both_reset = true;
    events.reset_joint_request = true;
  }

  if (rising_edge(left.joystick, previous_left_joystick_)) {
    if (reset_head_reference()) {
      events.head_reset = true;
    }
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
