#pragma once

#include <array>

#include "bw_teleop/algorithm/pose_mapper.hpp"
#include "bw_teleop/input/vr_input_mapper.hpp"

namespace bw_teleop::manager
{

using Side = algorithm::Side;
using ControlMode = algorithm::ControlMode;
using Pose = algorithm::Pose;
using WorkspaceLimits = algorithm::WorkspaceLimits;

using JoyInput = input::JoyInput;

struct MeasuredState
{
  bool left_gripper_valid{false};
  bool right_gripper_valid{false};
  bool lift_valid{false};
  double left_gripper{0.0};
  double right_gripper{0.0};
  double lift{0.0};
};

struct TeleopConfig
{
  std::array<Pose, 2> reset_poses{
    Pose{{0.40, 0.25, -0.15}, {}},
    Pose{{0.40, -0.25, -0.15}, {}}};
  WorkspaceLimits workspace;
  double relative_position_scale{1.0};
  double absolute_position_scale{1.0};
  double watchdog_timeout_sec{0.20};
  double feedback_timeout_sec{0.20};
  double arm_pose_timeout_sec{0.20};
  double long_press_sec{1.0};
  bool auto_reset_on_vr_connect{false};
  double reset_duration_sec{1.5};
  double reset_timeout_sec{20.0};
  double joystick_deadzone{0.08};
  double gripper_min{0.0};
  double gripper_max{0.04965};
  double base_max_forward{0.50};
  double base_max_lateral{0.35};
  double base_max_yaw{0.80};
  std::array<double, 3> base_speed_scales{0.25, 0.50, 1.0};
  int base_initial_gear{0};
  double lift_min{-500.0};
  double lift_max{0.0};
  double lift_initial_position{-100.0};
  double lift_jog_speed{200.0};
  std::array<double, 3> head_min{-0.785, -1.570, -0.349};
  std::array<double, 3> head_max{0.524, 1.570, 0.349};
};

struct TeleopOutput
{
  Pose left_target;
  Pose right_target;
  bool left_target_valid{false};
  bool right_target_valid{false};
  std::array<double, 3> head_position{0.0, 0.0, 0.0};
  bool head_position_valid{false};
  std::array<double, 2> gripper_command{0.0, 0.0};
  double lift_command{-100.0};
  bool auxiliary_commands_valid{false};
  double base_vx{0.0};
  double base_vy{0.0};
  double base_wz{0.0};
  bool safe_hold{true};
  ControlMode control_mode{ControlMode::relative};
  int base_speed_gear{0};
};

struct ManagerEvents
{
  bool mode_changed{false};
  bool left_reset{false};
  bool right_reset{false};
  bool both_reset{false};
  bool head_reset{false};
  bool speed_gear_changed{false};
  bool entered_safe_hold{false};
  bool exited_safe_hold{false};
  bool reset_joint_request{false};
};

struct TickResult
{
  TeleopOutput output;
  ManagerEvents events;
};

/**
 * @brief 编排 VR 状态、按键事件、watchdog 和整机基础命令。
 */
class TeleopManager
{
public:
  explicit TeleopManager(TeleopConfig config);

  void update_vr_frame(
    const input::MappedVrFrame & frame, double received_sec) noexcept;
  void clear_vr_input() noexcept;
  void update_measured_state(
    const MeasuredState & state, double received_sec) noexcept;
  void invalidate_measured_state() noexcept;
  void invalidate_measured_gripper(Side side) noexcept;
  void invalidate_measured_lift() noexcept;
  void update_measured_pose(
    Side side, const Pose & pose, double received_sec) noexcept;
  void invalidate_measured_pose(Side side) noexcept;
  TickResult tick(double now_sec) noexcept;

private:
  struct TimedPose
  {
    Pose value;
    double received_sec{0.0};
    bool received{false};
  };

  struct TimedJoy
  {
    JoyInput value;
    double received_sec{0.0};
    bool received{false};
  };

  struct LongPressState
  {
    double started_sec{0.0};
    bool active{false};
    bool fired{false};
  };

  static TeleopConfig normalize_config(TeleopConfig config);
  static algorithm::PoseMapperConfig make_pose_mapper_config(
    const TeleopConfig & config) noexcept;
  static std::size_t index(Side side) noexcept;
  static bool valid_joy(const JoyInput & joy) noexcept;
  static bool pose_to_head_euler(
    const Pose & pose, std::array<double, 3> & pitch_yaw_roll) noexcept;
  static bool is_fresh(
    double now_sec, double received_sec, double timeout_sec, bool received) noexcept;
  void clear_hand_input(std::size_t hand_index) noexcept;
  void clear_all_vr_input() noexcept;
  bool all_inputs_fresh(double now_sec) const noexcept;
  void update_head_angles(const std::array<double, 3> & pitch_yaw_roll) noexcept;
  void update_head_command() noexcept;
  void hold_head_command() noexcept;
  bool reset_head_reference() noexcept;
  bool update_long_press(
    bool condition, double now_sec, LongPressState & state) noexcept;
  static bool rising_edge(bool current, bool & previous) noexcept;
  double apply_deadzone(double value) const noexcept;
  void clear_interaction_states() noexcept;
  void apply_measured_hold() noexcept;
  void process_button_events(double now_sec, ManagerEvents & events) noexcept;
  void begin_reset(Side side) noexcept;
  void advance_reset(Side side, double dt_sec) noexcept;
  static Pose interpolate_pose(
    const Pose & start, const Pose & end, double progress) noexcept;
  [[nodiscard]] std::array<bool, 2> update_active_commands(
    double dt_sec, const std::array<bool, 2> & arm_feedback_fresh,
    bool auxiliary_updates_enabled) noexcept;

  TeleopConfig config_;
  algorithm::PoseMapper pose_mapper_;
  std::array<TimedPose, 2> hand_poses_;
  // PICO 报告的手柄连接状态: 未连接的一侧不参与输入新鲜度判定。
  std::array<bool, 2> hand_connected_{{false, false}};
  TimedPose head_pose_;
  std::array<TimedPose, 2> measured_arm_poses_;
  std::array<TimedJoy, 2> joys_;
  MeasuredState measured_state_;
  std::array<double, 3> measured_received_sec_{0.0, 0.0, 0.0};
  TeleopOutput output_;

  LongPressState mode_press_;
  LongPressState left_reset_press_;
  LongPressState right_reset_press_;
  LongPressState both_reset_press_;
  bool previous_left_joystick_{false};
  bool previous_right_joystick_{false};
  std::array<double, 3> head_unwrapped_angles_{0.0, 0.0, 0.0};
  std::array<double, 3> head_last_raw_angles_{0.0, 0.0, 0.0};
  std::array<double, 3> head_zero_offsets_{0.0, 0.0, 0.0};
  bool head_angles_initialized_{false};
  bool head_reanchor_required_{false};
  bool recovery_release_required_{true};
  bool auto_reset_pending_{false};
  std::array<Pose, 2> reset_start_{};
  std::array<double, 2> reset_progress_{{0.0, 0.0}};
  std::array<double, 2> reset_elapsed_{{0.0, 0.0}};
  std::array<bool, 2> reset_active_{{false, false}};
  bool previous_safe_hold_{true};
  bool previous_auxiliary_feedback_fresh_{false};
  bool auxiliary_hold_pending_{false};
  int base_speed_gear_{0};
  bool tick_initialized_{false};
  double last_tick_sec_{0.0};
};

}  // namespace bw_teleop::manager
