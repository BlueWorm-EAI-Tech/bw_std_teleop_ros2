#include "bw_kinematics/manager/kinematics_manager.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace bw_kinematics
{

KinematicsManager::KinematicsManager() = default;

bool KinematicsManager::configure(
  const std::string & robot_description,
  const KinematicsManagerConfig & config,
  std::string & error_message)
{
  configured_ = false;
  feedback_received_ = false;
  if (!std::isfinite(config.feedback_timeout_sec) || config.feedback_timeout_sec <= 0.0) {
    error_message = "反馈超时必须为有限正数";
    return false;
  }
  std::string solver_error;
  if (!left_solver_.configure(robot_description, config.left_arm, solver_error)) {
    error_message = "左臂配置失败: " + solver_error;
    return false;
  }
  if (!right_solver_.configure(robot_description, config.right_arm, solver_error)) {
    error_message = "右臂配置失败: " + solver_error;
    return false;
  }

  left_joint_names_ = left_solver_.joint_names();
  right_joint_names_ = right_solver_.joint_names();
  std::unordered_set<std::string> unique_names;
  for (const auto & name : left_joint_names_) {
    unique_names.insert(name);
  }
  for (const auto & name : right_joint_names_) {
    if (!unique_names.insert(name).second) {
      error_message = "左右运动链包含重复关节: " + name;
      return false;
    }
  }

  left_state_ = ArmState{};
  right_state_ = ArmState{};
  left_state_.measured_positions.resize(left_joint_names_.size(), 0.0);
  left_state_.command_positions.resize(left_joint_names_.size(), 0.0);
  left_state_.measured.resize(left_joint_names_.size(), false);
  right_state_.measured_positions.resize(right_joint_names_.size(), 0.0);
  right_state_.command_positions.resize(right_joint_names_.size(), 0.0);
  right_state_.measured.resize(right_joint_names_.size(), false);

  joint_names_ = left_joint_names_;
  joint_names_.insert(
    joint_names_.end(), right_joint_names_.cbegin(), right_joint_names_.cend());
  feedback_timeout_sec_ = config.feedback_timeout_sec;
  configured_ = true;
  error_message.clear();
  return true;
}

bool KinematicsManager::update_measured_state(
  const std::vector<std::string> & names,
  const std::vector<double> & positions,
  const double now_sec)
{
  if (!configured_ || names.size() != positions.size() || !std::isfinite(now_sec)) {
    invalidate_measured_state();
    return false;
  }

  std::unordered_map<std::string, double> measured_by_name;
  measured_by_name.reserve(names.size());
  for (std::size_t index = 0U; index < names.size(); ++index) {
    if (!measured_by_name.emplace(names[index], positions[index]).second) {
      invalidate_measured_state();
      return false;
    }
  }

  ArmState next_left = left_state_;
  ArmState next_right = right_state_;
  const bool resynchronize_commands = !feedback_received_ ||
    now_sec < last_feedback_time_sec_ ||
    now_sec - last_feedback_time_sec_ > feedback_timeout_sec_;
  if (resynchronize_commands) {
    next_left.command_initialized = false;
    next_right.command_initialized = false;
  }
  const auto collect_arm = [&measured_by_name](
      const std::vector<std::string> & required_names, ArmState & state) {
      for (std::size_t index = 0U; index < required_names.size(); ++index) {
        const auto measured = measured_by_name.find(required_names[index]);
        if (measured == measured_by_name.cend() || !std::isfinite(measured->second)) {
          return false;
        }
        state.measured_positions[index] = measured->second;
        state.measured[index] = true;
      }
      return true;
    };

  if (!collect_arm(left_joint_names_, next_left) ||
    !collect_arm(right_joint_names_, next_right))
  {
    invalidate_measured_state();
    return false;
  }

  left_state_ = std::move(next_left);
  right_state_ = std::move(next_right);
  last_feedback_time_sec_ = now_sec;
  feedback_received_ = true;
  initialize_command_if_ready(left_state_);
  initialize_command_if_ready(right_state_);
  return true;
}

KinematicsCommand KinematicsManager::solve_target(
  const ArmSide side,
  const CartesianPose & target,
  const double now_sec)
{
  if (!ready(now_sec)) {
    return make_command(false, "双臂测量关节状态尚未就绪或已超时", now_sec);
  }

  DlsIkSolver & solver = side == ArmSide::left ? left_solver_ : right_solver_;
  ArmState & state = side == ArmSide::left ? left_state_ : right_state_;
  const IkSolveResult result = solver.solve(target, state.measured_positions);
  if (!result.success) {
    return make_command(false, result.message, now_sec);
  }

  state.command_positions = result.positions;
  state.command_initialized = true;
  return make_command(true, result.message, now_sec);
}

ForwardKinematicsResult KinematicsManager::measured_pose(
  const ArmSide side, const double now_sec) const
{
  if (!ready(now_sec)) {
    ForwardKinematicsResult result;
    result.message = "双臂测量关节状态尚未就绪或已超时";
    return result;
  }

  const DlsIkSolver & solver = side == ArmSide::left ? left_solver_ : right_solver_;
  const ArmState & state = side == ArmSide::left ? left_state_ : right_state_;
  return solver.compute_fk(state.measured_positions);
}

bool KinematicsManager::ready(const double now_sec) const noexcept
{
  const bool feedback_is_fresh = feedback_received_ && std::isfinite(now_sec) &&
    now_sec >= last_feedback_time_sec_ &&
    now_sec - last_feedback_time_sec_ <= feedback_timeout_sec_;
  return configured_ && feedback_is_fresh && arm_ready(left_state_) && arm_ready(right_state_) &&
         left_state_.command_initialized && right_state_.command_initialized;
}

bool KinematicsManager::arm_ready(const ArmState & state) noexcept
{
  return !state.measured.empty() &&
         std::all_of(state.measured.cbegin(), state.measured.cend(), [](const bool value) {
           return value;
         });
}

void KinematicsManager::invalidate_measured_state() noexcept
{
  std::fill(left_state_.measured.begin(), left_state_.measured.end(), false);
  std::fill(right_state_.measured.begin(), right_state_.measured.end(), false);
  left_state_.command_initialized = false;
  right_state_.command_initialized = false;
  feedback_received_ = false;
}

void KinematicsManager::initialize_command_if_ready(ArmState & state)
{
  if (!state.command_initialized && arm_ready(state)) {
    state.command_positions = state.measured_positions;
    state.command_initialized = true;
  }
}

KinematicsCommand KinematicsManager::make_command(
  const bool requested_arm_succeeded,
  const std::string & message,
  const double now_sec) const
{
  KinematicsCommand command;
  command.requested_arm_succeeded = requested_arm_succeeded;
  command.message = message;
  if (!ready(now_sec)) {
    return command;
  }

  command.has_command = true;
  command.joint_names = joint_names_;
  command.positions = left_state_.command_positions;
  command.positions.insert(
    command.positions.end(),
    right_state_.command_positions.cbegin(), right_state_.command_positions.cend());
  return command;
}

}  // namespace bw_kinematics
