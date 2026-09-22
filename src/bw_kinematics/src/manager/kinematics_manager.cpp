#include "bw_kinematics/manager/kinematics_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

namespace bw_kinematics
{
namespace
{

constexpr double kQuaternionNormEpsilon = 1.0e-12;

double position_error(
  const CartesianPose & actual, const CartesianPose & target) noexcept
{
  const double dx = actual.position[0] - target.position[0];
  const double dy = actual.position[1] - target.position[1];
  const double dz = actual.position[2] - target.position[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientation_error(
  const CartesianPose & actual, const CartesianPose & target) noexcept
{
  const auto norm = [](const std::array<double, 4> & value) {
      return std::sqrt(
        value[0] * value[0] + value[1] * value[1] + value[2] * value[2] +
        value[3] * value[3]);
    };
  const double actual_norm = norm(actual.orientation_xyzw);
  const double target_norm = norm(target.orientation_xyzw);
  if (actual_norm < kQuaternionNormEpsilon || target_norm < kQuaternionNormEpsilon) {
    return std::numeric_limits<double>::infinity();
  }
  double dot = 0.0;
  for (std::size_t index = 0U; index < 4U; ++index) {
    dot += actual.orientation_xyzw[index] * target.orientation_xyzw[index];
  }
  dot = std::abs(dot / (actual_norm * target_norm));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

}  // namespace

KinematicsManager::KinematicsManager() = default;

bool KinematicsManager::configure(
  const std::string & standard_ik_urdf_xml,
  const KinematicsManagerConfig & config,
  std::string & error_message)
{
  configured_ = false;
  feedback_received_ = false;
  measured_complete_ = false;
  command_initialized_ = false;
  if (!std::isfinite(config.feedback_timeout_sec) || config.feedback_timeout_sec <= 0.0 ||
    !std::isfinite(config.max_joint_step_rad) || config.max_joint_step_rad <= 0.0)
  {
    error_message = "反馈超时和单周期关节步长必须为有限正数";
    return false;
  }

  auto model = std::make_shared<StandardKinematicsModel>();
  if (!model->init(standard_ik_urdf_xml, error_message)) {
    return false;
  }
  auto const_model = std::shared_ptr<const StandardKinematicsModel>{model};
  if (!fk_solver_.init(const_model, error_message)) {
    return false;
  }
  if (!ik_solver_.init(const_model, config.ik, error_message)) {
    return false;
  }

  joint_names_ = model->joint_names();
  lower_limits_ = model->lower_limits();
  upper_limits_ = model->upper_limits();
  if (joint_names_.size() != kStandardArmDof || lower_limits_.size() != kStandardArmDof ||
    upper_limits_.size() != kStandardArmDof)
  {
    error_message = "Standard 运动学模型未返回完整 14 轴契约";
    return false;
  }
  model_ = std::move(model);
  measured_positions_.assign(kStandardArmDof, 0.0);
  command_positions_.assign(kStandardArmDof, 0.0);
  feedback_timeout_sec_ = config.feedback_timeout_sec;
  max_joint_step_rad_ = config.max_joint_step_rad;
  max_position_residual_ = config.ik.max_position_residual;
  max_orientation_residual_ = config.ik.max_orientation_residual;
  if (!std::isfinite(max_position_residual_) || max_position_residual_ <= 0.0) {
    max_position_residual_ = 0.03;
  }
  if (!std::isfinite(max_orientation_residual_) || max_orientation_residual_ <= 0.0) {
    max_orientation_residual_ = 0.25;
  }
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
    if (!std::isfinite(positions[index]) ||
      !measured_by_name.emplace(names[index], positions[index]).second)
    {
      invalidate_measured_state();
      return false;
    }
  }

  std::vector<double> next_positions(kStandardArmDof, 0.0);
  for (std::size_t index = 0U; index < joint_names_.size(); ++index) {
    const auto found = measured_by_name.find(joint_names_[index]);
    if (found == measured_by_name.end() || !std::isfinite(found->second)) {
      invalidate_measured_state();
      return false;
    }
    next_positions[index] = found->second;
  }
  if (!finite_and_bounded(next_positions)) {
    invalidate_measured_state();
    return false;
  }

  const bool resynchronize = !feedback_received_ || !feedback_is_fresh(now_sec);
  measured_positions_ = std::move(next_positions);
  feedback_received_ = true;
  measured_complete_ = true;
  last_feedback_time_sec_ = now_sec;
  if (resynchronize || !command_initialized_) {
    initialize_command_from_measurement();
  }
  return true;
}

KinematicsCommand KinematicsManager::solve_target(
  const ArmSide side, const CartesianPose & target, const double now_sec)
{
  if (side != ArmSide::left && side != ArmSide::right) {
    return make_command(false, "Standard IK side 非法", now_sec);
  }
  if (!ready(now_sec)) {
    return make_command(false, "双臂 14 轴反馈尚未就绪或已超时", now_sec);
  }
  if (!is_valid_pose(target)) {
    return make_command(false, "末端目标位姿非法", now_sec);
  }

  BimanualPoseTarget target_pair;
  const BimanualForwardKinematicsResult current_pose =
    fk_solver_.get_bimanual_pose(measured_positions_);
  if (!current_pose.success) {
    return make_command(false, "当前反馈 FK 失败: " + current_pose.message, now_sec);
  }
  target_pair.left = current_pose.pose.left;
  target_pair.right = current_pose.pose.right;
  if (side == ArmSide::left) {
    target_pair.left = target;
  } else {
    target_pair.right = target;
  }

  const std::vector<double> & seed =
    (command_initialized_ && command_positions_.size() == kStandardArmDof) ?
    command_positions_ : measured_positions_;
  const IkSolveResult result = ik_solver_.solve(target_pair, seed);
  if (!result.success || result.positions.size() != kStandardArmDof) {
    return make_command(false, result.message, now_sec);
  }
  if (!finite_and_bounded(result.positions) || exceeds_joint_step(result.positions)) {
    return make_command(false, "IK 输出未通过限位或单周期关节步长门禁", now_sec);
  }
  const bool progressive = !result.success;
  if (progressive && !residual_improves(target_pair, current_pose, result)) {
    return make_command(false, result.message, now_sec);
  }

  command_positions_ = result.positions;
  command_initialized_ = true;
  return make_command(
    true, progressive ? (result.message + "; 按渐进增量下发") : result.message, now_sec);
}

ForwardKinematicsResult KinematicsManager::measured_pose(
  const ArmSide side, const double now_sec) const
{
  ForwardKinematicsResult result;
  if (side != ArmSide::left && side != ArmSide::right) {
    result.message = "Standard FK side 非法";
    return result;
  }
  if (!ready(now_sec)) {
    result.message = "双臂 14 轴反馈尚未就绪或已超时";
    return result;
  }
  return fk_solver_.get_ee_coordinate(side, measured_positions_);
}

bool KinematicsManager::ready(const double now_sec) const noexcept
{
  return configured_ && measured_complete_ && command_initialized_ &&
         feedback_is_fresh(now_sec);
}

bool KinematicsManager::feedback_is_fresh(const double now_sec) const noexcept
{
  return feedback_received_ && std::isfinite(now_sec) && std::isfinite(last_feedback_time_sec_) &&
         now_sec >= last_feedback_time_sec_ &&
         now_sec - last_feedback_time_sec_ <= feedback_timeout_sec_;
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
  command.positions = command_positions_;
  return command;
}

void KinematicsManager::invalidate_measured_state() noexcept
{
  measured_complete_ = false;
  feedback_received_ = false;
  command_initialized_ = false;
}

void KinematicsManager::initialize_command_from_measurement() noexcept
{
  if (measured_complete_ && measured_positions_.size() == kStandardArmDof) {
    command_positions_ = measured_positions_;
    command_initialized_ = true;
  }
}

bool KinematicsManager::copy_measured_positions(
  std::vector<double> & out, const double now_sec) const noexcept
{
  if (!configured_ || !measured_complete_ || !feedback_is_fresh(now_sec) ||
    measured_positions_.size() != kStandardArmDof)
  {
    return false;
  }
  for (const double value : measured_positions_) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  out = measured_positions_;
  return true;
}

bool KinematicsManager::clamp_to_limits(std::vector<double> & positions) const noexcept
{
  if (positions.size() != kStandardArmDof || lower_limits_.size() != kStandardArmDof ||
    upper_limits_.size() != kStandardArmDof)
  {
    return false;
  }
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    if (!std::isfinite(positions[index]) || !std::isfinite(lower_limits_[index]) ||
      !std::isfinite(upper_limits_[index]) || lower_limits_[index] > upper_limits_[index])
    {
      return false;
    }
    positions[index] = std::clamp(positions[index], lower_limits_[index], upper_limits_[index]);
  }
  return true;
}

bool KinematicsManager::finite_and_bounded(
  const std::vector<double> & positions) const noexcept
{
  if (positions.size() != kStandardArmDof || lower_limits_.size() != kStandardArmDof ||
    upper_limits_.size() != kStandardArmDof)
  {
    return false;
  }
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    if (!std::isfinite(positions[index]) || !std::isfinite(lower_limits_[index]) ||
      !std::isfinite(upper_limits_[index]) || lower_limits_[index] > upper_limits_[index] ||
      positions[index] < lower_limits_[index] || positions[index] > upper_limits_[index])
    {
      return false;
    }
  }
  return true;
}

bool KinematicsManager::exceeds_joint_step(
  const std::vector<double> & positions) const noexcept
{
  if (positions.size() != kStandardArmDof ||
    measured_positions_.size() != positions.size())
  {
    return true;
  }
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    if (!std::isfinite(measured_positions_[index]) ||
      std::abs(positions[index] - measured_positions_[index]) > max_joint_step_rad_)
    {
      return true;
    }
  }
  return false;
}

bool KinematicsManager::residual_improves(
  const BimanualPoseTarget & targets,
  const BimanualForwardKinematicsResult & current,
  const IkSolveResult & candidate) const noexcept
{
  if (!current.success) {
    return false;
  }
  const auto worst = [this](
    const double left_position, const double right_position,
    const double left_orientation, const double right_orientation) {
      const double position_scale = max_position_residual_ > 0.0 ? max_position_residual_ : 0.03;
      const double orientation_scale =
        max_orientation_residual_ > 0.0 ? max_orientation_residual_ : 0.25;
      return std::max(
        std::max(left_position / position_scale, right_position / position_scale),
        std::max(left_orientation / orientation_scale, right_orientation / orientation_scale));
    };
  const double current_worst = worst(
    position_error(current.pose.left, targets.left),
    position_error(current.pose.right, targets.right),
    orientation_error(current.pose.left, targets.left),
    orientation_error(current.pose.right, targets.right));
  const double candidate_worst = worst(
    candidate.left_position_error, candidate.right_position_error,
    candidate.left_orientation_error, candidate.right_orientation_error);
  if (!std::isfinite(current_worst) || !std::isfinite(candidate_worst)) {
    return false;
  }
  return candidate_worst < current_worst;
}

}  // namespace bw_kinematics
