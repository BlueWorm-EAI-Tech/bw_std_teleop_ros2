#include "bw_teleop/algorithm/pose_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace bw_teleop::algorithm
{
namespace
{

constexpr double quaternion_epsilon{1.0e-12};

Quaternion normalized_quaternion(const Quaternion & value) noexcept
{
  const double norm = std::sqrt(
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
  if (!std::isfinite(norm) || norm < quaternion_epsilon) {
    return Quaternion{};
  }
  return Quaternion{value.x / norm, value.y / norm, value.z / norm, value.w / norm};
}

bool has_valid_orientation(const Pose & pose) noexcept
{
  const Quaternion & value = pose.orientation;
  const double norm_squared =
    value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w;
  return std::isfinite(norm_squared) && norm_squared >= quaternion_epsilon * quaternion_epsilon;
}

bool valid_workspace(const WorkspaceLimits & workspace) noexcept
{
  const Vector3 & minimum = workspace.minimum;
  const Vector3 & maximum = workspace.maximum;
  return std::isfinite(minimum.x) && std::isfinite(minimum.y) &&
         std::isfinite(minimum.z) && std::isfinite(maximum.x) &&
         std::isfinite(maximum.y) && std::isfinite(maximum.z) &&
         minimum.x <= maximum.x && minimum.y <= maximum.y && minimum.z <= maximum.z;
}

bool inside_workspace(const Pose & pose, const WorkspaceLimits & workspace) noexcept
{
  return pose.position.x >= workspace.minimum.x &&
         pose.position.x <= workspace.maximum.x &&
         pose.position.y >= workspace.minimum.y &&
         pose.position.y <= workspace.maximum.y &&
         pose.position.z >= workspace.minimum.z &&
         pose.position.z <= workspace.maximum.z;
}

}  // namespace

bool is_finite_pose(const Pose & pose) noexcept
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

bool is_valid_pose(const Pose & pose) noexcept
{
  return is_finite_pose(pose) && has_valid_orientation(pose);
}

PoseMapper::PoseMapper(PoseMapperConfig config)
: config_(std::move(config))
{
  if (!std::isfinite(config_.relative_position_scale) ||
    config_.relative_position_scale < 0.0 ||
    !std::isfinite(config_.absolute_position_scale) ||
    config_.absolute_position_scale < 0.0)
  {
    throw std::invalid_argument("位姿映射比例必须为有限非负数");
  }
  if (!valid_workspace(config_.workspace)) {
    throw std::invalid_argument("工作空间边界必须有限且按 min <= max 排列");
  }

  for (std::size_t hand_index = 0; hand_index < hands_.size(); ++hand_index) {
    if (!is_valid_pose(config_.reset_poses[hand_index]) ||
      !inside_workspace(config_.reset_poses[hand_index], config_.workspace))
    {
      throw std::invalid_argument("reset pose 必须有限、姿态有效且位于工作空间内");
    }
    config_.reset_poses[hand_index].orientation =
      normalize(config_.reset_poses[hand_index].orientation);
    hands_[hand_index].target = config_.reset_poses[hand_index];
    hands_[hand_index].target_baseline = hands_[hand_index].target;
  }
}

void PoseMapper::set_control_mode(ControlMode mode) noexcept
{
  if (mode == control_mode_) {
    return;
  }
  control_mode_ = mode;
  release_clutch(Side::left);
  release_clutch(Side::right);
}

ControlMode PoseMapper::control_mode() const noexcept
{
  return control_mode_;
}

bool PoseMapper::update(
  Side side, const Pose & hand_pose, const bool clutch_pressed,
  const Pose & measured_target, const bool measured_target_valid) noexcept
{
  HandState & hand = hands_[index(side)];
  if (!clutch_pressed || !is_valid_pose(hand_pose)) {
    hand.clutch_active = false;
    return false;
  }

  Pose normalized_hand = hand_pose;
  normalized_hand.orientation = normalize(hand_pose.orientation);

  if (control_mode_ == ControlMode::absolute) {
    const Pose & reset = config_.reset_poses[index(side)];
    Pose target;
    target.position = Vector3{
      reset.position.x + normalized_hand.position.x * config_.absolute_position_scale,
      reset.position.y + normalized_hand.position.y * config_.absolute_position_scale,
      reset.position.z + normalized_hand.position.z * config_.absolute_position_scale};
    target.orientation = normalized_hand.orientation;
    hand.target = clamp_to_workspace(target);
    hand.clutch_active = true;
    return true;
  }

  if (!hand.clutch_active) {
    if (!measured_target_valid || !is_valid_pose(measured_target) ||
      !inside_workspace(measured_target, config_.workspace))
    {
      return false;
    }
    hand.hand_baseline = normalized_hand;
    hand.target_baseline = measured_target;
    hand.target_baseline.orientation = normalize(measured_target.orientation);
    hand.target = hand.target_baseline;
    hand.clutch_active = true;
  }

  const Vector3 delta{
    normalized_hand.position.x - hand.hand_baseline.position.x,
    normalized_hand.position.y - hand.hand_baseline.position.y,
    normalized_hand.position.z - hand.hand_baseline.position.z};
  const Quaternion rotation_delta = multiply(
    conjugate(hand.hand_baseline.orientation), normalized_hand.orientation);

  Pose target;
  target.position = Vector3{
    hand.target_baseline.position.x + delta.x * config_.relative_position_scale,
    hand.target_baseline.position.y + delta.y * config_.relative_position_scale,
    hand.target_baseline.position.z + delta.z * config_.relative_position_scale};
  target.orientation = normalize(multiply(hand.target_baseline.orientation, rotation_delta));
  hand.target = clamp_to_workspace(target);
  return true;
}

void PoseMapper::release_clutch(Side side) noexcept
{
  hands_[index(side)].clutch_active = false;
}

void PoseMapper::reset(Side side) noexcept
{
  HandState & hand = hands_[index(side)];
  hand.target = clamp_to_workspace(config_.reset_poses[index(side)]);
  hand.target_baseline = hand.target;
  hand.clutch_active = false;
}

Pose PoseMapper::target(Side side) const noexcept
{
  return hands_[index(side)].target;
}

std::size_t PoseMapper::index(Side side) noexcept
{
  return static_cast<std::size_t>(side);
}

Quaternion PoseMapper::normalize(const Quaternion & value) noexcept
{
  return normalized_quaternion(value);
}

Quaternion PoseMapper::conjugate(const Quaternion & value) noexcept
{
  return Quaternion{-value.x, -value.y, -value.z, value.w};
}

Quaternion PoseMapper::multiply(
  const Quaternion & left, const Quaternion & right) noexcept
{
  return Quaternion{
    left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
    left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
    left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
    left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z};
}

Pose PoseMapper::clamp_to_workspace(const Pose & pose) const noexcept
{
  Pose result = pose;
  result.position.x = std::clamp(
    result.position.x, config_.workspace.minimum.x, config_.workspace.maximum.x);
  result.position.y = std::clamp(
    result.position.y, config_.workspace.minimum.y, config_.workspace.maximum.y);
  result.position.z = std::clamp(
    result.position.z, config_.workspace.minimum.z, config_.workspace.maximum.z);
  result.orientation = normalize(result.orientation);
  return result;
}

}  // namespace bw_teleop::algorithm
