#include "bw_kinematics/algorithm/dls_ik_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace bw_kinematics
{
namespace
{

constexpr std::size_t kTaskDimension{6U};
constexpr double kMinimumQuaternionNorm{1.0e-12};
constexpr double kMinimumJointRange{1.0e-9};

bool all_finite(const std::vector<double> & values)
{
  return std::all_of(
    values.cbegin(), values.cend(), [](const double value) {return std::isfinite(value);});
}

bool pose_is_finite(const CartesianPose & pose)
{
  return all_finite(
    {pose.position[0], pose.position[1], pose.position[2], pose.orientation_xyzw[0],
      pose.orientation_xyzw[1], pose.orientation_xyzw[2], pose.orientation_xyzw[3]});
}

bool frame_is_finite(const KDL::Frame & frame)
{
  for (unsigned int row = 0U; row < 3U; ++row) {
    if (!std::isfinite(frame.p[row])) {
      return false;
    }
    for (unsigned int column = 0U; column < 3U; ++column) {
      if (!std::isfinite(frame.M(row, column))) {
        return false;
      }
    }
  }
  return true;
}

KDL::JntArray to_joint_array(const std::vector<double> & positions)
{
  KDL::JntArray joints{static_cast<unsigned int>(positions.size())};
  for (std::size_t index = 0U; index < positions.size(); ++index) {
    joints(static_cast<unsigned int>(index)) = positions[index];
  }
  return joints;
}

CartesianPose to_cartesian_pose(const KDL::Frame & frame)
{
  CartesianPose pose;
  pose.position = {frame.p.x(), frame.p.y(), frame.p.z()};
  frame.M.GetQuaternion(
    pose.orientation_xyzw[0], pose.orientation_xyzw[1],
    pose.orientation_xyzw[2], pose.orientation_xyzw[3]);
  return pose;
}

bool to_kdl_frame(const CartesianPose & pose, KDL::Frame & frame)
{
  if (!pose_is_finite(pose)) {
    return false;
  }

  const double quaternion_norm = std::sqrt(
    pose.orientation_xyzw[0] * pose.orientation_xyzw[0] +
    pose.orientation_xyzw[1] * pose.orientation_xyzw[1] +
    pose.orientation_xyzw[2] * pose.orientation_xyzw[2] +
    pose.orientation_xyzw[3] * pose.orientation_xyzw[3]);
  if (!std::isfinite(quaternion_norm) || quaternion_norm < kMinimumQuaternionNorm) {
    return false;
  }

  const double inverse_norm = 1.0 / quaternion_norm;
  frame = KDL::Frame{
    KDL::Rotation::Quaternion(
      pose.orientation_xyzw[0] * inverse_norm,
      pose.orientation_xyzw[1] * inverse_norm,
      pose.orientation_xyzw[2] * inverse_norm,
      pose.orientation_xyzw[3] * inverse_norm),
    KDL::Vector{pose.position[0], pose.position[1], pose.position[2]}};
  return frame_is_finite(frame);
}

}  // namespace

class DlsIkSolver::Impl
{
public:
  DlsIkConfig config;
  KDL::Chain chain;
  std::vector<std::string> joint_names;
  std::vector<double> lower_limits;
  std::vector<double> upper_limits;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
  bool configured{false};
};

DlsIkSolver::DlsIkSolver()
: impl_{std::make_unique<Impl>()}
{
}

DlsIkSolver::~DlsIkSolver() = default;
DlsIkSolver::DlsIkSolver(DlsIkSolver &&) noexcept = default;
DlsIkSolver & DlsIkSolver::operator=(DlsIkSolver &&) noexcept = default;

bool DlsIkSolver::configure(
  const std::string & robot_description,
  const DlsIkConfig & config,
  std::string & error_message)
{
  impl_->configured = false;
  impl_->joint_names.clear();
  impl_->lower_limits.clear();
  impl_->upper_limits.clear();

  if (robot_description.empty()) {
    error_message = "robot_description 为空";
    return false;
  }
  if (config.root_link.empty() || config.tip_link.empty()) {
    error_message = "根链接和末端链接不能为空";
    return false;
  }
  if (config.max_iterations == 0U || !std::isfinite(config.damping) || config.damping <= 0.0 ||
    !std::isfinite(config.max_joint_step) || config.max_joint_step <= 0.0 ||
    !std::isfinite(config.position_tolerance) || config.position_tolerance <= 0.0 ||
    !std::isfinite(config.orientation_tolerance) || config.orientation_tolerance <= 0.0 ||
    !std::isfinite(config.joint_centering_gain) || config.joint_centering_gain < 0.0)
  {
    error_message = "IK 数值参数非法";
    return false;
  }

  urdf::Model model;
  if (!model.initString(robot_description)) {
    error_message = "无法解析 robot_description";
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    error_message = "无法从 URDF 构建 KDL Tree";
    return false;
  }

  KDL::Chain chain;
  if (!tree.getChain(config.root_link, config.tip_link, chain)) {
    error_message = "KDL 链不存在: " + config.root_link + " -> " + config.tip_link;
    return false;
  }
  if (chain.getNrOfJoints() == 0U) {
    error_message = "KDL 链不包含可动关节";
    return false;
  }

  std::vector<std::string> joint_names;
  std::vector<double> lower_limits;
  std::vector<double> upper_limits;
  joint_names.reserve(chain.getNrOfJoints());
  lower_limits.reserve(chain.getNrOfJoints());
  upper_limits.reserve(chain.getNrOfJoints());

  for (unsigned int segment_index = 0U; segment_index < chain.getNrOfSegments(); ++segment_index) {
    const KDL::Joint & kdl_joint = chain.getSegment(segment_index).getJoint();
    if (kdl_joint.getType() == KDL::Joint::None) {
      continue;
    }

    const auto urdf_joint = model.getJoint(kdl_joint.getName());
    if (!urdf_joint || !urdf_joint->limits) {
      error_message = "运动链关节缺少硬限位: " + kdl_joint.getName();
      return false;
    }
    const double lower = urdf_joint->limits->lower;
    const double upper = urdf_joint->limits->upper;
    if (!std::isfinite(lower) || !std::isfinite(upper) || upper - lower <= kMinimumJointRange) {
      error_message = "运动链关节限位非法: " + kdl_joint.getName();
      return false;
    }

    joint_names.push_back(kdl_joint.getName());
    lower_limits.push_back(lower);
    upper_limits.push_back(upper);
  }

  if (joint_names.size() != chain.getNrOfJoints()) {
    error_message = "KDL 关节数量与 URDF 限位数量不一致";
    return false;
  }

  impl_->config = config;
  impl_->chain = std::move(chain);
  impl_->joint_names = std::move(joint_names);
  impl_->lower_limits = std::move(lower_limits);
  impl_->upper_limits = std::move(upper_limits);
  impl_->fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(impl_->chain);
  impl_->jacobian_solver = std::make_unique<KDL::ChainJntToJacSolver>(impl_->chain);
  impl_->configured = true;
  error_message.clear();
  return true;
}

bool DlsIkSolver::is_configured() const noexcept
{
  return impl_->configured;
}

std::vector<std::string> DlsIkSolver::joint_names() const
{
  return impl_->joint_names;
}

ForwardKinematicsResult DlsIkSolver::compute_fk(
  const std::vector<double> & positions) const
{
  ForwardKinematicsResult result;
  if (!impl_->configured) {
    result.message = "IK 求解器尚未配置";
    return result;
  }
  if (positions.size() != impl_->joint_names.size() || !all_finite(positions)) {
    result.message = "FK 输入关节数量错误或包含非有限值";
    return result;
  }

  KDL::Frame frame;
  if (impl_->fk_solver->JntToCart(to_joint_array(positions), frame) < 0 ||
    !frame_is_finite(frame))
  {
    result.message = "KDL FK 计算失败";
    return result;
  }

  result.success = true;
  result.pose = to_cartesian_pose(frame);
  return result;
}

JacobianResult DlsIkSolver::compute_jacobian(
  const std::vector<double> & positions) const
{
  JacobianResult result;
  if (!impl_->configured) {
    result.message = "IK 求解器尚未配置";
    return result;
  }
  if (positions.size() != impl_->joint_names.size() || !all_finite(positions)) {
    result.message = "Jacobian 输入关节数量错误或包含非有限值";
    return result;
  }

  KDL::Jacobian jacobian{static_cast<unsigned int>(positions.size())};
  if (impl_->jacobian_solver->JntToJac(to_joint_array(positions), jacobian) < 0 ||
    !jacobian.data.allFinite())
  {
    result.message = "KDL Jacobian 计算失败";
    return result;
  }

  result.success = true;
  result.rows = kTaskDimension;
  result.columns = positions.size();
  result.values.reserve(result.rows * result.columns);
  for (std::size_t row = 0U; row < result.rows; ++row) {
    for (std::size_t column = 0U; column < result.columns; ++column) {
      result.values.push_back(
        jacobian.data(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)));
    }
  }
  return result;
}

IkSolveResult DlsIkSolver::solve(
  const CartesianPose & target,
  const std::vector<double> & seed) const
{
  IkSolveResult result;
  result.positions = seed;
  if (!impl_->configured) {
    result.message = "IK 求解器尚未配置";
    return result;
  }
  if (seed.size() != impl_->joint_names.size() || !all_finite(seed)) {
    result.message = "IK seed 数量错误或包含非有限值";
    return result;
  }

  KDL::Frame target_frame;
  if (!to_kdl_frame(target, target_frame)) {
    result.message = "IK 目标位姿非法";
    return result;
  }

  Eigen::VectorXd joints = Eigen::Map<const Eigen::VectorXd>(seed.data(), seed.size());
  const Eigen::Index joint_count = joints.size();
  for (Eigen::Index index = 0; index < joint_count; ++index) {
    joints(index) = std::clamp(
      joints(index),
      impl_->lower_limits[static_cast<std::size_t>(index)],
      impl_->upper_limits[static_cast<std::size_t>(index)]);
  }
  const Eigen::MatrixXd identity_task = Eigen::MatrixXd::Identity(kTaskDimension, kTaskDimension);
  const Eigen::MatrixXd identity_joint = Eigen::MatrixXd::Identity(joint_count, joint_count);

  for (std::size_t iteration = 0U; iteration < impl_->config.max_iterations; ++iteration) {
    std::vector<double> current_positions(
      joints.data(), joints.data() + static_cast<std::ptrdiff_t>(joint_count));
    const KDL::JntArray kdl_joints = to_joint_array(current_positions);

    KDL::Frame current_frame;
    KDL::Jacobian kdl_jacobian{static_cast<unsigned int>(joint_count)};
    if (impl_->fk_solver->JntToCart(kdl_joints, current_frame) < 0 ||
      impl_->jacobian_solver->JntToJac(kdl_joints, kdl_jacobian) < 0 ||
      !frame_is_finite(current_frame) || !kdl_jacobian.data.allFinite())
    {
      result.message = "IK 迭代中的 FK 或 Jacobian 计算失败";
      return result;
    }

    const KDL::Twist error = KDL::diff(current_frame, target_frame);
    result.position_error = error.vel.Norm();
    result.orientation_error = error.rot.Norm();
    result.iterations = iteration;
    if (!std::isfinite(result.position_error) || !std::isfinite(result.orientation_error)) {
      result.message = "IK 位姿误差包含非有限值";
      return result;
    }
    if (result.position_error <= impl_->config.position_tolerance &&
      result.orientation_error <= impl_->config.orientation_tolerance)
    {
      result.success = true;
      result.positions.assign(joints.data(), joints.data() + joint_count);
      result.message = "IK 收敛";
      return result;
    }

    Eigen::Matrix<double, 6, 1> task_error;
    task_error << error.vel.x(), error.vel.y(), error.vel.z(),
      error.rot.x(), error.rot.y(), error.rot.z();

    const Eigen::MatrixXd & jacobian = kdl_jacobian.data;
    Eigen::MatrixXd damped_system = jacobian * jacobian.transpose();
    damped_system.diagonal().array() += impl_->config.damping * impl_->config.damping;
    const Eigen::LDLT<Eigen::MatrixXd> decomposition{damped_system};
    if (decomposition.info() != Eigen::Success) {
      result.message = "DLS 阻尼矩阵分解失败";
      return result;
    }

    const Eigen::MatrixXd pseudo_inverse =
      jacobian.transpose() * decomposition.solve(identity_task);
    if (!pseudo_inverse.allFinite()) {
      result.message = "DLS 伪逆包含非有限值";
      return result;
    }

    Eigen::VectorXd centering_gradient{joint_count};
    for (Eigen::Index index = 0; index < joint_count; ++index) {
      const double lower = impl_->lower_limits[static_cast<std::size_t>(index)];
      const double upper = impl_->upper_limits[static_cast<std::size_t>(index)];
      const double midpoint = 0.5 * (lower + upper);
      centering_gradient(index) = (midpoint - joints(index)) / (upper - lower);
    }

    Eigen::VectorXd joint_delta = pseudo_inverse * task_error;
    joint_delta += impl_->config.joint_centering_gain *
      (identity_joint - pseudo_inverse * jacobian) * centering_gradient;
    if (!joint_delta.allFinite()) {
      result.message = "IK 关节增量包含非有限值";
      return result;
    }

    for (Eigen::Index index = 0; index < joint_count; ++index) {
      joint_delta(index) = std::clamp(
        joint_delta(index), -impl_->config.max_joint_step, impl_->config.max_joint_step);
      joints(index) = std::clamp(
        joints(index) + joint_delta(index),
        impl_->lower_limits[static_cast<std::size_t>(index)],
        impl_->upper_limits[static_cast<std::size_t>(index)]);
    }
  }

  // 不发布未收敛候选解，调用方保持该臂最后有效目标。
  result.iterations = impl_->config.max_iterations;
  result.message = "IK 达到最大迭代次数但未收敛";
  return result;
}

}  // namespace bw_kinematics
