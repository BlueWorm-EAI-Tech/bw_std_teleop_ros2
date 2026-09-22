#include "bw_kinematics/node/kinematics_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace bw_kinematics
{
namespace
{

constexpr double kMinimumQuaternionNorm{1.0e-12};

double steady_now_sec() noexcept
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

double require_positive(const std::string & name, const double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument("参数 " + name + " 必须为有限正数");
  }
  return value;
}

std::string read_file(const std::string & path)
{
  std::ifstream stream{path};
  if (!stream.is_open()) {
    throw std::runtime_error("无法读取 Standard IK URDF: " + path);
  }
  return {std::istreambuf_iterator<char>{stream}, std::istreambuf_iterator<char>{}};
}

void validate_topic_name(
  const rclcpp::Node * node, const std::string & parameter_name,
  const std::string & topic_name)
{
  if (topic_name.empty()) {
    throw std::invalid_argument(parameter_name + " 不能为空");
  }
  try {
    static_cast<void>(rclcpp::expand_topic_or_service_name(
      topic_name, node->get_name(), node->get_namespace()));
  } catch (const std::exception & error) {
    throw std::invalid_argument(
            parameter_name + " 不是合法 ROS topic: " + error.what());
  }
}

}  // namespace

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node{"kinematics_node", options}, manager_{std::make_unique<KinematicsManager>()}
{
  root_link_ = declare_parameter<std::string>("root_link", std::string{kStandardRootFrame});
  const std::string left_tip_link = declare_parameter<std::string>(
    "left_tip_link", std::string{kStandardLeftEndEffectorFrame});
  const std::string right_tip_link = declare_parameter<std::string>(
    "right_tip_link", std::string{kStandardRightEndEffectorFrame});
  const std::string locked_lift_joint = declare_parameter<std::string>(
    "locked_lift_joint", std::string{kStandardLiftJointName});
  const std::string model_path = declare_parameter<std::string>("ik_model_path", "");
  const int max_iterations = declare_parameter<int>("max_iterations", 30);
  const int joint_state_timeout_ms = declare_parameter<int>("joint_state_timeout_ms", 200);
  const double continuity_weight = declare_parameter<double>("continuity_weight", 0.1);
  const double max_position_residual = require_positive(
    "max_position_residual", declare_parameter<double>("max_position_residual", 0.03));
  const double max_orientation_residual = require_positive(
    "max_orientation_residual", declare_parameter<double>("max_orientation_residual", 0.25));
  const double max_joint_step_rad = require_positive(
    "max_joint_step_rad", declare_parameter<double>("max_joint_step_rad", 0.35));
  trajectory_max_velocity_rad_s_ = require_positive(
    "trajectory_max_velocity_rad_s",
    declare_parameter<double>("trajectory_max_velocity_rad_s", 3.0));
  trajectory_max_acceleration_rad_s2_ = require_positive(
    "trajectory_max_acceleration_rad_s2",
    declare_parameter<double>("trajectory_max_acceleration_rad_s2", 20.0));
  tracking_rate_hz_ = require_positive(
    "tracking_rate_hz", declare_parameter<double>("tracking_rate_hz", 200.0));
  reset_duration_sec_ = require_positive(
    "reset_duration_sec", declare_parameter<double>("reset_duration_sec", 3.0));
  const std::string reset_topic = declare_parameter<std::string>(
    "reset_topic", "/kinematics/reset_arms");
  validate_topic_name(this, "reset_topic", reset_topic);

  if (root_link_ != kStandardRootFrame ||
    left_tip_link != kStandardLeftEndEffectorFrame ||
    right_tip_link != kStandardRightEndEffectorFrame ||
    locked_lift_joint != kStandardLiftJointName || max_iterations <= 0 ||
    joint_state_timeout_ms <= 0 || !std::isfinite(continuity_weight) || continuity_weight < 0.0)
  {
    throw std::invalid_argument("Standard 运动学参数必须匹配固定模型契约");
  }

  TrajectorySmootherConfig smoother_config;
  smoother_config.dof = kStandardArmDof;
  smoother_config.period_sec = 1.0 / tracking_rate_hz_;
  smoother_config.max_velocity_rad_s = trajectory_max_velocity_rad_s_;
  smoother_config.max_acceleration_rad_s2 = trajectory_max_acceleration_rad_s2_;
  smoother_config.max_jerk_rad_s3 =
    declare_parameter<double>("trajectory_max_jerk_rad_s3", 200.0);
  smoother_config.target_velocity_scale =
    declare_parameter<double>("target_velocity_scale", 0.5);
  smoother_config.max_target_velocity_rad_s =
    declare_parameter<double>("max_target_velocity_rad_s", 3.0);
  smoother_config.target_velocity_filter_alpha =
    declare_parameter<double>("target_velocity_filter_alpha", 0.15);
  smoother_config.target_prediction_max_time_sec =
    declare_parameter<double>("target_prediction_max_time_sec", 0.02);

  smoother_ = std::make_unique<StandardTrajectorySmoother>();
  std::string smoother_error;
  if (!smoother_->init(smoother_config, smoother_error)) {
    throw std::invalid_argument("Standard 轨迹平滑器配置失败: " + smoother_error);
  }

  std::string resolved_model_path = model_path;
  if (resolved_model_path.empty()) {
    resolved_model_path =
      ament_index_cpp::get_package_share_directory("bw_kinematics") +
      "/assets/standard_ik/standard_ik.urdf";
  }
  const std::string standard_ik_urdf_xml = read_file(resolved_model_path);

  KinematicsManagerConfig manager_config;
  manager_config.feedback_timeout_sec = static_cast<double>(joint_state_timeout_ms) / 1000.0;
  manager_config.max_joint_step_rad = max_joint_step_rad;
  manager_config.ik.root_frame = root_link_;
  manager_config.ik.left_end_effector_frame = left_tip_link;
  manager_config.ik.right_end_effector_frame = right_tip_link;
  manager_config.ik.locked_lift_joint = locked_lift_joint;
  manager_config.ik.max_iterations = static_cast<std::size_t>(max_iterations);
  manager_config.ik.continuity_weight = continuity_weight;
  manager_config.ik.max_position_residual = max_position_residual;
  manager_config.ik.max_orientation_residual = max_orientation_residual;

  std::string configure_error;
  if (!manager_->configure(standard_ik_urdf_xml, manager_config, configure_error)) {
    throw std::runtime_error("Standard 运动学 Manager 配置失败: " + configure_error);
  }

  const std::string joint_state_topic = declare_parameter<std::string>(
    "joint_state_topic", "/joint_states");
  const std::string left_target_topic = declare_parameter<std::string>(
    "left_target_topic", "/target_left_pose");
  const std::string right_target_topic = declare_parameter<std::string>(
    "right_target_topic", "/target_right_pose");
  const std::string left_current_pose_topic = declare_parameter<std::string>(
    "left_current_pose_topic", "/current_left_pose");
  const std::string right_current_pose_topic = declare_parameter<std::string>(
    "right_current_pose_topic", "/current_right_pose");
  const std::string trajectory_topic = declare_parameter<std::string>(
    "trajectory_topic", "/dual_arm_controller/joint_trajectory");
  validate_topic_name(this, "joint_state_topic", joint_state_topic);
  validate_topic_name(this, "left_target_topic", left_target_topic);
  validate_topic_name(this, "right_target_topic", right_target_topic);
  validate_topic_name(this, "left_current_pose_topic", left_current_pose_topic);
  validate_topic_name(this, "right_current_pose_topic", right_current_pose_topic);
  validate_topic_name(this, "trajectory_topic", trajectory_topic);

  left_current_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    left_current_pose_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable());
  right_current_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    right_current_pose_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable());
  trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    trajectory_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable());
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic, rclcpp::SensorDataQoS{},
    std::bind(&KinematicsNode::joint_state_callback, this, std::placeholders::_1));
  left_target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    left_target_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      target_callback(ArmSide::left, message);
    });
  right_target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    right_target_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      target_callback(ArmSide::right, message);
    });
  reset_subscription_ = create_subscription<std_msgs::msg::Bool>(
    reset_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable(),
    std::bind(&KinematicsNode::reset_callback, this, std::placeholders::_1));
  tracking_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / tracking_rate_hz_),
    std::bind(&KinematicsNode::publish_tracking_step, this));

  RCLCPP_INFO(
    get_logger(), "Standard Pinocchio/CasADi 双臂运动学节点已启动, root=%s",
    root_link_.c_str());
}

void KinematicsNode::reset_callback(const std_msgs::msg::Bool::SharedPtr message)
{
  if (message == nullptr || !message->data) {
    return;
  }
  const double now_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  publish_joint_space_reset(now_sec);
}

void KinematicsNode::publish_joint_space_reset(const double now_sec)
{
  std::lock_guard<std::mutex> lock(manager_mutex_);
  std::vector<double> measured;
  if (!manager_->copy_measured_positions(measured, now_sec)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "关节空间复位被跳过: 14 轴反馈尚未就绪或已超时");
    return;
  }
  constexpr std::size_t kSampleCount = 60U;
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.joint_names.reserve(kStandardArmJointNames.size());
  for (const auto name : kStandardArmJointNames) {
    trajectory.joint_names.emplace_back(name);
  }
  trajectory.points.reserve(kSampleCount);
  for (std::size_t index = 1U; index <= kSampleCount; ++index) {
    const double scale = static_cast<double>(index) / static_cast<double>(kSampleCount);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(measured.size());
    for (const double value : measured) {
      point.positions.push_back(value * (1.0 - scale));
    }
    point.time_from_start = rclcpp::Duration::from_seconds(reset_duration_sec_ * scale);
    trajectory.points.push_back(std::move(point));
  }
  trajectory_publisher_->publish(std::move(trajectory));
  // 复位后把平滑器状态对齐到位零, 避免平滑器把手臂拉回旧目标。
  const std::vector<double> reset_positions(kStandardArmDof, 0.0);
  std::string smoother_error;
  if (!smoother_->reset(reset_positions, smoother_error)) {
    RCLCPP_ERROR(get_logger(), "轨迹平滑器复位失败: %s", smoother_error.c_str());
    return;
  }
  RCLCPP_INFO(
    get_logger(), "关节空间复位轨迹已下发: %zu 点 / %.1f s",
    kSampleCount, reset_duration_sec_);
}

void KinematicsNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message == nullptr) {
    return;
  }
  if (message->name.size() != message->position.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "忽略 name/position 数量不一致的 JointState");
    return;
  }

  ForwardKinematicsResult left;
  ForwardKinematicsResult right;
  const double now_sec = steady_now_sec();
  {
    std::lock_guard<std::mutex> lock{manager_mutex_};
    if (!manager_->update_measured_state(message->name, message->position, now_sec)) {
      return;
    }
    left = manager_->measured_pose(ArmSide::left, now_sec);
    right = manager_->measured_pose(ArmSide::right, now_sec);
  }

  if (left.success && right.success) {
    publish_measured_poses(left, right);
  }
}

void KinematicsNode::target_callback(
  const ArmSide side,
  const geometry_msgs::msg::PoseStamped::SharedPtr message)
{
  if (message == nullptr) {
    return;
  }
  CartesianPose target;
  if (!pose_message_to_target(*message, target)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "忽略坐标系错误、非有限或四元数无效的 Standard 末端目标");
    return;
  }

  KinematicsCommand command;
  {
    std::lock_guard<std::mutex> lock{manager_mutex_};
    command = manager_->solve_target(side, target, steady_now_sec());
  }

  if (!command.has_command) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "%s", command.message.c_str());
    return;
  }
  if (!command.requested_arm_succeeded) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Standard 双臂 IK 未通过门禁, 保持安全目标: %s", command.message.c_str());
  }
  publish_command(command);
}

bool KinematicsNode::pose_message_to_target(
  const geometry_msgs::msg::PoseStamped & message,
  CartesianPose & target) const
{
  if (!message.header.frame_id.empty() && message.header.frame_id != root_link_) {
    return false;
  }

  const auto & position = message.pose.position;
  const auto & orientation = message.pose.orientation;
  const double quaternion_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
    !std::isfinite(quaternion_norm) || quaternion_norm < kMinimumQuaternionNorm)
  {
    return false;
  }

  target.position = {position.x, position.y, position.z};
  const double inverse_norm = 1.0 / quaternion_norm;
  target.orientation_xyzw = {
    orientation.x * inverse_norm, orientation.y * inverse_norm,
    orientation.z * inverse_norm, orientation.w * inverse_norm};
  return is_valid_pose(target);
}

void KinematicsNode::publish_measured_poses(
  const ForwardKinematicsResult & left,
  const ForwardKinematicsResult & right)
{
  const auto stamp = now();
  const auto make_message = [this, &stamp](const CartesianPose & pose) {
      geometry_msgs::msg::PoseStamped message;
      message.header.stamp = stamp;
      message.header.frame_id = root_link_;
      message.pose.position.x = pose.position[0];
      message.pose.position.y = pose.position[1];
      message.pose.position.z = pose.position[2];
      message.pose.orientation.x = pose.orientation_xyzw[0];
      message.pose.orientation.y = pose.orientation_xyzw[1];
      message.pose.orientation.z = pose.orientation_xyzw[2];
      message.pose.orientation.w = pose.orientation_xyzw[3];
      return message;
    };

  left_current_pose_publisher_->publish(make_message(left.pose));
  right_current_pose_publisher_->publish(make_message(right.pose));
}

void KinematicsNode::publish_command(const KinematicsCommand & command)
{
  if (command.joint_names.size() != command.positions.size() ||
    command.positions.size() != kStandardArmDof)
  {
    RCLCPP_ERROR(get_logger(), "Standard IK 返回的 14 轴关节契约不完整");
    return;
  }
  std::string smoother_error;
  if (!smoother_->is_tracking()) {
    std::vector<double> measured;
    bool has_measured = false;
    {
      std::lock_guard<std::mutex> lock{manager_mutex_};
      has_measured = manager_->copy_measured_positions(measured, steady_now_sec());
    }
    const std::vector<double> & initial = has_measured ? measured : command.positions;
    if (!smoother_->reset(initial, smoother_error)) {
      RCLCPP_ERROR(get_logger(), "轨迹平滑器初始化失败: %s", smoother_error.c_str());
      return;
    }
  }
  if (!smoother_->update_target(command.positions, smoother_error)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "轨迹平滑器拒绝新目标: %s",
      smoother_error.c_str());
  }
}

void KinematicsNode::publish_tracking_step()
{
  const TrajectorySmootherState state = smoother_->step();
  if (!state.valid || state.positions.size() != kStandardArmDof) {
    return;
  }
  std::vector<double> clamped = state.positions;
  {
    std::lock_guard<std::mutex> lock{manager_mutex_};
    if (!manager_->clamp_to_limits(clamped)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "平滑器输出无法钳制到模型限位, 丢弃本帧轨迹");
      return;
    }
  }
  const double period = 1.0 / tracking_rate_hz_;
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.joint_names.reserve(kStandardArmJointNames.size());
  for (const auto name : kStandardArmJointNames) {
    trajectory.joint_names.emplace_back(name);
  }
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = std::move(clamped);
  point.time_from_start = rclcpp::Duration::from_seconds(period);
  trajectory.points.push_back(std::move(point));
  trajectory_publisher_->publish(std::move(trajectory));
}

}  // namespace bw_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(bw_kinematics::KinematicsNode)
