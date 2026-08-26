#include "bw_kinematics/node/kinematics_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

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

}  // namespace

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node{"kinematics_node", options}, manager_{std::make_unique<KinematicsManager>()}
{
  const std::string robot_description = declare_parameter<std::string>("robot_description", "");
  root_link_ = declare_parameter<std::string>("root_link", "C_Link");
  const std::string left_tip_link =
    declare_parameter<std::string>("left_tip_link", "A_left_Degree7_link");
  const std::string right_tip_link =
    declare_parameter<std::string>("right_tip_link", "A_right_Degree7_link");
  const int max_iterations = declare_parameter<int>("max_iterations", 40);
  const int joint_state_timeout_ms = declare_parameter<int>("joint_state_timeout_ms", 200);
  const double damping = require_positive("damping", declare_parameter<double>("damping", 0.05));
  const double max_joint_step =
    require_positive("max_joint_step", declare_parameter<double>("max_joint_step", 0.04));
  const double position_tolerance =
    require_positive("position_tolerance", declare_parameter<double>("position_tolerance", 0.001));
  const double orientation_tolerance = require_positive(
    "orientation_tolerance", declare_parameter<double>("orientation_tolerance", 0.008726646259971648));
  const double joint_centering_gain =
    declare_parameter<double>("joint_centering_gain", 0.02);
  trajectory_duration_sec_ = require_positive(
    "trajectory_duration_sec", declare_parameter<double>("trajectory_duration_sec", 0.08));

  if (max_iterations <= 0 || joint_state_timeout_ms <= 0) {
    throw std::invalid_argument("参数 max_iterations 和 joint_state_timeout_ms 必须大于 0");
  }
  if (!std::isfinite(joint_centering_gain) || joint_centering_gain < 0.0) {
    throw std::invalid_argument("参数 joint_centering_gain 必须为有限非负数");
  }

  DlsIkConfig shared_config;
  shared_config.root_link = root_link_;
  shared_config.max_iterations = static_cast<std::size_t>(max_iterations);
  shared_config.damping = damping;
  shared_config.max_joint_step = max_joint_step;
  shared_config.position_tolerance = position_tolerance;
  shared_config.orientation_tolerance = orientation_tolerance;
  shared_config.joint_centering_gain = joint_centering_gain;

  KinematicsManagerConfig manager_config;
  manager_config.feedback_timeout_sec = static_cast<double>(joint_state_timeout_ms) / 1000.0;
  manager_config.left_arm = shared_config;
  manager_config.left_arm.tip_link = left_tip_link;
  manager_config.right_arm = shared_config;
  manager_config.right_arm.tip_link = right_tip_link;

  std::string configure_error;
  if (!manager_->configure(robot_description, manager_config, configure_error)) {
    throw std::runtime_error("运动学 Manager 配置失败: " + configure_error);
  }

  const std::string joint_state_topic =
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
  const std::string left_target_topic =
    declare_parameter<std::string>("left_target_topic", "/target_left_pose");
  const std::string right_target_topic =
    declare_parameter<std::string>("right_target_topic", "/target_right_pose");
  const std::string left_current_pose_topic =
    declare_parameter<std::string>("left_current_pose_topic", "/current_left_pose");
  const std::string right_current_pose_topic =
    declare_parameter<std::string>("right_current_pose_topic", "/current_right_pose");
  const std::string trajectory_topic = declare_parameter<std::string>(
    "trajectory_topic", "/dual_arm_controller/joint_trajectory");

  left_current_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    left_current_pose_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable());
  right_current_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    right_current_pose_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable());
  trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
    trajectory_topic, rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable());
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic,
    rclcpp::SensorDataQoS{},
    std::bind(&KinematicsNode::joint_state_callback, this, std::placeholders::_1));
  left_target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    left_target_topic,
    rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      target_callback(ArmSide::left, message);
    });
  right_target_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    right_target_topic,
    rclcpp::QoS{rclcpp::KeepLast{1U}}.reliable(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
      target_callback(ArmSide::right, message);
    });

  RCLCPP_INFO(
    get_logger(), "KDL 双臂运动学节点已启动，运动链根链接: %s", root_link_.c_str());
}

void KinematicsNode::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message->name.size() != message->position.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "忽略 name/position 数量不一致的 JointState");
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
  CartesianPose target;
  if (!pose_message_to_target(*message, target)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "忽略坐标系错误、非有限或四元数无效的末端目标");
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
      "单臂 IK 失败，保持该臂最后有效目标: %s", command.message.c_str());
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
  return true;
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
  if (command.joint_names.size() != command.positions.size()) {
    RCLCPP_ERROR(get_logger(), "Manager 返回的关节名和位置数量不一致");
    return;
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.joint_names = command.joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = command.positions;
  point.time_from_start = rclcpp::Duration::from_seconds(trajectory_duration_sec_);
  trajectory.points.push_back(std::move(point));
  trajectory_publisher_->publish(std::move(trajectory));
}

}  // namespace bw_kinematics

RCLCPP_COMPONENTS_REGISTER_NODE(bw_kinematics::KinematicsNode)
