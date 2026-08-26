#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "bw_kinematics/manager/kinematics_manager.hpp"

namespace bw_kinematics
{

/**
 * @brief 双臂运动学 ROS 适配节点，仅负责参数、消息转换和轨迹发布。
 */
class KinematicsNode : public rclcpp::Node
{
public:
  explicit KinematicsNode(const rclcpp::NodeOptions & options);

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void target_callback(
    ArmSide side,
    const geometry_msgs::msg::PoseStamped::SharedPtr message);
  [[nodiscard]] bool pose_message_to_target(
    const geometry_msgs::msg::PoseStamped & message,
    CartesianPose & target) const;
  void publish_measured_poses(
    const ForwardKinematicsResult & left,
    const ForwardKinematicsResult & right);
  void publish_command(const KinematicsCommand & command);

  std::unique_ptr<KinematicsManager> manager_;
  std::mutex manager_mutex_;
  std::string root_link_;
  double trajectory_duration_sec_{0.08};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_current_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_current_pose_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
};

}  // namespace bw_kinematics
