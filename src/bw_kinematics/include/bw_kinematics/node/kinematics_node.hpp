#ifndef BW_KINEMATICS__NODE__KINEMATICS_NODE_HPP_
#define BW_KINEMATICS__NODE__KINEMATICS_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "bw_kinematics/algorithm/standard_trajectory_smoother.hpp"
#include "bw_kinematics/manager/kinematics_manager.hpp"

namespace bw_kinematics
{

/**
 * @brief Standard 双臂运动学 ROS 适配节点.
 *
 * Node 只负责参数、ROS 消息转换和 JointTrajectory 发布.
 */
class KinematicsNode : public rclcpp::Node
{
public:
  explicit KinematicsNode(const rclcpp::NodeOptions & options);

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void target_callback(
    ArmSide side, const geometry_msgs::msg::PoseStamped::SharedPtr message);
  [[nodiscard]] bool pose_message_to_target(
    const geometry_msgs::msg::PoseStamped & message,
    CartesianPose & target) const;
  void publish_measured_poses(
    const ForwardKinematicsResult & left,
    const ForwardKinematicsResult & right);
  void publish_command(const KinematicsCommand & command);
  void publish_tracking_step();
  void reset_callback(const std_msgs::msg::Bool::SharedPtr message);
  void publish_joint_space_reset(double now_sec);

  std::unique_ptr<KinematicsManager> manager_;
  std::unique_ptr<StandardTrajectorySmoother> smoother_;
  std::mutex manager_mutex_;
  std::string root_link_{std::string{kStandardRootFrame}};
  double trajectory_max_velocity_rad_s_{3.0};
  double trajectory_max_acceleration_rad_s2_{20.0};
  double tracking_rate_hz_{200.0};
  rclcpp::TimerBase::SharedPtr tracking_timer_;
  double reset_duration_sec_{3.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_current_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_current_pose_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
};

}  // namespace bw_kinematics

#endif  // BW_KINEMATICS__NODE__KINEMATICS_NODE_HPP_
