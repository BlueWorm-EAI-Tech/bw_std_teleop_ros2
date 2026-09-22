#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "bw_teleop/manager/vr_stream_manager.hpp"

namespace bw_teleop::node
{

/**
 * @brief 基础遥操作 ROS IO 适配节点。
 */
class BwTeleop final : public rclcpp::Node
{
public:
  explicit BwTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~BwTeleop() override;

  BwTeleop(const BwTeleop &) = delete;
  BwTeleop & operator=(const BwTeleop &) = delete;

private:
  struct UdpConfig
  {
    std::string bind_ip;
    int port{12345};
    int poll_period_ms{2};
    std::size_t max_packets_per_cycle{64U};
  };

  struct Topics
  {
    std::string joint_states_input;
    std::string left_measured_pose_input;
    std::string right_measured_pose_input;
    std::string left_target_output;
    std::string right_target_output;
    std::string gripper_command_output;
    std::string lift_command_output;
    std::string base_command_output;
    std::string head_pose_output;
    // 遥操作控制活跃标志: 供底盘控制器映射到 safety/power, VR 断流后自动掉电。
    std::string control_active_output;
    // 关节空间复位触发(发给运动学节点)。
    std::string reset_request_output;
  };

  struct JointNames
  {
    std::string left_gripper;
    std::string right_gripper;
    std::string lift;
  };

  manager::VrStreamConfig load_stream_config();
  UdpConfig load_udp_config();
  Topics load_topics();
  JointNames load_joint_names();
  void open_socket(const UdpConfig & config);
  void create_publishers();
  void create_subscriptions();
  void create_timers(double publish_frequency_hz, int poll_period_ms);
  double steady_now_sec() noexcept;

  void receive_available_packets();
  void measured_pose_callback(
    manager::Side side, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_commands();
  void log_events(const manager::TickResult & result);

  static manager::Pose to_internal_pose(const geometry_msgs::msg::Pose & pose) noexcept;
  static geometry_msgs::msg::Pose to_ros_pose(const manager::Pose & pose) noexcept;

  Topics topics_;
  JointNames joint_names_;
  int socket_fd_{-1};
  std::size_t max_packets_per_cycle_{64U};
  std::string target_frame_id_;
  std::string base_frame_id_;
  std::unique_ptr<manager::VrStreamManager> manager_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lift_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr head_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_request_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_measured_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_measured_pose_sub_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace bw_teleop::node
