#include <chrono>
#include <memory>
#include <optional>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "bw_kinematics/node/kinematics_node.hpp"

namespace bw_kinematics
{
namespace
{

using namespace std::chrono_literals;

constexpr char kTwoArmUrdf[] = R"(
<robot name="two_arm_test">
  <link name="root"/>
  <link name="left_tip"/>
  <link name="right_tip"/>
  <joint name="left_joint" type="revolute">
    <parent link="root"/><child link="left_tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="right_joint" type="revolute">
    <parent link="root"/><child link="right_tip"/>
    <origin xyz="0 1 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>)";

class KinematicsNodeTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(KinematicsNodeTest, PublishesConfiguredMeasuredEndEffectorTopics)
{
  const auto options = rclcpp::NodeOptions{}.parameter_overrides({
    rclcpp::Parameter{"robot_description", kTwoArmUrdf},
    rclcpp::Parameter{"root_link", "root"},
    rclcpp::Parameter{"left_tip_link", "left_tip"},
    rclcpp::Parameter{"right_tip_link", "right_tip"},
    rclcpp::Parameter{"joint_state_topic", "/test/joint_states"},
    rclcpp::Parameter{"left_current_pose_topic", "/test/current_left_pose"},
    rclcpp::Parameter{"right_current_pose_topic", "/test/current_right_pose"},
  });
  const auto kinematics_node = std::make_shared<KinematicsNode>(options);
  const auto io_node = std::make_shared<rclcpp::Node>("kinematics_node_test_io");

  std::optional<geometry_msgs::msg::PoseStamped> left_pose;
  std::optional<geometry_msgs::msg::PoseStamped> right_pose;
  const auto left_subscription = io_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/test/current_left_pose", rclcpp::QoS{1U}.reliable(),
    [&left_pose](geometry_msgs::msg::PoseStamped::SharedPtr message) {
      left_pose = *message;
    });
  const auto right_subscription = io_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/test/current_right_pose", rclcpp::QoS{1U}.reliable(),
    [&right_pose](geometry_msgs::msg::PoseStamped::SharedPtr message) {
      right_pose = *message;
    });
  const auto joint_state_publisher = io_node->create_publisher<sensor_msgs::msg::JointState>(
    "/test/joint_states", rclcpp::SensorDataQoS{});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(kinematics_node);
  executor.add_node(io_node);

  sensor_msgs::msg::JointState state;
  state.name = {"right_joint", "left_joint"};
  state.position = {0.0, 0.0};
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while ((!left_pose.has_value() || !right_pose.has_value()) &&
    std::chrono::steady_clock::now() < deadline)
  {
    joint_state_publisher->publish(state);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(left_pose.has_value());
  ASSERT_TRUE(right_pose.has_value());
  EXPECT_EQ(left_pose->header.frame_id, "root");
  EXPECT_EQ(right_pose->header.frame_id, "root");
  EXPECT_DOUBLE_EQ(left_pose->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(left_pose->pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(right_pose->pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(right_pose->pose.position.y, 1.0);

  (void)left_subscription;
  (void)right_subscription;
}

}  // namespace
}  // namespace bw_kinematics
