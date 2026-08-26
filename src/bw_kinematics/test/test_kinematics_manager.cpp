#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "bw_kinematics/manager/kinematics_manager.hpp"

namespace bw_kinematics
{
namespace
{

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

KinematicsManager make_manager()
{
  KinematicsManager manager;
  KinematicsManagerConfig config;
  config.left_arm.root_link = "root";
  config.left_arm.tip_link = "left_tip";
  config.right_arm.root_link = "root";
  config.right_arm.tip_link = "right_tip";
  config.feedback_timeout_sec = 0.2;
  std::string error;
  EXPECT_TRUE(manager.configure(kTwoArmUrdf, config, error)) << error;
  return manager;
}

TEST(KinematicsManagerTest, RejectsMeasuredPoseUntilBothArmsAreComplete)
{
  auto manager = make_manager();

  EXPECT_FALSE(manager.measured_pose(ArmSide::left, 1.0).success);
  EXPECT_FALSE(manager.update_measured_state({"left_joint"}, {0.0}, 1.0));
  EXPECT_FALSE(manager.measured_pose(ArmSide::left, 1.0).success);
}

TEST(KinematicsManagerTest, ComputesCurrentEndEffectorPosesFromMeasuredJoints)
{
  auto manager = make_manager();
  ASSERT_TRUE(manager.update_measured_state(
    {"right_joint", "left_joint"}, {0.0, 0.0}, 1.0));

  const auto left = manager.measured_pose(ArmSide::left, 1.0);
  const auto right = manager.measured_pose(ArmSide::right, 1.0);

  ASSERT_TRUE(left.success) << left.message;
  ASSERT_TRUE(right.success) << right.message;
  EXPECT_DOUBLE_EQ(left.pose.position[0], 1.0);
  EXPECT_DOUBLE_EQ(left.pose.position[1], 0.0);
  EXPECT_DOUBLE_EQ(right.pose.position[0], 0.0);
  EXPECT_DOUBLE_EQ(right.pose.position[1], 1.0);
}

TEST(KinematicsManagerTest, InvalidatesReadinessOnIncompleteOrNonFiniteFrames)
{
  auto manager = make_manager();
  ASSERT_TRUE(manager.update_measured_state(
    {"right_joint", "left_joint"}, {0.0, 0.0}, 1.0));
  ASSERT_TRUE(manager.ready(1.0));

  EXPECT_FALSE(manager.update_measured_state({"left_joint"}, {0.1}, 1.1));
  EXPECT_FALSE(manager.ready(1.1));
  EXPECT_FALSE(manager.measured_pose(ArmSide::left, 1.1).success);

  EXPECT_FALSE(manager.update_measured_state(
    {"right_joint", "left_joint"},
    {0.0, std::numeric_limits<double>::quiet_NaN()}, 1.2));
  EXPECT_FALSE(manager.ready(1.2));
}

TEST(KinematicsManagerTest, RejectsFkAndIkAfterMeasuredFeedbackExpires)
{
  auto manager = make_manager();
  ASSERT_TRUE(manager.update_measured_state(
    {"right_joint", "left_joint"}, {0.0, 0.0}, 2.0));
  EXPECT_TRUE(manager.ready(2.19));
  EXPECT_FALSE(manager.ready(2.21));
  EXPECT_FALSE(manager.measured_pose(ArmSide::left, 2.21).success);

  CartesianPose target;
  target.position = {1.0, 0.0, 0.0};
  const auto command = manager.solve_target(ArmSide::left, target, 2.21);
  EXPECT_FALSE(command.has_command);
}

TEST(KinematicsManagerTest, ResynchronizesBothCommandsWhenFeedbackRecoversAfterTimeout)
{
  auto manager = make_manager();
  ASSERT_TRUE(manager.update_measured_state(
    {"right_joint", "left_joint"}, {0.0, 0.0}, 3.0));

  CartesianPose rotated_target;
  rotated_target.position = {1.0, 0.0, 0.0};
  rotated_target.orientation_xyzw = {0.0, 0.0, std::sin(0.2), std::cos(0.2)};
  const auto moved = manager.solve_target(ArmSide::left, rotated_target, 3.0);
  ASSERT_TRUE(moved.requested_arm_succeeded) << moved.message;
  ASSERT_NEAR(moved.positions.front(), 0.4, 0.02);

  ASSERT_TRUE(manager.update_measured_state(
    {"right_joint", "left_joint"}, {0.25, 0.25}, 4.0));
  CartesianPose unreachable_target;
  unreachable_target.position = {100.0, 0.0, 0.0};
  const auto held = manager.solve_target(ArmSide::left, unreachable_target, 4.0);

  ASSERT_TRUE(held.has_command);
  EXPECT_FALSE(held.requested_arm_succeeded);
  ASSERT_EQ(held.positions.size(), 2U);
  EXPECT_DOUBLE_EQ(held.positions[0], 0.25);
  EXPECT_DOUBLE_EQ(held.positions[1], 0.25);
}

}  // namespace
}  // namespace bw_kinematics
