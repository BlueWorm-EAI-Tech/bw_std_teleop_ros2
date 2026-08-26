#include <cmath>
#include <limits>
#include <string_view>

#include <gtest/gtest.h>

#include "bw_std_control/standard_mapping.hpp"

namespace bw_std_control
{
namespace
{

constexpr std::size_t index(const JointIndex joint) noexcept
{
  return static_cast<std::size_t>(joint);
}

TEST(StandardMapping, DefinesExactlySeventeenActiveJoints)
{
  static_assert(kStandardJointCount == 17U);
  EXPECT_EQ(kStandardJointNames[index(JointIndex::lift)], "C_joint");
  EXPECT_EQ(kStandardJointNames[index(JointIndex::left_degree1)], "A_left_Degree1_joint");
  EXPECT_EQ(kStandardJointNames[index(JointIndex::left_gripper)], "A_left_Degree8_joint");
  EXPECT_EQ(kStandardJointNames[index(JointIndex::right_degree1)], "A_right_Degree1_joint");
  EXPECT_EQ(kStandardJointNames[index(JointIndex::right_gripper)], "A_right_Degree8_joint");
}

TEST(StandardMapping, DecodesLiftArmsAndGrippersUsingStandardCalibration)
{
  V3FeedbackPayload feedback{};
  feedback.pelvis_height = -98.1825027466F;
  feedback.pelvis_velocity = 100.0F;
  for (std::size_t joint = 0; joint < 7U; ++joint) {
    feedback.left_joint_position[joint] = static_cast<float>(joint + 1U);
    feedback.right_joint_position[joint] = static_cast<float>(joint + 1U);
  }
  feedback.left_joint_position[7] = 0.0F;
  feedback.right_joint_position[7] = 1.0F;
  feedback.left_joint_velocity[7] = 0.4F;
  feedback.right_joint_velocity[7] = -0.5F;
  MappingParameters parameters{};
  parameters.left_arm_raw_zero_rad[3] = 0.5;
  parameters.right_arm_raw_zero_rad[3] = 0.25;

  StandardState state{};
  ASSERT_TRUE(decode_standard_state(feedback, parameters, state));

  EXPECT_NEAR(state.position[index(JointIndex::lift)], -0.0981825027466, 1e-9);
  EXPECT_DOUBLE_EQ(state.velocity[index(JointIndex::lift)], 0.1);
  EXPECT_DOUBLE_EQ(state.position[index(JointIndex::left_degree1)], 1.0);
  EXPECT_DOUBLE_EQ(state.position[index(JointIndex::right_degree1)], 1.0);
  EXPECT_DOUBLE_EQ(state.position[index(JointIndex::left_degree4)], 3.5);
  EXPECT_DOUBLE_EQ(state.position[index(JointIndex::right_degree4)], 3.75);
  EXPECT_DOUBLE_EQ(state.position[index(JointIndex::left_gripper)], 0.0);
  EXPECT_NEAR(state.position[index(JointIndex::right_gripper)], 0.04965, 1e-9);
  EXPECT_NEAR(state.velocity[index(JointIndex::left_gripper)], 0.4 * 0.04965, 1e-9);
  EXPECT_NEAR(state.velocity[index(JointIndex::right_gripper)], -0.5 * 0.04965, 1e-9);
}

TEST(StandardMapping, EncodesRoundTripAndNeverCommandsHeadMotion)
{
  StandardCommand command{};
  command.position[index(JointIndex::lift)] = 0.0;
  command.position[index(JointIndex::left_degree1)] = 0.2;
  command.position[index(JointIndex::right_degree1)] = -0.3;
  command.position[index(JointIndex::left_gripper)] = 0.0;
  command.position[index(JointIndex::right_gripper)] = 0.04965;
  command.base_velocity = {0.1, -0.2, 0.3};
  const HeadHold head_hold{0.4F, -0.5F, 0.6F};

  V3CommandPayload payload{};
  ASSERT_TRUE(encode_standard_command(command, MappingParameters{}, head_hold, true, payload));

  EXPECT_EQ(payload.control_flag, 1U);
  EXPECT_FLOAT_EQ(payload.pelvis_height, 0.0F);
  EXPECT_FLOAT_EQ(payload.left_joint_position[0], 0.2F);
  EXPECT_FLOAT_EQ(payload.right_joint_position[0], -0.3F);
  EXPECT_FLOAT_EQ(payload.left_joint_position[7], 0.0F);
  EXPECT_FLOAT_EQ(payload.right_joint_position[7], 1.0F);
  EXPECT_FLOAT_EQ(payload.waist_position, head_hold.waist_position);
  EXPECT_FLOAT_EQ(payload.head_yaw_position, head_hold.head_yaw_position);
  EXPECT_FLOAT_EQ(payload.head_pitch_position, head_hold.head_pitch_position);
  EXPECT_FLOAT_EQ(payload.waist_max_velocity, 0.0F);
  EXPECT_FLOAT_EQ(payload.head_yaw_max_velocity, 0.0F);
  EXPECT_FLOAT_EQ(payload.head_pitch_max_velocity, 0.0F);
}

TEST(StandardMapping, RejectsNonFiniteCommandWithoutChangingOutput)
{
  StandardCommand command{};
  command.position[0] = std::numeric_limits<double>::quiet_NaN();
  V3CommandPayload payload{};
  payload.control_flag = 0x7FU;

  EXPECT_FALSE(
    encode_standard_command(
      command, MappingParameters{}, HeadHold{}, true, payload));
  EXPECT_EQ(payload.control_flag, 0x7FU);
}

TEST(StandardMapping, CoversAllArmPositionVelocityEffortAndInverseDirections)
{
  constexpr std::array<double, 7> left_direction{-1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0};
  constexpr std::array<double, 7> right_direction{-1.0, -1.0, 1.0, 1.0, -1.0, -1.0, -1.0};
  constexpr std::array<std::size_t, 7> left_motor_indices{6U, 5U, 4U, 3U, 2U, 1U, 0U};
  constexpr std::array<std::size_t, 7> right_motor_indices{1U, 0U, 3U, 2U, 5U, 4U, 6U};
  V3FeedbackPayload feedback{};
  feedback.pelvis_height = -98.1825027466F;
  feedback.waist_position = 0.1F;
  feedback.head_yaw_position = 0.2F;
  feedback.head_pitch_position = 0.3F;
  MappingParameters parameters{};
  parameters.left_arm_motor_indices = left_motor_indices;
  parameters.right_arm_motor_indices = right_motor_indices;
  parameters.left_arm_direction = left_direction;
  parameters.right_arm_direction = right_direction;
  parameters.left_arm_raw_zero_rad = {0.05, -0.1, 0.15, 0.25, -0.3, 0.35, -0.4};
  parameters.right_arm_raw_zero_rad = {-0.05, 0.1, -0.15, -0.5, 0.3, -0.35, 0.4};
  for (std::size_t joint = 0; joint < 7U; ++joint) {
    feedback.left_joint_position[joint] = static_cast<float>(0.1 * (joint + 1U));
    feedback.right_joint_position[joint] = static_cast<float>(-0.2 * (joint + 1U));
    feedback.left_joint_velocity[joint] = static_cast<float>(1.0 + joint);
    feedback.right_joint_velocity[joint] = static_cast<float>(2.0 + joint);
    feedback.left_joint_torque[joint] = static_cast<float>(3.0 + joint);
    feedback.right_joint_torque[joint] = static_cast<float>(4.0 + joint);
  }

  StandardState state{};
  HeadHold head_hold{};
  ASSERT_TRUE(decode_complete_feedback(feedback, parameters, state, head_hold));
  for (std::size_t joint = 0; joint < 7U; ++joint) {
    const auto left = index(JointIndex::left_degree1) + joint;
    const auto right = index(JointIndex::right_degree1) + joint;
    EXPECT_NEAR(
      state.position[left],
      left_direction[joint] *
      (feedback.left_joint_position[left_motor_indices[joint]] -
      parameters.left_arm_raw_zero_rad[joint]), 1e-7);
    EXPECT_NEAR(
      state.position[right],
      right_direction[joint] *
      (feedback.right_joint_position[right_motor_indices[joint]] -
      parameters.right_arm_raw_zero_rad[joint]), 1e-7);
    EXPECT_NEAR(
      state.velocity[left],
      left_direction[joint] * feedback.left_joint_velocity[left_motor_indices[joint]], 1e-7);
    EXPECT_NEAR(
      state.velocity[right],
      right_direction[joint] * feedback.right_joint_velocity[right_motor_indices[joint]], 1e-7);
    EXPECT_NEAR(
      state.effort[left],
      left_direction[joint] * feedback.left_joint_torque[left_motor_indices[joint]], 1e-7);
    EXPECT_NEAR(
      state.effort[right],
      right_direction[joint] * feedback.right_joint_torque[right_motor_indices[joint]], 1e-7);
  }

  StandardCommand command{};
  command.position = state.position;
  V3CommandPayload encoded{};
  ASSERT_TRUE(encode_standard_command(command, parameters, head_hold, true, encoded));
  for (std::size_t joint = 0; joint < 7U; ++joint) {
    EXPECT_NEAR(
      encoded.left_joint_position[left_motor_indices[joint]],
      feedback.left_joint_position[left_motor_indices[joint]], 1e-6);
    EXPECT_NEAR(
      encoded.right_joint_position[right_motor_indices[joint]],
      feedback.right_joint_position[right_motor_indices[joint]], 1e-6);
  }
}

TEST(StandardMapping, RejectsInvalidMotorPermutationOrDirectionWithoutChangingOutput)
{
  V3FeedbackPayload feedback{};
  MappingParameters parameters{};
  StandardState state{};
  state.position.fill(7.0);

  parameters.right_arm_motor_indices = {0U, 1U, 2U, 3U, 4U, 5U, 5U};
  EXPECT_FALSE(decode_standard_state(feedback, parameters, state));
  EXPECT_DOUBLE_EQ(state.position[0], 7.0);

  parameters.right_arm_motor_indices = {0U, 1U, 2U, 3U, 4U, 5U, 7U};
  EXPECT_FALSE(decode_standard_state(feedback, parameters, state));

  parameters.right_arm_motor_indices = {0U, 1U, 2U, 3U, 4U, 5U, 6U};
  parameters.right_arm_direction[4] = 0.0;
  EXPECT_FALSE(decode_standard_state(feedback, parameters, state));

  parameters.right_arm_direction[4] = 0.5;
  EXPECT_FALSE(decode_standard_state(feedback, parameters, state));
}

TEST(StandardMapping, MapsStandardLiftMillimetresDirectlyToSourceUrdfMetres)
{
  MappingParameters parameters{};
  parameters.left_arm_raw_zero_rad = {
    0.1514402479, 0.0107447943, -0.1020239592, 1.3941645622,
    1.7408245802, 0.2749960124, -0.0367294028};
  parameters.right_arm_raw_zero_rad = {
    0.1449110061, 0.0034568873, 0.1173603907, 1.3947396278,
    2.9670596123, 0.7853981853, -1.5707963705};

  V3FeedbackPayload feedback{};
  feedback.pelvis_height = 0.0F;
  // 按用户 JointState 的关节名称还原为 V3 Degree1..Degree8 协议顺序。
  feedback.left_joint_position = {
    0.1514402479F, 0.0107447943F, -0.1020239592F, 1.3941645622F,
    1.7408245802F, 0.2749960124F, -0.0367294028F, 1.0F};
  feedback.right_joint_position = {
    0.1449110061F, 0.0034568873F, 0.1173603907F, 1.3947396278F,
    2.9670596123F, 0.7853981853F, -1.5707963705F, 1.0F};

  StandardState state{};
  ASSERT_TRUE(decode_standard_state(feedback, parameters, state));
  EXPECT_NEAR(state.position[index(JointIndex::lift)], 0.0, 1e-6);
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
    EXPECT_NEAR(state.position[index(JointIndex::left_degree1) + joint], 0.0, 1e-6);
    EXPECT_NEAR(state.position[index(JointIndex::right_degree1) + joint], 0.0, 1e-6);
  }

  StandardCommand command{};
  command.position = state.position;
  V3CommandPayload encoded{};
  ASSERT_TRUE(encode_standard_command(command, parameters, HeadHold{}, false, encoded));
  EXPECT_NEAR(encoded.pelvis_height, feedback.pelvis_height, 1e-5);
  for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
    EXPECT_NEAR(encoded.left_joint_position[joint], feedback.left_joint_position[joint], 1e-6);
    EXPECT_NEAR(encoded.right_joint_position[joint], feedback.right_joint_position[joint], 1e-6);
  }
}

TEST(StandardMapping, RejectsIncompleteInitialHeadFeedbackWithoutChangingOutputs)
{
  V3FeedbackPayload feedback{};
  feedback.pelvis_height = -98.1825027466F;
  feedback.waist_position = std::numeric_limits<float>::quiet_NaN();
  StandardState state{};
  state.position.fill(7.0);
  HeadHold head_hold{1.0F, 2.0F, 3.0F};

  EXPECT_FALSE(
    decode_complete_feedback(
      feedback, MappingParameters{}, state, head_hold));
  EXPECT_DOUBLE_EQ(state.position[0], 7.0);
  EXPECT_FLOAT_EQ(head_hold.waist_position, 1.0F);
}

TEST(StandardMapping, RejectsDoubleOutsideFloatRangeAndOversizedParameters)
{
  StandardCommand command{};
  command.position[index(JointIndex::left_degree1)] = 1e308;
  V3CommandPayload payload{};
  payload.control_flag = 0x7FU;
  EXPECT_FALSE(
    encode_standard_command(
      command, MappingParameters{}, HeadHold{}, true, payload));
  EXPECT_EQ(payload.control_flag, 0x7FU);

  command = {};
  command.base_velocity[0] = 1e308;
  EXPECT_FALSE(
    encode_standard_command(
      command, MappingParameters{}, HeadHold{}, true, payload));

  command = {};
  command.position[index(JointIndex::lift)] = 1e308;
  EXPECT_FALSE(
    encode_standard_command(
      command, MappingParameters{}, HeadHold{}, true, payload));

  MappingParameters parameters{};
  parameters.base_max_acceleration_x = 1e308;
  EXPECT_FALSE(encode_standard_command(command = {}, parameters, HeadHold{}, true, payload));
  parameters = {};
  parameters.pelvis_max_velocity_mm_s = 1e308;
  EXPECT_FALSE(encode_standard_command(command, parameters, HeadHold{}, true, payload));
  parameters = {};
  parameters.arm_max_velocity_rad_s = 1e308;
  EXPECT_FALSE(encode_standard_command(command, parameters, HeadHold{}, true, payload));
  parameters = {};
  parameters.gripper_max_velocity_normalized_s = 1e308;
  EXPECT_FALSE(encode_standard_command(command, parameters, HeadHold{}, true, payload));
  parameters = {};
  parameters.left_arm_raw_zero_rad[5] = 1e308;
  EXPECT_FALSE(encode_standard_command(command, parameters, HeadHold{}, true, payload));
}

TEST(StandardMapping, SuccessfulEncodingProducesOnlyFinitePayloadFields)
{
  V3CommandPayload payload{};
  ASSERT_TRUE(
    encode_standard_command(
      StandardCommand{}, MappingParameters{}, HeadHold{}, true, payload));
  EXPECT_TRUE(v3_command_payload_is_finite(payload));
}

}  // namespace
}  // namespace bw_std_control
