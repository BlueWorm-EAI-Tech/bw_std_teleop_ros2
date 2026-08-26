#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "bw_teleop/input/vr_input_mapper.hpp"
#include "bw_teleop/manager/client_session_manager.hpp"
#include "bw_teleop/manager/teleop_manager.hpp"
#include "bw_teleop/manager/vr_stream_manager.hpp"
#include "bw_teleop/protocol/vr_packet_parser.hpp"
#include "bw_teleop/transport/udp_datagram.hpp"

namespace
{

using bw_teleop::protocol::VrPacketParser;

void append_u8(std::vector<std::uint8_t> & bytes, const std::uint8_t value)
{
  bytes.push_back(value);
}

void append_u32(std::vector<std::uint8_t> & bytes, const std::uint32_t value)
{
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    bytes.push_back(static_cast<std::uint8_t>((value >> (8U * index)) & 0xffU));
  }
}

void append_f32(std::vector<std::uint8_t> & bytes, const float value)
{
  std::uint32_t bits{0U};
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  append_u32(bytes, bits);
}

void append_f64(std::vector<std::uint8_t> & bytes, const double value)
{
  std::uint64_t bits{0U};
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    bytes.push_back(static_cast<std::uint8_t>((bits >> (8U * index)) & 0xffU));
  }
}

void append_pose(
  std::vector<std::uint8_t> & bytes,
  const std::array<float, 3> & position = {1.0F, 2.0F, 3.0F})
{
  for (const float value : position) {
    append_f32(bytes, value);
  }
  append_f32(bytes, 0.0F);
  append_f32(bytes, 0.0F);
  append_f32(bytes, 0.0F);
  append_f32(bytes, 2.0F);
}

std::vector<std::uint8_t> make_control_packet()
{
  std::vector<std::uint8_t> bytes;
  bytes.reserve(VrPacketParser::packet_size);
  append_u32(bytes, VrPacketParser::packet_magic);
  append_u8(bytes, VrPacketParser::packet_version);
  append_u8(bytes, 0U);
  append_f64(bytes, 42.5);
  append_pose(bytes);
  append_u8(bytes, 1U);
  append_u8(bytes, 1U);
  append_u8(bytes, 1U);
  append_u8(bytes, 0U);
  append_u8(bytes, 0U);
  append_u8(bytes, 1U);
  append_f32(bytes, 0.1F);
  append_f32(bytes, 0.2F);
  append_f32(bytes, 0.25F);
  append_f32(bytes, 0.75F);
  append_pose(bytes, {0.1F, 0.2F, 0.3F});
  append_pose(bytes, {-0.1F, -0.2F, -0.3F});
  append_u8(bytes, 1U);
  append_u8(bytes, 0U);
  append_u8(bytes, 0U);
  append_u8(bytes, 1U);
  append_f32(bytes, 0.10F);
  append_f32(bytes, 0.50F);
  append_u8(bytes, 0U);
  append_f32(bytes, -0.75F);
  append_f32(bytes, 0.25F);
  append_u8(bytes, 1U);
  append_u8(bytes, 1U);
  return bytes;
}

TEST(VrPacketParserTest, ParsesDirectControlPacketAndNormalizesQuaternion)
{
  const auto result = VrPacketParser{}.parse(make_control_packet());

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.ignored);
  EXPECT_DOUBLE_EQ(result.sample.sender_timestamp, 42.5);
  EXPECT_TRUE(result.sample.left_connected);
  EXPECT_TRUE(result.sample.right_connected);
  EXPECT_NEAR(result.sample.head_pose.orientation_xyzw[3], 1.0, 1.0e-12);
  EXPECT_NEAR(result.sample.right_trigger_value, 0.75, 1.0e-6);
}

TEST(VrPacketParserTest, UnwrapsVersionThreeRoutingEnvelope)
{
  const auto payload = make_control_packet();
  std::vector<std::uint8_t> packet{'B', 'W', 'V', 'R', 3U, 0U, 3U, 'S', 'N', '1'};
  packet.insert(packet.end(), payload.cbegin(), payload.cend());

  const auto result = VrPacketParser{}.parse(packet);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.sample.menu);
}

TEST(VrPacketParserTest, IgnoresCompleteHandJointsPacket)
{
  std::vector<std::uint8_t> packet(1054U, 0U);
  packet[0] = 'J';
  packet[1] = 'H';
  packet[2] = 'W';
  packet[3] = 'B';
  packet[4] = 2U;

  const auto result = VrPacketParser{}.parse(packet);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.ignored);
}

TEST(VrPacketParserTest, RejectsMalformedRoutingEnvelope)
{
  std::vector<std::uint8_t> wrong_version{'B', 'W', 'V', 'R', 2U, 0U, 0U};
  EXPECT_FALSE(VrPacketParser{}.parse(wrong_version).success);

  std::vector<std::uint8_t> missing_payload{'B', 'W', 'V', 'R', 3U, 0U, 3U, 'S', 'N', '1'};
  EXPECT_FALSE(VrPacketParser{}.parse(missing_payload).success);
}

TEST(VrPacketParserTest, RejectsNonBooleanControlField)
{
  auto packet = make_control_packet();
  constexpr std::size_t left_connected_offset{42U};
  packet[left_connected_offset] = 2U;

  EXPECT_FALSE(VrPacketParser{}.parse(packet).success);
}

TEST(ClientSessionManagerTest, TransfersOwnershipOnlyAfterTimeout)
{
  bw_teleop::manager::ClientSessionManager sessions({true, 3.0});

  EXPECT_TRUE(sessions.accept_valid_packet("10.0.0.1", 1.0));
  EXPECT_FALSE(sessions.accept_valid_packet("10.0.0.2", 3.5));
  EXPECT_TRUE(sessions.accept_valid_packet("10.0.0.2", 4.1));
  EXPECT_EQ(sessions.owner(), "10.0.0.2");
}

TEST(VrInputMapperTest, AppliesDeadzoneAndMapsControllerSemantics)
{
  const auto parsed = VrPacketParser{}.parse(make_control_packet());
  ASSERT_TRUE(parsed.success);
  bw_teleop::input::VrInputMapper mapper({0.15});

  const auto result = mapper.map(parsed.sample);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_DOUBLE_EQ(result.frame.left_joy.x, 0.0);
  EXPECT_NEAR(result.frame.left_joy.y, (0.50 - 0.15) / 0.85, 1.0e-6);
  EXPECT_NEAR(result.frame.right_joy.x, (-0.75 + 0.15) / 0.85, 1.0e-6);
  EXPECT_NEAR(result.frame.left_joy.trigger_value, 0.25, 1.0e-6);
  EXPECT_TRUE(result.frame.left_joy.grip);
  EXPECT_TRUE(result.frame.left_joy.menu);
  EXPECT_FALSE(result.frame.right_joy.menu);
}

TEST(VrInputMapperTest, AcceptsOnlyEmptyOrMatchingMeasuredPoseFrame)
{
  EXPECT_TRUE(bw_teleop::input::is_compatible_frame("", "C_Link"));
  EXPECT_TRUE(bw_teleop::input::is_compatible_frame("C_Link", "C_Link"));
  EXPECT_FALSE(bw_teleop::input::is_compatible_frame("base_link", "C_Link"));
}

TEST(TeleopManagerTest, HoldsStandardDefaultsUntilFeedbackArrives)
{
  bw_teleop::manager::TeleopConfig config;
  bw_teleop::manager::TeleopManager manager(config);
  bw_teleop::input::MappedVrFrame frame;
  frame.left_connected = true;
  frame.right_connected = true;
  frame.left_joy.grip = true;
  frame.left_joy.trigger_value = 1.0;
  frame.right_joy.trigger_value = 1.0;
  manager.update_vr_frame(frame, 10.0);

  EXPECT_TRUE(manager.tick(10.0).output.safe_hold);
  frame.left_joy.grip = false;
  manager.update_vr_frame(frame, 10.01);
  const auto output = manager.tick(10.01).output;

  EXPECT_FALSE(output.safe_hold);
  EXPECT_FALSE(output.auxiliary_commands_valid);
  EXPECT_DOUBLE_EQ(output.gripper_command[0], 0.0);
  EXPECT_DOUBLE_EQ(output.gripper_command[1], 0.0);
  EXPECT_DOUBLE_EQ(output.lift_command, 0.0);
}

TEST(TeleopManagerTest, WatchdogStopsBaseAfterCompleteFrameExpires)
{
  bw_teleop::manager::TeleopManager manager({});
  bw_teleop::input::MappedVrFrame frame;
  frame.left_connected = true;
  frame.right_connected = true;
  frame.left_joy.y = 1.0;
  manager.update_vr_frame(frame, 20.0);
  (void)manager.tick(20.0);
  manager.update_vr_frame(frame, 20.01);
  EXPECT_GT(manager.tick(20.01).output.base_vx, 0.0);

  const auto expired = manager.tick(20.25).output;
  EXPECT_TRUE(expired.safe_hold);
  EXPECT_DOUBLE_EQ(expired.base_vx, 0.0);
  EXPECT_DOUBLE_EQ(expired.base_vy, 0.0);
  EXPECT_DOUBLE_EQ(expired.base_wz, 0.0);
}

TEST(TeleopManagerTest, EnablesAuxiliaryCommandsOnlyAfterCompleteFeedback)
{
  bw_teleop::manager::TeleopConfig config;
  config.feedback_timeout_sec = 0.20;
  bw_teleop::manager::TeleopManager manager(config);
  EXPECT_FALSE(manager.tick(1.0).output.auxiliary_commands_valid);

  bw_teleop::manager::MeasuredState partial;
  partial.left_gripper_valid = true;
  partial.left_gripper = 0.01;
  partial.right_gripper_valid = true;
  partial.right_gripper = 0.02;
  manager.update_measured_state(partial, 1.0);
  EXPECT_FALSE(manager.tick(1.01).output.auxiliary_commands_valid);

  bw_teleop::manager::MeasuredState complete;
  complete.lift_valid = true;
  complete.lift = 0.15;
  manager.update_measured_state(complete, 1.01);
  const auto initialized = manager.tick(1.02).output;
  EXPECT_TRUE(initialized.auxiliary_commands_valid);
  EXPECT_DOUBLE_EQ(initialized.gripper_command[0], 0.01);
  EXPECT_DOUBLE_EQ(initialized.gripper_command[1], 0.02);
  EXPECT_DOUBLE_EQ(initialized.lift_command, 0.15);

  bw_teleop::input::MappedVrFrame frame;
  frame.left_connected = true;
  frame.right_connected = true;
  frame.left_joy.trigger_value = 1.0;
  frame.right_joy.trigger_value = 1.0;
  manager.update_vr_frame(frame, 1.03);
  const auto active = manager.tick(1.03).output;
  EXPECT_DOUBLE_EQ(active.gripper_command[0], 0.04965);
  EXPECT_DOUBLE_EQ(active.gripper_command[1], 0.04965);

  EXPECT_FALSE(manager.tick(1.22).output.auxiliary_commands_valid);
}

TEST(TeleopManagerTest, PublishesArmTargetOnlyWhileClutched)
{
  bw_teleop::manager::TeleopManager manager({});
  bw_teleop::input::MappedVrFrame frame;
  frame.left_connected = true;
  frame.right_connected = true;
  bw_teleop::manager::Pose measured_left;
  measured_left.position = {0.40, 0.20, 0.10};
  manager.update_measured_pose(
    bw_teleop::manager::Side::left, measured_left, 2.0);
  manager.update_vr_frame(frame, 2.0);
  EXPECT_FALSE(manager.tick(2.0).output.left_target_valid);

  frame.left_joy.grip = true;
  manager.update_vr_frame(frame, 2.01);
  EXPECT_TRUE(manager.tick(2.01).output.left_target_valid);

  frame.left_joy.grip = false;
  manager.update_vr_frame(frame, 2.02);
  EXPECT_FALSE(manager.tick(2.02).output.left_target_valid);

  frame.left_joy.grip = true;
  manager.update_vr_frame(frame, 2.03);
  EXPECT_TRUE(manager.tick(2.03).output.left_target_valid);
  EXPECT_FALSE(manager.tick(2.30).output.left_target_valid);
}

TEST(TeleopManagerTest, DoesNotArmWithoutMeasuredPose)
{
  bw_teleop::manager::TeleopManager manager({});
  bw_teleop::input::MappedVrFrame frame;
  frame.left_connected = true;
  frame.right_connected = true;
  manager.update_vr_frame(frame, 2.5);
  (void)manager.tick(2.5);

  frame.left_joy.grip = true;
  manager.update_vr_frame(frame, 2.51);
  EXPECT_FALSE(manager.tick(2.51).output.left_target_valid);
}

TEST(TeleopManagerTest, RejectsNonFiniteMotionLimits)
{
  bw_teleop::manager::TeleopConfig config;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  config.gripper_min = nan;
  config.gripper_max = nan;
  config.lift_min = nan;
  config.lift_max = nan;
  config.lift_initial_position = nan;
  config.base_max_forward = nan;
  config.base_max_lateral = nan;
  config.base_max_yaw = nan;
  config.base_speed_scales = {nan, nan, nan};
  EXPECT_THROW(
    bw_teleop::manager::TeleopManager manager(config), std::invalid_argument);
}

TEST(PoseMapperTest, UsesFreshMeasuredPoseAsRelativeRobotBaseline)
{
  bw_teleop::algorithm::PoseMapperConfig config;
  config.workspace.minimum = {0.0, -1.0, -1.0};
  config.workspace.maximum = {1.0, 1.0, 1.0};
  config.reset_poses[0].position = {0.25, 0.25, 0.0};
  config.reset_poses[1].position = {0.25, -0.25, 0.0};
  bw_teleop::algorithm::PoseMapper mapper(config);
  bw_teleop::algorithm::Pose hand;
  hand.position = {0.10, 0.20, 0.30};
  bw_teleop::algorithm::Pose measured;
  measured.position = {0.50, 0.25, 0.10};

  EXPECT_TRUE(mapper.update(
    bw_teleop::algorithm::Side::left, hand, true, measured, true));
  EXPECT_DOUBLE_EQ(
    mapper.target(bw_teleop::algorithm::Side::left).position.x, 0.50);

  hand.position.x = 0.20;
  EXPECT_TRUE(mapper.update(
    bw_teleop::algorithm::Side::left, hand, true, measured, true));
  EXPECT_DOUBLE_EQ(
    mapper.target(bw_teleop::algorithm::Side::left).position.x, 0.60);
}

TEST(PoseMapperTest, RejectsResetPoseOutsideWorkspace)
{
  bw_teleop::algorithm::PoseMapperConfig config;
  config.workspace.minimum = {0.10, -0.80, -0.60};
  config.workspace.maximum = {0.90, 0.80, 0.50};
  config.reset_poses[0].position = {-0.34, 0.25, 0.0};
  config.reset_poses[1].position = {0.25, -0.25, 0.0};

  EXPECT_THROW(
    bw_teleop::algorithm::PoseMapper mapper(config), std::invalid_argument);
}

TEST(PoseMapperTest, RejectsReversedWorkspaceBounds)
{
  bw_teleop::algorithm::PoseMapperConfig config;
  config.workspace.minimum.x = 1.0;
  config.workspace.maximum.x = 0.0;

  EXPECT_THROW(
    bw_teleop::algorithm::PoseMapper mapper(config), std::invalid_argument);
}

TEST(TeleopManagerTest, StopsArmOutputWhenMeasuredPoseExpires)
{
  bw_teleop::manager::TeleopConfig config;
  config.arm_pose_timeout_sec = 0.20;
  bw_teleop::manager::TeleopManager manager(config);
  bw_teleop::manager::Pose measured;
  measured.position = {0.40, 0.20, 0.10};
  manager.update_measured_pose(bw_teleop::manager::Side::left, measured, 4.0);

  bw_teleop::input::MappedVrFrame frame;
  frame.left_connected = true;
  frame.right_connected = true;
  manager.update_vr_frame(frame, 4.0);
  (void)manager.tick(4.0);
  frame.left_joy.grip = true;
  manager.update_vr_frame(frame, 4.01);
  EXPECT_TRUE(manager.tick(4.01).output.left_target_valid);

  manager.update_vr_frame(frame, 4.21);
  EXPECT_FALSE(manager.tick(4.21).output.left_target_valid);
}

TEST(VrStreamManagerTest, DeliversOnlyValidOwnerFramesToTeleopManager)
{
  bw_teleop::manager::VrStreamConfig config;
  bw_teleop::manager::VrStreamManager manager(config);
  bw_teleop::transport::UdpDatagram owner_packet{make_control_packet(), "10.0.0.1"};

  const auto accepted = manager.ingest(owner_packet, 30.0);
  EXPECT_EQ(accepted.disposition, bw_teleop::manager::DatagramDisposition::accepted);
  EXPECT_EQ(manager.owner(), "10.0.0.1");

  bw_teleop::transport::UdpDatagram other_packet{make_control_packet(), "10.0.0.2"};
  const auto rejected = manager.ingest(other_packet, 30.1);
  EXPECT_EQ(rejected.disposition, bw_teleop::manager::DatagramDisposition::non_owner);
  EXPECT_EQ(manager.owner(), "10.0.0.1");
}

}  // namespace
