#include <algorithm>
#include <array>
#include <chrono>
#include <string_view>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "bw_std_control/feedback_handoff.hpp"
#include "bw_std_control/hardware_contract.hpp"
#include "bw_std_control/v3_command_queue.hpp"

namespace bw_std_control
{
namespace
{

TEST(V3CommandQueue, ReportsFullDeterministicallyAndPreservesFifo)
{
  V3CommandQueue queue;
  EXPECT_TRUE(queue.is_lock_free());
  V3CommandFrame frame{};
  for (std::size_t index = 0; index < V3CommandQueue::capacity(); ++index) {
    frame[0] = static_cast<std::uint8_t>(index);
    ASSERT_TRUE(queue.try_push(frame));
  }
  EXPECT_FALSE(queue.try_push(frame));
  V3CommandFrame value{};
  ASSERT_TRUE(queue.try_pop(value));
  EXPECT_EQ(value[0], 0U);
  std::size_t popped = 1U;
  while (queue.try_pop(value)) {
    ++popped;
  }
  EXPECT_EQ(popped, V3CommandQueue::capacity());
}

TEST(V3CommandMailbox, PowerOffBypassesAFullNormalCommandQueue)
{
  V3CommandMailbox mailbox;
  V3CommandFrame normal{};
  normal[kV3PrefixSize] = 1U;
  for (std::size_t index = 0; index < V3CommandQueue::capacity(); ++index) {
    ASSERT_TRUE(mailbox.try_push_normal(normal));
  }
  EXPECT_FALSE(mailbox.try_push_normal(normal));

  V3CommandFrame power_off{};
  power_off[kV3PrefixSize] = 0U;
  EXPECT_TRUE(mailbox.try_push_power_off(power_off));

  V3CommandFrame next{};
  bool is_power_off = false;
  ASSERT_TRUE(mailbox.try_pop_next(next, is_power_off));
  EXPECT_TRUE(is_power_off);
  EXPECT_EQ(next[kV3PrefixSize], 0U);
  EXPECT_FALSE(mailbox.try_pop_next(next, is_power_off));
  EXPECT_FALSE(mailbox.try_push_normal(normal));
  EXPECT_FALSE(mailbox.wait_for_power_off(std::chrono::milliseconds{0}));

  mailbox.complete_power_off_from_consumer();
  EXPECT_TRUE(mailbox.wait_for_power_off(std::chrono::milliseconds{0}));
  EXPECT_TRUE(mailbox.try_push_normal(normal));
  ASSERT_TRUE(mailbox.try_pop_next(next, is_power_off));
  EXPECT_FALSE(is_power_off);
}

TEST(FeedbackHandoff, RealtimeReadReturnsLatestPublishedCompleteSample)
{
  FeedbackHandoff handoff;
  StampedFeedback first{};
  first.feedback.pelvis_height = 1500.0F;
  first.received_at = std::chrono::steady_clock::time_point{std::chrono::seconds{1}};
  first.valid = true;
  StampedFeedback latest{first};
  latest.feedback.pelvis_height = 1520.0F;
  latest.received_at = std::chrono::steady_clock::time_point{std::chrono::seconds{2}};

  handoff.publish(first);
  handoff.publish(latest);
  StampedFeedback received{};
  ASSERT_TRUE(handoff.read_latest(received));
  EXPECT_FLOAT_EQ(received.feedback.pelvis_height, 1520.0F);
  EXPECT_EQ(received.received_at, latest.received_at);
}

TEST(HardwareContract, AcceptsAnyOrderButRejectsMissingDuplicateOrExtraJoint)
{
  std::vector<std::string_view> names{kStandardJointNames.begin(), kStandardJointNames.end()};
  std::reverse(names.begin(), names.end());
  EXPECT_TRUE(has_exact_standard_joint_names(names));
  names.pop_back();
  EXPECT_FALSE(has_exact_standard_joint_names(names));
  names.push_back(names.front());
  EXPECT_FALSE(has_exact_standard_joint_names(names));
  names.push_back("head_joint");
  EXPECT_FALSE(has_exact_standard_joint_names(names));
}

TEST(HardwareContract, ParsesExactlySevenFiniteArmRawZeros)
{
  std::array<double, kArmJointCount> values{};
  EXPECT_TRUE(parse_arm_raw_zero_parameter("0.1,-0.2,0.3,-0.4,0.5,-0.6,0.7", values));
  EXPECT_DOUBLE_EQ(values[0], 0.1);
  EXPECT_DOUBLE_EQ(values[6], 0.7);

  EXPECT_FALSE(parse_arm_raw_zero_parameter("0,0,0,0,0,0", values));
  EXPECT_FALSE(parse_arm_raw_zero_parameter("0,0,0,0,0,0,0,0", values));
  EXPECT_FALSE(parse_arm_raw_zero_parameter("0,0,0,nan,0,0,0", values));
  EXPECT_FALSE(parse_arm_raw_zero_parameter("0,0,0,inf,0,0,0", values));
  EXPECT_FALSE(parse_arm_raw_zero_parameter("0,0,0,0,0,0,", values));
}

TEST(HardwareContract, ParsesOnlyArmMotorPermutations)
{
  std::array<std::size_t, kArmJointCount> values{};
  EXPECT_TRUE(parse_arm_motor_indices_parameter("6,5,4,3,2,1,0", values));
  EXPECT_EQ(values[0], 6U);
  EXPECT_EQ(values[6], 0U);

  EXPECT_FALSE(parse_arm_motor_indices_parameter("0,1,2,3,4,5", values));
  EXPECT_FALSE(parse_arm_motor_indices_parameter("0,1,2,3,4,5,6,7", values));
  EXPECT_FALSE(parse_arm_motor_indices_parameter("0,1,2,3,4,5,5", values));
  EXPECT_FALSE(parse_arm_motor_indices_parameter("0,1,2,3,4,5,7", values));
  EXPECT_FALSE(parse_arm_motor_indices_parameter("0,1,2,3,4,5,-1", values));
}

TEST(HardwareContract, ParsesOnlySignedArmDirections)
{
  std::array<double, kArmJointCount> values{};
  EXPECT_TRUE(parse_arm_direction_parameter("-1,1,-1,1,-1,1,-1", values));
  EXPECT_DOUBLE_EQ(values[0], -1.0);
  EXPECT_DOUBLE_EQ(values[6], -1.0);

  EXPECT_FALSE(parse_arm_direction_parameter("1,1,1,1,1,1", values));
  EXPECT_FALSE(parse_arm_direction_parameter("1,1,1,1,1,1,1,1", values));
  EXPECT_FALSE(parse_arm_direction_parameter("1,1,1,1,1,1,0", values));
  EXPECT_FALSE(parse_arm_direction_parameter("1,1,1,1,1,1,0.5", values));
  EXPECT_FALSE(parse_arm_direction_parameter("1,1,1,1,1,1,nan", values));
}

TEST(HardwareContract, EnablesSoftwarePowerOnlyForCalibratedMapping)
{
  EXPECT_FALSE(arm_mapping_enables_software_power(false, false));
  EXPECT_FALSE(arm_mapping_enables_software_power(false, true));
  EXPECT_TRUE(arm_mapping_enables_software_power(true, true));
  EXPECT_FALSE(arm_mapping_enables_software_power(true, false));
}

}  // namespace
}  // namespace bw_std_control
