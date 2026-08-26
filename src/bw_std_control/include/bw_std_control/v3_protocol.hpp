#ifndef BW_STD_CONTROL__V3_PROTOCOL_HPP_
#define BW_STD_CONTROL__V3_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace bw_std_control
{

constexpr std::uint8_t kV3Header0 = 0x55;
constexpr std::uint8_t kV3Header1 = 0xAA;
constexpr std::uint8_t kV3CommandType = 0x01;
constexpr std::uint8_t kV3FeedbackType = 0x02;
constexpr std::size_t kV3PrefixSize = 5;
constexpr std::size_t kV3CrcSize = 2;
constexpr std::size_t kV3CommandPayloadSize = 249;
constexpr std::size_t kV3FeedbackPayloadSize = 248;
constexpr std::size_t kV3CommandFrameSize =
  kV3PrefixSize + kV3CommandPayloadSize + kV3CrcSize;
constexpr std::size_t kV3FeedbackFrameSize =
  kV3PrefixSize + kV3FeedbackPayloadSize + kV3CrcSize;

#pragma pack(push, 1)
struct V3CommandPayload
{
  std::uint8_t control_flag{0};
  float vx{0.0F};
  float vy{0.0F};
  float omega{0.0F};
  float max_acc_x{0.0F};
  float max_acc_y{0.0F};
  float max_acc_omega{0.0F};
  float pelvis_height{0.0F};
  float pelvis_velocity{0.0F};
  std::array<float, 8> left_joint_position{};
  std::array<float, 8> right_joint_position{};
  float waist_position{0.0F};
  float head_yaw_position{0.0F};
  float head_pitch_position{0.0F};
  std::array<float, 8> left_joint_max_velocity{};
  std::array<float, 8> right_joint_max_velocity{};
  float waist_max_velocity{0.0F};
  float head_yaw_max_velocity{0.0F};
  float head_pitch_max_velocity{0.0F};
  std::array<float, 8> left_joint_torque{};
  std::array<float, 8> right_joint_torque{};
};

struct V3FeedbackPayload
{
  std::uint8_t status_flags{0};
  std::uint8_t left_arm_status_flags{0};
  std::uint8_t right_arm_status_flags{0};
  std::uint8_t chassis_status_flags{0};
  float chassis_vx{0.0F};
  float chassis_vy{0.0F};
  float chassis_omega{0.0F};
  float odom_x{0.0F};
  float odom_y{0.0F};
  float robot_heading{0.0F};
  float robot_heading_velocity{0.0F};
  float pelvis_height{0.0F};
  float pelvis_velocity{0.0F};
  std::array<float, 8> left_joint_position{};
  std::array<float, 8> right_joint_position{};
  float waist_position{0.0F};
  float head_yaw_position{0.0F};
  float head_pitch_position{0.0F};
  std::array<float, 8> left_joint_velocity{};
  std::array<float, 8> right_joint_velocity{};
  float waist_velocity{0.0F};
  std::array<float, 8> left_joint_torque{};
  std::array<float, 8> right_joint_torque{};
};
#pragma pack(pop)

static_assert(sizeof(float) == 4, "V3 protocol requires 32-bit float");
static_assert(sizeof(V3CommandPayload) == kV3CommandPayloadSize);
static_assert(sizeof(V3FeedbackPayload) == kV3FeedbackPayloadSize);

using V3CommandFrame = std::array<std::uint8_t, kV3CommandFrameSize>;
using V3FeedbackFrame = std::array<std::uint8_t, kV3FeedbackFrameSize>;

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__V3_PROTOCOL_HPP_
