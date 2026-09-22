#ifndef BW_STD_CONTROL__PROTOCOL_HPP_
#define BW_STD_CONTROL__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace bw_std_control
{

constexpr std::uint8_t kHeader0 = 0x55;
constexpr std::uint8_t kHeader1 = 0xAA;
constexpr std::uint8_t kCommandType = 0x01;
constexpr std::uint8_t kFeedbackType = 0x02;
// 控制字位定义(与 standard_0907 一致): bit0 软件上电, bit2 底盘, bit3 左臂, bit4 右臂, bit5 头部。
constexpr std::uint8_t kControlSoftwarePower = 0x01U;
constexpr std::uint8_t kControlChassis = 0x04U;
constexpr std::uint8_t kControlLeftArm = 0x08U;
constexpr std::uint8_t kControlRightArm = 0x10U;
constexpr std::uint8_t kControlHead = 0x20U;
constexpr std::uint8_t kArmControlMask =
  static_cast<std::uint8_t>(kControlLeftArm | kControlRightArm);
constexpr std::uint8_t kActiveControlFlag =
  static_cast<std::uint8_t>(
  kControlSoftwarePower | kControlChassis | kArmControlMask | kControlHead);
// 预充电帧: 仅关闭双臂使能位。
constexpr std::uint8_t kArmsDisabledControlFlag =
  static_cast<std::uint8_t>(kActiveControlFlag & ~kArmControlMask);
static_assert(kActiveControlFlag == 0x3DU, "active control flag must stay 0x3D");
static_assert(kArmControlMask == 0x18U, "arm control mask must stay 0x18");
static_assert(kArmsDisabledControlFlag == 0x25U, "precharge control flag must stay 0x25");
constexpr std::size_t kPrefixSize = 5;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kCommandPayloadSize = 249;
constexpr std::size_t kFeedbackPayloadSize = 248;
constexpr std::size_t kCommandFrameSize =
  kPrefixSize + kCommandPayloadSize + kCrcSize;
constexpr std::size_t kFeedbackFrameSize =
  kPrefixSize + kFeedbackPayloadSize + kCrcSize;

#pragma pack(push, 1)
struct CommandPayload
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
  // byte 177: 头部 roll 位置(旧头部速度字段已废弃)。
  float head_roll_position{0.0F};
  float head_pitch_max_velocity{0.0F};
  std::array<float, 8> left_joint_torque{};
  std::array<float, 8> right_joint_torque{};
};

struct FeedbackPayload
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

static_assert(sizeof(float) == 4, "protocol requires 32-bit float");
static_assert(sizeof(CommandPayload) == kCommandPayloadSize);
static_assert(offsetof(CommandPayload, head_roll_position) == 177U);
static_assert(sizeof(FeedbackPayload) == kFeedbackPayloadSize);

using CommandFrame = std::array<std::uint8_t, kCommandFrameSize>;
using FeedbackFrame = std::array<std::uint8_t, kFeedbackFrameSize>;

constexpr std::uint8_t kChassisCommandType = 0x21U;
constexpr std::size_t kChassisCommandPayloadSize = 37U;
constexpr std::size_t kChassisCommandFrameSize =
  kPrefixSize + kChassisCommandPayloadSize + kCrcSize;

#pragma pack(push, 1)
struct ChassisCommandPayload
{
  std::uint8_t control_flag{0};
  float vx{0.0F};
  float vy{0.0F};
  float omega{0.0F};
  float max_acc_x{0.0F};
  float max_acc_y{0.0F};
  float max_acc_omega{0.0F};
  float pelvis_height{0.0F};
  float pelvis_max_velocity{0.0F};
  float pelvis_acceleration{0.0F};
};
#pragma pack(pop)

static_assert(
  sizeof(ChassisCommandPayload) == kChassisCommandPayloadSize,
  "chassis command payload must be 37 bytes");

using ChassisCommandFrame = std::array<std::uint8_t, kChassisCommandFrameSize>;

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__PROTOCOL_HPP_
