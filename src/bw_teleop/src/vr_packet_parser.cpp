#include "bw_teleop/protocol/vr_packet_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace bw_teleop::protocol
{
namespace
{

constexpr double kMinimumQuaternionNorm{1.0e-12};
constexpr std::array<std::uint8_t, 4U> kRoutingMagic{0x42U, 0x57U, 0x56U, 0x52U};
constexpr std::uint8_t kRoutingVersion{3U};
constexpr std::size_t kRoutingHeaderBaseSize{7U};
constexpr std::uint32_t kHandJointsPacketMagic{0x4257484AU};
constexpr std::uint8_t kHandJointsPacketVersion{2U};
constexpr std::size_t kHandJointsPacketMinSize{1054U};

class LittleEndianReader
{
public:
  explicit LittleEndianReader(const std::vector<std::uint8_t> & packet)
  : packet_{packet}
  {
  }

  bool read_u8(std::uint8_t & value)
  {
    if (offset_ + 1U > packet_.size()) {
      return false;
    }
    value = packet_[offset_++];
    return true;
  }

  bool read_u32(std::uint32_t & value)
  {
    if (offset_ + 4U > packet_.size()) {
      return false;
    }
    value = static_cast<std::uint32_t>(packet_[offset_]) |
      (static_cast<std::uint32_t>(packet_[offset_ + 1U]) << 8U) |
      (static_cast<std::uint32_t>(packet_[offset_ + 2U]) << 16U) |
      (static_cast<std::uint32_t>(packet_[offset_ + 3U]) << 24U);
    offset_ += 4U;
    return true;
  }

  bool read_f32(double & value)
  {
    std::uint32_t bits{0U};
    if (!read_u32(bits)) {
      return false;
    }
    float raw{0.0F};
    static_assert(sizeof(raw) == sizeof(bits));
    std::memcpy(&raw, &bits, sizeof(raw));
    value = static_cast<double>(raw);
    return true;
  }

  bool read_f64(double & value)
  {
    if (offset_ + 8U > packet_.size()) {
      return false;
    }
    std::uint64_t bits{0U};
    for (std::size_t index = 0U; index < 8U; ++index) {
      bits |= static_cast<std::uint64_t>(packet_[offset_ + index]) << (8U * index);
    }
    offset_ += 8U;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return true;
  }

  bool read_bool(bool & value)
  {
    std::uint8_t raw{0U};
    if (!read_u8(raw) || raw > 1U) {
      return false;
    }
    value = raw == 1U;
    return true;
  }

  [[nodiscard]] std::size_t offset() const noexcept
  {
    return offset_;
  }

private:
  const std::vector<std::uint8_t> & packet_;
  std::size_t offset_{0U};
};

bool read_pose(LittleEndianReader & reader, VrPose & pose)
{
  for (double & value : pose.position) {
    if (!reader.read_f32(value)) {
      return false;
    }
  }
  for (double & value : pose.orientation_xyzw) {
    if (!reader.read_f32(value)) {
      return false;
    }
  }
  const auto finite = [](const double value) {return std::isfinite(value);};
  return std::all_of(pose.position.cbegin(), pose.position.cend(), finite) &&
         std::all_of(pose.orientation_xyzw.cbegin(), pose.orientation_xyzw.cend(), finite);
}

bool normalize_pose(VrPose & pose, const bool allow_zero_quaternion)
{
  double norm_squared{0.0};
  for (const double value : pose.orientation_xyzw) {
    norm_squared += value * value;
  }
  const double norm = std::sqrt(norm_squared);
  if (!std::isfinite(norm) || norm < kMinimumQuaternionNorm) {
    if (!allow_zero_quaternion) {
      return false;
    }
    pose.orientation_xyzw = {0.0, 0.0, 0.0, 1.0};
    return true;
  }
  for (double & value : pose.orientation_xyzw) {
    value /= norm;
  }
  return true;
}

bool controls_are_finite(const VrSample & sample)
{
  const std::array<double, 8> values{
    sample.left_grip_value, sample.right_grip_value,
    sample.left_trigger_value, sample.right_trigger_value,
    sample.left_joystick_x, sample.left_joystick_y,
    sample.right_joystick_x, sample.right_joystick_y};
  return std::all_of(values.cbegin(), values.cend(), [](const double value) {
    return std::isfinite(value);
  });
}

std::uint32_t read_packet_magic(const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < sizeof(std::uint32_t)) {
    return 0U;
  }
  return static_cast<std::uint32_t>(packet[0U]) |
         (static_cast<std::uint32_t>(packet[1U]) << 8U) |
         (static_cast<std::uint32_t>(packet[2U]) << 16U) |
         (static_cast<std::uint32_t>(packet[3U]) << 24U);
}

bool unwrap_routing_packet(
  const std::vector<std::uint8_t> & packet,
  std::vector<std::uint8_t> & payload,
  std::string & error)
{
  if (packet.size() < kRoutingMagic.size() ||
    !std::equal(kRoutingMagic.cbegin(), kRoutingMagic.cend(), packet.cbegin()))
  {
    payload = packet;
    return true;
  }
  if (packet.size() < kRoutingHeaderBaseSize) {
    error = "VR version 3 路由封装头不完整";
    return false;
  }
  if (packet[4U] != kRoutingVersion) {
    error = "VR 路由封装 version 必须为 3";
    return false;
  }
  const std::size_t header_size = kRoutingHeaderBaseSize + packet[6U];
  if (header_size >= packet.size()) {
    error = "VR 路由封装头长非法或 payload 为空";
    return false;
  }
  payload.assign(
    packet.cbegin() + static_cast<std::vector<std::uint8_t>::difference_type>(header_size),
    packet.cend());
  return true;
}

}  // namespace

VrParseResult VrPacketParser::parse(const std::vector<std::uint8_t> & packet) const
{
  VrParseResult result;
  std::vector<std::uint8_t> payload;
  if (!unwrap_routing_packet(packet, payload, result.message)) {
    return result;
  }

  const std::uint32_t magic = read_packet_magic(payload);
  if (magic == kHandJointsPacketMagic && payload.size() >= 5U &&
    payload[4U] == kHandJointsPacketVersion)
  {
    if (payload.size() < kHandJointsPacketMinSize) {
      result.message = "BWHJ v2 帧长度不足";
      return result;
    }
    result.ignored = true;
    result.message = "基础遥操作忽略 HandJoints v2 UDP 帧";
    return result;
  }
  if (magic != packet_magic) {
    result.message = "VR 帧 magic 错误";
    return result;
  }
  if (payload.size() != packet_size) {
    result.message = "VR 协议 v1 二进制帧必须为 143 字节，实际为 " +
      std::to_string(payload.size()) + " 字节";
    return result;
  }

  LittleEndianReader reader{payload};
  std::uint32_t parsed_magic{0U};
  std::uint8_t version{0U};
  std::uint8_t flags{0U};
  if (!reader.read_u32(parsed_magic) || parsed_magic != packet_magic) {
    result.message = "VR 帧 magic 错误";
    return result;
  }
  if (!reader.read_u8(version) || version != packet_version) {
    result.message = "VR 帧 version 错误";
    return result;
  }
  if (!reader.read_u8(flags)) {
    result.message = "VR 帧 flags 缺失";
    return result;
  }
  (void)flags;

  VrSample & sample = result.sample;
  if (!reader.read_f64(sample.sender_timestamp) || !std::isfinite(sample.sender_timestamp)) {
    result.message = "VR 帧时间戳非法";
    return result;
  }
  if (!read_pose(reader, sample.head_pose) || !normalize_pose(sample.head_pose, false)) {
    result.message = "VR 头显位姿非法";
    return result;
  }
  if (!reader.read_bool(sample.left_connected) || !reader.read_bool(sample.right_connected) ||
    !reader.read_bool(sample.left_grip) || !reader.read_bool(sample.right_grip) ||
    !reader.read_bool(sample.left_trigger) || !reader.read_bool(sample.right_trigger) ||
    !reader.read_f32(sample.left_grip_value) || !reader.read_f32(sample.right_grip_value) ||
    !reader.read_f32(sample.left_trigger_value) || !reader.read_f32(sample.right_trigger_value))
  {
    result.message = "VR 连接、clutch 或 trigger 字段非法";
    return result;
  }
  if (!read_pose(reader, sample.left_pose) || !read_pose(reader, sample.right_pose) ||
    !normalize_pose(sample.left_pose, !sample.left_connected) ||
    !normalize_pose(sample.right_pose, !sample.right_connected))
  {
    result.message = "VR 手柄位姿非法";
    return result;
  }
  if (!reader.read_bool(sample.left_ax_button) || !reader.read_bool(sample.left_by_button) ||
    !reader.read_bool(sample.right_ax_button) || !reader.read_bool(sample.right_by_button) ||
    !reader.read_f32(sample.left_joystick_x) || !reader.read_f32(sample.left_joystick_y) ||
    !reader.read_bool(sample.left_joystick_clicked) ||
    !reader.read_f32(sample.right_joystick_x) || !reader.read_f32(sample.right_joystick_y) ||
    !reader.read_bool(sample.right_joystick_clicked) || !reader.read_bool(sample.menu))
  {
    result.message = "VR 按键或摇杆字段非法";
    return result;
  }
  if (reader.offset() != packet_size || !controls_are_finite(sample)) {
    result.message = "VR 帧字段数量错误或控制量包含非有限值";
    return result;
  }

  result.success = true;
  result.message = "VR 帧有效";
  return result;
}

}  // namespace bw_teleop::protocol
