#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bw_teleop::protocol
{

struct VrPose
{
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct VrSample
{
  double sender_timestamp{0.0};
  VrPose head_pose;
  bool left_connected{false};
  bool right_connected{false};
  bool left_grip{false};
  bool right_grip{false};
  bool left_trigger{false};
  bool right_trigger{false};
  double left_grip_value{0.0};
  double right_grip_value{0.0};
  double left_trigger_value{0.0};
  double right_trigger_value{0.0};
  VrPose left_pose;
  VrPose right_pose;
  bool left_ax_button{false};
  bool left_by_button{false};
  bool right_ax_button{false};
  bool right_by_button{false};
  double left_joystick_x{0.0};
  double left_joystick_y{0.0};
  bool left_joystick_clicked{false};
  double right_joystick_x{0.0};
  double right_joystick_y{0.0};
  bool right_joystick_clicked{false};
  bool menu{false};
};

struct VrParseResult
{
  bool success{false};
  bool ignored{false};
  VrSample sample;
  std::string message;
};

class VrPacketParser
{
public:
  static constexpr std::uint32_t packet_magic{0x42575652U};
  static constexpr std::uint8_t packet_version{1U};
  static constexpr std::size_t packet_size{143U};

  [[nodiscard]] VrParseResult parse(const std::vector<std::uint8_t> & packet) const;
};

}  // namespace bw_teleop::protocol
