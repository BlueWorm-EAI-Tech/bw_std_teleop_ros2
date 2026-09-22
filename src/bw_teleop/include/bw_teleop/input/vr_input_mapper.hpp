#pragma once

#include <string>

#include "bw_teleop/algorithm/pose_mapper.hpp"
#include "bw_teleop/protocol/vr_packet_parser.hpp"

namespace bw_teleop::input
{

[[nodiscard]] bool is_compatible_frame(
  const std::string & received_frame, const std::string & target_frame) noexcept;

struct JoyInput
{
  double x{0.0};
  double y{0.0};
  double trigger_value{0.0};
  bool xa{false};
  bool yb{false};
  bool grip{false};
  bool menu{false};
  bool trigger{false};
  bool joystick{false};
};

struct MappedVrFrame
{
  double sender_timestamp{0.0};
  algorithm::Pose head_pose;
  algorithm::Pose left_pose;
  algorithm::Pose right_pose;
  JoyInput left_joy;
  JoyInput right_joy;
  bool left_connected{false};
  bool right_connected{false};
};

struct VrInputMapResult
{
  bool success{false};
  MappedVrFrame frame;
  std::string message;
};

struct VrInputMapperConfig
{
  double joystick_deadzone{0.15};
};

class VrInputMapper
{
public:
  explicit VrInputMapper(VrInputMapperConfig config);

  [[nodiscard]] VrInputMapResult map(const protocol::VrSample & sample) const;

private:
  [[nodiscard]] double apply_deadzone(double value) const noexcept;

  VrInputMapperConfig config_;
};

}  // namespace bw_teleop::input
