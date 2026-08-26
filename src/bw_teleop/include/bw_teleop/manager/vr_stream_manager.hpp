#pragma once

#include <string>

#include "bw_teleop/input/vr_input_mapper.hpp"
#include "bw_teleop/manager/client_session_manager.hpp"
#include "bw_teleop/manager/teleop_manager.hpp"
#include "bw_teleop/protocol/vr_packet_parser.hpp"
#include "bw_teleop/transport/udp_datagram.hpp"

namespace bw_teleop::manager
{

enum class DatagramDisposition
{
  accepted,
  ignored,
  invalid,
  non_owner,
};

struct DatagramResult
{
  DatagramDisposition disposition{DatagramDisposition::invalid};
  std::string message;
  bool owner_changed{false};
};

struct VrStreamConfig
{
  TeleopConfig teleop;
  input::VrInputMapperConfig input;
  ClientSessionConfig session;
};

class VrStreamManager
{
public:
  explicit VrStreamManager(VrStreamConfig config);

  [[nodiscard]] DatagramResult ingest(
    const transport::UdpDatagram & datagram, double monotonic_now_sec);
  void update_measured_state(
    const MeasuredState & state, double monotonic_now_sec) noexcept;
  void update_measured_pose(
    Side side, const Pose & pose, double monotonic_now_sec) noexcept;
  [[nodiscard]] TickResult tick(double monotonic_now_sec) noexcept;
  [[nodiscard]] const std::string & owner() const noexcept;

private:
  protocol::VrPacketParser parser_;
  input::VrInputMapper input_mapper_;
  ClientSessionManager session_manager_;
  TeleopManager teleop_manager_;
};

}  // namespace bw_teleop::manager
