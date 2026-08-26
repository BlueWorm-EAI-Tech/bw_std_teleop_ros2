#include "bw_teleop/manager/vr_stream_manager.hpp"

#include <utility>

namespace bw_teleop::manager
{

VrStreamManager::VrStreamManager(VrStreamConfig config)
: input_mapper_{std::move(config.input)},
  session_manager_{std::move(config.session)},
  teleop_manager_{std::move(config.teleop)}
{
}

DatagramResult VrStreamManager::ingest(
  const transport::UdpDatagram & datagram, const double monotonic_now_sec)
{
  const protocol::VrParseResult parsed = parser_.parse(datagram.payload);
  if (!parsed.success) {
    return DatagramResult{
      parsed.ignored ? DatagramDisposition::ignored : DatagramDisposition::invalid,
      parsed.message,
      false};
  }

  const std::string previous_owner = session_manager_.owner();
  if (!session_manager_.accept_valid_packet(datagram.source_ip, monotonic_now_sec)) {
    return DatagramResult{
      DatagramDisposition::non_owner, "VR 数据报来自非 owner 客户端", false};
  }

  const input::VrInputMapResult mapped = input_mapper_.map(parsed.sample);
  if (!mapped.success) {
    return DatagramResult{DatagramDisposition::invalid, mapped.message, false};
  }
  teleop_manager_.update_vr_frame(mapped.frame, monotonic_now_sec);
  return DatagramResult{
    DatagramDisposition::accepted,
    mapped.message,
    previous_owner != session_manager_.owner()};
}

void VrStreamManager::update_measured_state(
  const MeasuredState & state, const double monotonic_now_sec) noexcept
{
  teleop_manager_.update_measured_state(state, monotonic_now_sec);
}

void VrStreamManager::update_measured_pose(
  const Side side, const Pose & pose, const double monotonic_now_sec) noexcept
{
  teleop_manager_.update_measured_pose(side, pose, monotonic_now_sec);
}

TickResult VrStreamManager::tick(const double monotonic_now_sec) noexcept
{
  return teleop_manager_.tick(monotonic_now_sec);
}

const std::string & VrStreamManager::owner() const noexcept
{
  return session_manager_.owner();
}

}  // namespace bw_teleop::manager
