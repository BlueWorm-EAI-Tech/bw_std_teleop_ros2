#include "bw_teleop/manager/vr_stream_manager.hpp"

#include <cmath>
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
  if (!std::isfinite(monotonic_now_sec)) {
    clear_input_if_owned(datagram.source_ip);
    return DatagramResult{
      DatagramDisposition::invalid, "VR 本地接收时间戳非法", false};
  }

  const protocol::VrParseResult parsed = parser_.parse(datagram.payload);
  if (!parsed.success) {
    if (!parsed.ignored) {
      clear_input_if_owned(datagram.source_ip);
    }
    return DatagramResult{
      parsed.ignored ? DatagramDisposition::ignored : DatagramDisposition::invalid,
      parsed.message,
      false};
  }

  const input::VrInputMapResult mapped = input_mapper_.map(parsed.sample);
  if (!mapped.success) {
    clear_input_if_owned(datagram.source_ip);
    return DatagramResult{DatagramDisposition::invalid, mapped.message, false};
  }

  const std::string previous_owner = session_manager_.owner();
  // owner 过期可清除 owner, 但本检查不刷新 heartbeat; sender 顺序检查先于接受数据报,
  // 因此也先于 last_seen_sec_ 推进。
  if (!session_manager_.can_accept_valid_packet(datagram.source_ip, monotonic_now_sec)) {
    return DatagramResult{
      DatagramDisposition::non_owner, "VR 数据报来自非 owner 客户端", false};
  }
  if (previous_owner != session_manager_.owner()) {
    last_sender_timestamps_.clear();
  }

  const auto previous_timestamp = last_sender_timestamps_.find(datagram.source_ip);
  if (previous_timestamp != last_sender_timestamps_.end() &&
    mapped.frame.sender_timestamp <= previous_timestamp->second)
  {
    return DatagramResult{
      DatagramDisposition::out_of_order, "VR 发送端时间戳非严格递增", false};
  }
  if (!session_manager_.accept_valid_packet(datagram.source_ip, monotonic_now_sec)) {
    return DatagramResult{
      DatagramDisposition::non_owner, "VR 数据报来自非 owner 客户端", false};
  }
  const bool owner_changed = previous_owner != session_manager_.owner();
  last_sender_timestamps_[datagram.source_ip] = mapped.frame.sender_timestamp;
  teleop_manager_.update_vr_frame(mapped.frame, monotonic_now_sec);
  return DatagramResult{
    DatagramDisposition::accepted,
    mapped.message,
    owner_changed};
}

void VrStreamManager::update_measured_state(
  const MeasuredState & state, const double monotonic_now_sec) noexcept
{
  teleop_manager_.update_measured_state(state, monotonic_now_sec);
}

void VrStreamManager::invalidate_measured_state() noexcept
{
  teleop_manager_.invalidate_measured_state();
}

void VrStreamManager::invalidate_measured_gripper(const Side side) noexcept
{
  teleop_manager_.invalidate_measured_gripper(side);
}

void VrStreamManager::invalidate_measured_lift() noexcept
{
  teleop_manager_.invalidate_measured_lift();
}

void VrStreamManager::update_measured_pose(
  const Side side, const Pose & pose, const double monotonic_now_sec) noexcept
{
  teleop_manager_.update_measured_pose(side, pose, monotonic_now_sec);
}

void VrStreamManager::invalidate_measured_pose(const Side side) noexcept
{
  teleop_manager_.invalidate_measured_pose(side);
}

TickResult VrStreamManager::tick(const double monotonic_now_sec) noexcept
{
  return teleop_manager_.tick(monotonic_now_sec);
}

const std::string & VrStreamManager::owner() const noexcept
{
  return session_manager_.owner();
}

void VrStreamManager::clear_input_if_owned(const std::string & source_ip) noexcept
{
  if (session_manager_.owns_or_unlocked(source_ip)) {
    teleop_manager_.clear_vr_input();
  }
}

}  // namespace bw_teleop::manager
