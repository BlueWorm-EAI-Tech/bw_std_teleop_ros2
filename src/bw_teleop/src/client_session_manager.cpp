#include "bw_teleop/manager/client_session_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace bw_teleop::manager
{

ClientSessionManager::ClientSessionManager(ClientSessionConfig config)
: config_{std::move(config)}
{
  if (!std::isfinite(config_.timeout_sec) || config_.timeout_sec <= 0.0) {
    throw std::invalid_argument("客户端锁超时必须为有限正数");
  }
}

bool ClientSessionManager::accept_valid_packet(
  const std::string & client_ip, const double monotonic_now_sec)
{
  if (!can_accept_valid_packet(client_ip, monotonic_now_sec)) {
    return false;
  }
  if (!config_.lock_enabled) {
    return true;
  }
  if (owner_.empty()) {
    owner_ = client_ip;
  }
  last_seen_sec_ = monotonic_now_sec;
  return true;
}

bool ClientSessionManager::can_accept_valid_packet(
  const std::string & client_ip, const double monotonic_now_sec)
{
  if (client_ip.empty() || !std::isfinite(monotonic_now_sec)) {
    return false;
  }
  if (!config_.lock_enabled) {
    return true;
  }
  release_if_expired(monotonic_now_sec);
  return owner_.empty() || owner_ == client_ip;
}

const std::string & ClientSessionManager::owner() const noexcept
{
  return owner_;
}

bool ClientSessionManager::owns_or_unlocked(const std::string & client_ip) const noexcept
{
  return !config_.lock_enabled || (!owner_.empty() && owner_ == client_ip);
}

void ClientSessionManager::release_if_expired(const double monotonic_now_sec)
{
  if (!owner_.empty() && monotonic_now_sec - last_seen_sec_ >= config_.timeout_sec) {
    owner_.clear();
    last_seen_sec_ = 0.0;
  }
}

}  // namespace bw_teleop::manager
