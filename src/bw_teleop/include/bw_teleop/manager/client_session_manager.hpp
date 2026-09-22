#pragma once

#include <string>

namespace bw_teleop::manager
{

struct ClientSessionConfig
{
  bool lock_enabled{true};
  double timeout_sec{3.0};
};

class ClientSessionManager
{
public:
  explicit ClientSessionManager(ClientSessionConfig config);

  [[nodiscard]] bool accept_valid_packet(
    const std::string & client_ip, double monotonic_now_sec);
  [[nodiscard]] const std::string & owner() const noexcept;
  [[nodiscard]] bool owns_or_unlocked(const std::string & client_ip) const noexcept;
  // 检查 owner 并释放过期会话, 不推进 last_seen_sec_。
  [[nodiscard]] bool can_accept_valid_packet(
    const std::string & client_ip, double monotonic_now_sec);

private:
  void release_if_expired(double monotonic_now_sec);

  ClientSessionConfig config_;
  std::string owner_;
  double last_seen_sec_{0.0};
};

}  // namespace bw_teleop::manager
