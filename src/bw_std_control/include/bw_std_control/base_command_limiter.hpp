#ifndef BW_STD_CONTROL__BASE_COMMAND_LIMITER_HPP_
#define BW_STD_CONTROL__BASE_COMMAND_LIMITER_HPP_

#include <array>
#include <chrono>
#include <optional>

namespace bw_std_control
{

class BaseCommandLimiter
{
public:
  BaseCommandLimiter(
    std::array<double, 3> limits, std::chrono::steady_clock::duration timeout);

  std::optional<std::array<double, 3>> filter(
    const std::array<double, 3> & requested) const noexcept;
  bool expired(
    std::chrono::steady_clock::time_point received_at,
    std::chrono::steady_clock::time_point now) const noexcept;

private:
  const std::array<double, 3> limits_;
  const std::chrono::steady_clock::duration timeout_;
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__BASE_COMMAND_LIMITER_HPP_
