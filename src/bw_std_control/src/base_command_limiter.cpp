#include "bw_std_control/base_command_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bw_std_control
{

BaseCommandLimiter::BaseCommandLimiter(
  const std::array<double, 3> limits, const std::chrono::steady_clock::duration timeout)
: limits_(limits), timeout_(timeout)
{
  if (timeout_ <= std::chrono::steady_clock::duration::zero() ||
    !std::all_of(
      limits_.begin(), limits_.end(),
      [](const double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument("base limits and timeout must be finite positive values");
  }
}

std::optional<std::array<double, 3>> BaseCommandLimiter::filter(
  const std::array<double, 3> & requested) const noexcept
{
  if (!std::all_of(
      requested.begin(), requested.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return std::nullopt;
  }
  std::array<double, 3> filtered{};
  for (std::size_t index = 0; index < requested.size(); ++index) {
    filtered[index] = std::clamp(requested[index], -limits_[index], limits_[index]);
  }
  return filtered;
}

bool BaseCommandLimiter::expired(
  const std::chrono::steady_clock::time_point received_at,
  const std::chrono::steady_clock::time_point now) const noexcept
{
  return now < received_at || now - received_at > timeout_;
}

}  // namespace bw_std_control
