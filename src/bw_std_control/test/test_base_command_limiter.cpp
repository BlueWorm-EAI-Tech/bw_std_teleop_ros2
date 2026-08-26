#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "bw_std_control/base_command_limiter.hpp"

namespace bw_std_control
{
namespace
{

using namespace std::chrono_literals;

TEST(BaseCommandLimiter, ClampsFiniteVelocityAndExpiresToZero)
{
  BaseCommandLimiter limiter{{0.5, 0.4, 0.8}, 100ms};
  const auto now = std::chrono::steady_clock::time_point{1s};

  const auto filtered = limiter.filter({1.0, -0.2, -2.0});
  ASSERT_TRUE(filtered.has_value());
  EXPECT_EQ(*filtered, (std::array<double, 3>{0.5, -0.2, -0.8}));
  EXPECT_FALSE(limiter.expired(now, now + 50ms));
  EXPECT_TRUE(limiter.expired(now, now + 101ms));
}

TEST(BaseCommandLimiter, RejectsInvalidInputWithoutMutableHistory)
{
  const BaseCommandLimiter limiter{{0.5, 0.4, 0.8}, 100ms};

  EXPECT_FALSE(limiter.filter(
      {0.1, std::numeric_limits<double>::quiet_NaN(), 0.3}).has_value());
}

TEST(BaseCommandLimiter, RejectsInvalidConfiguration)
{
  EXPECT_THROW((BaseCommandLimiter{{0.5, 0.0, 0.8}, 100ms}), std::invalid_argument);
  EXPECT_THROW((BaseCommandLimiter{{0.5, 0.4, 0.8}, 0ms}), std::invalid_argument);
}

TEST(BaseCommandLimiter, ConstFilterSupportsConcurrentLifecycleIndependentUse)
{
  const BaseCommandLimiter limiter{{0.5, 0.4, 0.8}, 100ms};
  std::atomic_bool valid{true};
  std::vector<std::thread> workers;
  for (std::size_t worker = 0; worker < 4U; ++worker) {
    workers.emplace_back([&limiter, &valid]() {
      for (std::size_t iteration = 0; iteration < 1000U; ++iteration) {
        const auto value = limiter.filter({1.0, -1.0, 2.0});
        if (!value.has_value() ||
          *value != std::array<double, 3>{0.5, -0.4, 0.8})
        {
          valid.store(false, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }
  EXPECT_TRUE(valid.load(std::memory_order_relaxed));
}

}  // namespace
}  // namespace bw_std_control
