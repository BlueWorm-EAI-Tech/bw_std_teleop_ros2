#include <limits>

#include <gtest/gtest.h>

#include "bw_std_control/command_safety.hpp"

namespace bw_std_control
{
namespace
{

TEST(CommandSafety, RejectsNonFiniteAndOutOfRangeCommands)
{
  StandardCommand command{};
  CommandLimits limits{};
  limits.lower.fill(-1.0);
  limits.upper.fill(1.0);
  limits.velocity.fill(0.5);

  EXPECT_TRUE(command_within_limits(command, limits));
  command.position[2] = 1.01;
  EXPECT_FALSE(command_within_limits(command, limits));
  command.position[2] = 0.0;
  command.base_velocity[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(command_within_limits(command, limits));
}

TEST(CommandSafety, ClampsEachJointToSingleCycleIncrement)
{
  StandardCommand previous{};
  StandardCommand requested{};
  requested.position.fill(1.0);
  CommandLimits limits{};
  limits.lower.fill(-2.0);
  limits.upper.fill(2.0);
  limits.velocity.fill(0.5);

  ASSERT_TRUE(limit_command_step(requested, previous, limits, 0.1));
  for (const double position : requested.position) {
    EXPECT_NEAR(position, 0.05, 1e-12);
  }
}

TEST(CommandSafety, RejectsInvalidElapsedTimeOrVelocityLimit)
{
  StandardCommand previous{};
  StandardCommand requested{};
  CommandLimits limits{};
  limits.lower.fill(-1.0);
  limits.upper.fill(1.0);
  limits.velocity.fill(1.0);

  EXPECT_FALSE(limit_command_step(requested, previous, limits, 0.0));
  limits.velocity[5] = -1.0;
  EXPECT_FALSE(limit_command_step(requested, previous, limits, 0.1));
}

}  // namespace
}  // namespace bw_std_control
