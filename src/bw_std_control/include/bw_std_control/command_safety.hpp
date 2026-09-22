#ifndef BW_STD_CONTROL__COMMAND_SAFETY_HPP_
#define BW_STD_CONTROL__COMMAND_SAFETY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "bw_std_control/standard_mapping.hpp"

namespace bw_std_control
{

constexpr double kMaxArmStartupLimitToleranceRad = 0.002;
// 夹爪零位原始值实测可低至约 -1.5e-5 m; 上限取 1 mm 以接纳真实零偏。
constexpr double kMaxGripperStartupLimitToleranceM = 0.001;
// 头部边界容差: 头显顶到机械停止位时固件可回报高出模型限位约 2e-4 rad。
constexpr double kHeadBoundaryToleranceRad = 0.01;

struct CommandLimits
{
  std::array<double, kStandardJointCount> lower{};
  std::array<double, kStandardJointCount> upper{};
  std::array<double, kStandardJointCount> velocity{};
  std::array<double, kHeadInterfaceCount> head_lower{
    kHeadPitchMin, kHeadYawMin, kHeadRollMin};
  std::array<double, kHeadInterfaceCount> head_upper{
    kHeadPitchMax, kHeadYawMax, kHeadRollMax};
  std::array<double, kHeadInterfaceCount> head_velocity{1.0, 1.0, 1.0};
};

enum class CommandSafetyFailure : std::uint8_t
{
  none,
  joint_position,
  base_velocity,
  head_position,
  startup_tolerance,
  recovery_direction
};

struct CommandSafetyDiagnostic
{
  CommandSafetyFailure failure{CommandSafetyFailure::none};
  std::size_t index{0U};
  double value{0.0};
  double lower{0.0};
  double upper{0.0};

  [[nodiscard]] bool ok() const noexcept;
};

enum class LimitRecoveryDirection : std::uint8_t
{
  none,
  decreasing,
  increasing
};

[[nodiscard]] CommandSafetyDiagnostic check_command_limits(
  const StandardCommand & command, const CommandLimits & limits) noexcept;
[[nodiscard]] CommandSafetyDiagnostic check_auxiliary_limits(
  const StandardCommand & command, const CommandLimits & limits) noexcept;
[[nodiscard]] std::array<double, kStandardJointCount>
make_standard_startup_limit_tolerance(
  double arm_tolerance_rad, double gripper_tolerance_m) noexcept;

class StartupLimitRecovery final
{
public:
  [[nodiscard]] CommandSafetyDiagnostic begin(
    const StandardCommand & measured, const CommandLimits & limits,
    const std::array<double, kStandardJointCount> & tolerance) noexcept;
  [[nodiscard]] CommandSafetyDiagnostic validate(
    const StandardCommand & command, const StandardCommand & previous,
    const CommandLimits & limits) const noexcept;
  void note_command_queued(
    const StandardCommand & command, const StandardCommand & previous,
    const CommandLimits & limits, std::uint64_t sequence) noexcept;
  [[nodiscard]] CommandSafetyDiagnostic update_measured_state(
    const StandardState & measured, const CommandLimits & limits,
    const std::array<bool, kStandardJointCount> & entry_commands_completed,
    std::uint64_t feedback_sequence) noexcept;
  void reset() noexcept;
  [[nodiscard]] bool has_active_recovery() const noexcept;
  [[nodiscard]] LimitRecoveryDirection direction(std::size_t joint) const noexcept;
  [[nodiscard]] std::uint64_t entry_command_sequence(std::size_t joint) const noexcept;

private:
  std::array<LimitRecoveryDirection, kStandardJointCount> directions_{};
  // 逐关节边界容差: 量测与命令路径都按该容差接纳边界噪声。
  std::array<double, kStandardJointCount> tolerance_{};
  std::array<std::uint64_t, kStandardJointCount> entry_command_sequences_{};
  std::array<bool, kStandardJointCount> entry_commands_completed_{};
  std::array<std::uint64_t, kStandardJointCount>
  entry_completion_feedback_sequences_{};
};

bool command_within_limits(
  const StandardCommand & command, const CommandLimits & limits,
  double tolerance = 0.0) noexcept;// 有限性检查: 位置、头部和底盘速度均必须为有限值。
bool command_is_finite(const StandardCommand & command) noexcept;
// 将位置与头部命令截断到限位内(仅保留兼容接口, 上电路径不再依赖截断放行)。
void clamp_command_to_limits(
  StandardCommand & command, const CommandLimits & limits) noexcept;
bool limit_command_step(
  StandardCommand & command, const StandardCommand & previous,
  const CommandLimits & limits, double elapsed_sec) noexcept;

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__COMMAND_SAFETY_HPP_
