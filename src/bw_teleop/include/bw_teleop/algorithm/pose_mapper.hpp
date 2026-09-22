#pragma once

#include <array>
#include <cstddef>

namespace bw_teleop::algorithm
{

enum class Side : std::size_t
{
  left = 0,
  right = 1,
};

enum class ControlMode
{
  relative,
  absolute,
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct Pose
{
  Vector3 position;
  Quaternion orientation;
};

struct WorkspaceLimits
{
  Vector3 minimum{0.10, -0.80, -0.60};
  Vector3 maximum{0.90, 0.80, 0.50};
};

struct PoseMapperConfig
{
  std::array<Pose, 2> reset_poses{
    Pose{{0.40, 0.25, -0.15}, {}},
    Pose{{0.40, -0.25, -0.15}, {}}};
  WorkspaceLimits workspace;
  double relative_position_scale{1.0};
  double absolute_position_scale{1.0};
};

bool is_finite_pose(const Pose & pose) noexcept;
bool is_valid_pose(const Pose & pose) noexcept;

/**
 * @brief 将双手位姿映射为受工作空间约束的末端目标。
 */
class PoseMapper
{
public:
  explicit PoseMapper(PoseMapperConfig config);

  void set_control_mode(ControlMode mode) noexcept;
  ControlMode control_mode() const noexcept;
  [[nodiscard]] bool update(
    Side side, const Pose & hand_pose, bool clutch_pressed,
    const Pose & measured_target, bool measured_target_valid) noexcept;
  void release_clutch(Side side) noexcept;
  void reset(Side side) noexcept;
  // 直接设定目标(用于分步复位等内部序列), 同时重置相对运动基准。
  void set_target(Side side, const Pose & pose) noexcept;
  Pose target(Side side) const noexcept;

private:
  struct HandState
  {
    bool clutch_active{false};
    Pose hand_baseline;
    Pose target_baseline;
    Pose target;
  };

  static std::size_t index(Side side) noexcept;
  static Quaternion normalize(const Quaternion & value) noexcept;
  static Quaternion conjugate(const Quaternion & value) noexcept;
  static Quaternion multiply(
    const Quaternion & left, const Quaternion & right) noexcept;
  Pose clamp_to_workspace(const Pose & pose) const noexcept;

  PoseMapperConfig config_;
  std::array<HandState, 2> hands_;
  ControlMode control_mode_{ControlMode::relative};
};

}  // namespace bw_teleop::algorithm
