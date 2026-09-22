#ifndef BW_KINEMATICS__ALGORITHM__STANDARD_KINEMATICS_TYPES_HPP_
#define BW_KINEMATICS__ALGORITHM__STANDARD_KINEMATICS_TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace bw_kinematics
{

constexpr std::size_t kStandardArmDof = 14U;
constexpr std::size_t kStandardSingleArmDof = 7U;
constexpr std::size_t kTaskDimension = 6U;

inline constexpr std::array<std::string_view, kStandardArmDof> kStandardArmJointNames{
  "A_left_Degree1_joint", "A_left_Degree2_joint", "A_left_Degree3_joint",
  "A_left_Degree4_joint", "A_left_Degree5_joint", "A_left_Degree6_joint",
  "A_left_Degree7_joint", "A_right_Degree1_joint", "A_right_Degree2_joint",
  "A_right_Degree3_joint", "A_right_Degree4_joint", "A_right_Degree5_joint",
  "A_right_Degree6_joint", "A_right_Degree7_joint"};

inline constexpr std::string_view kStandardRootFrame{"C_Link"};
inline constexpr std::string_view kStandardLeftEndEffectorFrame{"A_left_Degree7_link"};
inline constexpr std::string_view kStandardRightEndEffectorFrame{"A_right_Degree7_link"};
inline constexpr std::string_view kStandardLiftJointName{"C_joint"};

enum class ArmSide : std::uint8_t
{
  left = 0U,
  right = 1U,
};

struct CartesianPose
{
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct BimanualPose
{
  CartesianPose left;
  CartesianPose right;
};

using BimanualPoseTarget = BimanualPose;

struct IkConfig
{
  std::string root_frame{std::string{kStandardRootFrame}};
  std::string left_end_effector_frame{std::string{kStandardLeftEndEffectorFrame}};
  std::string right_end_effector_frame{std::string{kStandardRightEndEffectorFrame}};
  std::string locked_lift_joint{std::string{kStandardLiftJointName}};
  std::size_t max_iterations{30U};
  double continuity_weight{0.1};
  double max_position_residual{0.03};
  double max_orientation_residual{0.25};
};

struct ForwardKinematicsResult
{
  bool success{false};
  CartesianPose pose;
  std::string message;
};

struct BimanualForwardKinematicsResult
{
  bool success{false};
  BimanualPose pose;
  std::string message;
};

struct JacobianResult
{
  bool success{false};
  std::size_t rows{0U};
  std::size_t columns{0U};
  // 行优先: 线速度 x/y/z, 角速度 x/y/z。
  std::vector<double> values;
  std::string message;
};

struct IkSolveResult
{
  bool success{false};
  std::vector<double> positions;
  std::size_t iterations{0U};
  double left_position_error{0.0};
  double right_position_error{0.0};
  double left_orientation_error{0.0};
  double right_orientation_error{0.0};
  std::string message;
};

bool is_finite_pose(const CartesianPose & pose) noexcept;
bool is_valid_pose(const CartesianPose & pose) noexcept;
bool is_valid_bimanual_target(const BimanualPoseTarget & target) noexcept;

namespace detail
{
class StandardKinematicsBackend;
}

class StandardKinematicsModel
{
public:
  StandardKinematicsModel();
  ~StandardKinematicsModel();

  StandardKinematicsModel(const StandardKinematicsModel &) = delete;
  StandardKinematicsModel & operator=(const StandardKinematicsModel &) = delete;
  StandardKinematicsModel(StandardKinematicsModel &&) noexcept;
  StandardKinematicsModel & operator=(StandardKinematicsModel &&) noexcept;

  bool init(const std::string & urdf_xml, std::string & error_message);
  [[nodiscard]] bool is_initialized() const noexcept;
  [[nodiscard]] std::vector<std::string> joint_names() const;
  [[nodiscard]] std::vector<double> lower_limits() const;
  [[nodiscard]] std::vector<double> upper_limits() const;

private:
  friend class StandardFkSolver;
  friend class StandardIkSolver;

  std::shared_ptr<detail::StandardKinematicsBackend> backend_;
};

class StandardFkSolver
{
public:
  StandardFkSolver();
  ~StandardFkSolver();

  StandardFkSolver(const StandardFkSolver &) = delete;
  StandardFkSolver & operator=(const StandardFkSolver &) = delete;
  StandardFkSolver(StandardFkSolver &&) noexcept;
  StandardFkSolver & operator=(StandardFkSolver &&) noexcept;

  bool init(
    std::shared_ptr<const StandardKinematicsModel> model, std::string & error_message);
  [[nodiscard]] bool is_initialized() const noexcept;
  [[nodiscard]] std::vector<std::string> joint_names() const;
  [[nodiscard]] ForwardKinematicsResult get_ee_coordinate(
    ArmSide side, const std::vector<double> & positions) const;
  [[nodiscard]] BimanualForwardKinematicsResult get_bimanual_pose(
    const std::vector<double> & positions) const;
  [[nodiscard]] JacobianResult get_jacobian(
    ArmSide side, const std::vector<double> & positions) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class StandardIkSolver
{
public:
  StandardIkSolver();
  ~StandardIkSolver();

  StandardIkSolver(const StandardIkSolver &) = delete;
  StandardIkSolver & operator=(const StandardIkSolver &) = delete;
  StandardIkSolver(StandardIkSolver &&) noexcept;
  StandardIkSolver & operator=(StandardIkSolver &&) noexcept;

  bool init(
    std::shared_ptr<const StandardKinematicsModel> model,
    const IkConfig & config,
    std::string & error_message);
  [[nodiscard]] bool is_initialized() const noexcept;
  [[nodiscard]] IkSolveResult solve(
    const BimanualPoseTarget & target, const std::vector<double> & seed) const;

private:
  class Impl;
  IkSolveResult solve_locked(
    const BimanualPoseTarget & target, const std::vector<double> & seed) const;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bw_kinematics

#endif  // BW_KINEMATICS__ALGORITHM__STANDARD_KINEMATICS_TYPES_HPP_
