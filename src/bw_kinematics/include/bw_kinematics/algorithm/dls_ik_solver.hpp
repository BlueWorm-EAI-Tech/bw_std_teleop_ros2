#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace bw_kinematics
{

struct CartesianPose
{
  std::array<double, 3> position{};
  std::array<double, 4> orientation_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct DlsIkConfig
{
  std::string root_link{"C_Link"};
  std::string tip_link;
  std::size_t max_iterations{40U};
  double damping{0.05};
  double max_joint_step{0.04};
  double position_tolerance{0.001};
  double orientation_tolerance{0.008726646259971648};
  double joint_centering_gain{0.02};
};

struct ForwardKinematicsResult
{
  bool success{false};
  CartesianPose pose;
  std::string message;
};

struct JacobianResult
{
  bool success{false};
  std::size_t rows{0U};
  std::size_t columns{0U};
  std::vector<double> values;
  std::string message;
};

struct IkSolveResult
{
  bool success{false};
  std::vector<double> positions;
  std::size_t iterations{0U};
  double position_error{0.0};
  double orientation_error{0.0};
  std::string message;
};

/**
 * @brief 单臂 KDL 运动学与阻尼最小二乘逆解算法。
 *
 * 本类只依赖 STL、Eigen、URDF 与 KDL，不包含 ROS 节点、消息或通信逻辑。
 */
class DlsIkSolver
{
public:
  DlsIkSolver();
  ~DlsIkSolver();

  DlsIkSolver(const DlsIkSolver &) = delete;
  DlsIkSolver & operator=(const DlsIkSolver &) = delete;
  DlsIkSolver(DlsIkSolver &&) noexcept;
  DlsIkSolver & operator=(DlsIkSolver &&) noexcept;

  bool configure(
    const std::string & robot_description,
    const DlsIkConfig & config,
    std::string & error_message);

  [[nodiscard]] bool is_configured() const noexcept;
  [[nodiscard]] std::vector<std::string> joint_names() const;
  [[nodiscard]] ForwardKinematicsResult compute_fk(
    const std::vector<double> & positions) const;
  [[nodiscard]] JacobianResult compute_jacobian(
    const std::vector<double> & positions) const;
  [[nodiscard]] IkSolveResult solve(
    const CartesianPose & target,
    const std::vector<double> & seed) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bw_kinematics
