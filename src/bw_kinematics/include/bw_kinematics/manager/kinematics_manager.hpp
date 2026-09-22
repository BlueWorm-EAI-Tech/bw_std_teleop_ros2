#ifndef BW_KINEMATICS__MANAGER__KINEMATICS_MANAGER_HPP_
#define BW_KINEMATICS__MANAGER__KINEMATICS_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "bw_kinematics/algorithm/standard_fk_solver.hpp"
#include "bw_kinematics/algorithm/standard_ik_solver.hpp"
#include "bw_kinematics/algorithm/standard_kinematics_model.hpp"
#include "bw_kinematics/algorithm/standard_kinematics_types.hpp"

namespace bw_kinematics
{

struct KinematicsManagerConfig
{
  IkConfig ik;
  double feedback_timeout_sec{0.2};
  double max_joint_step_rad{0.35};
};

struct KinematicsCommand
{
  bool has_command{false};
  bool requested_arm_succeeded{false};
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::string message;
};

/**
 * @brief 管理 Standard 双臂反馈、目标、联合 IK 和安全保持。
 *
 * Manager 输入输出均为 Standard 14 轴关节名和 SI 单位.
 */
class KinematicsManager
{
public:
  KinematicsManager();

  bool configure(
    const std::string & standard_ik_urdf_xml,
    const KinematicsManagerConfig & config,
    std::string & error_message);

  bool update_measured_state(
    const std::vector<std::string> & names,
    const std::vector<double> & positions,
    double now_sec);

  [[nodiscard]] KinematicsCommand solve_target(
    ArmSide side,
    const CartesianPose & target,
    double now_sec);
  [[nodiscard]] ForwardKinematicsResult measured_pose(
    ArmSide side, double now_sec) const;
  [[nodiscard]] bool ready(double now_sec) const noexcept;
  [[nodiscard]] bool copy_measured_positions(
    std::vector<double> & out, double now_sec) const noexcept;
  [[nodiscard]] bool clamp_to_limits(std::vector<double> & positions) const noexcept;

private:
  [[nodiscard]] bool feedback_is_fresh(double now_sec) const noexcept;
  [[nodiscard]] KinematicsCommand make_command(
    bool requested_arm_succeeded, const std::string & message,
    double now_sec) const;
  void invalidate_measured_state() noexcept;
  void initialize_command_from_measurement() noexcept;
  [[nodiscard]] bool finite_and_bounded(
    const std::vector<double> & positions) const noexcept;
  [[nodiscard]] bool exceeds_joint_step(
    const std::vector<double> & positions) const noexcept;
  [[nodiscard]] bool residual_improves(
    const BimanualPoseTarget & targets,
    const BimanualForwardKinematicsResult & current,
    const IkSolveResult & candidate) const noexcept;

  std::shared_ptr<StandardKinematicsModel> model_;
  StandardFkSolver fk_solver_;
  StandardIkSolver ik_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> measured_positions_;
  std::vector<double> command_positions_;
  double feedback_timeout_sec_{0.2};
  double max_joint_step_rad_{0.35};
  double max_position_residual_{0.03};
  double max_orientation_residual_{0.25};
  double last_feedback_time_sec_{0.0};
  bool feedback_received_{false};
  bool measured_complete_{false};
  bool command_initialized_{false};
  bool configured_{false};
};

}  // namespace bw_kinematics

#endif  // BW_KINEMATICS__MANAGER__KINEMATICS_MANAGER_HPP_
