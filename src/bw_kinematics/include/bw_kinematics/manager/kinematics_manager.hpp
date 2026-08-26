#pragma once

#include <array>
#include <string>
#include <vector>

#include "bw_kinematics/algorithm/dls_ik_solver.hpp"

namespace bw_kinematics
{

enum class ArmSide
{
  left,
  right
};

struct KinematicsManagerConfig
{
  DlsIkConfig left_arm;
  DlsIkConfig right_arm;
  double feedback_timeout_sec{0.2};
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
 * @brief 调度左右臂求解并维护最后有效的双臂目标。
 */
class KinematicsManager
{
public:
  KinematicsManager();

  bool configure(
    const std::string & robot_description,
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
  [[nodiscard]] ForwardKinematicsResult measured_pose(ArmSide side, double now_sec) const;
  [[nodiscard]] bool ready(double now_sec) const noexcept;

private:
  struct ArmState
  {
    std::vector<double> measured_positions;
    std::vector<double> command_positions;
    std::vector<bool> measured;
    bool command_initialized{false};
  };

  [[nodiscard]] static bool arm_ready(const ArmState & state) noexcept;
  void invalidate_measured_state() noexcept;
  void initialize_command_if_ready(ArmState & state);
  [[nodiscard]] KinematicsCommand make_command(
    bool requested_arm_succeeded,
    const std::string & message,
    double now_sec) const;

  DlsIkSolver left_solver_;
  DlsIkSolver right_solver_;
  ArmState left_state_;
  ArmState right_state_;
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  std::vector<std::string> joint_names_;
  double feedback_timeout_sec_{0.2};
  double last_feedback_time_sec_{0.0};
  bool feedback_received_{false};
  bool configured_{false};
};

}  // namespace bw_kinematics
