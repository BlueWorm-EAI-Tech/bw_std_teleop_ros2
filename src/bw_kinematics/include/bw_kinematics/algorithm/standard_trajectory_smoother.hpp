#ifndef BW_KINEMATICS__ALGORITHM__STANDARD_TRAJECTORY_SMOOTHER_HPP_
#define BW_KINEMATICS__ALGORITHM__STANDARD_TRAJECTORY_SMOOTHER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace bw_kinematics
{

/**
 * @brief Standard 关节空间在线平滑配置.
 *
 * 周期为高频输出周期, 三组上限为 Ruckig 的速度/加速度/jerk 约束; 其余参数控制低频目标
 * 的速度前馈与短时预测.
 */
struct TrajectorySmootherConfig
{
  std::size_t dof{0U};
  double period_sec{0.005};
  double max_velocity_rad_s{6.0};
  double max_acceleration_rad_s2{20.0};
  double max_jerk_rad_s3{200.0};
  double target_velocity_scale{0.5};
  double max_target_velocity_rad_s{3.0};
  double target_velocity_filter_alpha{0.15};
  double target_prediction_max_time_sec{0.02};
};

struct TrajectorySmootherState
{
  bool valid{false};
  bool finished{false};
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;
};

/**
 * @brief 追踪移动目标的在线轨迹平滑器.
 *
 * update_target() 接收低频目标并估算目标速度前馈, step() 在每个高频周期推进一次
 * Ruckig 轨迹. 该类不含 ROS 依赖, 与 Ruckig 的接口细节由实现库内部承担.
 */
class StandardTrajectorySmoother
{
public:
  StandardTrajectorySmoother();
  ~StandardTrajectorySmoother();

  StandardTrajectorySmoother(const StandardTrajectorySmoother &) = delete;
  StandardTrajectorySmoother & operator=(const StandardTrajectorySmoother &) = delete;
  StandardTrajectorySmoother(StandardTrajectorySmoother &&) noexcept;
  StandardTrajectorySmoother & operator=(StandardTrajectorySmoother &&) noexcept;

  bool init(const TrajectorySmootherConfig & config, std::string & error_message);
  [[nodiscard]] bool is_initialized() const noexcept;
  [[nodiscard]] bool is_tracking() const noexcept;

  bool reset(const std::vector<double> & positions, std::string & error_message);
  bool set_constraints(
    double max_velocity_rad_s, double max_acceleration_rad_s2, double max_jerk_rad_s3,
    std::string & error_message);
  bool update_target(
    const std::vector<double> & target_positions, std::string & error_message);
  [[nodiscard]] TrajectorySmootherState step();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bw_kinematics

#endif  // BW_KINEMATICS__ALGORITHM__STANDARD_TRAJECTORY_SMOOTHER_HPP_
