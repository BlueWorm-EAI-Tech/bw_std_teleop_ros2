#ifndef BW_STD_CONTROL__STANDARD_HEAD_CONTROLLER_HPP_
#define BW_STD_CONTROL__STANDARD_HEAD_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace bw_std_control
{

struct RealtimeHeadCommand
{
  std::array<double, 3> position{};
  std::chrono::steady_clock::time_point received_at{};
  bool valid{false};
};

class StandardHeadController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void command_callback(sensor_msgs::msg::JointState::SharedPtr message);
  bool write_command(const std::array<double, 3> & position);
  bool read_existing_command(std::array<double, 3> & position) const;
  bool valid_position(const std::array<double, 3> & position) const noexcept;
  std::array<double, 3> clamp_position(const std::array<double, 3> & position) const noexcept;

  std::array<std::string, 3> joint_names_{
    "head_pitch_joint", "head_yaw_joint", "head_roll_joint"};
  std::array<double, 3> lower_limits_{-0.785, -1.570, -0.349};
  std::array<double, 3> upper_limits_{0.524, 1.570, 0.349};
  std::array<double, 3> safe_position_{};
  std::array<double, 3> held_position_{};
  std::chrono::steady_clock::duration command_timeout_{std::chrono::milliseconds{200}};

  realtime_tools::RealtimeBuffer<RealtimeHeadCommand> command_buffer_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_{};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__STANDARD_HEAD_CONTROLLER_HPP_
