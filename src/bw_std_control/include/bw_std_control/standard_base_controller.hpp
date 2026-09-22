#ifndef BW_STD_CONTROL__STANDARD_BASE_CONTROLLER_HPP_
#define BW_STD_CONTROL__STANDARD_BASE_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/bool.hpp"

#include "bw_std_control/base_command_limiter.hpp"

namespace bw_std_control
{

struct RealtimeBaseCommand
{
  std::array<double, 3> velocity{};
  std::chrono::steady_clock::time_point received_at{};
  bool valid{false};
};

class StandardBaseController final : public controller_interface::ControllerInterface
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
  void command_callback(geometry_msgs::msg::TwistStamped::SharedPtr message);
  void control_active_callback(std_msgs::msg::Bool::SharedPtr message);
  bool write_command(const std::array<double, 3> & velocity);
  bool write_zero_command();
  // 写入 safety/power 控制活跃状态; 断流后自动置 0。
  bool write_safety_power(bool active);

  realtime_tools::RealtimeBuffer<RealtimeBaseCommand> command_buffer_{};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_subscription_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_active_subscription_{};
  std::shared_ptr<const BaseCommandLimiter> limiter_{};
  std::chrono::steady_clock::duration command_timeout_{std::chrono::milliseconds{100}};
  std::chrono::steady_clock::duration control_active_timeout_{std::chrono::milliseconds{500}};
  std::chrono::steady_clock::time_point control_active_stamp_{};
  bool control_active_request_{false};
  std::array<double, 3> limits_{0.5, 0.5, 0.8};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__STANDARD_BASE_CONTROLLER_HPP_
