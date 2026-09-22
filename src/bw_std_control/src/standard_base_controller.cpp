#include "bw_std_control/standard_base_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "hardware_interface/version.h"
#include "pluginlib/class_list_macros.hpp"

namespace bw_std_control
{
namespace
{

bool set_command_value(
  hardware_interface::LoanedCommandInterface & command_interface,
  const double value)
{
#if HARDWARE_INTERFACE_VERSION_MAJOR >= 4
  return command_interface.set_value(value);
#else
  command_interface.set_value(value);
  return true;
#endif
}

}  // namespace

controller_interface::CallbackReturn StandardBaseController::on_init()
{
  try {
    auto_declare<double>("command_timeout_sec", 0.1);
    auto_declare<double>("max_vx", 0.5);
    auto_declare<double>("max_vy", 0.5);
    auto_declare<double>("max_wz", 0.8);
    auto_declare<std::string>("control_active_topic", "/teleop/control_active");
    auto_declare<double>("control_active_timeout_sec", 0.5);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "声明底盘参数失败: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StandardBaseController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"base/vx", "base/vy", "base/wz", "safety/power"}};
}

controller_interface::InterfaceConfiguration
StandardBaseController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn StandardBaseController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const double timeout_sec = get_node()->get_parameter("command_timeout_sec").as_double();
  limits_ = {
    get_node()->get_parameter("max_vx").as_double(),
    get_node()->get_parameter("max_vy").as_double(),
    get_node()->get_parameter("max_wz").as_double()};
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "底盘命令超时必须为有限正数");
    return controller_interface::CallbackReturn::ERROR;
  }
  command_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>{timeout_sec});
  try {
    auto limiter = std::make_shared<const BaseCommandLimiter>(limits_, command_timeout_);
    std::atomic_store(&limiter_, std::move(limiter));
  } catch (const std::invalid_argument & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "底盘限制参数无效: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  command_subscription_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/reference", rclcpp::SystemDefaultsQoS(),
    std::bind(&StandardBaseController::command_callback, this, std::placeholders::_1));
  const double control_active_timeout_sec =
    get_node()->get_parameter("control_active_timeout_sec").as_double();
  if (!std::isfinite(control_active_timeout_sec) || control_active_timeout_sec <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "遥操作控制超时必须为有限正数");
    return controller_interface::CallbackReturn::ERROR;
  }
  control_active_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>{control_active_timeout_sec});
  control_active_subscription_ = get_node()->create_subscription<std_msgs::msg::Bool>(
    get_node()->get_parameter("control_active_topic").as_string(), rclcpp::SystemDefaultsQoS(),
    std::bind(&StandardBaseController::control_active_callback, this, std::placeholders::_1));
  control_active_request_ = false;
  control_active_stamp_ = std::chrono::steady_clock::time_point{};
  command_buffer_.writeFromNonRT(RealtimeBaseCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StandardBaseController::on_activate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(RealtimeBaseCommand{});
  if (command_interfaces_.size() != 4U || !write_zero_command() || !write_safety_power(false)) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "激活时无法清零底盘 command interface 或初始化 safety/power");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StandardBaseController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const bool zeroed = write_zero_command();
  const bool power_cleared = write_safety_power(false);
  command_buffer_.writeFromNonRT(RealtimeBaseCommand{});
  return (zeroed && power_cleared) ? controller_interface::CallbackReturn::SUCCESS :
         controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn StandardBaseController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  command_subscription_.reset();
  std::atomic_store(&limiter_, std::shared_ptr<const BaseCommandLimiter>{});
  command_buffer_.writeFromNonRT(RealtimeBaseCommand{});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type StandardBaseController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const RealtimeBaseCommand command{*command_buffer_.readFromRT()};
  const auto now = std::chrono::steady_clock::now();
  const bool control_active = control_active_request_ &&
    now - control_active_stamp_ <= control_active_timeout_;
  const bool power_written = write_safety_power(control_active);
  if (!command.valid || now < command.received_at ||
    now - command.received_at > command_timeout_)
  {
    const bool zeroed = write_zero_command();
    return (zeroed && power_written) ? controller_interface::return_type::OK :
           controller_interface::return_type::ERROR;
  }
  if (!write_command(command.velocity) || !power_written) {
    static_cast<void>(write_zero_command());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void StandardBaseController::control_active_callback(std_msgs::msg::Bool::SharedPtr message)
{
  if (message == nullptr) {
    control_active_request_ = false;
    return;
  }
  control_active_request_ = message->data;
  control_active_stamp_ = std::chrono::steady_clock::now();
}

void StandardBaseController::command_callback(
  geometry_msgs::msg::TwistStamped::SharedPtr message)
{
  if (message == nullptr) {
    command_buffer_.writeFromNonRT(RealtimeBaseCommand{});
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const std::array<double, 3> requested{
    message->twist.linear.x, message->twist.linear.y, message->twist.angular.z};
  const auto limiter = std::atomic_load(&limiter_);
  const auto filtered = limiter ? limiter->filter(requested) : std::nullopt;
  if (!filtered.has_value()) {
    command_buffer_.writeFromNonRT(RealtimeBaseCommand{});
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "拒绝包含非有限值的底盘命令");
    return;
  }
  RealtimeBaseCommand command{};
  command.velocity = *filtered;
  command.received_at = now;
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

bool StandardBaseController::write_command(const std::array<double, 3> & velocity)
{
  if (command_interfaces_.size() < velocity.size()) {
    return false;
  }
  bool written = true;
  for (std::size_t index = 0; index < velocity.size(); ++index) {
    written = set_command_value(command_interfaces_[index], velocity[index]) && written;
  }
  return written;
}

bool StandardBaseController::write_zero_command()
{
  return write_command({0.0, 0.0, 0.0});
}

bool StandardBaseController::write_safety_power(const bool active)
{
  for (std::size_t index = 0; index < command_interfaces_.size(); ++index) {
    if (command_interfaces_[index].get_name() != "safety/power") {
      continue;
    }
    return set_command_value(command_interfaces_[index], active ? 1.0 : 0.0);
  }
  return false;
}

}  // namespace bw_std_control

PLUGINLIB_EXPORT_CLASS(
  bw_std_control::StandardBaseController, controller_interface::ControllerInterface)
