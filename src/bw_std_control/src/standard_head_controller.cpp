#include "bw_std_control/standard_head_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "hardware_interface/version.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

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

controller_interface::CallbackReturn StandardHeadController::on_init()
{
  try {
    auto_declare<double>("command_timeout_sec", 0.2);
    auto_declare<std::vector<double>>("safe_position", {0.0, 0.0, 0.0});
    auto_declare<std::vector<double>>("lower_limits", {-0.785, -1.570, -0.349});
    auto_declare<std::vector<double>>("upper_limits", {0.524, 1.570, 0.349});
    auto_declare<std::string>("command_topic", "/Teleop/head_pose");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "声明头部参数失败: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
StandardHeadController::command_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"head/pitch", "head/yaw", "head/roll"}};
}

controller_interface::InterfaceConfiguration
StandardHeadController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn StandardHeadController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const double timeout_sec = get_node()->get_parameter("command_timeout_sec").as_double();
  const auto safe = get_node()->get_parameter("safe_position").as_double_array();
  const auto lower = get_node()->get_parameter("lower_limits").as_double_array();
  const auto upper = get_node()->get_parameter("upper_limits").as_double_array();
  const std::string topic = get_node()->get_parameter("command_topic").as_string();
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0 || safe.size() != 3U ||
    lower.size() != 3U || upper.size() != 3U || topic.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "头部参数长度、超时或 topic 非法");
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t index = 0; index < 3U; ++index) {
    lower_limits_[index] = lower[index];
    upper_limits_[index] = upper[index];
    safe_position_[index] = safe[index];
    if (!std::isfinite(lower_limits_[index]) || !std::isfinite(upper_limits_[index]) ||
      lower_limits_[index] > upper_limits_[index] ||
      lower_limits_[index] > 0.0 || upper_limits_[index] < 0.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "头部限位必须有限、有序且包含安全零位");
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  if (!valid_position(safe_position_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "头部安全位置超出 pitch/yaw/roll 限位");
    return controller_interface::CallbackReturn::ERROR;
  }
  command_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>{timeout_sec});
  if (command_timeout_ <= std::chrono::steady_clock::duration::zero()) {
    RCLCPP_ERROR(get_node()->get_logger(), "头部 command_timeout_sec 精度不足");
    return controller_interface::CallbackReturn::ERROR;
  }
  try {
    const std::string expanded_topic = rclcpp::expand_topic_or_service_name(
      topic, get_node()->get_name(), get_node()->get_namespace());
    command_subscription_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      expanded_topic, rclcpp::SystemDefaultsQoS(),
      std::bind(&StandardHeadController::command_callback, this, std::placeholders::_1));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "头部 command_topic 非法: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
  held_position_ = safe_position_;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StandardHeadController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != 3U) {
    RCLCPP_ERROR(get_node()->get_logger(), "头部 controller 必须获得三个 command interface");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::array<double, 3> existing{};
  held_position_ = read_existing_command(existing) && valid_position(existing) ? existing : safe_position_;
  command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
  if (!write_command(held_position_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "头部 controller 激活时无法写入安全保持位置");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn StandardHeadController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
  return write_command(held_position_) ? controller_interface::CallbackReturn::SUCCESS :
         controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn StandardHeadController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  command_subscription_.reset();
  command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
  held_position_ = safe_position_;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type StandardHeadController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const RealtimeHeadCommand command{*command_buffer_.readFromRT()};
  const auto now = std::chrono::steady_clock::now();
  if (!command.valid || now < command.received_at || now - command.received_at > command_timeout_) {
    return write_command(held_position_) ? controller_interface::return_type::OK :
           controller_interface::return_type::ERROR;
  }
  const auto clamped = clamp_position(command.position);
  if (!std::all_of(command.position.begin(), command.position.end(), [](const double value) {
      return std::isfinite(value);
    }) || !write_command(clamped))
  {
    static_cast<void>(write_command(held_position_));
    return controller_interface::return_type::ERROR;
  }
  held_position_ = clamped;
  return controller_interface::return_type::OK;
}

void StandardHeadController::command_callback(sensor_msgs::msg::JointState::SharedPtr message)
{
  if (message == nullptr) {
    command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
    return;
  }
  if (message->name.size() != message->position.size()) {
    command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "拒绝 name/position 数量不一致的头部命令");
    return;
  }
  std::array<double, 3> position{};
  std::array<bool, 3> found{};
  for (std::size_t message_index = 0; message_index < message->name.size(); ++message_index) {
    const auto found_joint = std::find(
      joint_names_.begin(), joint_names_.end(), message->name[message_index]);
    if (found_joint == joint_names_.end()) {
      continue;
    }
    const std::size_t index = static_cast<std::size_t>(
      std::distance(joint_names_.begin(), found_joint));
    if (found[index] || message_index >= message->position.size() ||
      !std::isfinite(message->position[message_index]))
    {
      command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
      return;
    }
    position[index] = message->position[message_index];
    found[index] = true;
  }
  if (!std::all_of(found.begin(), found.end(), [](const bool value) {return value;}) ||
    !std::all_of(position.begin(), position.end(), [](const double value) {
      return std::isfinite(value);
    }) || !valid_position(position))
  {
    command_buffer_.writeFromNonRT(RealtimeHeadCommand{});
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "拒绝不完整、非有限或越过限位的头部命令");
    return;
  }
  RealtimeHeadCommand command;
  command.position = position;
  command.received_at = std::chrono::steady_clock::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

bool StandardHeadController::write_command(const std::array<double, 3> & position)
{
  if (command_interfaces_.size() != position.size() || !valid_position(position)) {
    return false;
  }
  bool written = true;
  for (std::size_t index = 0; index < position.size(); ++index) {
    written = set_command_value(command_interfaces_[index], position[index]) && written;
  }
  return written;
}

bool StandardHeadController::read_existing_command(std::array<double, 3> & position) const
{
  if (command_interfaces_.size() != position.size()) {
    return false;
  }
  for (std::size_t index = 0; index < position.size(); ++index) {
    position[index] = command_interfaces_[index].get_value();
  }
  return true;
}

bool StandardHeadController::valid_position(const std::array<double, 3> & position) const noexcept
{
  for (std::size_t index = 0; index < position.size(); ++index) {
    if (!std::isfinite(position[index]) || !std::isfinite(lower_limits_[index]) ||
      !std::isfinite(upper_limits_[index]) || lower_limits_[index] > upper_limits_[index] ||
      position[index] < lower_limits_[index] || position[index] > upper_limits_[index])
    {
      return false;
    }
  }
  return true;
}

std::array<double, 3> StandardHeadController::clamp_position(
  const std::array<double, 3> & position) const noexcept
{
  std::array<double, 3> clamped{};
  for (std::size_t index = 0; index < position.size(); ++index) {
    clamped[index] = std::clamp(position[index], lower_limits_[index], upper_limits_[index]);
  }
  return clamped;
}

}  // namespace bw_std_control

PLUGINLIB_EXPORT_CLASS(
  bw_std_control::StandardHeadController, controller_interface::ControllerInterface)
