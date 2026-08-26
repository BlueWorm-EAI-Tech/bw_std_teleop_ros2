#include "bw_std_control/standard_system_hardware.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string>
#include <system_error>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace bw_std_control
{
namespace
{

const rclcpp::Logger kLogger = rclcpp::get_logger("bw_std_control");
constexpr std::size_t kMaxConsecutiveHandleAccessFailures = 10U;

const hardware_interface::ComponentInfo * find_component(
  const std::vector<hardware_interface::ComponentInfo> & components,
  const std::string_view name)
{
  const auto found = std::find_if(
    components.begin(), components.end(),
    [name](const auto & component) {return component.name == name;});
  return found == components.end() ? nullptr : &(*found);
}

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string_view name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [name](const auto & interface_info) {return interface_info.name == name;});
}

const hardware_interface::InterfaceInfo * find_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string_view name)
{
  const auto found = std::find_if(
    interfaces.begin(), interfaces.end(),
    [name](const auto & interface_info) {return interface_info.name == name;});
  return found == interfaces.end() ? nullptr : &(*found);
}

bool parse_finite_double(const std::string & text, double & value)
{
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_arm_raw_zero_hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  std::array<double, kArmJointCount> & values)
{
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ||
         parse_arm_raw_zero_parameter(found->second, values);
}

bool parse_arm_motor_indices_hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  std::array<std::size_t, kArmJointCount> & values)
{
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ||
         parse_arm_motor_indices_parameter(found->second, values);
}

bool parse_arm_direction_hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  std::array<double, kArmJointCount> & values)
{
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ||
         parse_arm_direction_parameter(found->second, values);
}

bool parse_positive_double(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  double & value)
{
  const auto found = info.hardware_parameters.find(name);
  if (found == info.hardware_parameters.end()) {
    return true;
  }
  double parsed = 0.0;
  if (!parse_finite_double(found->second, parsed) || parsed <= 0.0) {
    return false;
  }
  value = parsed;
  return true;
}

bool parse_strict_bool(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  bool & value)
{
  const auto found = info.hardware_parameters.find(name);
  if (found == info.hardware_parameters.end()) {
    return true;
  }
  if (found->second == "true") {
    value = true;
    return true;
  }
  if (found->second == "false") {
    value = false;
    return true;
  }
  return false;
}

template<typename IntegerT>
bool parse_positive_integer(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  IntegerT & value)
{
  const auto found = info.hardware_parameters.find(name);
  if (found == info.hardware_parameters.end()) {
    return true;
  }
  IntegerT parsed{};
  const char * const begin = found->second.data();
  const char * const end = begin + found->second.size();
  const auto result = std::from_chars(begin, end, parsed);
  if (result.ec != std::errc{} || result.ptr != end || parsed <= 0) {
    return false;
  }
  value = parsed;
  return true;
}

}  // namespace

StandardSystemHardware::~StandardSystemHardware()
{
  transport_.close();
}

#if BW_STD_CONTROL_JAZZY_API
hardware_interface::CallbackReturn StandardSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const auto result = hardware_interface::SystemInterface::on_init(params);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  return initialize(params.hardware_info);
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
StandardSystemHardware::on_export_state_interfaces()
{
  auto interfaces = hardware_interface::SystemInterface::on_export_state_interfaces();
  cache_jazzy_state_interfaces();
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
StandardSystemHardware::on_export_command_interfaces()
{
  auto interfaces = hardware_interface::SystemInterface::on_export_command_interfaces();
  cache_jazzy_command_interfaces();
  return interfaces;
}
#else
hardware_interface::CallbackReturn StandardSystemHardware::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  const auto result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != hardware_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  return initialize(hardware_info);
}

std::vector<hardware_interface::StateInterface>
StandardSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kStandardJointCount * 3U);
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    const std::string name{kStandardJointNames[joint]};
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &position_states_[joint]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[joint]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_states_[joint]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
StandardSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kStandardJointCount + kBaseInterfaceCount);
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    interfaces.emplace_back(
      std::string{kStandardJointNames[joint]}, hardware_interface::HW_IF_POSITION,
      &position_commands_[joint]);
  }
  for (std::size_t index = 0; index < kBaseInterfaceCount; ++index) {
    interfaces.emplace_back(
      "base", std::string{kBaseInterfaceNames[index]}, &base_commands_[index]);
  }
  return interfaces;
}
#endif

hardware_interface::CallbackReturn StandardSystemHardware::initialize(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (!validate_hardware_info(hardware_info) || !load_parameters(hardware_info) ||
    !load_joint_limits(hardware_info) || !update_joint_velocity_limits())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool StandardSystemHardware::validate_hardware_info(
  const hardware_interface::HardwareInfo & hardware_info) const
{
  std::vector<std::string_view> joint_names;
  joint_names.reserve(hardware_info.joints.size());
  for (const auto & joint : hardware_info.joints) {
    joint_names.emplace_back(joint.name);
  }
  if (!has_exact_standard_joint_names(joint_names)) {
    RCLCPP_ERROR(
      kLogger, "Standard hardware requires the exact %zu-joint name set, got %zu joints",
      kStandardJointCount, hardware_info.joints.size());
    return false;
  }
  for (const auto name : kStandardJointNames) {
    const auto * const component = find_component(hardware_info.joints, name);
    if (component == nullptr) {
      RCLCPP_ERROR(kLogger, "Missing required Standard joint: %s", std::string{name}.c_str());
      return false;
    }
    if (component->command_interfaces.size() != 1U ||
      component->state_interfaces.size() != 3U ||
      !has_interface(component->command_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(component->state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(component->state_interfaces, hardware_interface::HW_IF_VELOCITY) ||
      !has_interface(component->state_interfaces, hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        kLogger, "Joint %s must expose only position command and position/velocity/effort states",
        component->name.c_str());
      return false;
    }
  }
  if (hardware_info.gpios.size() != 1U || hardware_info.gpios.front().name != "base") {
    RCLCPP_ERROR(kLogger, "Hardware must expose exactly one GPIO resource named base");
    return false;
  }
  const auto & base = hardware_info.gpios.front();
  if (base.command_interfaces.size() != kBaseInterfaceCount ||
    !base.state_interfaces.empty())
  {
    RCLCPP_ERROR(kLogger, "base must expose exactly three command interfaces and no states");
    return false;
  }
  for (const auto name : kBaseInterfaceNames) {
    if (!has_interface(base.command_interfaces, name)) {
      RCLCPP_ERROR(kLogger, "base is missing command interface %s", std::string{name}.c_str());
      return false;
    }
  }
  return true;
}

bool StandardSystemHardware::load_parameters(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (const auto found = hardware_info.hardware_parameters.find("serial_port");
    found != hardware_info.hardware_parameters.end())
  {
    serial_port_ = found->second;
  }
  if (serial_port_.empty()) {
    RCLCPP_ERROR(kLogger, "serial_port must not be empty");
    return false;
  }

  std::uint32_t baud_rate = baud_rate_;
  std::int64_t feedback_timeout_ms = feedback_timeout_.count();
  double command_rate_hz = 40.0;
  const bool valid =
    parse_positive_integer(hardware_info, "baud_rate", baud_rate) &&
    parse_positive_integer(hardware_info, "feedback_timeout_ms", feedback_timeout_ms) &&
    parse_strict_bool(hardware_info, "power_on_on_activate", power_on_on_activate_) &&
    parse_strict_bool(hardware_info, "arm_mapping_calibrated", arm_mapping_calibrated_) &&
    parse_positive_double(hardware_info, "command_rate_hz", command_rate_hz) &&
    parse_arm_motor_indices_hardware_parameter(
    hardware_info, "left_arm_motor_indices",
    mapping_parameters_.left_arm_motor_indices) &&
    parse_arm_motor_indices_hardware_parameter(
    hardware_info, "right_arm_motor_indices",
    mapping_parameters_.right_arm_motor_indices) &&
    parse_arm_direction_hardware_parameter(
    hardware_info, "left_arm_direction",
    mapping_parameters_.left_arm_direction) &&
    parse_arm_direction_hardware_parameter(
    hardware_info, "right_arm_direction",
    mapping_parameters_.right_arm_direction) &&
    parse_arm_raw_zero_hardware_parameter(
    hardware_info, "left_arm_raw_zero_rad",
    mapping_parameters_.left_arm_raw_zero_rad) &&
    parse_arm_raw_zero_hardware_parameter(
    hardware_info, "right_arm_raw_zero_rad",
    mapping_parameters_.right_arm_raw_zero_rad) &&
    parse_positive_double(
    hardware_info, "pelvis_max_velocity_mm_s",
    mapping_parameters_.pelvis_max_velocity_mm_s) &&
    parse_positive_double(
    hardware_info, "arm_max_velocity_rad_s",
    mapping_parameters_.arm_max_velocity_rad_s) &&
    parse_positive_double(
    hardware_info, "gripper_max_velocity_normalized_s",
    mapping_parameters_.gripper_max_velocity_normalized_s) &&
    parse_positive_double(
    hardware_info, "base_max_acceleration_x",
    mapping_parameters_.base_max_acceleration_x) &&
    parse_positive_double(
    hardware_info, "base_max_acceleration_y",
    mapping_parameters_.base_max_acceleration_y) &&
    parse_positive_double(
    hardware_info, "base_max_acceleration_omega",
    mapping_parameters_.base_max_acceleration_omega);
  if (!valid) {
    RCLCPP_ERROR(kLogger, "Invalid bw_std_control hardware parameter");
    return false;
  }

  baud_rate_ = baud_rate;
  feedback_timeout_ = std::chrono::milliseconds{feedback_timeout_ms};
  command_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>{1.0 / command_rate_hz});
  return true;
}

bool StandardSystemHardware::load_joint_limits(
  const hardware_interface::HardwareInfo & hardware_info)
{
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    const auto * const component = find_component(hardware_info.joints, kStandardJointNames[joint]);
    if (component == nullptr) {
      return false;
    }
    const auto * const position = find_interface(
      component->command_interfaces, hardware_interface::HW_IF_POSITION);
    if (position == nullptr || position->min.empty() || position->max.empty() ||
      !parse_finite_double(position->min, command_limits_.lower[joint]) ||
      !parse_finite_double(position->max, command_limits_.upper[joint]) ||
      command_limits_.lower[joint] > command_limits_.upper[joint])
    {
      RCLCPP_ERROR(
        kLogger, "Joint %s requires finite position min/max with min <= max",
        component->name.c_str());
      return false;
    }
  }
  return true;
}

bool StandardSystemHardware::update_joint_velocity_limits() noexcept
{
  command_limits_.velocity.fill(mapping_parameters_.arm_max_velocity_rad_s);
  command_limits_.velocity[static_cast<std::size_t>(JointIndex::lift)] =
    mapping_parameters_.pelvis_max_velocity_mm_s / 1000.0;
  command_limits_.velocity[static_cast<std::size_t>(JointIndex::left_gripper)] =
    mapping_parameters_.gripper_max_velocity_normalized_s * mapping_parameters_.gripper_travel_m;
  command_limits_.velocity[static_cast<std::size_t>(JointIndex::right_gripper)] =
    mapping_parameters_.gripper_max_velocity_normalized_s * mapping_parameters_.gripper_travel_m;
  return std::all_of(
    command_limits_.velocity.begin(), command_limits_.velocity.end(),
    [](const double value) {return std::isfinite(value) && value > 0.0;});
}

hardware_interface::CallbackReturn StandardSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  transport_.close();
  parser_.clear();
  feedback_handoff_.reset();
  serial_fault_.store(false, std::memory_order_release);
  active_.store(false, std::memory_order_release);
  head_hold_ = {};
  command_elapsed_sec_ = 0.0;
  consecutive_state_access_failures_ = 0U;
  consecutive_command_access_failures_ = 0U;
  try {
    transport_.open(
      serial_port_, baud_rate_,
      [this](const std::uint8_t * data, const std::size_t size) {
        handle_serial_data(data, size);
      },
      [this](const std::string & message) {handle_serial_error(message);});
  } catch (const std::exception & error) {
    RCLCPP_ERROR(kLogger, "Failed to open serial port %s: %s", serial_port_.c_str(), error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    kLogger, "Standard V3 serial configured: port=%s baud=%u timeout=%ldms",
    serial_port_.c_str(), baud_rate_, feedback_timeout_.count());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  transport_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  StampedFeedback stamped{};
  {
    std::unique_lock<std::mutex> lock(activation_mutex_);
    const bool ready = feedback_condition_.wait_for(
      lock, feedback_timeout_, [this]() {
        return serial_fault_.load(std::memory_order_acquire) ||
        feedback_handoff_.published_sequence() > 0U;
      });
    if (serial_fault_.load(std::memory_order_acquire) || !ready) {
      RCLCPP_ERROR(kLogger, "Cannot activate without a valid current V3 feedback frame");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (!feedback_handoff_.read_latest(stamped)) {
    RCLCPP_ERROR(kLogger, "Cannot read the first complete V3 feedback sample");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - stamped.received_at > feedback_timeout_) {
    RCLCPP_ERROR(kLogger, "Cannot activate with stale V3 feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }

  StandardState state{};
  HeadHold initial_head_hold{};
  if (!decode_complete_feedback(
      stamped.feedback, mapping_parameters_, state, initial_head_hold))
  {
    RCLCPP_ERROR(kLogger, "Cannot activate with incomplete or non-finite measured feedback");
    return hardware_interface::CallbackReturn::ERROR;
  }
  head_hold_ = initial_head_hold;
  safe_hold_command_.position = state.position;
  safe_hold_command_.base_velocity.fill(0.0);
  const bool power_enabled = arm_mapping_enables_software_power(
    power_on_on_activate_, arm_mapping_calibrated_);
  if (power_enabled && !command_within_limits(safe_hold_command_, command_limits_)) {
    RCLCPP_ERROR(kLogger, "Cannot power on with measured joints outside configured limits");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!assign_state_values(state) || !synchronize_commands(state)) {
    RCLCPP_ERROR(kLogger, "Cannot synchronize ros2_control interfaces during activation");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  V3CommandPayload payload{};
  if (!encode_standard_command(
      safe_hold_command_, mapping_parameters_, head_hold_, power_enabled, payload) ||
    !transport_.async_write(encode_command_frame(payload)))
  {
    RCLCPP_ERROR(kLogger, "Failed to queue activation safety frame");
    return hardware_interface::CallbackReturn::ERROR;
  }
  last_command_stamp_ = now;
  last_sent_command_ = safe_hold_command_;
  command_elapsed_sec_ = 0.0;
  fault_reason_.store(FaultReason::none, std::memory_order_release);
  consecutive_state_access_failures_ = 0U;
  consecutive_command_access_failures_ = 0U;
  active_.store(true, std::memory_order_release);
  if (power_enabled) {
    RCLCPP_WARN(kLogger, "Hardware activated with explicit software power-on enabled");
  } else if (power_on_on_activate_) {
    RCLCPP_ERROR(
      kLogger,
      "Software power-on request rejected because the Standard arm mapping is not calibrated");
  } else {
    RCLCPP_INFO(kLogger, "Hardware active in measured feedback-only power-off mode");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  if (!queue_power_off() || !transport_.wait_for_power_off(feedback_timeout_)) {
    RCLCPP_ERROR(kLogger, "Failed to complete software power-off during deactivation");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  const bool power_off_queued = queue_power_off();
  transport_.close();
  return power_off_queued ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  const FaultReason reason = fault_reason_.exchange(FaultReason::none, std::memory_order_acq_rel);
  const bool power_off_queued = queue_power_off();
  RCLCPP_ERROR(
    kLogger, "Standard hardware fault code %u; priority power-off queued=%s",
    static_cast<unsigned int>(reason), power_off_queued ? "true" : "false");
  transport_.close();
  return power_off_queued ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type StandardSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fault_.load(std::memory_order_acquire)) {
    fault_reason_.store(FaultReason::serial_transport, std::memory_order_release);
    if (active_.exchange(false, std::memory_order_acq_rel)) {
      static_cast<void>(queue_power_off());
    }
    return hardware_interface::return_type::ERROR;
  }
  StampedFeedback stamped{};
  if (!feedback_handoff_.read_latest(stamped)) {
    return active_.load(std::memory_order_acquire) ?
           hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
  }
  if (active_.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() - stamped.received_at > feedback_timeout_)
  {
    active_.store(false, std::memory_order_release);
    fault_reason_.store(FaultReason::feedback_timeout, std::memory_order_release);
    static_cast<void>(queue_power_off());
    return hardware_interface::return_type::ERROR;
  }

  StandardState state{};
  if (!decode_standard_state(stamped.feedback, mapping_parameters_, state)) {
    serial_fault_.store(true, std::memory_order_release);
    active_.store(false, std::memory_order_release);
    fault_reason_.store(FaultReason::invalid_feedback, std::memory_order_release);
    static_cast<void>(queue_power_off());
    return hardware_interface::return_type::ERROR;
  }
  static_cast<void>(update_head_hold(stamped.feedback));
  if (!assign_state_values(state)) {
    ++consecutive_state_access_failures_;
    if (consecutive_state_access_failures_ <= kMaxConsecutiveHandleAccessFailures) {
      return hardware_interface::return_type::OK;
    }
    return fail_active_command(FaultReason::state_access);
  }
  consecutive_state_access_failures_ = 0U;
  safe_hold_command_.position = state.position;
  safe_hold_command_.base_velocity.fill(0.0);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StandardSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const bool power_enabled = arm_mapping_enables_software_power(
    power_on_on_activate_, arm_mapping_calibrated_);
  if (!active_.load(std::memory_order_acquire) || !power_enabled) {
    return hardware_interface::return_type::OK;
  }
  if (serial_fault_.load(std::memory_order_acquire) || !transport_.is_open()) {
    return fail_active_command(FaultReason::serial_transport);
  }
  const double period_sec = period.seconds();
  if (!std::isfinite(period_sec) || period_sec <= 0.0) {
    return fail_active_command(FaultReason::invalid_period);
  }
  command_elapsed_sec_ += period_sec;
  if (!std::isfinite(command_elapsed_sec_)) {
    return fail_active_command(FaultReason::invalid_period);
  }

  StandardCommand command{};
  if (!collect_command_values(command)) {
    ++consecutive_command_access_failures_;
    if (consecutive_command_access_failures_ <= kMaxConsecutiveHandleAccessFailures) {
      return hardware_interface::return_type::OK;
    }
    return fail_active_command(FaultReason::command_access);
  }
  consecutive_command_access_failures_ = 0U;
  if (!command_within_limits(command, command_limits_)) {
    return fail_active_command(FaultReason::command_limits);
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_command_stamp_ < command_period_) {
    return hardware_interface::return_type::OK;
  }
  if (!limit_command_step(command, last_sent_command_, command_limits_, command_elapsed_sec_)) {
    return fail_active_command(FaultReason::command_limits);
  }
  V3CommandPayload payload{};
  if (!encode_standard_command(command, mapping_parameters_, head_hold_, true, payload)) {
    return fail_active_command(FaultReason::command_encoding);
  }
  if (!transport_.async_write(encode_command_frame(payload))) {
    serial_fault_.store(true, std::memory_order_release);
    return fail_active_command(FaultReason::command_queue);
  }
  last_command_stamp_ = now;
  last_sent_command_ = command;
  command_elapsed_sec_ = 0.0;
  return hardware_interface::return_type::OK;
}

void StandardSystemHardware::handle_serial_data(
  const std::uint8_t * const data, const std::size_t size) noexcept
{
  bool received = false;
  parser_.append(data, size);
  V3FeedbackPayload feedback{};
  while (parser_.pop_feedback(feedback)) {
    StampedFeedback stamped{};
    stamped.feedback = feedback;
    stamped.received_at = std::chrono::steady_clock::now();
    stamped.valid = true;
    feedback_handoff_.publish(stamped);
    received = true;
  }
  if (received) {
    feedback_condition_.notify_all();
  }
}

void StandardSystemHardware::handle_serial_error(const std::string & message) noexcept
{
  serial_fault_.store(true, std::memory_order_release);
  fault_reason_.store(FaultReason::serial_transport, std::memory_order_release);
  feedback_condition_.notify_all();
  RCLCPP_ERROR(kLogger, "%s", message.c_str());
}

bool StandardSystemHardware::assign_state_values(const StandardState & state) noexcept
{
#if BW_STD_CONTROL_JAZZY_API
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    if (!position_state_handles_[joint]->set_value(state.position[joint], false) ||
      !velocity_state_handles_[joint]->set_value(state.velocity[joint], false) ||
      !effort_state_handles_[joint]->set_value(state.effort[joint], false))
    {
      return false;
    }
  }
#else
  position_states_ = state.position;
  velocity_states_ = state.velocity;
  effort_states_ = state.effort;
#endif
  return true;
}

bool StandardSystemHardware::collect_command_values(StandardCommand & command) noexcept
{
#if BW_STD_CONTROL_JAZZY_API
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    if (!position_command_handles_[joint]->get_value(command.position[joint], false)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < kBaseInterfaceCount; ++index) {
    if (!base_command_handles_[index]->get_value(command.base_velocity[index], false)) {
      return false;
    }
  }
#else
  command.position = position_commands_;
  command.base_velocity = base_commands_;
#endif
  return true;
}

bool StandardSystemHardware::synchronize_commands(const StandardState & state) noexcept
{
#if BW_STD_CONTROL_JAZZY_API
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    if (!position_command_handles_[joint]->set_value(state.position[joint], false)) {
      return false;
    }
  }
  for (const auto & handle : base_command_handles_) {
    if (!handle->set_value(0.0, false)) {
      return false;
    }
  }
#else
  position_commands_ = state.position;
  base_commands_.fill(0.0);
#endif
  return true;
}

bool StandardSystemHardware::update_head_hold(const V3FeedbackPayload & feedback) noexcept
{
  return try_extract_head_hold(feedback, head_hold_);
}

hardware_interface::return_type StandardSystemHardware::fail_active_command(
  const FaultReason reason) noexcept
{
  active_.store(false, std::memory_order_release);
  fault_reason_.store(reason, std::memory_order_release);
  static_cast<void>(queue_power_off());
  return hardware_interface::return_type::ERROR;
}

bool StandardSystemHardware::queue_power_off() noexcept
{
  safe_hold_command_.base_velocity.fill(0.0);
  V3CommandPayload payload{};
  if (!encode_standard_command(
      safe_hold_command_, mapping_parameters_, head_hold_, false, payload))
  {
    return false;
  }
  return transport_.async_write_power_off(encode_command_frame(payload));
}

std::string StandardSystemHardware::interface_key(
  const std::string_view component, const std::string_view interface_name)
{
  return std::string{component} + "/" + std::string{interface_name};
}

#if BW_STD_CONTROL_JAZZY_API
void StandardSystemHardware::cache_jazzy_state_interfaces()
{
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    position_state_handles_[joint] = get_state_interface_handle(
      interface_key(kStandardJointNames[joint], hardware_interface::HW_IF_POSITION));
    velocity_state_handles_[joint] = get_state_interface_handle(
      interface_key(kStandardJointNames[joint], hardware_interface::HW_IF_VELOCITY));
    effort_state_handles_[joint] = get_state_interface_handle(
      interface_key(kStandardJointNames[joint], hardware_interface::HW_IF_EFFORT));
  }
}

void StandardSystemHardware::cache_jazzy_command_interfaces()
{
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    position_command_handles_[joint] = get_command_interface_handle(
      interface_key(kStandardJointNames[joint], hardware_interface::HW_IF_POSITION));
  }
  for (std::size_t index = 0; index < kBaseInterfaceCount; ++index) {
    base_command_handles_[index] = get_command_interface_handle(
      interface_key("base", kBaseInterfaceNames[index]));
  }
}
#endif

}  // namespace bw_std_control

PLUGINLIB_EXPORT_CLASS(
  bw_std_control::StandardSystemHardware, hardware_interface::SystemInterface)
