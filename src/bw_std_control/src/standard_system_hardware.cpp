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

bool parse_bounded_nonnegative_hardware_parameter(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const double maximum, double & value)
{
  const auto found = info.hardware_parameters.find(name);
  return found == info.hardware_parameters.end() ||
         parse_bounded_nonnegative_parameter(found->second, maximum, value);
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

void log_command_safety_diagnostic(
  const char * const context, const CommandSafetyDiagnostic & diagnostic)
{
  if (diagnostic.ok()) {
    return;
  }
  if (diagnostic.failure == CommandSafetyFailure::base_velocity) {
    const std::string_view name = diagnostic.index < kBaseInterfaceNames.size() ?
      kBaseInterfaceNames[diagnostic.index] : std::string_view{"unknown"};
    RCLCPP_ERROR(
      kLogger, "%s: base/%.*s value=%.9f is not finite", context,
      static_cast<int>(name.size()), name.data(), diagnostic.value);
    return;
  }
  if (diagnostic.failure == CommandSafetyFailure::head_position) {
    const std::string_view name = diagnostic.index < kHeadInterfaceNames.size() ?
      kHeadInterfaceNames[diagnostic.index] : std::string_view{"unknown"};
    RCLCPP_ERROR(
      kLogger, "%s: head/%.*s value=%.9f allowed=[%.9f, %.9f]", context,
      static_cast<int>(name.size()), name.data(), diagnostic.value,
      diagnostic.lower, diagnostic.upper);
    return;
  }
  const std::string_view name = diagnostic.index < kStandardJointNames.size() ?
    kStandardJointNames[diagnostic.index] : std::string_view{"unknown"};
  RCLCPP_ERROR(
    kLogger, "%s: joint=%.*s value=%.9f allowed=[%.9f, %.9f] failure=%u",
    context, static_cast<int>(name.size()), name.data(), diagnostic.value,
    diagnostic.lower, diagnostic.upper,
    static_cast<unsigned int>(diagnostic.failure));
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
  interfaces.reserve(kStandardJointCount + kBaseInterfaceCount + kHeadInterfaceCount);
  for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
    interfaces.emplace_back(
      std::string{kStandardJointNames[joint]}, hardware_interface::HW_IF_POSITION,
      &position_commands_[joint]);
  }
  for (std::size_t index = 0; index < kBaseInterfaceCount; ++index) {
    interfaces.emplace_back(
      "base", std::string{kBaseInterfaceNames[index]}, &base_commands_[index]);
  }
  for (std::size_t index = 0; index < kHeadInterfaceCount; ++index) {
    interfaces.emplace_back(
      "head", std::string{kHeadInterfaceNames[index]}, &head_commands_[index]);
  }
  interfaces.emplace_back("safety", "power", &safety_power_command_);
  return interfaces;
}
#endif

hardware_interface::CallbackReturn StandardSystemHardware::initialize(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (!validate_hardware_info(hardware_info) || !load_parameters(hardware_info) ||
    !load_joint_limits(hardware_info) || !load_head_limits(hardware_info) ||
    !update_joint_velocity_limits())
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
  if (hardware_info.gpios.size() != 3U) {
    RCLCPP_ERROR(kLogger, "Hardware must expose exactly base, head and safety GPIO resources");
    return false;
  }
  const auto * const base = find_component(hardware_info.gpios, "base");
  const auto * const head = find_component(hardware_info.gpios, "head");
  const auto * const safety = find_component(hardware_info.gpios, "safety");
  if (base == nullptr || head == nullptr || safety == nullptr) {
    RCLCPP_ERROR(kLogger, "Hardware must expose GPIO resources named base, head and safety");
    return false;
  }
  if (safety->command_interfaces.size() != 1U || !safety->state_interfaces.empty() ||
    !has_interface(safety->command_interfaces, "power"))
  {
    RCLCPP_ERROR(kLogger, "safety must expose exactly one power command interface and no states");
    return false;
  }
  if (base->command_interfaces.size() != kBaseInterfaceCount ||
    !base->state_interfaces.empty())
  {
    RCLCPP_ERROR(kLogger, "base must expose exactly three command interfaces and no states");
    return false;
  }
  for (const auto name : kBaseInterfaceNames) {
    if (!has_interface(base->command_interfaces, name)) {
      RCLCPP_ERROR(kLogger, "base is missing command interface %s", std::string{name}.c_str());
      return false;
    }
  }
  if (head->command_interfaces.size() != kHeadInterfaceCount ||
    !head->state_interfaces.empty())
  {
    RCLCPP_ERROR(kLogger, "head must expose exactly three command interfaces and no states");
    return false;
  }
  for (const auto name : kHeadInterfaceNames) {
    if (!has_interface(head->command_interfaces, name)) {
      RCLCPP_ERROR(kLogger, "head is missing command interface %s", std::string{name}.c_str());
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
  double arm_startup_limit_tolerance_rad = 0.0;
  double gripper_startup_limit_tolerance_m = 0.0;
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
    parse_bounded_nonnegative_hardware_parameter(
    hardware_info, "arm_startup_limit_tolerance_rad",
    kMaxArmStartupLimitToleranceRad, arm_startup_limit_tolerance_rad) &&
    parse_bounded_nonnegative_hardware_parameter(
    hardware_info, "gripper_startup_limit_tolerance_m",
    kMaxGripperStartupLimitToleranceM, gripper_startup_limit_tolerance_m) &&
    parse_positive_double(
    hardware_info, "head_max_velocity_rad_s",
    mapping_parameters_.head_max_velocity_rad_s) &&
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
  startup_limit_tolerance_ = make_standard_startup_limit_tolerance(
    arm_startup_limit_tolerance_rad, gripper_startup_limit_tolerance_m);
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

bool StandardSystemHardware::load_head_limits(
  const hardware_interface::HardwareInfo & hardware_info)
{
  const auto * const head = find_component(hardware_info.gpios, "head");
  if (head == nullptr) {
    return false;
  }
  std::array<double, kHeadInterfaceCount> lower{};
  std::array<double, kHeadInterfaceCount> upper{};
  for (std::size_t index = 0; index < kHeadInterfaceCount; ++index) {
    const auto * const interface_info = find_interface(
      head->command_interfaces, kHeadInterfaceNames[index]);
    if (interface_info == nullptr || interface_info->min.empty() || interface_info->max.empty() ||
      !parse_finite_double(interface_info->min, lower[index]) ||
      !parse_finite_double(interface_info->max, upper[index]) || lower[index] > upper[index] ||
      lower[index] > 0.0 || upper[index] < 0.0)
    {
      RCLCPP_ERROR(
        kLogger, "Head interface %s requires finite min/max",
        std::string{kHeadInterfaceNames[index]}.c_str());
      return false;
    }
  }
  command_limits_.head_lower = lower;
  command_limits_.head_upper = upper;
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
  command_limits_.head_velocity.fill(mapping_parameters_.head_max_velocity_rad_s);
  return std::all_of(
    command_limits_.velocity.begin(), command_limits_.velocity.end(),
    [](const double value) {return std::isfinite(value) && value > 0.0;}) &&
         std::all_of(
    command_limits_.head_velocity.begin(), command_limits_.head_velocity.end(),
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
  fault_reason_.store(FaultReason::none, std::memory_order_release);
  startup_limit_recovery_.reset();
  last_command_safety_diagnostic_ = {};
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
    kLogger, "Standard serial configured: port=%s baud=%u timeout=%ldms",
    serial_port_.c_str(), baud_rate_, feedback_timeout_.count());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  startup_limit_recovery_.reset();
  last_command_safety_diagnostic_ = {};
  transport_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  startup_limit_recovery_.reset();
  last_command_safety_diagnostic_ = {};
  arm_power_on_sequence_.reset();
  arm_enable_frame_sent_ = false;
  has_logged_arm_enable_wait_ = false;
  StampedFeedback stamped{};
  StandardState state{};
  const bool power_enabled = arm_mapping_enables_software_power(
    power_on_on_activate_, arm_mapping_calibrated_);
  if (power_enabled) {
    if (!power_on_with_ready_arms(stamped, state)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    std::unique_lock<std::mutex> lock(activation_mutex_);
    const bool ready = feedback_condition_.wait_for(
      lock, feedback_timeout_, [this]() {
        return serial_fault_.load(std::memory_order_acquire) ||
        feedback_handoff_.published_sequence() > 0U;
      });
    if (serial_fault_.load(std::memory_order_acquire) || !ready) {
      RCLCPP_ERROR(kLogger, "Cannot activate without a valid current feedback frame");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!feedback_handoff_.read_latest(stamped)) {
      RCLCPP_ERROR(kLogger, "Cannot read the first complete feedback sample");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (std::chrono::steady_clock::now() - stamped.received_at > feedback_timeout_) {
      RCLCPP_ERROR(kLogger, "Cannot activate with stale feedback");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!adopt_measured_hold(stamped, state)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  CommandPayload payload{};
  if (!encode_standard_command(
      safe_hold_command_, mapping_parameters_, head_hold_, power_enabled, payload))
  {
    RCLCPP_ERROR(kLogger, "Failed to encode activation safety frame");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (transport_.async_write(encode_command_frame(payload)) == 0U) {
    RCLCPP_ERROR(kLogger, "Failed to queue activation safety frame");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (power_enabled) {
    arm_power_on_sequence_.mark_enabled();
    RCLCPP_INFO(
      kLogger, "arm enable frame queued: control_flag=0x%02X",
      static_cast<unsigned int>(kActiveControlFlag));
  }
  last_command_stamp_ = std::chrono::steady_clock::now();
  last_sent_command_ = safe_hold_command_;
  command_elapsed_sec_ = 0.0;
  fault_reason_.store(FaultReason::none, std::memory_order_release);
  consecutive_state_access_failures_ = 0U;
  consecutive_command_access_failures_ = 0U;
  active_.store(true, std::memory_order_release);
  chassis_wakeup_frames_remaining_ = chassis_wakeup_frames_;
  if (startup_limit_recovery_.has_active_recovery()) {
    for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
      const LimitRecoveryDirection direction = startup_limit_recovery_.direction(joint);
      if (direction == LimitRecoveryDirection::none) {
        continue;
      }
      const std::string_view name = kStandardJointNames[joint];
      RCLCPP_WARN(
        kLogger,
        "Startup limit recovery active: joint=%.*s measured=%.9f hard_limits=[%.9f, %.9f] "
        "allowed_motion=%s",
        static_cast<int>(name.size()), name.data(), state.position[joint],
        command_limits_.lower[joint], command_limits_.upper[joint],
        direction == LimitRecoveryDirection::decreasing ? "decreasing" : "increasing");
    }
  }
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

bool StandardSystemHardware::adopt_measured_hold(
  const StampedFeedback & stamped, StandardState & state) noexcept
{
  StandardState decoded{};
  HeadHold head_hold{};
  if (!decode_complete_feedback(stamped.feedback, mapping_parameters_, decoded, head_hold)) {
    RCLCPP_ERROR(kLogger, "Cannot adopt incomplete or non-finite measured feedback");
    return false;
  }
  head_hold_ = head_hold;
  safe_hold_command_.position = decoded.position;
  safe_hold_command_.base_velocity.fill(0.0);
  safe_hold_command_.head_position = {
    static_cast<double>(head_hold_.head_pitch_position),
    static_cast<double>(head_hold_.head_yaw_position),
    static_cast<double>(head_hold_.head_roll_position)};
  if (!assign_state_values(decoded) || !synchronize_commands(decoded)) {
    RCLCPP_ERROR(kLogger, "Cannot synchronize ros2_control interfaces during activation");
    return false;
  }
  state = decoded;
  return true;
}

std::uint64_t StandardSystemHardware::queue_arms_disabled_power_frame() noexcept
{
  CommandPayload payload{};
  if (!encode_standard_command(
      safe_hold_command_, mapping_parameters_, head_hold_, true, payload))
  {
    RCLCPP_ERROR(kLogger, "Failed to encode arms-disabled power frame");
    return 0U;
  }
  payload.control_flag = kArmsDisabledControlFlag;
  const std::uint64_t sequence = transport_.async_write(encode_command_frame(payload));
  if (sequence == 0U) {
    RCLCPP_ERROR(kLogger, "Failed to queue arms-disabled power frame");
  }
  return sequence;
}

bool StandardSystemHardware::power_on_with_ready_arms(
  StampedFeedback & ready_feedback, StandardState & state) noexcept
{
  // 阶段 1: 在上电前取实测位置作为权威 measured-hold; 系统上电后的短窗口内右臂位置
  // 会短暂归零, 因此使能后不得重读。
  bool ready = false;
  {
    const auto deadline = std::chrono::steady_clock::now() + kArmEnableReadyTimeout;
    std::uint64_t observed_sequence = 0U;
    FeedbackPayload last_feedback{};
    while (std::chrono::steady_clock::now() < deadline) {
      StampedFeedback stamped{};
      std::unique_lock<std::mutex> lock(activation_mutex_);
      if (serial_fault_.load(std::memory_order_acquire)) {
        RCLCPP_ERROR(
          kLogger, "Cannot power on: serial transport fault while waiting for arm feedback");
        return false;
      }
      static_cast<void>(feedback_condition_.wait_for(
        lock, command_period_, [this, observed_sequence]() {
          return serial_fault_.load(std::memory_order_acquire) ||
          feedback_handoff_.published_sequence() > observed_sequence;
        }));
      if (!feedback_handoff_.read_latest(stamped) || stamped.sequence == observed_sequence) {
        continue;
      }
      observed_sequence = stamped.sequence;
      last_feedback = stamped.feedback;
      if (std::chrono::steady_clock::now() - stamped.received_at > feedback_timeout_) {
        continue;
      }
      if (!arm_power_on_sequence_.observe_feedback(stamped.feedback)) {
        continue;
      }
      ready_feedback = stamped;
      ready = true;
      break;
    }
    if (!ready) {
      RCLCPP_ERROR(
        kLogger,
        "Cannot power on: no usable arm feedback for %zu consecutive frames within %d ms "
        "(last status=0x%02X left=0x%02X right=0x%02X chassis=0x%02X)",
        arm_power_on_sequence_.required_ready_frames(),
        static_cast<int>(kArmEnableReadyTimeout.count()),
        static_cast<unsigned int>(last_feedback.status_flags),
        static_cast<unsigned int>(last_feedback.left_arm_status_flags),
        static_cast<unsigned int>(last_feedback.right_arm_status_flags),
        static_cast<unsigned int>(last_feedback.chassis_status_flags));
      return false;
    }
  }

  if (!adopt_measured_hold(ready_feedback, state)) {
    return false;
  }
  const CommandSafetyDiagnostic diagnostic = startup_limit_recovery_.begin(
    safe_hold_command_, command_limits_, startup_limit_tolerance_);
  if (!diagnostic.ok()) {
    last_command_safety_diagnostic_ = diagnostic;
    fault_reason_.store(FaultReason::command_limits, std::memory_order_release);
    log_command_safety_diagnostic("Cannot power on from measured position", diagnostic);
    return false;
  }

  // 阶段 2: 预充电帧写入完成且保持满最小时长后才使能双臂。
  const auto precharge_deadline = std::chrono::steady_clock::now() + kArmEnablePrechargeTimeout;
  auto next_stream_at = std::chrono::steady_clock::now();
  std::uint64_t observed_sequence = 0U;
  while (std::chrono::steady_clock::now() < precharge_deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (arm_power_on_sequence_.stage() == ArmPowerStage::awaiting_precharge_completion &&
      arm_power_on_sequence_.precharge_completed(
        now, transport_.write_completed(arm_power_on_sequence_.pending_precharge_sequence())))
    {
      RCLCPP_INFO(
        kLogger, "arm precharge completed; enabling arms (control_flag=0x%02X)",
        static_cast<unsigned int>(kActiveControlFlag));
      return true;
    }
    if (now >= next_stream_at) {
      const bool starting = arm_power_on_sequence_.stage() == ArmPowerStage::ready_for_precharge;
      const std::uint64_t sequence = queue_arms_disabled_power_frame();
      if (sequence == 0U) {
        return false;
      }
      if (starting) {
        RCLCPP_INFO(
          kLogger, "arm precharge started: control_flag=0x%02X arms disabled",
          static_cast<unsigned int>(kArmsDisabledControlFlag));
      }
      static_cast<void>(arm_power_on_sequence_.note_precharge_queued(sequence, now));
      next_stream_at = now + command_period_;
    }
    StampedFeedback stamped{};
    std::unique_lock<std::mutex> lock(activation_mutex_);
    if (serial_fault_.load(std::memory_order_acquire)) {
      RCLCPP_ERROR(kLogger, "Cannot enable arms: serial transport fault during precharge");
      return false;
    }
    static_cast<void>(feedback_condition_.wait_for(
      lock, command_period_, [this, observed_sequence]() {
        return serial_fault_.load(std::memory_order_acquire) ||
        feedback_handoff_.published_sequence() > observed_sequence;
      }));
    if (!feedback_handoff_.read_latest(stamped) || stamped.sequence == observed_sequence) {
      continue;
    }
    observed_sequence = stamped.sequence;
    if (!arm_power_on_sequence_.observe_feedback(stamped.feedback)) {
      RCLCPP_ERROR(kLogger, "Aborting arm enable: arm feedback became unready during precharge");
      return false;
    }
  }
  RCLCPP_ERROR(
    kLogger, "Cannot enable arms: precharge write completion or minimum duration not met");
  return false;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  const std::uint64_t power_off_sequence = queue_power_off();
  const bool powered_off = power_off_sequence != 0U &&
    transport_.wait_for_power_off(power_off_sequence, feedback_timeout_);
  startup_limit_recovery_.reset();
  last_command_safety_diagnostic_ = {};
  if (!powered_off) {
    RCLCPP_ERROR(kLogger, "Failed to complete software power-off during deactivation");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  const bool power_off_queued = queue_power_off() != 0U;
  startup_limit_recovery_.reset();
  last_command_safety_diagnostic_ = {};
  transport_.close();
  return power_off_queued ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn StandardSystemHardware::on_error(
  const rclcpp_lifecycle::State &)
{
  active_.store(false, std::memory_order_release);
  const FaultReason reason = fault_reason_.exchange(FaultReason::none, std::memory_order_acq_rel);
  if (reason == FaultReason::command_limits) {
    log_command_safety_diagnostic(
      "Standard command limit fault", last_command_safety_diagnostic_);
  }
  const bool power_off_queued = queue_power_off() != 0U;
  RCLCPP_ERROR(
    kLogger, "Standard hardware fault code %u; priority power-off queued=%s",
    static_cast<unsigned int>(reason), power_off_queued ? "true" : "false");
  startup_limit_recovery_.reset();
  last_command_safety_diagnostic_ = {};
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
  if (active_.load(std::memory_order_acquire)) {
    static_cast<void>(arm_power_on_sequence_.observe_feedback(stamped.feedback));
  }

  StandardState state{};
  if (!decode_standard_state(stamped.feedback, mapping_parameters_, state)) {
    serial_fault_.store(true, std::memory_order_release);
    active_.store(false, std::memory_order_release);
    fault_reason_.store(FaultReason::invalid_feedback, std::memory_order_release);
    static_cast<void>(queue_power_off());
    return hardware_interface::return_type::ERROR;
  }
  if (!update_head_hold(stamped.feedback)) {
    serial_fault_.store(true, std::memory_order_release);
    active_.store(false, std::memory_order_release);
    fault_reason_.store(FaultReason::invalid_feedback, std::memory_order_release);
    static_cast<void>(queue_power_off());
    return hardware_interface::return_type::ERROR;
  }
  if (!assign_state_values(state)) {
    ++consecutive_state_access_failures_;
    if (consecutive_state_access_failures_ <= kMaxConsecutiveHandleAccessFailures) {
      return hardware_interface::return_type::OK;
    }
    return fail_active_command(FaultReason::state_access);
  }
  consecutive_state_access_failures_ = 0U;
  const bool power_enabled = arm_mapping_enables_software_power(
    power_on_on_activate_, arm_mapping_calibrated_);
  if (active_.load(std::memory_order_acquire) && power_enabled) {
    std::array<bool, kStandardJointCount> entry_commands_completed{};
    for (std::size_t joint = 0U; joint < kStandardJointCount; ++joint) {
      entry_commands_completed[joint] = transport_.write_completed(
        startup_limit_recovery_.entry_command_sequence(joint));
    }
    last_command_safety_diagnostic_ = startup_limit_recovery_.update_measured_state(
      state, command_limits_, entry_commands_completed, stamped.sequence);
    if (!last_command_safety_diagnostic_.ok()) {
      return fail_active_command(FaultReason::command_limits);
    }
  }
  safe_hold_command_.position = state.position;
  safe_hold_command_.base_velocity.fill(0.0);
  safe_hold_command_.head_position = {
    static_cast<double>(head_hold_.head_pitch_position),
    static_cast<double>(head_hold_.head_yaw_position),
    static_cast<double>(head_hold_.head_roll_position)};
  if (active_.load(std::memory_order_acquire) && power_enabled) {
    last_command_safety_diagnostic_ = check_auxiliary_limits(
      safe_hold_command_, command_limits_);
    if (!last_command_safety_diagnostic_.ok()) {
      return fail_active_command(FaultReason::command_limits);
    }
  }
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
  last_command_safety_diagnostic_ = startup_limit_recovery_.validate(
    command, last_sent_command_, command_limits_);
  if (!last_command_safety_diagnostic_.ok()) {
    return fail_active_command(FaultReason::command_limits);
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - last_command_stamp_ < command_period_) {
    return hardware_interface::return_type::OK;
  }
  if (chassis_wakeup_frames_remaining_ > 0U) {
    --chassis_wakeup_frames_remaining_;
    if (!queue_chassis_wakeup_frame(true)) {
      serial_fault_.store(true, std::memory_order_release);
      return fail_active_command(FaultReason::command_queue);
    }
    last_command_stamp_ = now;
    return hardware_interface::return_type::OK;
  }
  const StandardCommand previous_command = last_sent_command_;
  if (!limit_command_step(command, previous_command, command_limits_, command_elapsed_sec_)) {
    last_command_safety_diagnostic_ = {};
    return fail_active_command(FaultReason::command_limits);
  }
  CommandPayload payload{};
  double safety_power = 0.0;
  if (!collect_safety_power(safety_power) || !std::isfinite(safety_power)) {
    return fail_active_command(FaultReason::command_access);
  }
  // safety/power=0 (VR 断流) 时持续发送软件掉电帧。
  const bool power_requested = safety_power >= 0.5;
  if (!has_logged_command_power_ || last_command_power_ != power_requested) {
    has_logged_command_power_ = true;
    last_command_power_ = power_requested;
    RCLCPP_INFO(
      kLogger, "software power request changed: %s", power_requested ? "on" : "off");
  }
  if (!power_requested) {
    // 掉电请求立即作废未完成的使能时序, 重新上电必须重新满足就绪与预充电条件。
    arm_power_on_sequence_.reset();
    arm_enable_frame_sent_ = false;
  } else if (arm_power_on_sequence_.stage() == ArmPowerStage::awaiting_precharge_completion &&
    arm_power_on_sequence_.precharge_completed(
      now, transport_.write_completed(arm_power_on_sequence_.pending_precharge_sequence())))
  {
    arm_power_on_sequence_.mark_enabled();
    arm_enable_frame_sent_ = false;
    RCLCPP_INFO(kLogger, "arm precharge completed; requested arm bits enabled");
  }
  const ArmPowerFrameDecision power_frame = decide_arm_power_frame(
    power_requested, arm_power_on_sequence_.stage());
  const bool waiting_for_arm_feedback = power_requested &&
    arm_power_on_sequence_.stage() == ArmPowerStage::awaiting_ready_feedback;
  if (waiting_for_arm_feedback && !has_logged_arm_enable_wait_) {
    has_logged_arm_enable_wait_ = true;
    RCLCPP_WARN(
      kLogger,
      "software power requested but measured arm feedback is not usable; "
      "system powered with arms disabled");
  } else if (!waiting_for_arm_feedback) {
    has_logged_arm_enable_wait_ = false;
  }
  const double commanded_pelvis_mm =
    command.position[static_cast<std::size_t>(JointIndex::lift)] * 1000.0;
  if (!has_logged_pelvis_command_ ||
    std::abs(commanded_pelvis_mm - last_logged_pelvis_mm_) > 0.5)
  {
    has_logged_pelvis_command_ = true;
    last_logged_pelvis_mm_ = commanded_pelvis_mm;
    RCLCPP_INFO(
      kLogger, "command pelvis_height=%.2f mm (limit %.1f..%.1f mm)",
      commanded_pelvis_mm, command_limits_.lower[static_cast<std::size_t>(JointIndex::lift)] * 1000.0,
      command_limits_.upper[static_cast<std::size_t>(JointIndex::lift)] * 1000.0);
  }
  const bool frame_powered = power_frame.kind != ArmPowerFrameKind::power_off;
  // 使能帧使用实测保持位置, 不用可能过时的 controller 命令驱动手臂。
  StandardCommand frame_command = command;
  if (power_frame.kind == ArmPowerFrameKind::enabled && !arm_enable_frame_sent_) {
    frame_command.position = safe_hold_command_.position;
    frame_command.base_velocity.fill(0.0);
    frame_command.head_position = safe_hold_command_.head_position;
    arm_enable_frame_sent_ = true;
  }
  std::array<double, kStandardJointCount> joint_velocities{};
  if (frame_powered && command_elapsed_sec_ > 0.0) {
    for (std::size_t joint = 0; joint < kStandardJointCount; ++joint) {
      joint_velocities[joint] =
        (frame_command.position[joint] - last_sent_command_.position[joint]) / command_elapsed_sec_;
    }
  }
  if (!encode_standard_command(
      frame_command, mapping_parameters_, head_hold_, frame_powered, payload,
      frame_powered ? &joint_velocities : nullptr))
  {
    return fail_active_command(FaultReason::command_encoding);
  }
  payload.control_flag = power_frame.control_flag;
  const std::uint64_t sequence = transport_.async_write(encode_command_frame(payload));
  if (sequence == 0U) {
    serial_fault_.store(true, std::memory_order_release);
    return fail_active_command(FaultReason::command_queue);
  }
  if (power_frame.kind == ArmPowerFrameKind::precharge) {
    if (arm_power_on_sequence_.stage() == ArmPowerStage::ready_for_precharge) {
      RCLCPP_INFO(
        kLogger, "arm precharge frame queued: control_flag=0x%02X arms disabled",
        static_cast<unsigned int>(kArmsDisabledControlFlag));
    }
    static_cast<void>(arm_power_on_sequence_.note_precharge_queued(sequence, now));
  } else if (power_frame.kind == ArmPowerFrameKind::enabled) {
    startup_limit_recovery_.note_command_queued(
      frame_command, previous_command, command_limits_, sequence);
  }
  head_hold_.head_pitch_position = static_cast<float>(
    frame_command.head_position[static_cast<std::size_t>(HeadIndex::pitch)]);
  head_hold_.head_yaw_position = static_cast<float>(
    frame_command.head_position[static_cast<std::size_t>(HeadIndex::yaw)]);
  head_hold_.head_roll_position = static_cast<float>(
    frame_command.head_position[static_cast<std::size_t>(HeadIndex::roll)]);
  last_command_stamp_ = now;
  last_sent_command_ = frame_command;
  command_elapsed_sec_ = 0.0;
  return hardware_interface::return_type::OK;
}

void StandardSystemHardware::handle_serial_data(
  const std::uint8_t * const data, const std::size_t size) noexcept
{
  bool received = false;
  parser_.append(data, size);
  FeedbackPayload feedback{};
  while (parser_.pop_feedback(feedback)) {
    if (feedback.status_flags != last_status_flags_ ||
      feedback.chassis_status_flags != last_chassis_status_flags_ ||
      feedback.left_arm_status_flags != last_left_arm_status_flags_ ||
      feedback.right_arm_status_flags != last_right_arm_status_flags_)
    {
      last_status_flags_ = feedback.status_flags;
      last_chassis_status_flags_ = feedback.chassis_status_flags;
      last_left_arm_status_flags_ = feedback.left_arm_status_flags;
      last_right_arm_status_flags_ = feedback.right_arm_status_flags;
      RCLCPP_INFO(
        kLogger,
        "feedback status changed: status=0x%02X left=0x%02X right=0x%02X chassis=0x%02X",
        static_cast<unsigned int>(feedback.status_flags),
        static_cast<unsigned int>(feedback.left_arm_status_flags),
        static_cast<unsigned int>(feedback.right_arm_status_flags),
        static_cast<unsigned int>(feedback.chassis_status_flags));
    }
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

bool StandardSystemHardware::collect_safety_power(double & value) noexcept
{
#if BW_STD_CONTROL_JAZZY_API
  if (safety_command_handle_ == nullptr) {
    return false;
  }
  return safety_command_handle_->get_value(value, false);
#else
  value = safety_power_command_;
  return std::isfinite(value);
#endif
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
  for (std::size_t index = 0; index < kHeadInterfaceCount; ++index) {
    if (!head_command_handles_[index]->get_value(command.head_position[index], false)) {
      return false;
    }
  }
#else
  command.position = position_commands_;
  command.base_velocity = base_commands_;
  command.head_position = head_commands_;
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
  const std::array<double, kHeadInterfaceCount> head_position{
    static_cast<double>(head_hold_.head_pitch_position),
    static_cast<double>(head_hold_.head_yaw_position),
    static_cast<double>(head_hold_.head_roll_position)};
  for (std::size_t index = 0; index < kHeadInterfaceCount; ++index) {
    if (!head_command_handles_[index]->set_value(head_position[index], false)) {
      return false;
    }
  }
  if (safety_command_handle_ == nullptr || !safety_command_handle_->set_value(0.0, false)) {
    return false;
  }
#else
  position_commands_ = state.position;
  base_commands_.fill(0.0);
  head_commands_ = {
    static_cast<double>(head_hold_.head_pitch_position),
    static_cast<double>(head_hold_.head_yaw_position),
    static_cast<double>(head_hold_.head_roll_position)};
  safety_power_command_ = 0.0;
#endif
  return true;
}

bool StandardSystemHardware::update_head_hold(const FeedbackPayload & feedback) noexcept
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

bool StandardSystemHardware::queue_chassis_wakeup_frame(const bool power_enabled) noexcept
{
  ChassisCommandPayload payload{};
  payload.control_flag = power_enabled ? 0x01U : 0x00U;
  payload.pelvis_height = 0.0F;
  payload.pelvis_max_velocity = 0.0F;
  payload.pelvis_acceleration = 0.0F;
  const ChassisCommandFrame chassis_frame = encode_chassis_command_frame(payload);
  CommandFrame frame{};
  std::copy(chassis_frame.cbegin(), chassis_frame.cend(), frame.begin());
  return transport_.async_write(frame);
}

std::uint64_t StandardSystemHardware::queue_power_off() noexcept
{
  safe_hold_command_.base_velocity.fill(0.0);
  safe_hold_command_.head_position = {
    static_cast<double>(head_hold_.head_pitch_position),
    static_cast<double>(head_hold_.head_yaw_position),
    static_cast<double>(head_hold_.head_roll_position)};
  CommandPayload payload{};
  if (!encode_standard_command(
      safe_hold_command_, mapping_parameters_, head_hold_, false, payload))
  {
    return 0U;
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
  for (std::size_t index = 0; index < kHeadInterfaceCount; ++index) {
    head_command_handles_[index] = get_command_interface_handle(
      interface_key("head", kHeadInterfaceNames[index]));
  }
  safety_command_handle_ = get_command_interface_handle(interface_key("safety", "power"));
}
#endif

}  // namespace bw_std_control

PLUGINLIB_EXPORT_CLASS(
  bw_std_control::StandardSystemHardware, hardware_interface::SystemInterface)
