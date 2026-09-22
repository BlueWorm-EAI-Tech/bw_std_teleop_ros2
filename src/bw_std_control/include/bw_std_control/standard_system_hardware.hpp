#ifndef BW_STD_CONTROL__STANDARD_SYSTEM_HARDWARE_HPP_
#define BW_STD_CONTROL__STANDARD_SYSTEM_HARDWARE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "bw_std_control/arm_power_on_sequence.hpp"
#include "bw_std_control/async_serial_transport.hpp"
#include "bw_std_control/command_safety.hpp"
#include "bw_std_control/feedback_handoff.hpp"
#include "bw_std_control/hardware_contract.hpp"
#include "bw_std_control/standard_mapping.hpp"
#include "bw_std_control/system_interface_compat.hpp"
#include "bw_std_control/frame_codec.hpp"

namespace bw_std_control
{

class StandardSystemHardware final : public hardware_interface::SystemInterface
{
public:
  StandardSystemHardware() = default;
  ~StandardSystemHardware() override;

#if BW_STD_CONTROL_JAZZY_API
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;
#else
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
#endif

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class FaultReason : std::uint8_t
  {
    none,
    serial_transport,
    feedback_timeout,
    invalid_feedback,
    state_access,
    invalid_period,
    command_access,
    command_limits,
    command_encoding,
    command_queue
  };

  hardware_interface::CallbackReturn initialize(
    const hardware_interface::HardwareInfo & hardware_info);
  bool validate_hardware_info(const hardware_interface::HardwareInfo & hardware_info) const;
  bool load_parameters(const hardware_interface::HardwareInfo & hardware_info);
  bool load_joint_limits(const hardware_interface::HardwareInfo & hardware_info);
  bool load_head_limits(const hardware_interface::HardwareInfo & hardware_info);
  bool update_joint_velocity_limits() noexcept;
  void handle_serial_data(const std::uint8_t * data, std::size_t size) noexcept;
  void handle_serial_error(const std::string & message) noexcept;
  bool assign_state_values(const StandardState & state) noexcept;
  bool collect_command_values(StandardCommand & command) noexcept;
  // safety/power: 读不到时按掉电处理。
  bool collect_safety_power(double & value) noexcept;
  bool synchronize_commands(const StandardState & state) noexcept;
  bool update_head_hold(const FeedbackPayload & feedback) noexcept;
  bool power_on_with_ready_arms(StampedFeedback & ready_feedback, StandardState & state) noexcept;
  // 返回写入序列; 0 表示失败。
  std::uint64_t queue_arms_disabled_power_frame() noexcept;
  bool adopt_measured_hold(const StampedFeedback & stamped, StandardState & state) noexcept;
  hardware_interface::return_type fail_active_command(FaultReason reason) noexcept;
  std::uint64_t queue_power_off() noexcept;
  // 下盘(底盘/滑台)独立控制器的 0x21 唤醒帧。
  bool queue_chassis_wakeup_frame(bool power_enabled) noexcept;
  static std::string interface_key(
    std::string_view component, std::string_view interface_name);

#if BW_STD_CONTROL_JAZZY_API
  void cache_jazzy_state_interfaces();
  void cache_jazzy_command_interfaces();
  std::array<hardware_interface::StateInterface::SharedPtr, kStandardJointCount>
  position_state_handles_{};
  std::array<hardware_interface::StateInterface::SharedPtr, kStandardJointCount>
  velocity_state_handles_{};
  std::array<hardware_interface::StateInterface::SharedPtr, kStandardJointCount>
  effort_state_handles_{};
  std::array<hardware_interface::CommandInterface::SharedPtr, kStandardJointCount>
  position_command_handles_{};
  std::array<hardware_interface::CommandInterface::SharedPtr, kBaseInterfaceCount>
  base_command_handles_{};
  std::array<hardware_interface::CommandInterface::SharedPtr, kHeadInterfaceCount>
  head_command_handles_{};
  hardware_interface::CommandInterface::SharedPtr safety_command_handle_{};
#else
  std::array<double, kStandardJointCount> position_states_{};
  std::array<double, kStandardJointCount> velocity_states_{};
  std::array<double, kStandardJointCount> effort_states_{};
  std::array<double, kStandardJointCount> position_commands_{};
  std::array<double, kBaseInterfaceCount> base_commands_{};
  std::array<double, kHeadInterfaceCount> head_commands_{};
#endif
  double safety_power_command_{0.0};
  bool has_logged_command_power_{false};
  bool last_command_power_{false};
  std::uint8_t last_status_flags_{0U};
  std::uint8_t last_chassis_status_flags_{0U};
  std::uint8_t last_left_arm_status_flags_{0U};
  std::uint8_t last_right_arm_status_flags_{0U};
  bool has_logged_arm_enable_wait_{false};
  bool arm_enable_frame_sent_{false};
  bool has_logged_pelvis_command_{false};
  double last_logged_pelvis_mm_{0.0};
  std::size_t chassis_wakeup_frames_{200U};
  std::size_t chassis_wakeup_frames_remaining_{0U};

  std::string serial_port_{"/dev/ttyACM0"};
  std::uint32_t baud_rate_{2000000U};
  std::chrono::milliseconds feedback_timeout_{100};
  std::chrono::steady_clock::duration command_period_{std::chrono::milliseconds{25}};
  bool power_on_on_activate_{false};
  bool arm_mapping_calibrated_{false};
  MappingParameters mapping_parameters_{};
  CommandLimits command_limits_{};
  std::array<double, kStandardJointCount> startup_limit_tolerance_{};
  StartupLimitRecovery startup_limit_recovery_{};
  CommandSafetyDiagnostic last_command_safety_diagnostic_{};

  std::mutex activation_mutex_;
  std::condition_variable feedback_condition_;
  StreamParser parser_{};
  FeedbackHandoff feedback_handoff_{};

  std::atomic_bool serial_fault_{false};
  std::atomic_bool active_{false};
  std::atomic<FaultReason> fault_reason_{FaultReason::none};
  StandardCommand safe_hold_command_{};
  StandardCommand last_sent_command_{};
  HeadHold head_hold_{};
  double command_elapsed_sec_{0.0};
  std::size_t consecutive_state_access_failures_{0U};
  std::size_t consecutive_command_access_failures_{0U};
  std::chrono::steady_clock::time_point last_command_stamp_{};
  ArmPowerOnSequence arm_power_on_sequence_{};
  AsyncSerialTransport transport_{};
};

}  // namespace bw_std_control

#endif  // BW_STD_CONTROL__STANDARD_SYSTEM_HARDWARE_HPP_
