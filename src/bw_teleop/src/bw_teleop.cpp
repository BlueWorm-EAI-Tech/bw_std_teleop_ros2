#include "bw_teleop/node/bw_teleop.hpp"

#include <arpa/inet.h>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <netinet/in.h>
#include <rclcpp/expand_topic_or_service_name.hpp>
#include <stdexcept>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace bw_teleop::node
{
namespace
{

constexpr std::size_t kReceiveBufferSize{65536U};

manager::Pose make_pose(
  const std::vector<double> & position, const std::vector<double> & orientation)
{
  manager::Pose pose;
  pose.position = {position[0], position[1], position[2]};
  pose.orientation = {orientation[0], orientation[1], orientation[2], orientation[3]};
  return pose;
}

}  // namespace

BwTeleop::BwTeleop(const rclcpp::NodeOptions & options)
: rclcpp::Node("bw_teleop", options)
{
  const double publish_frequency_hz =
    declare_parameter<double>("publish_frequency_hz", 90.0);
  if (!std::isfinite(publish_frequency_hz) || publish_frequency_hz <= 0.0) {
    throw std::invalid_argument("publish_frequency_hz 必须为有限正数");
  }

  target_frame_id_ = declare_parameter<std::string>("target_frame", "C_Link");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  if (target_frame_id_.empty() || base_frame_id_.empty()) {
    throw std::invalid_argument("target_frame 和 base_frame_id 不能为空");
  }
  topics_ = load_topics();
  joint_names_ = load_joint_names();
  const UdpConfig udp_config = load_udp_config();
  manager_ = std::make_unique<manager::VrStreamManager>(load_stream_config());

  open_socket(udp_config);
  create_publishers();
  create_subscriptions();
  create_timers(publish_frequency_hz, udp_config.poll_period_ms);
  RCLCPP_INFO(
    get_logger(), "bw_teleop 监听 UDP %s:%d，命令发布频率 %.1f Hz",
    udp_config.bind_ip.c_str(), udp_config.port, publish_frequency_hz);
}

BwTeleop::~BwTeleop()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

manager::VrStreamConfig BwTeleop::load_stream_config()
{
  const auto declare_array = [this](
    const std::string & name, const std::vector<double> & defaults,
    const std::size_t expected_size)
    {
      const std::vector<double> values =
        declare_parameter<std::vector<double>>(name, defaults);
      if (values.size() != expected_size) {
        throw std::invalid_argument(name + " 长度非法");
      }
      return values;
    };

  const std::vector<double> left_reset_position = declare_array(
    "left_reset_position", {0.40, 0.25, -0.15}, 3U);
  const std::vector<double> left_reset_orientation = declare_array(
    "left_reset_orientation", {0.0, 0.0, 0.0, 1.0}, 4U);
  const std::vector<double> right_reset_position = declare_array(
    "right_reset_position", {0.40, -0.25, -0.15}, 3U);
  const std::vector<double> right_reset_orientation = declare_array(
    "right_reset_orientation", {0.0, 0.0, 0.0, 1.0}, 4U);
  const std::vector<double> workspace_min = declare_array(
    "workspace_min", {0.10, -0.80, -0.60}, 3U);
  const std::vector<double> workspace_max = declare_array(
    "workspace_max", {0.90, 0.80, 0.50}, 3U);
  const std::vector<double> base_speed_scales = declare_array(
    "base_speed_scales", {0.25, 0.50, 1.0}, 3U);

  manager::VrStreamConfig stream_config;
  manager::TeleopConfig & config = stream_config.teleop;
  config.reset_poses[0] = make_pose(left_reset_position, left_reset_orientation);
  config.reset_poses[1] = make_pose(right_reset_position, right_reset_orientation);
  config.workspace.minimum = {workspace_min[0], workspace_min[1], workspace_min[2]};
  config.workspace.maximum = {workspace_max[0], workspace_max[1], workspace_max[2]};
  config.relative_position_scale =
    declare_parameter<double>("relative_position_scale", 1.0);
  config.absolute_position_scale =
    declare_parameter<double>("absolute_position_scale", 1.0);
  config.watchdog_timeout_sec = static_cast<double>(
    declare_parameter<std::int64_t>("watchdog_timeout_ms", 200)) / 1000.0;
  config.feedback_timeout_sec = static_cast<double>(
    declare_parameter<std::int64_t>("joint_state_timeout_ms", 200)) / 1000.0;
  config.arm_pose_timeout_sec = static_cast<double>(
    declare_parameter<std::int64_t>("arm_pose_timeout_ms", 200)) / 1000.0;
  config.long_press_sec = declare_parameter<double>("long_press_sec", 1.0);
  config.auto_reset_on_vr_connect =
    declare_parameter<bool>("auto_reset_on_vr_connect", false);
  config.reset_duration_sec = declare_parameter<double>("reset_duration_sec", 1.5);
  config.reset_timeout_sec = declare_parameter<double>("reset_timeout_sec", 20.0);
  config.joystick_deadzone =
    declare_parameter<double>("command_joystick_deadzone", 0.08);
  config.gripper_min = declare_parameter<double>("gripper_min", 0.0);
  config.gripper_max = declare_parameter<double>("gripper_max", 0.04965);
  config.base_max_forward = declare_parameter<double>("base_max_forward", 0.50);
  config.base_max_lateral = declare_parameter<double>("base_max_lateral", 0.35);
  config.base_max_yaw = declare_parameter<double>("base_max_yaw", 0.80);
  config.base_speed_scales = {
    base_speed_scales[0], base_speed_scales[1], base_speed_scales[2]};
  config.base_initial_gear = static_cast<int>(
    declare_parameter<std::int64_t>("base_initial_gear", 0));
  config.lift_min = declare_parameter<double>("lift_min_mm", -500.0);
  config.lift_max = declare_parameter<double>("lift_max_mm", 0.0);
  config.lift_initial_position = declare_parameter<double>("lift_initial_position_mm", -100.0);
  config.lift_jog_speed = declare_parameter<double>("lift_jog_speed_mm_s", 200.0);
  const std::vector<double> head_min = declare_array(
    "head_min", {-0.785, -1.570, -0.349}, 3U);
  const std::vector<double> head_max = declare_array(
    "head_max", {0.524, 1.570, 0.349}, 3U);
  config.head_min = {head_min[0], head_min[1], head_min[2]};
  config.head_max = {head_max[0], head_max[1], head_max[2]};

  stream_config.input.joystick_deadzone =
    declare_parameter<double>("input_joystick_deadzone", 0.15);
  stream_config.session.lock_enabled =
    declare_parameter<bool>("single_client_lock_enabled", true);
  stream_config.session.timeout_sec =
    declare_parameter<double>("client_lock_timeout_sec", 3.0);
  return stream_config;
}

BwTeleop::UdpConfig BwTeleop::load_udp_config()
{
  UdpConfig config;
  config.bind_ip = declare_parameter<std::string>("bind_ip", "0.0.0.0");
  config.port = declare_parameter<int>("port", 12345);
  config.poll_period_ms = declare_parameter<int>("poll_period_ms", 2);
  const int max_packets = declare_parameter<int>("max_packets_per_cycle", 64);
  if (config.poll_period_ms <= 0 || max_packets <= 0) {
    throw std::invalid_argument("poll_period_ms 和 max_packets_per_cycle 必须大于 0");
  }
  config.max_packets_per_cycle = static_cast<std::size_t>(max_packets);
  return config;
}

BwTeleop::Topics BwTeleop::load_topics()
{
  Topics topics;
  topics.joint_states_input = declare_parameter<std::string>(
    "topics.joint_states_input", "/joint_states");
  topics.left_measured_pose_input = declare_parameter<std::string>(
    "topics.left_measured_pose_input", "/current_left_pose");
  topics.right_measured_pose_input = declare_parameter<std::string>(
    "topics.right_measured_pose_input", "/current_right_pose");
  topics.left_target_output = declare_parameter<std::string>(
    "topics.left_target_output", "/target_left_pose");
  topics.right_target_output = declare_parameter<std::string>(
    "topics.right_target_output", "/target_right_pose");
  topics.gripper_command_output = declare_parameter<std::string>(
    "topics.gripper_command_output", "/gripper_controller/commands");
  topics.lift_command_output = declare_parameter<std::string>(
    "topics.lift_command_output", "/lift_controller/commands");
  topics.base_command_output = declare_parameter<std::string>(
    "topics.base_command_output", "/base_controller/reference");
  topics.head_pose_output = declare_parameter<std::string>(
    "topics.head_pose_output", "/Teleop/head_pose");
  topics.control_active_output = declare_parameter<std::string>(
    "topics.control_active_output", "/teleop/control_active");
  topics.reset_request_output = declare_parameter<std::string>(
    "topics.reset_request_output", "/kinematics/reset_arms");

  const auto validate_topic = [this](const std::string & parameter_name,
    const std::string & topic_name) {
      if (topic_name.empty()) {
        throw std::invalid_argument(parameter_name + " 不能为空");
      }
      try {
        static_cast<void>(rclcpp::expand_topic_or_service_name(
          topic_name, get_name(), get_namespace()));
      } catch (const std::exception & error) {
        throw std::invalid_argument(
                parameter_name + " 不是合法 ROS topic: " + error.what());
      }
    };
  validate_topic("topics.joint_states_input", topics.joint_states_input);
  validate_topic("topics.left_measured_pose_input", topics.left_measured_pose_input);
  validate_topic("topics.right_measured_pose_input", topics.right_measured_pose_input);
  validate_topic("topics.left_target_output", topics.left_target_output);
  validate_topic("topics.right_target_output", topics.right_target_output);
  validate_topic("topics.gripper_command_output", topics.gripper_command_output);
  validate_topic("topics.lift_command_output", topics.lift_command_output);
  validate_topic("topics.base_command_output", topics.base_command_output);
  validate_topic("topics.head_pose_output", topics.head_pose_output);
  validate_topic("topics.control_active_output", topics.control_active_output);
  validate_topic("topics.reset_request_output", topics.reset_request_output);
  return topics;
}

BwTeleop::JointNames BwTeleop::load_joint_names()
{
  JointNames names;
  names.left_gripper = declare_parameter<std::string>(
    "joints.left_gripper", "A_left_Degree8_joint");
  names.right_gripper = declare_parameter<std::string>(
    "joints.right_gripper", "A_right_Degree8_joint");
  names.lift = declare_parameter<std::string>("joints.lift", "C_joint");
  return names;
}

void BwTeleop::open_socket(const UdpConfig & config)
{
  if (config.port <= 0 || config.port > 65535) {
    throw std::invalid_argument("UDP port 必须位于 1..65535");
  }
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    throw std::runtime_error("创建 UDP socket 失败: " + std::string{std::strerror(errno)});
  }
  const int reuse_address{1};
  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse_address, sizeof(reuse_address)) < 0) {
    close(socket_fd);
    throw std::runtime_error("设置 SO_REUSEADDR 失败: " + std::string{std::strerror(errno)});
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(config.port));
  if (inet_pton(AF_INET, config.bind_ip.c_str(), &address.sin_addr) != 1) {
    close(socket_fd);
    throw std::invalid_argument("bind_ip 不是合法 IPv4 地址: " + config.bind_ip);
  }
  if (bind(socket_fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    close(socket_fd);
    throw std::runtime_error("绑定 UDP socket 失败: " + error);
  }
  socket_fd_ = socket_fd;
  max_packets_per_cycle_ = config.max_packets_per_cycle;
}

void BwTeleop::create_publishers()
{
  const rclcpp::QoS command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  left_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    topics_.left_target_output, command_qos);
  right_target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    topics_.right_target_output, command_qos);
  gripper_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    topics_.gripper_command_output, command_qos);
  lift_command_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    topics_.lift_command_output, command_qos);
  base_command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    topics_.base_command_output, command_qos);
  head_pose_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    topics_.head_pose_output, command_qos);
  control_active_pub_ = create_publisher<std_msgs::msg::Bool>(
    topics_.control_active_output, command_qos);
  reset_request_pub_ = create_publisher<std_msgs::msg::Bool>(
    topics_.reset_request_output, command_qos);
}

void BwTeleop::create_subscriptions()
{
  const rclcpp::SensorDataQoS sensor_qos;
  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    topics_.joint_states_input, sensor_qos,
    std::bind(&BwTeleop::joint_states_callback, this, std::placeholders::_1));
  left_measured_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    topics_.left_measured_pose_input, sensor_qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      measured_pose_callback(manager::Side::left, msg);
    });
  right_measured_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    topics_.right_measured_pose_input, sensor_qos,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      measured_pose_callback(manager::Side::right, msg);
    });
}

void BwTeleop::create_timers(const double publish_frequency_hz, const int poll_period_ms)
{
  receive_timer_ = create_wall_timer(
    std::chrono::milliseconds{poll_period_ms},
    std::bind(&BwTeleop::receive_available_packets, this));
  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_frequency_hz),
    std::bind(&BwTeleop::publish_commands, this));
}

double BwTeleop::steady_now_sec() noexcept
{
  return steady_clock_.now().seconds();
}

void BwTeleop::receive_available_packets()
{
  std::array<std::uint8_t, kReceiveBufferSize> buffer{};
  for (std::size_t count = 0U; count < max_packets_per_cycle_; ++count) {
    sockaddr_in source_address{};
    socklen_t source_length = sizeof(source_address);
    const ssize_t received = recvfrom(
      socket_fd_, buffer.data(), buffer.size(), MSG_DONTWAIT,
      reinterpret_cast<sockaddr *>(&source_address), &source_length);
    if (received < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "UDP 接收失败: %s", std::strerror(errno));
      }
      break;
    }
    if (received == 0) {
      continue;
    }

    char source_ip_buffer[INET_ADDRSTRLEN]{};
    const char * converted = inet_ntop(
      AF_INET, &source_address.sin_addr, source_ip_buffer, sizeof(source_ip_buffer));
    transport::UdpDatagram datagram;
    datagram.source_ip = converted == nullptr ? "" : std::string{converted};
    datagram.payload.assign(
      buffer.cbegin(), buffer.cbegin() + static_cast<std::ptrdiff_t>(received));
    const manager::DatagramResult result = manager_->ingest(datagram, steady_now_sec());
    if (result.disposition == manager::DatagramDisposition::invalid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "丢弃无效 VR UDP 帧: %s", result.message.c_str());
    } else if (result.disposition == manager::DatagramDisposition::non_owner) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "拒绝非 owner VR 客户端 %s",
        datagram.source_ip.c_str());
    } else if (result.disposition == manager::DatagramDisposition::out_of_order) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "丢弃倒序 VR UDP 帧: %s", result.message.c_str());
    } else if (result.owner_changed) {
      RCLCPP_INFO(get_logger(), "VR 客户端锁定为 %s", manager_->owner().c_str());
    }
  }
}

void BwTeleop::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg == nullptr || msg->name.size() != msg->position.size()) {
    manager_->invalidate_measured_state();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "忽略空或 name/position 数量不一致的 JointState");
    return;
  }
  manager::MeasuredState measured;
  const std::size_t count = msg->name.size();
  for (std::size_t joint_index = 0U; joint_index < count; ++joint_index) {
    const std::string & name = msg->name[joint_index];
    const double position = msg->position[joint_index];
    if (name == joint_names_.left_gripper) {
      measured.left_gripper = position;
      measured.left_gripper_valid = true;
    } else if (name == joint_names_.right_gripper) {
      measured.right_gripper = position;
      measured.right_gripper_valid = true;
    } else if (name == joint_names_.lift) {
      // C_joint: ros2_control 用米, Manager 用毫米。
      measured.lift = position * 1000.0;
      measured.lift_valid = true;
    }
  }
  if (!measured.left_gripper_valid) {
    manager_->invalidate_measured_gripper(manager::Side::left);
  }
  if (!measured.right_gripper_valid) {
    manager_->invalidate_measured_gripper(manager::Side::right);
  }
  if (!measured.lift_valid) {
    manager_->invalidate_measured_lift();
  }
  manager_->update_measured_state(measured, steady_now_sec());
}

void BwTeleop::measured_pose_callback(
  const manager::Side side, const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (msg == nullptr) {
    manager_->invalidate_measured_pose(side);
    return;
  }
  if (!input::is_compatible_frame(msg->header.frame_id, target_frame_id_)) {
    manager_->invalidate_measured_pose(side);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "丢弃实测末端位姿: frame_id=%s, 期望=%s",
      msg->header.frame_id.c_str(), target_frame_id_.c_str());
    return;
  }
  manager_->update_measured_pose(side, to_internal_pose(msg->pose), steady_now_sec());
}

void BwTeleop::publish_commands()
{
  const manager::TickResult result = manager_->tick(steady_now_sec());
  const rclcpp::Time stamp = now();
  if (result.output.left_target_valid) {
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = stamp;
    target.header.frame_id = target_frame_id_;
    target.pose = to_ros_pose(result.output.left_target);
    left_target_pub_->publish(target);
  }
  if (result.output.right_target_valid) {
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = stamp;
    target.header.frame_id = target_frame_id_;
    target.pose = to_ros_pose(result.output.right_target);
    right_target_pub_->publish(target);
  }

  if (result.output.head_position_valid || result.output.safe_hold) {
    sensor_msgs::msg::JointState head_command;
    head_command.header.stamp = stamp;
    head_command.name = {
      "head_pitch_joint", "head_yaw_joint", "head_roll_joint"};
    head_command.position.assign(
      result.output.head_position.cbegin(), result.output.head_position.cend());
    head_pose_pub_->publish(head_command);
  }

  if (result.output.auxiliary_commands_valid) {
    std_msgs::msg::Float64MultiArray gripper_command;
    gripper_command.data = {
      result.output.gripper_command[0], result.output.gripper_command[1]};
    gripper_command_pub_->publish(gripper_command);

    std_msgs::msg::Float64MultiArray lift_command;
    // C_joint 命令: 毫米 -> 米。
    lift_command.data = {result.output.lift_command / 1000.0};
    lift_command_pub_->publish(lift_command);
  }

  geometry_msgs::msg::TwistStamped base_command;
  base_command.header.stamp = stamp;
  base_command.header.frame_id = base_frame_id_;
  base_command.twist.linear.x = result.output.base_vx;
  base_command.twist.linear.y = result.output.base_vy;
  base_command.twist.angular.z = result.output.base_wz;
  base_command_pub_->publish(base_command);

  std_msgs::msg::Bool control_active;
  control_active.data = !result.output.safe_hold;
  control_active_pub_->publish(control_active);

  if (result.events.reset_joint_request) {
    std_msgs::msg::Bool reset_request;
    reset_request.data = true;
    reset_request_pub_->publish(reset_request);
  }
  log_events(result);
}

void BwTeleop::log_events(const manager::TickResult & result)
{
  if (result.events.mode_changed) {
    const char * mode = result.output.control_mode == manager::ControlMode::relative ?
      "relative" : "absolute";
    RCLCPP_INFO(get_logger(), "Control mode changed to %s", mode);
  }
  if (result.events.left_reset) {
    RCLCPP_INFO(get_logger(), "Left arm reset requested");
  }
  if (result.events.right_reset) {
    RCLCPP_INFO(get_logger(), "Right arm reset requested");
  }
  if (result.events.both_reset) {
    RCLCPP_INFO(get_logger(), "Both arms reset requested");
  }
  if (result.events.head_reset) {
    RCLCPP_INFO(get_logger(), "Head zero reference reset requested");
  }
  if (result.events.speed_gear_changed) {
    RCLCPP_INFO(
      get_logger(), "Base speed gear changed to %d", result.output.base_speed_gear);
  }
  if (result.events.entered_safe_hold) {
    RCLCPP_ERROR(get_logger(), "VR watchdog timeout: SAFE_HOLD");
  }
  if (result.events.exited_safe_hold) {
    RCLCPP_INFO(get_logger(), "VR inputs restored after grip release");
  }
}

geometry_msgs::msg::Pose BwTeleop::to_ros_pose(const manager::Pose & pose) noexcept
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose.position.x;
  message.position.y = pose.position.y;
  message.position.z = pose.position.z;
  message.orientation.x = pose.orientation.x;
  message.orientation.y = pose.orientation.y;
  message.orientation.z = pose.orientation.z;
  message.orientation.w = pose.orientation.w;
  return message;
}

manager::Pose BwTeleop::to_internal_pose(const geometry_msgs::msg::Pose & pose) noexcept
{
  manager::Pose result;
  result.position = {pose.position.x, pose.position.y, pose.position.z};
  result.orientation = {
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
  return result;
}

}  // namespace bw_teleop::node
