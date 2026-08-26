# bw_std_control

## 职责

`bw_std_control` 是 Standard 机器人的 V3 `ros2_control` 控制包，包含异步串口、
固定帧编解码、执行器映射、系统硬件插件和机体速度控制器。它不包含机器人模型、
控制器 YAML、Launch、VR 输入或运动学；部署配置由 `bw_std_bringup` 管理。

核心映射和安全算法不依赖 ROS。串口系统调用只在 ASIO 线程执行；反馈通过
`RealtimeBuffer` 双缓冲交接，普通命令通过固定容量 SPSC 队列交接。软件掉电使用
独立优先队列和 emergency latch：锁存后拒绝新普通命令并抑制既有普通积压，直到
串口消费者确认掉电帧发送完成；session 关闭时也会安全复位 mailbox。
`controller_manager` 更新线程不会等待串口或反馈解析互斥锁。

关节坐标以 `bw_std_description/urdf/standard.urdf` 的零位、origin 和 axis 为
唯一真源。V3 安装零偏只在映射层换算，不通过修改 URDF 或补偿 TF 处理。
Standard 的电机映射必须独立标定：每侧手臂的 V3 通道索引、方向和零位均须
显式配置并严格校验，编码与解码使用同一映射保持可逆。

## 插件与接口

- `bw_std_control/StandardSystemHardware`：V3 系统硬件插件。
- `bw_std_control/StandardBaseController`：订阅 `~/reference`
  (`geometry_msgs/msg/TwistStamped`) 的底盘控制器。

硬件插件严格按名称查找 17 个主动关节：`C_joint`、左右
`A_*_Degree1_joint` 至 `A_*_Degree7_joint`，以及左右
`A_*_Degree8_joint`。每个关节必须提供 `position` 命令接口及
`position/velocity/effort` 状态接口。`Degree9` 是 URDF mimic 关节，不能导出
硬件接口。底盘仅提供 `base/vx`、`base/vy`、`base/wz` 命令接口。

头部和 waist 不导出状态或命令接口。发给 V3 的对应位置字段保持最近一次有限的
原始反馈值，三个最大速度字段始终为零，因此本包不会产生头部运动。

## 硬件参数

- `serial_port`：串口设备，默认 `/dev/ttyACM0`。
- `baud_rate`：波特率，默认 `2000000`。
- `feedback_timeout_ms`：反馈 watchdog，默认 `100` ms。
- `power_on_on_activate`：激活时软件上电，默认 `false`，只接受 `true/false`。
- `arm_mapping_calibrated`：Standard 双臂映射是否已完成逐轴标定，默认 `false`。
  只有它和 `power_on_on_activate` 同时为 `true` 才允许发送上电帧。
- `command_rate_hz`：V3 最大发送频率，默认 `40` Hz。
- 升降不提供运行时零偏或参考点参数。V3 位置毫米值直接对应源 URDF 的
  `C_joint` 米制值（`0 mm <-> 0 m`），编码使用严格逆式。
- `left_arm_motor_indices`、`right_arm_motor_indices`：按 URDF Degree1 到 Degree7
  排列的 V3 Motor 索引，必须是 `0..6` 的完整一一置换。
- `left_arm_direction`、`right_arm_direction`：按 Degree1 到 Degree7 排列的方向，
  每项只能为 `-1` 或 `1`。
- `left_arm_raw_zero_rad`、`right_arm_raw_zero_rad`：按 Degree1 到 Degree7 排列，
  保存对应 Motor 在源 URDF `q=0` 时的原始值。数组必须恰好 7 项且全部有限。

未标定默认使用 identity 索引、全 `+1` 方向和全零偏，仅用于掉电诊断，**不代表
Standard 实机映射**。单帧绝对位置不能确定通道、方向和零位；必须逐轴采集增量。
- `pelvis_max_velocity_mm_s`、`arm_max_velocity_rad_s`、
  `gripper_max_velocity_normalized_s`：执行器速度上限。
- `base_max_acceleration_x/y/omega`：V3 底盘加速度上限。

夹爪行程固定为 `0.04965 m`，V3 值 `0` 对应 URDF 零位（闭合），`1` 对应
`0.04965 m`（张开）。
底盘控制器参数为 `command_timeout_sec`、`max_vx`、`max_vy`、`max_wz`。

## 安全行为

激活前必须收到一帧新鲜、完整且有限的反馈，包含三个有限的头部原始位置字段。
首帧反馈用于同步所有状态和命令，
形成 measured-hold；默认不会软件上电。命令经过名称契约、有限值、URDF 限位和
单周期增量限制。串口故障、反馈超时或非法命令会停用写入并优先发送软件掉电帧，
掉电之后不会继续发送积压的上电命令。
底盘控制器还有独立 watchdog，超时、停用和非法输入均写零速度。故障后不会自动
重新上电，需由上层显式恢复生命周期。
零偏不会从启动首帧自动生成，避免机器人在任意姿态重启时静默重定义源 URDF 坐标。
未标定时即使请求 `power_on_on_activate=true`，插件也会拒绝软件上电并保持
`control_flag=0` 的只读反馈模式。

## 构建与测试

```bash
source /opt/ros/$ROS_DISTRO/setup.zsh
colcon build --packages-select bw_std_control --symlink-install
colcon test --packages-select bw_std_control
colcon test-result --verbose
```

自动化测试覆盖 V3 CRC/乱流重同步、双臂通道置换/方向/全轴零偏、
位置/速度/力矩及逆映射、
夹爪和升降换算、float 可表示性、首帧头部完整性、精确关节名、SPSC 满队列、
双缓冲交接、限位与单周期增量、底盘 watchdog、串口关闭行为和 pluginlib 加载。
当前只有旧链路的右臂经过有限真机验证；本包尚未完成 Standard 双臂、整机或
大规模 HIL 验证。
