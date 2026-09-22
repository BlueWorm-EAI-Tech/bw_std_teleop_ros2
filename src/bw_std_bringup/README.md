# bw_std_bringup

Standard ros2_control, 运动学与 VR 遥操作链路的部署组合包; 只保存 Launch, controller
manager 配置与启动顺序.

## 启动入口

真机: 默认不上电, 运动控制器 inactive, 不启动 Teleop 与运动学.

```bash
ros2 launch bw_std_bringup standard.launch.py
```

```text
use_mock_hardware=false
power_on_on_activate=false
arm_mapping_calibrated=false
activate_motion_controllers=false
start_teleop=false
serial_port=/dev/ttyACM0
baud_rate=2000000
feedback_timeout_ms=100
```

真机入口即使默认不上电也会配置串口, 执行前必须确认设备与硬件操作已获批准.

Mock: 不接硬件, 自动激活全部控制器, 启动 Teleop 与运动学.

```bash
ros2 launch bw_std_bringup standard_mock.launch.py
```

```text
use_mock_hardware=true
activate_motion_controllers=true
power_on_on_activate=false
start_teleop=true
```

Mock 使用 `mock_components/GenericSystem`; 无有效 PICO 数据时 Teleop 必须进入
SAFE_HOLD.

## 控制器

配置见 `config/standard_controllers.yaml`:

- `joint_state_broadcaster`: 关节状态.
- `dual_arm_controller`: 14 个位置关节, 左 `Degree1..7` 后接右 `Degree1..7`, 不允许
  partial goal, 不使用 open-loop.
- `gripper_controller`: `[A_left_Degree8_joint, A_right_Degree8_joint]`.
- `lift_controller`: `[C_joint]`.
- `base_controller`: `base/vx`, `base/vy`, `base/wz`, command timeout `0.1 s`.
- `head_controller`: `head/pitch`, `head/yaw`, `head/roll`, command timeout `0.2 s`.

## Launch 参数

传往 Standard Xacro 与硬件插件:

```text
use_mock_hardware, serial_port, baud_rate, feedback_timeout_ms
power_on_on_activate, arm_mapping_calibrated
left/right_arm_motor_indices, left/right_arm_direction, left/right_arm_raw_zero_rad
arm_max_velocity_rad_s=12.0
pelvis_max_velocity_mm_s=200.0
gripper_max_velocity_normalized_s=1.0
head_max_velocity_rad_s=1.0
command_rate_hz=200.0    # 2 Mbaud 带宽约束, 见 bw_std_control README
```

控制器选项:

- `activate_motion_controllers`: 默认 `false`.
- `start_teleop`: 默认 `false`; 为 `true` 时同时 include `bw_teleop` 与 `bw_kinematics`
  启动文件.

默认 mapping 为 identity index, 全 `+1` direction, 全零 raw-zero, 仅用于接口与掉电诊断;
实机必须先逐轴确认 motor index, direction 与 raw-zero.

## 运行时连接

```text
standard.xacro
   |
   +-> robot_state_publisher
   +-> ros2_control_node
           |
           +-> StandardSystemHardware or GenericSystem
           +-> controller_manager
                   |
                   +-> dual_arm_controller
                   +-> gripper_controller
                   +-> lift_controller
                   +-> base_controller
                   +-> head_controller

start_teleop=true  -> bw_teleop -> target/head/auxiliary topics
                             |
                             +-> bw_kinematics -> dual_arm_controller
```

运动学节点使用包内 `bw_kinematics/assets/standard_ik/standard_ik.urdf`, 不依赖 bringup
传递 `robot_description`; 完整模型描述只用于状态发布与 ros2_control 资源解析.

## 真机安全顺序

1. 保持 `power_on_on_activate=false`, `arm_mapping_calibrated=false`,
   `activate_motion_controllers=false`, `start_teleop=false`, 只做反馈与资源核对.
2. 核对 17 个主动关节名称, 状态字段, 通道, 方向, 单位与模型限位.
3. 填入现场逐轴 motor index, direction 与 raw-zero; 禁止用默认 identity 代替标定.
4. 获得明确批准后单独打开软件上电, 先确认 measured-hold 与故障掉电路径.
5. 按双臂, 夹爪, 升降, 底盘, 头部低速顺序逐项放行控制器.
6. 最后启动 Teleop 与运动学, 验证 owner, watchdog, 目标坐标系与 SAFE_HOLD.

上电/掉电链路: 遥操作发布 `/teleop/control_active`, 底盘控制器写 `safety/power` 命令
GPIO; VR 断流 `0.5 s` 后软件掉电. VR 接入或断流恢复时经 `/kinematics/reset_arms` 触发
`3 s` 关节空间斜坡复位. 故障后必须重新激活生命周期.

## 构建

```bash
colcon build --packages-up-to bw_std_bringup --symlink-install
```

