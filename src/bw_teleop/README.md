# bw_teleop

`bw_teleop` 是 Standard 机器人的单一 VR 遥操作包。正式运行只启动一个
`bw_teleop` 节点：节点直接接收 PICO UDP 数据报，包内完成协议解析、输入映射、
clutch 与 watchdog 编排，再发布标准 ROS 控制接口。本包不依赖自定义 VR 消息，
也不通过 ROS Pose/Joy topic 串联内部阶段。

## 架构与边界

```text
UDP -> Node -> VrStreamManager -> TeleopManager -> PoseMapper
              |                 |
              +-> Parser/Input  +-> clutch/mode/watchdog/command routing
```

- `transport`：ROS-free UDP 数据报值类型。
- `protocol`：ROS-free PICO 143 字节控制帧与 v3 路由封装解析。
- `input`：ROS-free 四元数校验、摇杆死区和控制语义映射。
- `manager`：单客户端 owner、模式、复位、clutch、watchdog 与命令编排。
- `algorithm`：ROS-free 双臂位姿映射和工作空间限位。
- `node`：非阻塞 UDP IO、ROS 参数、JointState 输入和命令发布。

节点不发布 VR 私有消息、原始手柄 Pose/Joy 或头部命令。头显位姿仍随有效帧
进入 watchdog，供未来头部语义使用；当前不读取任何头部关节测量。

## 启动

```bash
colcon build --packages-select bw_teleop --symlink-install
source install/setup.bash
ros2 launch bw_teleop bw_teleop.launch.py
```

默认监听 `0.0.0.0:12345`。节点名、可执行文件和正式 launch 均为
`bw_teleop`。`poll_period_ms=2`，每轮最多读取 64 个数据报，socket 使用
`MSG_DONTWAIT`，不会阻塞 ROS executor。

## 输入与输出

ROS 输入包括：

- `/current_left_pose`、`/current_right_pose`
  (`geometry_msgs/msg/PoseStamped`)：由 `bw_kinematics` 提供的左右实测末端位姿。
  非空 `frame_id` 必须等于 `target_frame`（默认 `C_Link`），否则整帧丢弃。
- `/joint_states` (`sensor_msgs/msg/JointState`)：按名称读取以下关节：

- `A_left_Degree8_joint`、`A_right_Degree8_joint`：夹爪主动关节。
- `C_joint`：升降关节。

ROS 输出均使用 Reliable、Keep Last 1：

- `/target_left_pose`、`/target_right_pose`
  (`geometry_msgs/msg/PoseStamped`)，默认 frame 为 `C_Link`；对应 IK tip 为
  `A_left_Degree7_link`、`A_right_Degree7_link`。相对模式首次按下 grip 时，必须
  先收到该侧新鲜的 current pose，并以它作为机器人基线；不会使用 reset pose
  替代测量基线。
- `/gripper_controller/commands` (`std_msgs/msg/Float64MultiArray`)，顺序为
  `[A_left_Degree8_joint, A_right_Degree8_joint]`。
- `/lift_controller/commands` (`std_msgs/msg/Float64MultiArray`)，顺序为
  `[C_joint]`。
- `/base_controller/reference` (`geometry_msgs/msg/TwistStamped`)。

## 协议与输入语义

控制 payload 固定为 143 字节、小端、magic `0x42575652`、version `1`。也支持
`"BWVR" + version 3 + flags + SN length + SN + payload` 路由封装。解析器严格
校验长度、布尔编码、有限值和四元数，并静默忽略完整 HandJoints v2 帧。

首个有效来源 IP 获得 owner；默认连续 `3s` 无有效帧后才允许其他 IP 接管。
摇杆先应用 `input_joystick_deadzone=0.15` 线性死区映射。`grip` 控制对应手臂
clutch，`trigger_value` 控制夹爪，左摇杆控制底盘平移，右摇杆控制偏航和升降。
双侧 `AX+BY` 长按切换相对/绝对模式，单侧长按复位该臂，menu 长按复位双臂，
右摇杆点击切换底盘速度档。

## Standard 限位与安全

- 夹爪范围：`0.0..0.04965m`。
- 升降初值：源 URDF 零位 `0.0m`；范围：`[-0.231, 0.231]m`。收到完整反馈后
  立即使用实测位置建立 measured-hold。
- watchdog 默认 `200ms`。头显、任一手柄或任一控制器输入不完整/超时即进入
  `SAFE_HOLD`，底盘立即归零，夹爪和升降保持最新有效测量值。
- 双臂目标仅在对应 clutch 按下或触发显式复位的周期发布；释放 clutch 或进入
  `SAFE_HOLD` 后停止发布，避免持续触发 IK 与新轨迹。
- 任一 current pose 超过 `arm_pose_timeout_ms=200` 未刷新时，停止该侧目标输出
  并撤销 clutch；反馈恢复后需重新建立 measured baseline。
- 左右夹爪与升降三项反馈未全部到达，或任一反馈超过
  `joint_state_timeout_ms=200` 未刷新时，不发布夹爪或升降命令。首次反馈按实测值
  初始化 measured-hold。
- 输入恢复后必须先释放双侧 grip，下一次按下才重新建立 clutch 基准。
- 所有控制配置见 `config/bw_teleop.yaml`；topic、关节名和限位变更必须同步
  检查 `bw_std_bringup` 与 controller 配置。
- 所有运动参数与数组元素必须有限，超时必须为正数，min/max、reset pose、
  workspace、速度档和初值必须满足范围约束；非法配置会拒绝节点启动，不会静默
  交换、钳制或替换为默认值。

## 测试

```bash
colcon test --packages-select bw_teleop
colcon test-result --verbose
```

单元测试覆盖直接/路由控制帧、HandJoints 忽略、单客户端锁、输入死区、
Standard 夹爪与升降限位、首次 measured-hold、相对模式实测末端基线、反馈过期、
非法参数拒绝、clutch/watchdog 停止目标，以及 watchdog 底盘归零。真机仍需逐
模块低速验证双臂方向、夹爪开合、升降方向和底盘坐标符号。
