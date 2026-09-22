# bw_teleop

Standard 单节点 PICO VR 遥操作包: 节点直接接收 UDP 数据报, 完成协议解析, 输入映射,
owner/session 管理, clutch, 复位, 头部角度处理, watchdog 与命令发布; 不使用自定义 VR
ROS message, 不通过 ROS Pose/Joy topic 串联内部阶段.

## 分层

```text
UDP datagram
    |
    v
BwTeleop Node
    |
    v
VrStreamManager -> ClientSessionManager / VrPacketParser / VrInputMapper
    |
    v
TeleopManager -> PoseMapper
```

- `transport`: ROS-free UDP 数据报值类型.
- `protocol`: PICO 控制帧解析.
- `input`: 四元数, 模拟量与摇杆范围检查及死区映射.
- `manager`: owner, timestamp, watchdog, SAFE_HOLD, 模式, 复位与命令编排.
- `algorithm`: ROS-free 双臂位姿映射与工作空间限制.
- `node`: 非阻塞 UDP, ROS 参数, JointState/实测位姿输入与命令发布.

默认节点名与可执行文件均为 `bw_teleop`.

## 启动

```bash
ros2 launch bw_teleop bw_teleop.launch.py
```

- `params_file`: 默认 `config/bw_teleop.yaml`.
- `bind_ip`: `0.0.0.0`; `port`: `12345`; `poll_period_ms`: `2`;
  `max_packets_per_cycle`: `64`; `publish_frequency_hz`: `90`.

## ROS 输入

- `/current_left_pose`, `/current_right_pose` (`geometry_msgs/msg/PoseStamped`):
  `bw_kinematics` 实测末端 FK; 非空 `frame_id` 必须等于 `target_frame`(默认 `C_Link`).
- `/joint_states` (`sensor_msgs/msg/JointState`): 按名称读取
  `A_left_Degree8_joint`, `A_right_Degree8_joint` 与 `C_joint` (米).

## ROS 输出

所有命令 publisher 使用 Reliable, Keep Last 1:

- `/target_left_pose`, `/target_right_pose` (`geometry_msgs/msg/PoseStamped`): `C_Link`
  下末端目标.
- `/gripper_controller/commands` (`std_msgs/msg/Float64MultiArray`):
  `[A_left_Degree8_joint, A_right_Degree8_joint]`, 米.
- `/lift_controller/commands` (`std_msgs/msg/Float64MultiArray`): `[C_joint]`, 米.
- `/base_controller/reference` (`geometry_msgs/msg/TwistStamped`): 机体 `vx`, `vy`, `wz`.
- `/Teleop/head_pose` (`sensor_msgs/msg/JointState`):
  `name = [head_pitch_joint, head_yaw_joint, head_roll_joint]`,
  `position = [pitch, yaw, roll]`, 弧度.
- `/teleop/control_active` (`std_msgs/msg/Bool`): 每周期发布, `!safe_hold` 时为 `true`.

双臂目标只在对应 clutch 有效且该侧实测末端新鲜时可发布. SAFE_HOLD 后停止双臂目标, 继续
发布底盘零速与头部保持值, 并在反馈新鲜时发布夹爪与升降保持值.

## PICO 协议与输入校验

直接控制 payload 固定 `143` 字节, 小端, magic `0x42575652`, version `1`; 同时支持
`BWVR` version 3 路由封装, 完整 HandJoints v2 帧按协议语义忽略. 解析器校验长度, 布尔
编码, 有限值, 四元数与控制字段.

- 四个摇杆轴必须在 `[-1, 1]` 且有限; trigger/grip 必须在 `[0, 1]` 且有限.
- 头显与已连接手柄的位姿必须有限且四元数有效; 越界值不静默钳制.
- `input_joystick_deadzone` 默认 `0.15` (输入映射阶段线性死区);
  `command_joystick_deadzone` 默认 `0.08` (底盘与升降命令阶段再次死区).

## Owner 和 timestamp

- `single_client_lock_enabled` 启用时首个有效来源 IP 获得 owner, 默认
  `client_lock_timeout_sec=3.0 s` 内拒绝其他来源; 超时未见的 owner 被释放.
- 只有 parser, mapper, 时间戳与 owner 全部通过的帧才刷新 heartbeat.
- 同一来源 sender timestamp 必须严格递增, 重复同样拒绝; 倒序或重复帧不延长 owner.
- owner 切换或超时释放清除 timestamp 历史; 非 owner 非法帧不清除当前 owner 输入.
- owner 或未锁定来源的非法帧清空 VR 输入并进入恢复流程.

## 控制语义

- 手柄 `grip` 控制对应手臂 clutch.
- 相对模式首次按下 clutch 前必须有该侧新鲜 current pose, 以实测位姿建立基线.
- 绝对模式以该侧 reset pose 加手柄位置生成目标, 最终按 workspace 限制.
- 左右手柄同时长按 `AX+BY` 超过 `long_press_sec` 切换相对/绝对模式.
- 单侧 `AX+BY` 长按复位对应手臂; 任一 menu 长按复位双臂.
- trigger 映射夹爪 `0.0..0.04965 m`.
- 左摇杆 `y` 前后, 左摇杆 `x` 横移, 右摇杆 `x` 偏航, 右摇杆 `y` 以 `mm/s` 调整升降.
- 左摇杆点击复位头部零位; 右摇杆点击切换底盘速度档.

## 升降和头部

- 升降内部固定毫米: 范围 `[-500, 0] mm`, 初始 `-100 mm`, jog speed `200 mm/s`; ROS
  输入从米转毫米, 输出从毫米转米; 不从启动姿态生成零偏.
- 头部四元数转换为 `[pitch, yaw, roll]`; 首个有效样本建立零参考, 左摇杆点击把当前
  展开角重设为零; watchdog, 无效头显, SAFE_HOLD 或断连恢复时保持最近一次有限且已夹紧
  的命令, 恢复后先 re-anchor, 不跳回默认零位.
- 头部限位: pitch `[-0.524, 0.785]`(ROS 侧; 固件侧 `[-0.785, 0.524]`, 固定 -1 变换的
  镜像), yaw `[-1.570, 1.570]`, roll `[-0.349, 0.349]`; 不读取头部 state feedback.

## 复位

- VR 首次接入或断流恢复 (`auto_reset_on_vr_connect: true`) 时向 `/kinematics/reset_arms`
  发一次 `Bool=true`, 由 KinematicsNode 下发 "实测关节 -> 零位" 的关节空间斜坡; teleop
  自身不生成笛卡尔路点.
- Reset 姿态为零关节角位姿 (`C_Link` 参考):
  `[0.329847758956320, ±0.178995684237027/-0.179005303496523, -0.263747491045859]`,
  姿态为单位四元数.
- 未连接的一侧手柄不参与新鲜度判定 (只保持该侧构型), 可单手操作.

## Watchdog 和反馈门控

- `watchdog_timeout_ms=200`; 头显与左右手柄 pose/joy 任一缺失, 断连或过期即进入
  SAFE_HOLD: 释放左右 clutch, 底盘零速, 头部保持最后有效值, 夹爪与升降在完整新鲜反馈
  时保持实测值, 并要求恢复后先释放双侧 grip 再建立新基线.
- 夹爪与升降反馈必须全部有效且在 `joint_state_timeout_ms=200` 内; 左右 current pose
  分别在 `arm_pose_timeout_ms=200` 内.
- 所有 topic, 范围, 超时, reset pose, workspace 与速度档参数在启动时检查, 非法配置
  直接拒绝启动.

## 关键默认值

```yaml
target_frame: C_Link
base_frame_id: base_link
single_client_lock_enabled: false  # 当前部署值; 置 true 时只接受首个来源 IP
left_reset_position: [0.329847758956320, 0.178995684237027, -0.263747491045859]
right_reset_position: [0.329847758956320, -0.179005303496523, -0.263747491045859]
workspace_min: [0.10, -0.80, -0.60]
workspace_max: [0.90, 0.80, 0.50]
lift_min_mm: -500.0
lift_max_mm: 0.0
lift_initial_position_mm: -100.0
head_min: [-0.524, -1.570, -0.349]
head_max: [0.785, 1.570, 0.349]
auto_reset_on_vr_connect: true
topics.control_active_output: /teleop/control_active
topics.reset_request_output: /kinematics/reset_arms
```

完整配置见 `config/bw_teleop.yaml`.


