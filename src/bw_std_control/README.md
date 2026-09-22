# bw_std_control

Standard `ros2_control` 控制包:

- `StandardSystemHardware`: 异步串口系统硬件插件.
- `StandardBaseController`: 底盘 `TwistStamped` 控制器.
- `StandardHeadController`: 头部 command-only GPIO 控制器.
- frame codec, 反馈 handoff, 固定容量命令队列与命令安全检查.

模型与接口契约由 `bw_std_description` 提供, 部署组合由 `bw_std_bringup` 提供.

## 资源契约

硬件必须严格提供:

- 17 个主动关节, 每关节只提供 `position` command interface 与 `position`, `velocity`,
  `effort` state interface:

```text
C_joint
A_left_Degree1_joint ... A_left_Degree7_joint
A_right_Degree1_joint ... A_right_Degree7_joint
A_left_Degree8_joint
A_right_Degree8_joint
```

- command-only GPIO, 无 state interface:

```text
base/vx, base/vy, base/wz
head/pitch, head/yaw, head/roll
```

- `Degree9` 是 URDF mimic, 不导出独立硬件接口.
- 头部命令顺序固定 pitch, yaw, roll, 限位:

```text
pitch [-0.524, 0.785] rad   # ROS 侧; 固件侧 [-0.785, 0.524] 经固定 -1 变换的镜像
yaw   [-1.570, 1.570] rad
roll  [-0.349, 0.349] rad
```

`StandardHeadController` 订阅 `/Teleop/head_pose`, 只接受三个正式头部关节名与完整有限的
位置; 不申请头部 state interface, 命令过期, 非法或 controller 停用时保持最近安全位置.

## 协议

- 帧头 `0x55 0xAA`; command type `0x01`, feedback type `0x02`; CRC 2 字节.
- command payload `249 B` (frame `256 B`); feedback payload `248 B` (frame `255 B`).
- 下盘 (底盘/滑台) 独立控制器 `0x21` 帧: payload `37 B` (frame `44 B`); 激活后先连发
  `200` 个唤醒帧 (带软件开关位, 速度全零, 滑台最大速度 `0`), 之后回到整机 `0x01` 帧.
- `CommandPayload` offset `177` 固定为 `head_roll_position`, 不得恢复旧头部速度字段语义;
  waist 编码固定 `0`; 软件掉电帧保持有限安全位置字段.

## 映射和单位

每侧手臂必须配置:

- `*_arm_motor_indices`: `0..6` 的完整一一置换.
- `*_arm_direction`: 每项只能为 `-1` 或 `1`.
- `*_arm_raw_zero_rad`: 恰好 7 个有限原始零位.

```text
q = direction * (raw_position - raw_zero)
raw_position = direction * q + raw_zero
```

- pelvis height 用毫米, velocity 用毫米每秒, ROS `C_joint` 用米.
- 夹爪 ROS `0..0.04965 m` 对应协议 `0..1`.
- 头部 ROS `[pitch, yaw, roll]` 按协议固定符号转换; feedback 无 roll state field, roll
  hold 取最近一次有限命令.

未标定默认值 (identity index, 全 `+1` direction, 全零 raw-zero) 只用于掉电诊断与接口
联调; 没有现场逐轴数据时不得启用真实运动.

## 硬件参数

- `serial_port` 默认 `/dev/ttyACM0`; `baud_rate` 默认 `2000000`;
  `feedback_timeout_ms` 默认 `100`.
- `command_rate_hz` 默认 `40`, Standard bringup 传 `200` 
- `power_on_on_activate` 默认 `false`; `arm_mapping_calibrated` 默认 `false`.
- `left/right_arm_motor_indices`, `left/right_arm_direction`,
  `left/right_arm_raw_zero_rad`.
- `pelvis_max_velocity_mm_s` 默认 `200`.
- `arm_max_velocity_rad_s` 默认 `1.0`, Standard bringup 传 `12.0` (对齐 standard_0907
  `ruckig.max_velocity`), 作为每周期步长上限与每关节 `max velocity` 字段上限.
- `gripper_max_velocity_normalized_s` 默认 `1.0`.
- `arm_startup_limit_tolerance_rad` 默认 `0`, 上限 `0.002`;
  `gripper_startup_limit_tolerance_m` 默认 `0`, 上限 `0.001`.
- `head_max_velocity_rad_s` 默认 `1.0`.
- `base_max_acceleration_x`, `base_max_acceleration_y`, `base_max_acceleration_omega`:
  默认值由映射参数结构提供.

只有 `power_on_on_activate=true` 且 `arm_mapping_calibrated=true` 时, 正常 command frame
才允许使用软件上电 control flag; 否则激活帧与运行路径保持 `control_flag=0`.

控制字与 standard_0907 一致: `bit0` 软件上电, `bit2` 底盘, `bit3` 左臂, `bit4` 右臂,
`bit5` 头部. 控制活跃发 `0x3D`; `safety/power` 为 `0` (默认, `/teleop/control_active`
为 false 或超时 `0.5 s`) 时持续发送 `control_flag=0` 的安全保持帧, 即软件掉电.

## 软件上电时序

不允许直接跳到 `0x3D`:

1. 未上电状态先取权威 measured-hold: 连续 `5` 帧臂位置字段有限即认为可用, 等待上限
   `3000 ms` (位置字段可信, 即使右臂状态掩码为 `0x00`); 超时拒绝激活.
2. 发关闭双臂位的 `0x25` 控制字 (软件上电 + 底盘 + 头部) 并保持至少 `10 ms`, 且该帧写入
   串口完成后才发 `0x3D` 整字使能帧.
3. 使能过渡帧使用步骤 1 的实测保持位置, 上电后不重读: 上电短窗口内右臂会短暂上报零值
   位置 (臂实际姿态不变), 重读会把旧零值当成保持位置下发.

预充电期间任一臂反馈转为非有限值, 出现掉电请求或写入未完成都会作废本次时序; 重新上电
必须重新满足上述条件. 每关节 `max velocity` 字段由本帧指令增量推算, 上限为
`arm_max_velocity_rad_s`.

## 生命周期和安全行为

- 激活前必须收到新鲜, 完整且有限的 feedback, 包含 17 个主动关节状态字段以及 waist,
  head yaw, head pitch 的有限字段; 首帧反馈初始化状态与 command measured-hold: 17 个
  关节命令同步到实测位置, base 三轴同步为 `0`, 头部同步到 feedback hold, waist 保持 `0`.
- 激活时 measured-hold 必须通过完整关节与头部限位检查; 默认容差 `0`, 显式配置容差后只
  允许容差内边界偏差且只允许朝硬限位内恢复, 超容差拒绝激活并打印具体关节与数值.
- 更新线程不等待串口或反馈解析锁; 串口线程解析 frame 后通过 feedback handoff 交给
  ros2_control; 普通命令使用固定容量队列, 软件掉电使用独立优先路径和 latch.
- 以下情况停止 active command 路径并请求优先软件掉电: 串口 transport 故障或关闭;
  feedback 超时; feedback 非有限, 缺失或越过配置限位; ros2_control interface 访问失败;
  command 非有限或单周期增量超限; command 编码或队列失败.
- 指令位置越过模型限位不再直接掉电: 非有限值仍故障, 有限值记录 `command clamped` 并
  截断到限位内; 反馈状态字变化会打印 `feedback status changed`.
- 故障后不自动重新上电, 必须重新走生命周期恢复; 未标定时即使请求
  `power_on_on_activate=true`, 也只允许掉电反馈模式.

控制器:

- `StandardBaseController` 订阅 `~/reference`, 默认 `0.1 s` command timeout, 限速或非法
  输入时输出零速度; 订阅 `control_active_topic` (默认 `/teleop/control_active`, 超时
  `control_active_timeout_sec` 默认 `0.5 s`) 并写入 `safety/power` 命令 GPIO.
- `StandardHeadController` 订阅 `/Teleop/head_pose`, 具有独立 timeout, 完整关节名检查,
  限位检查和安全保持.

## 构建

```bash
colcon build --packages-select bw_std_control --symlink-install
```
