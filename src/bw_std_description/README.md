# bw_std_description

`bw_std_description` 是 Standard 机器人唯一的模型与硬件接口契约包，提供
URDF/Xacro、mesh、RViz 配置和 `ros2_control` 描述。驱动、控制器、串口协议和
业务节点不属于本包。

## 模型与接口

顶层模型为 `urdf/standard.xacro`。左右臂各 7 个关节、两侧主动夹爪
`Degree8` 和升降 `C_joint` 导出位置命令及 position/velocity/effort 状态。
`Degree9` 通过 `multiplier=1.0` mimic 对应的 `Degree8`。三个底盘轮为固定关节，
底盘只通过 `base/vx`、`base/vy`、`base/wz` GPIO 接收机体速度命令。头部三轴
保留在模型中，但当前不导出硬件接口。

`standard.xacro` 直接包含源模型副本 `standard.urdf`。该副本的 joint origin、
axis 和所有 link visual/collision/inertial origin 与 `../standard/urdf/standard.urdf`
一致；发布适配不得修改这些坐标。包 URI、运行限位、固定轮和夹爪 mimic 属于非坐标
运行适配。

真机与 mock 模型共用 command-only `base` GPIO 契约。Humble/Jazzy 的
`mock_components/GenericSystem` 直接导出这三项命令接口；底盘没有状态接口，
因此不会生成虚拟底盘或轮关节反馈。所有 `ros2_control` joint 都必须对应模型中的
非固定 URDF joint，满足 Jazzy 的资源解析约束。

## Xacro 参数

- `use_mock_hardware`：默认 `false`；`true` 使用
  `mock_components/GenericSystem`。
- `serial_port`、`baud_rate`、`feedback_timeout_ms`：V3 串口通信参数。
- `power_on_on_activate`：默认 `false`，禁止激活时自动上电。
- `arm_mapping_calibrated`：默认 `false`；未完成 Standard 逐轴映射标定时禁止上电。
- 升降没有零偏或参考点 Xacro 参数；控制包仅进行 `mm <-> m` 单位换算，
  `0 mm` 对应源 URDF `C_joint=0`，不修改本包的 URDF origin 或 axis。
- `left_arm_motor_indices`、`right_arm_motor_indices`：Degree1 到 Degree7 对应的
  V3 Motor 索引；未标定默认是 `0,1,2,3,4,5,6`，只用于掉电诊断。
- `left_arm_direction`、`right_arm_direction`：Degree1 到 Degree7 的方向；未标定
  默认全 `+1`，不代表 Standard 实机结论。
- `left_arm_raw_zero_rad`、`right_arm_raw_zero_rad`：Degree1 到 Degree7 的 V3
  原始零位数组，默认 `0,0,0,0,0,0,0`。
- `arm_max_velocity_rad_s`、`pelvis_max_velocity_mm_s`、
  `gripper_max_velocity_normalized_s`：默认 `0.05`、`20.0`、`0.2`，用于
  commissioning 阶段的单周期增量限制。

展开并检查 mock 模型：

```bash
xacro src/bw_std_description/urdf/standard.xacro use_mock_hardware:=true \
  | check_urdf /dev/stdin
```

显示模型：

```bash
ros2 launch bw_std_description display.launch.py
```

## 安全与验证

真机插件为 `bw_std_control/StandardSystemHardware`。首次运行必须保持
`power_on_on_activate=false`、`arm_mapping_calibrated=false`，先核对完整反馈、
关节名称、通道、方向、单位和限位，再由 bringup 分模块低速激活。头部在嵌入式
契约完成前保持禁用。当前仅旧链路右臂经过有限真机验证，不代表整机经过大规模测试
或达到生产就绪状态。

运行包内契约测试：

```bash
colcon test --packages-select bw_std_description
colcon test-result --verbose
```
