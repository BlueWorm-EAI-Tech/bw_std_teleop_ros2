# bw_std_description

Standard 机器人模型与 ros2_control 资源契约包:

- `urdf/standard.urdf`: 几何, joint origin, axis, 惯性与 visual/collision 真源.
- `urdf/standard.xacro`: 真机与 Mock 组合入口.
- `urdf/standard.ros2_control.xacro`: 统一 ros2_control joint/GPIO 接口.
- `meshes/` 与显示 Launch.

驱动, 编解码, 控制器, VR 输入与运动学算法不属于本包.

## 模型契约

- 运动学根 `C_Link`, 末端 `A_left_Degree7_link` / `A_right_Degree7_link`, 每侧 7 轴.
- 每侧主动 `Degree8` 夹爪与 URDF mimic 驱动的 `Degree9`; `Degree9` 不导出独立硬件接口.
- `C_joint` 限位 `[-0.5, 0.0] m`, ros2_control 初始值 `-0.1 m`; Teleop 内部使用
  `-500..0 mm`, 默认 `-100 mm`, 在 ROS 边界转换.
- 算法使用独立 reduced model `bw_kinematics/assets/standard_ik/standard_ik.urdf`, 锁定
  `C_joint` 到 `0 m`; 两个模型的控制单位, 锁定语义与路径不得混用.

`standard.ros2_control.xacro` 对真机选择 `bw_std_control/StandardSystemHardware`, 对 Mock
选择 `mock_components/GenericSystem`, 两者使用相同的接口集合. Xacro 参数与默认值见
`urdf/standard.xacro`, bringup launch 按部署覆盖运动与带宽限制.

## ros2_control 接口

17 个主动关节, 每个只提供 `position` command interface 与 `position`, `velocity`,
`effort` state interface:

```text
C_joint
A_left_Degree1_joint ... A_left_Degree7_joint
A_right_Degree1_joint ... A_right_Degree7_joint
A_left_Degree8_joint
A_right_Degree8_joint
```

command-only GPIO, 不提供 state interface:

```text
base/vx, base/vy, base/wz
head/pitch, head/yaw, head/roll
safety/power
```

头部 command 顺序固定为 pitch, yaw, roll:

```text
pitch [-0.524, 0.785] rad   # ROS 侧; 固件侧 [-0.785, 0.524] 经固定 -1 变换的镜像
yaw   [-1.570, 1.570] rad
roll  [-0.349, 0.349] rad
```

`safety/power` 为 `0/1` 命令接口, 写入与超时语义由 `bw_std_control` 底盘控制器负责;
固定底盘轮不导出虚构的轮关节反馈; 头部硬件状态不伪造.

## 安全边界

- 不通过修改 URDF origin, axis, TF 或放宽 joint limits 补偿电机映射.
- `C_joint` 只在 ros2_control 边界使用米.
- 默认不软件上电, 默认不自动激活运动控制器.
- 头部限位必须包含安全零位 `0`.
- 所有主动关节必须有有限, 有序的 position min/max.
- 真机硬件插件校验 base/head/safety GPIO 的精确 command-only 资源契约.

## 查看和静态检查

```bash
xacro src/bw_std_description/urdf/standard.xacro use_mock_hardware:=true \
  | check_urdf /dev/stdin

ros2 launch bw_std_description display.launch.py
```

## 验证状态

- Humble 与 Jazzy Mock bringup 均加载 `standard.xacro`, GenericSystem, 17 个主动关节与
  base/head command-only GPIO; KinematicsNode 以 `C_Link` 为根发布 FK.
- Jazzy `xacro | check_urdf` 检查通过.
- 真机逐轴映射标定, HIL 与低速 commissioning 未完成; 整机结论见 `bw_std_bringup` README.
