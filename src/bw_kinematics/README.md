# bw_kinematics

## 1. 概述

`bw_kinematics` 是 Standard 真机遥操作链路的双臂运动学包。它从统一的
`robot_description` 解析左右手臂 KDL 链，以真机 `/joint_states` 作为每次求解
的 seed，将左右末端 `PoseStamped` 目标转换为 14 轴短时域
`JointTrajectory`，交给 `joint_trajectory_controller` 插值执行。

本包使用同一份 C++17 源码支持 ROS 2 Humble 与 Jazzy，不包含发行版条件分支。

## 2. 职责边界

范围内：

- 从 `C_Link` 到左右 `A_*_Degree7_link` 解析 KDL Chain。
- 提供 FK、Jacobian 和固定最大迭代次数的阻尼最小二乘 IK。
- 应用 URDF 硬关节限位、零空间关节居中、单次迭代最大关节增量和有限值检查。
- 使用测量关节位置作为 IK seed，并维护双臂最后有效命令。
- 单臂求解失败时保持该臂最后有效目标，另一臂仍可独立更新。

范围外：

- 不处理 VR 按键、相对/绝对模式、复位或 watchdog 状态机。
- 不生成 500 Hz 插值，不直接访问 ros2_control command interface。
- 不发布 `/joint_states`，不把目标关节伪装成反馈。
- 不做碰撞检测、动力学补偿或 MoveIt Servo 规划。
- 不做坐标变换；输入位姿必须已在 `root_link` 坐标系表达。

## 3. 分层架构

```text
KinematicsNode (Node)
  ROS 参数、订阅、消息校验、JointTrajectory 发布
          |
          v
KinematicsManager (Manager)
  测量状态缓存、左右臂调度、最后有效命令保持
          |
          v
DlsIkSolver (Algorithm)
  URDF/KDL、FK、Jacobian、DLS、限位与有限值检查
```

Algorithm 层不依赖 `rclcpp` 或 ROS 消息；Manager 层不创建任何 ROS 实体；Node
层不实现 IK 数值算法。

## 4. 目录

```text
bw_kinematics/
├── include/bw_kinematics/
│   ├── algorithm/dls_ik_solver.hpp
│   ├── manager/kinematics_manager.hpp
│   └── node/kinematics_node.hpp
├── src/
│   ├── algorithm/dls_ik_solver.cpp
│   ├── manager/kinematics_manager.cpp
│   └── node/
│       ├── kinematics_node.cpp
│       └── kinematics_node_main.cpp
├── config/kinematics.yaml
├── launch/kinematics.launch.py
├── CMakeLists.txt
└── package.xml
```

## 5. 节点与组件

- 节点名：`kinematics_node`
- 组件插件：`bw_kinematics::KinematicsNode`
- 独立可执行文件：`kinematics_node`
- 正式启动文件：`kinematics.launch.py`

节点构造时必须收到非空、可解析的 `robot_description`。模型缺链、关节无硬限位
或参数非法时节点直接拒绝启动，避免在真机链路中带病运行。

## 6. Topic 接口

订阅：

- `/joint_states` (`sensor_msgs/msg/JointState`)：ros2_control 真机测量反馈。
- `/target_left_pose` (`geometry_msgs/msg/PoseStamped`)：左腕绝对目标位姿。
- `/target_right_pose` (`geometry_msgs/msg/PoseStamped`)：右腕绝对目标位姿。

发布：

- `/current_left_pose` (`geometry_msgs/msg/PoseStamped`)：最新左臂测量关节的 FK 位姿。
- `/current_right_pose` (`geometry_msgs/msg/PoseStamped`)：最新右臂测量关节的 FK 位姿。
- `/dual_arm_controller/joint_trajectory`
  (`trajectory_msgs/msg/JointTrajectory`)：按 KDL 链顺序排列的完整 14 轴目标。

所有 Topic 名均可通过参数覆盖。目标位姿 `frame_id` 为空时按 `root_link` 解释；
非空时必须与 `root_link` 完全一致，否则丢弃。需要 TF 变换时应由上游
`bw_teleop` 完成。

## 7. 参数

- `robot_description`：展开后的 URDF XML，无默认有效值，必须由 bringup 注入。
- `root_link`：左右臂共同根链接，默认 `C_Link`。
- `left_tip_link`：左臂末端，默认 `A_left_Degree7_link`。
- `right_tip_link`：右臂末端，默认 `A_right_Degree7_link`。
- `joint_state_topic`：测量关节输入，默认 `/joint_states`。
- `left_target_topic`：左末端目标，默认 `/target_left_pose`。
- `right_target_topic`：右末端目标，默认 `/target_right_pose`。
- `left_current_pose_topic`：左末端测量 FK 输出，默认 `/current_left_pose`。
- `right_current_pose_topic`：右末端测量 FK 输出，默认 `/current_right_pose`。
- `joint_state_timeout_ms`：双臂完整测量反馈有效期，默认 `200 ms`；超时后拒绝 FK 和 IK。
- `trajectory_topic`：双臂轨迹输出，默认
  `/dual_arm_controller/joint_trajectory`。
- `max_iterations`：每帧单臂最大 IK 迭代次数，默认 `40`。
- `damping`：DLS 阻尼系数，默认 `0.05`。
- `max_joint_step`：每次 IK 迭代单关节最大变化，默认 `0.04 rad`。
- `position_tolerance`：收敛位置误差，默认 `0.001 m`。
- `orientation_tolerance`：收敛姿态误差，默认 `0.008726646 rad`（0.5 度）。
- `joint_centering_gain`：零空间关节居中增益，默认 `0.02`，设为 `0` 可关闭。
- `trajectory_duration_sec`：轨迹点执行时域，默认 `0.08 s`。

## 8. 求解与失败语义

1. `/joint_states` 按正式 URDF 关节名映射，不依赖消息数组顺序；单帧必须包含完整、
   有限的双臂 14 轴，缺失、非有限或超时会立即撤销就绪状态。
2. 收齐左右臂测量状态后，初始双臂命令同步为测量位置，避免首帧跳变。
   反馈超时后恢复时也会重新同步双臂 measured-hold，不沿用超时前命令。
3. 收到某侧目标后，该侧使用最新测量位置作为 seed 执行 IK。
4. 每步通过 DLS 伪逆求任务空间修正，并在 Jacobian 零空间加入关节居中项。
5. 每次增量被 `max_joint_step` 限制，更新结果再钳制到 URDF 硬限位。
6. 未收敛、非有限值或 KDL 失败时不采用候选解，保持该侧最后有效目标。
7. 成功或失败保持都会发布完整 14 轴目标，避免只更新一侧破坏 JTC 关节集合。

该“保持”只处理单帧 IK 失败，不替代 `bw_teleop` 的 VR watchdog，也不替代
硬件层串口故障停机。

## 9. 构建与启动

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon --log-base "log/${ROS_DISTRO}" build \
  --build-base "build/${ROS_DISTRO}" --install-base "install/${ROS_DISTRO}" \
  --symlink-install --packages-select bw_kinematics
source "install/${ROS_DISTRO}/setup.bash"
```

正式系统由 `bw_std_bringup` 展开 Standard xacro，并将同一
`robot_description` 参数传给本组件。例如在上级 launch 中加载：

```python
ComposableNode(
    package="bw_kinematics",
    plugin="bw_kinematics::KinematicsNode",
    name="kinematics_node",
    parameters=[kinematics_yaml, {"robot_description": robot_description}],
)
```

包内 `kinematics.launch.py` 也接受 `robot_description` 和 `parameters_file` 启动
参数，但空模型无法启动，这是有意的真机安全约束。

## 10. 依赖与兼容性

核心依赖：`orocos_kdl_vendor`、`kdl_parser`、`urdf`、`Eigen3`、`rclcpp`、
`rclcpp_components`、`geometry_msgs`、`sensor_msgs` 和 `trajectory_msgs`。

- Humble：Ubuntu 22.04，C++17。
- Jazzy：Ubuntu 24.04，C++17。
- 环境选择只读取 `ROS_DISTRO`，本包不硬编码具体发行版路径。

## 11. 当前限制与真机注意事项

- 已在 Ubuntu 22.04 / ROS 2 Humble 完成构建、Manager/配置测试和 current-pose
  Node 集成测试；Jazzy 尚未编译。
- DLS 对奇异点采用固定阻尼，尚未根据最小奇异值自适应调整。
- 末端不可达时会保持旧目标；上层应限制工作空间并监控连续失败次数。
- 上真机前仍需验证关节方向、KDL 链顺序、轨迹控制器关节集合和低速单臂动作。
- 参数默认值是 MVP 起点，必须通过真机分级放行后再确定生产值。
