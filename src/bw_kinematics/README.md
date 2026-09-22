# bw_kinematics

Standard ROS-free 双臂 FK, Jacobian 与 CasADi 联合 IK; ROS 节点把安全通过的 14 轴位置
目标发布给 `joint_trajectory_controller`. 不提供 VR 输入, 控制器生命周期, 串口访问与
动力学补偿.

## 模型约定

- root frame `C_Link`; 左末端 `A_left_Degree7_link`, 右末端 `A_right_Degree7_link`.
- reduced model 锁定 `C_joint` 到 `0 m`.
- 14 轴顺序固定:

```text
A_left_Degree1_joint ... A_left_Degree7_joint
A_right_Degree1_joint ... A_right_Degree7_joint
```

算法模型为 `assets/standard_ik/standard_ik.urdf`, 与完整机器人描述分开维护, 节点默认从
包 share 目录读取, 可由 `ik_model_path` 覆盖. 公共 header 只暴露 STL 与项目自定义类型.


## 分层

```text
KinematicsNode
  参数、ROS 消息校验、JointState 转换、JointTrajectory 发布
       |
       +---------------------------------+
       |                                 |
       v                                 v
KinematicsManager                 StandardTrajectorySmoother
  14 轴反馈缓存、freshness、        14 轴目标 -> 高频平滑轨迹
  双臂目标合成、限位、步长          (Ruckig 在线追踪 + 速度前馈)
       |
       v
StandardKinematicsModel / StandardFkSolver / StandardIkSolver
  由本包内 Algorithm Provider 提供
```

Algorithm 层不依赖 `rclcpp` 或 ROS message; Manager 层不创建 ROS 实体; Node 层不实现
IK 数值算法与轨迹平滑. 公共接口唯一位于 `include/bw_kinematics/algorithm/`. 两个实现默认
均为 `PREBUILT` (本包不携带算法源码); `SOURCE` 是 `algorithm` 的备用构建开关, 用于自行
提供源码, 平滑器没有该开关.

## Algorithm 接口

- FK: `StandardFkSolver` 提供单臂与双臂 FK; 输入必须是完整, 有限且处于 URDF 限位内的
  位置, 非法 `ArmSide`, 长度错误, 非有限值或越限输入直接失败.
- Jacobian: 单臂 `6 x 7`, 列顺序为对应侧 `Degree1..7`, 行优先且行序固定:

```text
linear x, linear y, linear z, angular x, angular y, angular z
```



```text
max_iterations=80            # 默认
tol_pr = tol_du = 1e-1       
continuity_weight=0.02
max_position_residual=0.03 m; max_orientation_residual=0.25 rad
```


## ROS Topic

订阅:

- `/joint_states` (`sensor_msgs/msg/JointState`): 完整双臂实测位置.
- `/target_left_pose`, `/target_right_pose` (`geometry_msgs/msg/PoseStamped`): 末端目标.
- `/kinematics/reset_arms` (`std_msgs/msg/Bool`): `true` 触发关节空间复位 (实测关节 ->
  零位, `reset_duration_sec` 默认 `3.0 s`, 共 `60` 个采样点).

发布:

- `/current_left_pose`, `/current_right_pose` (`geometry_msgs/msg/PoseStamped`): 实测
  14 轴 FK.
- `/dual_arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`): 完整
  14 轴位置轨迹, 由 `tracking_rate_hz` (默认 `200 Hz`) 的 Ruckig 在线追踪器逐帧下发,
  每帧一点, 执行时域 `1 / tracking_rate_hz`.

约束与前馈参数见 `config/kinematics.yaml`; 首次命令前平滑状态从实测反馈对齐, 复位时对齐到位零.

位姿目标 `frame_id` 为空时按 `C_Link` 解释, 非空时必须严格等于 `root_link`; 节点不执行
TF 变换, 由上游 Teleop 输出正确坐标系.

## 启动和构建

```bash
ros2 launch bw_kinematics kinematics.launch.py

ros2 launch bw_kinematics kinematics.launch.py \
  ik_model_path:=/path/to/standard_ik.urdf \
  parameters_file:=/path/to/kinematics.yaml
```

默认 `BW_KINEMATICS_ALGORITHM_MODE=PREBUILT`, 导入 `algorithm/lib/` 下
`libbw_kinematics_algorithm.so` (FK/Jacobian/联合 IK) 与 `libbw_kinematics_smoother.so`
(Ruckig 在线轨迹平滑). 仓库不携带算法源码, 两个库都是预编译交付;

```bash
colcon build --packages-select bw_kinematics --symlink-install
```

算法接入 (预编译 `.so` 与自定义算法包) 见 `algorithm/README.md`. SOURCE 模式需先在
`algorithm/source/` 放入实现源码与 `CMakeLists.txt`:

```bash
colcon build --packages-select bw_kinematics --symlink-install \
  --cmake-args -DBW_KINEMATICS_ALGORITHM_MODE=SOURCE
```

参数及默认值见 `config/kinematics.yaml`; root, tip 与锁定关节只能使用固定 Standard 契约;
所有 topic 在节点构造时校验, 非法参数直接拒绝启动.
