# BW Standard 遥操作

## 包结构

```
src/
├── bw_std_description/  # URDF、mesh、mimic、限位与 ros2_control 契约
├── bw_std_control/      # 串口、SystemInterface、底盘与头部控制器
├── bw_teleop/           # 单节点 PICO UDP 接入与遥操作编排
├── bw_kinematics/       # 双臂运动学编排与 Algorithm Provider
└── bw_std_bringup/      # 控制器配置、真机和 mock 启动入口
```

## 运行时数据流

```text
PICO UDP
   -> bw_teleop
        |-- /target_left_pose, /target_right_pose -> bw_kinematics
        |                                             -> /dual_arm_controller/joint_trajectory
        |-- /gripper_controller/commands
        |-- /lift_controller/commands
        |-- /base_controller/reference
        |-- /Teleop/head_pose
        |-- /teleop/control_active -> base_controller -> safety/power
        '-- /kinematics/reset_arms -> 关节空间复位
controller_manager -> StandardSystemHardware -> 串口帧 (0x01/0x21)
```


## 环境与构建

支持 Ubuntu 22.04 / ROS 2 Humble, 兼容 Ubuntu 24.04 / Jazzy:

```bash
# 执行 sudo、修改 Zsh 配置并构建, 执行前必须输入 `APPLY` 二次确认.
./scripts/local/ros_env_setup.sh --apply

./scripts/local/ros_env_setup.sh --check  # 只读检查环境.
```

```bash
echo $ROS_DISTRO  # 确认 ROS2 版本
source /opt/ros/$ROS_DISTRO/setup.zsh
rosdep install --from-paths src --ignore-src -r -y --skip-keys casadi
colcon --log-base log/$ROS_DISTRO build \
  --build-base build/$ROS_DISTRO \
  --install-base install/$ROS_DISTRO \
  --symlink-install
source install/$ROS_DISTRO/setup.zsh
```

## 启动与安全

真机放行顺序: 只读反馈核对 -> 逐轴标定写入 -> 单独批准软件上电 -> 低速逐项放行控制器 ->
最后启动 Teleop 与运动学. VR 断流会 `0.5 s` 内软件掉电; 故障后必须重新走生命周期.

### 进程托管 (scripts/local/start.sh)

```bash
./scripts/local/start.sh start           # real (默认): 上电 + 激活运动控制器 + 遥操作与运动学
./scripts/local/start.sh start readonly  # 只读: 不上电, 运动控制器 inactive, 不启动遥操作
./scripts/local/start.sh start mock      # Mock: 不接硬件, 自动激活全部控制器

./scripts/local/start.sh status        # session、遗留进程、串口占用与日志尾部
./scripts/local/start.sh attach        # 接入 session, Ctrl-b d 离开
./scripts/local/start.sh logs -f       # 跟随本次启动日志
./scripts/local/start.sh stop          # 先发 Ctrl-C 优雅收尾
./scripts/local/start.sh stop --force  # 优雅收尾超时或仍有残留进程时强杀
./scripts/local/start.sh restart       # 等价于 stop --force + start
./scripts/local/start.sh list          # 列出所有 tmux session
```

约束:

- 同一时刻只允许一个 session; 启动前检查 session、本工作区遗留进程与串口占用。
- 日志固定写入 `log/<ROS_DISTRO>/<session>.log`。

环境变量:

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `BW_START_SESSION` | `bw_std` | tmux session 名 |
| `BW_START_SERIAL_PORT` | `/dev/ttyACM0` | 串口设备 |
| `BW_START_ARGS` | 空 | 追加的 `ros2 launch` 参数 |
| `BW_START_STOP_TIMEOUT_SEC` | `20` | 优雅收尾超时秒数 |

### 运行 Mock 验证

```bash
./scripts/local/start.sh start mock
./scripts/local/start.sh logs -f
```

核对:

```bash
source /opt/ros/$ROS_DISTRO/setup.zsh
source install/$ROS_DISTRO/setup.zsh
ros2 control list_controllers          # 期望 6 个都是 active
ros2 topic hz /joint_states            # 期望约 500 Hz 的 17 轴反馈
ros2 topic echo /current_left_pose --once
```

### 运行真机验证

```bash
# 第一步: 只读反馈, 不上电、不激活运动控制器、不启动遥操作.
./scripts/local/start.sh start readonly
./scripts/local/start.sh logs -f
```

核对 17 轴名称、顺序、单位与方向 (另开终端):

```bash
source /opt/ros/$ROS_DISTRO/setup.zsh
source install/$ROS_DISTRO/setup.zsh
ros2 topic echo /joint_states --once
```

核对通过后:

```bash
# 第二步: 确认 /dev/ttyACM0 和 UDP 12345 无人占用后, 放行软件上电与遥操作.
./scripts/local/start.sh restart real
```

### 备选: 直接使用 ros2 launch

无 session 互斥与串口占用检查, 需自行确认没有同一条控制链在运行。

```bash
source /opt/ros/$ROS_DISTRO/setup.zsh
source install/$ROS_DISTRO/setup.zsh

# Mock
ros2 launch bw_std_bringup standard_mock.launch.py

# 真机只读
ros2 launch bw_std_bringup standard.launch.py

# 真机放行 (需现场标定已完成); 其余参数等于 launch 默认值.
ros2 launch bw_std_bringup standard.launch.py \
  power_on_on_activate:=true activate_motion_controllers:=true \
  arm_mapping_calibrated:=true start_teleop:=true \
  arm_startup_limit_tolerance_rad:=0.002 gripper_startup_limit_tolerance_m:=0.001
```
