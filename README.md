# BW Standard 遥操作

## 包结构

```
src/
├── bw_std_description/  # URDF、mesh、mimic、限位与 ros2_control 契约
├── bw_std_control/      # V3 串口、SystemInterface 与底盘控制器
├── bw_teleop/           # 单节点 PICO UDP 接入与遥操作编排
├── bw_kinematics/       # 双臂 KDL FK 与 DLS IK
└── bw_std_bringup/      # 控制器配置、真机和 mock 启动入口src
```

## 环境与构建



支持 Ubuntu 22.04 / ROS 2 Humble，兼容 Ubuntu 24.04 / Jazzy：


```bash
# 执行sudo、修改 Zsh 配置并构建，执行前必须输入 `APPLY` 二次确认。
./scripts/local/ros_env_setup.sh --apply

```

```bash
./scripts/local/ros_env_setup.sh --check # 只读检查环境。
```

```bash
echo $ROS_DISTRO  # 确认ROS2版本
source /opt/ros/$ROS_DISTRO/setup.zsh
rosdep install --from-paths src --ignore-src -r -y
colcon --log-base log/$ROS_DISTRO build \
  --build-base build/$ROS_DISTRO \
  --install-base install/$ROS_DISTRO \
  --symlink-install
source install/$ROS_DISTRO/setup.zsh
colcon test --build-base build/$ROS_DISTRO
colcon test-result --test-result-base build/$ROS_DISTRO --verbose
```


## 启动与安全

```bash
# 真机：默认不上电、不启动遥操作、运动控制器 inactive
ros2 launch bw_std_bringup standard.launch.py

# Mock：自动激活双臂、夹爪、升降和底盘，并启动遥操作/运动学
ros2 launch bw_std_bringup standard_mock.launch.py
```