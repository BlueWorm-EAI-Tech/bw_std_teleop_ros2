# bw_std_bringup

## 概述

`bw_std_bringup` 是 Standard 机器人 ros2_control 与 VR 遥操作系统的唯一部署入口。包内只保存 Launch、controller manager 配置和启动顺序，不实现控制逻辑。

## 启动入口

```bash
# 真机默认不上电、不启动遥操作，所有运动控制器保持 inactive
ros2 launch bw_std_bringup standard.launch.py

# mock hardware 自动激活双臂、夹爪、升降和底盘控制器
ros2 launch bw_std_bringup standard_mock.launch.py
```

## 控制器

- `dual_arm_controller`：Standard 双臂 14 个位置关节。
- `gripper_controller`：左右 `Degree8` 主动指节；`Degree9` 由 URDF mimic。
- `lift_controller`：`C_joint`。
- `base_controller`：机体 `vx/vy/wz`，自带独立命令超时。

头部控制器有意缺失。嵌入式头部契约完成前，Standard 的三个头部关节不导出 command interface，也不得由 Bringup 激活。

## 关键参数

- `serial_port`：V3 串口，默认 `/dev/ttyACM0`。
- `baud_rate`：默认 `2000000`。
- `feedback_timeout_ms`：反馈超时，默认 `100 ms`。
- `power_on_on_activate`：是否在硬件激活时软件上电，默认 `false`。
- `arm_mapping_calibrated`：是否已完成 Standard 双臂逐轴标定，默认 `false`；未标定
  时所有正常命令帧保持掉电。
- `activate_motion_controllers`：是否自动激活运动控制器，默认 `false`。
- 升降没有零偏或参考点 Launch 参数；控制包仅进行 `mm <-> m` 单位换算，
  `0 mm <-> C_joint=0 m`。
- `left_arm_motor_indices` / `right_arm_motor_indices`：按 Degree1 到 Degree7 排列的
  V3 Motor 索引，必须是 `0..6` 的完整置换。
- `left_arm_direction` / `right_arm_direction`：按 Degree1 到 Degree7 排列，每项
  只能是 `-1` 或 `1`。
- `left_arm_raw_zero_rad` / `right_arm_raw_zero_rad`：Degree1 到 Degree7 的 7 项
  原始零位数组。发布默认全零，真机必须显式标定。
- `arm_max_velocity_rad_s`、`pelvis_max_velocity_mm_s`、`gripper_max_velocity_normalized_s`：低速 commissioning 上限。

## 真机安全顺序

1. `start_teleop:=false`、`power_on_on_activate:=false`、
   `arm_mapping_calibrated:=false`、`activate_motion_controllers:=false` 启动只读反馈。
2. 逐轴确认 Motor 索引、方向和 URDF 零位，再核对 17 个主动关节反馈与模型限位。
3. 写入完整 Standard 标定并显式设置 `arm_mapping_calibrated:=true`、
   `power_on_on_activate:=true`，只验证 measured-hold，不激活运动控制器。
4. 按双臂、夹爪、升降、底盘顺序逐项低速激活。
5. 最后启动 `bw_teleop` 完成 VR 链路验证。

只读复验必须显式保持所有安全门关闭；手臂零偏数组使用经过逐轴确认的 Standard
标定结果，不能从任意单帧自动生成：

```bash
ros2 launch bw_std_bringup standard.launch.py \
  power_on_on_activate:=false arm_mapping_calibrated:=false \
  activate_motion_controllers:=false \
  start_teleop:=false
```

升降独立按原始毫米值直接换算为 `C_joint` 米制值，不使用启动姿态生成零偏。
不得修改 URDF origin、axis、TF 或放宽限位来补偿。

故障恢复必须重新激活生命周期。不得自动上电，不得把当前有限右臂验证描述为整机或大规模测试通过。

## 构建

```bash
colcon build --packages-up-to bw_std_bringup --symlink-install
colcon test --packages-select bw_std_bringup
colcon test-result --verbose
```
