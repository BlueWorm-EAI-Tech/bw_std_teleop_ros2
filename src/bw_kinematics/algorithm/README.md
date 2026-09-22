# 算法实现接入

`bw_kinematics` 的 FK, Jacobian, 联合 IK 与轨迹平滑由两个独立共享库提供:

| 实现库 | 公共头文件 | 形态 |
| --- | --- | --- |
| `lib/libbw_kinematics_algorithm.so` | `standard_kinematics_types.hpp`, `standard_kinematics_model.hpp`, `standard_fk_solver.hpp`, `standard_ik_solver.hpp` | 默认 PREBUILT, 可选 SOURCE |
| `lib/libbw_kinematics_smoother.so` | `standard_trajectory_smoother.hpp` | 仅 PREBUILT |

仓库只交付 `algorithm/lib/` 中的预编译 `.so`, 不携带算法源码. 两种接入方式:

- 使用 `.so` (默认): 覆盖 `algorithm/lib/` 中的库即可更换实现.
- 自定义算法包: 把自有源码与 `CMakeLists.txt` 放入 `algorithm/source/`, 以 SOURCE 模式
  构建; 该开关仅 `algorithm` 提供.

模型契约与接口语义见 `../README.md`, 细节要求见 `lib/README.md` 和 `source/README.md`.

## 使用 .so

```bash
# 覆盖 algorithm/lib/libbw_kinematics_algorithm.so 或 libbw_kinematics_smoother.so 后重新构建.
colcon --log-base log/$ROS_DISTRO build \
  --build-base build/$ROS_DISTRO \
  --install-base install/$ROS_DISTRO \
  --packages-select bw_kinematics \
  --symlink-install
```

约束:

- SONAME 与文件名一致, 不带版本后缀; 与 `bw_kinematics` 同编译器, C++17, 同 C++ ABI.
- `algorithm` 库: 目标机需能解析 Pinocchio, CasADi (`/opt/openrobots`, 3.7) 与 urdfdom.
- `smoother` 库: 静态链入 Ruckig, `DT_NEEDED` 仅系统 C/C++ 运行库.

核对:

```bash
source /opt/ros/$ROS_DISTRO/setup.zsh
source install/$ROS_DISTRO/setup.zsh
LIB=install/$ROS_DISTRO/bw_kinematics/lib

readelf -d $LIB/libbw_kinematics_algorithm.so | grep SONAME   # 与文件名一致
ldd $LIB/libbw_kinematics_algorithm.so | grep 'not found'     # 期望无输出
nm -DC --defined-only $LIB/libbw_kinematics_algorithm.so | grep 'bw_kinematics::Standard'
```

## 自定义算法包

### SOURCE 模式

```bash
# 把实现源码与 CMakeLists.txt 放入 algorithm/source/ 后构建.
colcon --log-base log/$ROS_DISTRO build \
  --build-base build/$ROS_DISTRO \
  --install-base install/$ROS_DISTRO \
  --packages-select bw_kinematics \
  --symlink-install \
  --cmake-args -DBW_KINEMATICS_ALGORITHM_MODE=SOURCE
```

- target 名, 共享库类型与别名要求见 `source/README.md`; 命名空间别名需 CMake >= 3.18.
- `smoother` 无 SOURCE 模式, 只能按 "使用 .so" 提供预编译库.

### 独立构建 .so

```cmake
set_target_properties(<impl> PROPERTIES OUTPUT_NAME bw_kinematics_algorithm)
# 不设 VERSION/SOVERSION: SONAME 必须无版本后缀.
```

编译时包含本包 `include/`, 产物覆盖 `algorithm/lib/` 后按 "使用 .so" 重建.
