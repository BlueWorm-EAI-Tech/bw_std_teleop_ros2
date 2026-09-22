# 预编译实现库

```text
libbw_kinematics_algorithm.so   # 双臂联合 IK
libbw_kinematics_smoother.so    # 在线轨迹平滑
```

本目录只含预编译库, 不含算法源码. 公共接口头文件统一维护在
`include/bw_kinematics/algorithm/`; 替换流程与核对命令见 `../README.md`, 自定义源码接入
约定见 `../source/README.md`.

## 兼容性要求

- Linux x86-64 ELF `SHARED` 库, SONAME 与文件名一致.
- 导出 `include/bw_kinematics/algorithm/` 声明的完整 C++ API, 不导出内部实现符号.
- 与 `bw_kinematics` 同编译器, 同 C++ 标准与 C++ ABI.
- `libbw_kinematics_algorithm.so` 必须与目标系统的 Pinocchio, CasADi, urdfdom 及
  `package.xml` 声明的其它动态依赖兼容; CasADi 由 robotpkg 提供 (`/opt/openrobots`, 3.7,
  `_GLIBCXX_USE_CXX11_ABI=1`).
- `libbw_kinematics_smoother.so` 静态链入 Ruckig 0.15.5, `DT_NEEDED` 只有系统 C/C++
  运行库, 不依赖目标机 `libruckig.so`, 也不会被 `/usr/local/lib` 与 `/opt/ros` 中的同名库
  抢占; 替换时必须保持自包含, 否则需同步声明并固定 Ruckig 运行时版本.

Provider 把两个库安装到 `bw_kinematics` 的 `lib/`; Manager, Component 与节点通过安装
RPATH 从同一目录查找.
