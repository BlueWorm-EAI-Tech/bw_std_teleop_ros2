# 算法源码 Provider 约定

把完整实现源码与 `CMakeLists.txt` 放在本目录, 用
`-DBW_KINEMATICS_ALGORITHM_MODE=SOURCE` 构建 `bw_kinematics`; 接入流程见
`../README.md`. 本目录只保存约定, 当前不包含实现源码.

`CMakeLists.txt` 必须创建 `SHARED` library target, 名称二选一:

```cmake
add_library(bw_kinematics_algorithm SHARED ...)

# 或实现 target 加命名空间别名 (需 CMake >= 3.18).
add_library(bw_kinematics_algorithm_impl SHARED ...)
add_library(
  bw_kinematics_algorithm::bw_kinematics_algorithm
  ALIAS bw_kinematics_algorithm_impl)
```

约束:

- 实现 `include/bw_kinematics/algorithm/` 声明的公共 API.
- 使用与 `bw_kinematics` 相同的 C++ 标准, 编译器和 C++ ABI.
- 自行查找并链接 Pinocchio, CasADi 及其它实现依赖.
- 只编译共享库, 不创建静态库或可执行文件.
- 不要自行 `install()`: Provider 自动把本包公共 include 加入 target, 并将库安装到本包
  `lib/`.
