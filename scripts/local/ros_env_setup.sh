#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
SYSTEM_TOOLS_FILE="${SCRIPT_DIR}/dependencies/system_tools.txt"
ROS_DEV_TOOLS_FILE="${SCRIPT_DIR}/dependencies/ros_dev_tools.txt"
ROS_ROOT="${BW_ROS_ENV_SETUP_ROS_ROOT:-/opt/ros}"
ROBOTPKG_PREFIX="${BW_ROS_ENV_SETUP_ROBOTPKG_PREFIX:-/opt/openrobots}"
INSTALL_PREFIX="${BW_ROS_ENV_SETUP_INSTALL_PREFIX:-${ROBOTPKG_PREFIX}}"
ROBOTPKG_DEB_BASE="${BW_ROS_ENV_SETUP_ROBOTPKG_DEB_BASE:-http://robotpkg.openrobots.org/packages/debian/pub}"
ROBOTPKG_KEY_URL="${BW_ROS_ENV_SETUP_ROBOTPKG_KEY_URL:-http://robotpkg.openrobots.org/packages/debian/robotpkg.key}"
ROBOTPKG_KEYRING_FILE="${BW_ROS_ENV_SETUP_ROBOTPKG_KEYRING_FILE:-/etc/apt/keyrings/robotpkg.asc}"
ROBOTPKG_SOURCES_FILE="${BW_ROS_ENV_SETUP_ROBOTPKG_SOURCES_FILE:-/etc/apt/sources.list.d/robotpkg.list}"
CASADI_PACKAGE="robotpkg-casadi"
ZSHRC_FILE="${BW_ROS_ENV_SETUP_ZSHRC:-${HOME}/.zshrc}"
ROSDEP_SOURCES_FILE="${BW_ROS_ENV_SETUP_ROSDEP_SOURCES_FILE:-/etc/ros/rosdep/sources.list.d/20-default.list}"
FISHROS_URL="${BW_ROS_ENV_SETUP_FISHROS_URL:-https://fishros.com/install}"
DEFAULT_ROSDISTRO_INDEX_URL="https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml"
ROSDISTRO_INDEX_URL_VALUE=""
CASADI_ROSDEP_KEY="casadi"
GAZEBO_ROS_ROSDEP_KEY="gazebo_ros"
GAZEBO_ROS_PACKAGE=""
ROSDEP_SKIP_KEYS=()
ROS_TARGET_DISTRO=""
UBUNTU_VERSION=""
UBUNTU_CODENAME=""
BUILD_BASE=""
INSTALL_BASE=""
LOG_BASE=""
SYSTEM_PACKAGES=()
ROS_PACKAGES=()

usage() {
  cat <<'EOF'
用法：
  ./scripts/local/ros_env_setup.sh --check
  ./scripts/local/ros_env_setup.sh --apply

选项：
  --check  只读检查 Ubuntu、ROS 2、ros2_control、Pinocchio、CasADi(robotpkg)、Gazebo Classic 与 submodule 依赖
  --apply  确认后安装依赖(含 robotpkg 提供的 CasADi 3.7)、配置 Zsh，并执行 colcon build
  --help   显示帮助
EOF
}

die() {
  printf '错误：%s\n' "$*" >&2
  return 1
}

detect_target_distro() {
  local os_release_file="${BW_ROS_ENV_SETUP_OS_RELEASE_FILE:-/etc/os-release}"
  local os_id=""
  local version_id=""
  local detected_distro=""

  [[ -r "${os_release_file}" ]] || die "无法读取系统版本：${os_release_file}"
  os_id="$(. "${os_release_file}"; printf '%s' "${ID:-}")"
  version_id="$(. "${os_release_file}"; printf '%s' "${VERSION_ID:-}")"
  [[ "${os_id}" == "ubuntu" ]] || die "不支持的系统：${os_id:-unknown}"

  case "${version_id}" in
    "22.04") detected_distro="humble"; UBUNTU_CODENAME="jammy" ;;
    "24.04") detected_distro="jazzy"; UBUNTU_CODENAME="noble" ;;
    *) die "仅支持 Ubuntu 22.04/Humble 或 Ubuntu 24.04/Jazzy，当前为 ${version_id:-unknown}" ;;
  esac

  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${detected_distro}" ]]; then
    die "ROS_DISTRO=${ROS_DISTRO} 与 Ubuntu ${version_id} 目标 ${detected_distro} 不匹配"
  fi
  ROS_TARGET_DISTRO="${ROS_DISTRO:-${detected_distro}}"
  UBUNTU_VERSION="${version_id}"
  BUILD_BASE="${BW_ROS_ENV_SETUP_BUILD_BASE:-${REPO_ROOT}/build/${ROS_TARGET_DISTRO}}"
  INSTALL_BASE="${BW_ROS_ENV_SETUP_INSTALL_BASE:-${REPO_ROOT}/install/${ROS_TARGET_DISTRO}}"
  LOG_BASE="${BW_ROS_ENV_SETUP_LOG_BASE:-${REPO_ROOT}/log/${ROS_TARGET_DISTRO}}"
  export ROS_DISTRO="${ROS_TARGET_DISTRO}"
}

read_package_file() {
  local package_file="$1"
  local target_name="$2"
  local line=""
  local expanded=""
  local -n target_packages="${target_name}"

  [[ -r "${package_file}" ]] || die "无法读取依赖清单：${package_file}"
  target_packages=()
  while IFS= read -r line || [[ -n "${line}" ]]; do
    [[ -z "${line//[[:space:]]/}" ]] && continue
    [[ "${line}" =~ ^[[:space:]]*# ]] && continue
    expanded="${line//'${ROS_DISTRO}'/${ROS_TARGET_DISTRO}}"
    [[ "${expanded}" =~ ^[a-z0-9][a-z0-9+._:-]*$ ]] ||
      die "依赖清单包含非法条目：${line}"
    target_packages+=("${expanded}")
  done < "${package_file}"
}

load_dependency_packages() {
  read_package_file "${SYSTEM_TOOLS_FILE}" SYSTEM_PACKAGES
  read_package_file "${ROS_DEV_TOOLS_FILE}" ROS_PACKAGES

  # casadi 无上游 rosdep 规则，固定跳过并由 robotpkg 提供。
  ROSDEP_SKIP_KEYS=("${CASADI_ROSDEP_KEY}")
  # Gazebo Classic 只发行到 Humble；Jazzy 无 gazebo_ros 规则，只能显式跳过。
  if [[ "${ROS_TARGET_DISTRO}" == "humble" ]]; then
    GAZEBO_ROS_PACKAGE="ros-${ROS_TARGET_DISTRO}-gazebo-ros"
    ROS_PACKAGES+=("${GAZEBO_ROS_PACKAGE}")
  else
    GAZEBO_ROS_PACKAGE=""
    ROSDEP_SKIP_KEYS+=("${GAZEBO_ROS_ROSDEP_KEY}")
  fi
}

# 解析并校验 rosdep 索引 URL，默认使用清华镜像，避免访问 raw.githubusercontent.com 超时。
resolve_rosdistro_index_url() {
  local configured_url="${BW_ROS_ENV_SETUP_ROSDISTRO_INDEX_URL:-${DEFAULT_ROSDISTRO_INDEX_URL}}"

  if [[ ! "${configured_url}" =~ ^https://[A-Za-z0-9]([A-Za-z0-9.-]*[A-Za-z0-9])?(:[0-9]+)?/[^[:space:]]+$ ]]; then
    die "rosdistro 索引 URL 非法，必须为包含主机和路径的完整 HTTPS URL"
  fi
  ROSDISTRO_INDEX_URL_VALUE="${configured_url}"
}

ros_is_ready() {
  [[ -f "${ROS_ROOT}/${ROS_TARGET_DISTRO}/setup.bash" ]] &&
    [[ -f "${ROS_ROOT}/${ROS_TARGET_DISTRO}/setup.zsh" ]]
}

source_ros_environment() {
  local setup_file="${ROS_ROOT}/${ROS_TARGET_DISTRO}/setup.bash"
  [[ -f "${setup_file}" ]] || die "ROS 环境不存在：${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

report_check() {
  local label="$1"
  shift
  if "$@"; then
    printf '[OK] %s\n' "${label}"
    return 0
  fi
  printf '[缺失] %s\n' "${label}"
  return 1
}

check_build_tools() {
  command -v cmake >/dev/null 2>&1 &&
    command -v colcon >/dev/null 2>&1 &&
    command -v rosdep >/dev/null 2>&1 &&
    command -v tmux >/dev/null 2>&1 &&
    command -v zsh >/dev/null 2>&1
}

# submodule 未拉取('-' 前缀)或提交不匹配('+' 前缀)时视为未就绪; 无 .gitmodules 视为就绪。
submodules_are_ready() {
  local status_line=""

  [[ -f "${REPO_ROOT}/.gitmodules" ]] || return 0
  while IFS= read -r status_line; do
    case "${status_line}" in
      "-"* | "+"*) return 1 ;;
    esac
  done < <(git -C "${REPO_ROOT}" submodule status --recursive 2>/dev/null)
  return 0
}

sync_submodules() {
  [[ -f "${REPO_ROOT}/.gitmodules" ]] || return 0
  if submodules_are_ready; then
    printf '[跳过] submodule 已就绪\n'
    return 0
  fi

  printf '[拉取] git submodule update --init --recursive\n'
  git -C "${REPO_ROOT}" submodule update --init --recursive
  submodules_are_ready ||
    die "submodule 未就绪：检查 ${REPO_ROOT}/.gitmodules 的 URL 与网络访问"
}

check_ros_packages() {
  local package=""
  for package in \
    controller_manager hardware_interface joint_state_broadcaster \
    joint_trajectory_controller pinocchio position_controllers xacro
  do
    ros2 pkg prefix "${package}" >/dev/null 2>&1 || return 1
  done
}

casadi_is_ready() {
  [[ -f "${INSTALL_PREFIX}/include/casadi/casadi.hpp" ]] &&
    [[ -f "${INSTALL_PREFIX}/lib/cmake/casadi/casadi-config.cmake" ]] &&
    [[ -f "${INSTALL_PREFIX}/lib/libcasadi.so.3.7" ]] &&
    [[ -f "${INSTALL_PREFIX}/lib/libcasadi_nlpsol_sqpmethod.so.3.7" ]] &&
    [[ -f "${INSTALL_PREFIX}/lib/libcasadi_conic_qrqp.so.3.7" ]] &&
    casadi_uses_new_abi
}

# 预编译算法库按 _GLIBCXX_USE_CXX11_ABI=1 链接, 必须拒绝 old ABI 的 CasADi。
casadi_uses_new_abi() {
  local symbols=""

  command -v nm >/dev/null 2>&1 || return 1
  symbols="$(nm -D --defined-only "${INSTALL_PREFIX}/lib/libcasadi.so.3.7" 2>/dev/null || true)"
  [[ "${symbols}" == *NSt7__cxx1112basic_string* ]]
}

# gazebo_ros 只由 upstream bw_std_description 的显示入口使用，仅 Humble 有发行版。
check_gazebo_ros() {
  [[ -n "${GAZEBO_ROS_PACKAGE}" ]] || return 0
  ros2 pkg prefix gazebo_ros >/dev/null 2>&1
}

check_rosdep_dependencies() {
  export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL_VALUE}"
  rosdep check --from-paths "${REPO_ROOT}/src" --ignore-src \
    --rosdistro "${ROS_TARGET_DISTRO}" \
    --skip-keys "${ROSDEP_SKIP_KEYS[*]}" >/dev/null 2>&1
}

check_environment() {
  local missing=0

  printf '[OK] Ubuntu %s -> ROS 2 %s\n' "${UBUNTU_VERSION}" "${ROS_TARGET_DISTRO}"
  report_check "ROS 2 ${ROS_TARGET_DISTRO}" ros_is_ready || missing=1
  report_check "构建与托管工具（cmake/colcon/rosdep/tmux/zsh）" check_build_tools || missing=1
  report_check "工作区 submodule（含 src/bw_std_description）" submodules_are_ready || missing=1
  if ros_is_ready; then
    source_ros_environment
    report_check "ros2_control、ROS 2 Controllers、Pinocchio 与 xacro" check_ros_packages || missing=1
    if [[ -n "${GAZEBO_ROS_PACKAGE}" ]]; then
      report_check "Gazebo Classic（${GAZEBO_ROS_PACKAGE}）" check_gazebo_ros || missing=1
    fi
    report_check "工作区 rosdep 依赖（casadi/gazebo_ros 由本脚本按发行版处理）" check_rosdep_dependencies || missing=1
  fi
  report_check "CasADi 3.7 核心库与 sqpmethod/qrqp 插件（robotpkg，${INSTALL_PREFIX}）" casadi_is_ready || missing=1

  if ((missing == 0)); then
    printf '[OK] 环境满足 Standard 遥操作工作区构建要求\n'
    return 0
  fi
  printf '[缺失] 请执行 --apply 完成环境配置\n'
  return 1
}

confirm_apply() {
  local confirmation=""
  cat <<EOF
即将执行高风险系统操作：
  - 使用 sudo 更新 APT 并安装系统/ROS 依赖
  - ROS 缺失时下载并交互执行 FishROS 安装脚本
  - 初始化或更新 rosdep；casadi 无上游 rosdep 规则，由脚本显式 skip 并由 robotpkg 提供
  - Gazebo Classic 仅 Humble 提供发行版：Humble 安装 ${GAZEBO_ROS_PACKAGE}，Jazzy 无对应包，rosdep 跳过 gazebo_ros
  - 构建前执行 git submodule update --init --recursive 拉取 src/bw_std_description（默认 https 访问 GitHub）
  - 写入 robotpkg APT 源并安装 ${CASADI_PACKAGE}(CasADi 3.7 二进制)到 ${ROBOTPKG_PREFIX}
  - 更新 ${ZSHRC_FILE} 中带边界标记的项目环境区块（CASADIPATH/LD_LIBRARY_PATH/CMAKE_PREFIX_PATH）
  - 使用发行版隔离目录执行 colcon build --symlink-install
    build:   ${BUILD_BASE}
    install: ${INSTALL_BASE}
    log:     ${LOG_BASE}

FishROS 是未固定版本和校验和的第三方远程安装器，请确认网络与脚本来源可信。
robotpkg 签名密钥只能通过明文 http 获取，请在可信网络下执行。
确认继续请输入 APPLY：
EOF
  IFS= read -r confirmation || true
  [[ "${confirmation}" == "APPLY" ]] || die "已取消环境配置"
}

install_system_dependencies() {
  sudo apt-get update
  sudo apt-get install -y "${SYSTEM_PACKAGES[@]}"
}

install_ros_if_needed() {
  local installer=""
  if ros_is_ready; then
    printf '[跳过] ROS 2 %s 已安装\n' "${ROS_TARGET_DISTRO}"
    return 0
  fi

  printf '[注意] 即将启动 FishROS 交互安装，请选择 ROS 2 %s。\n' "${ROS_TARGET_DISTRO}"
  installer="$(mktemp "${TMPDIR:-/tmp}/bw-fishros.XXXXXX")"
  curl --fail --location --proto '=https' --tlsv1.2 "${FISHROS_URL}" --output "${installer}"
  /bin/bash "${installer}"
  rm -f "${installer}"
  ros_is_ready || die "安装后仍未找到 ROS 2 ${ROS_TARGET_DISTRO}"
}

install_ros_dependencies() {
  sudo apt-get update
  sudo apt-get install -y "${ROS_PACKAGES[@]}"
}

install_rosdep_dependencies() {
  if [[ ! -f "${ROSDEP_SOURCES_FILE}" ]]; then
    sudo rosdep init
  fi
  export ROSDISTRO_INDEX_URL="${ROSDISTRO_INDEX_URL_VALUE}"
  rosdep update
  rosdep install --from-paths "${REPO_ROOT}/src" --ignore-src -r -y \
    --rosdistro "${ROS_TARGET_DISTRO}" \
    --skip-keys "${ROSDEP_SKIP_KEYS[*]}"
}

robotpkg_source_is_ready() {
  [[ -r "${ROBOTPKG_KEYRING_FILE}" && -r "${ROBOTPKG_SOURCES_FILE}" ]] &&
    grep -Fq "signed-by=${ROBOTPKG_KEYRING_FILE}" "${ROBOTPKG_SOURCES_FILE}" &&
    grep -Fq " ${UBUNTU_CODENAME} robotpkg" "${ROBOTPKG_SOURCES_FILE}"
}

configure_robotpkg_source() {
  robotpkg_source_is_ready && return 0
  sudo install -d -m 0755 "$(dirname -- "${ROBOTPKG_KEYRING_FILE}")" \
    "$(dirname -- "${ROBOTPKG_SOURCES_FILE}")"
  curl --fail --location "${ROBOTPKG_KEY_URL}" |
    sudo tee "${ROBOTPKG_KEYRING_FILE}" >/dev/null
  printf 'deb [arch=amd64 signed-by=%s] %s %s robotpkg\n' \
    "${ROBOTPKG_KEYRING_FILE}" "${ROBOTPKG_DEB_BASE}" "${UBUNTU_CODENAME}" |
    sudo tee "${ROBOTPKG_SOURCES_FILE}" >/dev/null
}

install_casadi() {
  if casadi_is_ready; then
    printf '[跳过] CasADi 3.7 已就绪：%s\n' "${INSTALL_PREFIX}"
    return 0
  fi
  if [[ "${INSTALL_PREFIX}" != "${ROBOTPKG_PREFIX}" ]]; then
    die "CasADi 3.7 只由 robotpkg 提供，固定前缀 ${ROBOTPKG_PREFIX}，当前 ${INSTALL_PREFIX} 无 3.7 库"
  fi

  configure_robotpkg_source
  sudo apt-get update
  sudo apt-get install -y "${CASADI_PACKAGE}"
  casadi_is_ready ||
    die "${CASADI_PACKAGE} 安装后未在 ${ROBOTPKG_PREFIX} 找到 CasADi 3.7 (含 sqpmethod/qrqp 插件与 new ABI)"
  printf '[OK] CasADi 3.7 (%s) 已安装到 %s\n' "${CASADI_PACKAGE}" "${ROBOTPKG_PREFIX}"
}

export_casadi_environment() {
  export CASADIPATH="${INSTALL_PREFIX}/lib"
  export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export CMAKE_PREFIX_PATH="${INSTALL_PREFIX}:${CMAKE_PREFIX_PATH:-}"
}

configure_zsh_environment() {
  local start_marker="# >>> BW Standard ROS environment >>>"
  local end_marker="# <<< BW Standard ROS environment <<<"
  local temporary_file=""
  local start_count=0
  local end_count=0
  local escaped_ros_setup=""
  local escaped_workspace_setup=""
  local escaped_install_prefix=""
  local escaped_install_lib=""
  local escaped_rosdistro_index_url=""

  [[ -n "${ROSDISTRO_INDEX_URL_VALUE}" ]] || resolve_rosdistro_index_url
  printf -v escaped_rosdistro_index_url '%q' "${ROSDISTRO_INDEX_URL_VALUE}"

  mkdir -p "$(dirname -- "${ZSHRC_FILE}")"
  touch "${ZSHRC_FILE}"
  start_count="$(grep -Fxc -- "${start_marker}" "${ZSHRC_FILE}" || true)"
  end_count="$(grep -Fxc -- "${end_marker}" "${ZSHRC_FILE}" || true)"
  if [[ "${start_count}" != "${end_count}" ]] || ((start_count > 1)); then
    die "${ZSHRC_FILE} 中的 BW Standard 环境标记不完整或重复"
  fi

  printf -v escaped_ros_setup '%q' "${ROS_ROOT}/${ROS_TARGET_DISTRO}/setup.zsh"
  printf -v escaped_workspace_setup '%q' "${INSTALL_BASE}/setup.zsh"
  printf -v escaped_install_prefix '%q' "${INSTALL_PREFIX}"
  printf -v escaped_install_lib '%q' "${INSTALL_PREFIX}/lib"
  temporary_file="$(mktemp "${ZSHRC_FILE}.tmp.XXXXXX")"
  awk -v start="${start_marker}" -v end="${end_marker}" '
    $0 == start { skipping = 1; next }
    $0 == end { skipping = 0; next }
    !skipping { print }
  ' "${ZSHRC_FILE}" > "${temporary_file}"

  cat >> "${temporary_file}" <<EOF
${start_marker}
export ROS_DISTRO=${ROS_TARGET_DISTRO}
export BW_ROS_ENV_SETUP_INSTALL_PREFIX=${escaped_install_prefix}
export ROSDISTRO_INDEX_URL=${escaped_rosdistro_index_url}
source ${escaped_ros_setup}
export CASADIPATH=${escaped_install_lib}
export LD_LIBRARY_PATH=${escaped_install_lib}:\${LD_LIBRARY_PATH:-}
export CMAKE_PREFIX_PATH=${escaped_install_prefix}:\${CMAKE_PREFIX_PATH:-}
if [[ -f ${escaped_workspace_setup} ]]; then
  source ${escaped_workspace_setup}
fi
${end_marker}
EOF
  mv "${temporary_file}" "${ZSHRC_FILE}"
  printf '[OK] 已更新 %s\n' "${ZSHRC_FILE}"
}

build_workspace() {
  (
    cd "${REPO_ROOT}"
    colcon --log-base "${LOG_BASE}" build \
      --build-base "${BUILD_BASE}" \
      --install-base "${INSTALL_BASE}" \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  )
}

apply_environment() {
  sudo -v
  install_system_dependencies
  install_ros_if_needed
  source_ros_environment
  install_ros_dependencies
  install_rosdep_dependencies
  install_casadi
  export_casadi_environment
  configure_zsh_environment
  sync_submodules
  check_environment
  build_workspace
  printf '[OK] 环境配置和工作区构建完成\n'
}

main() {
  local mode="${1:---help}"
  case "${mode}" in
    --check)
      detect_target_distro
      resolve_rosdistro_index_url
      load_dependency_packages
      check_environment
      ;;
    --apply)
      detect_target_distro
      resolve_rosdistro_index_url
      load_dependency_packages
      confirm_apply
      apply_environment
      ;;
    --help)
      usage
      ;;
    *)
      usage >&2
      die "不支持的参数：${mode}"
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
