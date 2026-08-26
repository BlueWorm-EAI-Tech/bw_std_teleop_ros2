#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
SYSTEM_TOOLS_FILE="${SCRIPT_DIR}/dependencies/system_tools.txt"
ROS_DEV_TOOLS_FILE="${SCRIPT_DIR}/dependencies/ros_dev_tools.txt"
ROS_ROOT="${BW_ROS_ENV_SETUP_ROS_ROOT:-/opt/ros}"
ZSHRC_FILE="${BW_ROS_ENV_SETUP_ZSHRC:-${HOME}/.zshrc}"
ROSDEP_SOURCES_FILE="${BW_ROS_ENV_SETUP_ROSDEP_SOURCES_FILE:-/etc/ros/rosdep/sources.list.d/20-default.list}"
FISHROS_URL="${BW_ROS_ENV_SETUP_FISHROS_URL:-https://fishros.com/install}"
ROS_TARGET_DISTRO=""
UBUNTU_VERSION=""
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
  --check  只读检查 Ubuntu、ROS 2、ros2_control 和项目依赖
  --apply  确认后安装依赖、配置 Zsh，并执行 colcon build
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
    "22.04") detected_distro="humble" ;;
    "24.04") detected_distro="jazzy" ;;
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
    command -v zsh >/dev/null 2>&1
}

check_ros_packages() {
  local package=""
  for package in \
    controller_manager hardware_interface joint_state_broadcaster \
    joint_trajectory_controller kdl_parser position_controllers xacro
  do
    ros2 pkg prefix "${package}" >/dev/null 2>&1 || return 1
  done
}

check_rosdep_dependencies() {
  rosdep check --from-paths "${REPO_ROOT}/src" --ignore-src \
    --rosdistro "${ROS_TARGET_DISTRO}" >/dev/null 2>&1
}

check_environment() {
  local missing=0

  printf '[OK] Ubuntu %s -> ROS 2 %s\n' "${UBUNTU_VERSION}" "${ROS_TARGET_DISTRO}"
  report_check "ROS 2 ${ROS_TARGET_DISTRO}" ros_is_ready || missing=1
  report_check "构建工具（cmake/colcon/rosdep/zsh）" check_build_tools || missing=1
  if ros_is_ready; then
    source_ros_environment
    report_check "ros2_control、ROS 2 Controllers、KDL 与 xacro" check_ros_packages || missing=1
    report_check "工作区 rosdep 依赖" check_rosdep_dependencies || missing=1
  fi

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
  - 初始化或更新 rosdep
  - 更新 ${ZSHRC_FILE} 中带边界标记的项目环境区块
  - 使用发行版隔离目录执行 colcon build --symlink-install
    build:   ${BUILD_BASE}
    install: ${INSTALL_BASE}
    log:     ${LOG_BASE}

FishROS 是未固定版本和校验和的第三方远程安装器，请确认网络与脚本来源可信。
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
  rosdep update --rosdistro "${ROS_TARGET_DISTRO}"
  rosdep install --from-paths "${REPO_ROOT}/src" --ignore-src -r -y \
    --rosdistro "${ROS_TARGET_DISTRO}"
}

configure_zsh_environment() {
  local start_marker="# >>> BW Standard ROS environment >>>"
  local end_marker="# <<< BW Standard ROS environment <<<"
  local temporary_file=""
  local start_count=0
  local end_count=0
  local escaped_ros_setup=""
  local escaped_workspace_setup=""

  mkdir -p "$(dirname -- "${ZSHRC_FILE}")"
  touch "${ZSHRC_FILE}"
  start_count="$(grep -Fxc -- "${start_marker}" "${ZSHRC_FILE}" || true)"
  end_count="$(grep -Fxc -- "${end_marker}" "${ZSHRC_FILE}" || true)"
  if [[ "${start_count}" != "${end_count}" ]] || ((start_count > 1)); then
    die "${ZSHRC_FILE} 中的 BW Standard 环境标记不完整或重复"
  fi

  printf -v escaped_ros_setup '%q' "${ROS_ROOT}/${ROS_TARGET_DISTRO}/setup.zsh"
  printf -v escaped_workspace_setup '%q' "${INSTALL_BASE}/setup.zsh"
  temporary_file="$(mktemp "${ZSHRC_FILE}.tmp.XXXXXX")"
  awk -v start="${start_marker}" -v end="${end_marker}" '
    $0 == start { skipping = 1; next }
    $0 == end { skipping = 0; next }
    !skipping { print }
  ' "${ZSHRC_FILE}" > "${temporary_file}"

  cat >> "${temporary_file}" <<EOF
${start_marker}
export ROS_DISTRO=${ROS_TARGET_DISTRO}
source ${escaped_ros_setup}
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
  configure_zsh_environment
  check_environment
  build_workspace
  printf '[OK] 环境配置和工作区构建完成\n'
}

main() {
  local mode="${1:---help}"
  case "${mode}" in
    --check)
      detect_target_distro
      load_dependency_packages
      check_environment
      ;;
    --apply)
      detect_target_distro
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
