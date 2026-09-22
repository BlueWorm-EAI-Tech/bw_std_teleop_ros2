#!/usr/bin/env bash
# Standard 遥操作进程托管: 用 tmux session 启动/停止 bw_std_bringup。
#
# 约定:
#   - 同一时刻只允许一个 session; 启动前检查 session、ros2_control_node 与串口占用, 拒绝双栈抢串口。
#   - 停止优先用 Ctrl-C 语义 (控制器 deactivate + hardware shutdown 掉电帧), 超时才强杀 session。
#   - pane 输出通过 tmux pipe-pane 落盘到 log/<distro>/<session>.log。

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd -P)"
SESSION="${BW_START_SESSION:-bw_std}"
ROS_ROOT="${BW_ROS_ENV_SETUP_ROS_ROOT:-/opt/ros}"
SERIAL_PORT="${BW_START_SERIAL_PORT:-/dev/ttyACM0}"
STOP_TIMEOUT_SEC="${BW_START_STOP_TIMEOUT_SEC:-20}"
ROS_TARGET_DISTRO=""
WORKSPACE_SETUP_FILE=""
LOG_DIR=""
LOG_FILE=""

usage() {
  cat <<'EOF'
用法: ./scripts/local/start.sh <命令> [参数]

命令:
  start [real|readonly|mock] [额外的 ros2 launch 参数...]  在 tmux session 中启动
  stop [--force]                                           优雅收尾并结束 session
  restart [real|readonly|mock] [...]                       等价于 stop + start
  attach                                                   接入 session (Ctrl-b d 离开)
  status                                                   查看 session、进程与串口占用
  logs [-f]                                                查看本次启动日志
  list                                                     列出所有 tmux session

模式 (默认 real):
  real      真机放行: 上电 + 激活运动控制器 + 启动遥操作与运动学 (需现场标定已完成)
  readonly  真机只读: 不上电, 运动控制器 inactive, 不启动遥操作 (核对 17 轴反馈用)
  mock      Mock: 不接硬件, 自动激活全部控制器

环境变量:
  BW_START_SESSION             tmux session 名 (默认 bw_std)
  BW_START_SERIAL_PORT         串口 (默认 /dev/ttyACM0)
  BW_START_ARGS                追加的 ros2 launch 参数
  BW_START_STOP_TIMEOUT_SEC    优雅收尾超时秒数 (默认 20)
EOF
}

die() {
  printf '错误：%s\n' "$*" >&2
  return 1
}

detect_ros_distro() {
  local setup_file=""
  local candidate=""

  if [[ -n "${ROS_DISTRO:-}" && -f "${ROS_ROOT}/${ROS_DISTRO}/setup.bash" ]]; then
    ROS_TARGET_DISTRO="${ROS_DISTRO}"
  else
    for candidate in "${ROS_ROOT}"/*/setup.bash; do
      [[ -f "${candidate}" ]] || continue
      ROS_TARGET_DISTRO="$(basename -- "$(dirname -- "${candidate}")")"
      break
    done
  fi
  [[ -n "${ROS_TARGET_DISTRO}" ]] || die "未找到 ROS 2 安装：${ROS_ROOT}/<distro>/setup.bash"

  setup_file="${REPO_ROOT}/install/${ROS_TARGET_DISTRO}/setup.bash"
  [[ -f "${setup_file}" ]] || setup_file="${REPO_ROOT}/install/setup.bash"
  [[ -f "${setup_file}" ]] ||
    die "工作区未构建：先执行 ./scripts/local/ros_env_setup.sh --apply 或 colcon build"
  WORKSPACE_SETUP_FILE="${setup_file}"
  LOG_DIR="${REPO_ROOT}/log/${ROS_TARGET_DISTRO}"
  LOG_FILE="${LOG_DIR}/${SESSION}.log"
}

require_tmux() {
  command -v tmux >/dev/null 2>&1 || die "未安装 tmux：sudo apt-get install -y tmux"
}

session_exists() {
  tmux has-session -t "${SESSION}" 2>/dev/null
}

port_holders() {
  fuser "${SERIAL_PORT}" 2>/dev/null | tr -s ' ' | sed 's/^ //; s/ $//' || true
}

control_node_pids() {
  pgrep -f "ros2_control_node" 2>/dev/null || true
}

# 只认本工作区的遗留: cmdline 引用本工作区 install, 或容器 cwd 就是本工作区。
leftover_pids() {
  local candidates=""
  local pid=""

  candidates="$(pgrep -f "${REPO_ROOT}/install" 2>/dev/null || true)"
  while read -r pid; do
    [[ -n "${pid}" ]] || continue
    if [[ "$(readlink -f "/proc/${pid}/cwd" 2>/dev/null)" == "${REPO_ROOT}" ]]; then
      candidates="${candidates}${candidates:+$'\n'}${pid}"
    fi
  done < <(pgrep -f "__node:=kinematics_container" 2>/dev/null || true)

  printf '%s\n' "${candidates}" | sed '/^$/d' | sort -u
}

# pgrep 输出换行 -> ps -p 需要的逗号列表。
pid_csv() {
  tr '\n' ',' <<<"$1" | sed 's/,$//'
}

launch_spec() {
  case "$1" in
    real)
      printf '%s' "standard.launch.py use_mock_hardware:=false \
power_on_on_activate:=true arm_mapping_calibrated:=true \
arm_startup_limit_tolerance_rad:=0.002 gripper_startup_limit_tolerance_m:=0.001 \
arm_max_velocity_rad_s:=12.0 activate_motion_controllers:=true start_teleop:=true \
serial_port:=${SERIAL_PORT} baud_rate:=2000000 feedback_timeout_ms:=100"
      ;;
    readonly)
      printf '%s' "standard.launch.py use_mock_hardware:=false \
power_on_on_activate:=false arm_mapping_calibrated:=false \
activate_motion_controllers:=false start_teleop:=false \
serial_port:=${SERIAL_PORT} baud_rate:=2000000 feedback_timeout_ms:=100"
      ;;
    mock)
      printf '%s' "standard_mock.launch.py"
      ;;
    *)
      die "未知模式：$1 (可选 real|readonly|mock)"
      ;;
  esac
}

preflight_start() {
  local mode="$1"
  local pids=""
  local holders=""
  local foreign=""

  session_exists && die "session ${SESSION} 已在运行：./scripts/local/start.sh attach (或 stop)"
  pids="$(leftover_pids)"
  [[ -z "${pids}" ]] || die "本工作区仍有遗留进程 (pid $(pid_csv "${pids}") )；先执行 ./scripts/local/start.sh stop --force"
  if [[ "${mode}" != "mock" ]]; then
    holders="$(port_holders)"
    [[ -z "${holders}" ]] ||
      die "${SERIAL_PORT} 已被占用 (pid ${holders// /, })；禁止双栈抢串口"
  fi
  foreign="$(control_node_pids)"
  if [[ -n "${foreign}" ]]; then
    printf '[警告] 另有 ros2_control_node 在运行 (pid %s), 确认不是同一条控制链再继续\n' "$(pid_csv "${foreign}")"
  fi
}

# 仅在 tmux pane 内调用：准备好环境后把自身替换成 ros2 launch。
run_in_session() {
  local mode="$1"
  shift
  local spec=""

  detect_ros_distro
  spec="$(launch_spec "${mode}")"
  [[ -n "${BW_START_ARGS:-}" ]] && spec="${spec} ${BW_START_ARGS}"
  if (( $# > 0 )); then
    spec="${spec} $*"
  fi

  mkdir -p "${LOG_DIR}"
  # tmux server 可能携带旧工作区的环境, 清掉 ROS/colcon 相关变量后重新 source。
  unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH CMAKE_PREFIX_PATH \
    ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
  # ROS 的 setup.bash 不兼容 set -u。
  set +u
  # shellcheck disable=SC1090
  source "${ROS_ROOT}/${ROS_TARGET_DISTRO}/setup.bash"
  # shellcheck disable=SC1090
  source "${WORKSPACE_SETUP_FILE}"
  set -u
  cd "${REPO_ROOT}"
  printf '[start.sh] 模式=%s 命令: ros2 launch bw_std_bringup %s\n' "${mode}" "${spec}"
  # 允许词拆分, spec 由本脚本构造。
  # shellcheck disable=SC2086
  exec ros2 launch bw_std_bringup ${spec}
}

cmd_start() {
  local mode="real"
  local inner=""

  if (( $# > 0 )); then
    case "$1" in
      real | readonly | mock)
        mode="$1"
        shift
        ;;
    esac
  fi

  require_tmux
  detect_ros_distro
  preflight_start "${mode}"
  mkdir -p "${LOG_DIR}"
  : > "${LOG_FILE}"

  inner="${SCRIPT_DIR}/start.sh __run ${mode}"
  if (( $# > 0 )); then
    inner="${inner} $*"
  fi
  tmux new-session -d -s "${SESSION}" -c "${REPO_ROOT}" "${inner}"
  tmux pipe-pane -t "${SESSION}" "cat >> ${LOG_FILE}"
  sleep 2

  if ! session_exists; then
    printf '[错误] session 启动后立即退出, 日志尾部:\n'
    tail -n 20 "${LOG_FILE}" || true
    return 1
  fi
  printf '[OK] 已启动 session=%s 模式=%s\n' "${SESSION}" "${mode}"
  printf '     日志: %s\n' "${LOG_FILE}"
  printf '     接入: ./scripts/local/start.sh attach    停止: ./scripts/local/start.sh stop\n'
}

cmd_stop() {
  local force=0
  local waited=0
  local pids=""
  local holders=""

  [[ "${1:-}" == "--force" ]] && force=1
  require_tmux

  if session_exists; then
    printf '发送 Ctrl-C 优雅收尾 (最多 %s s)\n' "${STOP_TIMEOUT_SEC}"
    tmux send-keys -t "${SESSION}" C-c
    while session_exists && (( waited < STOP_TIMEOUT_SEC )); do
      sleep 1
      waited=$((waited + 1))
    done
    if session_exists; then
      printf '优雅收尾超时, 强制结束 session\n'
      tmux kill-session -t "${SESSION}"
    fi
  else
    printf 'session %s 未运行\n' "${SESSION}"
  fi

  sleep 1
  pids="$(leftover_pids)"
  if [[ -n "${pids}" ]]; then
    printf '[警告] 仍有相关进程: %s\n' "$(ps -o pid=,cmd= -p "$(pid_csv "${pids}")" 2>/dev/null | cut -c1-100 | tr '\n' ';')"
    if (( force == 1 )); then
      printf '按 --force 清理上述进程\n'
      # shellcheck disable=SC2086
      kill -INT ${pids} 2>/dev/null || true
      sleep 2
      pids="$(leftover_pids)"
      if [[ -n "${pids}" ]]; then
        # shellcheck disable=SC2086
        kill -TERM ${pids} 2>/dev/null || true
      fi
    else
      printf '     如确认无其他栈在用, 执行: ./scripts/local/start.sh stop --force\n'
    fi
  fi

  holders="$(port_holders)"
  if [[ -n "${holders}" ]]; then
    printf '[警告] %s 仍被占用: pid %s\n' "${SERIAL_PORT}" "${holders// /, }"
  else
    printf '[OK] %s 已释放\n' "${SERIAL_PORT}"
  fi
}

cmd_status() {
  require_tmux
  detect_ros_distro

  if session_exists; then
    printf '[session] %s 运行中\n' "${SESSION}"
    tmux list-panes -t "${SESSION}" -F '  pane #{pane_index} pid=#{pane_pid} #{pane_current_command}' 2>/dev/null || true
  else
    printf '[session] %s 未运行\n' "${SESSION}"
  fi

  printf '[串口] '
  if [[ -e "${SERIAL_PORT}" ]]; then
    local holders
    holders="$(port_holders)"
    if [[ -n "${holders}" ]]; then
      printf '%s 被 pid %s 占用\n' "${SERIAL_PORT}" "${holders// /, }"
    else
      printf '%s 存在且空闲\n' "${SERIAL_PORT}"
    fi
  else
    printf '%s 不存在\n' "${SERIAL_PORT}"
  fi

  printf '[进程] 本工作区遗留: %s\n' "$(pid_csv "$(leftover_pids)")"
  local pids
  pids="$(leftover_pids)"
  if [[ -n "${pids}" ]]; then
    ps -o pid=,etime=,cmd= -p "$(pid_csv "${pids}")" 2>/dev/null | cut -c1-130 || true
  fi
  printf '       全部 ros2_control_node: %s\n' "$(pid_csv "$(control_node_pids)")"
  if [[ -f "${LOG_FILE}" ]]; then
    printf '[日志] %s (尾部 5 行)\n' "${LOG_FILE}"
    tail -n 5 "${LOG_FILE}" | cut -c1-130 || true
  fi
}

cmd_logs() {
  detect_ros_distro
  [[ -f "${LOG_FILE}" ]] || die "日志不存在：${LOG_FILE}"
  if [[ "${1:-}" == "-f" ]]; then
    tail -f "${LOG_FILE}"
  else
    tail -n "${BW_START_LOG_LINES:-60}" "${LOG_FILE}"
  fi
}

main() {
  local command="${1:-}"
  case "${command}" in
    start)
      shift
      cmd_start "$@"
      ;;
    stop)
      shift
      cmd_stop "$@"
      ;;
    restart)
      shift
      cmd_stop --force
      cmd_start "$@"
      ;;
    attach)
      require_tmux
      session_exists || die "session ${SESSION} 未运行"
      printf '离开 session: Ctrl-b d\n'
      tmux attach -t "${SESSION}"
      ;;
    status)
      cmd_status
      ;;
    logs)
      shift
      cmd_logs "$@"
      ;;
    list)
      require_tmux
      tmux ls
      ;;
    __run)
      shift
      run_in_session "$@"
      ;;
    *)
      usage
      [[ -z "${command}" || "${command}" == "-h" || "${command}" == "--help" ]] && return 0
      die "未知命令：${command}"
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
