#!/usr/bin/env bash
set -euo pipefail

# Uses the ordinary, disabled-on-start real launch. Movement is requested by
# the operator through the panel; this script never publishes a motion goal.
for capture_tool in roslaunch rospack strace python3; do
  if ! command -v "$capture_tool" >/dev/null 2>&1; then
    echo "缺少命令：$capture_tool；请先 source ROS 工作空间并安装 strace。" >&2
    exit 2
  fi
done
capture_package="$(rospack find aubo_ros_control)"
capture_directory="${AUBO_JOG_CAPTURE_DIR:-$capture_package/logs/sdk_capture_$(date +%Y%m%d_%H%M%S)_$$}"
mkdir -p -- "$capture_directory"
capture_directory="$(cd -- "$capture_directory" && pwd)"
capture_prefix="$(python3 - "$capture_directory/sdk" <<'PY'
import shlex
import sys
args = ['strace', '-ff', '-ttt', '-T', '-o', sys.argv[1],
        '-e', 'trace=connect,sendto,sendmsg,recvfrom,recvmsg,select,pselect6,poll,ppoll,futex',
        '-e', 'raw=sendto,sendmsg,recvfrom,recvmsg']
print(' '.join(shlex.quote(arg) for arg in args))
PY
)"
echo "采集目录：$capture_directory"
echo "启动后保持禁用；本脚本不自动发送 home 或点动指令。"
echo "strace 会扰动调度，日志用于定位等待阶段，不作为实时性能验证。"
trap 'echo "采集结果保存在：$capture_directory"' EXIT
PYTHONUNBUFFERED=1 roslaunch aubo_ros_control cartesian_jog_real.launch backend:=sdk "$@" \
  "node_launch_prefix:=$capture_prefix" 2>&1 | tee "$capture_directory/console.log"
