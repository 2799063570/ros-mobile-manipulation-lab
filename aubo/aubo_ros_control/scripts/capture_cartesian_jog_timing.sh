#!/usr/bin/env bash
set -euo pipefail

# This launcher starts the ordinary SDK node disabled. Only the operator can
# request motion through the panel; there is no goal publisher or retry loop.
capture_prepare=false
case "${1:-}" in
  --help|-h)
    echo '用法：capture_cartesian_jog_timing.sh [--prepare] [roslaunch参数:=值 ...]'
    echo '--prepare：仅准备唯一采集目录和配置记录，不启动ROS或连接机械臂。'
    echo 'AUBO_JOG_TIMING_ROOT 可指定采集根目录；每次创建独立子目录。'
    exit 0 ;;
  --prepare) capture_prepare=true; shift ;;
esac

for capture_tool in rospack python3; do
  command -v "$capture_tool" >/dev/null 2>&1 || {
    echo "缺少命令：$capture_tool；请先 source ROS 工作空间。" >&2; exit 2;
  }
done
if ! "$capture_prepare"; then
  command -v roslaunch >/dev/null 2>&1 || { echo '缺少 roslaunch' >&2; exit 2; }
fi

capture_package="$(rospack find aubo_ros_control)"
capture_root="${AUBO_JOG_TIMING_ROOT:-$capture_package/logs}"
mkdir -p -- "$capture_root"
capture_directory="$(mktemp -d "$capture_root/jog_timing_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
capture_directory="$(cd -- "$capture_directory" && pwd)"
capture_config="$capture_package/config/cartesian_jog.yaml"
for capture_arg in "$@"; do
  case "$capture_arg" in
    config:=*) capture_config="${capture_arg#config:=}" ;;
    backend:=*|timing_trace_path:=*|node_launch_prefix:=*)
      echo "采集入口固定 SDK 后端、无 strace、独立CSV路径；请移除 $capture_arg" >&2
      exit 2 ;;
  esac
done
capture_command=(roslaunch aubo_ros_control cartesian_jog_real.launch "$@"
  backend:=sdk "timing_trace_path:=$capture_directory/timing.csv")

python3 - "$capture_directory" "$capture_package" "$capture_config" "${capture_command[@]}" <<'PY'
import datetime
import hashlib
import json
import shutil
import sys
from pathlib import Path
record, package, config = (Path(value).resolve() for value in sys.argv[1:4])
shutil.copyfile(config, record / 'config.yaml')
sources = ['src/cartesian_jog.cpp', 'src/direct_sdk_backend.cpp',
           'include/aubo_ros_control/cartesian_jog_common.h',
           'include/aubo_ros_control/sdk_jog_timing.h',
           'include/aubo_ros_control/sdk_flow_control.h',
           'include/aubo_ros_control/motion_timing_trace.h',
           'launch/cartesian_jog_real.launch', 'launch/cartesian_jog.launch']
hashes = {name: hashlib.sha256((package / name).read_bytes()).hexdigest()
          for name in sources if (package / name).is_file()}
(record / 'manifest.json').write_text(json.dumps(dict(
    created_at=datetime.datetime.now().astimezone().isoformat(),
    argv=sys.argv[4:], config_path=str(config),
    config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(), source_sha256=hashes,
    note='Source/config snapshot; does not certify binary identity or capture all effective ROS parameters. No automatic motion or retries.'),
    ensure_ascii=False, indent=2) + '\n')
PY

echo "采集目录：$capture_directory"
if "$capture_prepare"; then
  printf '仅准备，未执行：'; printf ' %q' "${capture_command[@]}"; printf '\n'
  exit 0
fi

capture_finish() {
  local capture_status=$?
  trap - EXIT
  printf '%s\n' "$capture_status" > "$capture_directory/launch_exit_code.txt"
  if python3 "$capture_package/scripts/analyze_motion_timing.py" \
       "$capture_directory/timing.csv" --output "$capture_directory/analysis" \
       > "$capture_directory/analysis.log" 2>&1; then
    echo "时间线、批次水位、耗时统计：$capture_directory/analysis"
  else
    echo "未生成完整分析；保留原始记录，详情：$capture_directory/analysis.log" >&2
  fi
  echo "采集结果：$capture_directory"
  exit "$capture_status"
}
trap capture_finish EXIT
# Ctrl+C is delivered to the foreground launch/tee as well. Let the node finish
# its normal shutdown/dump before the EXIT handler reads the trace.
trap ':' INT
echo '启动后默认禁用，请通过面板操作；结束后 Ctrl+C，等待节点导出并自动分析。'
set +e
PYTHONUNBUFFERED=1 "${capture_command[@]}" 2>&1 | tee "$capture_directory/console.log"
capture_status=${PIPESTATUS[0]}
set -e
exit "$capture_status"
