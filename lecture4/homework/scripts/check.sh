#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
mkdir -p test-output
# Isolate this check from other ROS nodes on the computer.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY=1
pub_pid=''
sub_pid=''
cleanup() {
  for pid in "$pub_pid" "$sub_pid"; do
    if [[ -n "$pid" ]]; then kill -INT "$pid" 2>/dev/null || true; fi
  done
  for pid in "$pub_pid" "$sub_pid"; do
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
}
trap cleanup EXIT
# Run installed executables directly so signals reach the actual nodes.
"$(ros2 pkg prefix qos_debugger)/lib/qos_debugger/qos_debugger_sub" >test-output/subscriber.log 2>&1 &
sub_pid=$!
sleep 2
"$(ros2 pkg prefix qos_debugger)/lib/qos_debugger/qos_debugger_pub" >test-output/publisher.log 2>&1 &
pub_pid=$!
sleep 10
kill -0 "$sub_pid"
kill -0 "$pub_pid"
cleanup
sub_pid=''
pub_pid=''
python3 - <<'PY'
import re
from pathlib import Path
s = Path('test-output/subscriber.log').read_text()
fps = [float(x) for x in re.findall(r'接收帧率: ([0-9.]+) Hz', s)]
assert len(fps) >= 6, 'Insufficient one-second reports'
assert sum(80 <= x <= 120 for x in fps) >= 5, f'Unexpected receive rates: {fps}'
assert '检测到丢包' not in s, 'Sequence gaps detected; inspect load and QoS'
assert '丢包率 0.00%' in s, 'Missing loss report'
print('PASS: nodes stay alive, receive near 100 Hz, report zero sequence gaps')
PY
