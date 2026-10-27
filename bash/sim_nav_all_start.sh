#!/usr/bin/env bash
set -eo pipefail
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$WS_DIR/install/setup.bash" ]]; then
  echo "请先运行 bash scripts/build.sh"; exit 1
fi
source "$WS_DIR/install/setup.bash"
exec ros2 launch sp_nav_bringup evaluation.launch.py
