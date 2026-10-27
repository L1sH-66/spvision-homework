#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
bash scripts/check_core.sh
python3 scripts/check_static.py
source /opt/ros/humble/setup.bash
# Single job prevents exprtk/Qt compilation exhausting a small VM.
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-g0
source install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
python3 scripts/check_static.py
