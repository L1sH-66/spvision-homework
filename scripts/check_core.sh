#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p test-output
g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 \
 -Isrc/sp_controller_server/include -Isrc/sp_global_planner/include \
 tests/core_test.cpp -o test-output/core_test
./test-output/core_test
