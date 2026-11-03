#!/usr/bin/env bash
# 树莓派低内存构建入口：只编译和测试，不启动任何舵机动作。
set -eo pipefail
workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace_root"
source /opt/ros/jazzy/setup.bash
# colcon 会为 make 自动添加按 CPU 核数确定的 -j；必须显式设置 MAKEFLAGS。
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
# 只影响本脚本及其子进程；内存不足时让编译失败，避免耗尽整机内存。
ulimit -v 524288
colcon build --packages-select hexapod_core --executor sequential \
  --event-handlers console_direct+ --cmake-args \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug '-DCMAKE_CXX_FLAGS_DEBUG=-O0 -g0' \
  "-Dnlohmann_json_DIR=$workspace_root/dependencies/json-3.11.3/cmake"
colcon test --packages-select hexapod_core --event-handlers console_direct+
colcon test-result --verbose
source install/setup.bash
ros2 pkg prefix hexapod_core
