#!/usr/bin/env bash
# Smoke test inside the ROS 2 image: launch both nodes on the simulated source,
# check topics, rates and messages, record a short rosbag, then run colcon test.
# Usage (from the repo root):
#   docker build -t serial_imu_bridge .
#   docker run --rm -v "$PWD/tools:/tools:ro" serial_imu_bridge bash /tools/ros2_smoke_test.sh
# (no "set -u": the ROS setup scripts read unset variables)
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
cd /ws

section() { echo; echo "=== $* ==="; }

ros2 launch serial_imu_bridge bridge.launch.py with_filter:=true > /tmp/launch.log 2>&1 &
LAUNCH_PID=$!
sleep 6

section "nodes"
ros2 node list

section "topics"
ros2 topic list -t

section "rate /imu/data_raw (8 s)"
timeout 8 ros2 topic hz /imu/data_raw --window 200 2>&1 | tail -n 4

section "rate /imu/data (8 s)"
timeout 8 ros2 topic hz /imu/data --window 200 2>&1 | tail -n 4

section "one /imu/data message"
timeout 5 ros2 topic echo --once /imu/data

section "one /diagnostics message"
timeout 5 ros2 topic echo --once /diagnostics

section "QoS of /imu/data_raw"
ros2 topic info -v /imu/data_raw

section "rosbag (10 s)"
rm -rf /tmp/smoke_bag
timeout -s INT 10 ros2 bag record -o /tmp/smoke_bag /imu/data_raw /imu/data /diagnostics > /tmp/bag.log 2>&1
ros2 bag info /tmp/smoke_bag

# Background jobs in a non-interactive shell ignore SIGINT, so stop the launch with SIGTERM.
kill -TERM "$LAUNCH_PID" 2>/dev/null
for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
kill -KILL "$LAUNCH_PID" 2>/dev/null

section "launch log (last lines)"
tail -n 15 /tmp/launch.log

section "colcon test"
colcon test --packages-select serial_imu_bridge --event-handlers console_direct+ > /tmp/colcon_test.log 2>&1
tail -n 5 /tmp/colcon_test.log
colcon test-result --verbose | tail -n 5
