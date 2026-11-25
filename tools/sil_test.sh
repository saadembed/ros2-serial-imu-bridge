#!/usr/bin/env bash
# Software-in-the-loop test inside the ROS 2 image, without an STM32:
#   virtual_board (the real firmware driver + frame code, simulated MPU-6050)
#     -> socat pty pair (a virtual serial cable)
#     -> bridge_node on a real serial port -> imu_filter -> sil_check.py
# Then the cable is unplugged and plugged back to test the node's reconnect.
#
# Usage (from the repo root):
#   docker build -t serial_imu_bridge .
#   docker run --rm -v "$PWD/tools:/tools:ro" serial_imu_bridge bash /tools/sil_test.sh
# (no "set -u": the ROS setup scripts read unset variables)
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
FW=/ws/src/serial_imu_bridge/firmware
ROLL=20
PITCH=-10
FAIL=0

section() { echo; echo "=== $* ==="; }

section "build the virtual board from the firmware sources"
gcc -std=c99 -Wall -Wextra -Werror -O2 -I"$FW/protocol" -I"$FW/drivers" \
    -o /tmp/virtual_board "$FW/sim/virtual_board.c" "$FW/protocol/frame.c" "$FW/drivers/mpu6050.c" -lm \
    && echo "built /tmp/virtual_board" || exit 1

plug_cable() {
    socat pty,raw,echo=0,link=/tmp/ttyMCU pty,raw,echo=0,link=/tmp/ttyHOST 2>/tmp/socat.log &
    SOCAT_PID=$!
    for _ in $(seq 20); do [ -e /tmp/ttyMCU ] && [ -e /tmp/ttyHOST ] && break; sleep 0.1; done
    /tmp/virtual_board --port /tmp/ttyMCU --rate 100 --roll "$ROLL" --pitch "$PITCH" 2>>/tmp/board.log &
    BOARD_PID=$!
}
unplug_cable() {
    kill -TERM "$BOARD_PID" "$SOCAT_PID" 2>/dev/null
    wait "$BOARD_PID" "$SOCAT_PID" 2>/dev/null
}

plug_cable
ros2 launch serial_imu_bridge bridge.launch.py port:=/tmp/ttyHOST with_filter:=true > /tmp/launch.log 2>&1 &
LAUNCH_PID=$!
sleep 5

section "phase 1: virtual board on a serial port, held at roll ${ROLL}, pitch ${PITCH}"
python3 /tools/sil_check.py --seconds 8 --roll "$ROLL" --pitch "$PITCH" || FAIL=1

section "phase 2: unplug the virtual cable for 4 s"
unplug_cable
sleep 4
timeout 5 ros2 topic echo --once /diagnostics | grep -E "level|message|key: connected" -A1 | head -n 8

section "phase 3: plug it back, the node must reconnect"
plug_cable
sleep 4
python3 /tools/sil_check.py --seconds 6 --roll "$ROLL" --pitch "$PITCH" || FAIL=1

unplug_cable
# Background jobs in a non-interactive shell ignore SIGINT, so stop the launch with SIGTERM.
kill -TERM "$LAUNCH_PID" 2>/dev/null
for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
kill -KILL "$LAUNCH_PID" 2>/dev/null

section "bridge log: serial events"
grep -E "serial|port|reconnect|open" /tmp/launch.log | head -n 10
section "virtual board log"
cat /tmp/board.log

section "result"
if [ "$FAIL" -eq 0 ]; then echo "SIL test PASSED"; else echo "SIL test FAILED"; fi
exit "$FAIL"
