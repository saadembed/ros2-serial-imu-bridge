"""Listen to /imu/data and /diagnostics for a while and check the result.

Used by tools/sil_test.sh: the virtual board holds a known roll and pitch, so
the filter output must converge to those angles, at the board's frame rate,
with no CRC errors on the link.

Exit code 0 when every check passes, 1 otherwise.
"""
import argparse
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def roll_pitch_deg(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch)


class Checker(Node):
    def __init__(self):
        super().__init__("sil_check")
        self.samples = []
        self.diag = {}
        self.create_subscription(Imu, "imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, 10)

    def _on_imu(self, msg):
        self.samples.append((time.monotonic(), *roll_pitch_deg(msg.orientation)))

    def _on_diag(self, msg):
        for status in msg.status:
            if status.name.startswith("imu_bridge"):
                self.diag = {kv.key: kv.value for kv in status.values}
                self.diag["message"] = status.message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--roll", type=float, required=True)
    ap.add_argument("--pitch", type=float, required=True)
    ap.add_argument("--tolerance-deg", type=float, default=1.0)
    args = ap.parse_args()

    rclpy.init()
    node = Checker()
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    samples = node.samples
    rate = len(samples) / args.seconds
    tail = samples[len(samples) // 2:]  # second half: the filter has converged
    roll = sum(s[1] for s in tail) / len(tail) if tail else float("nan")
    pitch = sum(s[2] for s in tail) / len(tail) if tail else float("nan")
    crc_errors = int(node.diag.get("crc_errors", "-1"))

    checks = {
        f"rate {rate:.1f} Hz within 10% of {args.rate:.0f} Hz": abs(rate - args.rate) <= 0.1 * args.rate,
        f"roll {roll:.2f} deg vs {args.roll:.2f} deg": abs(roll - args.roll) <= args.tolerance_deg,
        f"pitch {pitch:.2f} deg vs {args.pitch:.2f} deg": abs(pitch - args.pitch) <= args.tolerance_deg,
        f"crc_errors = {crc_errors}": crc_errors == 0,
    }
    print(f"messages on /imu/data: {len(samples)} in {args.seconds:.0f} s")
    print(f"last diagnostics: {node.diag}")
    for name, ok in checks.items():
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
