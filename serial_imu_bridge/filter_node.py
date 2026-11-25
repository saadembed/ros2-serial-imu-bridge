"""ROS 2 node: complementary filter for roll and pitch.

Subscribes: imu/data_raw  sensor_msgs/Imu  (accel + gyro, no orientation)
Publishes:  imu/data      sensor_msgs/Imu  (same data plus orientation)

dt comes from the message header stamps, not from arrival time. Yaw is held at 0:
it is unobservable without a magnetometer, so its variance is published as a
very large number and consumers (e.g. robot_localization) should not fuse it.
The math lives in complementary.py, which has no ROS dependency.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu

from .complementary import ComplementaryFilter

# Placeholder 1-sigma for roll/pitch, not calibrated against a reference.
DEFAULT_TILT_STDDEV_RAD = 0.05
YAW_VARIANCE_UNOBSERVABLE = 1e6


class ImuFilter(Node):
    def __init__(self) -> None:
        super().__init__("imu_filter")
        self.declare_parameter("alpha", 0.98)
        self.declare_parameter("max_dt", 0.5)
        self.declare_parameter("accel_gate", 0.2)  # fraction of g; <= 0 disables the gate
        self.declare_parameter("tilt_stddev", DEFAULT_TILT_STDDEV_RAD)

        gate = self.get_parameter("accel_gate").value
        self._filter = ComplementaryFilter(
            alpha=self.get_parameter("alpha").value,
            max_dt=self.get_parameter("max_dt").value,
            accel_gate=gate if gate > 0 else None,
        )
        tilt_var = self.get_parameter("tilt_stddev").value ** 2
        self._orientation_cov = [
            tilt_var, 0.0, 0.0,
            0.0, tilt_var, 0.0,
            0.0, 0.0, YAW_VARIANCE_UNOBSERVABLE,
        ]
        self._rejected_reported = 0
        self._resets_reported = 0

        self._pub = self.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self._sub = self.create_subscription(Imu, "imu/data_raw", self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f"complementary filter: alpha={self._filter.alpha}, max_dt={self._filter.max_dt}s; "
            "yaw is not estimated (no magnetometer)"
        )

    def _on_imu(self, raw: Imu) -> None:
        t = Time.from_msg(raw.header.stamp).nanoseconds * 1e-9
        a, w = raw.linear_acceleration, raw.angular_velocity
        updated = self._filter.update(t, (a.x, a.y, a.z), (w.x, w.y, w.z))
        self._report_guards()
        if not updated:
            return

        out = Imu()
        out.header = raw.header
        x, y, z, qw = self._filter.quaternion()
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = x, y, z, qw
        out.orientation_covariance = self._orientation_cov
        out.angular_velocity = raw.angular_velocity
        out.angular_velocity_covariance = raw.angular_velocity_covariance
        out.linear_acceleration = raw.linear_acceleration
        out.linear_acceleration_covariance = raw.linear_acceleration_covariance
        self._pub.publish(out)

    def _report_guards(self) -> None:
        f = self._filter
        if f.rejected_samples != self._rejected_reported:
            self._rejected_reported = f.rejected_samples
            self.get_logger().warn(
                f"dropped sample with non-increasing stamp ({f.rejected_samples} so far)",
                throttle_duration_sec=5.0,
            )
        if f.resets != self._resets_reported:
            self._resets_reported = f.resets
            self.get_logger().warn(
                f"gap > {f.max_dt}s between samples: re-initialised from accelerometer ({f.resets} so far)",
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImuFilter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
