"""ROS 2 node: read framed IMU data from a serial port and publish it.

Topics:
    imu/data_raw    sensor_msgs/Imu   (sensor-data QoS: best effort, keep last 5)
    /diagnostics    diagnostic_msgs/DiagnosticArray   (link health, 1 Hz)

With an empty `port` parameter the node reads from a simulated source.
If the serial port cannot be opened, or fails while running, the node keeps
running, reports ERROR on /diagnostics and retries the port once per second.
"""
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu

from .frame import FrameParser
from .imu import PAYLOAD_SIZE, decode_imu
from .simulator import SimulatedImuSource
from .stamping import batch_stamps_ns

RECONNECT_PERIOD_NS = 1_000_000_000


class ImuBridge(Node):
    def __init__(self) -> None:
        super().__init__("imu_bridge")
        self.declare_parameter("port", "")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("poll_hz", 200.0)
        # Nominal output rate of the firmware; only used to back-date frames that
        # arrive in the same read (see stamping.py).
        self.declare_parameter("sample_rate_hz", 100.0)
        self.declare_parameter("sim_rate_hz", 100.0)
        self.declare_parameter("sim_error_rate", 0.02)

        poll_hz = self._positive("poll_hz")
        self._port = self.get_parameter("port").value
        self._baud = self.get_parameter("baud").value
        self._frame_id = self.get_parameter("frame_id").value

        self._parser = FrameParser()
        self._serial = None
        self._sim = None
        self._last_open_attempt_ns = None
        if self._port:
            self._period_ns = int(1e9 / self._positive("sample_rate_hz"))
            self._try_open_serial()
        else:
            sim_rate = self._positive("sim_rate_hz")
            self._period_ns = int(1e9 / sim_rate)
            self._sim = SimulatedImuSource(
                rate_hz=sim_rate,
                error_rate=self.get_parameter("sim_error_rate").value,
            )
            self.get_logger().info("no port given: using the simulated source")

        self._imu_pub = self.create_publisher(Imu, "imu/data_raw", qos_profile_sensor_data)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._last_stamp_ns = None
        self._last_errors = 0
        self._last_frames_ok = 0
        self._last_diag_ns = self.get_clock().now().nanoseconds
        self._poll_timer = self.create_timer(1.0 / poll_hz, self._poll)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)

    def _positive(self, name: str) -> float:
        value = self.get_parameter(name).value
        if value <= 0:
            raise ValueError(f"parameter '{name}' must be > 0, got {value}")
        return value

    # --- serial port handling -------------------------------------------------

    def _try_open_serial(self) -> None:
        import serial  # pyserial

        self._last_open_attempt_ns = self.get_clock().now().nanoseconds
        try:
            self._serial = serial.Serial(self._port, self._baud, timeout=0)
        except (serial.SerialException, OSError, ValueError) as exc:
            self._serial = None
            self.get_logger().error(
                f"cannot open {self._port} at {self._baud} baud: {exc}; retrying",
                throttle_duration_sec=10.0,
            )
            return
        self._parser.reset_link()
        self.get_logger().info(f"reading serial port {self._port} at {self._baud} baud")

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:  # noqa: BLE001 - closing a dead port must not raise
                pass
            self._serial = None

    def _read_bytes(self) -> bytes:
        if self._sim is not None:
            return self._sim.read()
        if self._serial is None:
            now = self.get_clock().now().nanoseconds
            if now - self._last_open_attempt_ns >= RECONNECT_PERIOD_NS:
                self._try_open_serial()
            return b""
        try:
            waiting = self._serial.in_waiting
            return self._serial.read(waiting) if waiting else b""
        except OSError as exc:  # serial.SerialException derives from OSError
            self.get_logger().error(f"serial port {self._port} failed: {exc}; reconnecting")
            self._close_serial()
            return b""

    # --- timers ---------------------------------------------------------------

    def _poll(self) -> None:
        data = self._read_bytes()
        if not data:
            return
        read_time_ns = self.get_clock().now().nanoseconds
        frames = [f for f in self._parser.feed(data) if len(f.payload) == PAYLOAD_SIZE]
        stamps = batch_stamps_ns(read_time_ns, len(frames), self._period_ns, self._last_stamp_ns)
        for frame, stamp_ns in zip(frames, stamps):
            ax, ay, az, gx, gy, gz = decode_imu(frame.payload)
            msg = Imu()
            msg.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            msg.header.frame_id = self._frame_id
            msg.orientation_covariance[0] = -1.0  # this sensor does not provide orientation
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
            self._imu_pub.publish(msg)
        if stamps:
            self._last_stamp_ns = stamps[-1]

    def _publish_diagnostics(self) -> None:
        p = self._parser
        now_ns = self.get_clock().now().nanoseconds
        elapsed = max((now_ns - self._last_diag_ns) * 1e-9, 1e-9)
        new_frames = p.frames_ok - self._last_frames_ok
        errors = p.crc_errors + p.lost_frames
        connected = self._sim is not None or self._serial is not None

        status = DiagnosticStatus()
        status.name = "imu_bridge: serial link"
        status.hardware_id = self._port or "simulated"
        if not connected:
            status.level, status.message = DiagnosticStatus.ERROR, "serial port not open"
        elif new_frames == 0:
            status.level, status.message = DiagnosticStatus.ERROR, "no valid frames in the last period"
        elif errors != self._last_errors:
            status.level, status.message = DiagnosticStatus.WARN, "errors in the last period"
        else:
            status.level, status.message = DiagnosticStatus.OK, "ok"
        status.values = [
            KeyValue(key="connected", value=str(connected)),
            KeyValue(key="frame_rate_hz", value=f"{new_frames / elapsed:.1f}"),
            KeyValue(key="frames_ok", value=str(p.frames_ok)),
            KeyValue(key="crc_errors", value=str(p.crc_errors)),
            KeyValue(key="lost_frames", value=str(p.lost_frames)),
            KeyValue(key="resyncs", value=str(p.resyncs)),
        ]
        self._last_errors = errors
        self._last_frames_ok = p.frames_ok
        self._last_diag_ns = now_ns

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self._diag_pub.publish(array)

    def destroy_node(self) -> None:
        self._close_serial()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImuBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
