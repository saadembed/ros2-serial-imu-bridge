"""Complementary filter for roll and pitch (no ROS dependency).

Conventions (REP-103 body frame): x forward, y left, z up. At rest the
accelerometer measures +g on z (it measures specific force, i.e. the reaction to
gravity). Angles are ZYX Euler angles in radians; gyro rates are in rad/s.

    predicted = angle + angle_rate(gyro) * dt          (short term: gyro)
    angle     = predicted + (1 - alpha) * wrap(accel_angle - predicted)

which is the usual alpha * gyro + (1 - alpha) * accel blend, written so that it
stays correct when roll crosses +/-pi. alpha is applied per sample, so the
effective time constant depends on the sample period:
tau = alpha * dt / (1 - alpha) (e.g. alpha = 0.98 at 100 Hz -> tau ~= 0.49 s).

Yaw is not estimated: without a magnetometer (or another heading reference) it is
unobservable from gravity, and integrating gz would only drift. It is held at 0.
"""
import math
from typing import Optional, Sequence, Tuple

Quaternion = Tuple[float, float, float, float]  # (x, y, z, w), same order as geometry_msgs


def wrap_angle(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def tilt_from_accel(ax: float, ay: float, az: float) -> Tuple[float, float]:
    """Roll and pitch implied by the gravity vector measured by the accelerometer."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.hypot(ay, az))
    return roll, pitch


def euler_rates(roll: float, pitch: float, gx: float, gy: float, gz: float) -> Tuple[float, float]:
    """ZYX Euler angle rates (roll_dot, pitch_dot) from body angular velocity.

    Singular at pitch = +/-90 deg (gimbal lock); the tan term is clamped there.
    """
    sr, cr = math.sin(roll), math.cos(roll)
    cp = math.cos(pitch)
    tp = math.sin(pitch) / cp if abs(cp) > 1e-6 else math.copysign(1e6, math.sin(pitch))
    roll_dot = gx + (sr * gy + cr * gz) * tp
    pitch_dot = cr * gy - sr * gz
    return roll_dot, pitch_dot


def quaternion_from_rpy(roll: float, pitch: float, yaw: float = 0.0) -> Quaternion:
    """ZYX (yaw, then pitch, then roll) Euler angles to a unit quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ComplementaryFilter:
    """Roll/pitch estimator. Feed it samples with their timestamps via update().

    alpha:       weight of the gyro prediction per sample, in [0, 1].
    max_dt:      a gap longer than this (seconds) re-initialises from the
                 accelerometer instead of integrating the gyro over the gap.
    accel_gate:  skip the accelerometer correction when |a| differs from g by more
                 than this fraction (the sensor is accelerating, so the
                 accelerometer is not measuring gravity alone). None disables it.
    """

    def __init__(
        self,
        alpha: float = 0.98,
        max_dt: float = 0.5,
        accel_gate: Optional[float] = 0.2,
        gravity: float = 9.80665,
    ) -> None:
        if not 0.0 <= alpha <= 1.0:
            raise ValueError(f"alpha must be in [0, 1], got {alpha}")
        if max_dt <= 0:
            raise ValueError(f"max_dt must be > 0, got {max_dt}")
        self.alpha = alpha
        self.max_dt = max_dt
        self.accel_gate = accel_gate
        self.gravity = gravity
        self.roll = 0.0
        self.pitch = 0.0
        self.initialized = False
        self._last_t: Optional[float] = None
        self.rejected_samples = 0  # dt <= 0
        self.resets = 0  # dt > max_dt

    def _accel_usable(self, accel: Sequence[float]) -> bool:
        norm = math.sqrt(sum(a * a for a in accel))
        if norm < 1e-6:
            return False
        if self.accel_gate is None:
            return True
        return abs(norm - self.gravity) <= self.accel_gate * self.gravity

    def reset(self) -> None:
        self.initialized = False
        self._last_t = None

    def update(self, t: float, accel: Sequence[float], gyro: Sequence[float]) -> bool:
        """Process one sample taken at time t (seconds).

        Returns True if the estimate was updated, False if the sample was rejected
        (non-increasing timestamp, or no usable accelerometer reading to start from).
        """
        if self._last_t is not None:
            dt = t - self._last_t
            if dt <= 0.0:
                self.rejected_samples += 1
                return False
            if dt > self.max_dt and self.initialized:
                self.resets += 1
                self.initialized = False

        if not self.initialized:
            if not self._accel_usable(accel):
                return False
            self.roll, self.pitch = tilt_from_accel(*accel)
            self.initialized = True
            self._last_t = t
            return True

        roll_dot, pitch_dot = euler_rates(self.roll, self.pitch, *gyro)
        roll = self.roll + roll_dot * dt
        pitch = self.pitch + pitch_dot * dt
        if self._accel_usable(accel):
            acc_roll, acc_pitch = tilt_from_accel(*accel)
            k = 1.0 - self.alpha
            roll += k * wrap_angle(acc_roll - roll)
            pitch += k * wrap_angle(acc_pitch - pitch)
        self.roll = wrap_angle(roll)
        self.pitch = max(-math.pi / 2, min(math.pi / 2, pitch))
        self._last_t = t
        return True

    def quaternion(self) -> Quaternion:
        return quaternion_from_rpy(self.roll, self.pitch, 0.0)
