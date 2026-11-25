import math

import pytest

from serial_imu_bridge.complementary import (
    ComplementaryFilter,
    quaternion_from_rpy,
    tilt_from_accel,
    wrap_angle,
)

G = 9.80665
RATE_HZ = 100.0
DT = 1.0 / RATE_HZ


def gravity_in_body(roll: float, pitch: float):
    """Accelerometer reading of a static sensor at (roll, pitch), REP-103 axes."""
    return (
        -G * math.sin(pitch),
        G * math.sin(roll) * math.cos(pitch),
        G * math.cos(roll) * math.cos(pitch),
    )


def run(f, accel, gyro, seconds, t0=0.0):
    n = int(round(seconds * RATE_HZ))
    for i in range(n):
        f.update(t0 + i * DT, accel, gyro)
    return t0 + n * DT


def test_static_level_sensor_stays_at_zero():
    f = ComplementaryFilter(alpha=0.98)
    run(f, (0.0, 0.0, G), (0.0, 0.0, 0.0), seconds=2.0)
    assert f.roll == pytest.approx(0.0, abs=1e-9)
    assert f.pitch == pytest.approx(0.0, abs=1e-9)


def test_level_sensor_converges_from_wrong_initial_estimate():
    f = ComplementaryFilter(alpha=0.98)
    f.update(0.0, gravity_in_body(0.5, -0.4), (0.0, 0.0, 0.0))  # initialises at a tilt
    assert f.roll == pytest.approx(0.5)
    run(f, (0.0, 0.0, G), (0.0, 0.0, 0.0), seconds=5.0, t0=DT)
    # tau ~= 0.49 s at alpha 0.98 / 100 Hz, so 5 s is ~10 time constants.
    assert f.roll == pytest.approx(0.0, abs=1e-3)
    assert f.pitch == pytest.approx(0.0, abs=1e-3)


@pytest.mark.parametrize("roll,pitch", [(math.radians(30), 0.0), (0.0, math.radians(-20)), (0.3, 0.4)])
def test_known_tilt_from_gravity_converges(roll, pitch):
    f = ComplementaryFilter(alpha=0.98)
    f.update(0.0, (0.0, 0.0, G), (0.0, 0.0, 0.0))  # start level, then hold the tilt
    run(f, gravity_in_body(roll, pitch), (0.0, 0.0, 0.0), seconds=5.0, t0=DT)
    assert f.roll == pytest.approx(roll, abs=1e-3)
    assert f.pitch == pytest.approx(pitch, abs=1e-3)


def test_tilt_from_accel_matches_known_angles():
    roll, pitch = tilt_from_accel(*gravity_in_body(0.2, -0.7))
    assert (roll, pitch) == (pytest.approx(0.2), pytest.approx(-0.7))


def test_pure_gyro_rotation_integrates_over_short_time():
    # alpha = 1 ignores the accelerometer, so this isolates the integration.
    f = ComplementaryFilter(alpha=1.0)
    f.update(0.0, (0.0, 0.0, G), (0.0, 0.0, 0.0))
    rate = math.radians(45)  # rad/s about x
    run(f, (0.0, 0.0, G), (rate, 0.0, 0.0), seconds=0.5, t0=DT)
    # 50 updates of 10 ms each (the first sample at t=0 only initialises).
    assert f.roll == pytest.approx(rate * 0.5, rel=1e-9)
    assert f.pitch == pytest.approx(0.0, abs=1e-12)


def test_gyro_dominates_short_term_with_default_alpha():
    # A 0.2 s rotation at 45 deg/s with the accelerometer still saying "level":
    # the estimate follows most of the gyro motion, i.e. the filter is not just the accel.
    f = ComplementaryFilter(alpha=0.98)
    f.update(0.0, (0.0, 0.0, G), (0.0, 0.0, 0.0))
    rate = math.radians(45)
    run(f, (0.0, 0.0, G), (0.0, rate, 0.0), seconds=0.2, t0=DT)
    assert 0.6 * rate * 0.2 < f.pitch < rate * 0.2


@pytest.mark.parametrize("roll,pitch", [(0.0, 0.0), (0.3, -0.2), (-2.5, 1.2), (math.pi / 2, 0.0)])
def test_quaternion_is_unit_and_matches_roll_pitch(roll, pitch):
    x, y, z, w = quaternion_from_rpy(roll, pitch)
    assert math.sqrt(x * x + y * y + z * z + w * w) == pytest.approx(1.0)
    # Back to ZYX Euler angles (standard formulas).
    back_roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    back_pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    back_yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    assert back_roll == pytest.approx(roll, abs=1e-9)
    assert back_pitch == pytest.approx(pitch, abs=1e-9)
    assert back_yaw == pytest.approx(0.0, abs=1e-9)


def test_quaternion_rotates_gravity_into_measured_accel():
    # The body-to-world rotation applied to the static accel reading must give +g on world z.
    roll, pitch = 0.4, -0.3
    x, y, z, w = quaternion_from_rpy(roll, pitch)
    ax, ay, az = gravity_in_body(roll, pitch)
    # Third row of the rotation matrix of q, dotted with the body vector.
    world_z = 2 * (x * z - w * y) * ax + 2 * (y * z + w * x) * ay + (1 - 2 * (x * x + y * y)) * az
    assert world_z == pytest.approx(G)


def test_filter_quaternion_matches_its_angles():
    f = ComplementaryFilter()
    f.update(0.0, gravity_in_body(0.25, 0.1), (0.0, 0.0, 0.0))
    assert f.quaternion() == pytest.approx(quaternion_from_rpy(0.25, 0.1, 0.0))


def test_non_increasing_timestamp_is_rejected():
    f = ComplementaryFilter(alpha=1.0)
    assert f.update(1.0, (0.0, 0.0, G), (0.0, 0.0, 0.0))
    assert f.update(1.01, (0.0, 0.0, G), (1.0, 0.0, 0.0))
    roll = f.roll
    assert not f.update(1.01, (0.0, 0.0, G), (1.0, 0.0, 0.0))  # dt == 0
    assert not f.update(1.00, (0.0, 0.0, G), (1.0, 0.0, 0.0))  # dt < 0
    assert f.roll == roll
    assert f.rejected_samples == 2
    assert f.update(1.02, (0.0, 0.0, G), (1.0, 0.0, 0.0))  # dt measured from 1.01
    assert f.roll == pytest.approx(roll + 0.01)


def test_large_gap_reinitialises_from_accel_instead_of_integrating():
    f = ComplementaryFilter(alpha=1.0, max_dt=0.5)
    f.update(0.0, (0.0, 0.0, G), (0.0, 0.0, 0.0))
    # 10 s gap with a big gyro rate: integrating would give 10 rad of roll.
    assert f.update(10.0, gravity_in_body(0.3, 0.0), (1.0, 0.0, 0.0))
    assert f.roll == pytest.approx(0.3)
    assert f.resets == 1


def test_accel_gate_ignores_accelerometer_while_accelerating():
    f = ComplementaryFilter(alpha=0.5, accel_gate=0.2)
    f.update(0.0, (0.0, 0.0, G), (0.0, 0.0, 0.0))
    f.update(DT, (2 * G, 0.0, G), (0.0, 0.0, 0.0))  # |a| ~ 2.2 g: not gravity alone
    assert f.pitch == pytest.approx(0.0)


def test_roll_blend_is_correct_across_pi():
    f = ComplementaryFilter(alpha=0.5)
    f.update(0.0, gravity_in_body(math.pi - 0.05, 0.0), (0.0, 0.0, 0.0))
    f.update(DT, gravity_in_body(-math.pi + 0.05, 0.0), (0.0, 0.0, 0.0))
    # Halfway between pi-0.05 and -pi+0.05 the short way round is pi, not 0.
    assert abs(wrap_angle(f.roll - math.pi)) == pytest.approx(0.0, abs=1e-9)


def test_invalid_alpha_raises():
    with pytest.raises(ValueError):
        ComplementaryFilter(alpha=1.5)
