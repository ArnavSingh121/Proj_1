# Invalid or missing simulated odometry is detected and handled (issues #12, #14).
import math

import sensor_checks
from sensor_checks import Staleness, odometry_problem, with_covariance

LEVEL = (0.0, 0.0, 0.0, 1.0)  # identity quaternion


def test_valid_odometry_is_accepted():
    assert odometry_problem((1.0, 2.0, 3.0), LEVEL, 10.0) is None
    assert odometry_problem((1.1, 2.0, 3.0), LEVEL, 10.1, last=((1.0, 2.0, 3.0), 10.0)) is None


def test_nan_and_infinite_values_are_rejected():
    assert 'NaN' in odometry_problem((math.nan, 0.0, 0.0), LEVEL, 1.0)
    assert 'NaN' in odometry_problem((0.0, 0.0, 0.0), (0.0, 0.0, math.inf, 1.0), 1.0)
    assert 'NaN' in odometry_problem((0.0, 0.0, 0.0), LEVEL, math.nan)


def test_non_unit_quaternion_is_rejected():
    assert 'quaternion' in odometry_problem((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), 1.0)
    assert 'quaternion' in odometry_problem((0.0, 0.0, 0.0), (0.0, 0.0, 0.5, 1.0), 1.0)
    s = math.sqrt(0.5)
    assert odometry_problem((0.0, 0.0, 0.0), (0.0, 0.0, s, s), 1.0) is None  # 90 deg yaw


def test_time_going_backwards_is_rejected():
    assert 'backwards' in odometry_problem((0.0, 0.0, 0.0), LEVEL, 9.0, last=((0.0, 0.0, 0.0), 10.0))


def test_teleport_jump_is_rejected():
    last = ((0.0, 0.0, 0.0), 10.0)
    assert 'jumped' in odometry_problem((50.0, 0.0, 0.0), LEVEL, 10.1, last=last, max_speed=20.0)
    assert odometry_problem((1.9, 0.0, 0.0), LEVEL, 10.1, last=last, max_speed=20.0) is None


def test_zero_covariance_gets_a_usable_variance():
    cov = with_covariance([0.0] * 36)
    diag = [cov[i * 7] for i in range(6)]
    assert diag[:3] == [sensor_checks.DEFAULT_LINEAR_VARIANCE] * 3
    assert diag[3:] == [sensor_checks.DEFAULT_ANGULAR_VARIANCE] * 3
    assert sum(cov) == sum(diag)  # off-diagonal untouched


def test_reported_covariance_is_kept_and_bad_entries_replaced():
    cov = [0.0] * 36
    cov[0], cov[7], cov[14] = 0.5, math.nan, -1.0
    out = with_covariance(cov)
    assert out[0] == 0.5
    assert out[7] == out[14] == sensor_checks.DEFAULT_LINEAR_VARIANCE
    assert len(with_covariance([1.0, 2.0])) == 36  # malformed length


def test_staleness_reports_silence_once_and_recovery():
    s = Staleness(timeout=1.0)
    assert s.report(0.0) is None  # nothing received yet: starts the clock
    assert s.report(0.5) is None
    assert 'no data' in s.report(1.6)
    assert s.report(5.0) is None  # only once
    assert s.seen(5.1) == 'resumed'
    assert s.seen(5.2) is None
    assert s.report(5.5) is None
