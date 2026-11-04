# Checks on simulated odometry, without ROS, used by odom_relay.py (and the tests).
import math

# Variance put on odometry that reports none (Gazebo's ground truth has an all-zero covariance, which
# makes RTAB-Map trust it absolutely and warn on every message).
DEFAULT_LINEAR_VARIANCE = 1e-4   # (1 cm)^2
DEFAULT_ANGULAR_VARIANCE = 1e-4  # (~0.6 deg)^2


def odometry_problem(position, orientation, stamp, last=None, max_speed=20.0):
    """Why an odometry sample must be dropped, or None if it is usable.

    position (x, y, z), orientation quaternion (x, y, z, w), stamp in seconds.
    last is the previous accepted (position, stamp), to catch time going backwards and teleport jumps
    faster than max_speed (m/s).
    """
    values = tuple(position) + tuple(orientation) + (stamp,)
    if not all(math.isfinite(v) for v in values):
        return 'NaN or infinite value'
    norm = math.sqrt(sum(q * q for q in orientation))
    if abs(norm - 1.0) > 0.01:
        return f'orientation is not a unit quaternion (norm {norm:.3f})'
    if last is not None:
        last_position, last_stamp = last
        dt = stamp - last_stamp
        if dt < 0:
            return f'timestamp went backwards by {-dt:.3f} s'
        jump = math.dist(position, last_position)
        # 5 cm of slack for samples that share a timestamp
        if jump > max_speed * dt + 0.05:
            return f'position jumped {jump:.2f} m in {dt:.3f} s'
    return None


def with_covariance(covariance, linear=DEFAULT_LINEAR_VARIANCE, angular=DEFAULT_ANGULAR_VARIANCE):
    """The 6x6 row-major covariance, with zero (or invalid) diagonal entries replaced by a usable variance."""
    cov = [float(c) for c in covariance]
    if len(cov) != 36:
        cov = [0.0] * 36
    for i in range(6):
        d = cov[i * 7]
        if not (math.isfinite(d) and d > 0):
            cov[i * 7] = linear if i < 3 else angular
    return cov


class Staleness:
    """Tracks whether a stream has gone silent; report() returns a message only when the state changes."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.last = None
        self.silent = False

    def seen(self, now):
        self.last = now
        if self.silent:
            self.silent = False
            return 'resumed'
        return None

    def report(self, now):
        if self.last is None:
            # Never received: report once timeout has passed since the first check.
            self.last = now
            return None
        if not self.silent and now - self.last > self.timeout:
            self.silent = True
            return f'no data for {now - self.last:.1f} s'
        return None
