"""Pure, clock-injected safety policy. No ROS or motor access."""
import math


def limit_twist(v, w, forward, reverse, angular):
    """Cap both axes by one factor; never change the requested turning radius."""
    scale = 1.0
    if abs(v) > 0:
        scale = min(scale, (forward if v > 0 else reverse) / abs(v))
    if abs(w) > 0:
        scale = min(scale, angular / abs(w))
    return (v * scale, w * scale)


class FreshnessGate:
    def __init__(self, limits, recovery_time=0.5, future_tolerance=0.3):
        if not limits or any(not math.isfinite(x) or x <= 0 for x in limits.values()):
            raise ValueError('Freshness limits must be finite and positive')
        self.limits = dict(limits)
        self.recovery_time = recovery_time
        self.future_tolerance = future_tolerance
        self.samples = {}
        self.healthy_since = None
        self.last_ros_time = None

    def observe(self, key, stamp, received):
        if key not in self.limits or not math.isfinite(stamp) or stamp <= 0:
            return
        previous = self.samples.get(key)
        # Repeated/frozen/reordered timestamps cannot feed the watchdog.
        if previous is None or stamp > previous[0]:
            self.samples[key] = (stamp, received)

    def check(self, now_ros, now_steady):
        if self.last_ros_time is not None and now_ros < self.last_ros_time - 0.01:
            self.samples.clear()
            self.healthy_since = None
        self.last_ros_time = now_ros
        for key, limit in self.limits.items():
            if key not in self.samples:
                self.healthy_since = None
                return False, key + ': missing'
            stamp, receipt = self.samples[key]
            age, silence = now_ros - stamp, now_steady - receipt
            if age < -self.future_tolerance or age > limit or silence > limit or silence < 0:
                self.healthy_since = None
                return False, '%s: age=%.3f silence=%.3f' % (key, age, silence)
        if self.healthy_since is None:
            self.healthy_since = now_steady
        if now_steady - self.healthy_since < self.recovery_time:
            return False, 'recovering: waiting for continuous fresh data'
        return True, 'ready'


def ramp_command(previous, target, dt, accel=(0.20, 0.30)):
    """Limit recovery acceleration, NEVER delay Collision Monitor braking.

    Output is a nonnegative scalar multiple (<= 1) of the safe input. A
    downstream gate must not smooth a requested reduction into a larger speed.
    Normal acceleration/deceleration smoothing belongs BEFORE collision checks.
    """
    if not all(math.isfinite(x) for x in target) or target == (0.0, 0.0):
        return (0.0, 0.0)
    dt = max(0.0, min(dt, 0.1))
    scale = 1.0
    for i in range(2):
        if target[i] * previous[i] < 0:
            return (0.0, 0.0)  # restart from zero on an unsmoothed reversal
        if abs(target[i]) > abs(previous[i]):
            scale = min(scale, (abs(previous[i]) + accel[i] * dt) / abs(target[i]))
    return tuple(scale * value for value in target)


class RecoveryRamp:
    """Ramp only after a safety outage, not every normal steering correction.

    The upstream velocity smoother owns normal motion. Re-applying a ramp on
    every command could insert a full stop when angular velocity changes sign.
    """
    def __init__(self):
        self.active = True

    def reset(self):
        self.active = True

    def apply(self, previous, target, dt):
        if not self.active:
            return target
        output = ramp_command(previous, target, dt)
        # Idle zero commands must not bypass the next nonzero recovery ramp.
        if target != (0.0, 0.0) and output == target:
            self.active = False
        return output
