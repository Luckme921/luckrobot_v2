"""Deterministic safety policy tests; no ROS, network or motor access."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/my_robot_bringup/scripts'))
from motion_freshness_core import FreshnessGate, RecoveryRamp, ramp_command, limit_twist


class Tests(unittest.TestCase):
    def test_recovery_does_not_filter_normal_steering(self):
        ramp = RecoveryRamp()
        previous = (0., 0.)
        for _ in range(20):
            previous = ramp.apply(previous, (.1, .05), .05)
        self.assertFalse(ramp.active)
        # Upstream-smoothed left/right corrections must not insert a full stop.
        self.assertEqual(ramp.apply((.1,.005),(.1,-.005),.05),(.1,-.005))
        self.assertEqual(ramp.apply((.1,-.005),(.01,0.),.05),(.01,0.))
        ramp.reset()
        self.assertEqual(ramp.apply((0.,0.),(0.,0.),.05),(0.,0.))
        self.assertTrue(ramp.active)
        self.assertLess(ramp.apply((0.,0.),(.1,.05),.05)[0],.02)

    def test_speed_caps_preserve_curvature(self):
        self.assertEqual(limit_twist(.3,.2,.15,.1,.35),(.15,.1))
        self.assertEqual(limit_twist(-.2,.4,.3,.1,.35),(-.1,.2))
        self.assertEqual(limit_twist(.1,.7,.3,.1,.35),(.05,.35))

    def fresh(self, gate, ros, steady):
        for key in gate.limits:
            gate.observe(key, ros, steady)
        return gate.check(ros, steady)

    def test_missing_source_stops(self):
        self.assertFalse(FreshnessGate({'camera': .35}).check(100., 1.)[0])

    def test_old_repeated_messages_do_not_refresh(self):
        g = FreshnessGate({'camera': .35}, recovery_time=0.)
        self.assertTrue(self.fresh(g, 100., 1.)[0])
        g.observe('camera', 100., 1.5)
        self.assertFalse(g.check(100.5, 1.5)[0])

    def test_out_of_order_ignored(self):
        g = FreshnessGate({'camera': .35}, recovery_time=0.)
        self.fresh(g, 100., 1.)
        g.observe('camera', 99., 1.1)
        self.assertEqual(g.samples['camera'], (100., 1.))

    def test_steady_timeout_with_ros_clock_paused(self):
        g = FreshnessGate({'camera': .35}, recovery_time=0.)
        self.fresh(g, 100., 1.)
        self.assertFalse(g.check(100., 1.5)[0])

    def test_continuous_recovery_required(self):
        g = FreshnessGate({'camera': .35})
        self.assertFalse(self.fresh(g, 100., 1.)[0])
        self.assertFalse(self.fresh(g, 100.3, 1.3)[0])
        self.assertTrue(self.fresh(g, 100.6, 1.6)[0])
        self.assertFalse(g.check(101., 2.)[0])
        self.assertFalse(self.fresh(g, 101.1, 2.1)[0])

    def test_future_and_clock_reset(self):
        g = FreshnessGate({'camera': .35}, recovery_time=0.)
        g.observe('camera', 110., 1.)
        self.assertFalse(g.check(100., 1.)[0])
        self.assertFalse(g.check(90., 1.1)[0])
        self.assertEqual(g.samples, {})
        self.assertTrue(self.fresh(g, 90.1, 1.2)[0])

    def test_ramp_keeps_arc_ratio(self):
        cmd = (0., 0.)
        for _ in range(30):
            prev = cmd
            cmd = ramp_command(cmd, (.30, .15), .05)
            self.assertAlmostEqual(cmd[1] / cmd[0], .5)
            self.assertLessEqual(cmd[0] - prev[0], .01000001)
        self.assertAlmostEqual(cmd[0], .30)

    def test_stop_and_nan_not_smoothed(self):
        self.assertEqual(ramp_command((.3, .2), (0., 0.), .05), (0., 0.))
        self.assertEqual(ramp_command((.3, .2), (float('nan'), .2), .05), (0., 0.))

    def test_delayed_callback_cannot_jump_to_target(self):
        v, w = ramp_command((0., 0.), (.3, .35), 3.)
        self.assertLessEqual(v, .020001)
        self.assertLessEqual(w, .030001)

    def test_collision_reductions_never_delayed(self):
        for target in ((.01,.01), (.0,.10), (.10,0.), (.2,.30), (-.1,.1)):
            output = ramp_command((.3,.2), target, .05)
            for actual, safe in zip(output,target):
                self.assertLessEqual(abs(actual),abs(safe))
                self.assertGreaterEqual(actual*safe,0.)
        self.assertEqual(ramp_command((.3,.2),(.01,.01),.05),(.01,.01))

    def test_unsafe_limits_rejected(self):
        for value in (0., -1., float('nan')):
            with self.assertRaises(ValueError):
                FreshnessGate({'camera': value})


if __name__ == '__main__':
    unittest.main()
