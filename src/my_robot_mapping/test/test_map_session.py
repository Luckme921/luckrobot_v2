"""Run with ROS sourced; no hardware, topics, or motion commands are created."""
import json
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_versions import MapVersions, write_grid
from map_session_manager import Manager, Interrupted


def artifacts(store):
    prefix = store.begin()
    prefix.with_suffix('.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    prefix.with_suffix('.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\x00\xff')
    prefix.with_suffix('.data').write_bytes(b'test data')
    prefix.with_suffix('.posegraph').write_bytes(b'test graph')
    return prefix


class VersionsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.store = MapVersions(self.tmp.name)

    def commit(self, complete=True):
        return self.store.commit(artifacts(self.store), complete=complete,
                                 session='test', pose=[1, 2, .3], reason='test')

    def test_complete_roundtrip(self):
        self.assertEqual(self.commit(), self.store.current())

    def test_grid_orientation_and_unknown(self):
        prefix = self.store.begin()
        write_grid(prefix, width=2, height=2, resolution=.05, origin=[1, 2, .3],
                   data=[0, 100, -1, 50])
        self.assertEqual(prefix.with_suffix('.pgm').read_bytes(), b'P5\n2 2\n255\n'+bytes([205,205,254,0]))

    def test_bad_grid_rejected(self):
        with self.assertRaises(ValueError):
            write_grid(self.store.begin(), width=2, height=2, resolution=.05, origin=[0,0,0], data=[0])

    def test_partial_never_promotes(self):
        self.commit(False)
        self.assertIsNone(self.store.current())

    def test_partial_preserves_previous(self):
        old = self.commit()
        self.commit(False)
        self.assertEqual(old, self.store.current())

    def test_incomplete_write_preserves_previous(self):
        old = self.commit()
        prefix = artifacts(self.store)
        prefix.with_suffix('.data').unlink()
        with self.assertRaises(ValueError):
            self.store.commit(prefix, complete=True, session='test', pose=[0, 0, 0], reason='test')
        self.assertEqual(old, self.store.current())

    def test_tamper_rejected(self):
        record = self.commit()
        Path(record['prefix']).with_suffix('.data').write_text('changed')
        with self.assertRaises(ValueError):
            self.store.current()

    def test_bad_pose_rejected(self):
        with self.assertRaises(ValueError):
            self.store.commit(artifacts(self.store), complete=True, session='x', pose=[0, float('nan'), 0], reason='x')
        self.assertIsNone(self.store.current())

    def test_image_escape_rejected(self):
        prefix = artifacts(self.store)
        prefix.with_suffix('.yaml').write_text('image: ../../outside.pgm\nresolution: .05\norigin: [0,0,0]\n')
        with self.assertRaises(ValueError):
            self.store.validate(prefix)


def manager():
    # Exercise the production methods without initializing a ROS node.
    m = Manager.__new__(Manager)
    m.state = 'EXPLORING'
    m.mode = 'mapping'
    m.owns_session = True
    m.record = None
    m.seed = None
    m.jobs = queue.Queue()
    m.interrupt = threading.Event()
    m.shutdown_event = threading.Event()
    m.busy = False
    m.children = {'explore': Mock()}
    m.accept_explore_status = True
    m.explorer_started = True
    m.retries = 0
    m.retry_at = None
    m.get_parameter = lambda name: S(value={'blocked_retry_limit': 3, 'blocked_retry_seconds': 30.0}[name])
    m.transition = Mock(side_effect=lambda state, reason: setattr(m, 'state', state))
    m.hold = Mock()
    m.explore = Mock()
    m.localize = Mock()
    m.save = Mock(return_value=({'complete': True}, [0, 0, 0]))
    return m


class SessionTest(unittest.TestCase):
    def test_manual_mapping_keeps_existing_map(self):
        m=manager();m.state='PAUSED';m.nav=Mock();m.start_slam=Mock()
        m.execute('manual_mapping')
        m.start_slam.assert_not_called();m.nav.assert_called_once_with(True)
        self.assertEqual(m.state,'MAPPING');m.explore.assert_not_called()

    def test_auto_mapping_starts_from_idle(self):
        m=manager();m.mode=None;m.state='IDLE';m.nav=Mock();m.start_slam=Mock()
        m.execute('auto_mapping')
        m.start_slam.assert_called_once_with('mapping');m.explore.assert_called_once()

    def test_client_finish_saves_then_localizes(self):
        m=manager();m.state='MAPPING';m.execute('finish_mapping')
        m.save.assert_called_once_with(True,'mapping_finished_by_client')
        m.localize.assert_called_once()

    def test_cannot_finish_while_exploring(self):
        m=manager();r=S(success=False,message='');m.command('finish_mapping',r)
        self.assertFalse(r.success);self.assertTrue(m.jobs.empty())

    def test_completion_saves_then_localizes(self):
        m = manager()
        m.execute('complete')
        m.save.assert_called_once_with(True, 'exploration_complete')
        m.localize.assert_called_once_with({'complete': True}, [0, 0, 0])

    def test_failed_save_cannot_start_localization(self):
        m = manager()
        m.save.side_effect = RuntimeError('disk full')
        with self.assertRaises(RuntimeError):
            m.execute('complete')
        m.localize.assert_not_called()

    def test_blocked_is_not_pause(self):
        m = manager()
        m.explore_status(S(status='exploration_blocked:no_safe_viewpoint'))
        self.assertEqual(m.state, 'BLOCKED')
        self.assertTrue(m.jobs.empty())
        self.assertGreater(m.retry_at, time.monotonic())

    def test_retry_keeps_branch_memory(self):
        m = manager()
        m.state = 'BLOCKED'
        m.execute('retry')
        m.hold.assert_called_once_with(keep_explorer=True)
        self.assertEqual(m.retries, 1)
        m.explore.assert_called_once()

    def test_exhaustion_is_incomplete_not_complete(self):
        m = manager()
        m.state, m.retries = 'BLOCKED', 3
        m.execute('retry')
        m.save.assert_called_once_with(False, 'blocked_retry_budget_exhausted')
        self.assertEqual(m.state, 'INCOMPLETE')
        m.localize.assert_not_called()

    def test_explicit_pause_does_not_auto_resume(self):
        m = manager()
        m.execute('pause')
        self.assertEqual(m.state, 'PAUSED')
        m.explore.assert_not_called()
        m.localize.assert_not_called()

    def test_cancel_only_checkpoints(self):
        m = manager()
        m.execute('cancel')
        m.save.assert_called_once_with(False, 'cancelled_by_client')
        self.assertEqual(m.state, 'CANCELLED')
        m.localize.assert_not_called()

    def test_stale_completion_ignored(self):
        m = manager()
        m.accept_explore_status = False
        m.explore_status(S(status='exploration_complete'))
        self.assertTrue(m.jobs.empty())

    def test_completion_before_start_ignored(self):
        m = manager()
        m.explorer_started = False
        m.explore_status(S(status='exploration_complete'))
        self.assertTrue(m.jobs.empty())

    def test_pause_discards_pending_completion(self):
        m = manager()
        m.jobs.put('complete')
        response = m.command('pause', S())
        self.assertTrue(response.success)
        self.assertEqual(m.jobs.get_nowait(), 'pause')
        self.assertTrue(m.jobs.empty())
        self.assertTrue(m.interrupt.is_set())

    def test_bad_command_does_not_mutate_session(self):
        m = manager()
        self.assertFalse(m.command('update_map', S()).success)
        self.assertTrue(m.jobs.empty())
        self.assertFalse(m.interrupt.is_set())

    def test_no_seed_never_uses_saved_shutdown_pose(self):
        m = manager()
        m.store = S(current=lambda: {'pose_at_save': [10, 10, 1]})
        m.execute('load_latest')
        self.assertEqual(m.state, 'WAITING_INITIAL_POSE')
        m.localize.assert_not_called()

    def test_inactive_navigation_hold_is_idempotent(self):
        m = manager()
        m.stop_child = Mock()
        m.nav_states = lambda: [2]*5
        m.nav = Mock()
        m.call = Mock()
        m.last_odom = (S(header=S(stamp=S(sec=0, nanosec=0)),
                         twist=S(twist=S(linear=S(x=0.), angular=S(z=0.)))), time.monotonic())
        m.get_clock = lambda: S(now=lambda: S(nanoseconds=0))
        # Refresh arrival as a sensor callback would.
        def refresh(_):
            m.last_odom = (m.last_odom[0], time.monotonic())
        with patch('map_session_manager.time.sleep', side_effect=refresh):
            Manager.hold(m)
            Manager.hold(m)
        m.call.assert_not_called()
        self.assertEqual(m.nav.call_count, 2)

    def test_interrupt_prevents_completion_save(self):
        m = manager()
        m.interrupt.set()
        with self.assertRaises(Interrupted):
            m.execute('complete')
        m.save.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
