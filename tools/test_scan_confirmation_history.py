"""Exercise production filter methods without ROS; run with numpy installed."""
import ast
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import numpy as np
import yaml

SOURCE = Path(__file__).resolve().parents[1] / 'src/my_robot_bringup/scripts/scan_confirmation_filter.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
           {'on_scan', 'remove_small_clusters', 'seen_in_frame', 'lookup_transform'}]
class TransformException(Exception): pass
class Time:
    @staticmethod
    def from_msg(stamp): return stamp
ns = {'np': np, 'math': math, 'TransformException': TransformException,
      'Time': Time, 'Duration': lambda **kwargs: kwargs}
exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), ns)

class Harness:
    on_scan = ns['on_scan']
    remove_small_clusters = ns['remove_small_clusters']
    seen_in_frame = ns['seen_in_frame']
    def __init__(self):
        self.history = deque()
        self.min_cluster_beams = 7
        self.cluster_range_tolerance = .08
        self.confirmation_frames = 5
        self.history_duration = .5
        self.neighbor_radius = 1
        self.tick = 0
        self.max_scan_age = .35
        self.last_scan_ns = -1
        for k in ['frames_received','frames_published','last_input_beams','input_beams',
                  'last_confirmed_beams','confirmed_beams']:setattr(self,k,0)
        self.publisher = NS(publish=lambda m: setattr(self,'output',m))
        self.clearing_publisher = NS(publish=lambda m: setattr(self,'clearing',m))
    def empty_copy(self,m):
        return NS(header=m.header,ranges=[math.inf]*len(m.ranges),intensities=[])
    def lookup_transform(self,*args):return NS(),False
    def fixed_cells(self,m,ids,ranges,transform):return [(25,0)]*len(ids)
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=self.tick))
    def feed(self,ranges):
        self.tick += 66_666_667
        self.on_scan(NS(ranges=ranges,range_min=.1,range_max=3.,
                        header=NS(frame_id='base_footprint',stamp=NS(
                            sec=self.tick//1_000_000_000, nanosec=self.tick%1_000_000_000)),intensities=[]))
        return int(np.isfinite(self.output.ranges).sum())

class Tests(unittest.TestCase):
    def test_costmaps_use_clear_only_finite_channel(self):
        params=yaml.safe_load((SOURCE.parents[2]/'my_robot_navigation/config/nav2_params.yaml').read_text())
        for name in ('global_costmap','local_costmap'):
            layer=params[name][name]['ros__parameters']['camera_obstacle_layer']
            self.assertIn('camera_clear',layer['observation_sources'].split())
            clear=layer['camera_clear']
            self.assertTrue(clear['clearing'])
            self.assertFalse(clear['marking'])
            self.assertFalse(clear['inf_is_valid'])
            self.assertEqual(clear['observation_persistence'],0.0)
            self.assertEqual(clear['topic'],'/scan_camera_clear')
            self.assertTrue(layer['camera']['marking'])

    def test_finite_background_clears_without_temporal_marking(self):
        h=Harness()
        self.assertEqual(h.feed([2.]*12),0)
        self.assertEqual(list(h.clearing.ranges),[2.]*12)
        for _ in range(3):self.assertEqual(h.feed([2.]*12),0)
        self.assertEqual(h.feed([2.]*12),12)

    def test_no_depth_or_small_clusters_never_generate_clearing(self):
        h=Harness()
        h.feed([math.inf]*12)
        self.assertFalse(np.isfinite(h.clearing.ranges).any())
        h.feed([1.]*3)
        self.assertFalse(np.isfinite(h.clearing.ranges).any())

    def test_missing_transform_never_publishes_clearing(self):
        h=Harness()
        h.feed([math.inf]*12)
        h.lookup_transform=lambda *args:(None,False)
        h.feed([2.]*12)
        self.assertFalse(np.isfinite(h.clearing.ranges).any())

    def test_latest_tf_fallback_is_bounded(self):
        for skew, expected in ((.10,True), (.21,False), (5.,False)):
            h=Harness()
            h.fixed_frame='odom'
            h.max_tf_skew=.2
            h.frames_latest_tf=h.frames_without_tf=0
            h.get_logger=lambda:NS(warning=lambda m:None)
            def lookup(*args,**kwargs):
                if isinstance(args[2], NS):
                    raise TransformException('exact unavailable')
                total=int((10.-skew)*1e9)
                return NS(header=NS(stamp=NS(sec=total//1_000_000_000,
                                            nanosec=total%1_000_000_000)))
            h.tf_buffer=NS(lookup_transform=lookup)
            result, used=ns['lookup_transform'](h,'base_footprint',NS(sec=10,nanosec=0))
            self.assertEqual(result is not None,expected)

    def test_small_speckles_cannot_seed_confirmation(self):
        h=Harness()
        for _ in range(4):self.assertEqual(h.feed([1.]*3),0)
        self.assertEqual(len(h.history),0)
        self.assertEqual(h.feed([1.]*7),0)
        for _ in range(3):self.assertEqual(h.feed([1.]*7),0)
        self.assertEqual(h.feed([1.]*7),7)
    def test_real_surface_confirms_on_fifth_frame(self):
        h=Harness()
        for _ in range(4):self.assertEqual(h.feed([1.]*12),0)
        self.assertEqual(h.feed([1.]*12),12)
    def test_range_jump_cannot_join_small_clusters(self):
        h=Harness()
        for _ in range(8):self.assertEqual(h.feed([.5]*4+[1.]*4),0)
    def test_expired_history_not_reused(self):
        h=Harness()
        for _ in range(5):h.feed([1.]*7)
        h.tick += 1_000_000_000
        self.assertEqual(h.feed([1.]*7),0)
    def test_empty_scan_is_published(self):
        h=Harness()
        self.assertEqual(h.feed([math.inf]*10),0)
        self.assertEqual(h.frames_published,1)

    def test_missing_tf_does_not_publish_free_space(self):
        h=Harness()
        h.lookup_transform=lambda *args:(None,False)
        h.feed([math.inf]*10)  # initialize output
        before=h.frames_published
        h.feed([1.]*12)
        self.assertEqual(h.frames_published,before)
        self.assertEqual(len(h.history),0)

    def test_stale_input_is_dropped(self):
        h=Harness()
        h.tick=2_000_000_000
        h.on_scan(NS(header=NS(stamp=NS(sec=1,nanosec=0))))
        self.assertEqual(h.frames_published,0)

if __name__=='__main__':unittest.main()
