#!/usr/bin/env python3
"""Read-only bag timing audit. Arrival times for unstamped Twist are approximate."""
import collections
import json
import math
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

db = sqlite3.connect('file:' + sys.argv[1] + '?mode=ro', uri=True)
topics = {i: (n, get_message(t)) for i, n, t in db.execute('select id,name,type from topics')}
data = collections.defaultdict(list)
tracking = []
for tid, ns, blob in db.execute('select topic_id,timestamp,data from messages order by timestamp'):
    name, cls = topics[tid]
    if name not in ('/cmd_vel_nav', '/cmd_vel', '/cmd_vel_safe', '/cmd_vel_drive',
                    '/wheel/odom', '/odometry/filtered', '/imu/data', '/luck_controller/tracking_error'):
        continue
    m = deserialize_message(blob, cls)
    t = ns / 1e9
    if name == '/luck_controller/tracking_error':
        tracking.append([t] + list(m.data))
        continue
    if name == '/imu/data':
        # Verified inverted imu_link from URDF: body Z = -raw Z.
        v, w = 0., -m.angular_velocity.z
    elif name.endswith('odom') or name.endswith('filtered'):
        v, w = m.twist.twist.linear.x, m.twist.twist.angular.z
    else:
        v, w = m.linear.x, m.angular.z
    data[name].append((t, v, w))
arrays = {k: np.asarray(v) for k, v in data.items()}
begin = max(v[0, 0] for v in arrays.values())
end = min(v[-1, 0] for v in arrays.values())
grid = np.arange(begin, end, .02)
sampled = {}
for k, a in arrays.items():
    idx = np.clip(np.searchsorted(a[:, 0], grid, side='right') - 1, 0, len(a)-1)
    sampled[k] = a[idx, 1:]
    sampled[k][grid - a[idx, 0] > .25] = np.nan

def stats(vals):
    vals = np.asarray(vals)
    vals = vals[np.isfinite(vals)]
    return list(map(float, np.quantile(vals, [.5, .9, .99, 1]))) if len(vals) else []

def lag_report(source, dest, axis):
    x, y = sampled[source][:, axis], sampled[dest][:, axis]
    results = []
    for lag in range(76):
        a, b = (x, y) if lag == 0 else (x[:-lag], y[lag:])
        valid = np.isfinite(a) & np.isfinite(b) & ((np.abs(a) > .025) | (np.abs(b) > .025))
        if np.sum(valid) < 100: continue
        results.append((float(np.sqrt(np.mean((a[valid]-b[valid])**2))), lag*.02))
    return {'best_rmse_lag_s': min(results) if results else None,
            'zero_lag': results[0] if results else None}

report = {'duration': end-begin, 'quantile_order': ['p50', 'p90', 'p99', 'max'],
          'note': 'Arrival-time comparison, not a causal hardware latency identification.',
          'topics': {}, 'lag': {}}
for k, a in arrays.items():
    report['topics'][k] = {'count': len(a), 'gap_s': stats(np.diff(a[:, 0])),
                           'abs_v': stats(np.abs(a[:, 1])), 'abs_w': stats(np.abs(a[:, 2]))}
for a,b in [('/cmd_vel_nav','/cmd_vel'),('/cmd_vel','/cmd_vel_drive'),
            ('/cmd_vel_drive','/wheel/odom'),('/cmd_vel_drive','/imu/data'),
            ('/imu/data','/odometry/filtered'),('/wheel/odom','/odometry/filtered')]:
    for axis in (0,1):
        if '/imu/data' in (a,b) and axis == 0: continue
        report['lag'][a+' -> '+b+(' v' if axis==0 else ' w')] = lag_report(a,b,axis)
if tracking:
    a=np.asarray(tracking)
    report['tracking_width']=a.shape[1]-1
    report['tracking_abs_error']=stats(np.abs(a[:,3]))
    report['tracking_modes']=dict(collections.Counter(map(int,a[:,12])))
    ordinary = (a[:,8] == 0) & (a[:,12] == 0) & (a[:,5] > .02)
    report['ordinary_heading_abs_rad']=stats(np.abs(a[ordinary,2]))
    report['ordinary_cross_abs_m']=stats(np.abs(a[ordinary,3]))
print(json.dumps(report, indent=2, allow_nan=False))
