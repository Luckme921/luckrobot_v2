"""Read-only bag audit. Recorded samples are not a closed-loop candidate test."""
import json
import sqlite3
import sys
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray

db = sqlite3.connect('file:' + sys.argv[1] + '?mode=ro', uri=True)
tid = db.execute("select id from topics where name='/luck_controller/tracking_error'").fetchone()[0]
a = np.asarray([deserialize_message(b, Float64MultiArray).data for (b,) in
                db.execute('select data from messages where topic_id=? order by timestamp', (tid,))])
report = []
for lo, hi in ((0.2, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 4.0)):
    sel = ((a[:, 7] == 0) & (a[:, 11] == 0) & (a[:, 0] > .8) &
           (a[:, 4] > .02) & (abs(a[:, 3]) >= lo) & (abs(a[:, 3]) < hi))
    x = a[sel]
    report.append({'curvature': [lo, hi], 'n': len(x), **({
        'command_v_p50_p90': np.quantile(x[:, 4], [.5, .9]).tolist(),
        'cross_abs_p50_p90': np.quantile(abs(x[:, 2]), [.5, .9]).tolist(),
        'yaw_command_abs_p90': float(np.quantile(abs(x[:, 5]), .9)),
        'yaw_execution_error_abs_p90': float(np.quantile(abs(x[:, 24]-x[:, 5]), .9)),
        'raw_curvature_scale_ratio_p50': float(np.median(
            np.clip(1/(1+.85*abs(x[:, 3])), .30, 1) /
            np.clip(1/(1+abs(x[:, 3])), .30, 1)))
    } if len(x) else {})})
print(json.dumps(report, indent=2))
