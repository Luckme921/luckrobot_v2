"""Read-only scan support audit; close returns are not automatically noise."""
import sqlite3,sys,math,json
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
t0=db.execute('select min(timestamp) from messages').fetchone()[0]
out={}
for tid,name in db.execute("select id,name from topics where type='sensor_msgs/msg/LaserScan'"):
    frames=near=sparse=0; nearest=99.; maximum=0; gaps=[];last=None
    for stamp,blob in db.execute('select timestamp,data from messages where topic_id=? and timestamp<? order by timestamp',(tid,t0+60_000_000_000)):
        m=deserialize_message(blob,LaserScan);frames+=1
        if last is not None:gaps.append((stamp-last)/1e9)
        last=stamp
        count=0
        for r in m.ranges:
            if math.isfinite(r) and m.range_min<=r<=m.range_max:
                nearest=min(nearest,r)
                count+=r<.45
        near+=count>0;sparse+=0<count<=2;maximum=max(maximum,count)
    out[name]=dict(frames=frames,near_frames=near,only_1_or_2_close_beams=sparse,
                   nearest_m=nearest,max_close_beams=maximum,max_arrival_gap_s=max(gaps,default=0))
print(json.dumps(out,indent=2))
