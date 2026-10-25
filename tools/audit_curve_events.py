"""Summarize recorded optimizer decisions, without replaying robot commands."""
import sqlite3, sys, collections, json, re
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import Log
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
row=db.execute("select id from topics where name='/rosout'").fetchone()
events=[]; warnings=collections.Counter()
if row:
    for blob, in db.execute('select data from messages where topic_id=? order by timestamp',row):
        m=deserialize_message(blob,Log)
        if 'Curve optimization' in m.msg:
            events.append(dict(time=m.stamp.sec+m.stamp.nanosec/1e9,message=m.msg))
        if m.level>=30: warnings[(m.name,m.msg)]+=1
print(json.dumps({'curve_events':events,'warnings':[(n,s,c) for (n,s),c in warnings.most_common(12)]},indent=2))
