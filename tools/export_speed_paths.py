import sqlite3,sys
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path
db=sqlite3.connect('file:'+sys.argv[1]+'?mode=ro',uri=True)
tid=db.execute("select id from topics where name='/plan'").fetchone()[0]
seen=set(); paths=[]
for blob, in db.execute('select data from messages where topic_id=? order by timestamp',(tid,)):
    msg=deserialize_message(blob,Path)
    points=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
    key=tuple((round(x,3),round(y,3)) for x,y in points)
    if len(points)<3 or key in seen:continue
    seen.add(key);paths.append(points)
with open(sys.argv[2],'w') as f:
    f.write(str(len(paths))+'\n')
    for points in paths:
        f.write(str(len(points))+'\n')
        for x,y in points:f.write(f'{x} {y}\n')
print('exported',len(paths),'path messages; includes cached truncations')
