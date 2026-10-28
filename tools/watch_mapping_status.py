#!/usr/bin/env python3
"""Display managed mapping transitions without starting or stopping anything."""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
LABELS = {'MAPPING':'建图已启动，等待开始探索', 'EXPLORING':'正在探索',
'BLOCKED':'探索受阻：保留未完成区域，等待自动重试',
'EXPLORATION_COMPLETE':'探索完成，准备保存', 'SAVING':'正在保存地图',
'NAV_READY':'地图已加载，定位与导航就绪', 'INCOMPLETE':'仍有未解决区域，未标记完整地图',
'PAUSED':'已暂停', 'FAULT':'发生故障'}
def main():
    rclpy.init()
    node=Node('mapping_status_console')
    previous=[None]
    def receive(msg):
        try: data=json.loads(msg.data)
        except (ValueError, TypeError): return
        key=(data.get('state'),data.get('reason'),data.get('retries'))
        if key==previous[0]: return
        previous[0]=key
        print(LABELS.get(key[0],key[0]), '|', key[1], '| 重试:',key[2],flush=True)
    node.create_subscription(String,'/mapping/status',receive,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    print('等待管理器状态；Ctrl+C 仅退出显示，不会停止机器人。',flush=True)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
