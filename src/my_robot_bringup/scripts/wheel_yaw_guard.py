#!/usr/bin/env python3
"""Conservative wheel-yaw admission for a no-TF EKF evaluation candidate."""
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException

def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def rotate_z(q, v):
    n=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
    if not math.isfinite(n) or n < 1e-6:raise ValueError('invalid quaternion')
    x,y,z,w=q.x/n,q.y/n,q.z/n,q.w/n
    return 2*(x*z-y*w)*v.x+2*(y*z+x*w)*v.y+(1-2*(x*x+y*y))*v.z

def consistent(wheel, imu, absolute=.12, relative=.25):
    return (math.isfinite(wheel) and math.isfinite(imu)
            and abs(wheel-imu)<=absolute+relative*max(abs(wheel),abs(imu)))

class WheelYawGuard(Node):
    def __init__(self):
        super().__init__('wheel_yaw_guard')
        self.declare_parameter('base_frame','base_footprint')
        self.base=str(self.get_parameter('base_frame').value)
        self.tf=Buffer(cache_time=Duration(seconds=5.))
        self.listener=TransformListener(self.tf,self)
        self.samples=deque(maxlen=100)
        self.good_since=None
        self.last_wheel=None
        self.pub=self.create_publisher(TwistWithCovarianceStamped,'/wheel/yaw_validated',10)
        self.health=self.create_publisher(Bool,'/wheel/yaw_healthy',10)
        self.create_subscription(Imu,'/imu/data',self.imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/wheel/odom',self.wheel,qos_profile_sensor_data)
        self.create_timer(.1,self.check_stale)
    def invalidate(self):
        self.good_since=None
        self.health.publish(Bool(data=False))
    def check_stale(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.last_wheel is None or not 0<=now-self.last_wheel<=.2:self.invalidate()
    def imu(self,m):
        try:
            if m.angular_velocity_covariance[0]<0:return
            tr=self.tf.lookup_transform(self.base,m.header.frame_id,Time.from_msg(m.header.stamp))
            value=rotate_z(tr.transform.rotation,m.angular_velocity)
            if math.isfinite(value):self.samples.append((seconds(m.header.stamp),value))
        except (TransformException,ValueError):
            pass
    def wheel(self,m):
        now=self.get_clock().now().nanoseconds*1e-9
        stamp=seconds(m.header.stamp)
        previous=self.last_wheel
        self.last_wheel=stamp
        if (m.child_frame_id!=self.base or not 0<=now-stamp<=.2 or
                (previous is not None and (stamp<=previous or stamp-previous>.2)) or not self.samples):
            self.invalidate();return
        it,iw=min(self.samples,key=lambda item:abs(item[0]-stamp))
        wz=m.twist.twist.angular.z
        if abs(it-stamp)>.05 or not consistent(wz,iw):
            self.invalidate();return
        if self.good_since is None:self.good_since=stamp
        if stamp-self.good_since<.5:
            self.health.publish(Bool(data=False));return
        out=TwistWithCovarianceStamped()
        out.header.stamp=m.header.stamp
        out.header.frame_id=self.base
        out.twist.twist.angular.z=wz
        out.twist.covariance=[0.]*36
        for i in [0,7,14,21,28]:out.twist.covariance[i]=1e6
        # Never give wheel yaw more confidence than the existing driver's 0.1.
        variance=float(m.twist.covariance[35])
        out.twist.covariance[35]=max(.1,variance) if math.isfinite(variance) else .1
        self.pub.publish(out)
        self.health.publish(Bool(data=True))

def main():
    rclpy.init();node=WheelYawGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
