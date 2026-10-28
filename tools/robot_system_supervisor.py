#!/usr/bin/env python3
"""Single-owner robot launch supervisor. Startup never requests motion."""
import fcntl,json,os,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

class Supervisor(Node):
 def __init__(self):
  super().__init__('robot_system_supervisor')
  root=Path.home()/'.local/state/luckrobot';root.mkdir(parents=True,exist_ok=True)
  self.lockfile=(root/'supervisor.lock').open('w');fcntl.flock(self.lockfile,fcntl.LOCK_EX|fcntl.LOCK_NB)
  self.proc=None;self.log=None;self.state='STOPPED';self.reason='awaiting_start';self.instance=None
  self.manager={};self.seen=0.;self.started=0.;self.busy=False;self.guard=threading.Lock()
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.pub=self.create_publisher(String,'/robot/system/status',qos)
  self.create_subscription(String,'/mapping/status',self.receive,qos)
  self.pause=self.create_client(Trigger,'/mapping/pause')
  self.create_service(Trigger,'/robot/system/start',self.start)
  self.create_service(Trigger,'/robot/system/stop',self.stop)
  self.create_service(Trigger,'/robot/system/restart',self.restart)
  self.create_timer(1.,self.tick)
 def receive(self,msg):
  try:self.manager=json.loads(msg.data);self.seen=time.monotonic()
  except ValueError:pass
 def start(self,req,res):
  with self.guard:
   if self.busy:res.success=False;res.message='busy';return res
   if self.proc and self.proc.poll() is None:
    res.success=True;res.message='already started; inspect /robot/system/status';return res
   # Never adopt/terminate independently started robot stacks.
   if any(n in self.get_node_names() for n in ('map_session_manager','controller_server','ekf_filter_node','slam_toolbox')):
    res.success=False;res.message='external robot stack exists; stop it explicitly first';return res
   import uuid
   self.instance=uuid.uuid4().hex;self.manager={};self.seen=0.
   path=Path.home()/'.local/state/luckrobot'/('stack_'+self.instance+'.log')
   self.log=path.open('a')
   self.proc=subprocess.Popen(['ros2','launch','my_robot_mapping','managed_mapping.launch.py','boot_mode:=idle','auto_explore:=false'],stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
   self.started=time.monotonic();self.state='STARTING';self.reason='managed_idle_launch_requested'
   res.success=True;res.message='accepted; wait for RUNNING and fresh mapping status';return res
 def stop(self,req,res):
  with self.guard:
   if self.busy:res.success=False;res.message='busy';return res
   if not self.proc or self.proc.poll() is not None:
    res.success=True;res.message='already stopped';return res
   self.busy=True;self.state='STOPPING';self.reason='requesting_pause'
   threading.Thread(target=self.stop_worker,daemon=True).start()
   res.success=True;res.message='accepted; wait for STOPPED';return res
 def restart(self,req,res):
  if not self.proc or self.proc.poll() is not None:return self.start(req,res)
  with self.guard:
   if self.busy:res.success=False;res.message="busy";return res
   self.busy=True;self.state="STOPPING";self.reason="restart_requested"
   threading.Thread(target=self.stop_worker,args=(True,),daemon=True).start()
   res.success=True;res.message="accepted; restart ends in RUNNING/IDLE, old tasks not resumed";return res
 def stop_worker(self,restart=False):
  try:
   if not self.pause.wait_for_service(timeout_sec=3):raise RuntimeError('pause service unavailable; stack retained')
   future=self.pause.call_async(Trigger.Request());event=threading.Event()
   future.add_done_callback(lambda _:event.set())
   if not event.wait(8) or not future.result().success:raise RuntimeError('pause not accepted; stack retained')
   end=time.monotonic()+45
   while time.monotonic()<end:
    if time.monotonic()-self.seen<3 and self.manager.get('state')=='PAUSED':break
    time.sleep(.2)
   else:raise RuntimeError('stationary pause not confirmed; stack retained')
   os.killpg(self.proc.pid,signal.SIGINT)
   try:self.proc.wait(timeout=45)
   except subprocess.TimeoutExpired:raise RuntimeError('launch shutdown timeout; no forced kill')
   self.state='STOPPED';self.reason='paused_then_launch_exited'
   if self.log:self.log.close();self.log=None
  except Exception as exc:self.state='FAULT';self.reason=str(exc)
  finally:self.busy=False
  if restart and self.state=="STOPPED":
   response=self.start(Trigger.Request(),Trigger.Response())
   if not response.success:self.state="FAULT";self.reason=response.message
 def tick(self):
  if not self.busy and self.proc:
   if self.proc.poll() is not None and self.state not in ('STOPPED','FAULT'):
    self.state='FAULT';self.reason='stack_exited_'+str(self.proc.returncode)
   elif self.state=='STARTING':
    if self.seen>self.started and time.monotonic()-self.seen<3 and self.manager.get('state')=='IDLE':
     self.state='RUNNING';self.reason='manager_idle_ready'
    elif time.monotonic()-self.started>90:self.state='FAULT';self.reason='startup_timeout; inspect log'
  fresh=self.seen>0 and time.monotonic()-self.seen<3
  self.pub.publish(String(data=json.dumps(dict(schema=1,state=self.state,reason=self.reason,instance_id=self.instance,stack_alive=bool(self.proc and self.proc.poll() is None),manager_fresh=fresh,manager_state=self.manager.get('state'),updated_unix=time.time()))))

def main():
 rclpy.init();node=Supervisor()
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
