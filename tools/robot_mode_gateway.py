#!/usr/bin/env python3
"""Authenticated wired TCP JSON-lines mode adapter. Never publishes motion goals."""
import argparse
from collections import OrderedDict
import hmac
import json
from pathlib import Path
import socketserver
import threading
import time

SERVICES = {'manual_mapping':'manual_mapping', 'auto_mapping':'auto_mapping',
            'navigation':'load_latest', 'finish_mapping':'finish_mapping',
            'pause':'pause', 'resume':'resume', 'cancel':'cancel'}
class Protocol:
    def __init__(self, token, dispatch, status):
        self.token=token; self.dispatch=dispatch; self.status=status
        self.cache=OrderedDict(); self.lock=threading.Lock()
    def handle(self, raw):
        try:
            if len(raw)>4096: raise ValueError('request too large')
            req=json.loads(raw)
            if not isinstance(req,dict): raise ValueError('expected object')
            token=req.get('token','')
            if not isinstance(token,str) or not hmac.compare_digest(token,self.token):
                return {'ok':False,'error':'unauthorized'}
            ident=req.get('id'); command=req.get('command')
            if not isinstance(ident,str) or not 1<=len(ident)<=80: raise ValueError('id required, max80 chars')
            if command=='status': return {'id':ident,'ok':True,'status':self.status()}
            if command not in SERVICES: raise ValueError('unsupported command')
            with self.lock:
                if ident in self.cache:
                    old_command,response=self.cache[ident]
                    if old_command!=command: raise ValueError('id reused for different command')
                    return response
                try: accepted,message=self.dispatch(SERVICES[command])
                except Exception:
                    accepted=False;message='outcome_unknown; query status before another request'
                response={'id':ident,'ok':accepted,'message':message,'status':self.status()}
                self.cache[ident]=(command,response)
                if len(self.cache)>256:self.cache.popitem(last=False)
                return response
        except (ValueError,TypeError):return {'ok':False,'error':'invalid request'}

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bind',required=True,help='specific wired IPv4 address, never 0.0.0.0')
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--token-file',default=str(Path.home()/'.config/luckrobot/mode_token'))
    args=parser.parse_args()
    import ipaddress
    address=ipaddress.ip_address(args.bind)
    if address.version!=4 or address.is_unspecified:parser.error('specific IPv4 address required')
    token=Path(args.token_file).read_text().strip()
    if len(token)<32:parser.error('token must contain at least32 characters')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init();node=Node('robot_mode_gateway')
    snapshot={'state':'UNAVAILABLE'};seen=[0.0]
    def receive(msg):
        try:
            data=json.loads(msg.data)
            snapshot.clear();snapshot.update(data);seen[0]=time.monotonic()
        except ValueError:pass
    node.create_subscription(String,'/mapping/status',receive,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients={name:node.create_client(Trigger,'/mapping/'+name) for name in set(SERVICES.values())}
    def status():return dict(snapshot, stale=time.monotonic()-seen[0]>3)
    def dispatch(name):
        if time.monotonic()-seen[0]>3:return False,'manager status stale'
        client=clients[name]
        if not client.wait_for_service(timeout_sec=1):return False,'service unavailable'
        future=client.call_async(Trigger.Request());done=threading.Event()
        future.add_done_callback(lambda _:done.set())
        if not done.wait(4):raise TimeoutError()
        result=future.result();return result.success,result.message
    protocol=Protocol(token,dispatch,status)
    slots=threading.BoundedSemaphore(4)
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(30)
            while True:
                try:raw=self.rfile.readline(4097)
                except (OSError,TimeoutError):return
                if not raw or len(raw)>4096:return
                reply=protocol.handle(raw)
                try:self.wfile.write((json.dumps(reply,ensure_ascii=False)+'\n').encode())
                except OSError:return
    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address=True;daemon_threads=True
        def verify_request(self,request,address):return slots.acquire(blocking=False)
        def process_request_thread(self,request,address):
            try:super().process_request_thread(request,address)
            finally:slots.release()
    server=Server((args.bind,args.port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print('Mode gateway ready on '+args.bind+':'+str(args.port),flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        server.shutdown();server.server_close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
