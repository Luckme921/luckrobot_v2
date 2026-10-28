#!/usr/bin/env python3
import argparse,json,socket,uuid
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('command',choices=['status','manual_mapping','auto_mapping','navigation','finish_mapping','pause','resume','cancel'])
p.add_argument('--host',default='10.0.0.51');p.add_argument('--port',type=int,default=8765)
p.add_argument('--token-file',default=str(Path.home()/'.config/luckrobot/mode_token'))
p.add_argument('--id',default=None)
a=p.parse_args()
req={'id':a.id or str(uuid.uuid4()),'command':a.command,'token':Path(a.token_file).read_text().strip()}
with socket.create_connection((a.host,a.port),timeout=8) as s:
 s.sendall((json.dumps(req)+'\n').encode());line=s.makefile('rb').readline(65537)
 print(json.dumps(json.loads(line),ensure_ascii=False,indent=2))
