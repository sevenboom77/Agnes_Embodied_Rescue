"""VLM input is exclusively current ROS Image bytes. No world/entity/registry reads."""
import base64
import hashlib
import json
import math
from pathlib import Path
import select
import struct
import subprocess
import threading
import zlib
import sys

ROOT=Path('/data-ssd/zhongchengkai/agnes')
def png_bytes(msg):
    if msg.encoding!='rgb8' or len(msg.data)!=msg.height*msg.step:
        raise ValueError('UNSUPPORTED_IMAGE_LAYOUT')
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data))
    raw=bytes(msg.data)
    rows=b''.join(b'\0'+raw[y*msg.step:y*msg.step+msg.width*3] for y in range(msg.height))
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',msg.width,msg.height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b'')

class Perception:
    def __init__(self):
        self.child=None
        self.lock=threading.Lock()
        self.sequence=0
        self.cache={}
    def request(self,method,png=None):
        if self.child is None:
            self.child=subprocess.Popen([str(ROOT/'tools/node/bin/node'),str(ROOT/'third_party/agnes-harness/node_modules/tsx/dist/cli.mjs'),
                str(ROOT/'services/robot-mcp/perception/vlm-worker.mts')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        self.sequence+=1
        request={'id':str(self.sequence),'method':method}
        if png: request['png']=base64.b64encode(png).decode()
        self.child.stdin.write(json.dumps(request)+'\n');self.child.stdin.flush()
        if not select.select([self.child.stdout],[],[],55)[0]:
            print('PERCEPTION_SAFE_ERROR=RESPONSE_TIMEOUT',file=sys.stderr,flush=True)
            self.close();self.child=None
            raise ValueError('PERCEPTION_TIMEOUT')
        response=json.loads(self.child.stdout.readline())
        if response.get('id')!=request['id'] or not response.get('ok'):
            stage=response.get('error',{}).get('stage')
            if stage in ['CONNECT','SESSION_NEW','IMAGE_PROMPT','EVENT_READING','JSON_PARSE','EMPTY_ASSISTANT_OUTPUT','NO_VALID_DETECTIONS_JSON']:
                print('PERCEPTION_SAFE_ERROR='+stage,file=sys.stderr,flush=True)
            raise ValueError('PERCEPTION_UNAVAILABLE')
        return response['result']
    def detect(self,msg):
        with self.lock:
            key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
            if key in self.cache:return self.cache[key]
            png=png_bytes(msg)
            # One retry for transient inference/output failure; input pixels unchanged.
            try:result=self.request('detect',png)
            except ValueError:result=self.request('detect',png)
            detections=[]
            for d in result['detections']:
                box=[d['bbox'][i]*(msg.width if i%2==0 else msg.height) for i in range(4)]
                digest=hashlib.sha256((str(key)+d['label']+str(box)).encode()).hexdigest()[:16]
                detections.append({'id':'visual_'+digest,'label':d['label'],'confidence':d['confidence'],
                    'bbox':box,'center':[(box[0]+box[2])/2,(box[1]+box[3])/2],
                    'source':'ros2_camera_perception','frame_id':msg.header.frame_id,
                    'timestamp':{'sec':key[0],'nanosec':key[1]},
                    'evidence':{'method':'vlm','provider':'agnes-ai','model':'agnes-3.0-flash',
                        'frame_timestamp':{'sec':key[0],'nanosec':key[1]},'frame_sha256':hashlib.sha256(png).hexdigest(),
                        'image_width':msg.width,'image_height':msg.height,'vlm_tool_calls':result['tool_calls']}})
            self.cache={key:detections}
            return detections
    def close(self):
        if self.child:
            self.child.stdin.close()
            try:self.child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                self.child.terminate()
                try:self.child.wait(timeout=3)
                except subprocess.TimeoutExpired:self.child.kill();self.child.wait(timeout=3)
