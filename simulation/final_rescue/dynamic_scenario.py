"""Scenario actuation and test evidence only. Never exposes scenario state to the agent."""
import json, os, time, math, subprocess, threading
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import String
ROOT=Path('/data-ssd/zhongchengkai/agnes')
OUT=Path(os.environ['FINAL_RUN_DIR'])
SCENARIO_FILE=Path(os.environ.get('SCENARIO_PRIVATE_FILE', ROOT/'artifacts/interactive-live/scenario-controller/current-private.json'))
SCENARIO=json.loads(SCENARIO_FILE.read_text()) if SCENARIO_FILE.exists() else {}
HIDDEN=SCENARIO.get('hidden_ground_truth', {})
EVENT=HIDDEN.get('dynamic_obstacle_event', {})
TRIGGER=HIDDEN.get('dynamic_obstacle_trigger', {}).get('kind', 'after_room_inspection')
WORLD_NAME=os.environ.get('SCENARIO_WORLD_NAME', 'final_rescue')
def save(name,data):
    p=OUT/name;t=p.with_suffix('.tmp');t.write_text(json.dumps(data,indent=2)+'\n');t.replace(p)
class DynamicScenarioController(Node):
    def __init__(self):
        super().__init__('dynamic_scenario_controller')
        self.scan=None;self.pose=None;self.fired=False;self.path_count=0;self.history=[]
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_pose,20)
        self.create_subscription(NavPath,'/plan',self.on_path,10)
        self.create_subscription(String,'/rescue/inspection_completed',self.trigger,10)
        # These are current-run evidence files.  Remove only the known generated
        # files so a new scene cannot inherit a prior event as live evidence.
        for name in ('dynamic-obstacle-trigger.json','lidar-before-block.json','lidar-after-block.json','nav2-paths.jsonl','physical-motion.json','scenario-physical-objects.json'):
            try: (OUT/name).unlink()
            except FileNotFoundError: pass
        save('dynamic-obstacle-trigger.json',{'triggered':False,'initially_blocking':False,'source':'new Gazebo runtime'})
        threading.Thread(target=self.spawn_randomized_objects,daemon=True).start()

    def spawn_randomized_objects(self):
        """Materialize the seed-selected objects in Gazebo; never publish their truth."""
        objects=HIDDEN.get('object_placements',[])
        person=HIDDEN.get('person_pose',{})
        specs=[('scenario_person',person.get('x',6),person.get('y',1),person.get('yaw',0),'person'),
               *[(item.get('id','scenario_object'),item.get('x',0),item.get('y',0),item.get('yaw',0),item.get('label','object')) for item in objects]]
        directory=ROOT/'artifacts/interactive-live/scenario-runtime'
        directory.mkdir(parents=True,exist_ok=True)
        for _ in range(12):
            try:
                for name in ('visual_01','visual_02','visual_03'):
                    subprocess.run(['gz','service','-s',f'/world/{WORLD_NAME}/remove','--reqtype','gz.msgs.Entity','--reptype','gz.msgs.Boolean','--timeout','1000','--req',f'name: "{name}"'],capture_output=True,text=True,timeout=3)
                evidence=[]
                for name,x,y,yaw,label in specs:
                    sdf=directory/f'{SCENARIO.get("seed", "current")}-{name}.sdf'
                    if label=='person':
                        body='<link name="body"><visual name="visual"><geometry><box><size>.28 .45 .65</size></box></geometry><material><diffuse>.8 .02 .8 1</diffuse></material></visual></link><link name="head"><pose>0 0 .48 0 0 0</pose><visual name="visual"><geometry><sphere><radius>.17</radius></sphere></geometry><material><diffuse>.8 .55 .4 1</diffuse></material></visual></link>'
                    elif label=='fire_extinguisher':
                        body='<link name="body"><pose>0 0 .35 0 0 0</pose><visual name="visual"><geometry><cylinder><radius>.12</radius><length>.65</length></cylinder></geometry><material><diffuse>.8 .02 .02 1</diffuse></material></visual></link>'
                    elif label=='toolbox':
                        body='<link name="body"><pose>0 0 .17 0 0 0</pose><visual name="visual"><geometry><box><size>.45 .3 .32</size></box></geometry><material><diffuse>.05 .15 .7 1</diffuse></material></visual></link>'
                    else:
                        body='<link name="body"><pose>0 0 .2 0 0 0</pose><visual name="visual"><geometry><box><size>.35 .35 .4</size></box></geometry><material><diffuse>.9 .75 .05 1</diffuse></material></visual></link>'
                    sdf.write_text(f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static><pose>{x} {y} 0 {0} {0} {yaw}</pose>{body}</model></sdf>')
                    result=subprocess.run(['gz','service','-s',f'/world/{WORLD_NAME}/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','2000','--req',f'sdf_filename: "{sdf}"'],capture_output=True,text=True,timeout=4)
                    evidence.append({'model':name,'label':label,'gazebo_response':result.stdout,'returncode':result.returncode})
                save('scenario-physical-objects.json',{'seed':SCENARIO.get('seed'),'source':'seeded ScenarioManager -> Gazebo EntityFactory','objects_spawned':evidence,'agent_visible_truth':False})
                return
            except Exception as error:
                save('scenario-physical-objects.json',{'seed':SCENARIO.get('seed'),'source':'seeded ScenarioManager -> Gazebo EntityFactory','objects_spawned':[],'error':type(error).__name__,'agent_visible_truth':False})
                time.sleep(1)
    def on_scan(self,m):
        self.scan={'timestamp':{'sec':m.header.stamp.sec,'nanosec':m.header.stamp.nanosec},'angle_min':m.angle_min,'angle_increment':m.angle_increment,'ranges':[float(v) if math.isfinite(v) else None for v in m.ranges],'pose':self.pose}
    def on_pose(self,m):
        q=m.pose.pose.orientation
        self.pose={'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
        if not self.history or time.monotonic()-self.history[-1]['wall_monotonic']>.4:
            self.history.append({**self.pose,'wall_monotonic':time.monotonic(),'sim_sec':m.header.stamp.sec+m.header.stamp.nanosec/1e9})
            save('physical-motion.json',self.history)
        if not self.fired and TRIGGER != 'after_room_inspection' and self.path_count >= 1:
            self.trigger(String(data=json.dumps({'room':'navigation_observation','status':'SUCCEEDED'})))
    def on_path(self,m):
        self.path_count+=1
        with (OUT/'nav2-paths.jsonl').open('a') as f:f.write(json.dumps({'seq':self.path_count,'timestamp':{'sec':m.header.stamp.sec,'nanosec':m.header.stamp.nanosec},'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses]})+'\n')
    def trigger(self,m):
        event=json.loads(m.data)
        if EVENT.get('type')=='none' or self.fired or event.get('status')!='SUCCEEDED':return
        self.fired=True
        threading.Thread(target=self.spawn,args=(event,),daemon=True).start()
    def spawn(self,event):
        save('lidar-before-block.json',self.scan)
        event_type=EVENT.get('type','cabinet_block')
        sdf_name={'cabinet_block':'cabinet_block.sdf','crate_block':'crate_block.sdf','door_closes':'door_closes.sdf'}.get(event_type,'cabinet_block.sdf')
        sdf=ROOT/'simulation/final_rescue/dynamic_events'/sdf_name
        result=subprocess.run(['gz','service','-s',f'/world/{WORLD_NAME}/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','5000','--req',f'sdf_filename: "{sdf}"'],capture_output=True,text=True,timeout=8)
        geometry={'cabinet_block':{'center':[4.25,1,.7],'size':[.8,1.8,1.4]},'crate_block':{'center':[6,-4,.55],'size':[1.1,1.1,1.1]},'door_closes':{'center':[4.25,-4,.8],'size':[.25,2,1.6]}}.get(event_type)
        record={'triggered':result.returncode==0 and 'true' in result.stdout,'event':event,'event_type':event_type,'route':EVENT.get('route'),'gazebo_service_response':result.stdout,'collision_geometry':geometry,'initially_blocking':False,'wall_time':time.time(),'agent_notified':False,'source':'real Gazebo EntityFactory + LiDAR/Nav2 evidence'}
        time.sleep(1)
        save('lidar-after-block.json',self.scan);save('dynamic-obstacle-trigger.json',record)
rclpy.init();n=DynamicScenarioController()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
except Exception:
    if rclpy.ok():raise
finally:n.destroy_node();rclpy.try_shutdown()
