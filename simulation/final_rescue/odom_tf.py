"""Real /odom plus one calibrated static transform batch; never controls."""
import os,math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
class CalibratedOdomTf(Node):
 def __init__(self):
  super().__init__('agnes_final_odom_tf',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.dynamic=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
  origin=TransformStamped();origin.header.frame_id='map';origin.child_frame_id='odom'
  origin.transform.translation.x=float(os.environ.get('SCENARIO_MAP_ODOM_X','0'))
  origin.transform.translation.y=float(os.environ.get('SCENARIO_MAP_ODOM_Y','0'))
  yaw=float(os.environ.get('SCENARIO_MAP_ODOM_YAW','0'))
  origin.transform.rotation.z=math.sin(yaw/2);origin.transform.rotation.w=math.cos(yaw/2)
  transforms=[] if os.environ.get('SCENARIO_LOCALIZATION')=='amcl' else [origin]
  for frame,x in [('lidar_link',.1),('camera_link',.25)]:
   tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id=frame
   tf.transform.translation.x=x;tf.transform.translation.z=.22;tf.transform.rotation.w=1.;transforms.append(tf)
  # One batch only: StaticTransformBroadcaster retains the first entry per child.
  self.static.sendTransform(transforms)
  self.create_subscription(Odometry,'/odom',self.on_odom,10)
 def on_odom(self,msg):
  t=TransformStamped();t.header=msg.header;t.header.frame_id='odom';t.child_frame_id='base_link'
  p=msg.pose.pose.position;t.transform.translation.x=p.x;t.transform.translation.y=p.y;t.transform.translation.z=p.z
  t.transform.rotation=msg.pose.pose.orientation;self.dynamic.sendTransform(t)
def main():
 rclpy.init();node=CalibratedOdomTf()
 try:rclpy.spin(node)
 except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
