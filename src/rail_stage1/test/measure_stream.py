import json, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
rclpy.init()
n=Node('rail_stream_readonly_measure')
seen={key:[] for key in ('rgb','mask','stats')}
started=time.monotonic()
def image_cb(key,m):
    ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
    seen[key].append([time.monotonic(),(n.get_clock().now().nanoseconds-ns)*1e-6])
for key,topic in [('rgb','/camera/color/image_raw'),('mask','/rails/mask')]:
    n.create_subscription(Image,topic,lambda m,k=key:image_cb(k,m),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
n.create_subscription(Float32MultiArray,'/rails/mask_stats',lambda m:seen['stats'].append(list(m.data)),qos_profile_sensor_data)
while time.monotonic()-started<12:
    rclpy.spin_once(n,timeout_sec=.1)
result={}
for key in ('rgb','mask'):
    frames=seen[key]
    result[key]={'n':len(frames)}
    if len(frames)>1:
        result[key].update(hz=(len(frames)-1)/(frames[-1][0]-frames[0][0]),age_ms_p50=float(np.median([f[1] for f in frames])),age_ms_p95=float(np.quantile([f[1] for f in frames],.95)))
if seen['stats']:
    result['inference_ms_p50']=float(np.median([f[0] for f in seen['stats']]))
print(json.dumps(result))
n.destroy_node()
rclpy.shutdown()
