"""Switchable detector + map registry. This node never commands motion."""
from collections import deque
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from .geometry import detect, rotation
from .store import RailStore, approach_pose


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000+header.stamp.nanosec


def image_array(message, dtype):
    dtype = np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    return np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data, strides=(message.step, dtype.itemsize)).copy()


def transform_values(tf):
    q, t = tf.transform.rotation, tf.transform.translation
    return rotation([q.x, q.y, q.z, q.w]), np.array([t.x, t.y, t.z])


class RailFinder(Node):
    def __init__(self):
        super().__init__('rail_finder')
        path = Path(get_package_share_directory('rail_stage1'))/'config/rails.yaml'
        defaults = yaml.safe_load(path.read_text())['rail_finder']['ros__parameters']
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.enabled = self.cfg['enabled']
        self.store = RailStore(self.cfg['database_path'], self.cfg['map_id'], self.cfg)
        self.depth = deque(maxlen=30)
        self.masks = deque(maxlen=8)
        self.info = None
        self.last_processed = -1
        self.debug_stamp = -1
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.registry_pub = self.create_publisher(String, '/rails/registry', durable)
        self.markers_pub = self.create_publisher(MarkerArray, '/rails/approach_markers', durable)
        self.status_pub = self.create_publisher(String, '/rails/search/status', durable)
        self.observation_pub = self.create_publisher(PoseStamped, '/rails/entry_observation', 10)
        self.debug_pub = self.create_publisher(Image, '/rails/search/debug_bev', 1)
        self.create_subscription(Image, self.cfg['mask_topic'], self.on_mask,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Image, self.cfg['depth_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_service(SetBool, '/rails/search/set_enabled', self.set_enabled)
        self.last_reason = 'search disabled' if not self.enabled else 'waiting for images'
        self.create_timer(1., self.publish_registry)
        self.publish_registry()

    def on_info(self, message):
        self.info = message

    def set_enabled(self, request, response):
        self.enabled = request.data
        self.masks.clear()
        self.store.pending.clear()
        self.last_reason = 'waiting for three lines' if self.enabled else 'search disabled'
        response.success = True
        response.message = self.last_reason+'; saved rails retained'
        self.publish_registry()
        return response

    def on_depth(self, message):
        if not self.enabled:
            return
        self.depth.append(message)
        self.process()

    def on_mask(self, message):
        if not self.enabled:
            return
        self.masks.append(message)
        self.process()

    def process(self):
        if not self.enabled or self.info is None or not self.depth or not self.masks:
            return
        mask_msg = self.masks[-1]
        ns = stamp_ns(mask_msg.header)
        if ns <= self.last_processed:
            return
        depth_msg = min(self.depth, key=lambda m: abs(stamp_ns(m.header)-ns))
        if abs(stamp_ns(depth_msg.header)-ns)*1e-9 > self.cfg['pair_tolerance_sec']:
            self.last_reason = 'waiting for matching depth'
            return
        self.last_processed = ns
        try:
            age = (self.get_clock().now().nanoseconds-ns)*1e-9
            if age < -.1 or age > self.cfg['max_observation_age_sec']:
                raise ValueError('stale image or clock mismatch')
            if mask_msg.encoding != 'mono8':
                raise ValueError('mask must be mono8')
            if depth_msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError('unsupported depth encoding')
            if (mask_msg.width, mask_msg.height) != (depth_msg.width, depth_msg.height):
                raise ValueError('depth must be aligned to mask')
            if self.info.header.frame_id != mask_msg.header.frame_id:
                raise ValueError('CameraInfo and mask frames differ')
            if depth_msg.header.frame_id != mask_msg.header.frame_id:
                raise ValueError('depth and mask optical frames differ')
            if (self.info.width, self.info.height) != (mask_msg.width, mask_msg.height):
                raise ValueError('CameraInfo resolution differs')
            if self.info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('unsupported camera distortion model')
            mask = image_array(mask_msg, np.uint8)
            if np.count_nonzero(mask) < self.cfg['min_mask_area_px']:
                raise ValueError('mask below minimum area')
            dtype = np.uint16 if depth_msg.encoding == '16UC1' else np.float32
            depth = image_array(depth_msg, dtype).astype(float)
            if dtype == np.uint16:
                depth *= .001
            stamp = Time.from_msg(mask_msg.header.stamp)
            camera_tf = self.tf.lookup_transform(self.cfg['base_frame'], mask_msg.header.frame_id,
                                                 stamp, timeout=Duration(seconds=.1))
            map_tf = self.tf.lookup_transform('map', self.cfg['base_frame'], stamp,
                                              timeout=Duration(seconds=.1))
            r, origin = transform_values(camera_tf)
            k = np.array(self.info.k).reshape(3, 3)
            point, direction, bev, lines = detect(mask, depth, k, np.array(self.info.d), r, origin, self.cfg,
                debug=lambda image, chosen: self.publish_debug(image, chosen, mask_msg.header.stamp))
            mr, mt = transform_values(map_tf)
            point, direction = mr @ point+mt, mr @ direction
            yaw = math.atan2(direction[1], direction[0])
            observation = PoseStamped()
            observation.header.frame_id = 'map'
            observation.header.stamp = mask_msg.header.stamp
            observation.pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            observation.pose.orientation.z = math.sin(yaw/2)
            observation.pose.orientation.w = math.cos(yaw/2)
            self.observation_pub.publish(observation)
            rail = self.store.observe(point, yaw, ns, time.monotonic())
            self.last_reason = f"saved {rail['id']}" if rail else 'three lines found; collecting confirmation'
            if rail:
                self.publish_registry()
        except (ValueError, cv2.error, tf2_ros.TransformException, OSError) as exc:
            self.last_reason = str(exc)
            if self.debug_stamp != ns:
                self.publish_debug(np.zeros((300, 320), np.uint8), [], mask_msg.header.stamp)

    def publish_debug(self, bev, lines, stamp):
        self.debug_stamp = stamp.sec*1_000_000_000+stamp.nanosec
        image = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        if not lines:
            cv2.putText(image, 'UNCONFIRMED', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 150, 255), 1)
        for line, color in zip(lines, [(255, 180, 0), (255, 0, 255), (0, 0, 255)]):
            pixels = [(int(round((p[0]-self.cfg['bev_x_min'])/self.cfg['bev_resolution'])),
                       int(round((p[1]+self.cfg['bev_half_width'])/self.cfg['bev_resolution'])))
                      for p in (line['a'], line['b'])]
            cv2.line(image, *pixels, color, 2)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.cfg['base_frame']
        msg.height, msg.width = image.shape[:2]
        msg.encoding, msg.step, msg.data = 'bgr8', image.shape[1]*3, image.tobytes()
        self.debug_pub.publish(msg)

    def publish_registry(self):
        data = dict(frame='map', map_id=self.cfg['map_id'], rails=list(self.store.rails.values()))
        self.registry_pub.publish(String(data=json.dumps(data)))
        self.status_pub.publish(String(data=json.dumps(dict(enabled=self.enabled,
            reason=self.last_reason, confirmed_rails=len(self.store.rails), pending=len(self.store.pending)))))
        array = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        array.markers.append(wipe)
        for index, rail in enumerate(self.store.rails.values()):
            p = np.array(rail['point'])
            goal, yaw = approach_pose(rail, self.cfg['approach_distance'])
            direction = np.array([math.cos(yaw), math.sin(yaw), 0.])
            for offset, kind, name, start, end, color in [
                    (0, Marker.SPHERE, 'entry', p, None, (0., 1., .2)),
                    (1, Marker.ARROW, 'rail_direction', p, p+.5*direction, (0., 1., .2)),
                    (2, Marker.ARROW, 'approach_goal', goal, goal+.35*direction, (0., .6, 1.)),
                    (3, Marker.LINE_STRIP, 'approach_offset', goal, p, (.6, .6, .6)),
                    (4, Marker.TEXT_VIEW_FACING, 'rail_id', p+np.array([0., 0., .25]), None, (1., 1., 1.))]:
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns, marker.id, marker.type = name, index*10+offset, kind
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.
                if end is None:
                    marker.pose.position = Point(x=float(start[0]), y=float(start[1]), z=float(start[2]))
                else:
                    marker.points = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2]+.025)) for v in (start, end)]
                if kind == Marker.SPHERE:
                    marker.scale.x = marker.scale.y = marker.scale.z = .10
                elif kind == Marker.ARROW:
                    marker.scale.x, marker.scale.y, marker.scale.z = .04, .09, .12
                elif kind == Marker.LINE_STRIP:
                    marker.scale.x = .015
                else:
                    marker.scale.z = .12
                    marker.text = rail['id']+' | entry / approach 0.90 m'
                array.markers.append(marker)
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = RailFinder()
    # TF callbacks must keep running while image processing waits for a TF
    # at the camera timestamp. A single-thread executor cannot do that.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
