"""Explicit ALIGN -> DRIVE action. Idle node never publishes velocity commands."""
from collections import deque
import json
import math
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from rail_stage1_interfaces.action import EnterRail
from .finder import image_array, stamp_ns, transform_values
from .geometry import detect
from .motion_lease import MotionLease
from .servo_control import (target_pixel, omni_ibvs_command, rail_heading, VelocitySlew,
                            AlignmentGate, Localization, StraightDrive)


class RailEntry(Node):
    def __init__(self):
        super().__init__('rail_visual_entry')
        root = Path(get_package_share_directory('rail_stage1'))/'config'
        defaults = yaml.safe_load((root/'entry.yaml').read_text())['rail_visual_entry']['ros__parameters']
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        geometry_defaults = yaml.safe_load((root/'rails.yaml').read_text())['rail_finder']['ros__parameters']
        self.geometry_cfg = {key: self.declare_parameter('geometry.'+key, value).value
                             for key, value in geometry_defaults.items()}
        self.validate_config()
        self.lock = threading.Lock()
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.action_group = ReentrantCallbackGroup()
        self.motion = MotionLease(self.cfg['motion_lock_path'])
        self.busy = False
        self.phase = 'IDLE'
        self.session = 0
        self.feature = None
        self.last_error = 'waiting for explicit entry action'
        self.status_phase = None
        self.status_extra = {}
        self.localization = None
        self.localization_error = ''
        self.info = None
        self.last_processed = -1
        self.mask_buffer = deque(maxlen=3)
        self.depth_buffer = deque(maxlen=30)
        self.rgb_buffer = deque(maxlen=10)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.cmd_pub = self.create_publisher(Twist, self.cfg['cmd_topic'], 1)
        self.debug_pub = self.create_publisher(Image, '/rails/entry/debug_image', 1)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/rails/entry/status', durable)
        self.marker_pub = self.create_publisher(MarkerArray, '/rails/entry/drive_markers', durable)
        self.create_subscription(CameraInfo, self.cfg['camera_info_topic'], self.on_info,
                                 qos_profile_sensor_data, callback_group=self.image_group)
        for topic, callback, kind in [(self.cfg['mask_topic'], self.on_mask, Image),
                                      (self.cfg['depth_topic'], self.on_depth, Image),
                                      (self.cfg['rgb_topic'], self.on_rgb, Image)]:
            qos = (qos_profile_sensor_data if topic == self.cfg['depth_topic'] else
                   QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.create_subscription(kind, topic, callback, qos, callback_group=self.image_group)
        self.create_subscription(Odometry, self.cfg['localization_topic'], self.on_localization,
                                 qos_profile_sensor_data, callback_group=self.action_group)
        self.server = ActionServer(self, EnterRail, '/rails/enter', self.execute,
                                  goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                  callback_group=self.action_group)
        self.create_timer(1., self.publish_status)
        self.publish_status()

    def validate_config(self):
        positive = ('default_distance', 'default_speed', 'max_drive_speed', 'control_rate',
                    'align_timeout_sec', 'feature_max_age_sec', 'feature_loss_timeout_sec',
                    'localization_max_age_sec', 'alignment_tolerance_px', 'alignment_frames',
                    'servo_gain', 'heading_gain', 'align_max_forward', 'align_max_reverse', 'align_max_yaw',
                    'align_max_lateral', 'alignment_heading_tolerance_deg', 'max_observed_heading_deg',
                    'align_linear_acceleration', 'align_yaw_acceleration')
        if any(not np.isfinite(self.cfg[k]) or self.cfg[k] <= 0 for k in positive):
            raise ValueError('entry parameters must be finite and positive')
        if self.cfg['default_speed'] > self.cfg['max_drive_speed']:
            raise ValueError('default drive speed exceeds maximum')
        for key, limit in [('align_min_linear', min(self.cfg[k] for k in
                           ('align_max_forward', 'align_max_reverse', 'align_max_lateral'))),
                           ('align_min_yaw', self.cfg['align_max_yaw'])]:
            if not np.isfinite(self.cfg[key]) or not 0 <= self.cfg[key] <= limit:
                raise ValueError('minimum alignment speed exceeds limit')
        if not self.cfg['alignment_heading_tolerance_deg'] < self.cfg['max_observed_heading_deg'] < 90:
            raise ValueError('invalid directed rail heading limits')
        if self.cfg['alignment_dwell_sec'] < 0 or self.cfg['target_bottom_offset_px'] < 0:
            raise ValueError('invalid alignment dwell or target offset')

    def goal(self, request):
        distance = request.drive_distance or self.cfg['default_distance']
        speed = request.drive_speed or self.cfg['default_speed']
        if not np.isfinite([distance, speed]).all() or distance <= 0 or not 0 < speed <= self.cfg['max_drive_speed']:
            return GoalResponse.REJECT
        with self.lock:
            if self.busy or not self.motion.acquire():
                return GoalResponse.REJECT
            self.busy = True
            self.session += 1
            self.feature = None
            self.phase = 'ALIGN'
            self.last_error = 'waiting for a fresh crossbar observation'
        return GoalResponse.ACCEPT

    def on_info(self, message):
        self.info = message

    def aligning(self):
        with self.lock:
            return self.busy and self.phase == 'ALIGN'

    def on_rgb(self, message):
        if self.aligning():
            self.rgb_buffer.append(message)

    def on_depth(self, message):
        if self.aligning():
            self.depth_buffer.append(message)
            self.process_image()

    def on_mask(self, message):
        if self.aligning():
            self.mask_buffer.append(message)
            self.process_image()

    def process_image(self):
        with self.lock:
            if not self.busy or self.phase != 'ALIGN':
                return
            session = self.session
        if self.info is None or not self.mask_buffer or not self.depth_buffer:
            return
        mask_msg = self.mask_buffer[-1]
        ns = stamp_ns(mask_msg.header)
        if ns <= self.last_processed:
            return
        depth_msg = min(self.depth_buffer, key=lambda m: abs(stamp_ns(m.header)-ns))
        if abs(stamp_ns(depth_msg.header)-ns)*1e-9 > self.geometry_cfg['pair_tolerance_sec']:
            return
        self.last_processed = ns
        feature = None
        error = ''
        try:
            age = (self.get_clock().now().nanoseconds-ns)*1e-9
            if not 0 <= age <= self.cfg['feature_max_age_sec']:
                raise ValueError('image is stale or has a future timestamp')
            if mask_msg.encoding != 'mono8' or depth_msg.encoding not in ('16UC1', '32FC1'):
                raise ValueError('unsupported mask/depth encoding')
            if (mask_msg.width, mask_msg.height) != (depth_msg.width, depth_msg.height):
                raise ValueError('depth is not aligned to the mask')
            if (self.info.width, self.info.height) != (mask_msg.width, mask_msg.height):
                raise ValueError('CameraInfo resolution differs')
            if not (mask_msg.header.frame_id == depth_msg.header.frame_id == self.info.header.frame_id):
                raise ValueError('mask, aligned depth and CameraInfo optical frames differ')
            if self.info.distortion_model not in ('plumb_bob', 'rational_polynomial', ''):
                raise ValueError('unsupported distortion model')
            mask = image_array(mask_msg, np.uint8)
            dtype = np.uint16 if depth_msg.encoding == '16UC1' else np.float32
            depth = image_array(depth_msg, dtype).astype(float)
            if dtype == np.uint16:
                depth *= .001
            tf = self.tf.lookup_transform(self.cfg['base_frame'], mask_msg.header.frame_id,
                                         Time.from_msg(mask_msg.header.stamp), timeout=Duration(seconds=.05))
            r, origin = transform_values(tf)
            k, distortion = np.array(self.info.k).reshape(3, 3), np.array(self.info.d)
            _, direction_base, _, _, center_base = detect(mask, depth, k, distortion, r, origin,
                                             self.geometry_cfg, return_feature=True)
            heading_error = rail_heading(direction_base)
            if abs(heading_error) > math.radians(self.cfg['max_observed_heading_deg']):
                raise ValueError('rail direction incompatible with forward entry')
            center_camera = r.T @ (center_base-origin)
            if center_camera[2] <= .05:
                raise ValueError('crossbar is behind the camera')
            pixel = cv2.projectPoints(center_camera[None], np.zeros(3), np.zeros(3), k, distortion)[0].reshape(2)
            if not (0 <= pixel[0] < mask_msg.width and 0 <= pixel[1] < mask_msg.height):
                raise ValueError('crossbar center outside image')
            x, y = np.rint(pixel).astype(int)
            x0, x1 = max(0, x-5), min(mask_msg.width, x+6)
            y0, y1 = max(0, y-5), min(mask_msg.height, y+6)
            depths = depth[y0:y1, x0:x1]
            valid = (mask[y0:y1, x0:x1] > 0) & np.isfinite(depths) & (depths > .1) & (depths < self.geometry_cfg['max_range'])
            if valid.sum() < 3:
                raise ValueError('crossbar center has no reliable depth')
            target = target_pixel(mask_msg.width, mask_msg.height, self.cfg['target_bottom_offset_px'])
            normalized = cv2.undistortPoints(np.array([pixel, target])[:, None], k, distortion).reshape(2, 2)
            camera_point = np.r_[normalized[0], 1.]*float(np.median(depths[valid]))
            # Inside each task tolerance stop that task, while retaining point
            # compensation for commanded yaw (camera lever arm included).
            point_aligned = np.max(np.abs(pixel-target)) <= self.cfg['alignment_tolerance_px']
            yaw_aligned = abs(heading_error) <= math.radians(self.cfg['alignment_heading_tolerance_deg'])
            command = omni_ibvs_command(camera_point, normalized[0] if point_aligned else normalized[1],
                                        0. if yaw_aligned else heading_error, r, origin,
                                        self.cfg['servo_gain'], self.cfg['heading_gain'],
                                        [self.cfg['align_max_forward'], self.cfg['align_max_lateral'], self.cfg['align_max_yaw']],
                                        [self.cfg['align_min_linear']]*2+[self.cfg['align_min_yaw']],
                                        self.cfg['jacobian_max_condition'], self.cfg['align_max_reverse'])
            axis_pixels = cv2.projectPoints(np.array([center_camera, r.T @ (center_base+.5*direction_base-origin)]),
                                           np.zeros(3), np.zeros(3), k, distortion)[0].reshape(2, 2)
            age = (self.get_clock().now().nanoseconds-ns)*1e-9
            if not 0 <= age <= self.cfg['feature_max_age_sec']:
                raise ValueError('image became stale during processing')
            feature = dict(stamp=ns, received=time.monotonic()-age, pixel=pixel, target=target,
                           error=pixel-target, heading_error=heading_error, axis_pixels=axis_pixels, command=command)
        except (ValueError, cv2.error, tf2_ros.TransformException) as exc:
            error = str(exc)
        with self.lock:
            if session != self.session or self.phase != 'ALIGN':
                return
            self.feature, self.last_error = feature, error or 'crossbar found'
        self.publish_debug(mask_msg, feature, error)

    def on_localization(self, message):
        try:
            now = time.monotonic()
            ns = stamp_ns(message.header)
            age = (self.get_clock().now().nanoseconds-ns)*1e-9
            if not 0 <= age <= self.cfg['localization_max_age_sec']:
                raise ValueError('localization timestamp stale or in the future')
            if message.header.frame_id != self.cfg['localization_frame']:
                raise ValueError('unexpected localization frame')
            if message.child_frame_id != self.cfg['base_frame']:
                raise ValueError('localization must describe the robot base, not camera')
            p, q = message.pose.pose.position, message.pose.pose.orientation
            vals = np.array([p.x, p.y, q.x, q.y, q.z, q.w], dtype=float)
            if not np.isfinite(vals).all() or np.linalg.norm(vals[2:]) < 1e-9:
                raise ValueError('invalid localization pose')
            x, y, z, w = vals[2:]/np.linalg.norm(vals[2:])
            yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            pose = Localization(p.x, p.y, yaw, message.header.frame_id, ns, now)
            with self.lock:
                if self.localization is not None and ns <= self.localization.stamp_ns:
                    if ns < self.localization.stamp_ns:
                        self.localization_error = 'localization timestamp moved backwards'
                    return
                self.localization, self.localization_error = pose, ''
        except ValueError as exc:
            with self.lock:
                self.localization_error = str(exc)

    def command(self, vx=0., vy=0., wz=0.):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(message)

    def stop(self):
        for _ in range(3):
            self.command()
            time.sleep(.02)

    def publish_status(self, **extra):
        with self.lock:
            if self.status_phase != self.phase:
                self.status_phase = self.phase
                self.status_extra = {}
            self.status_extra.update(extra)
            data = dict(phase=self.phase, busy=self.busy, reason=self.last_error)
            data.update(self.status_extra)
        self.status_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def feedback(self, handle, phase, error=None, distance=0., remaining=0.):
        message = EnterRail.Feedback()
        message.phase = phase
        if error is not None:
            message.pixel_error_x, message.pixel_error_y = float(error[0]), float(error[1])
        message.distance_travelled, message.drive_time_remaining = float(distance), float(remaining)
        handle.publish_feedback(message)

    def execute(self, handle):
        result = EnterRail.Result()
        drive = None
        try:
            gate = AlignmentGate(self.cfg['alignment_tolerance_px'], self.cfg['alignment_frames'],
                                 self.cfg['alignment_dwell_sec'], self.cfg['alignment_heading_tolerance_deg'])
            slew = VelocitySlew(self.cfg['align_linear_acceleration'], self.cfg['align_yaw_acceleration'])
            started = last_valid = time.monotonic()
            self.get_logger().info('ALIGN started: tolerance=%s px, frames=%s, dwell=%s s' %
                                   (gate.tolerance, gate.frames, gate.dwell))
            period = 1/self.cfg['control_rate']
            while rclpy.ok():
                now = time.monotonic()
                if handle.is_cancel_requested:
                    result.message = 'entry canceled during alignment'
                    handle.canceled()
                    return result
                if now-started >= self.cfg['align_timeout_sec']:
                    raise RuntimeError('alignment timeout')
                with self.lock:
                    feature = self.feature
                valid = feature is not None and now-feature['received'] <= self.cfg['feature_max_age_sec']
                if not valid:
                    self.command()
                    slew.reset()
                    gate.reset()
                    self.publish_status(alignment_frames=0, alignment_dwell_elapsed=0., observation_valid=False)
                    if now-last_valid >= self.cfg['feature_loss_timeout_sec']:
                        raise RuntimeError('crossbar observation lost: '+self.last_error)
                else:
                    last_valid = now
                    error = feature['error']
                    if gate.observe(error, feature['stamp'], now, feature['heading_error']):
                        self.command()
                        self.get_logger().info('ALIGN converged after %.2f s: error=%s, yaw=%.2f deg, frames=%d; switching to DRIVE' %
                                               (now-started, error.tolist(), math.degrees(feature['heading_error']), gate.count))
                        break
                    if np.max(np.abs(feature['command'])) <= 1e-8:
                        slew.reset()
                        velocity = np.zeros(3)
                    else:
                        velocity = slew.step(feature['command'], now)
                    self.command(*velocity)
                    self.feedback(handle, 'ALIGN', error)
                    self.publish_status(pixel_error=error.tolist(), target=feature['target'].tolist(),
                                        observed=feature['pixel'].tolist(), alignment_frames=gate.count,
                                        alignment_required_frames=gate.frames,
                                        alignment_dwell_elapsed=0. if gate.first_time is None else now-gate.first_time,
                                        alignment_tolerance_px=gate.tolerance, observation_valid=True,
                                        heading_error_deg=math.degrees(feature['heading_error']),
                                        heading_tolerance_deg=self.cfg['alignment_heading_tolerance_deg'],
                                        command_vx=float(velocity[0]), command_vy=float(velocity[1]),
                                        command_wz=float(velocity[2]))
                time.sleep(period)
            else:
                raise RuntimeError('ROS shutdown during alignment')
            if handle.is_cancel_requested:
                handle.canceled()
                result.message = 'entry canceled before drive'
                return result
            with self.lock:
                pose, localization_error = self.localization, self.localization_error
                self.phase = 'DRIVE'
            now = time.monotonic()
            if localization_error or pose is None or now-pose.received_at > self.cfg['localization_max_age_sec']:
                raise RuntimeError(localization_error or 'fresh localization required before drive')
            distance = handle.request.drive_distance or self.cfg['default_distance']
            speed = handle.request.drive_speed or self.cfg['default_speed']
            drive = StraightDrive(pose, distance, speed, now,
                                  self.cfg['localization_max_age_sec'], self.cfg['localization_jump_allowance'],
                                  self.cfg['max_localization_speed'], self.cfg['max_heading_deviation_deg'])
            self.publish_drive_markers(pose, distance)
            while rclpy.ok():
                with self.lock:
                    pose, localization_error = self.localization, self.localization_error
                if localization_error:
                    raise RuntimeError(localization_error)
                step = drive.step(pose, time.monotonic(), handle.is_cancel_requested)
                self.command(step.vx, 0., 0.)
                result.distance_travelled = step.distance
                self.feedback(handle, step.state, distance=step.distance, remaining=step.time_remaining)
                self.publish_status(distance=step.distance, time_remaining=step.time_remaining,
                                    speed=step.vx, drive_timeout=2*distance/speed)
                if step.state != 'DRIVE':
                    result.success, result.message = step.state == 'SUCCEEDED', step.reason
                    if result.success:
                        handle.succeed()
                    elif step.state == 'CANCELED':
                        handle.canceled()
                    else:
                        handle.abort()
                    return result
                time.sleep(period)
            raise RuntimeError('ROS shutdown during drive')
        except Exception as exc:
            result.message = str(exc)
            if drive is not None:
                result.distance_travelled = drive.progress
            handle.abort()
            return result
        finally:
            if rclpy.ok():
                self.stop()
            with self.lock:
                self.busy = False
                self.phase = 'SUCCEEDED' if result.success else 'STOPPED'
                self.last_error = result.message
                self.feature = None
                self.motion.release()
            if rclpy.ok():
                self.publish_status(distance=result.distance_travelled)

    def publish_debug(self, mask_msg, feature, error):
        rgb = next((m for m in self.rgb_buffer if stamp_ns(m.header) == stamp_ns(mask_msg.header)), None)
        if rgb is not None and rgb.encoding in ('rgb8', 'bgr8'):
            image = np.ndarray((rgb.height, rgb.width, 3), dtype=np.uint8,
                               buffer=rgb.data, strides=(rgb.step, 3, 1)).copy()
            if rgb.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            image = cv2.cvtColor(image_array(mask_msg, np.uint8), cv2.COLOR_GRAY2BGR)
        try:
            target = target_pixel(mask_msg.width, mask_msg.height, self.cfg['target_bottom_offset_px'])
            cv2.drawMarker(image, tuple(np.rint(target).astype(int)), (255, 160, 0), cv2.MARKER_CROSS, 24, 2)
            if feature is not None:
                point = tuple(np.rint(feature['pixel']).astype(int))
                cv2.circle(image, point, 7, (0, 255, 0), 2)
                cv2.line(image, point, tuple(np.rint(target).astype(int)), (0, 255, 255), 2)
                a, b = np.rint(feature['axis_pixels']).astype(int)
                cv2.arrowedLine(image, tuple(a), tuple(b), (255, 0, 255), 2, tipLength=.2)
                label = f"ALIGN dx={feature['error'][0]:+.1f} dy={feature['error'][1]:+.1f}px yaw={math.degrees(feature['heading_error']):+.1f}deg"
            else:
                label = 'STOP: '+error
        except ValueError as exc:
            label = 'STOP: '+str(exc)
        cv2.putText(image, label[:100], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
        msg = Image()
        msg.header = mask_msg.header
        msg.height, msg.width = image.shape[:2]
        msg.encoding, msg.step, msg.data = 'bgr8', image.shape[1]*3, image.tobytes()
        self.debug_pub.publish(msg)

    def publish_drive_markers(self, start, distance):
        marker = Marker()
        marker.header.frame_id = start.frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'straight_entry', 0, Marker.ARROW, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.points = [Point(x=start.x, y=start.y, z=.10),
                         Point(x=start.x+distance*math.cos(start.yaw),
                               y=start.y+distance*math.sin(start.yaw), z=.10)]
        marker.scale.x, marker.scale.y, marker.scale.z = .04, .10, .15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1., .6, 0., 1.
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = RailEntry()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        if rclpy.ok() and node.busy:
            node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
