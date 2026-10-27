"""Independent approach action. Delegates navigation to Nav2, no entry or cmd_vel."""
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rail_stage1_interfaces.action import ApproachRail
from .store import approach_pose
from .motion_lease import MotionLease
from .approach_control import ApproachGate


class RailApproach(Node):
    def __init__(self):
        super().__init__('rail_approach')
        self.distance = self.declare_parameter('approach_distance', .90).value
        self.timeout = self.declare_parameter('timeout_sec', 120.).value
        self.registry_timeout = self.declare_parameter('registry_timeout_sec', 5.).value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.xy_tolerance = self.declare_parameter('xy_tolerance', .15).value
        self.yaw_tolerance = self.declare_parameter('yaw_tolerance_deg', 10.).value
        self.pose_dwell = self.declare_parameter('pose_dwell_sec', .3).value
        self.pose_max_age = self.declare_parameter('pose_max_age_sec', .5).value
        ApproachGate(self.xy_tolerance, self.yaw_tolerance, self.pose_dwell)
        if not math.isfinite(self.pose_max_age) or self.pose_max_age <= 0:
            raise ValueError('pose_max_age_sec must be finite and positive')
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        action = self.declare_parameter('nav_action', '/navigate_to_pose').value
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.navigation_unresolved = False
        self.motion = MotionLease(self.declare_parameter('motion_lock_path', '/tmp/aida_nav_vlad_motion.lock').value)
        self.rails = {}
        self.registry_received = -float('inf')
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/rails/registry', self.on_registry, durable,
                                 callback_group=self.group)
        self.target_pub = self.create_publisher(PoseStamped, '/rails/approach_target', durable)
        self.nav = ActionClient(self, NavigateToPose, action, callback_group=self.group)
        self.server = ActionServer(self, ApproachRail, '/rails/approach', self.execute,
                                  goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                  callback_group=self.group)

    def on_registry(self, message):
        try:
            data = json.loads(message.data)
            if data['frame'] != 'map':
                raise ValueError('registry must be in map')
            with self.lock:
                self.rails = {rail['id']: rail for rail in data['rails']}
                self.registry_received = time.monotonic()
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().error(str(exc))

    def goal(self, request):
        with self.lock:
            if self.busy or self.navigation_unresolved or request.rail_id not in self.rails:
                return GoalResponse.REJECT
            if time.monotonic()-self.registry_received > self.registry_timeout:
                return GoalResponse.REJECT
            if not self.motion.acquire():
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def wait_future(self, future, deadline, handle=None):
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            if handle is not None and handle.is_cancel_requested:
                return False
            time.sleep(.02)
        return future.done()

    def cancel_nav(self, nav_handle):
        future = nav_handle.cancel_goal_async()
        if not self.wait_future(future, time.monotonic()+5.):
            self.get_logger().error('Nav2 cancellation acknowledgement timed out')
            return False
        return bool(future.result().goals_canceling)

    def pose_error(self, target, yaw):
        """Use actual map pose, never path length or the registry's moving estimate."""
        try:
            transform = self.tf.lookup_transform('map', self.base_frame, Time())
            stamp = Time.from_msg(transform.header.stamp).nanoseconds
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            if not 0 <= age <= self.pose_max_age:
                return None
            p, q = transform.transform.translation, transform.transform.rotation
            actual_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            return math.hypot(p.x-target.pose.position.x, p.y-target.pose.position.y), actual_yaw-yaw, stamp
        except tf2_ros.TransformException:
            return None

    def execute(self, handle):
        result = ApproachRail.Result()
        nav_handle = None
        terminal = False
        try:
            with self.lock:
                rail = dict(self.rails[handle.request.rail_id])
            point, yaw = approach_pose(rail, self.distance)
            target = PoseStamped()
            target.header.frame_id = 'map'
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y = float(point[0]), float(point[1])
            target.pose.orientation.z, target.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.target_pub.publish(target)
            if not self.nav.wait_for_server(timeout_sec=5.):
                raise RuntimeError('Nav2 action server unavailable')
            if handle.is_cancel_requested:
                handle.canceled()
                result.message = 'approach canceled before navigation'
                return result
            goal = NavigateToPose.Goal()
            goal.pose = target
            def feedback(message):
                fb = ApproachRail.Feedback()
                fb.phase = 'APPROACH'
                fb.distance_remaining = message.feedback.distance_remaining
                handle.publish_feedback(fb)
            future = self.nav.send_goal_async(goal, feedback_callback=feedback)
            if not self.wait_future(future, time.monotonic()+10.):
                # A late acceptance must not leave a navigation goal running.
                future.add_done_callback(lambda f: f.result().cancel_goal_async() if f.result().accepted else None)
                self.navigation_unresolved = True
                raise RuntimeError('Nav2 goal acknowledgement timed out; late goal will be canceled')
            nav_handle = future.result()
            if not nav_handle.accepted:
                raise RuntimeError('Nav2 rejected approach goal')
            nav_result = nav_handle.get_result_async()
            deadline = time.monotonic()+self.timeout
            gate = ApproachGate(self.xy_tolerance, self.yaw_tolerance, self.pose_dwell)
            coarse_reached = False
            self.get_logger().info('APPROACH started: coarse tolerance %.2f m, %.1f deg' %
                                   (self.xy_tolerance, self.yaw_tolerance))
            while not nav_result.done() and rclpy.ok() and time.monotonic() < deadline:
                if handle.is_cancel_requested:
                    break
                error = self.pose_error(target, yaw)
                if error is None:
                    gate.reset()
                elif gate.observe(*error, time.monotonic()):
                    coarse_reached = True
                    break
                time.sleep(.05)
            if coarse_reached and not handle.is_cancel_requested:
                # Nav2 must finish cancellation before releasing the motion lease.
                self.cancel_nav(nav_handle)
                terminal = self.wait_future(nav_result, time.monotonic()+5.)
                if not terminal:
                    self.navigation_unresolved = True
                    raise RuntimeError('coarse pose reached but Nav2 stop unconfirmed; further approaches blocked')
                error = self.pose_error(target, yaw)
                status = nav_result.result().status
                if handle.is_cancel_requested:
                    handle.canceled()
                    result.message = 'approach canceled'
                    return result
                if status not in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                    raise RuntimeError('Nav2 failed while stopping at the coarse pose')
                if error is None or not gate.contains(*error[:2]):
                    raise RuntimeError('robot left coarse tolerance while Nav2 was stopping')
                result.success = True
                result.message = 'coarse pre-entry pose reached; Nav2 stopped; IBVS was not started'
                self.get_logger().info(result.message)
                handle.succeed()
                return result
            if handle.is_cancel_requested or not nav_result.done():
                acknowledged = self.cancel_nav(nav_handle)
                self.wait_future(nav_result, time.monotonic()+5.)
                terminal = nav_result.done()
                result.message = ('approach canceled' if handle.is_cancel_requested else 'approach timed out')
                if not acknowledged and not terminal:
                    result.message += '; Nav2 cancellation unconfirmed'
                if not terminal:
                    self.navigation_unresolved = True
                    result.message += '; further approaches blocked until this module is restarted'
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                return result
            terminal = True
            result.success = nav_result.result().status == GoalStatus.STATUS_SUCCEEDED
            result.message = 'pre-entry pose reached; IBVS was not started' if result.success else 'Nav2 approach failed'
            handle.succeed() if result.success else handle.abort()
            return result
        except Exception as exc:
            if nav_handle is not None and nav_handle.accepted and not terminal:
                self.cancel_nav(nav_handle)
                finished = nav_handle.get_result_async()
                self.navigation_unresolved = not self.wait_future(finished, time.monotonic()+5.)
            result.message = str(exc)
            handle.abort()
            return result
        finally:
            with self.lock:
                self.busy = False
                if not self.navigation_unresolved:
                    self.motion.release()


def main(args=None):
    rclpy.init(args=args)
    node = RailApproach()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
