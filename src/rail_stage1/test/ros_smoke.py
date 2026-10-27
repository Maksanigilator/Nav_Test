"""Run only in an isolated ROS domain/container: uses a fake Nav2 server."""
import math
import tempfile
import threading
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateToPose
from rail_stage1_interfaces.action import ApproachRail
from rail_stage1.finder import RailFinder
from rail_stage1.approach import RailApproach
from test_stage1 import scene


def wait(future, seconds=8):
    deadline = time.monotonic()+seconds
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.02)
    assert future.done(), 'ROS future timed out'
    return future.result()


def main():
    with tempfile.TemporaryDirectory() as temp:
        rclpy.init(args=['--ros-args', '-p', f'database_path:={temp}/rails.json'])
        finder, approach, client = RailFinder(), RailApproach(), Node('rail_stage1_test_client')
        group = ReentrantCallbackGroup()
        targets = []
        hold = [False]
        def navigate(handle):
            targets.append(handle.request.pose)
            until = time.monotonic()+10
            while hold[0] and not handle.is_cancel_requested and time.monotonic() < until:
                time.sleep(.02)
            result = NavigateToPose.Result()
            handle.canceled() if handle.is_cancel_requested else handle.succeed()
            return result
        fake = ActionServer(client, NavigateToPose, '/navigate_to_pose', navigate,
                            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (finder, approach, client):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            assert not finder.enabled
            service = client.create_client(SetBool, '/rails/search/set_enabled')
            assert service.wait_for_service(timeout_sec=5)
            request = SetBool.Request(data=True)
            assert wait(service.call_async(request)).success
            mask, depth, k, distortion, r, origin = scene()
            angle = math.radians(30)
            # Optical -> base rotation is Rz(-90) * Rx(-120).
            qz = np.array([0., 0., math.sin(-math.pi/4), math.cos(-math.pi/4)])
            qx = np.array([math.sin(-(math.pi/2+angle)/2), 0., 0., math.cos(-(math.pi/2+angle)/2)])
            ax, ay, az, aw = qz
            bx, by, bz, bw = qx
            quat = [aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
                    aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz]
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'base_footprint', 'test_optical'
            transform.transform.translation.z = .6
            q = transform.transform.rotation
            q.x, q.y, q.z, q.w = quat
            finder.tf.set_transform_static(transform, 'test')
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
            transform.transform.translation.x, transform.transform.translation.y = 2., 3.
            transform.transform.rotation.z = math.sin(math.pi/4)
            transform.transform.rotation.w = math.cos(math.pi/4)
            finder.tf.set_transform_static(transform, 'test')
            info = CameraInfo()
            info.header.frame_id = 'test_optical'
            info.width, info.height = 848, 480
            info.k, info.d, info.distortion_model = k.ravel().tolist(), distortion.tolist(), 'plumb_bob'
            finder.on_info(info)
            for _ in range(5):
                stamp = client.get_clock().now().to_msg()
                dm = Image()
                dm.header.frame_id, dm.header.stamp = 'test_optical', stamp
                dm.width, dm.height, dm.encoding, dm.step = 848, 480, '32FC1', 848*4
                dm.data = depth.astype(np.float32).tobytes()
                mm = Image()
                mm.header.frame_id, mm.header.stamp = 'test_optical', stamp
                mm.width, mm.height, mm.encoding, mm.step = 848, 480, 'mono8', 848
                mm.data = mask.tobytes()
                finder.on_depth(dm)
                finder.on_mask(mm)
            assert len(finder.store.rails) == 1, finder.last_reason
            rail = finder.store.rails['rail_001']
            np.testing.assert_allclose(rail['point'], [2., 4., 0.], atol=.08)
            assert abs(rail['yaw']-math.pi/2) < .08
            request.data = False
            assert wait(service.call_async(request)).success
            assert len(finder.store.rails) == 1
            finder.publish_registry()
            deadline = time.monotonic()+5
            while not approach.rails and time.monotonic() < deadline:
                time.sleep(.02)
            action = ActionClient(client, ApproachRail, '/rails/approach')
            assert action.wait_for_server(timeout_sec=5)
            goal = ApproachRail.Goal(rail_id='rail_001')
            gh = wait(action.send_goal_async(goal))
            assert gh.accepted
            result = wait(gh.get_result_async())
            assert result.result.success, result.result.message
            target = targets[-1]
            assert target.header.frame_id == 'map'
            np.testing.assert_allclose([target.pose.position.x, target.pose.position.y], [2., 3.1], atol=.08)
            hold[0] = True
            gh = wait(action.send_goal_async(goal))
            assert gh.accepted
            deadline = time.monotonic()+5
            while len(targets) < 2 and time.monotonic() < deadline:
                time.sleep(.02)
            assert wait(gh.cancel_goal_async()).goals_canceling
            result = wait(gh.get_result_async())
            assert not result.result.success and result.status == 5
            bad = wait(action.send_goal_async(ApproachRail.Goal(rail_id='unknown')))
            assert not bad.accepted
            print('ROS smoke OK: enable/disable, camera TF -> map, persistence, approach -0.90 m, Nav2 success/cancel, unknown ID rejected')
        finally:
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            fake.destroy()
            approach.server.destroy()
            for node in (finder, approach, client):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
