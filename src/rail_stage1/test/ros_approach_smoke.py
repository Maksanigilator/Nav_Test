"""Isolated-domain test: fake Nav2, no velocity publishers or robot goals."""
import tempfile
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rail_stage1_interfaces.action import ApproachRail
from rail_stage1.approach import RailApproach
from rail_stage1.motion_lease import MotionLease


def wait(future):
    deadline = time.monotonic()+8
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.02)
    assert future.done(), 'test future timed out'
    return future.result()


def main():
    with tempfile.TemporaryDirectory() as root:
        path = root+'/motion.lock'
        rclpy.init(args=['--ros-args', '-p', 'motion_lock_path:='+path])
        approach, client = RailApproach(), Node('fake_nav2_client')
        group = ReentrantCallbackGroup()
        target = [None]
        stopped = [False]
        scenario = ['coarse']
        probe = MotionLease(path)

        def navigate(handle):
            target[0] = handle.request.pose
            if scenario[0] == 'abort':
                handle.abort()
                return NavigateToPose.Result()
            deadline = time.monotonic()+6
            while not handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(.02)
            assert handle.is_cancel_requested, 'coarse completion did not cancel Nav2'
            assert not probe.acquire(), 'lease released before Nav2 stopped'
            time.sleep(.2)  # cancellation acknowledgment is not terminal completion
            assert not probe.acquire(), 'lease released during stop'
            stopped[0] = True
            handle.canceled()
            return NavigateToPose.Result()

        fake = ActionServer(client, NavigateToPose, '/navigate_to_pose', navigate,
                            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

        def pose_update():
            if target[0] is None:
                return
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_footprint'
            transform.header.stamp = client.get_clock().now().to_msg()
            transform.transform.translation.x = target[0].pose.position.x+.10
            transform.transform.translation.y = target[0].pose.position.y
            transform.transform.rotation = target[0].pose.orientation
            approach.tf.set_transform(transform, 'fake_pose')

        timer = client.create_timer(.05, pose_update, callback_group=group)
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(approach)
        executor.add_node(client)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            with approach.lock:
                approach.rails = {'rail_001': {'point': [2., 0., 0.], 'yaw': 0.}}
                approach.registry_received = time.monotonic()
            action = ActionClient(client, ApproachRail, '/rails/approach')
            assert action.wait_for_server(timeout_sec=5)
            handle = wait(action.send_goal_async(ApproachRail.Goal(rail_id='rail_001')))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.status == 4 and result.result.success, result
            assert stopped[0], 'approach success reported before Nav2 stopped'
            assert probe.acquire()
            probe.release()
            scenario[0] = 'abort'
            handle = wait(action.send_goal_async(ApproachRail.Goal(rail_id='rail_001')))
            result = wait(handle.get_result_async())
            assert result.status == 6 and not result.result.success
            print('ROS coarse approach OK: actual pose, cancel, await stop, lease exclusion, Nav2 abort')
        finally:
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            client.destroy_timer(timer)
            fake.destroy()
            approach.server.destroy()
            approach.destroy_node()
            client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
