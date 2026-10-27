"""Isolated ROS domain only: fake perception/odometry, remapped velocity output."""
import tempfile
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rail_stage1_interfaces.action import EnterRail
from rail_stage1.entry import RailEntry


def wait(future, timeout=6):
    deadline = time.monotonic()+timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.02)
    assert future.done(), 'test future timed out'
    return future.result()


def main():
    with tempfile.TemporaryDirectory() as root:
        rclpy.init(args=['--ros-args', '-p', f'motion_lock_path:={root}/motion.lock',
                        '-r', '/cmd_nav_raw:=/test/omni_cmd'])
        entry, client = RailEntry(), Node('fake_entry_inputs')
        aligned = [False]
        commands = []
        group = ReentrantCallbackGroup()
        client.create_subscription(Twist, '/test/omni_cmd', lambda m: commands.append(m), 10,
                                   callback_group=group)

        def update():
            stamp = client.get_clock().now().to_msg()
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.pose.pose.orientation.w = 1.
            entry.on_localization(odom)
            with entry.lock:
                if entry.busy and entry.phase == 'ALIGN':
                    entry.feature = dict(stamp=stamp.sec*10**9+stamp.nanosec, received=time.monotonic(),
                                         error=np.zeros(2), pixel=np.array([423.5, 399.]), target=np.array([423.5, 399.]),
                                         heading_error=0. if aligned[0] else .15,
                                         command=np.zeros(3) if aligned[0] else np.array([.03, .08, .1]))

        timer = client.create_timer(.05, update, callback_group=group)
        executor = MultiThreadedExecutor(num_threads=6)
        for node in (entry, client):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            action = ActionClient(client, EnterRail, '/rails/enter')
            assert action.wait_for_server(timeout_sec=5)
            handle = wait(action.send_goal_async(EnterRail.Goal()))
            assert handle.accepted
            time.sleep(.55)
            assert entry.phase == 'ALIGN', 'point alone incorrectly triggered DRIVE'
            assert any(m.linear.y > .02 for m in commands), 'lateral velocity lost'
            assert any(m.angular.z > .05 for m in commands), 'yaw velocity lost'
            aligned[0] = True
            deadline = time.monotonic()+3
            while entry.phase != 'DRIVE' and time.monotonic() < deadline:
                time.sleep(.02)
            assert entry.phase == 'DRIVE', entry.phase
            time.sleep(.15)
            assert any(m.linear.x == .12 and m.linear.y == 0. and m.angular.z == 0. for m in commands)
            assert wait(handle.cancel_goal_async()).goals_canceling
            result = wait(handle.get_result_async())
            assert result.status == 5 and not result.result.success
            deadline = time.monotonic()+2
            while entry.busy and time.monotonic() < deadline:
                time.sleep(.02)
            time.sleep(.1)
            assert not entry.busy
            assert (commands[-1].linear.x, commands[-1].linear.y, commands[-1].angular.z) == (0., 0., 0.)
            print('ROS omni entry OK: both point and yaw required, vy/wz published, straight DRIVE, cancel stops all axes')
        finally:
            executor.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            client.destroy_timer(timer)
            entry.server.destroy()
            entry.destroy_node()
            client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
