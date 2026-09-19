#!/usr/bin/env python3
"""Переходник Twist -> TwistStamped для mecanum_drive_controller.

Контроллер из ros2_controllers — цепочечный (ChainableControllerInterface),
и его вход называется `~/reference`, а тип сообщения — TwistStamped.
Источники команд вокруг говорят простым Twist: teleop_twist_keyboard шлёт
Twist в /cmd_vel, Nav2 в Jazzy по умолчанию тоже (enable_stamped_cmd_vel
выключен). Отдельного пакета-переходника в образе нет, поэтому он здесь.

Метка времени не косметическая: у контроллера есть reference_timeout, и
команду со слишком старым штампом он отбрасывает, обнуляя колёса. Поэтому
штамп ставится в момент пересылки, а не берётся из источника.

    ros2 run maze_nav twist_to_stamped.py
    ros2 run maze_nav twist_to_stamped.py --ros-args \
        -r in:=/cmd_vel_nav -r out:=/mecanum_drive_controller/reference
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class TwistToStamped(Node):
    def __init__(self):
        super().__init__('twist_to_stamped')
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value

        # BEST_EFFORT со стороны публикации команд — обычное дело для teleop,
        # а подписка RELIABLE с таким издателем просто не соединится.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, 'out', qos)
        self.create_subscription(Twist, 'in', self.on_twist, qos)
        self.get_logger().info(
            f'{self.resolve_topic_name("in")} -> {self.resolve_topic_name("out")}')

    def on_twist(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
