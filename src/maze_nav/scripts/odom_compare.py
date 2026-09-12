#!/usr/bin/env python3
"""Сравнение трёх источников продольной скорости во время заезда.

Робот едет вперёд по линейке, скрипт интегрирует twist.linear.x каждого
источника и показывает, сколько метров каждый из них насчитал:

    /odom              — колёса, откалиброваны по линейке, это эталон масштаба
    /laser_odom        — rf2o, сопоставление сканов
    /odometry/filtered — то, что выдаёт EKF, и во что верит Nav2

Запуск в контейнере, затем проехать вперёд известное расстояние и Ctrl+C:

    python3 /root/ros_ws/src/maze_nav/scripts/odom_compare.py

Расхождение между колёсами и лазером — это и есть отставание робота на карте:
EKF взвешивает источники по ковариации, а у rf2o она в 40 раз «увереннее».
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Track:
    """Интегратор скорости по одному топику."""

    def __init__(self, name):
        self.name = name
        self.dist = 0.0      # интеграл |v| — сколько проехали по скорости
        self.signed = 0.0    # интеграл v — с учётом знака
        self.yaw_int = 0.0   # интеграл угловой скорости, радианы
        self.count = 0
        self.last_t = None
        self.vmax = 0.0
        self.pose_x = None
        self.pose_travel = 0.0
        self._px = self._py = None

    def add(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        if self.last_t is not None:
            dt = t - self.last_t
            # Пропуски связи дают дыры в секунды — такие интервалы не считаем.
            if 0.0 < dt < 0.5:
                self.dist += abs(v) * dt
                self.signed += v * dt
                self.yaw_int += w * dt
        self.last_t = t
        self.count += 1
        self.vmax = max(self.vmax, abs(v))

        # Путь по самой позе — контроль того, что интеграл скорости не врёт.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._px is not None:
            self.pose_travel += ((x - self._px) ** 2 + (y - self._py) ** 2) ** 0.5
        self._px, self._py = x, y


class Compare(Node):
    def __init__(self):
        super().__init__('odom_compare')
        self.tracks = {
            '/odom': Track('колёса   /odom'),
            '/laser_odom': Track('лазер    /laser_odom'),
            '/odometry/filtered': Track('EKF      /odometry/filtered'),
        }
        for topic, track in self.tracks.items():
            self.create_subscription(
                Odometry, topic, lambda m, t=track: t.add(m), 20)
        self.create_timer(1.0, self.report)
        self.get_logger().info('Поехали. Ctrl+C по завершении заезда.')

    def report(self):
        parts = []
        for track in self.tracks.values():
            parts.append(f'{track.name.split()[0]} {track.dist:5.2f}')
        self.get_logger().info(' | '.join(parts))

    def summary(self):
        print('\n' + '=' * 62)
        print(f'{"источник":<28}{"путь м":>9}{"по позе":>10}{"vmax":>8}{"сообщ":>7}')
        print('-' * 62)
        for track in self.tracks.values():
            print(f'{track.name:<28}{track.dist:9.3f}{track.pose_travel:10.3f}'
                  f'{track.vmax:8.3f}{track.count:7d}')
        print('-' * 62)
        wheels = self.tracks['/odom'].dist
        laser = self.tracks['/laser_odom'].dist
        ekf = self.tracks['/odometry/filtered'].dist
        if wheels > 0.05:
            print(f'лазер / колёса : {laser / wheels * 100:6.1f} %')
            print(f'EKF   / колёса : {ekf / wheels * 100:6.1f} %')
        print('=' * 62)


def main():
    rclpy.init()
    node = Compare()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.summary()
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
