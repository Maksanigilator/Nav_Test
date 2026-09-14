#!/usr/bin/env python3
"""Где препятствия по мнению робота: скан, пересчитанный в base_footprint.

Спор о развороте кадра laser нельзя решить, глядя на углы скана: они
заданы в собственном кадре лидара, а костмапы видят их уже после TF.
Скрипт делает ровно то же, что костмапа, и печатает результат по
секторам относительно направления движения.

    ros2 run maze_nav scan_sectors.py
    ros2 run maze_nav scan_sectors.py --topic /scan/filtered

Читать так: «вперёд» — то, куда робот поедет по команде linear.x > 0.
Если там стоит препятствие в десятке сантиметров, а физически перед
роботом пусто, значит кадр действительно развёрнут.
"""
import argparse
import math

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


SECTORS = [
    ('вперёд      ', -22.5, 22.5),
    ('вперёд-влево', 22.5, 67.5),
    ('влево       ', 67.5, 112.5),
    ('назад-влево ', 112.5, 157.5),
    ('назад       ', 157.5, -157.5),
    ('назад-вправо', -157.5, -112.5),
    ('вправо      ', -112.5, -67.5),
    ('вперёд-вправо', -67.5, -22.5),
]


def in_sector(a, lo, hi):
    if lo <= hi:
        return lo <= a < hi
    return a >= lo or a < hi          # сектор через 180


class ScanSectors(Node):
    def __init__(self, topic, count):
        super().__init__('scan_sectors')
        self.count = count
        self.got = 0
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(LaserScan, topic, self.cb, qos)
        self.acc = {name: [] for name, _, _ in SECTORS}
        self.get_logger().info(f'слушаю {topic}, нужно {count} сканов')

    def cb(self, m):
        if self.got >= self.count:
            return
        try:
            # Тот же пересчёт, что делает костмапа: из кадра скана в базу.
            tr = self.buf.lookup_transform('base_footprint', m.header.frame_id,
                                           rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'нет TF: {exc}')
            return
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        tx, ty = tr.transform.translation.x, tr.transform.translation.y

        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
                continue
            a = m.angle_min + i * m.angle_increment
            x = tx + r * math.cos(a + yaw)
            y = ty + r * math.sin(a + yaw)
            ang = math.degrees(math.atan2(y, x))
            d = math.hypot(x, y)
            for name, lo, hi in SECTORS:
                if in_sector(ang, lo, hi):
                    self.acc[name].append(d)
                    break
        self.got += 1

    def done(self):
        return self.got >= self.count

    def report(self):
        print('\nПрепятствия в кадре base_footprint '
              f'(по {self.got} сканам):\n')
        print(f'{"сектор":<16}{"ближайшее":>11}{"среднее":>10}{"лучей":>8}')
        print('-' * 46)
        for name, _, _ in SECTORS:
            v = self.acc[name]
            if not v:
                print(f'{name:<16}{"— пусто —":>11}{"":>10}{0:>8}')
                continue
            print(f'{name:<16}{min(v):11.2f}{sum(v)/len(v):10.2f}{len(v):8d}')
        print('-' * 46)
        print('Сравни с тем, что стоит вокруг робота на самом деле.')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topic', default='/scan')
    p.add_argument('--count', type=int, default=10)
    a, _ = p.parse_known_args()

    rclpy.init()
    node = ScanSectors(a.topic, a.count)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.report()
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
