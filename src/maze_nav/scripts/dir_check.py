#!/usr/bin/env python3
"""Проверка знаков: куда робот едет и куда крутится по команде.

Робот сообщает, что на команду «вперёд» уезжает назад. Если это так, то
вся навигация обречена: Nav2 отправляет его к точке, робот уходит в
противоположную сторону, ошибка растёт, и контроллер бесконечно
доворачивается, вместо того чтобы ехать.

Скрипт подаёт короткие заведомо известные команды и печатает, что на них
ответили датчики. Сравнивать надо с тем, что видно глазами.

    ros2 run maze_nav dir_check.py              вперёд и поворот
    ros2 run maze_nav dir_check.py --speed 0.06 медленнее

ВАЖНО: перед запуском остановить маршрут, иначе навигация будет
соперничать за /cmd_vel и результат окажется бессмысленным.
"""
import argparse
import math
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DirCheck(Node):
    def __init__(self, speed, turn, secs):
        super().__init__('dir_check')
        self.speed, self.turn, self.secs = speed, turn, secs
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vx = []
        self.wz = []
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 20)

    def on_odom(self, m):
        self.vx.append(m.twist.twist.linear.x)
        self.wz.append(m.twist.twist.angular.z)

    def drive(self, vx, wz, label):
        self.vx.clear()
        self.wz.clear()
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        end = time.monotonic() + self.secs
        while time.monotonic() < end:
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pub.publish(Twist())          # стоп
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        mvx = sum(self.vx) / len(self.vx) if self.vx else 0.0
        mwz = sum(self.wz) / len(self.wz) if self.wz else 0.0
        print(f'\n{label}')
        print(f'  команда:  vx={vx:+.3f}  wz={wz:+.3f}')
        print(f'  датчики:  vx={mvx:+.3f}  wz={mwz:+.3f}')
        if abs(vx) > 0.01:
            print('  продольно: ' + ('знак совпал' if vx * mvx > 0
                                     else 'ЗНАК ПРОТИВОПОЛОЖЕН'))
        if abs(wz) > 0.01:
            print('  вращение:  ' + ('знак совпал' if wz * mwz > 0
                                     else 'ЗНАК ПРОТИВОПОЛОЖЕН'))
        print('  смотри глазами: робот поехал туда, куда просили?')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--speed', type=float, default=0.08, help='м/с для пробы вперёд')
    p.add_argument('--turn', type=float, default=0.5, help='рад/с для пробы поворота')
    p.add_argument('--secs', type=float, default=2.0, help='длительность каждой пробы')
    a, _ = p.parse_known_args()

    rclpy.init()
    node = DirCheck(a.speed, a.turn, a.secs)
    try:
        print('Пробы по 2 секунды. Смотри на робота.')
        node.drive(a.speed, 0.0, '=== ВПЕРЁД (ожидаем движение вперёд) ===')
        time.sleep(1.0)
        node.drive(0.0, a.turn, '=== ПОВОРОТ ВЛЕВО (против часовой, ожидаем влево) ===')
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.pub.publish(Twist())
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
