#!/usr/bin/env python3
"""Контроль заезда: едем или стоим.

    ros2 run maze_nav rail_entry_monitor.py

Вопрос, на который отвечает узел, ровно один: ПРОДВИГАЕМСЯ ЛИ ВПЕРЁД,
когда вперёд скомандовано. Этого достаточно, чтобы решить, продолжать
попытку или откатываться и повторять.

Скорость на рельсах знать не нужно. У робота при переходе меняется радиус
качения — с меканума 101.4 мм на дорожку ролика 50 мм, — и линейная
скорость при тех же оборотах падает вдвое. Это любопытно, но для решения
бесполезно: если робот упёрся, неважно, какое у него передаточное.

А вот застревание В ПОЛОЖЕНИИ ЗАЕЗДА — реальный отказ, он случается и на
железе. Робот подкренивается, камера уходит выше, и он замирает, упершись
роликом в трубу. Снаружи это выглядит как обычная езда: колёса крутятся,
команда идёт, ток есть. Отличает только отсутствие продвижения.

Источник истины о продвижении — ЗРЕНИЕ, а не колёса: колёсная одометрия
при буксовании честно рапортует пройденный путь, которого не было.
Поэтому здоровье зрения проверяется отдельно: если оно потеряно, узел
так и говорит, вместо того чтобы делать выводы из мусора. Реакция на
потерю зрения должна быть другой, чем на затык, — остановиться и
осмотреться, а не пятиться вслепую.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String


class RailEntryMonitor(Node):
    def __init__(self):
        super().__init__('rail_entry_monitor')
        # Окно накопления. За 1 с на скорости 0.1 м/с набегает 10 см —
        # столько, чтобы отличить движение от дрожания оценки.
        self.declare_parameter('window_s', 1.0)
        # Какую долю ожидаемого пути считаем движением.
        #
        # На рельсах радиус качения вдвое меньше, и робот идёт примерно
        # 0.34-0.43 от скомандованного — замерено на заезде. Затык даёт
        # ноль. Порог ставим между ними с запасом в обе стороны: 0.15 это
        # вдвое ниже рабочей точки и заметно выше нуля.
        #
        # При 0.30 рабочая точка почти упиралась в порог, и одно колебание
        # оценки давало ложное «упёрлись» посреди нормального заезда.
        self.declare_parameter('progress_frac', 0.15)
        self.declare_parameter('yaw_warn_deg', 10.0)
        self.declare_parameter('min_inliers', 30)

        self.vis = []           # (t, x, y, yaw)
        self.cmd = []           # (t, vx)
        self.yaw0 = None
        self.vo_ok, self.vo_inliers = True, -1

        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, 'visual_odom', self.on_vis, qos)
        self.create_subscription(OdomInfo, 'odom_info', self.on_info, qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, qos)
        self.pub = self.create_publisher(String, 'state', qos)
        self.create_timer(0.2, self.tick)
        self.get_logger().info('контроль заезда: следим за продвижением')

    @staticmethod
    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_info(self, m):
        self.vo_ok, self.vo_inliers = not m.lost, m.inliers

    def on_cmd(self, m):
        self.cmd.append((self.now(), m.linear.x))
        self.trim(self.cmd)

    def on_vis(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.vis.append((t, p.x, p.y, self.yaw_of(m.pose.pose.orientation)))
        self.trim(self.vis)

    def trim(self, buf):
        w = float(self.get_parameter('window_s').value)
        while len(buf) > 2 and buf[-1][0] - buf[0][0] > w:
            buf.pop(0)

    def tick(self):
        if len(self.vis) < 3:
            return
        span = self.vis[-1][0] - self.vis[0][0]
        if span < 0.2:
            return
        # Путь по траектории, а не смещение концов: на дуге концы могут
        # сойтись, а робот при этом ехал.
        moved = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                    for a, b in zip(self.vis, self.vis[1:]))
        if self.yaw0 is None:
            self.yaw0 = self.vis[-1][3]
        dyaw = math.degrees(math.atan2(math.sin(self.vis[-1][3] - self.yaw0),
                                       math.cos(self.vis[-1][3] - self.yaw0)))
        # Сколько пути ОЖИДАЕМ по тому, что сами скомандовали.
        recent = [v for t, v in self.cmd if self.now() - t < span + 0.5]
        want = abs(sum(recent) / len(recent)) * span if recent else 0.0

        min_inl = int(self.get_parameter('min_inliers').value)
        if not self.vo_ok or (0 <= self.vo_inliers < min_inl):
            state = f'ЗРЕНИЕ ПОТЕРЯНО (инлаеров {self.vo_inliers})'
        elif want < 0.01:
            state = 'команды нет'
        elif moved >= want * float(self.get_parameter('progress_frac').value):
            state = 'ЕДЕМ'
        else:
            state = 'НЕ ЕДЕМ — упёрлись'
        if abs(dyaw) > float(self.get_parameter('yaw_warn_deg').value):
            state += f' | КУРС {dyaw:+.1f}°'

        self.pub.publish(String(data=state))
        self.get_logger().info(
            f'{state}: прошли {moved * 1000:5.0f} мм из ожидаемых '
            f'{want * 1000:5.0f}, курс {dyaw:+5.1f}°, инлаеров {self.vo_inliers}',
            throttle_duration_sec=1.0)

    def reset_yaw(self):
        self.yaw0 = self.vis[-1][3] if self.vis else None


def main():
    rclpy.init()
    n = RailEntryMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
