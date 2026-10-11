#!/usr/bin/env python3
"""Последний узел перед роботом: разворот знака и сторожевой таймер.

ЗАЧЕМ ОН ЕСТЬ. Привод робота понимает угловую скорость ПРОТИВОПОЛОЖНО
соглашению ROS, где плюс — против часовой стрелки. Замерено 6 октября
2026: команда angular.z = +0.1 рад/с развернула робота на -6.03 градуса
за две секунды, и физически он тоже ушёл вправо — то есть врёт не
одометрия, а привод.

Для навигации это разрушительно. Nav2 видит, что надо довернуть влево,
шлёт плюс, робот уходит вправо, ошибка растёт, Nav2 шлёт больше — и
робот отворачивается от цели тем сильнее, чем старательнее его туда
ведут.

ПОЧЕМУ ЛЕЧИМ ЗДЕСЬ, А НЕ НА РОБОТЕ. Починка там есть и она невелика:
angular_z_scale в odroid_driver.yaml поставить со знаком минус, а в
настройке пульта знак развернуть обратно, чтобы он остался прежним на
ощупь. Но это правка на чужой стороне, которую потом надо помнить, и
пульт при ошибке в одной из двух строк станет зеркальным. Робота решено
не трогать до того, как хозяин возьмётся за его логику сам.

КАК УБРАТЬ, КОГДА РОБОТА ПОЧИНЯТ. Поставить angular_z_sign в +1.0. Всё,
больше искать нечего: разворот живёт в одном месте и называется честно.

СТОРОЖЕВОЙ ТАЙМЕР. Связь с роботом идёт по Wi-Fi, и она небыстрая:
замерено 105-343 мс кругового хода при среднем 210, с потерями. Если
источник команд замолчит — упал узел, оборвалась сеть, человек нажал
Ctrl-C, — робот не должен уехать с последней принятой скоростью. После
cmd_timeout без сообщений сюда уходит ноль, и уходит НЕСКОЛЬКО РАЗ: одна
датаграмма может потеряться, а цена потери здесь — уехавший робот.
"""
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CmdSign(Node):
    def __init__(self):
        super().__init__('cmd_sign')
        p = self.declare_parameter
        p('cmd_in', 'cmd_nav_raw')
        p('cmd_out', 'cmd_nav')
        # Разворот по осям. Сейчас перевёрнут только поворот; линейные
        # оси оставлены параметрами, чтобы при следующей такой находке
        # не заводить второй узел.
        p('angular_z_sign', -1.0)
        p('linear_x_sign', 1.0)
        p('linear_y_sign', 1.0)
        p('cmd_timeout', 0.3)
        # Сколько нулей слать по молчанию. Один может не дойти.
        p('stop_repeats', 5)

        self.sx = float(self.get_parameter('linear_x_sign').value)
        self.sy = float(self.get_parameter('linear_y_sign').value)
        self.sz = float(self.get_parameter('angular_z_sign').value)
        self.left = 0

        self.pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_out').value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_in').value), self.on_cmd, 10)
        self.last = None
        self.create_timer(0.05, self.watch)
        self.get_logger().info(
            f'разворот знака: x {self.sx:+.0f} y {self.sy:+.0f} '
            f'поворот {self.sz:+.0f}; сторож '
            f'{float(self.get_parameter("cmd_timeout").value):.2f} с')

    def on_cmd(self, m):
        o = Twist()
        o.linear.x = self.sx * m.linear.x
        o.linear.y = self.sy * m.linear.y
        o.linear.z = m.linear.z
        o.angular.x = m.angular.x
        o.angular.y = m.angular.y
        o.angular.z = self.sz * m.angular.z
        self.pub.publish(o)
        self.last = self.get_clock().now()
        self.left = int(self.get_parameter('stop_repeats').value)

    def watch(self):
        if self.last is None or self.left <= 0:
            return
        age = (self.get_clock().now() - self.last).nanoseconds * 1e-9
        if age < float(self.get_parameter('cmd_timeout').value):
            return
        self.pub.publish(Twist())
        self.left -= 1
        if self.left == int(self.get_parameter('stop_repeats').value) - 1:
            self.get_logger().warn(
                f'команд нет {age:.2f} с — отправлен ноль',
                throttle_duration_sec=5.0)


def main():
    rclpy.init()
    n = CmdSign()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Уходя, гасим скорость: узел может сниматься и на ходу.
        try:
            for _ in range(5):
                n.pub.publish(Twist())
        except Exception:
            pass
        n.destroy_node()


if __name__ == '__main__':
    sys.exit(main() or 0)
