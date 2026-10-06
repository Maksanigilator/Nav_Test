#!/usr/bin/env python3
"""Живой угол наклона камеры по её собственному акселерометру.

Самый простой из возможных замеров: ни глубины, ни карты, ни TF, ни URDF.
Неподвижный акселерометр меряет направление «вверх», и этого достаточно,
чтобы сказать, под каким углом к горизонту смотрит камера.

ОСИ. Акселерометр D435i отдаёт данные в оптическом соглашении: X вправо,
Y вниз, Z вперёд по взгляду. Отсюда:

    камера смотрит горизонтально  ->  тяжесть по +Y,  a = (0, g, 0)
    камера смотрит строго вниз    ->  тяжесть по +Z,  a = (0, 0, g)

Значит угол наклона ниже горизонта есть atan2(Az, Ay), и больше ничего
знать не нужно.

ЧЕГО ЭТОТ ЗАМЕР НЕ ЗНАЕТ. Он меряет наклон относительно ТЯЖЕСТИ, а в URDF
записан наклон относительно РОБОТА. Совпадают они ровно настолько,
насколько ровно стоит сам робот: его перекос входит в результат как
ошибка. При роботе, выставленном с точностью до полутора градусов, такой
же будет и точность угла.

Способ без этого изъяна — сравнить вертикаль по тяжести с нормалью к
поверхности, на которой робот стоит; это делает cam_level.py. Но там
нужна глубина и ровная площадка перед роботом.
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.80665


class CamPitch(Node):
    def __init__(self):
        super().__init__('cam_pitch')
        p = self.declare_parameter
        p('topic', '/camera/accel/sample')
        # Целевой угол: то, что сейчас записано в URDF. Подсказка «куда
        # крутить» считается относительно него.
        p('target', 50.1)
        p('average_s', 1.0)
        p('rate', 2.0)
        self.buf = []
        self.create_subscription(
            Imu, str(self.get_parameter('topic').value), self.on_imu,
            qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter('rate').value),
                          self.show)
        self.t0 = time.monotonic()
        print('\nДержи камеру неподвижно. Ctrl-C чтобы выйти.\n')

    def on_imu(self, m):
        a = m.linear_acceleration
        self.buf.append((a.x, a.y, a.z))
        # Бегущее среднее: мгновенное значение дрожит, и читать его
        # глазами, подкручивая кронштейн, невозможно.
        n = max(1, int(200 * float(self.get_parameter('average_s').value)))
        del self.buf[:-n]

    def show(self):
        if len(self.buf) < 10:
            if time.monotonic() - self.t0 > 6:
                print('данных нет — включена ли ИНС камеры (cam_imu:=true)?')
            return
        a = np.array(self.buf).mean(axis=0)
        g = float(np.linalg.norm(a))
        # Наклон ниже горизонта и завал набок.
        pitch = math.degrees(math.atan2(-a[2], -a[1]))
        roll = math.degrees(math.atan2(a[0], math.hypot(a[1], a[2])))
        tgt = float(self.get_parameter('target').value)
        d = pitch - tgt

        # Модуль проверяем всегда: если он не около 9.81, камера движется
        # либо шкала не та, и угол тогда неверен.
        bad = '' if abs(g - G) < 0.6 else '   ВНИМАНИЕ: |a| не 9.81, камера движется?'
        if abs(d) < 0.3:
            hint = 'ПОПАЛИ'
        elif d > 0:
            hint = f'наклонена слишком ВНИЗ на {d:.1f}° — поднять нос'
        else:
            hint = f'задрана ВВЕРХ на {-d:.1f}° — опустить нос'
        print(f'наклон {pitch:6.2f}°   завал набок {roll:+5.2f}°   '
              f'|a| {g:5.2f}   цель {tgt:.1f}°   {hint}{bad}')


def main():
    rclpy.init()
    n = CamPitch()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        print()
    finally:
        n.destroy_node()


if __name__ == '__main__':
    sys.exit(main() or 0)
