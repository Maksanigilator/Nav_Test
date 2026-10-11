#!/usr/bin/env python3
"""Определяет, как развернуть оси ИНС, по двум положениям робота.

Плату MPU6050 можно привинтить двадцатью четырьмя способами, и угадывать,
какой из них совпал с соглашением ROS, бессмысленно. Проще измерить.

ROS ждёт правую систему: X вперёд, Y влево, Z вверх (REP-103). В покое
акселерометр показывает не тяжесть, а противодействующую ей силу, то есть
направление «вверх». Поэтому ровно стоящий датчик обязан давать
`az = +9.81`, а не минус.

Двух положений достаточно. Ровное даёт направление «вверх» в осях
датчика. Поднятый перед даёт горизонтальную добавку, направленную
вперёд. Третья ось получается векторным произведением, и знак её
определён однозначно — доугадывать нечего.

Результат — строка для параметра `axes` узла pico_imu_node, где для
каждой оси ROS (в порядке x, y, z) указано, какая ось датчика её питает
и с каким знаком.
"""
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

NAMES = ('x', 'y', 'z')


class AxisFinder(Node):
    def __init__(self):
        super().__init__('imu_axes')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('samples', 200)
        self.buf = []
        self.create_subscription(
            Imu, self.get_parameter('topic').value, self.on_imu,
            QoSPresetProfiles.SENSOR_DATA.value)

    def on_imu(self, m):
        a = m.linear_acceleration
        self.buf.append((a.x, a.y, a.z))

    def grab(self, label):
        """Среднее по пачке выборок: одиночная выборка слишком шумная."""
        n = int(self.get_parameter('samples').value)
        self.buf.clear()
        while len(self.buf) < n and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        v = np.array(self.buf[-n:]).mean(axis=0)
        print(f'  {label}: [{v[0]:+6.2f} {v[1]:+6.2f} {v[2]:+6.2f}] '
              f'|a|={np.linalg.norm(v):.2f}')
        return v


def nearest_axis(v, taken):
    """Ближайшая к v ось датчика со знаком, из ещё не занятых."""
    order = np.argsort(-np.abs(v))
    for i in order:
        if i not in taken:
            return int(i), (1.0 if v[i] >= 0 else -1.0)
    raise RuntimeError('оси кончились')


def main():
    rclpy.init()
    n = AxisFinder()
    try:
        print('\nПоставь робота РОВНО на горизонтальный пол и нажми Enter.')
        input()
        level = n.grab('ровно    ')
        g = np.linalg.norm(level)
        if g < 8.0 or g > 11.5:
            print(f'\n  Модуль {g:.2f} м/с² далёк от 9.81. Либо датчик '
                  f'двигался, либо шкала в прошивке не та, что в узле.')

        print('\nПриподними ПЕРЕД робота (градусов на пятнадцать-двадцать),')
        print('подержи неподвижно и нажми Enter.')
        input()
        nose = n.grab('перед выше')

        up = level / np.linalg.norm(level)
        # Горизонтальная добавка от наклона и есть направление «вперёд».
        d = nose - level
        fwd = d - np.dot(d, up) * up
        if np.linalg.norm(fwd) < 0.5:
            print('\n  Наклон слишком мал, направление вперёд не выделяется.')
            print('  Подними перед сильнее и повтори.')
            return 1
        fwd /= np.linalg.norm(fwd)

        taken = set()
        ix, sx = nearest_axis(fwd, taken); taken.add(ix)
        iz, sz = nearest_axis(up, taken); taken.add(iz)
        # Y не измеряем, а выводим: левая ось — это z векторно x.
        left = np.cross(up, fwd)
        iy, sy = nearest_axis(left, taken)

        spec = ' '.join(f'{"" if s > 0 else "-"}{NAMES[i]}'
                        for i, s in ((ix, sx), (iy, sy), (iz, sz)))
        print(f'\n  вперёд в осях датчика: [{fwd[0]:+.2f} {fwd[1]:+.2f} '
              f'{fwd[2]:+.2f}]')
        print(f'  вверх  в осях датчика: [{up[0]:+.2f} {up[1]:+.2f} '
              f'{up[2]:+.2f}]')
        print(f'\n  axes:  {spec}\n')
        print('Запускать так:')
        print(f"  ros2 launch maze_nav pico_imu.launch.py axes:='{spec}'")
        print('\nПроверка: при ровном роботе az должно стать +9.8, при')
        print('поднятом переде ax уходит в плюс, при поднятом левом боке —')
        print('ay в плюс.')
        return 0
    except KeyboardInterrupt:
        return 1
    finally:
        n.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
