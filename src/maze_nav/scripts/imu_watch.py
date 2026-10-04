#!/usr/bin/env python3
"""Живой просмотр ИНС: показывает, куда смотрят оси платы прямо сейчас.

Нужен, когда плату держишь в руках и надо понять, где у неё X, Y и Z и
куда у каждой положительное направление. Маркировки на дешёвых платах
обычно нет, а гадать незачем — это видно из данных.

Правило простое. Акселерометр в покое меряет не тяжесть, а
противодействующую ей силу, то есть направление «ВВЕРХ». Поэтому ось,
показывающая +9.8, прямо сейчас смотрит вверх. Повернул плату — смотри,
какая ось подхватила.

С гироскопом так же, только про вращение: знак положителен, когда
вращение идёт против часовой стрелки, если смотреть СО СТОРОНЫ
положительного конца оси. Покрути плату вокруг каждого ребра и увидишь,
какая ось отзывается и каким знаком.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

NAMES = ('X', 'Y', 'Z')
G = 9.80665


class Watch(Node):
    def __init__(self):
        super().__init__('imu_watch')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('rate', 5.0)
        # Сглаживание: на 200 Гц мгновенное значение дрожит и читать его
        # глазами невозможно. Берём бегущее среднее за полсекунды.
        self.declare_parameter('average_s', 0.5)
        self.a = []
        self.g = []
        self.create_subscription(
            Imu, self.get_parameter('topic').value, self.on_imu,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_timer(1.0 / float(self.get_parameter('rate').value),
                          self.show)
        self.n = max(1, int(200 * float(self.get_parameter('average_s').value)))
        print('\nКрути плату и смотри. Ctrl-C чтобы выйти.\n')

    def on_imu(self, m):
        la, av = m.linear_acceleration, m.angular_velocity
        self.a.append((la.x, la.y, la.z))
        self.g.append((av.x, av.y, av.z))
        del self.a[:-self.n]
        del self.g[:-self.n]

    def show(self):
        if not self.a:
            print('данных нет — запущен ли pico_imu.launch.py?')
            return
        a = [sum(r[k] for r in self.a) / len(self.a) for k in range(3)]
        g = [math.degrees(sum(r[k] for r in self.g) / len(self.g))
             for k in range(3)]

        mod = math.sqrt(sum(v * v for v in a))
        # Ось «вверх» ищем, только когда плата лежит спокойно: в движении
        # акселерометр меряет не одну тяжесть, и вывод был бы враньём.
        spin = max(abs(v) for v in g)
        if abs(mod - G) > 1.5 or spin > 20:
            up = 'плата движется — положи и подожди'
        else:
            k = max(range(3), key=lambda i: abs(a[i]))
            if abs(a[k]) < 0.8 * mod:
                up = 'плата стоит косо, выровняй по грани'
            else:
                up = f'ВВЕРХ смотрит {"+" if a[k] > 0 else "-"}{NAMES[k]}'

        k = max(range(3), key=lambda i: abs(g[i]))
        rot = (f'вращение вокруг {"+" if g[k] > 0 else "-"}{NAMES[k]}'
               if spin > 20 else '')

        print(f'ускорение  X{a[0]:+7.2f}  Y{a[1]:+7.2f}  Z{a[2]:+7.2f}   '
              f'|a|{mod:6.2f}   {up}')
        print(f'гироскоп   X{g[0]:+7.1f}  Y{g[1]:+7.1f}  Z{g[2]:+7.1f}   '
              f'град/с      {rot}')
        print('-' * 78)


def main():
    rclpy.init()
    n = Watch()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print()
    finally:
        n.destroy_node()


if __name__ == '__main__':
    main()
