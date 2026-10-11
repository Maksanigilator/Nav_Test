#!/usr/bin/env python3
"""Замер смещений нуля ИНС на НЕПОДВИЖНОМ роботе.

Поднимать и переворачивать ничего не надо — всё меряется стоя.

ЧТО МЕРЯЕТСЯ И ПОЧЕМУ ЭТО РАЗНЫЕ ВЕЩИ

Гироскоп. Неподвижный датчик обязан показывать ноль, и всё, что он
показывает, — смещение. Оно интегрируется в курс, поэтому и уводит его
на градусы в минуту. Это главная ошибка дешёвой ИНС, и она же легче
всего лечится.

Акселерометр. Здесь честно меряются только X и Y: у ровно стоящего
робота они обязаны быть нулями. Z трогать нельзя — в нём сидит тяжесть,
и отделить смещение от масштаба, не переворачивая датчик, невозможно.

ВАЖНАЯ ОГОВОРКА про X и Y. Если пол под роботом с уклоном, замер
припишет уклон датчику, и робот станет считать «горизонтом» именно эту
поверхность. Для езды по одному помещению это безвредно. Для точных
измерений наклона — нет, тогда нужен настоящий горизонт.

Смещение гироскопа зависит от температуры, поэтому мерить его надо на
прогретом датчике и переснимать, если робот долго стоял холодным.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

G = 9.80665


class Bias(Node):
    def __init__(self):
        super().__init__('imu_bias')
        self.declare_parameter('topic', '/imu/data_raw')
        self.declare_parameter('seconds', 30.0)
        self.a = []
        self.g = []
        self.create_subscription(
            Imu, self.get_parameter('topic').value, self.on_imu,
            QoSPresetProfiles.SENSOR_DATA.value)

    def on_imu(self, m):
        la, av = m.linear_acceleration, m.angular_velocity
        self.a.append((la.x, la.y, la.z))
        self.g.append((av.x, av.y, av.z))


def main():
    rclpy.init()
    n = Bias()
    sec = float(n.get_parameter('seconds').value)
    try:
        print(f'\nРобот должен стоять НЕПОДВИЖНО {sec:.0f} секунд.')
        print('Не облокачивайся на него и не ходи рядом по гулкому полу.')
        input('Готов — нажми Enter.')
        t0 = n.get_clock().now().nanoseconds
        while (n.get_clock().now().nanoseconds - t0) < sec * 1e9 and rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.2)
            k = len(n.g)
            if k and k % 1000 == 0:
                print(f'  собрано {k} выборок', end='\r')
        if len(n.g) < 100:
            print('\nвыборок почти нет — запущен ли pico_imu.launch.py?')
            return 1

        a = np.array(n.a)
        g = np.array(n.g)
        gm, gs = g.mean(0), g.std(0)
        am, asd = a.mean(0), a.std(0)
        print(f'\n\nвыборок {len(g)} за {sec:.0f} с\n')
        print('гироскоп, град/с:')
        for k, nm in enumerate('xyz'):
            print(f'   {nm}: смещение {math.degrees(gm[k]):+7.3f}   '
                  f'шум {math.degrees(gs[k]):6.3f}')
        print('\nакселерометр, м/с²:')
        for k, nm in enumerate('xyz'):
            print(f'   {nm}: среднее  {am[k]:+7.3f}   шум {asd[k]:6.3f}')
        print(f'   |a| = {np.linalg.norm(am):.3f} против 9.807 -> '
              f'масштаб завышен на {100*(np.linalg.norm(am)/G-1):+.1f}%')

        # Движение во время замера портит всё, поэтому проверяем явно.
        if max(abs(math.degrees(v)) for v in gs) > 2.0:
            print('\n  Шум гироскопа великоват — робота задевали? '
                  'Повтори замер.')
        # Уводит курс ТОЛЬКО вертикальная ось. Крен и тангаж фильтр
        # правит по тяжести и уйти им не даёт, а у рысканья опоры нет —
        # отсюда и весь дрейф. Норма всего вектора смещения, которую я
        # печатал раньше, физического смысла не имеет.
        yaw = abs(math.degrees(gm[2])) * 60
        print(f'\nКурс (рысканье) без поправки уходил бы на {yaw:.0f}° '
              f'за минуту.')
        print(f'Крен и тангаж не уходят: их держит тяжесть, даже при '
              f'смещении {math.degrees(gm[0]):+.1f} и '
              f'{math.degrees(gm[1]):+.1f} °/с.')
        res = math.degrees(gs[2]) * 60
        print(f'После поправки остаётся случайное блуждание порядка '
              f'{res:.1f}° за минуту плюс уход смещения от нагрева.')
        print('\nЗапускать так:')
        print(f"  ros2 launch maze_nav pico_imu.launch.py \\\\\n"
              f"      gyro_bias:='[{gm[0]:.6f}, {gm[1]:.6f}, {gm[2]:.6f}]' \\\\\n"
              f"      accel_bias:='[{am[0]:.4f}, {am[1]:.4f}, 0.0]'")
        print('\nТретье число у акселерометра намеренно ноль: в Z сидит')
        print('тяжесть, и отделить смещение от масштаба стоя невозможно.')
        return 0
    except KeyboardInterrupt:
        return 1
    finally:
        n.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
