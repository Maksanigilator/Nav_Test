#!/usr/bin/env python3
"""Калибровка колёсной одометрии по известной дистанции.

Считает тики обоих энкодеров за заезд и вычисляет, сколько метров
приходится на тик — отдельно для левого и правого колеса. Раздельно,
потому что колёса у Frob дают разное число тиков на оборот: в прошивке
arduino_new стоят 442 и 374, то есть расходятся на 18%, а odom.py
пользуется одним общим коэффициентом и потому уводит робота в сторону.

Как мерить прямую (meters_per_tick):
  1. поставить робота, отметить точку под его осью колёс;
  2. запустить: ./calib_odom.py --distance 2.0
  3. проехать телеопом ровно вперёд, остановиться;
  4. Ctrl+C и измерить линейкой фактически пройденное расстояние;
  5. скрипт напечатает коэффициенты.

Как мерить поворот (wheel_base):
  1. ./calib_odom.py --angle 360
  2. развернуть робота на месте ровно на один оборот;
  3. Ctrl+C.

Чем длиннее заезд, тем точнее: на двух метрах ошибка измерения линейкой
в сантиметр даёт полпроцента, на полуметре — два.
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class Calib(Node):
    def __init__(self):
        super().__init__('calib_odom')
        self.left = self.right = 0
        self.lmsgs = self.rmsgs = 0
        self.create_subscription(Int32, '/left_motor/encoder/delta',
                                 self.cb_l, 50)
        self.create_subscription(Int32, '/right_motor/encoder/delta',
                                 self.cb_r, 50)

    def cb_l(self, m):
        self.left += m.data
        self.lmsgs += 1

    def cb_r(self, m):
        self.right += m.data
        self.rmsgs += 1


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--distance', type=float,
                   help='фактически пройденное расстояние, м (для meters_per_tick)')
    p.add_argument('--angle', type=float,
                   help='фактический угол разворота на месте, градусов (для wheel_base)')
    p.add_argument('--mpt', type=float, default=0.00057,
                   help='текущий meters_per_tick, нужен при расчёте wheel_base')
    a, _ = p.parse_known_args()

    if not a.distance and not a.angle:
        p.error('укажи --distance или --angle')

    rclpy.init()
    n = Calib()
    print('СЧИТАЮ ТИКИ. Проедь нужный отрезок и нажми Ctrl+C')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

    L, R = n.left, n.right
    print(f"\nтиков: левое {L:+d} ({n.lmsgs} сообщений), правое {R:+d} ({n.rmsgs})")
    if L == 0 or R == 0:
        print("нет данных — проверь, запущен ли arduino_bridge")
        _finish(n)
        return

    if a.distance:
        mpt_l = a.distance / abs(L)
        mpt_r = a.distance / abs(R)
        print(f"\nпройдено по линейке: {a.distance} м")
        print(f"  левое  колесо: {mpt_l:.6f} м/тик")
        print(f"  правое колесо: {mpt_r:.6f} м/тик")
        print(f"  расхождение колёс: {abs(L - R) / max(abs(L), abs(R)) * 100:.1f}%")
        print(f"  среднее: {(mpt_l + mpt_r) / 2:.6f} м/тик")
        print("\nв odometry_params.yaml (если колёса близки — хватит среднего):")
        print(f"    meters_per_tick: {(mpt_l + mpt_r) / 2:.6f}")
        if abs(mpt_l - mpt_r) / max(mpt_l, mpt_r) > 0.05:
            print("\nКолёса расходятся больше чем на 5% — одним коэффициентом")
            print("их не описать, нужны раздельные (см. правку odom.py).")

    if a.angle:
        # при развороте на месте колёса идут в разные стороны:
        # база = путь колеса / (угол в радианах / 2)
        rad = math.radians(a.angle)
        dist = (abs(L) + abs(R)) / 2.0 * a.mpt
        base = 2.0 * dist / rad
        print(f"\nразворот по факту: {a.angle}°  ({rad:.3f} рад)")
        print(f"  средний путь колеса: {dist:.4f} м при mpt={a.mpt}")
        print(f"\nв odometry_params.yaml:")
        print(f"    wheel_base: {base:.4f}")

    _finish(n)


def _finish(node):
    # Ctrl+C уже мог закрыть контекст: повторный shutdown бросает RCLError
    # и портит вывод результатов, ради которых всё и затевалось.
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
