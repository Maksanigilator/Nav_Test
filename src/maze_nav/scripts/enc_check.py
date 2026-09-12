#!/usr/bin/env python3
"""Показывает оба энкодера робота рядом и копит статистику.

Читает /left_motor/encoder/delta и /right_motor/encoder/delta — приращения
тиков между запросами к Arduino (10 Гц). Здоровый канал даёт ровные
положительные дельты при движении вперёд; скачки, нули и отрицательные
значения на ходу означают, что энкодер врёт или теряет контакт.

    ./enc_check.py                  # живая лента, Ctrl+C для итога
    ./enc_check.py --time 20        # копить 20 с и напечатать итог
    ./enc_check.py --quiet          # без ленты, только итог
"""
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Из robot_params.h: WHEEL_DIAMETER 0.067 м, TICKS_PER_REV 370
METERS_PER_TICK = 0.067 * 3.14159265 / 370


class EncCheck(Node):
    def __init__(self, quiet):
        super().__init__('enc_check')
        self.quiet = quiet
        self.d = {'L': [], 'R': []}
        self.create_subscription(Int32, '/left_motor/encoder/delta',
                                 lambda m: self.cb('L', m), 50)
        self.create_subscription(Int32, '/right_motor/encoder/delta',
                                 lambda m: self.cb('R', m), 50)
        self.get_logger().info('слушаю энкодеры, Ctrl+C для итога')

    def cb(self, side, msg):
        self.d[side].append(msg.data)
        if self.quiet:
            return
        # печатаем парами: как только у обоих колёс накопилось поровну
        n = min(len(self.d['L']), len(self.d['R']))
        if n and len(self.d[side]) == n:
            l, r = self.d['L'][n - 1], self.d['R'][n - 1]
            flag = '  <-- РАСХОЖДЕНИЕ' if abs(l - r) > max(5, 0.3 * max(abs(l), abs(r))) else ''
            print(f'L {l:+5d}   R {r:+5d}{flag}')


def report(d):
    print('\n' + '=' * 46)
    for side in ('L', 'R'):
        a = d[side]
        if not a:
            print(f'{side}: НЕТ ДАННЫХ — проверь, запущен ли arduino_bridge')
            continue
        nz = [x for x in a if x]
        neg = [x for x in a if x < 0]
        print(f'{side}: пакетов {len(a)}, сумма {sum(a):+d} тиков '
              f'({sum(a) * METERS_PER_TICK:+.3f} м)')
        print(f'   дельты от {min(a):+d} до {max(a):+d}, '
              f'нулевых {len(a) - len(nz)}, отрицательных {len(neg)}')
    L, R = sum(d['L']), sum(d['R'])
    if L and R:
        print(f'\nотношение R/L = {R / L:.2f}  (норма около 1.00 при езде прямо)')
        # WHEEL_BASE 0.18 м из robot_params.h
        import math
        print(f'поворот по энкодерам: {math.degrees((R - L) * METERS_PER_TICK / 0.18):+.1f}°')
    print('=' * 46)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--time', type=float, default=0.0,
                   help='сколько секунд копить (0 — до Ctrl+C)')
    p.add_argument('--quiet', action='store_true', help='без живой ленты')
    a = p.parse_args()

    rclpy.init()
    node = EncCheck(a.quiet)
    import time
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if a.time and time.time() - t0 >= a.time:
                break
    except KeyboardInterrupt:
        pass
    report(node.d)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
