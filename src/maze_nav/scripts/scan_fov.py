#!/usr/bin/env python3
"""Измеряет фактический сектор обзора лидара по топику /scan.

Копит N сообщений и для каждого угла считает, как часто луч возвращает
корректную дальность. Лучи, перекрытые конструкцией робота, почти всегда
дают inf/nan/0 — по ним и виден слепой сектор.

    ./scan_fov.py                       # 50 сообщений с /scan
    ./scan_fov.py --topic /scan/filtered --count 100
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class FovProbe(Node):
    def __init__(self, topic, count):
        super().__init__('scan_fov')
        self.count, self.got, self.valid, self.meta = count, 0, None, None
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT   # лидары обычно BEST_EFFORT
        self.create_subscription(LaserScan, topic, self.cb, qos)
        self.get_logger().info(f'слушаю {topic}, нужно {count} сообщений...')

    def cb(self, m):
        if self.valid is None:
            self.valid = [0] * len(m.ranges)
            self.meta = (m.angle_min, m.angle_increment, m.range_min, m.range_max, len(m.ranges))
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                self.valid[i] += 1
        self.got += 1


def report(valid, meta, thresh):
    a_min, a_inc, r_min, r_max, n = meta
    print(f"\nлучей в скане: {n}, шаг {math.degrees(a_inc):.2f}°, "
          f"дальность {r_min:.2f}–{r_max:.1f} м")
    print(f"заявленный сектор: {math.degrees(a_min):.0f}° … "
          f"{math.degrees(a_min + a_inc*(n-1)):.0f}°\n")

    live = [v >= thresh for v in valid]
    # ищем непрерывные рабочие сектора (с учётом замыкания круга)
    segs, start = [], None
    for i, ok in enumerate(live):
        if ok and start is None: start = i
        elif not ok and start is not None: segs.append((start, i-1)); start = None
    if start is not None: segs.append((start, len(live)-1))
    if len(segs) > 1 and live[0] and live[-1]:      # склеить через 0°
        s0, s1 = segs[0], segs.pop()
        segs[0] = (s1[0], s0[1])

    print("РАБОЧИЕ СЕКТОРЫ (угол в кадре лидара):")
    total = 0
    for a, b in segs:
        span = ((b - a) % len(live) + 1) * math.degrees(a_inc)
        total += span
        print(f"  {math.degrees(a_min + a*a_inc):7.1f}° … "
              f"{math.degrees(a_min + b*a_inc):7.1f}°   ширина {span:5.1f}°")
    print(f"\nИТОГО обзор: {total:.0f}° из {math.degrees(a_inc)*len(live):.0f}° "
          f"({total/(math.degrees(a_inc)*len(live))*100:.0f}%)")

    # грубая картинка по 5° на символ
    print("\nкарта (# рабочий луч, · слепой), 0° слева:")
    step = max(1, len(live)//72)
    print('  ' + ''.join('#' if any(live[i:i+step]) else '·'
                         for i in range(0, len(live), step)))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--topic', default='/scan')
    p.add_argument('--count', type=int, default=50, help='сколько сканов накопить')
    p.add_argument('--thresh', type=float, default=0.5,
                   help='доля сканов, в которых луч должен быть валиден (0..1)')
    a = p.parse_args()

    rclpy.init()
    node = FovProbe(a.topic, a.count)
    try:
        while rclpy.ok() and node.got < a.count:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass

    if node.valid is None:
        print("не пришло ни одного скана — проверь топик и ROS_DOMAIN_ID")
    else:
        report(node.valid, node.meta, a.thresh * node.got)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
