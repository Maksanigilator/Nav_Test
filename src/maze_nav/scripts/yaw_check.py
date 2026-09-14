#!/usr/bin/env python3
"""Проверка курса на повороте: сверяет три источника с реальностью.

Робот теряется на поворотах, хотя прямые после калибровки идут ровно.
Курс в EKF берётся не от колёс: в ekf.yaml у них включена только Vx.
Значит виноват либо гироскоп, либо то, как его данные попадают в фильтр,
и надо смотреть на источники по отдельности.

Скрипт копит три величины за время замера:

    EKF        — курс из /odometry/filtered, им пользуется Nav2;
    гироскоп   — интеграл angular_velocity.z из /imu/mpu6050;
    колёса     — интеграл angular_velocity.z из /odom, считается по
                 разности путей колёс через wheel_base.

Как пользоваться: запустить, развернуть робота ровно на 90 или 180
градусов (лучше по разметке или угольнику), нажать Ctrl+C и сравнить
показания с тем, на сколько повернули на самом деле.

    ros2 run maze_nav yaw_check.py

Что означают расхождения:

    все три занижают одинаково   — масштаб гироскопа, лечится настройкой
                                   gyro_fs_sel или множителем в драйвере;
    колёса врут, гироскоп прав   — неверный wheel_base (сейчас 0.18 из
                                   URDF, никем не проверен);
    гироскоп прав, EKF занижает  — фильтр не доверяет гироскопу или
                                   подмешивает неверную абсолютную
                                   ориентацию (у MPU6050 её нет вовсе:
                                   кватернион нулевой, ковариация -1).
"""
import math

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    """Интегратор угловой скорости плюс, если есть, абсолютный курс."""

    def __init__(self, name):
        self.name = name
        self.integrated = 0.0     # радианы, интеграл angular.z
        self.absolute = None      # развёрнутый курс из позы, если он есть
        self._first = None
        self._prev = None
        self.count = 0
        self.wmax = 0.0
        self._t = None

    def add(self, stamp, wz, yaw=None):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._t is not None:
            dt = t - self._t
            # Дыры в секунды бывают при потере связи, такие интервалы
            # интегрировать нельзя — получим ложный доворот.
            if 0.0 < dt < 0.5:
                self.integrated += wz * dt
        self._t = t
        self.count += 1
        self.wmax = max(self.wmax, abs(wz))

        if yaw is not None:
            if self._first is None:
                self._first = yaw
                self.absolute = 0.0
                self._prev = yaw
            else:
                # Разворачиваем скачок через +-pi, иначе поворот на 180
                # градусов зачтётся как ноль.
                d = yaw - self._prev
                while d > math.pi:
                    d -= 2 * math.pi
                while d < -math.pi:
                    d += 2 * math.pi
                self.absolute += d
                self._prev = yaw


class YawCheck(Node):
    def __init__(self):
        super().__init__('yaw_check')
        self.ekf = Track('EKF      /odometry/filtered')
        self.gyro = Track('гироскоп /imu/mpu6050')
        self.whl = Track('колёса   /odom')

        self.create_subscription(Odometry, '/odometry/filtered',
                                 lambda m: self.ekf.add(m.header.stamp,
                                                        m.twist.twist.angular.z,
                                                        yaw_of(m.pose.pose.orientation)), 20)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self.whl.add(m.header.stamp,
                                                        m.twist.twist.angular.z), 20)
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Imu, '/imu/mpu6050',
                                 lambda m: self.gyro.add(m.header.stamp,
                                                         m.angular_velocity.z), qos)
        self.create_timer(1.0, self.report)
        self.get_logger().info('поворачивай робота, по окончании Ctrl+C')

    def report(self):
        d = math.degrees
        self.get_logger().info(
            f'EKF {d(self.ekf.absolute or 0):7.1f}  '
            f'гиро {d(self.gyro.integrated):7.1f}  '
            f'колёса {d(self.whl.integrated):7.1f}  (градусов)')

    def summary(self):
        d = math.degrees
        print('\n' + '=' * 64)
        print(f'{"источник":<28}{"по скорости":>13}{"по позе":>11}{"сообщ":>7}')
        print('-' * 64)
        for t in (self.ekf, self.gyro, self.whl):
            pose = f'{d(t.absolute):11.1f}' if t.absolute is not None else f'{"—":>11}'
            print(f'{t.name:<28}{d(t.integrated):13.1f}{pose}{t.count:7d}')
        print('-' * 64)
        print('Сравни с тем, на сколько повернул робота на самом деле.')
        print('Столбец «по позе» у EKF — это то, во что верит Nav2.')
        print('=' * 64)


def main():
    rclpy.init()
    node = YawCheck()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.summary()
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
