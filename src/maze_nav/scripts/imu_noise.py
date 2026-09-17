#!/usr/bin/env python3
"""Добавляет шум в идеальный поток IMU из Isaac Sim.

Зачем это нужно. У Gazebo шум инерциалки задан прямо в модели
(models/turtlebot3_waffle_d435/model.sdf, датчик tb3_imu): гироскоп 2e-4 рад/с,
акселерометр 1.7e-2 м/с^2 по каждой оси — уровень приличного MEMS класса
ICM-42688 или BMI088, как и записано в плане.

У Isaac Sim шума нет вообще: класс IMUSensor не имеет ни одного параметра
шума — только частота, положение и размер окна сглаживания. Получается
парадокс: переехав в более реалистичный симулятор, мы получили бы инерциалку
ЛУЧШЕ, чем была на этапе 2, и симулятор стал бы легче реальности — ровно
наоборот к тому, ради чего затевается этап 3.

Поэтому шум добавляем здесь, на стороне ROS, с теми же цифрами, что в модели
Gazebo. Тогда обе ветки дают на выходе сопоставимый сигнал, и сравнение ATE
между этапами 2 и 3 остаётся законным.

Место в цепочке:

    Gazebo:  gz tb3_imu (шум в SDF) ───────────────> /imu ──> imu_filter_madgwick
    Isaac:   ROS2PublishImu (идеально) ──> ЭТОТ УЗЕЛ ──> /imu ──> imu_filter_madgwick

То есть /imu — общий стык, и всё, что ниже по течению, одинаково для обоих
симуляторов. Гейзебовский launch этот узел не поднимает и знать о нём не знает.

    ros2 run maze_nav imu_noise.py --ros-args -p input_topic:=/imu/ideal

Ориентацию узел НЕ трогает и не добавляет: её вычисляет imu_filter_madgwick
из гироскопа и акселерометра, как и на реальном роботе. Isaac умеет отдавать
готовый кватернион, но это истинное значение из симулятора, которого на железе
взяться неоткуда, — брать его значило бы подыгрывать себе.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu

# Те же значения, что у датчика tb3_imu в модели Gazebo. Менять их имеет смысл
# только вместе с моделью — иначе этапы 2 и 3 перестанут быть сравнимыми.
GYRO_STDDEV = 2e-4      # рад/с
ACCEL_STDDEV = 1.7e-2   # м/с^2


class ImuNoise(Node):
    def __init__(self):
        super().__init__('imu_noise')

        self.declare_parameter('input_topic', '/imu/ideal')
        self.declare_parameter('output_topic', '/imu')
        self.declare_parameter('gyro_stddev', GYRO_STDDEV)
        self.declare_parameter('accel_stddev', ACCEL_STDDEV)
        # Смещение нуля: у реальной микросхемы оно есть и медленно плывёт,
        # но постоянная часть — главное, что видит фильтр. По умолчанию 0,
        # чтобы поведение совпадало с моделью Gazebo, где смещения тоже нет.
        self.declare_parameter('gyro_bias', 0.0)
        self.declare_parameter('seed', 0)

        p = self.get_parameter
        self.gyro_sd = p('gyro_stddev').value
        self.accel_sd = p('accel_stddev').value
        self.gyro_bias = p('gyro_bias').value

        seed = p('seed').value
        # Фиксированное зерно делает прогоны воспроизводимыми: при разборе
        # неудачного заезда важно уметь повторить ровно тот же шум.
        self.rng = np.random.default_rng(seed if seed else None)

        # Сенсорный QoS: best_effort, как и у любого потока с датчика.
        qos = QoSProfile(depth=20,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(Imu, p('output_topic').value, qos)
        self.sub = self.create_subscription(
            Imu, p('input_topic').value, self.on_imu, qos)

        self.count = 0
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"{p('input_topic').value} -> {p('output_topic').value}, "
            f'шум: гироскоп {self.gyro_sd} рад/с, '
            f'акселерометр {self.accel_sd} м/с^2')

    def on_imu(self, msg):
        out = Imu()
        # Штамп и фрейм копируем БЕЗ изменений. Переставлять время нельзя:
        # весь стек живёт на use_sim_time, и rgbd_odometry сопоставляет
        # инерциалку с кадрами именно по штампу.
        out.header = msg.header

        g = self.rng.normal(0.0, self.gyro_sd, 3) + self.gyro_bias
        out.angular_velocity.x = msg.angular_velocity.x + float(g[0])
        out.angular_velocity.y = msg.angular_velocity.y + float(g[1])
        out.angular_velocity.z = msg.angular_velocity.z + float(g[2])

        a = self.rng.normal(0.0, self.accel_sd, 3)
        out.linear_acceleration.x = msg.linear_acceleration.x + float(a[0])
        out.linear_acceleration.y = msg.linear_acceleration.y + float(a[1])
        out.linear_acceleration.z = msg.linear_acceleration.z + float(a[2])

        # Ковариации заполняем честно — фильтр их читает и взвешивает по ним.
        gv, av = self.gyro_sd ** 2, self.accel_sd ** 2
        out.angular_velocity_covariance = [gv, 0.0, 0.0,
                                           0.0, gv, 0.0,
                                           0.0, 0.0, gv]
        out.linear_acceleration_covariance = [av, 0.0, 0.0,
                                              0.0, av, 0.0,
                                              0.0, 0.0, av]
        # -1 в первом элементе — соглашение ROS: «ориентации в сообщении нет».
        # Без него потребитель решил бы, что кватернион (0,0,0,0) настоящий.
        out.orientation_covariance = [-1.0] + [0.0] * 8

        self.pub.publish(out)
        self.count += 1

    def report(self):
        if self.count == 0:
            self.get_logger().warn(
                'с входного топика не пришло ни одного сообщения — '
                'проверь, что сцена Isaac запущена и публикует IMU')
        else:
            self.get_logger().info(f'пропущено сообщений: {self.count}')


def main():
    rclpy.init()
    node = ImuNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
