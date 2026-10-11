#!/usr/bin/env python3
"""Проба DDS между хостом и контейнером — БЕЗ запуска Isaac Sim.

Зачем отдельно от симулятора. Isaac поднимается минутами (первый раз — ещё и
компиляция шейдеров), и если данные не пойдут, непонятно, что именно сломалось:
сцена, ros2_bridge или транспорт. Транспорт отделяется и проверяется за секунды,
потому что стек, которым пользуется bridge, — это обычные библиотеки Jazzy
внутри пакета isaacsim, и они грузятся в голый python. См. isaac/env.sh:
там ровно то окружение, которое получит bridge.

Проверяются три вещи, и ломаются они по-разному:

1. **discovery** — стороны видят друг друга. Проверяется `ros2 node list`
   в контейнере: там должна появиться нода /isaac_probe;
2. **поток хост -> контейнер БОЛЬШИМИ сообщениями.** Это направление камеры,
   и оно главное. Мелкий std_msgs/String проходит там, где кадр уже не проходит,
   поэтому проба публикует картинку настоящего рабочего размера:
   848x480 rgb8 = 1.22 МБ на сообщение, 30 Гц — как в модели камеры этапа 2;
3. **поток контейнер -> хост мелкими сообщениями.** Это направление /cmd_vel.

Главный подозреваемый при отказе — разделяемая память. Контейнер работает под
root, Isaac на хосте — под обычным пользователем, а Fast DDS по умолчанию возит
данные через /dev/shm. Характерный симптом ровно такой: **ноды видны, топиков
нет** — тот же, что описан в Dockerfile про Cyclone. Обход: перевести ОБЕ
стороны на UDP, `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.

Если после перехода на UDP картинка пойдёт рвано или не пойдёт вовсе, следующий
подозреваемый — размер приёмного буфера ядра: 1.22 МБ режется на IP-фрагменты,
и дефолтных net.core.rmem_max не хватает. Лечится на ХОСТЕ (контейнер с
--network host делит сетевой стек, отдельно внутри править не надо):

    sudo sysctl -w net.core.rmem_max=2147483647

Запуск. На хосте:

    source isaac/env.sh
    "$ISAAC_PYTHON" isaac/tests/dds_probe.py

В контейнере (второй терминал):

    ./run.sh ros2 node list                  # ожидается /isaac_probe
    ./run.sh ros2 topic hz /probe/image      # ожидается ~30 Гц, ровно
    ./run.sh ros2 topic pub -r 5 /probe/cmd geometry_msgs/msg/Twist "{}"
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

# Размер кадра из модели камеры этапа 2 (models/turtlebot3_waffle_d435/model.sdf).
# Проба должна нагружать транспорт так же, как это сделает реальная камера,
# иначе тест проходит, а сцена потом не едет.
WIDTH, HEIGHT = 848, 480
CHANNELS = 3  # rgb8


class Probe(Node):
    def __init__(self, rate_hz, reliable):
        super().__init__('isaac_probe')

        # QoS сознательно два разных. Sensor data в ROS обычно best_effort:
        # так публикуют камеры, и так же будет публиковать ros2_bridge Isaac.
        # Но при отладке транспорта best_effort ВРЁТ: потерянные сообщения
        # просто исчезают, и «тихо теряем половину» выглядит как «работает».
        # Поэтому по умолчанию reliable — чтобы проблема проявилась, а не
        # замаскировалась. Ключ --best-effort переключает на боевой режим.
        qos = QoSProfile(
            depth=1,
            reliability=(QoSReliabilityPolicy.RELIABLE if reliable
                         else QoSReliabilityPolicy.BEST_EFFORT))

        self.pub = self.create_publisher(Image, '/probe/image', qos)
        self.sub = self.create_subscription(Twist, '/probe/cmd', self.on_cmd, 10)

        # Буфер готовится один раз: пересоздавать массив на 1.2 МБ тридцать раз
        # в секунду — это мерить скорость питона, а не скорость DDS.
        self.msg = Image()
        self.msg.height = HEIGHT
        self.msg.width = WIDTH
        self.msg.encoding = 'rgb8'
        self.msg.is_bigendian = 0
        self.msg.step = WIDTH * CHANNELS
        self.msg.data = bytes(WIDTH * HEIGHT * CHANNELS)
        self.msg.header.frame_id = 'probe_optical_frame'

        self.sent = 0
        self.got_cmd = 0
        self.started = time.monotonic()

        self.create_timer(1.0 / rate_hz, self.tick)
        self.create_timer(2.0, self.report)

        mib = len(self.msg.data) / 1024 / 1024
        self.get_logger().info(
            f'публикую /probe/image {WIDTH}x{HEIGHT} rgb8 = {mib:.2f} МиБ '
            f'на сообщение, {rate_hz:g} Гц '
            f'({"reliable" if reliable else "best_effort"})')
        self.get_logger().info('слушаю /probe/cmd (обратное направление)')

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.sent += 1

    def on_cmd(self, _msg):
        self.got_cmd += 1

    def report(self):
        dt = time.monotonic() - self.started
        back = (f'{self.got_cmd} сообщений' if self.got_cmd
                else 'НИЧЕГО — обратное направление не работает')
        self.get_logger().info(
            f'отправлено {self.sent} кадров за {dt:.0f} с '
            f'({self.sent / dt:.1f} Гц) | из контейнера получено: {back}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--rate', type=float, default=30.0,
                    help='частота публикации, Гц (по умолчанию 30 — как камера)')
    ap.add_argument('--best-effort', action='store_true',
                    help='боевой QoS сенсора. По умолчанию reliable, чтобы '
                         'потери проявлялись, а не прятались')
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.rate, reliable=not args.best_effort)
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
