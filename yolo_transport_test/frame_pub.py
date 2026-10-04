#!/usr/bin/env python3
"""Стенд: гоняем один кадр через контейнер с YOLO и меряем, чего это стоит.

ЗАЧЕМ. Между навигационным контейнером и контейнером с моделью лежит DDS,
и до сих пор по нему не прошло ни одного кадра. Этот стенд проверяет путь
целиком и раскладывает время на составляющие, не поднимая ни камеру, ни
навигацию — то есть ошибку, если она есть, видно в чистом виде.

ЧТО ИМЕННО ПРОВЕРЯЕТСЯ. Кадр берётся из датасета и приведён к 848x480 —
размеру настоящего кадра камеры. Это не косметика: сообщение весит
1 221 120 байт, а разделяемая память Fast DDS ломается именно на размере
сообщения. На картинке поменьше стенд прошёл бы там, где настоящий кадр
не проходит, и доверять ему было бы нельзя.

ОТКУДА БЕРУТСЯ ЧИСЛА. Часы у обоих контейнеров одни и те же — это один
хост, — поэтому метки времени из разных процессов сравнимы напрямую:

    отправка (здесь)                     header.stamp
      -> приём в узле маски              он сообщает задержку приёма
      -> работа модели                   он сообщает время инференса
      -> возврат маски сюда              круговая задержка меряется здесь

Разность круговой задержки и первых двух слагаемых и есть цена обратной
передачи. Без такого разложения непонятно, что именно тормозит.

ПОЧЕМУ СМОТРЕТЬ НАДО В RViz НА ОБА ТОПИКА. Панель «отправлено» подписана
на тот же топик, в который мы публикуем, а не рисует файл из памяти.
Локальный предпросмотр выглядел бы правильно даже при полностью
сломанной передаче — ни сериализация, ни транспорт, ни совместимость QoS
им не проверяются.

Запуск — см. RUN.md, раздел про стенд передачи.
"""
import collections
import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster

HERE = os.path.dirname(os.path.abspath(__file__))


class FramePub(Node):
    def __init__(self):
        super().__init__('frame_pub')
        p = self.declare_parameter
        p('image', os.path.join(HERE, 'rail_frame.png'))
        p('image_topic', '/camera/color/image_raw')
        p('mask_topic', '/rails/mask')
        p('stats_topic', '/rails/mask_stats')
        # Кадр камеры подписан именно этим кадром координат. Имя то же, что
        # у настоящей камеры, чтобы конфиг RViz годился и для боевого запуска.
        p('frame_id', 'camera_color_optical_frame')
        # Начинаем с одного герца: сначала надо убедиться, что кадр вообще
        # доходит, и только потом разгонять.
        p('rate', 1.0)
        p('report_every', 10.0)

        path = str(self.get_parameter('image').value)
        bgr = cv2.imread(path)
        if bgr is None:
            raise RuntimeError(f'не прочитать кадр {path}')
        # Публикуем rgb8 — ровно так отдаёт настоящий realsense2_camera.
        self.rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        self.h, self.w = self.rgb.shape[:2]
        self.frame_id = str(self.get_parameter('frame_id').value)

        # Издателя делаем BEST_EFFORT, как камеру: терять кадры при затыке
        # для неё нормально, и стенд должен вести себя так же. Заодно это
        # покажет потери, если транспорт не справляется.
        pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value), pub_qos)
        # Подписчик BEST_EFFORT совместим с издателем любого вида, поэтому
        # годится и для reliable-маски из узла сегментации.
        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image, str(self.get_parameter('mask_topic').value),
            self.on_mask, sub_qos)
        self.create_subscription(
            Float32MultiArray, str(self.get_parameter('stats_topic').value),
            self.on_stats, sub_qos)

        # Статическое преобразование: без него RViz ругается на отсутствие
        # кадра координат, и за красной строкой не видно сути.
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.frame_id
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)

        self.sent = {}                       # (sec, nsec) -> время отправки
        self.n_sent = self.n_back = self.n_empty = 0
        self.trip = collections.deque(maxlen=2000)
        self.infer = collections.deque(maxlen=2000)
        self.inbound = collections.deque(maxlen=2000)
        self.cpu = collections.deque(maxlen=2000)
        self.gpu = collections.deque(maxlen=2000)

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(float(self.get_parameter('report_every').value),
                          self.report)
        self.get_logger().info(
            f'кадр {self.w}x{self.h}, {self.h * self.w * 3} байт в сообщении, '
            f'{rate:g} Гц -> {self.get_parameter("image_topic").value}')

    def tick(self):
        now = self.get_clock().now()
        m = Image()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.frame_id
        m.height, m.width = self.h, self.w
        m.encoding = 'rgb8'
        m.is_bigendian = 0
        m.step = self.w * 3
        m.data = self.rgb.tobytes()
        self.pub.publish(m)
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        self.sent[key] = now.nanoseconds
        self.n_sent += 1
        # Ключи старых кадров не копим: ответ, не пришедший за сотню кадров,
        # уже не придёт, а словарь рос бы без конца.
        if len(self.sent) > 100:
            for k in sorted(self.sent)[:len(self.sent) - 100]:
                del self.sent[k]

    def on_mask(self, m):
        key = (m.header.stamp.sec, m.header.stamp.nanosec)
        t0 = self.sent.pop(key, None)
        if t0 is None:
            # Маска на кадр, которого мы не посылали, или уже забытый.
            return
        self.n_back += 1
        self.trip.append((self.get_clock().now().nanoseconds - t0) * 1e-6)
        # Пустая маска — модель ничего не нашла. Для стенда это важно
        # отдельно от потери: путь работает, а узнавания нет.
        if not np.any(np.frombuffer(m.data, np.uint8)):
            self.n_empty += 1

    def on_stats(self, m):
        if len(m.data) >= 4:
            self.infer.append(m.data[0])
            self.inbound.append(m.data[1])
            self.cpu.append(m.data[2])
            self.gpu.append(m.data[3])

    @staticmethod
    def q(d, p):
        return float(np.percentile(d, p)) if d else float('nan')

    def report(self):
        # Печатаем с flush: под ros2 launch вывод идёт в канал, питон
        # буферизует его блоками, и сводка не появлялась бы до самого
        # конца прогона — то есть ровно тогда, когда она уже не нужна.
        if not self.n_sent:
            return
        lost = self.n_sent - self.n_back
        print('\n' + '=' * 64, flush=True)
        print(f'отправлено {self.n_sent}, вернулось {self.n_back}, '
              f'потеряно {lost} ({100.0 * lost / self.n_sent:.0f}%)'
              + (f', пустых масок {self.n_empty}' if self.n_empty else ''), flush=True)
        if not self.trip:
            print('ответов нет — смотри RUN.md, раздел про стенд передачи', flush=True)
            print('=' * 64, flush=True)
            return

        tr_med = self.q(self.trip, 50)
        print(f'{"круговая задержка":28} {tr_med:7.1f} мс  '
              f'(мин {min(self.trip):.1f}, 95% {self.q(self.trip, 95):.1f}, '
              f'макс {max(self.trip):.1f})')
        if self.infer:
            in_med = self.q(self.inbound, 50)
            inf_med = self.q(self.infer, 50)
            print(f'{"  из них туда":28} {in_med:7.1f} мс  '
                  f'(95% {self.q(self.inbound, 95):.1f})')
            print(f'{"  из них модель":28} {inf_med:7.1f} мс  '
                  f'(95% {self.q(self.infer, 95):.1f})')
            print(f'{"  из них обратно":28} {tr_med - in_med - inf_med:7.1f} мс  '
                  f'(остаток)', flush=True)
            print(f'{"узел маски: процессор":28} {self.q(self.cpu, 50):7.1f} %   '
                  f'видеопамять {self.q(self.gpu, 50):.0f} МБ', flush=True)
        else:
            print('замеров от узла маски нет: он старой версии либо '
                  'не публикует /rails/mask_stats', flush=True)
        print('=' * 64, flush=True)


def main():
    rclpy.init()
    n = FramePub()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Сводку печатаем и при Ctrl-C, и при внешней остановке: без неё
        # прогон, прерванный руками, пропал бы впустую.
        n.report()
    finally:
        n.destroy_node()


if __name__ == '__main__':
    main()
