#!/usr/bin/env python3
"""Поза робота от внешней камеры — в ROS.

Внешняя неподвижная камера видит робота по доске маркеров и знает, где он
НА САМОМ ДЕЛЕ. Приложение камеры живёт отдельно (репозиторий aida-ext-camera),
на своей машине, и ROS не знает вовсе: оно отдаёт позу потоком HTTP. Этот узел
забирает поток и публикует позу как обычный топик.

Зачем так, а не DDS напрямую. Сторонний участник может публиковать в ROS-топик,
но для этого придётся вручную воспроизвести внутренние соглашения ROS: имя
топика на проводе (`rt/...`), имя типа (`geometry_msgs::msg::dds_::...`), хеш
типа и совместимые QoS. Плюс стык двух реализаций DDS. Выигрыш — один узел,
цена — поддержка этих соглашений вручную. Не стоит того.

    ros2 run maze_nav ext_cam_pose.py
    ros2 run maze_nav ext_cam_pose.py --ros-args -p url:=http://macbook.local:8080

Адрес по умолчанию не нужен: камера объявляет себя в сети как служба
_extcam._tcp, и узел находит её сам. Это тот же механизм, которым находятся
принтеры и Chromecast. Работает только в пределах одной подсети: мультикаст
через маршрутизаторы не ходит.
"""
import json
import math
import threading
import time

import rclpy
import requests
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:
    from zeroconf import ServiceBrowser, Zeroconf
except ImportError:                                     # пакета нет в образе
    Zeroconf = None


class ExtCamPose(Node):
    def __init__(self):
        super().__init__('ext_cam_pose')
        self.declare_parameter('url', '')
        self.declare_parameter('service', '_extcam._tcp.local.')
        # Кадр пола внешней камеры, а НЕ map: поза приходит в системе
        # отсчёта напечатанной на полу сетки. Связь с map даёт
        # ext_cam_anchor; назвать этот кадр map значило бы заявить,
        # что они совпадают, а они совпадают только после привязки.
        self.declare_parameter('frame_id', 'ext_cam_floor')
        self.declare_parameter('topic', 'ext_cam/pose')
        self.declare_parameter('discovery_timeout', 15.0)
        self.declare_parameter('reconnect_delay', 2.0)
        # Запасная неопределённость для приблизительной позы. Когда доску видно
        # не на заданной высоте, приложение отдаёт свободную 6-мерную подгонку,
        # а std_xy_m и std_yaw_deg выставляет В НОЛЬ — не потому, что поза точна,
        # а потому, что для неё эти величины не считаются. Ноль в ковариации
        # означал бы идеальное доверие, то есть ровно наоборот, поэтому такие
        # кадры получают заведомо широкую оценку.
        self.declare_parameter('free_pose_std_m', 0.10)
        self.declare_parameter('free_pose_std_deg', 10.0)

        self.frame = self.get_parameter('frame_id').value
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('topic').value, qos)

        self.stop = threading.Event()
        self.lost_since = None
        self.warned_about_stamp = False
        threading.Thread(target=self.run, daemon=True).start()

    # ── поиск камеры в сети ───────────────────────────────────────────────
    def base_url(self):
        explicit = self.get_parameter('url').value
        if explicit:
            return explicit.rstrip('/')
        if Zeroconf is None:
            self.get_logger().error(
                'Модуля zeroconf нет в образе, а параметр url не задан. Либо '
                'укажите адрес вручную (-p url:=http://имя.local:8080), либо '
                'добавьте zeroconf в docker/Dockerfile и пересоберите образ.')
            return None
        return self.discover()

    def discover(self):
        found = []
        service = self.get_parameter('service').value

        class Listener:
            def add_service(self, zc, kind, name):
                info = zc.get_service_info(kind, name, timeout=3000)
                if info and info.parsed_addresses():
                    found.append(f'http://{info.parsed_addresses()[0]}:{info.port}')

            def update_service(self, *_):
                pass

            def remove_service(self, *_):
                pass

        zc = Zeroconf()
        try:
            ServiceBrowser(zc, service, Listener())
            deadline = time.monotonic() + float(
                self.get_parameter('discovery_timeout').value)
            while not found and time.monotonic() < deadline and not self.stop.is_set():
                time.sleep(0.2)
        finally:
            zc.close()
        if not found:
            self.get_logger().warning(f'Служба {service} в сети не найдена.')
            return None
        self.get_logger().info(f'Камера найдена: {found[0]}')
        return found[0]

    # ── поток позы ────────────────────────────────────────────────────────
    def run(self):
        delay = float(self.get_parameter('reconnect_delay').value)
        while not self.stop.is_set():
            url = self.base_url()
            if url:
                try:
                    self.stream(url)
                except Exception as error:                  # noqa: BLE001
                    self.get_logger().warning(f'Поток оборвался: {error}')
            self.stop.wait(delay)

    def stream(self, base):
        # Поток Server-Sent Events: обычный GET, который не закрывается, а
        # сервер дописывает в него строки «data: {...}» по мере появления
        # кадров. Сообщения разделены пустой строкой.
        with requests.get(f'{base}/api/robot/events', stream=True, timeout=(5, 30)) as response:
            response.raise_for_status()
            self.get_logger().info(f'Подключился к {base}')
            for line in response.iter_lines(decode_unicode=True):
                if self.stop.is_set():
                    return
                if not line or not line.startswith('data:'):
                    continue
                try:
                    self.on_event(json.loads(line[5:].strip()))
                except (ValueError, KeyError) as error:
                    self.get_logger().warning(f'Непонятное сообщение: {error}')

    # Имя НЕ handle: у rclpy.Node есть свойство handle, и метод с тем же
    # именем его перекрывает — узел падает прямо в конструкторе на
    # «method object does not support the context manager protocol».
    def on_event(self, event):
        robot = event.get('robot')
        if robot is None:
            if self.lost_since is None:
                self.lost_since = time.monotonic()
                self.get_logger().info(
                    f"Робот не виден: {event.get('reason', 'причина не указана')}")
            return
        if self.lost_since is not None:
            self.get_logger().info(
                f'Робот снова виден через {time.monotonic()-self.lost_since:.1f} с')
            self.lost_since = None
        self.pub.publish(self.message(robot, event))

    def message(self, robot, event):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame
        stamp = self.capture_time(event)
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp-int(stamp))*1e9)

        msg.pose.pose.position.x = float(robot['x_m'])
        msg.pose.pose.position.y = float(robot['y_m'])
        msg.pose.pose.position.z = float(robot.get('z_m') or 0.0)
        yaw = float(robot['yaw_rad'])
        msg.pose.pose.orientation.z = math.sin(yaw/2)
        msg.pose.pose.orientation.w = math.cos(yaw/2)

        approximate = robot.get('source') == 'free_6dof'
        std_xy = robot.get('std_xy_m') or 0.0
        std_yaw = robot.get('std_yaw_deg') or 0.0
        if approximate or std_xy <= 0:
            std_xy = float(self.get_parameter('free_pose_std_m').value)
        if approximate or std_yaw <= 0:
            std_yaw = float(self.get_parameter('free_pose_std_deg').value)

        # Ковариация 6x6 по строкам: x, y, z, крен, тангаж, курс. Камера меряет
        # только три величины — x, y и курс; остальным ставим заведомо большую
        # дисперсию, чтобы потребитель на них не опирался.
        unknown = 1e3
        covariance = [0.0]*36
        covariance[0] = covariance[7] = std_xy**2
        covariance[14] = covariance[21] = covariance[28] = unknown
        covariance[35] = math.radians(std_yaw)**2
        msg.pose.covariance = covariance
        return msg

    def capture_time(self, event):
        # Момент СЪЁМКИ кадра, а не момент, когда досчиталась детекция. На
        # четырёх мегапикселях расчёт занимает десятки миллисекунд, и на ходу
        # это уже сантиметры. Приложение камеры отдаёт съёмку отдельным полем
        # captured_at; пока его нет, берём time — там момент расчёта, и ошибка
        # равна длительности детекции.
        if event.get('captured_at'):
            return float(event['captured_at'])
        if not self.warned_about_stamp:
            self.warned_about_stamp = True
            self.get_logger().warning(
                'В потоке нет поля captured_at — метка времени относится к '
                'концу расчёта, а не к съёмке кадра. Отставание равно времени '
                'детекции (см. analysis_ms в /api/state).')
        return float(event['time'])

    def destroy_node(self):
        self.stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = ExtCamPose()
    try:
        rclpy.spin(node)
    # ExternalShutdownException прилетает, когда узел снимают сигналом —
    # так его завершает ros2 launch. Без неё в журнале остаётся traceback
    # на каждой штатной остановке стека.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
