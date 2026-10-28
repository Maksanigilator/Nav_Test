#!/usr/bin/env python3
"""Граница площадки от внешней камеры — как карта занятости для Nav2.

Раньше край платформы искали перепадом высоты по глубине. Здесь он берётся
готовым: камера смотрит на стенд сверху, её привязали к плоскости пола, и
граница известна в метрах — либо обведена руками на странице «Граница»,
либо, пока её не обвели, берётся поле зрения камеры на полу.

Многоугольник растрируется в nav_msgs/OccupancyGrid: внутри свободно (0),
снаружи занято (100). Для Nav2 это обычная статическая карта, и дальше с
ней работают штатно — добавляют препятствия, строят маршруты.

Карта публикуется в кадре пола (ext_cam_floor), а не в map. Связь между
ними даёт ext_cam_anchor, и благодаря этому карта не зависит от того, где
именно робот стоял, когда включили одометрию.

Отдаётся с долговечностью transient_local: подписчик, подключившийся
позже (RViz, Nav2), получит её сразу, а не будет ждать следующей посылки.
Посылка одна — многоугольник за прогон не меняется.
"""
import math
import sys
import time

import numpy as np
import rclpy
import requests
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from zeroconf import ServiceBrowser, Zeroconf
except ImportError:                                         # noqa: BLE001
    Zeroconf = None


def find_polygon(data):
    """Первый найденный список точек вида [[x, y], ...] в любом ответе.

    Схему страницы «Граница» на стороне камеры ещё могут поменять, а
    падать из-за переименованного поля не хочется: ищем по форме, а не по
    имени. Три точки — минимум, из которого получается область.
    """
    if isinstance(data, list):
        pairs = [p for p in data
                 if isinstance(p, (list, tuple)) and len(p) == 2
                 and all(isinstance(v, (int, float)) for v in p)]
        if len(pairs) >= 3 and len(pairs) == len(data):
            return [[float(p[0]), float(p[1])] for p in pairs]
        for item in data:
            found = find_polygon(item)
            if found:
                return found
    elif isinstance(data, dict):
        for value in data.values():
            found = find_polygon(value)
            if found:
                return found
    return None


class Grid(Node):
    def __init__(self):
        super().__init__('ext_cam_grid')
        self.declare_parameter('url', '')
        self.declare_parameter('service', '_extcam._tcp.local.')
        self.declare_parameter('discovery_timeout', 15.0)
        self.declare_parameter('frame_id', 'ext_cam_floor')
        self.declare_parameter('topic', '/ext_cam/grid')
        self.declare_parameter('resolution', 0.05)
        # Поля вокруг многоугольника: костмапа, упёртая в самый край
        # границы, не оставляет места даже под раздувание препятствий.
        self.declare_parameter('margin_m', 1.0)
        self.declare_parameter('retry_delay', 3.0)
        # Пока граница не обведена, опрашивать часто незачем.
        self.declare_parameter('boundary_poll', 10.0)

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('topic').value, qos)
        self.published = None
        self.waiting_logged = False
        self.timer = self.create_timer(
            float(self.get_parameter('retry_delay').value), self.attempt)
        self.attempt()

    # ── адрес камеры ──────────────────────────────────────────────────
    def base_url(self):
        explicit = self.get_parameter('url').value
        if explicit:
            return explicit.rstrip('/')
        if Zeroconf is None:
            self.get_logger().error('Нет zeroconf и не задан url.')
            return None
        found = []
        service = self.get_parameter('service').value

        class Listener:
            def add_service(self, zc, kind, name):
                info = zc.get_service_info(kind, name, timeout=3000)
                if info and info.parsed_addresses():
                    found.append(
                        f'http://{info.parsed_addresses()[0]}:{info.port}')

            def update_service(self, *_):
                pass

            def remove_service(self, *_):
                pass

        zc = Zeroconf()
        try:
            ServiceBrowser(zc, service, Listener())
            deadline = time.monotonic() + float(
                self.get_parameter('discovery_timeout').value)
            while not found and time.monotonic() < deadline:
                time.sleep(0.2)
        finally:
            zc.close()
        return found[0] if found else None

    # ── многоугольник ─────────────────────────────────────────────────
    def polygon(self, base):
        """Обведённая граница площадки — или ничего.

        Заглушки здесь намеренно НЕТ. Раньше, пока границу не обвели,
        вместо неё публиковалось поле зрения камеры на полу, и в RViz
        появлялась область, которой никто не задавал: разглядывая её,
        легко решить, что граница уже есть. Пусть лучше карты не будет
        вовсе, пока человек не нарисовал её сам.
        """
        answer = requests.get(f'{base}/api/boundary', timeout=5)
        if not answer.ok:
            return None
        return find_polygon(answer.json())

    def attempt(self):
        try:
            base = self.base_url()
            if not base:
                self.get_logger().warning('Камера в сети не найдена.')
                return
            points = self.polygon(base)
        except Exception as error:                          # noqa: BLE001
            self.get_logger().warning(f'Границу получить не вышло: {error}')
            return

        if points is None:
            if not self.waiting_logged:
                self.waiting_logged = True
                self.get_logger().info(
                    'Граница не обведена — карты не будет. Обведёте на '
                    'странице «Граница» — подхвачу без перезапуска.')
            self.timer.timer_period_ns = int(
                float(self.get_parameter('boundary_poll').value)*1e9)
            return
        if points == self.published:
            return

        self.published = points
        self.pub.publish(self.rasterize(points))
        self.timer.cancel()
        self.get_logger().info(
            f'Карта границы опубликована, {len(points)} вершин.')

    def rasterize(self, points):
        res = float(self.get_parameter('resolution').value)
        margin = float(self.get_parameter('margin_m').value)
        pts = np.asarray(points, float)
        low = pts.min(0) - margin
        high = pts.max(0) + margin
        width = int(math.ceil((high[0]-low[0])/res))
        height = int(math.ceil((high[1]-low[1])/res))

        # Центры ячеек, а не углы: точка проверяется на попадание внутрь.
        gx, gy = np.meshgrid(low[0] + (np.arange(width)+0.5)*res,
                             low[1] + (np.arange(height)+0.5)*res)
        inside = np.zeros(gx.shape, bool)
        x1, y1 = pts[:, 0], pts[:, 1]
        x2, y2 = np.roll(x1, -1), np.roll(y1, -1)
        for i in range(len(pts)):
            # Чётность пересечений луча, идущего влево из каждой ячейки.
            crosses = (y1[i] > gy) != (y2[i] > gy)
            cut = x1[i] + (x2[i]-x1[i])*(gy-y1[i])/(y2[i]-y1[i]+1e-12)
            inside ^= crosses & (gx < cut)

        grid = OccupancyGrid()
        grid.header.frame_id = self.get_parameter('frame_id').value
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = res
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(low[0])
        grid.info.origin.position.y = float(low[1])
        grid.info.origin.orientation.w = 1.0
        grid.data = np.where(inside, 0, 100).astype(np.int8).ravel().tolist()
        return grid


def main():
    rclpy.init()
    node = Grid()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
