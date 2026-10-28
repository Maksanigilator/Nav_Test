#!/usr/bin/env python3
"""Привязка карты RTAB-Map к полу внешней камеры — один раз, на старте.

Робот и камера меряют одно движение, но ведут отсчёт от разного: RTAB-Map
от места, где включили камеру (кадр map), внешняя камера — от угла
напечатанной на полу сетки (кадр ext_cam_floor). Пока эти два кадра не
связаны, траектории сравнивать не с чем.

Связь берётся из СТАРТОВОЙ ПОЗЫ. Робот стоит; мы знаем, где он в map (из
дерева TF) и где он в ext_cam_floor (от камеры) — из этой пары и выходит
постоянное преобразование map → ext_cam_floor.

Почему постоянное, а не пересчитываемое на ходу. Пересчёт на каждом кадре
сводил бы расхождение к нулю по построению, и мерить стало бы нечего.
Замороженное преобразование — это точка отсчёта: всё, что после старта
разъехалось, и есть ошибка одометрии. Ровно её мы и хотим увидеть.

Доска ArUco стоит на роботе НЕ в центре, поэтому её смещение относительно
camera_link задаётся параметрами board_x / board_y / board_yaw. Если их не
задать, в расхождение траекторий войдёт ещё и это смещение, постоянное и
никак не связанное с качеством одометрии.

Счёт ведётся плоско, в трёх числах (x, y, курс). Высота и наклоны в
сравнение не входят: камера меряет позу доски на плоскости пола и крена с
тангажом не даёт вовсе.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def compose(a, b):
    """Поза b, заданная в кадре a, выраженная в родителе кадра a."""
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca*b[0] - sa*b[1],
            a[1] + sa*b[0] + ca*b[1],
            a[2] + b[2])


def invert(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-ca*a[0] - sa*a[1], sa*a[0] - ca*a[1], -a[2])


def yaw_of(q):
    # Полная формула, а не 2*atan2(z, w): камера на роботе наклонена, и у
    # преобразования map → camera_link крен с тангажом не нулевые.
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1 - 2*(q.y*q.y + q.z*q.z))


class Anchor(Node):
    def __init__(self):
        super().__init__('ext_cam_anchor')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('floor_frame', 'ext_cam_floor')
        self.declare_parameter('topic', '/ext_cam/pose')
        # Смещение задаётся относительно ТОГО, НА ЧЁМ доска закреплена
        # (по умолчанию корпус камеры), а не относительно точки отсчёта
        # робота: мерить рулеткой от камеры проще, чем от середины шасси.
        # Связь между ними берётся из описания робота через TF.
        self.declare_parameter('board_frame', 'camera_link')
        # Поза доски в кадре camera_link: вперёд, влево, поворот в градусах.
        self.declare_parameter('board_x', 0.0)
        self.declare_parameter('board_y', 0.0)
        self.declare_parameter('board_yaw', 0.0)
        # Робот на старте стоит, поэтому позу камеры можно усреднить —
        # шум у неё доли миллиметра, но несколько кадров стоят дёшево.
        self.declare_parameter('samples', 10)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.static = StaticTransformBroadcaster(self)
        self.seen = []
        self.waiting_logged = False
        self.done = False

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('topic').value, self.on_pose, 10)
        self.get_logger().info(
            f"Доска относительно {self.get_parameter('board_frame').value}: "
            f'вперёд {self.get_parameter("board_x").value:+.3f} м, '
            f'влево {self.get_parameter("board_y").value:+.3f} м, разворот '
            f'{self.get_parameter("board_yaw").value:+.1f}°')
        self.get_logger().info(
            'Жду стартовую позу: робот должен стоять и быть виден камере.')

    def on_pose(self, msg):
        if self.done:
            return
        pose = msg.pose.pose
        self.seen.append((pose.position.x, pose.position.y,
                          pose.position.z, yaw_of(pose.orientation)))
        if len(self.seen) < self.get_parameter('samples').value:
            return
        self.anchor()

    def lookup(self, parent, child):
        try:
            tf = self.buffer.lookup_transform(parent, child,
                                              rclpy.time.Time())
        except Exception:                                   # noqa: BLE001
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_of(tf.transform.rotation))

    def board_in_base(self):
        offset = (self.get_parameter('board_x').value,
                  self.get_parameter('board_y').value,
                  math.radians(self.get_parameter('board_yaw').value))
        frame = self.get_parameter('board_frame').value
        base = self.get_parameter('base_frame').value
        if frame == base:
            return offset
        mount = self.lookup(base, frame)
        return None if mount is None else compose(mount, offset)

    def anchor(self):
        map_frame = self.get_parameter('map_frame').value
        base_frame = self.get_parameter('base_frame').value
        board_in_base = self.board_in_base()
        if board_in_base is None:
            # Описание робота ещё не разошлось — связи base → камера нет.
            self.seen = self.seen[-1:]
            return
        try:
            tf = self.buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time())
        except Exception as error:                          # noqa: BLE001
            # Одометрия ещё не разошлась — это нормально первые секунды.
            # Копить кадры дальше незачем: нужен самый свежий, под тот
            # момент, когда преобразование наконец появится.
            if not self.waiting_logged:
                self.waiting_logged = True
                self.get_logger().info(
                    f'Камера робота видит, жду одометрию ({error}).')
            self.seen = self.seen[-1:]
            return

        # Усреднение: курс по-круговому, иначе переход через ±180° даст
        # бессмысленную середину.
        n = len(self.seen)
        fx = sum(s[0] for s in self.seen)/n
        fy = sum(s[1] for s in self.seen)/n
        fz = sum(s[2] for s in self.seen)/n
        fyaw = math.atan2(sum(math.sin(s[3]) for s in self.seen)/n,
                          sum(math.cos(s[3]) for s in self.seen)/n)

        t = tf.transform.translation
        base_in_map = (t.x, t.y, yaw_of(tf.transform.rotation))
        board_in_map = compose(base_in_map, board_in_base)
        board_in_floor = (fx, fy, fyaw)

        # Искомое: такое map → floor, что поза доски, пересчитанная из
        # пола в map, совпадает с тем, где доска по версии одометрии.
        anchor = compose(board_in_map, invert(board_in_floor))

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_frame
        msg.child_frame_id = self.get_parameter('floor_frame').value
        msg.transform.translation.x = anchor[0]
        msg.transform.translation.y = anchor[1]
        # ВЫСОТА БЕРЁТСЯ ОТ ТОЧКИ ОТСЧЁТА РОБОТА, а не от доски. Пол
        # внешней камеры — это та самая плоскость, по которой робот
        # ездит, то есть плоскость base_footprint. Раньше сюда входила
        # высота доски над полом, и карта границы уезжала на эти 0.7 м
        # вниз: под роботом висела плоскость, к которой он не относится.
        msg.transform.translation.z = t.z
        msg.transform.rotation.z = math.sin(anchor[2]/2)
        msg.transform.rotation.w = math.cos(anchor[2]/2)
        self.static.sendTransform(msg)

        self.done = True
        self.destroy_subscription(self.sub)
        self.get_logger().info(
            f'Привязка готова: map → {msg.child_frame_id} = '
            f'x {anchor[0]:+.3f} м, y {anchor[1]:+.3f} м, '
            f'курс {math.degrees(anchor[2]):+.2f}°')
        self.get_logger().info(
            f'  старт по камере:    x {fx:+.3f}  y {fy:+.3f}  '
            f'курс {math.degrees(fyaw):+.2f}°  (кадров усреднено: {n})')
        self.get_logger().info(
            f'  старт по одометрии: x {base_in_map[0]:+.3f}  '
            f'y {base_in_map[1]:+.3f}  '
            f'курс {math.degrees(base_in_map[2]):+.2f}°')


def main():
    rclpy.init()
    node = Anchor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
