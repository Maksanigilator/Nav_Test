#!/usr/bin/env python3
"""Две траектории одной и той же точки робота — для сравнения глазами.

/track/odom    — где робот был по версии своей одометрии (RTAB-Map);
/track/ext_cam — где он был по версии внешней камеры.

Обе публикуются в кадре map и описывают ОДНУ точку — начало camera_link.
Поэтому позу доски, которую отдаёт камера, приходится пересчитывать: из
кадра пола в map (преобразование даёт ext_cam_anchor) и затем снять
смещение доски относительно корпуса. Без этого линии разошлись бы на
постоянную величину — на то, что доска стоит не в центре, — и это не
имело бы отношения к качеству одометрии.

В начале прогона линии совпадают по построению: именно по стартовой позе
их и связали. Дальше они расходятся, и расхождение — это и есть
накопленная ошибка одометрии, ради которой всё затевалось.

Точки прореживаются по пройденному расстоянию и повороту: RViz на
программном рендеринге не любит длинные ломаные, а для глаза сто точек
на метр ничем не лучше десяти.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca*b[0] - sa*b[1],
            a[1] + sa*b[0] + ca*b[1],
            a[2] + b[2])


def invert(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-ca*a[0] - sa*a[1], sa*a[0] - ca*a[1], -a[2])


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y),
                      1 - 2*(q.y*q.y + q.z*q.z))


class Track:
    """Одна ломаная: накопление с прореживанием и публикация."""

    def __init__(self, node, topic, frame, step, turn, limit):
        self.node = node
        self.frame = frame
        self.step = step
        self.turn = math.radians(turn)
        self.limit = limit
        self.path = Path()
        self.path.header.frame_id = frame
        self.last = None
        self.pub = node.create_publisher(Path, topic, 10)

    def add(self, pose, stamp):
        if self.last is not None:
            moved = math.hypot(pose[0]-self.last[0], pose[1]-self.last[1])
            turned = abs(math.atan2(math.sin(pose[2]-self.last[2]),
                                    math.cos(pose[2]-self.last[2])))
            if moved < self.step and turned < self.turn:
                return
        self.last = pose

        item = PoseStamped()
        item.header.frame_id = self.frame
        item.header.stamp = stamp
        item.pose.position.x = pose[0]
        item.pose.position.y = pose[1]
        item.pose.orientation.z = math.sin(pose[2]/2)
        item.pose.orientation.w = math.cos(pose[2]/2)
        self.path.poses.append(item)
        if len(self.path.poses) > self.limit:
            # Выкидываем каждую вторую, а не начало: форма всей траектории
            # важнее, чем её подробность. Прогон при этом не обрывается.
            self.path.poses = self.path.poses[::2]
        self.path.header.stamp = stamp
        self.pub.publish(self.path)


class Tracks(Node):
    def __init__(self):
        super().__init__('ext_cam_tracks')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('floor_frame', 'ext_cam_floor')
        # Смещение задаётся относительно ТОГО, НА ЧЁМ доска закреплена
        # (по умолчанию корпус камеры), а не относительно точки отсчёта
        # робота: мерить рулеткой от камеры проще, чем от середины шасси.
        # Связь между ними берётся из описания робота через TF.
        self.declare_parameter('board_frame', 'camera_link')
        self.declare_parameter('board_x', 0.0)
        self.declare_parameter('board_y', 0.0)
        self.declare_parameter('board_yaw', 0.0)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('min_step_m', 0.02)
        self.declare_parameter('min_turn_deg', 2.0)
        self.declare_parameter('max_points', 4000)
        self.declare_parameter('report_period', 5.0)

        step = float(self.get_parameter('min_step_m').value)
        turn = float(self.get_parameter('min_turn_deg').value)
        limit = int(self.get_parameter('max_points').value)
        frame = self.get_parameter('map_frame').value
        self.odom = Track(self, '/track/odom', frame, step, turn, limit)
        self.camera = Track(self, '/track/ext_cam', frame, step, turn, limit)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.anchor = None
        self.last_odom = None
        self.last_camera = None
        self.anchor_logged = False

        self.board = None
        self.get_logger().info(
            f"Доска относительно {self.get_parameter('board_frame').value}: "
            f'вперёд {self.get_parameter("board_x").value:+.3f} м, '
            f'влево {self.get_parameter("board_y").value:+.3f} м, разворот '
            f'{self.get_parameter("board_yaw").value:+.1f}°')

        self.create_subscription(PoseWithCovarianceStamped,
                                 '/ext_cam/pose', self.on_camera, 10)
        self.create_timer(1.0/float(self.get_parameter('rate').value),
                          self.on_tick)
        self.create_timer(float(self.get_parameter('report_period').value),
                          self.report)

    def board_in_base(self):
        # Связь жёсткая, из описания робота: ищем один раз и запоминаем.
        if self.board is not None:
            return self.board
        offset = (self.get_parameter('board_x').value,
                  self.get_parameter('board_y').value,
                  math.radians(self.get_parameter('board_yaw').value))
        frame = self.get_parameter('board_frame').value
        base = self.get_parameter('base_frame').value
        if frame == base:
            self.board = offset
        else:
            mount = self.lookup(base, frame)
            if mount is not None:
                self.board = compose(mount, offset)
        return self.board

    def lookup(self, parent, child):
        try:
            tf = self.buffer.lookup_transform(parent, child,
                                              rclpy.time.Time())
        except Exception:                                   # noqa: BLE001
            return None
        t = tf.transform.translation
        return (t.x, t.y, yaw_of(tf.transform.rotation))

    def on_tick(self):
        pose = self.lookup(self.get_parameter('map_frame').value,
                           self.get_parameter('base_frame').value)
        if pose is None:
            return
        self.last_odom = pose
        self.odom.add(pose, self.get_clock().now().to_msg())

    def on_camera(self, msg):
        # Привязка статическая, но появляется не сразу — ext_cam_anchor
        # ждёт, пока робота увидят и пока разойдётся одометрия.
        if self.anchor is None:
            self.anchor = self.lookup(
                self.get_parameter('map_frame').value,
                self.get_parameter('floor_frame').value)
            if self.anchor is None:
                return
            if not self.anchor_logged:
                self.anchor_logged = True
                self.get_logger().info('Привязка к полу получена, веду обе линии.')

        board_in_base = self.board_in_base()
        if board_in_base is None:
            return

        pose = msg.pose.pose
        board_in_floor = (pose.position.x, pose.position.y,
                          yaw_of(pose.orientation))
        board_in_map = compose(self.anchor, board_in_floor)
        base_in_map = compose(board_in_map, invert(board_in_base))
        self.last_camera = base_in_map
        self.camera.add(base_in_map, msg.header.stamp)

    def report(self):
        if self.last_odom is None or self.last_camera is None:
            return
        dx = self.last_camera[0] - self.last_odom[0]
        dy = self.last_camera[1] - self.last_odom[1]
        dyaw = math.atan2(math.sin(self.last_camera[2]-self.last_odom[2]),
                          math.cos(self.last_camera[2]-self.last_odom[2]))
        self.get_logger().info(
            f'Расхождение камера−одометрия: {math.hypot(dx, dy)*100:6.1f} см '
            f'(dx {dx*100:+.1f}, dy {dy*100:+.1f}), '
            f'курс {math.degrees(dyaw):+.1f}°')


def main():
    rclpy.init()
    node = Tracks()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
