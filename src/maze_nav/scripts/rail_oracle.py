#!/usr/bin/env python3
"""Идеальная имитация сегментации рельсов: маска в кадре камеры.

Настоящего распознавания рельсов пока нет. До этого «рельсами» считалось
всё, что поднялось выше порога над полом, — то есть трубы Ø51 мм попадали
в препятствия просто по высоте, а вместе с ними и остатки шума
дальномера. На карте это выглядело как зелёные точки в случайных местах.

Оракул берёт истинную геометрию рельсов из файла мира и рисует маску тех
пикселей, куда труба проецируется. Интерфейс намеренно такой же, какой
вернёт будущая модель CV: одноканальная маска размером с кадр глубины.
Замена оракула на модель — это правка одной подписки, а не переделка
потребителя.

Что оракул честно воспроизводит:

* перекрытие — пиксель попадает в маску только если измеренная глубина
  совпала с расчётной, поэтому труба за препятствием в маску не идёт;
* толщину в пикселях — она считается от дальности, как у настоящей трубы.

Чего он не воспроизводит и на чём поэтому нельзя мерить качество: ошибок
распознавания, пропусков на бликах и ложных срабатываний. Оракул нужен,
чтобы отладить логику заезда отдельно от качества детектора.
"""
import math
import re
import subprocess

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image

import cv2


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


def rpy_to_mat(r, p, y):
    """SDF задаёт поворот как R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rails_from_world(path, step=0.01):
    """Осевая линия рельсов из файла мира, точками через step метров.

    Геометрия берётся из самого мира, а не повторяется здесь по константам
    генератора: иначе два описания одной трубы неизбежно разъедутся после
    первой же правки колеи.

    Дуги угла П набраны короткими цилиндрами (тора в SDF нет), поэтому
    разбирать отдельно прямые и дуги не нужно — любой цилиндр даёт отрезок
    оси, а вместе они дают всю осевую линию.
    """
    s = open(path).read()
    m = re.search(r'<model name="rails">(.*?)</model>', s, re.S)
    if not m:
        raise RuntimeError(f'в {path} нет модели rails')
    pts, rad = [], None
    for c in re.finditer(
            r'<collision name="[^"]*">\s*<pose>([^<]+)</pose>\s*'
            r'<geometry><cylinder><radius>([\d.]+)</radius>'
            r'<length>([\d.]+)</length>', m.group(1)):
        x, y, z, rr, pp, yy = (float(v) for v in c.group(1).split())
        rad = float(c.group(2))
        ln = float(c.group(3))
        axis = rpy_to_mat(rr, pp, yy) @ np.array([0.0, 0.0, 1.0])
        n = max(2, int(ln / step) + 1)
        t = np.linspace(-ln / 2, ln / 2, n)
        pts.append(np.array([x, y, z]) + t[:, None] * axis)
    return np.vstack(pts), rad


class RailOracle(Node):
    def __init__(self):
        super().__init__('rail_oracle')
        self.declare_parameter('world_file', '')
        self.declare_parameter('model', 'agro_robot')
        self.declare_parameter('world_name', 'rails')
        # Допуск на совпадение измеренной глубины с расчётной. Радиус трубы
        # плюс шум дальномера, который растёт как квадрат дальности.
        self.declare_parameter('depth_tol', 0.03)
        self.declare_parameter('depth_tol_k', 0.02)
        self.declare_parameter('camera_frame', 'camera_depth_optical_frame')
        # Диск рисуется вокруг ОСЕВОЙ линии, а видимая часть трубы шире:
        # камера смотрит сверху под углом и видит верх и ближний бок, они
        # проецируются за пределы диска по осевому радиусу. Замерено: без
        # запаса маска накрывает 89.9% пикселей на высоте трубы, и
        # оставшийся силуэт уходит в «не рельс».
        self.declare_parameter('radius_scale', 1.8)

        self.rails, self.pipe_r = rails_from_world(
            self.get_parameter('world_file').value)
        self.get_logger().info(
            f'осевая линия рельсов: {len(self.rails)} точек, '
            f'труба Ø{self.pipe_r * 2000:.0f} мм')

        self.k = None
        self.T_wo = None          # world -> odom, ставится по первому кадру
        self.T_bc = None          # base -> камера, статика, берётся однажды
        # История поз одометрии. Смотреть позу через TF по метке кадра
        # нельзя: одометрия публикуется ПОСЛЕ обработки кадра и отстаёт от
        # него на полкадра, поэтому запрос на точную метку упирается в
        # экстраполяцию в будущее, а ждать внутри обработчика нельзя —
        # исполнитель однопоточный и сам себя заблокирует. В /odom же метка
        # равна метке кадра, который эту позу и породил, так что достаточно
        # хранить недавние позы и брать нужную по метке.
        self.odom = []
        self.bridge = CvBridge()
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

        sub = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        pub = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, 'camera_info', self.on_info, sub)
        self.create_subscription(Image, 'depth', self.on_depth, sub)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.mask = self.create_publisher(Image, 'rails/mask', pub)

    def on_info(self, m):
        self.k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_odom(self, m):
        q, t = m.pose.pose.orientation, m.pose.pose.position
        T = np.eye(4)
        T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
        T[:3, 3] = (t.x, t.y, t.z)
        self.odom.append((stamp_s(m.header.stamp), T, m.child_frame_id))
        del self.odom[:-40]

    def pose_at(self, stamp):
        """Поза одометрии на момент снимка, из истории."""
        if not self.odom:
            return None, None
        want = stamp_s(stamp)
        i = min(range(len(self.odom)), key=lambda j: abs(self.odom[j][0] - want))
        if abs(self.odom[i][0] - want) > 0.1:
            return None, None
        return self.odom[i][1], self.odom[i][2]

    def anchor(self):
        """Привязка кадра одометрии к миру, один раз за запуск.

        Одометрия стартует из единицы в позе спавна, поэтому world->odom
        равно истинной позе робота в момент первого кадра. Берём её разово
        из Gazebo: дальше кадр odom неподвижен относительно мира, а
        накопленный дрейф одометрии на маску не влияет, потому что маска
        строится по той же цепочке TF, по которой робот себя и видит.
        """
        w = self.get_parameter('world_name').value
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/world/{w}/pose/info', '-n', '1'],
            capture_output=True, text=True, timeout=15).stdout
        name = self.get_parameter('model').value
        blk = out.split(f'name: "{name}"')[1][:800]
        g = lambda pat, txt: float(re.search(pat, txt).group(1))
        px = g(r'x: (-?[\d.e+-]+)', blk)
        py = g(r'y: (-?[\d.e+-]+)', blk)
        pz = g(r'z: (-?[\d.e+-]+)', blk)
        q = blk.split('orientation')[1]
        R = quat_to_mat(g(r'x: (-?[\d.e+-]+)', q), g(r'y: (-?[\d.e+-]+)', q),
                        g(r'z: (-?[\d.e+-]+)', q), g(r'w: (-?[\d.e+-]+)', q))
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, (px, py, pz)
        self.get_logger().info(
            f'кадр одометрии привязан к миру: {px:+.3f} {py:+.3f} {pz:+.3f}')
        return T

    def on_depth(self, m):
        if self.k is None:
            return
        cam = self.get_parameter('camera_frame').value
        T_ob, base = self.pose_at(m.header.stamp)
        if T_ob is None:
            self.get_logger().warn('нет позы одометрии на момент снимка',
                                   throttle_duration_sec=5.0)
            return
        if self.T_bc is None:
            # Связка корпус->камера неподвижна, поэтому берётся один раз и
            # по последней метке: экстраполировать статику не нужно.
            try:
                tr = self.buf.lookup_transform(base, cam, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f'нет TF {base} <- {cam}: {e}',
                                       throttle_duration_sec=5.0)
                return
            t, q = tr.transform.translation, tr.transform.rotation
            self.T_bc = np.eye(4)
            self.T_bc[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
            self.T_bc[:3, 3] = (t.x, t.y, t.z)
        if self.T_wo is None:
            try:
                self.T_wo = self.anchor()
            except Exception as e:
                self.get_logger().warn(f'нет позы из Gazebo: {e}',
                                       throttle_duration_sec=5.0)
                return

        T_wc = self.T_wo @ T_ob @ self.T_bc
        R, o = T_wc[:3, :3], T_wc[:3, 3]
        P = (self.rails - o) @ R          # world -> camera optical

        fx, fy, cx, cy = self.k
        z = P[:, 2]
        fwd = z > 0.15
        P, z = P[fwd], z[fwd]
        depth = self.bridge.imgmsg_to_cv2(m).astype(np.float32)
        if m.encoding == '16UC1':
            depth = depth / 1000.0
        h, w = depth.shape
        u = (fx * P[:, 0] / z + cx)
        v = (fy * P[:, 1] / z + cy)
        vis = (u > -20) & (u < w + 20) & (v > -20) & (v < h + 20)
        u, v, z = u[vis], v[vis], z[vis]

        # Z-БУФЕР ТРУБЫ вместо простого диска.
        #
        # Диск по осевому радиусу не накрывает силуэт: камера смотрит
        # сверху под углом и видит верх и ближний бок трубы, а они
        # проецируются за пределы диска — замерено, 10% пикселей трубы
        # оставались снаружи. Раздуть диск нельзя: он тут же перехлёстывает
        # на пол за кромкой настила, и точность падает до 71%.
        #
        # Поэтому рисуем диск с запасом, но в буфер ДАЛЬНОСТИ, а маской
        # берём только те пиксели, где измеренная глубина совпала с трубой.
        # Запас тогда добирает силуэт, а на далёкий пол не распространяется,
        # потому что там глубина совсем другая. Перекрытие получается тем же
        # условием: за препятствием глубина тоже не совпадёт.
        order = np.argsort(-z)      # дальние рисуем первыми, ближние поверх
        ui = np.clip(u[order].astype(np.int32), 0, w - 1)
        vi = np.clip(v[order].astype(np.int32), 0, h - 1)
        zs = z[order]
        scale = float(self.get_parameter('radius_scale').value)
        rad = np.maximum(1, (scale * fx * self.pipe_r / zs)).astype(np.int32)
        # Непокрытые пиксели держим заведомо недостижимой дальностью, а не
        # бесконечностью: с inf дальше пришлось бы считать inf * k.
        zbuf = np.full((h, w), -1e6, np.float32)
        for uu_, vv_, rr, zz in zip(ui, vi, rad, zs):
            cv2.circle(zbuf, (int(uu_), int(vv_)), int(rr), float(zz), -1)

        tol = (self.get_parameter('depth_tol').value
               + self.get_parameter('depth_tol_k').value * np.abs(zbuf))
        img = (np.isfinite(depth) & (depth > 0)
               & (np.abs(depth - zbuf) < tol + self.pipe_r)).astype(np.uint8) * 255

        out = self.bridge.cv2_to_imgmsg(img, encoding='mono8')
        out.header = m.header
        self.mask.publish(out)
        self.get_logger().info(
            f'маска рельсов: {int((img > 0).sum())} пикселей из {h * w}',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    n = RailOracle()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()


if __name__ == '__main__':
    main()
