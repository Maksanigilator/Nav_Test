#!/usr/bin/env python3
"""Меряет, насколько УГОЛ КАМЕРЫ расходится с тем, что записано в URDF.

ЗАЧЕМ ОТДЕЛЬНЫЙ ЗАМЕР. Детектор кромки подгоняет плоскость земли и
сообщает её наклон — но он считает, что робот стоит РОВНО, и потому
записывает в ошибку камеры всё подряд: и перекос крепления, и настоящий
уклон пола, и наклон самого робота. Замерено на стенде: детектор говорил
3.4 градуса, тогда как робот физически стоял под градусом, а ИНС была
привинчена с перекосом 1.65. Разделить эти слагаемые подгонка не может по
устройству: у неё один отсчёт, и тот от URDF.

КАК РАЗВОДИМ. Нужен независимый отсчёт вертикали, и он есть —
акселерометр. В покое он меряет направление «вверх», и это физика, а не
чья-то оценка. Тогда:

* плоскость земли по глубине даёт вертикаль ГЛАЗАМИ КАМЕРЫ;
* гравитация даёт вертикаль ПО ТЯЖЕСТИ;
* угол между ними — расхождение креплений камеры и ИНС, и оно НЕ зависит
  от того, ровно ли стоит робот: наклонится робот — повернутся оба
  вектора разом, а угол между ними останется.

Это и есть искомое. Отдельно печатается наклон каждого вектора от оси Z
кадра робота — их разность показывает, сколько приходится на настоящий
наклон, а сколько на крепления.

ЧТО ПОТОМ ДЕЛАТЬ С ЧИСЛОМ. Если камера расходится с тяжестью, правится
угол в URDF: генератор `gen_robot_urdf.py` принимает `--cam-pitch`.
Прибавить найденный тангаж к нынешним 45 градусам и перегенерировать.

ТРЕБОВАНИЯ К ЗАМЕРУ. Робот стоит неподвижно, перед камерой — ровный
участок пола или настила без предметов. Рельсы в поле зрения не мешают:
отбор по самому населённому уровню их отбрасывает.
"""
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

BASE = 'base_footprint'


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rp(v, name):
    """Крен и тангаж вектора относительно оси Z, в градусах."""
    u = v / np.linalg.norm(v)
    return (f'{name}: отклонение {math.degrees(math.acos(min(1.0, abs(u[2])))):5.2f}°  '
            f'крен {math.degrees(math.atan2(u[1], u[2])):+6.2f}°  '
            f'тангаж {math.degrees(math.atan2(-u[0], math.hypot(u[1], u[2]))):+6.2f}°')


class CamLevel(Node):
    def __init__(self):
        super().__init__('cam_level')
        p = self.declare_parameter
        p('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        p('info_topic', '/camera/color/camera_info')
        p('imu_topic', '/imu/data_raw')
        # Кадр, в котором ИНС отдаёт свои оси. Для внешней платы это
        # imu_link; для собственной ИНС камеры — её кадр, и тогда замер
        # становится независимым от того, что записано про камеру в URDF.
        p('imu_frame', 'imu_link')
        # Окно подгонки: только земля под роботом, подальше от кромки.
        p('fit_range', 1.1)
        p('seconds', 8.0)
        self.k = self.depth = None
        self.acc = []
        self.create_subscription(CameraInfo,
                                 str(self.get_parameter('info_topic').value),
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image,
                                 str(self.get_parameter('depth_topic').value),
                                 self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu,
                                 str(self.get_parameter('imu_topic').value),
                                 self.on_imu, qos_profile_sensor_data)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

    def on_info(self, m):
        self.k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_depth(self, m):
        self.depth = m

    def on_imu(self, m):
        a = m.linear_acceleration
        self.acc.append((a.x, a.y, a.z))

    def tf_mat(self, child):
        t = self.buf.lookup_transform(BASE, child, rclpy.time.Time())
        q = t.transform.rotation
        return quat_to_mat(q.x, q.y, q.z, q.w), np.array(
            [t.transform.translation.x, t.transform.translation.y,
             t.transform.translation.z])

    def ground_normal(self):
        """Нормаль плоскости земли в кадре робота, по данным камеры."""
        m, step = self.depth, 2
        if m.encoding == '16UC1':
            d = (np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
                 [::step, ::step].astype(np.float32) * 0.001)
        else:
            d = np.frombuffer(m.data, np.float32).reshape(
                m.height, m.width)[::step, ::step]
        fx, fy, cx, cy = self.k
        uu, vv = np.meshgrid((np.arange(0, m.width, step) - cx) / fx,
                             (np.arange(0, m.height, step) - cy) / fy)
        R, O = self.tf_mat(m.header.frame_id)
        P = (np.stack([uu, vv, np.ones_like(uu)], -1) * d[..., None]) @ R.T + O
        ok = np.isfinite(d) & (d > 0.1) & (d < 3.0)
        r = np.hypot(P[..., 0], P[..., 1])
        sel = ok & (r < float(self.get_parameter('fit_range').value)) & \
            (np.abs(P[..., 2]) < 0.25)
        if sel.sum() < 500:
            raise RuntimeError(f'земли в окне мало ({int(sel.sum())} точек)')
        G = P[sel]
        # Затравка по самому населённому уровню и несимметричная полоса —
        # земля это НИЖНЯЯ поверхность, всё приподнятое (рельсы, предметы)
        # в оценку попадать не должно.
        lo, hi = float(G[:, 2].min()), float(G[:, 2].max())
        cnt, edges = np.histogram(G[:, 2], bins=max(2, int((hi - lo) / 0.02)),
                                  range=(lo, hi))
        z0 = 0.5 * (edges[int(np.argmax(cnt))] + edges[int(np.argmax(cnt)) + 1])
        G = G[(G[:, 2] > z0 - 0.03) & (G[:, 2] < z0 + 0.02)]
        A = np.c_[G[:, 0], G[:, 1], np.ones(len(G))]
        for _ in range(2):
            co = np.linalg.lstsq(A, G[:, 2], rcond=None)[0]
            keep = np.abs(G[:, 2] - A @ co) < 0.03
            if keep.sum() < 300:
                break
            G, A = G[keep], A[keep]
        n = np.array([-co[0], -co[1], 1.0])
        return n / np.linalg.norm(n), len(G)


def main():
    rclpy.init()
    n = CamLevel()
    secs = float(n.get_parameter('seconds').value)
    print(f'\nРобот стоит неподвижно, перед камерой ровный пол. Набираю {secs:.0f} с...')
    t0 = time.time()
    while time.time() - t0 < secs and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.depth is None or n.k is None:
        print('\nНет глубины или калибровки — запущен ли стек?')
        return 1
    if len(n.acc) < 20:
        print('\nНет данных ИНС — без неё сравнивать не с чем.')
        return 1

    try:
        n_cam, used = n.ground_normal()
        R_imu, _ = n.tf_mat(str(n.get_parameter('imu_frame').value))
        # Нынешний наклон камеры берём ИЗ TF, а не из памяти. Сначала здесь
        # стояло число 45 прямо в коде, и после первой же правки URDF
        # помощник стал советовать поправку к устаревшему значению: при
        # камере, уже выставленной на 50.1, он предлагал 44.9 вместо 50.0.
        R_cam, _ = n.tf_mat('camera_link')
        cur_pitch = math.degrees(math.atan2(-R_cam[2, 0],
                                            math.hypot(R_cam[2, 1], R_cam[2, 2])))
    except Exception as e:
        print(f'\nне вышло: {e}')
        return 1

    a = np.array(n.acc).mean(axis=0)
    g_base = R_imu @ a
    g_base /= np.linalg.norm(g_base)

    print(f'\nточек в подгонке земли: {used}, выборок ИНС: {len(n.acc)}')
    print('\n' + rp(n_cam, 'вертикаль по КАМЕРЕ '))
    print(rp(g_base, 'вертикаль по ТЯЖЕСТИ'))

    cosang = float(np.clip(n_cam @ g_base, -1.0, 1.0))
    diff = math.degrees(math.acos(cosang))
    # Разворачиваем расхождение в крен и тангаж: тангаж и есть поправка
    # к --cam-pitch, крен означал бы, что камера завалена набок.
    d_pitch = (math.degrees(math.atan2(-n_cam[0], math.hypot(n_cam[1], n_cam[2])))
               - math.degrees(math.atan2(-g_base[0], math.hypot(g_base[1], g_base[2]))))
    d_roll = (math.degrees(math.atan2(n_cam[1], n_cam[2]))
              - math.degrees(math.atan2(g_base[1], g_base[2])))

    print(f'\nРАСХОЖДЕНИЕ КАМЕРЫ С ТЯЖЕСТЬЮ: {diff:.2f}°')
    print(f'  по тангажу {d_pitch:+.2f}°   по крену {d_roll:+.2f}°')
    print('\nЭта величина НЕ зависит от того, ровно ли стоит робот:')
    print('наклонится робот — повернутся оба вектора разом.')

    if diff < 0.4:
        print('\nКамера и ИНС согласны. Угол в URDF править незачем,')
        print('а наблюдаемый наклон карты идёт от наклона самого робота')
        print('или от ухода одометрии.')
    else:
        print(f'\nУгол камеры в URDF расходится с действительностью.')
        print(f'Сейчас в URDF наклон {cur_pitch:.1f}°, перегенерировать с:')
        print(f'\n    --cam-pitch {cur_pitch + d_pitch:.1f}\n')
        if abs(d_roll) > 0.4 and abs(d_pitch) < 0.4:
            print(f'Внимание: расхождение почти целиком по КРЕНУ ({d_roll:+.2f}°),')
            print('а тангаж уже выставлен. Крен — это завал камеры набок, и')
            print('параметром --cam-pitch он не правится: нужен поворот по X')
            print('в стыке camera_joint либо подкладка под кронштейн.')
        print('Знак проверить обязательно: после перегенерации запустить')
        print('этот помощник снова, расхождение должно упасть, а не вырасти.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
