#!/usr/bin/env python3
"""Автомат заезда робота на рельсы.

Последовательность: ОСМОТР -> ПОИСК -> ПОДХОД -> ВЫВЕРКА -> ЗАЕЗД ->
КОНТРОЛЬ, и ОТКАТ с повтором, если заезд не удался.

Разделение труда между Nav2 и собственным управлением намеренное.

Nav2 ведёт только ПОДХОД — проезд по настилу к точке старта заезда. Там он
на своём месте: есть карта, есть препятствия, есть куда объезжать, и робот
меканумный, так что на тесной площадке он доезжает боком.

Сам ЗАЕЗД ведётся напрямую. Причин две. Первая: при переходе на рельсы
радиус качения падает со 101.4 до 50 мм, робот идёт примерно вдвое
медленнее скомандованного, и любой регулятор, считающий это отставанием,
начнёт разгоняться. Вторая: на рельсах свободы манёвра нет вовсе, доворот
допустим в пределах ±10°, а всё сверх того — это промах, который надо
признать и повторить, а не выправлять на ходу.

Успех заезда определяется по одному признаку: продвигается робот вперёд
или стоит. Знать при этом правильную скорость не нужно и вредно — робот
при заезде подкренивается и может застрять в этом положении, и тогда важно
только то, что он перестал двигаться.
"""
import math
import time

import numpy as np
import rclpy
import rclpy.signals
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Point, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RailEntry(Node):
    def __init__(self):
        super().__init__('rail_entry')
        p = self.declare_parameter
        p('scan_rate', 0.30)            # рад/с на осмотре
        p('approach_back', 0.90)        # где встать до начала рельсов, м
        p('align_lat_tol', 0.02)        # допуск по поперечному смещению, м
        p('align_yaw_tol', 0.02)        # допуск по курсу на выверке, рад
        p('align_gain', 0.8)
        p('align_timeout', 25.0)
        p('drive_speed', 0.12)          # м/с на заезде
        # Сколько проехать ПОСЛЕ точки старта. Отсчёт идёт от неё, а не от
        # кромки, поэтому сюда входит и подход к кромке (0.9 м), и сам
        # проезд по рельсам. 3.5 м означает около 2.6 м по трубам.
        p('drive_distance', 3.50)
        # Ниже этой доли от скомандованного пути считаем, что стоим.
        # На рельсах робот идёт 0.34-0.43 от команды — замерено, — а затык
        # даёт ноль, так что порог стоит между ними с запасом.
        p('progress_frac', 0.15)
        p('window_s', 1.0)
        p('yaw_abort_deg', 10.0)        # больше — промах, повтор попытки
        p('tilt_abort_deg', 12.0)       # кренится сильнее — стоп немедленно
        p('retries', 3)
        # Запас к пути отката сверх пройденного вперёд. Нужен, потому что
        # вперёд и назад робот идёт по слегка разным дугам.
        p('retreat_margin', 0.15)

        # ПО УМОЛЧАНИЮ ПУБЛИКУЕМ В cmd_nav, а не в cmd_vel.
        #
        # На настоящем роботе /cmd_vel — это ВЫХОД мультиплексора, который
        # идёт прямо на моторы. Писать туда значит обойти и мультиплексор,
        # и кнопку X на геймпаде, и вдобавок драться с ним: он публикует
        # туда 20 раз в секунду, и команды перемешались бы.
        #
        # /cmd_nav же он слушает и пропускает только в режиме auto, то
        # есть после осознанного нажатия кнопки. Это и есть защита.
        self.cmd = self.create_publisher(Twist, 'cmd_nav', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(Imu, 'imu', self.on_imu, 20)
        self.create_subscription(PointCloud2, 'obstacles', self.on_cloud, 5)
        # Маркеры для RViz: ось рельсов, их начало и поза старта заезда.
        # Долговечность transient_local — чтобы RViz, открытый позже,
        # всё равно получил последнюю картинку, а не ждал нового поиска.
        self.mk = self.create_publisher(
            MarkerArray, 'rail_entry/markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Переиздаём раз в секунду: маркеры живут между фазами, а показать
        # их надо и тому, кто подключился в середине заезда.
        self.create_timer(1.0, self.publish_markers)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

        self.odom = None
        self.hist = []                  # (t, x, y, yaw) для оценки движения
        self.roll = self.pitch = 0.0
        self.rails = None               # (точка на оси, направление, начало)
        self.gone = 0.0                 # сколько проехали в текущей попытке
        self.cloud = None

    # ── данные ────────────────────────────────────────────────────────
    def on_odom(self, m):
        t = self.now()
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        self.odom = (x, y, yaw_of(m.pose.pose.orientation))
        self.hist.append((t, x, y))
        self.hist = [h for h in self.hist if t - h[0] < 5.0]

    def on_imu(self, m):
        q = m.orientation
        self.roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                               1 - 2 * (q.x ** 2 + q.y ** 2))
        self.pitch = math.asin(max(-1.0, min(1.0,
                                             2 * (q.w * q.y - q.z * q.x))))

    def on_cloud(self, m):
        """Из карты препятствий берём ТОЛЬКО зелёные точки — рельсы.

        Цвет проставил детектор по маске сегментации, поэтому здесь не надо
        ни искать трубы заново, ни знать, чем они отличаются от шума: всё,
        что зелёное, — рельс, и это единственное место, где автомат зависит
        от распознавания. Когда оракул сменится на модель CV, менять тут
        нечего.
        """
        pts = []
        for x, y, z, rgb in pc2.read_points(m, ('x', 'y', 'z', 'rgb'),
                                            skip_nans=True):
            c = np.float32(rgb).view(np.uint32)
            if ((c >> 8) & 255) > 200 and ((c >> 16) & 255) < 100:
                pts.append((x, y))
        self.cloud = np.array(pts) if pts else None
        # Кадр берём ИЗ ЗАГОЛОВКА облака, а не предполагаем. Слой рельсов
        # живёт в odom, а не в map — почему, подробно написано у параметра
        # rail_map_frame в детекторе. Поза робота для сравнения обязана
        # браться В ТОМ ЖЕ кадре, иначе сравнивались бы координаты из
        # разных систем, и ошибка выглядела бы как смещение рельсов.
        self.cloud_frame = m.header.frame_id or 'map'

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, sec):
        t0 = self.now()
        while self.now() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def go(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self.cmd.publish(t)

    def stop(self):
        for _ in range(3):
            self.go()
            self.spin(0.05)

    def say(self, state, msg):
        self.get_logger().info(f'[{state}] {msg}')

    # ── ОСМОТР ────────────────────────────────────────────────────────
    def scan(self):
        w = float(self.get_parameter('scan_rate').value)
        self.say('ОСМОТР', 'полный оборот на месте, набираем карту')
        t0 = self.now()
        while self.now() - t0 < 2 * math.pi / w and rclpy.ok():
            self.go(wz=w)
            self.spin(0.05)
        self.stop()
        self.spin(2.0)
        return True

    # ── ПОИСК ─────────────────────────────────────────────────────────
    def find_rails(self):
        """Две нитки по зелёным точкам: направление, ось, начало рельсов."""
        self.spin(2.0)
        P = self.cloud
        if P is None or len(P) < 20:
            self.say('ПОИСК', f'зелёных точек мало ({0 if P is None else len(P)})')
            return False
        # НАПРАВЛЕНИЕ СЧИТАЕМ ПО ДВУМ НИТКАМ ОТДЕЛЬНО.
        #
        # Главная ось PCA по всем точкам сразу даёт смещённую оценку:
        # рельсы видны частично, и если одну нитку видно на большем
        # протяжении, чем другую, ось заваливается в сторону разноса ниток.
        # Замерено: ошибка вышла 15.6°, робот вставал на ось, которой нет,
        # въезжал в трубы под углом и трижды подряд получал «увело на 10°».
        #
        # Поэтому каждую нитку центрируем на её собственной середине и
        # только потом ищем общее направление. Тогда разнос ниток в оценку
        # не входит вовсе, а вклад даёт только их вытянутость. Разбиение
        # уточняем в несколько проходов: первое деление идёт по грубой оси.
        X = P - P.mean(0)
        _, _, vt = np.linalg.svd(X, full_matrices=False)
        u = vt[0]
        A = B = None
        for _ in range(4):
            n = np.array([-u[1], u[0]])
            q = (P - P.mean(0)) @ n
            # Делим по медиане, а не по нулю: нитку, которую видно хуже,
            # среднее сдвигает, и по нулю обе могли уехать в одну группу.
            thr = np.median(q)
            A, B = P[q > thr], P[q <= thr]
            if len(A) < 5 or len(B) < 5:
                break
            Z = np.vstack([A - A.mean(0), B - B.mean(0)])
            _, _, vt2 = np.linalg.svd(Z, full_matrices=False)
            un = vt2[0]
            u = un if un @ u >= 0 else -un
        if A is None or len(A) < 5 or len(B) < 5:
            self.say('ПОИСК', f'вижу одну нитку '
                              f'({0 if A is None else len(A)}/'
                              f'{0 if B is None else len(B)})')
            return False
        n = np.array([-u[1], u[0]])
        axis = (A.mean(0) + B.mean(0)) / 2
        gauge = abs((A.mean(0) - B.mean(0)) @ n)
        # Направление — «от робота вдоль рельсов вдаль».
        #
        # Раньше знак выбирался по тому, куда уходит бОльшая часть точек.
        # Признак бессмысленный: ось считается как середина двух ниток,
        # поэтому среднее проекций равно нулю ПО ПОСТРОЕНИЮ, и знак решал
        # шум. Дважды повезло, на третий раз направление вышло развёрнутым
        # на 180°, точка подхода уехала за спину и Nav2 до неё не доехал.
        #
        # Правильный признак опирается на робота: рельсы уходят от него
        # прочь, значит дальний конец дальше от робота, чем ближний.
        t = (P - axis) @ u
        rp = self.pose_xy()
        if rp is None:
            self.say('ПОИСК', 'нет позы робота')
            return False
        far, near = P[np.argmax(t)], P[np.argmin(t)]
        if np.linalg.norm(far - rp) < np.linalg.norm(near - rp):
            u, t = -u, -t
        start = axis + u * t.min()      # где рельсы начинаются
        self.rails = (axis, u, start)
        self.gauge = gauge
        self.publish_markers()
        self.say('ПОИСК', f'нашёл рельсы: колея {gauge * 1000:.0f} мм, '
                          f'направление {math.degrees(math.atan2(u[1], u[0])):+.1f}°, '
                          f'начало x={start[0]:+.2f} y={start[1]:+.2f}, '
                          f'точек {len(P)}')
        return True

    # ── ПОДХОД ────────────────────────────────────────────────────────
    def approach(self):
        axis, u, start = self.rails
        back = float(self.get_parameter('approach_back').value)
        goal_xy = start - u * back
        yaw = math.atan2(u[1], u[0])
        self.say('ПОДХОД', f'цель x={goal_xy[0]:+.2f} y={goal_xy[1]:+.2f} '
                           f'курс {math.degrees(yaw):+.1f}°')
        if not self.nav.wait_for_server(timeout_sec=15.0):
            self.say('ПОДХОД', 'Nav2 не отвечает')
            return False
        g = NavigateToPose.Goal()
        # Цель отдаём В ТОМ ЖЕ кадре, в котором пришло облако рельсов.
        # Раньше здесь стояло 'map' жёстко, и это было верно, пока слой
        # жил в карте. После перевода слоя в odom координаты цели стали
        # одометрическими, а подпись осталась карточной — Nav2 увёз бы
        # робота на величину расхождения map и odom. Он умеет принимать
        # цель в любом кадре, который может преобразовать, так что просто
        # называем кадр честно.
        g.pose.header.frame_id = self.frame()
        g.pose.pose.position.x = float(goal_xy[0])
        g.pose.pose.position.y = float(goal_xy[1])
        g.pose.pose.orientation.z = math.sin(yaw / 2)
        g.pose.pose.orientation.w = math.cos(yaw / 2)
        fut = self.nav.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.say('ПОДХОД', 'цель не принята')
            return False
        res = gh.get_result_async()
        t0 = self.now()
        while not res.done() and self.now() - t0 < 120 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
        ok = res.done() and res.result().status == 4
        self.say('ПОДХОД', 'доехали' if ok else 'не доехал')
        return ok

    # ── ВЫВЕРКА ───────────────────────────────────────────────────────
    def pose_xy(self):
        try:
            tr = self.buf.lookup_transform(self.frame(), 'base_footprint',
                                           rclpy.time.Time())
        except Exception:
            return None
        return np.array([tr.transform.translation.x,
                         tr.transform.translation.y])

    def arrow(self, mid, p0, p1, rgb, z=0.10, width=0.04):
        """Стрелка от p0 к p1. Задаём двумя точками, а не позой с кватернионом:
        направление здесь и есть смысл маркера, и так его не переврать."""
        m = Marker()
        m.header.frame_id = self.frame()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'rail_entry', mid
        m.type, m.action = Marker.ARROW, Marker.ADD
        m.pose.orientation.w = 1.0
        for p, pt in ((p0, Point()), (p1, Point())):
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), z
            m.points.append(pt)
        # x — толщина древка, y — диаметр наконечника, z — его длина.
        m.scale.x, m.scale.y, m.scale.z = width, width * 2.2, width * 3.0
        m.color.r, m.color.g, m.color.b, m.color.a = (*rgb, 0.9)
        return m

    def publish_markers(self):
        """Что именно автомат считает рельсами и куда собирается встать.

        Нужно не для красоты: колея и направление — это ОЦЕНКА по облаку,
        и она может сесть мимо. Глазами промах виден мгновенно, а по
        числам в логе — нет: «колея 261 мм» ни о чём не говорит, пока не
        увидишь, что линия легла поперёк труб.
        """
        if getattr(self, 'rails', None) is None:
            return
        axis, u, start = self.rails
        back = float(self.get_parameter('approach_back').value)
        length = float(self.get_parameter('drive_distance').value)
        goal = start - u * back

        a = MarkerArray()
        # Ось рельсов от их начала вперёд на всю длину заезда, зелёная.
        a.markers.append(self.arrow(0, start, start + u * length,
                                    (0.24, 0.88, 0.35)))
        # Подход: от точки старта к началу рельсов, голубая. Длина стрелки
        # и есть approach_back, так что видно, где робот встанет.
        a.markers.append(self.arrow(1, goal, start, (0.25, 0.75, 1.0),
                                    z=0.06, width=0.03))
        # Начало рельсов — жёлтый шар.
        m = Marker()
        m.header.frame_id = self.frame()
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'rail_entry', 2
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = float(start[0]), float(start[1])
        m.pose.position.z, m.pose.orientation.w = 0.10, 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.85, 0.1, 0.9
        a.markers.append(m)
        # Подпись с измеренной колеёй: промах вписывания виден по ней
        # сразу, если помнить, что настоящая колея 526 мм
        # (обмер: просвет 470 мм плюс труба 56 мм).
        t = Marker()
        t.header = m.header
        t.ns, t.id = 'rail_entry', 3
        t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position.x, t.pose.position.y = float(goal[0]), float(goal[1])
        t.pose.position.z, t.pose.orientation.w = 0.45, 1.0
        t.scale.z = 0.14
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = (f'колея {getattr(self, "gauge", 0.0) * 1000:.0f} мм, '
                  f'курс {math.degrees(math.atan2(u[1], u[0])):+.0f}°')
        a.markers.append(t)
        self.mk.publish(a)

    def frame(self):
        """Кадр, в котором пришло облако рельсов. Он же используется для
        позы робота и для маркеров — все три обязаны совпадать."""
        return getattr(self, 'cloud_frame', 'map')

    def rail_error(self):
        """Поперечное смещение и курсовая ошибка относительно оси рельсов.

        Считается в кадре карты и переводится в кадр робота: так ошибка не
        зависит от того, насколько робот уже повернулся.
        """
        try:
            tr = self.buf.lookup_transform(self.frame(), 'base_footprint',
                                           rclpy.time.Time())
        except Exception:
            return None
        axis, u, _ = self.rails
        n = np.array([-u[1], u[0]])
        p = np.array([tr.transform.translation.x, tr.transform.translation.y])
        lat = float((p - axis) @ n)
        dyaw = wrap(math.atan2(u[1], u[0]) - yaw_of(tr.transform.rotation))
        return lat, dyaw

    def align(self):
        lat_tol = float(self.get_parameter('align_lat_tol').value)
        yaw_tol = float(self.get_parameter('align_yaw_tol').value)
        k = float(self.get_parameter('align_gain').value)
        t0 = self.now()
        while self.now() - t0 < float(self.get_parameter('align_timeout').value):
            e = self.rail_error()
            if e is None:
                self.spin(0.1)
                continue
            lat, dyaw = e
            # Большой курс здесь НЕ повод сдаваться. Раньше выверка
            # прерывалась, если ошибка превышала те же ±10°, что и на
            # заезде, — и после первой неудачи автомат отказывался даже
            # пробовать: «курс разошёлся на -12.2° — это промах», три раза
            # подряд с одним и тем же числом. Но допуск ±10° относится к
            # ЗАЕЗДУ, где править уже негде. Выверка для того и нужна,
            # чтобы такую ошибку убрать, и её единственный предел — время.
            if abs(lat) < lat_tol and abs(dyaw) < yaw_tol:
                self.stop()
                self.say('ВЫВЕРКА', f'на оси: смещение {lat * 1000:+.0f} мм, '
                                    f'курс {math.degrees(dyaw):+.2f}°')
                return True
            # Смещаемся боком, а не разворотами: у меканума это прямее, и
            # курс при этом не портится.
            self.go(vy=max(-0.08, min(0.08, -k * lat)),
                    wz=max(-0.25, min(0.25, k * dyaw)))
            self.spin(0.05)
        self.stop()
        self.say('ВЫВЕРКА', 'не сошлось за отведённое время')
        return False

    # ── ЗАЕЗД ─────────────────────────────────────────────────────────
    def moved(self, span):
        """Сколько робот прошёл за последние span секунд."""
        t = self.now()
        h = [x for x in self.hist if t - x[0] <= span]
        if len(h) < 2:
            return 0.0
        return math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2])

    def drive(self):
        v = float(self.get_parameter('drive_speed').value)
        need = float(self.get_parameter('drive_distance').value)
        span = float(self.get_parameter('window_s').value)
        frac = float(self.get_parameter('progress_frac').value)
        lim = math.radians(float(self.get_parameter('yaw_abort_deg').value))
        tilt = math.radians(float(self.get_parameter('tilt_abort_deg').value))
        start = np.array(self.odom[:2])
        self.say('ЗАЕЗД', f'идём вперёд {need:.2f} м на {v:.2f} м/с')
        t0 = self.now()
        last = t0
        while rclpy.ok():
            self.go(vx=v)
            self.spin(0.05)
            gone = float(np.linalg.norm(np.array(self.odom[:2]) - start))
            self.gone = gone
            if gone >= need:
                self.stop()
                self.say('ЗАЕЗД', f'прошли {gone:.2f} м')
                return True
            # Крен и тангаж: робот при заезде подкренивается, и это норма,
            # но если он валится — останавливаемся сразу, не дожидаясь
            # оценки продвижения.
            if abs(self.roll) > tilt or abs(self.pitch) > tilt:
                self.stop()
                self.say('ЗАЕЗД', f'кренится: крен '
                                  f'{math.degrees(self.roll):+.1f}°, тангаж '
                                  f'{math.degrees(self.pitch):+.1f}°')
                return False
            e = self.rail_error()
            if e and abs(e[1]) > lim:
                self.stop()
                self.say('ЗАЕЗД', f'увело на {math.degrees(e[1]):+.1f}° — '
                                  f'больше допуска, это промах')
                return False
            if self.now() - t0 > span + 0.5 and self.now() - last > 0.5:
                last = self.now()
                went = self.moved(span)
                want = v * span
                if went < want * frac:
                    self.stop()
                    self.say('ЗАЕЗД', f'упёрлись: за {span:.1f} с прошли '
                                      f'{went * 1000:.0f} мм при ожидаемых '
                                      f'{want * 1000:.0f}')
                    return False
                self.say('ЗАЕЗД', f'едем: {gone:.2f} из {need:.2f} м, '
                                  f'за окно {went * 1000:.0f} мм, '
                                  f'курс {math.degrees(e[1]) if e else 0:+.1f}°')
            if self.now() - t0 > 4 * need / v + 20:
                self.stop()
                self.say('ЗАЕЗД', 'слишком долго')
                return False

    # ── ОТКАТ ─────────────────────────────────────────────────────────
    def retreat(self, to, gone):
        """Назад по своему же следу, не дальше, чем проехали вперёд.

        Назад едем сами, а не через Nav2: за кромкой карта неизвестна, и
        планировщик туда путь не проложит — а робот при неудачном заезде
        стоит как раз там.

        Разобрано подробнее, потому что соблазн поручить откат Nav2
        возникает естественно. Типичная неудача выглядит так: передние
        колёса встали на рельсы, дальше не пошло. Поручить отсюда Nav2
        возврат в точку старта нельзя по трём причинам сразу.

        NavFn расчищает под роботом РОВНО ОДНУ клетку (clearRobotCell).
        Этого хватает, когда робот краем задел препятствие, и не хватает
        здесь: выезжать надо вдоль рельсов, а они непроезжие по всей
        длине, и потенциальное поле через летальные клетки не идёт.
        Контроллер MPPI вдобавок проверяет весь габарит 1.22x0.90, а он
        лежит на летальных клетках — все траектории отбрасываются.

        А главное, при неудаче планирования штатное дерево поведения
        запускает восстановления, и среди включённых у нас есть spin.
        Разворот на месте, когда робот стоит передними колёсами на
        рельсах, — это не бездействие, а вредное действие: так можно
        заклинить ходовую или свалить машину с настила.

        Отсюда же требование к ОТКАТУ: он обязан вывести робота ПОЛНОСТЬЮ
        за пределы рельсов, иначе следующий ПОДХОД, который уже идёт
        через Nav2, начнётся из положения в препятствии и сорвётся. За
        это отвечает retreat_margin. Если повторы начнут срываться сразу
        после неудачного заезда — смотреть сюда.

        Здесь было падение. Откат ехал прямо назад по текущему курсу и
        останавливался, только оказавшись в 10 см от точки старта попытки.
        Но курс к этому моменту уже был сбит на 10°, поэтому робот проходил
        мимо цели, условие не срабатывало ни разу, и он пятился все
        отведённые 60 секунд — 3.6 м вместо 1.5 — пока не съехал с настила
        и не упал на пол.
        
        Поэтому теперь два независимых ограничителя. Первый: пройденный
        назад путь не больше того, что проехали вперёд, плюс небольшой
        запас. Он работает, даже если до точки не доехали. Второй:
        смещение вбок гасится на ходу, чтобы робот возвращался по оси
        рельсов, а не по косой, — так он и до точки доезжает, и не
        подставляет борт под кромку.
        """
        v = float(self.get_parameter('drive_speed').value)
        k = float(self.get_parameter('align_gain').value)
        limit = gone + float(self.get_parameter('retreat_margin').value)
        start = np.array(self.odom[:2])
        self.say('ОТКАТ', f'отходим назад, не дальше {limit:.2f} м')
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < 90:
            p = np.array(self.odom[:2])
            back = float(np.linalg.norm(p - start))
            if back >= limit:
                self.say('ОТКАТ', f'выбран лимит пути назад ({back:.2f} м)')
                break
            if float(np.linalg.norm(p - to)) < 0.15:
                self.say('ОТКАТ', f'вернулись в точку старта, назад {back:.2f} м')
                break
            e = self.rail_error()
            vy = max(-0.06, min(0.06, -k * e[0])) if e else 0.0
            wz = max(-0.20, min(0.20, k * e[1])) if e else 0.0
            self.go(vx=-v, vy=vy, wz=wz)
            self.spin(0.05)
        else:
            self.say('ОТКАТ', 'вышло время')
        self.stop()
        e = self.rail_error()
        if e:
            self.say('ОТКАТ', f'смещение {e[0] * 1000:+.0f} мм, '
                              f'курс {math.degrees(e[1]):+.1f}°')

    # ── всё вместе ────────────────────────────────────────────────────
    def run(self):
        self.spin(3.0)
        if not self.scan():
            return False
        if not self.find_rails():
            return False
        if not self.approach():
            return False
        for attempt in range(1, int(self.get_parameter('retries').value) + 1):
            self.say('ПОПЫТКА', f'номер {attempt}')
            here = np.array(self.odom[:2])
            self.gone = 0.0
            if not self.align():
                self.retreat(here, self.gone)
                continue
            if self.drive():
                self.say('ИТОГ', f'ЗАЕХАЛИ с попытки {attempt}')
                return True
            self.retreat(here, self.gone)
        self.say('ИТОГ', 'не заехал за все попытки')
        return False


def main():
    # Сигналы перехватываем САМИ, не отдавая их rclpy.
    #
    # По умолчанию rclpy по Ctrl-C гасит контекст немедленно, и наш
    # finally с остановкой падает: «publisher's context is invalid».
    # Нулевая команда при этом не уходит, а мультиплексор на роботе
    # продолжает слать последнюю принятую скорость по 20 раз в секунду —
    # робот после Ctrl-C так и остаётся ехать. Поэтому контекст живёт до
    # тех пор, пока мы не отправим ноль своими руками.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    n = RailEntry()
    try:
        n.run()
    except KeyboardInterrupt:
        print()
    finally:
        try:
            n.stop()
        except Exception:
            pass
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
