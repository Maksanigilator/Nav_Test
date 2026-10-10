#!/usr/bin/env python3
"""Сервер заезда на рельсы и съезда с них.

Узел ЖИВЁТ ПОСТОЯННО и ничего не делает, пока его не позовут. Поднимается
вместе со стеком и предоставляет два действия:

    /rail_entry   заезд с произвольной точки настила на рельсы
    /rail_exit    съезд с рельсов задним ходом

ПОЧЕМУ СЕРВЕР, А НЕ ОДНОКРАТНЫЙ СЦЕНАРИЙ. Раньше это был скрипт, который
отрабатывал один раз и выходил. Поднятый вместе с запуском, он успевал
стартовать РАНЬШЕ сегментации, не находил зелёных точек и молча завершался;
снаружи это выглядело как «заезд не едет», причём процесса в ps уже не было.
Сервер эту ловушку снимает конструктивно: пропустить ему нечего, он спит.

ПОЧЕМУ ДВА ДЕЙСТВИЯ, А НЕ ОДНО С ПОЛЕМ РЕЖИМА. У заезда и съезда разные
предусловия (на настиле / на рельсах) и разные признаки успеха. Поле режима
— зародыш мешанины: следом захочется enter_and_exit, потом ещё пара. Nav2
по той же причине держит Spin, BackUp и DriveOnHeading отдельно.

ПОЧЕМУ В ОДНОМ УЗЛЕ. Они делят состояние — позу рельсов, одометрию,
подписку на облако — и никогда не должны идти одновременно. В одном
процессе взаимное исключение это одна проверка, а не договорённость
между двумя узлами.

Последовательность заезда: ОСМОТР -> ПОИСК -> ПОДХОД -> ВЫВЕРКА -> ЗАЕЗД,
и ОТКАТ с повтором, если заезд не удался.

Разделение труда между Nav2 и собственным управлением намеренное.

Nav2 ведёт только ПОДХОД — проезд по настилу к точке старта заезда. Там он
на своём месте: есть карта, есть препятствия, есть куда объезжать, и робот
меканумный, так что на тесной площадке он доезжает боком.

Сам ЗАЕЗД ведётся напрямую. Причин две. Первая: при переходе на рельсы
радиус качения падает со 101.4 до 50 мм, робот идёт примерно вдвое
медленнее скомандованного, и любой регулятор, считающий это отставанием,
начнёт разгоняться. Вторая: на рельсах свободы манёвра нет вовсе, доворот
допустим в пределах +-10°, а всё сверх того — это промах, который надо
признать и повторить, а не выправлять на ходу.

Успех заезда определяется по одному признаку: продвигается робот вперёд
или стоит. Знать при этом правильную скорость не нужно и вредно — робот
при заезде подкренивается и может застрять в этом положении, и тогда важно
только то, что он перестал двигаться.

ПОТОКИ. Узел крутит MultiThreadedExecutor, а все обратные вызовы сидят в
одной реентерабельной группе. Это обязательно: тело действия выполняется
в потоке исполнителя и ДОЛГО блокируется, а подписки обязаны продолжать
обновляться — иначе одометрия замрёт ровно тогда, когда она нужнее всего.
По той же причине внутри фаз нельзя звать rclpy.spin_once: узел уже
крутится, и крутить его вторично нельзя.
"""
import math
import threading
import time

import numpy as np
import rclpy
import rclpy.signals
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from maze_nav_interfaces.action import RailEntry, RailExit
from rclpy.action import (ActionClient, ActionServer, CancelResponse,
                          GoalResponse)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class RailServer(Node):
    def __init__(self):
        super().__init__('rail_server')
        p = self.declare_parameter
        # Прокрута на месте ЗДЕСЬ НЕТ НАРОЧНО: он приходит полем scan в
        # цели действия, а не параметром узла. Узел живёт постоянно и
        # обслуживает разные вызовы — одному нужен осмотр на свежей карте,
        # другому нет, и общий параметр на оба не натянешь.
        p('scan_rate', 0.30)            # рад/с на осмотре
        # Колея. Своей подгонки здесь больше нет, фигуру вписывает
        # детектор, поэтому число берётся отсюда, а не из данных.
        p('gauge', 0.526)
        # Насколько старой может быть поза от детектора. Он публикует её
        # дважды в секунду; две секунды значат, что он жив и видит рельсы.
        p('pose_max_age', 2.0)
        # ОБНОВЛЕНИЕ ОСИ ПО ХОДУ ЗАЕЗДА.
        #
        # Раньше ось бралась снимком в ПОИСКЕ и не менялась до конца
        # прогона. Пока подгонка стоит на месте, это безразлично, а когда
        # она поплыла — заезд продолжал ехать по устаревшему снимку, и
        # расхождение было видно только глазом: нарисованная фигура жила
        # своей жизнью, ось заезда своей.
        #
        # Теперь ось подтягивается всё время, пока выполняется цель.
        p('rail_refresh', True)
        # Как часто подтягивать. Чаще незачем: детектор публикует позу
        # раз в rail_publish_period, то есть дважды в секунду.
        p('rail_refresh_period', 0.4)
        # СКАЧКИ НЕ ПРИНИМАЕМ. Обновление обязано быть уточнением, а не
        # подменой цели: если свежая поза разошлась с нынешней больше
        # этих порогов, верим старой и предупреждаем. Так сорвавшаяся
        # подгонка (10 октября она уехала целым блоком за площадку) не
        # уведёт робота за собой, а проявится записью в журнале.
        #
        # ПЯТЬ ГРАДУСОВ, А НЕ БОЛЬШЕ, и это связано с yaw_abort_deg (10).
        # Обновление сдвигает ОТСЧЁТ, от которого считается курсовая
        # ошибка, значит ошибка меняется ровно на величину сдвига. Будь
        # допуск обновления больше порога срыва, законное уточнение само
        # вызывало бы «увело на 10 градусов — это промах». Поэтому он
        # обязан оставаться заметно ниже.
        p('rail_refresh_max_yaw', 5.0)     # градусов
        p('rail_refresh_max_jump', 0.30)   # метров
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
        p('stall_windows', 3)           # столько плохих окон подряд — упор
        p('window_s', 1.0)
        p('yaw_abort_deg', 10.0)        # больше — промах, повтор попытки
        p('tilt_abort_deg', 12.0)       # кренится сильнее — стоп немедленно
        p('retries', 3)
        # Запас к пути отката сверх пройденного вперёд. Нужен, потому что
        # вперёд и назад робот идёт по слегка разным дугам.
        p('retreat_margin', 0.15)
        # Съезд: сколько отъехать назад, если в цели не задано.
        p('exit_distance', 1.50)

        # ПО УМОЛЧАНИЮ ПУБЛИКУЕМ В cmd_nav, а не в cmd_vel.
        #
        # На настоящем роботе /cmd_vel — это ВЫХОД мультиплексора, который
        # идёт прямо на моторы. Писать туда значит обойти и мультиплексор,
        # и кнопку X на геймпаде, и вдобавок драться с ним: он публикует
        # туда 20 раз в секунду, и команды перемешались бы.
        #
        # /cmd_nav же он слушает и пропускает только в режиме auto, то
        # есть после осознанного нажатия кнопки. Это и есть защита.
        # ПИШЕМ В СЫРОЙ ТОПИК, А НЕ ПРЯМО РОБОТУ. Между нами и приводом
        # стоит cmd_sign: он разворачивает знак поворота (привод робота
        # понимает его наоборот соглашению ROS) и гасит скорость, если
        # команды пропали.
        #
        # Имя по умолчанию выбрано так НАРОЧНО. Если cmd_sign забудут
        # поднять, робот не получит ничего и останется стоять. Публикуй
        # мы сразу в /cmd_nav — та же забывчивость обернулась бы
        # поворотом в противоположную сторону на полном ходу.
        # ОДНА РЕЕНТЕРАБЕЛЬНАЯ ГРУППА НА ВСЁ. Тело действия блокируется
        # надолго, и подписки обязаны обновляться параллельно ему: иначе
        # одометрия замрёт ровно на заезде, когда по ней считается
        # продвижение. С группой по умолчанию (взаимно исключающей) они
        # встали бы в очередь за телом действия и не выполнились бы ни разу.
        self.cbg = ReentrantCallbackGroup()
        self.cmd = self.create_publisher(Twist, 'cmd_nav_raw', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20,
                                 callback_group=self.cbg)
        self.create_subscription(Imu, 'imu', self.on_imu, 20,
                                 callback_group=self.cbg)
        self.create_subscription(PointCloud2, 'obstacles', self.on_cloud, 5,
                                 callback_group=self.cbg)
        self.create_subscription(PoseStamped, 'rails/pose', self.on_rail_pose,
                                 5, callback_group=self.cbg)
        # Маркеры для RViz: ось рельсов, их начало и поза старта заезда.
        # Долговечность transient_local — чтобы RViz, открытый позже,
        # всё равно получил последнюю картинку, а не ждал нового поиска.
        self.mk = self.create_publisher(
            MarkerArray, 'rail_entry/markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # Переиздаём раз в секунду: маркеры живут между фазами, а показать
        # их надо и тому, кто подключился в середине заезда.
        self.create_timer(1.0, self.publish_markers,
                          callback_group=self.cbg)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                callback_group=self.cbg)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

        self.odom = None
        self.hist = []                  # (t, x, y, yaw) для оценки движения
        self.roll = self.pitch = 0.0
        self.rails = None               # (точка на оси, направление, начало)
        self.rail_pose = None           # готовая поза от детектора
        self.gone = 0.0                 # сколько проехали в текущей попытке
        self.cloud = None
        self.attempt = 0
        self._refreshed = 0.0           # когда ось подтягивали в прошлый раз

        # ОДНА ЦЕЛЬ ЗА РАЗ. Отклоняем вторую, а не вытесняем первую:
        # вытеснение посреди заезда означало бы бросить робота на трубах
        # с недоговорённым состоянием.
        self._lock = threading.Lock()
        self._busy = False
        self._gh = None                 # текущая цель, для проверки отмены
        self._emit = None               # куда слать обратную связь
        self._nav_gh = None             # цель Nav2, чтобы гасить её при отмене

        self.entry_srv = ActionServer(
            self, RailEntry, 'rail_entry',
            execute_callback=self.execute_entry,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.cbg)
        self.exit_srv = ActionServer(
            self, RailExit, 'rail_exit',
            execute_callback=self.execute_exit,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.cbg)
        self.get_logger().info('сервер рельсов готов: /rail_entry, /rail_exit')

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

    def on_rail_pose(self, m):
        """Готовая поза рельсов от детектора: торец и курс вдоль труб.

        ПОЧЕМУ БЕРЁМ ЕЁ, А НЕ СЧИТАЕМ САМИ. Своя подгонка здесь делила
        точки пополам ПО МЕДИАНЕ поперёк. Загиб П пересекает колею на всю
        ширину и лежит на той же высоте, что трубы, поэтому при делении
        пополам его точки обязаны попасть в обе половины и тянут оба
        центра к середине. Замерено на стенде по 1546 клеткам: колея
        выходила 496 мм вместо 526, и туда же уезжали ось с направлением.

        Детектор вписывает ИЗВЕСТНУЮ фигуру перебором по позе с уточнением
        устойчивым МНК, и даёт 524 мм. Считать одно и то же двумя разными
        способами незачем: расходясь, они дали бы необъяснимое поведение.
        """
        q = m.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rail_pose = (
            np.array([m.pose.position.x, m.pose.position.y]),
            np.array([math.cos(yaw), math.sin(yaw)]),
            m.header.frame_id or 'odom',
            self.now())

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin(self, sec):
        """Пауза внутри фазы.

        Здесь НЕЛЬЗЯ звать rclpy.spin_once: узел уже крутит исполнитель в
        другом потоке, и крутить его вторично нельзя. Подписки обновляются
        сами, нам достаточно подождать.

        Заодно это единственная точка, где отмена прерывает любую фазу:
        все они ждут через spin, так что проверка здесь избавляет от
        необходимости рассыпать её по всем циклам.
        """
        t0 = self.now()
        while self.now() - t0 < sec and rclpy.ok() and not self.cancelled():
            time.sleep(0.02)
        self.refresh_rails()

    def refresh_rails(self):
        """Подтянуть ось рельсов к свежей позе детектора.

        Зовётся из spin(), через который ждут ВСЕ фазы, — поэтому
        обновление само собой идёт от начала цели и до её конца, и нигде
        больше его прописывать не надо.

        Скачок отвергается: обновление должно уточнять цель, а не
        подменять её. Это же и защита от сорвавшейся подгонки.
        """
        if self._gh is None or self.rails is None:
            return
        if not bool(self.get_parameter('rail_refresh').value):
            return
        now = self.now()
        if now - self._refreshed < \
                float(self.get_parameter('rail_refresh_period').value):
            return
        self._refreshed = now
        if self.rail_pose is None:
            return
        start, u, frame, got = self.rail_pose
        if now - got > float(self.get_parameter('pose_max_age').value):
            return
        _axis, u_old, start_old = self.rails
        dyaw = abs(wrap(math.atan2(u[1], u[0])
                        - math.atan2(u_old[1], u_old[0])))
        jump = float(np.linalg.norm(start - start_old))
        if dyaw > math.radians(
                float(self.get_parameter('rail_refresh_max_yaw').value)) \
                or jump > float(
                    self.get_parameter('rail_refresh_max_jump').value):
            self.say('ОСЬ', f'свежая поза отвергнута: курс разошёлся на '
                            f'{math.degrees(dyaw):+.1f}°, начало на '
                            f'{jump * 1000:.0f} мм — еду по прежней')
            return
        self.cloud_frame = frame
        self.rails = (start, u, start)
        self.publish_markers()

    def cancelled(self):
        gh = self._gh
        return gh is not None and gh.is_cancel_requested

    def go(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        self.cmd.publish(t)

    def stop(self):
        # Пауза здесь СВОЯ, не через spin: тот прерывается по отмене, а
        # остановка обязана отработать именно на отмене. Иначе нули не
        # уйдут, и робот доедет на последней принятой скорости.
        for _ in range(3):
            self.go()
            time.sleep(0.05)

    def say(self, state, msg):
        """В журнал и, если цель выполняется, в обратную связь действия.

        Одна точка на все фазы: что видно в логе, то же приходит
        вызывающему с --feedback. Расхождению между ними взяться неоткуда.
        """
        self.get_logger().info(f'[{state}] {msg}')
        emit = self._emit
        if emit is not None:
            try:
                emit(state, msg)
            except Exception:
                pass

    # ── ОСМОТР ────────────────────────────────────────────────────────
    def scan(self, enabled):
        if not enabled:
            self.say('ОСМОТР', 'пропущен: карта уже собрана')
            return True
        w = float(self.get_parameter('scan_rate').value)
        self.say('ОСМОТР', 'полный оборот на месте, набираем карту')
        t0 = self.now()
        while (self.now() - t0 < 2 * math.pi / w and rclpy.ok()
               and not self.cancelled()):
            self.go(wz=w)
            self.spin(0.05)
        self.stop()
        self.spin(2.0)
        return True

    # ── ПОИСК ─────────────────────────────────────────────────────────
    def find_rails(self):
        """Две нитки по зелёным точкам: направление, ось, начало рельсов."""
        self.spin(2.0)
        # Сперва спрашиваем детектор: у него вписана фигура с известными
        # размерами, и это лучше всего, что можно получить здесь.
        age_max = float(self.get_parameter('pose_max_age').value)
        if self.rail_pose is not None:
            start, u, frame, got = self.rail_pose
            age = self.now() - got
            if age <= age_max:
                self.cloud_frame = frame
                self.rails = (start, u, start)
                self.gauge = float(self.get_parameter('gauge').value)
                self.publish_markers()
                # Сверка направления: рельсы обязаны уходить ОТ робота.
                # Детектор ставит торец на загиб П, а заезжают именно
                # через него, так что обычно сходится. Если нет — молчать
                # нельзя, робот поедет не туда.
                rp = self.pose_xy()
                if rp is not None and (start - rp) @ u > 0:
                    self.say('ПОИСК', 'ВНИМАНИЕ: торец рельсов дальше от '
                                      'робота, чем их продолжение — загиб '
                                      'может быть не с той стороны')
                self.say('ПОИСК', f'беру позу от детектора: начало '
                                  f'x={start[0]:+.2f} y={start[1]:+.2f}, '
                                  f'направление '
                                  f'{math.degrees(math.atan2(u[1], u[0])):+.1f}°, '
                                  f'возраст {age:.1f} с')
                return True
            self.say('ПОИСК', f'поза от детектора устарела ({age:.1f} с), '
                              f'считаю сам')
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
        # ЖДЁМ БУДУЩЕЕ ОПРОСОМ, А НЕ spin_until_future_complete.
        # Та функция крутит узел сама, а его уже крутит исполнитель —
        # второй раз нельзя. Исполнитель и так обслуживает ответы Nav2 в
        # соседнем потоке, нам остаётся только подождать результата.
        fut = self.nav.send_goal_async(g)
        t0 = self.now()
        while not fut.done() and self.now() - t0 < 15 and rclpy.ok():
            time.sleep(0.05)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            self.say('ПОДХОД', 'цель не принята')
            return False
        # Держим цель Nav2: при отмене её надо погасить, иначе планировщик
        # продолжит везти робота после того, как мы сдались.
        self._nav_gh = gh
        res = gh.get_result_async()
        t0 = self.now()
        while not res.done() and self.now() - t0 < 120 and rclpy.ok():
            if self.cancelled():
                self.say('ПОДХОД', 'отмена — гашу цель Nav2')
                gh.cancel_goal_async()
                self._nav_gh = None
                return False
            time.sleep(0.05)
        self._nav_gh = None
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
        # Подписи в сцене НЕТ: текстовые маркеры висели в воздухе
        # растянутыми строками и мешали смотреть на карту. Колея и курс
        # и так печатаются в строке [ПОИСК] при нахождении рельсов.
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
        if self.rails is None:
            return None
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
        while (self.now() - t0 < float(self.get_parameter('align_timeout').value)
               and not self.cancelled()):
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

    def drive(self, need):
        v = float(self.get_parameter('drive_speed').value)
        span = float(self.get_parameter('window_s').value)
        frac = float(self.get_parameter('progress_frac').value)
        lim = math.radians(float(self.get_parameter('yaw_abort_deg').value))
        tilt = math.radians(float(self.get_parameter('tilt_abort_deg').value))
        hard = int(self.get_parameter('stall_windows').value)
        start = np.array(self.odom[:2])
        self.say('ЗАЕЗД', f'идём вперёд {need:.2f} м на {v:.2f} м/с')
        t0 = self.now()
        last = t0
        bad = 0
        while rclpy.ok() and not self.cancelled():
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
                # ОДНОГО плохого окна мало. Замерено 9 октября 2026 на
                # стенде: окна шли 60, 60, 60, 37, 42, 52, 40 и затем 14 мм
                # при пороге 18 — попытка оборвалась ровно в тот миг, когда
                # на трубы лезли ЗАДНИЕ колёса. А 14 мм за секунду это всё
                # ещё движение, и через секунду робот бы дожал.
                #
                # Запас frac=0.15 заведён под ПОСТОЯННОЕ падение скорости:
                # на рельсах радиус качения падает со 101.4 до 50 мм, и
                # робот идёт вдвое медленнее скомандованного — те самые
                # 60 мм из 120. Нарастающей нагрузки он не покрывает,
                # потому что порог считается от КОМАНДЫ, а она постоянна.
                #
                # Скорость при этом НЕ поднимаем: регулятор, принявший
                # падение за отставание, начал бы разгоняться — ровно от
                # этого заезд и уведён мимо Nav2.
                if went < want * frac:
                    bad += 1
                    self.say('ЗАЕЗД', f'тяжело: за {span:.1f} с прошли '
                                      f'{went * 1000:.0f} мм при ожидаемых '
                                      f'{want * 1000:.0f} — окно {bad} из '
                                      f'{hard}')
                    if bad >= hard:
                        self.stop()
                        self.say('ЗАЕЗД', f'упёрлись: {hard} окна подряд '
                                          f'без продвижения')
                        return False
                else:
                    bad = 0
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
        while rclpy.ok() and self.now() - t0 < 90 and not self.cancelled():
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

    # ── СЪЕЗД ─────────────────────────────────────────────────────────
    def back_off(self, need):
        """Назад вдоль оси рельсов на need метров.

        Чем отличается от ОТКАТА. Тот аварийный: ограничен пройденным
        вперёд путём и нужен лишь затем, чтобы освободить рельсы для
        следующей попытки. Здесь расстояние задаёт вызывающий, и съезд —
        самостоятельная цель со своим признаком успеха.

        Сторож упора тот же, что на заезде, и по той же причине: на трубах
        робот идёт примерно вдвое медленнее скомандованного, так что одного
        плохого окна мало, нужна выдержка.

        Поперечное смещение гасим на ходу, чтобы выезжать по оси, а не по
        косой: иначе борт подставляется под кромку. Если рельсы не найдены,
        rail_error молчит, и съезд идёт просто прямо назад.
        """
        v = float(self.get_parameter('drive_speed').value)
        k = float(self.get_parameter('align_gain').value)
        span = float(self.get_parameter('window_s').value)
        frac = float(self.get_parameter('progress_frac').value)
        hard = int(self.get_parameter('stall_windows').value)
        start = np.array(self.odom[:2])
        self.say('СЪЕЗД', f'отходим назад на {need:.2f} м')
        t0 = self.now()
        last = t0
        bad = 0
        while rclpy.ok() and not self.cancelled():
            gone = float(np.linalg.norm(np.array(self.odom[:2]) - start))
            self.gone = gone
            if gone >= need:
                self.stop()
                self.say('СЪЕЗД', f'съехали, назад {gone:.2f} м')
                return True
            e = self.rail_error()
            vy = max(-0.06, min(0.06, -k * e[0])) if e else 0.0
            wz = max(-0.20, min(0.20, k * e[1])) if e else 0.0
            self.go(vx=-v, vy=vy, wz=wz)
            self.spin(0.05)
            if self.now() - t0 > span + 0.5 and self.now() - last > 0.5:
                last = self.now()
                went = self.moved(span)
                want = v * span
                if went < want * frac:
                    bad += 1
                    self.say('СЪЕЗД', f'тяжело: за {span:.1f} с прошли '
                                      f'{went * 1000:.0f} мм при ожидаемых '
                                      f'{want * 1000:.0f} — окно {bad} из '
                                      f'{hard}')
                    if bad >= hard:
                        self.stop()
                        self.say('СЪЕЗД', f'упёрлись: {hard} окна подряд '
                                          f'без продвижения')
                        return False
                else:
                    bad = 0
                    self.say('СЪЕЗД', f'едем: {gone:.2f} из {need:.2f} м, '
                                      f'за окно {went * 1000:.0f} мм')
            if self.now() - t0 > 4 * need / v + 20:
                self.stop()
                self.say('СЪЕЗД', 'слишком долго')
                return False
        self.stop()
        return False

    # ── действия ──────────────────────────────────────────────────────
    def on_goal(self, goal):
        with self._lock:
            if self._busy:
                self.get_logger().warn('цель отклонена: уже выполняется другая')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, gh):
        # Отмена ТОЛЬКО останавливает. Самовольно откатываться не будем:
        # если робот стоит верхом на трубах, уезжать без спроса опаснее,
        # чем остаться на месте и доложить, где он.
        self.get_logger().warn('пришла отмена — останавливаюсь на месте')
        return CancelResponse.ACCEPT

    def ready(self, sec=5.0):
        """Дождаться одометрии: без неё ни одна фаза не умеет мерить путь."""
        t0 = self.now()
        while self.odom is None and self.now() - t0 < sec and rclpy.ok():
            time.sleep(0.05)
        return self.odom is not None

    def finish(self, gh, result, ok, reason):
        # Сначала говорим, потом завершаем цель: say шлёт обратную связь, а
        # на уже завершённой цели это исключение.
        self.stop()
        self.say('ИТОГ', reason)
        result.success = ok
        result.reason = reason
        result.gone = float(self.gone)
        if gh.is_cancel_requested:
            gh.canceled()
        elif ok:
            gh.succeed()
        else:
            gh.abort()
        return result

    def execute_entry(self, gh):
        res = RailEntry.Result()
        g = gh.request
        # Ноль в цели означает «взять умолчание из параметра узла», так что
        # вызов без аргументов остаётся осмысленным.
        need = float(g.drive_distance) or float(
            self.get_parameter('drive_distance').value)
        tries = int(g.retries) or int(self.get_parameter('retries').value)
        self.attempt = 0
        self.gone = 0.0

        def emit(phase, detail):
            fb = RailEntry.Feedback()
            fb.phase, fb.detail = phase, detail
            fb.attempt = int(self.attempt)
            fb.gone, fb.need = float(self.gone), float(need)
            e = self.rail_error()
            fb.lateral = float(e[0]) if e else 0.0
            fb.yaw_error = float(e[1]) if e else 0.0
            gh.publish_feedback(fb)

        self._gh, self._emit = gh, emit
        try:
            if not self.ready():
                return self.finish(gh, res, False, 'нет одометрии')
            if not self.scan(bool(g.scan)):
                return self.finish(gh, res, False, 'осмотр не удался')
            # ПОИСК И ПОДХОД ВНУТРИ ЦИКЛА, А НЕ ПЕРЕД НИМ.
            #
            # Раньше они шли один раз, и вторая попытка выверялась к
            # ОСИ ИЗ ПЕРВОГО ПОИСКА, с того места, где её оставил откат.
            # Это неверно по двум причинам. Во-первых, подгонка к этому
            # моменту могла уточниться — ради того и затевалось
            # обновление оси, — но обновление отвергает скачки, а между
            # попытками скачок как раз и надо принять. Во-вторых, откат
            # по условию выводит робота ПОЛНОСТЬЮ за пределы рельсов,
            # значит Nav2 снова применим, и ехать на новую точку старта
            # не только можно, но и нужно.
            #
            # Для первой попытки поведение не меняется.
            for attempt in range(1, tries + 1):
                if self.cancelled():
                    break
                self.attempt = attempt
                self.say('ПОПЫТКА', f'номер {attempt}')
                if not self.find_rails():
                    return self.finish(gh, res, False, 'рельсы не найдены')
                if not self.approach():
                    return self.finish(gh, res, False,
                                       'не доехал до точки старта')
                here = np.array(self.odom[:2])
                self.gone = 0.0
                if not self.align():
                    self.retreat(here, self.gone)
                    continue
                if self.drive(need):
                    return self.finish(gh, res, True,
                                       f'ЗАЕХАЛИ с попытки {attempt}')
                self.retreat(here, self.gone)
            if self.cancelled():
                return self.finish(gh, res, False, 'отменено')
            return self.finish(gh, res, False, 'не заехал за все попытки')
        finally:
            self._gh = self._emit = self._nav_gh = None
            with self._lock:
                self._busy = False

    def execute_exit(self, gh):
        res = RailExit.Result()
        need = float(gh.request.distance) or float(
            self.get_parameter('exit_distance').value)
        self.gone = 0.0

        def emit(phase, detail):
            fb = RailExit.Feedback()
            fb.phase, fb.detail = phase, detail
            fb.gone, fb.need = float(self.gone), float(need)
            e = self.rail_error()
            fb.lateral = float(e[0]) if e else 0.0
            fb.yaw_error = float(e[1]) if e else 0.0
            gh.publish_feedback(fb)

        self._gh, self._emit = gh, emit
        try:
            if not self.ready():
                return self.finish(gh, res, False, 'нет одометрии')
            # Ось нужна лишь для поперечной поправки. Не нашлась — съезжаем
            # прямо назад, это хуже, но не повод отказывать: робот стоит на
            # рельсах, и снять его оттуда важнее.
            if self.rails is None:
                self.find_rails()
            ok = self.back_off(need)
            if self.cancelled():
                return self.finish(gh, res, False, 'отменено')
            return self.finish(gh, res, ok, f'съехали, назад {self.gone:.2f} м'
                               if ok else 'съехать не удалось')
        finally:
            self._gh = self._emit = None
            with self._lock:
                self._busy = False


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
    n = RailServer()
    # ИСПОЛНИТЕЛЬ ОБЯЗАТЕЛЬНО МНОГОПОТОЧНЫЙ. Тело действия блокируется на
    # всё время заезда, и в одном потоке подписки за ним в очередь не
    # пролезли бы: одометрия замерла бы ровно на заезде.
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        print()
    finally:
        try:
            n.stop()
        except Exception:
            pass
        try:
            ex.shutdown()
        except Exception:
            pass
        n.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
