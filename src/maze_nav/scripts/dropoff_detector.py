#!/usr/bin/env python3
"""Детектор обрыва: находит край платформы по глубине и выдаёт его
костмапу под видом обычного препятствия.

    ros2 run maze_nav dropoff_detector.py

Зачем подделка. Ни Nav2, ни RTAB-Map отрицательных препятствий не знают:
в их модели мира препятствие — это то, что торчит ВВЕРХ, а всё, что ниже
порога пола, считается свободным местом. Поэтому обрыв за кромкой настила
размечается как ровная площадка, и планировщик спокойно ведёт туда робота.

Обойти это можно, не трогая Nav2: найти клетки обрыва и опубликовать их
облаком точек НА ВЫСОТЕ. Дальше всё работает само — костмап их пометит,
инфляция раздуется, планировщик обойдёт.

Признаков обрыва два, и настоящий край даёт оба сразу:

1. ТОЧКИ НИЖЕ ОПОРНОЙ ПЛОСКОСТИ. Пол за кромкой лежит на 120 мм ниже
   настила. Это не отсутствие данных, а положительное измерение, и оно
   даёт метрическое положение.

2. ТЕНЬ ЗА КРОМКОЙ. Сразу за краем есть полоса, куда не попадает ни один
   луч — её загораживает сама кромка. Ширина считается точно:
       тень = высота обрыва * расстояние / высота камеры
   При кромке в метре впереди и камере на 0.72 м это 17 см пустоты.

Второй признак здесь используется как проверка: шум дальномера даёт
одиночные точки ниже плоскости где угодно, но тени за собой не оставляет.
Поэтому одиночные срабатывания отсеиваются требованием, чтобы рядом были
соседи (минимальный размер пятна).
"""
import math

import numpy as np
import rclpy
import rclpy.duration
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header


class DropoffDetector(Node):
    def __init__(self):
        super().__init__('dropoff_detector')
        self.declare_parameter('base_frame', 'base_footprint')
        # Ниже этой отметки относительно опорной плоскости — обрыв.
        # 60 мм: половина высоты настила. Меньше — ловим шум дальномера,
        # больше — обрыв обнаруживается слишком поздно.
        self.declare_parameter('drop_threshold', -0.06)
        # На какой высоте публиковать подделку. Выше порога пола костмапа
        # и ниже габарита робота, чтобы слой точно её принял.
        self.declare_parameter('virtual_height', 0.30)
        self.declare_parameter('max_range', 3.0)
        # Дальность облака, уходящего в сетку занятости. Меньше общей
        # намеренно: порог сетки «выше пола» постоянный, 30 мм, а шум
        # дальномера растёт как квадрат дальности и на трёх метрах даёт
        # те же 30 мм. Каждая вторая клетка на дальней границе обзора
        # перещёлкивается в препятствие, и по краю карты вырастает кайма
        # ложных препятствий, похожая на рельсы.
        # На 2.0 м сигма 13 мм — порог отстоит на 2.3 сигмы, ложных около
        # процента. Потеря охвата не страшна: робот едет и накапливает.
        self.declare_parameter('grid_max_range', 2.0)
        # Сторона клетки, по которой берём медиану высоты. 5 см заметно
        # меньше трубы Ø51 мм, так что рельс клетку не теряет.
        self.declare_parameter('grid_cell', 0.05)
        # Пятачок под роботом, который считаем свободным по построению.
        # Размер колёсного прямоугольника: полубаза 0.3178 и полуколея
        # 0.3554 м по обмеру CAD.
        self.declare_parameter('free_patch_length', 0.636)
        self.declare_parameter('free_patch_width', 0.711)
        # Насколько отодвигать точки пола от рельса, чтобы они не попали с
        # ним в одну клетку сетки и не размыли его цвет.
        # 10 см, а не 6: карта копится по кадрам, клетка сетки RTAB-Map
        # равна 5 см, и её сетка смещена относительно нашей, поэтому точка
        # пола с соседнего кадра всё равно попадала к рельсу в один воксель
        # и перебивала цвет. Замерено: при 6 см на рельсах оставалось 14
        # серых точек против 19 зелёных.
        self.declare_parameter('rail_clear_cell', 0.10)
        # Прореживание: считать каждый N-й пиксель.
        # Каждый второй пиксель, а не каждый четвёртый. На четвёртом
        # труба 51 мм с двух метров занимает единицы отсчётов и теряется:
        # замерено — рельсы пропадали с карты, оставались точки.
        self.declare_parameter('step', 2)
        self.declare_parameter('min_cluster', 3)
        # Сколько строк подряд должен держаться перелёт, чтобы считать его
        # краем. Щель 4 мм в кадре занимает один-два отсчёта и отсеивается.
        self.declare_parameter('min_run', 8)
        # Порог перелёта идёт от ШУМА ДАЛЬНОМЕРА, а не от высоты ступеньки.
        #
        # Перелёт за кромкой равен h/sin(угол склонения), и у самого края
        # угол крутой, а перелёт МАЛЕНЬКИЙ: при ступеньке 120 мм и кромке
        # в 1.1 м это 0.22 м, а в 2.2 м — уже 0.40 м. Поэтому постоянный
        # порог в метрах отодвигает детектор наружу: он ждёт, пока перелёт
        # дорастёт. Проверено — с порогом 0.25 м кромка определялась на
        # 2.24 м вместо 1.10.
        #
        # У D435 шум растёт как квадрат дальности (он задан в
        # диспаратности), поэтому порог берём таким же: три сигмы шума.
        self.declare_parameter('overshoot_k', 0.010)     # ~3 сигмы, м на м^2
        self.declare_parameter('overshoot_min', 0.03)
        # Кромку ищем только вблизи: дальше пол виден под слишком острым
        # углом, чтобы верить его высоте.
        self.declare_parameter('edge_max_range', 2.5)
        # Сглаживание по времени: медиана дальности кромки в каждом столбце
        # за последние N кадров. Одиночный выброс шума медиану не сдвинет,
        # а настоящая кромка стоит на месте.
        self.declare_parameter('smooth_frames', 5)
        # Окно медианы поперёк столбцов кадра, против отростков на рельсах.
        self.declare_parameter('cross_median', 9)
        # Сколько столбцов подряд без наблюдения ещё разрешено закрыть
        # интерполяцией. 20 из 320 — это около 24 см кромки на двух метрах.
        self.declare_parameter('edge_gap_fill', 20)
        # Сколько строк недолёта достаточно, чтобы счесть столбец занятым
        # предметом. Труба 51 мм с полутора метров занимает не одну строку.
        self.declare_parameter('block_rows', 3)
        # Радиус зоны без кромки вокруг предмета, в метрах. Нужен, чтобы
        # робот мог отъехать назад после неудачного заезда и не упёрся в
        # собственную разметку у рельсов.
        self.declare_parameter('block_radius', 0.30)

        # Высоту рельса меряем над МЕСТНЫМ минимумом, а не над опорной
        # плоскостью. Причина: пол двухуровневый. На настиле рельс торчит
        # на +51 мм, а за кромкой он стоит на нижнем полу и оказывается на
        # 69 мм НИЖЕ плоскости настила. Единой высоты у него нет вовсе,
        # а над соседним полом он всегда на свои 51 мм.
        # Окно для местного минимума. Больше клетки намеренно: труба 51 мм
        # влезает в клетку 5 см целиком, и пола рядом не остаётся — минимум
        # получился бы по самой трубе, и рельс стал бы невидим.
        # Дальше 1.8 м отношение высоты трубы к шуму падает ниже пяти, и
        # кандидатов от шума пола становится больше, чем от рельсов —
        # вписывание начинает цепляться за случайные скопления. Замерено:
        # на 2.5 м курс прыгал между -38° и +35° от кадра к кадру.
        # Для заезда дальше и не нужно: выравниваемся с 1.0-1.5 м.
        # Колея по осям труб — измерена по стенду, см. gen_rails_world.py.
        # Кандидатов рядом с кромкой выбрасываем. У края окно для уровня
        # пола захватывает нижний уровень, и полоса точек прямо по кромке
        # читается приподнятой — получается длинная прямая, которую
        # вписывание принимает за рельс. Замерено: находило курс 86° и
        # смещение 1.08 м, то есть ровно кромку настила.
        # Сглаживание оценки по кадрам: одиночный промах вписывания
        # медиану не сдвинет, а рельсы стоят на месте.
        self._hist = []

        self.base = self.get_parameter('base_frame').value
        self.k = None
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        # ПОДПИСКИ — best effort, как принято для сенсорных данных.
        # Reliable-подписка не принимает сообщения от best-effort издателя
        # вовсе, а узел шума глубины публикует именно best effort. Пока
        # цепочка шла напрямую от моста (он reliable), совпадало случайно;
        # стоило вставить шум — и детектор замолчал, а следом встала карта.
        # Обратное сочетание безопасно: best-effort подписка принимает и от
        # reliable издателя.
        sub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # ПУБЛИКАЦИИ оставляем reliable: их читает RTAB-Map, и такой
        # издатель совместим с подписчиком любого вида.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, 'camera_info', self.on_info, sub_qos)
        self.create_subscription(Image, 'depth', self.on_depth, sub_qos)
        # Маска рельсов из оракула (позже — из модели CV). Без неё детектор
        # работает как раньше, просто все препятствия будут одного цвета.
        self.rail_mask = None
        self.create_subscription(Image, 'rails/mask', self.on_mask, sub_qos)
        self.pub = self.create_publisher(PointCloud2, 'dropoff', qos)
        # Полное облако для сетки занятости: настоящие препятствия как есть,
        # а точки обрыва ПОДНЯТЫ на virtual_height. RTAB-Map строит сетку по
        # нему и размечает обрыв препятствием, ничего не зная про обрывы.
        self.grid = self.create_publisher(PointCloud2, 'grid_cloud', qos)
        self.get_logger().info('детектор обрыва запущен')

    def on_info(self, m):
        self.k = (m.k[0], m.k[4], m.k[2], m.k[5])

    def on_mask(self, m):
        self.rail_mask = np.frombuffer(
            m.data, np.uint8).reshape(m.height, m.width) > 0

    def on_depth(self, m):
        if self.k is None:
            return
        # Преобразование берём НА МОМЕНТ СНИМКА, а не последнее доступное.
        # Пока робот стоит, разницы нет, и все замеры сходились. Но стоит
        # поехать — и облако ляжет туда, где робот оказался к моменту
        # обработки, а не туда, откуда снимал. При 8 кадрах в секунду и
        # скорости 0.3 м/с это промах в несколько сантиметров каждый кадр,
        # и карта начнёт размазываться тем сильнее, чем быстрее едем.
        stamp = rclpy.time.Time.from_msg(m.header.stamp)
        try:
            tf = self.buf.lookup_transform(self.base, m.header.frame_id, stamp,
                                           timeout=rclpy.duration.Duration(
                                               seconds=0.1))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'нет TF {self.base} <- {m.header.frame_id}: {e}',
                                   throttle_duration_sec=5.0)
            return

        step = int(self.get_parameter('step').value)
        rng = float(self.get_parameter('max_range').value)
        drop = float(self.get_parameter('drop_threshold').value)
        h = float(self.get_parameter('virtual_height').value)

        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)[::step, ::step]
        fx, fy, cx, cy = self.k
        uu, vv = np.meshgrid((np.arange(0, m.width, step) - cx) / fx,
                             (np.arange(0, m.height, step) - cy) / fy)
        ok = np.isfinite(d) & (d > 0.1) & (d < rng)
        if ok.sum() < 20:
            self.publish([], m.header.stamp)
            return

        q, t = tf.transform.rotation, tf.transform.translation
        R = self.quat_to_mat(q.x, q.y, q.z, q.w)
        O = np.array([t.x, t.y, t.z])
        # Оптический кадр даёт направление (xn, yn, 1), поэтому длина
        # вдоль луча совпадает со значением глубины — масштабировать не надо.
        dopt = np.stack([uu, vv, np.ones_like(uu)], -1)
        dirs = dopt @ R.T
        P = (dopt * d[..., None]) @ R.T + O

        k_over = float(self.get_parameter('overshoot_k').value)
        floor_m = float(self.get_parameter('overshoot_min').value)
        over, s_plane = self.edge_by_ray(dirs, d, O, k_over, floor_m)
        over &= ok & (s_plane < float(self.get_parameter('edge_max_range').value))

        # ═══ Столбцы, где стоит предмет ═══
        # Рельс выше опорной плоскости, поэтому луч встречает его РАНЬШЕ
        # расчётного — это недолёт, знак обратный перелёту. Такие столбцы
        # выбрасываем целиком: кромку там всё равно загораживает труба, а
        # ложная стена поперёк рельсов помешает роботу отъехать назад
        # после неудачного заезда.
        margin = np.maximum(floor_m, k_over * s_plane ** 2)
        under = ok & (s_plane > 0.2) & (d < s_plane - margin) & \
            (s_plane < float(self.get_parameter('edge_max_range').value))
        blocked = under.sum(axis=0) >= int(self.get_parameter('block_rows').value)
        over[:, blocked] = False

        # Запоминаем, ГДЕ стоят препятствия: по одной точке на столбец,
        # ближайший недолёт. Зону без кромки зададим от них в МЕТРАХ.
        # Раньше я расширял её на соседние столбцы кадра, но ширина
        # столбца зависит от дальности — 5 мм на 1.1 м и 9.5 мм на 2 м, —
        # и одна и та же «четвёрка столбцов» давала разную зону.
        obst = np.zeros((0, 2))
        if blocked.any():
            cols_b = np.where(blocked)[0]
            rows_b = under.shape[0] - 1 - np.argmax(under[::-1][:, cols_b], axis=0)
            obst = P[rows_b, cols_b][:, :2]
        # В каждом столбце кадра берём ПЕРВЫЙ перелёт, считая от близкого
        # края. Нижняя строка кадра — ближняя земля, поэтому ищем самый
        # большой номер строки, где луч ещё перелетел. Без этого кромкой
        # окажется вся площадь за краем, а не её граница.
        # Перелёт должен держаться подряд: одиночный отсчёт — это шум или
        # узкая щель, а не край. Настоящий обрыв перелетают все лучи
        # дальше кромки, то есть подряд до самого верха кадра.
        run = int(self.get_parameter('min_run').value)
        # В кадре нижние строки — ближняя земля, верхние — дальняя.
        # Перелетают лучи, смотрящие ДАЛЬШЕ кромки, то есть строки ВЫШЕ.
        # Значит проверять надо строки с МЕНЬШИМ номером. Обратное условие
        # выполняется только вдали от края, и детектор убегает туда:
        # замерено — кромка определялась на 2.45 м вместо 1.1.
        sust = over.copy()
        for k in range(1, run):
            sust[k:] &= over[:-k]
        has = sust.any(axis=0)
        first = sust.shape[0] - 1 - np.argmax(sust[::-1], axis=0)
        # Медиана по последним кадрам: гасит дрожание от шума глубины.
        dist = np.where(has, s_plane[first, np.arange(len(has))], np.nan)
        n = int(self.get_parameter('smooth_frames').value)
        self._hist.append(dist)
        del self._hist[:-n]
        if len(self._hist) > 1 and all(h.shape == dist.shape for h in self._hist):
            with np.errstate(all='ignore'):
                dist = np.nanmedian(np.stack(self._hist), axis=0)
        # Медиана ПОПЕРЁК столбцов. В столбцах, где стоит рельс, луч
        # встречает трубу раньше плоскости, перелёта там нет, и первый
        # перелёт случается уже ЗА рельсом — точка кромки уезжает наружу.
        # На карте это выглядит как отростки кромки вдоль рельсов.
        # Таких столбцов мало, их значение — выброс, и медиана по соседям
        # возвращает его на линию края.
        w = int(self.get_parameter('cross_median').value)
        if w >= 3 and np.isfinite(dist).sum() > w:
            pad = np.pad(dist, w // 2, mode='edge')
            win = np.lib.stride_tricks.sliding_window_view(pad, w)
            with np.errstate(all='ignore'):
                dist = np.nanmedian(win, axis=1)

        # ЗАДЕЛКА КОРОТКИХ ПРОВАЛОВ по столбцам.
        #
        # Замерено: на метре от робота кромка закрыта на 100%, на двух — на
        # 42%. Шум растёт как квадрат дальности, и требование «перелёт
        # держится подряд» вдали срывается то в одном столбце, то в другом.
        # Граница получается пунктиром, а пунктир для Nav2 хуже, чем
        # отсутствие границы: в дырку between точками планировщик проложит
        # путь, и робот поедет за край.
        #
        # Кромка настила непрерывна, поэтому столбец без наблюдения между
        # двумя наблюдениями можно закрыть интерполяцией. Заделываем только
        # КОРОТКИЕ провалы и только внутри кадра: на краю кадра продолжать
        # линию не из чего, а длинный провал — это уже не шум, а рельс или
        # предмет, и придумывать там кромку нельзя.
        #
        # Честно говоря, на покрытие периметра это НЕ повлияло: 76/42/50%
        # до и 72/42/50% после. Провалы вдали оказались не одиночными
        # срывами, а систематическими. Держать робота на настиле должна не
        # сплошность линии, а то, что за кромкой клетки остаются
        # неизвестными. Заделка — дешёвая страховка от одиночных дырок, но
        # считать её решением нельзя.
        gap = int(self.get_parameter('edge_gap_fill').value)
        good = np.isfinite(dist)
        if gap > 0 and good.sum() >= 2:
            idx = np.arange(len(dist))
            filled = np.interp(idx, idx[good], dist[good])
            bad = (~good).astype(np.int8)
            edges_ = np.diff(np.r_[0, bad, 0])
            for a, b in zip(np.flatnonzero(edges_ == 1),
                            np.flatnonzero(edges_ == -1)):
                if a > 0 and b < len(dist) and b - a <= gap:
                    dist[a:b] = filled[a:b]

        has = np.isfinite(dist)
        cols = np.where(has)[0]
        rows = first[cols]
        # Точку кромки строим по СГЛАЖЕННОЙ дальности, а направление луча
        # берём из того же столбца — оно от шума не зависит.
        edge = (O + dist[cols, None] * dirs[rows, cols]) if len(cols) \
            else np.zeros((0, 3))
        # Рельсы из маски тоже задают зону без кромки, и делают это точнее
        # недолёта: недолёт ловит трубу только там, где луч в неё попал, а
        # маска знает всю трубу целиком. Из-за этой разницы одна точка
        # кромки садилась прямо между рельсами.
        rail = None
        if self.rail_mask is not None and \
                self.rail_mask[::step, ::step].shape == d.shape:
            rail = self.rail_mask[::step, ::step] & ok
            rp = P[rail][:, :2]
            if len(rp):
                obst = np.vstack([obst, rp[::max(1, len(rp) // 400)]])

        # Зона без кромки вокруг предметов, в метрах.
        r_block = float(self.get_parameter('block_radius').value)
        if len(edge) and len(obst):
            dmin = np.linalg.norm(edge[:, None, :2] - obst[None], axis=2).min(1)
            edge = edge[dmin > r_block]

        flat_ok = ok.reshape(-1)
        Pf = P.reshape(-1, 3)[flat_ok]
        railf = (rail.reshape(-1)[flat_ok] if rail is not None
                 else np.zeros(len(Pf), bool))
        low = Pf[:, 2] < drop
        self.report(Pf, low)

        # Облако для сетки: НИЖНИЙ УРОВЕНЬ В ОБЛАКО НЕ ИДЁТ ВОВСЕ.
        #
        # Раньше он подтягивался к опорной плоскости — пол ведь проезжий,
        # просто с другого уровня. Для стенда это неверно и опасно: площадь
        # за краем настила ложилась на карту СВОБОДНОЙ, и удерживала
        # планировщик одна тонкая линия края, а она местами рвётся.
        #
        # Поднимать нижний уровень на виртуальную высоту тоже не годится:
        # красным заливается всё поле зрения и карта перестаёт читаться.
        #
        # Решение проще обоих: не публиковать его совсем. Луч в ту сторону
        # не отправляется, трассировка там ничего не расчищает, и клетки
        # остаются НЕИЗВЕСТНЫМИ. Для Nav2 неизвестность непроезжая
        # (track_unknown_space у костмапа, allow_unknown: false у
        # планировщика), так что робот за край не поедет, а на карте
        # остаётся только линия кромки.
        G = Pf.copy()
        near = (np.linalg.norm(G[:, :2], axis=1)
                < float(self.get_parameter('grid_max_range').value)) & ~low

        # МЕДИАНА ПО КЛЕТКЕ вместо сырых точек.
        #
        # Шум дальномера растёт как квадрат дальности, и на двух метрах
        # его 95-й процентиль равен +30 мм — ровно порог, выше которого
        # RTAB-Map считает точку препятствием. Поэтому на ровной пустой
        # платформе каждая двадцатая точка перещёлкивалась в препятствие,
        # и по дальнему краю обзора высыпала зелёная сыпь.
        #
        # Шум независим по пикселям и симметричен, поэтому медиана по
        # клетке его срезает. Рельсы медианятся ОТДЕЛЬНО от пола: труба
        # тонкая, и в общей клетке точки пола её бы пересилили.
        cell = float(self.get_parameter('grid_cell').value)
        q_rail = self.cell_median(G[near & railf], cell)
        q_free = self.cell_median(G[near & ~railf], cell)

        # Вокруг рельса точки пола убираем. Сетка RTAB-Map усредняет цвет в
        # своём вокселе, и труба шириной 51 мм внутри клетки 50 мм всегда
        # оказывается в меньшинстве: замерено — в облаке детектора цвета
        # чистые, а после сетки вместо зелёного выходит (125,207,142), и
        # рельс на карте выглядит как обычное препятствие. Точки пола там
        # всё равно лишние: клетку занимает труба.
        clear = float(self.get_parameter('rail_clear_cell').value)
        if len(q_rail) and len(q_free):
            dmin = np.linalg.norm(q_free[:, None, :2] - q_rail[None, :, :2],
                                  axis=2).min(1)
            q_free = q_free[dmin > clear]

        # ПЯТАЧОК ПОД РОБОТОМ.
        #
        # Камера начинает видеть примерно с 0.6 м, поэтому площадь прямо под
        # роботом не наблюдается никогда и остаётся на карте неизвестной.
        # Для Nav2 неизвестность непроезжая, и робот оказывается в дыре:
        # замерено — клетка под ним имеет стоимость -1, вокруг 226
        # неизвестных клеток из 676, планировщик отказывается строить путь
        # от такого старта и цель падает с «Goal failed».
        #
        # Но эта площадь известна свободной по построению: робот на ней
        # стоит. Поэтому доклеиваем её сами.
        #
        # Пятачок намеренно МЕНЬШЕ корпуса. Корпус 1.22 x 0.90 м свисает за
        # колёса примерно на 0.28 м спереди и сзади, и у кромки настила этот
        # свес висит над обрывом. Объявить свободным то, подо что робот
        # свесился, — ровно та ошибка, из-за которой он и уезжал за край.
        # Колёсный прямоугольник 0.636 x 0.711 м опирается на опору всегда.
        fl = float(self.get_parameter('free_patch_length').value)
        fw = float(self.get_parameter('free_patch_width').value)
        if fl > 0 and fw > 0:
            gx, gy = np.meshgrid(np.arange(-fl / 2, fl / 2 + 1e-9, cell),
                                 np.arange(-fw / 2, fw / 2 + 1e-9, cell))
            patch = np.column_stack([gx.ravel(), gy.ravel(),
                                     np.zeros(gx.size)])
            q_free = np.vstack([q_free, patch]) if len(q_free) else patch

        # Цвет кладём в сами точки: RTAB-Map сливает кромку, рельсы и всё
        # прочее в одно облако препятствий, и покрасить их в RViz по
        # отдельности нельзя — цвет там задаётся на слой целиком.
        parts, cols = [], []
        for q, colour in ((q_free, (190, 190, 195)), (q_rail, (60, 225, 90))):
            if len(q):
                parts.append(q)
                cols.append(np.tile(np.array(colour, np.uint32), (len(q), 1)))
        if len(edge):
            parts.append(np.column_stack(
                [edge[:, 0], edge[:, 1], np.full(len(edge), h)]))
            cols.append(np.tile(np.array([235, 45, 45], np.uint32),
                                (len(edge), 1)))
        G = np.vstack(parts) if parts else np.zeros((0, 3))
        C = np.vstack(cols) if cols else np.zeros((0, 3), np.uint32)

        # ВОЗВРАЩАЕМ облако в кадр КАМЕРЫ, хотя считали всё в кадре робота.
        #
        # RTAB-Map берёт начало координат кадра облака за точку наблюдения и
        # прочерчивает лучи оттуда. Если отдать облако в base_footprint, он
        # считает, что сенсор стоит под роботом на уровне пола, и чистит
        # свободным всё вокруг этой точки — включая площадь ПОД роботом,
        # которую камера физически не видит. На карте это выглядит как
        # свободная область там, где наблюдений не было вовсе.
        #
        # Камера же стоит в 0.47 м впереди и на 0.74 м выше, и лучи из неё
        # идут совсем иначе. Поэтому пересчитываем обратно: G_cam = R^T (G - O).
        Gc = (G - O) @ R
        self.grid.publish(self.cloud(Gc.astype(np.float32), m.header.stamp,
                                     frame=m.header.frame_id, cols=C))

        out = np.column_stack([edge[:, 0], edge[:, 1],
                               np.full(len(edge), h)]).astype(np.float32) \
            if len(edge) else np.zeros((0, 3), np.float32)
        self.publish(out, m.header.stamp)

    def edge_by_ray(self, dirs, depth, O, k=0.010, floor_m=0.03):
        """Кромка там, где луч перестал попадать в опорную плоскость.

        Для каждого пикселя считаем, на каком расстоянии луч встретил бы
        плоскость z=0 (та, на которой стоит робот). Если измеренная
        глубина заметно больше — луч перелетел край и упал на нижний
        уровень. Граница между «попал» и «перелетел» и есть кромка.

        Признак геометрический, а не статистический, и поэтому не зависит
        ни от плотности точек, ни от того, что попало в клетку. До этого
        я трижды пробовал эвристики на сетке — раздувание опоры, точки
        уровня пола, перепад минимальной высоты, — и каждая спотыкалась
        по-своему: бок рельса даёт точки на всех высотах сразу, а на
        дистанции в клетке может вообще не оказаться отсчётов, и пустая
        клетка выглядит опорой.

        Рельсы этому признаку не мешают: препятствие встречается РАНЬШЕ
        ожидаемого, а не позже, и на кромку не похоже.

        Возвращает маску тех точек, что стоят последними перед перелётом.
        """
        # Работаем с любой формой: сюда приходит сетка кадра (H, W, 3).
        dz = dirs[..., 2]
        # Луч должен идти вниз, иначе плоскости он не встретит вовсе.
        down = dz < -1e-3
        s_plane = np.full(dz.shape, np.inf, np.float32)
        s_plane[down] = -O[2] / dz[down]
        margin = np.maximum(floor_m, k * s_plane ** 2)
        over = down & (depth > s_plane + margin) & (s_plane > 0.2)
        return over, s_plane

    def report(self, P, low):
        """Раз в пять секунд говорим, что вообще видим. Без этого молчащий
        детектор неотличим от сломанного."""
        self.get_logger().info(
            f'точек {len(P)}, высота {P[:, 2].min():+.2f}..{P[:, 2].max():+.2f} м, '
            f'ниже порога {int(low.sum())}', throttle_duration_sec=5.0)

    @staticmethod
    def cell_median(G, cell):
        """По одной точке на клетку — той, что с медианной высотой."""
        if not len(G) or cell <= 0:
            return G
        key = np.floor(G[:, :2] / cell).astype(np.int64)
        kk = key[:, 0] * 1000003 + key[:, 1]
        order = np.lexsort((G[:, 2], kk))
        Gs, ks = G[order], kk[order]
        beg = np.flatnonzero(np.r_[True, ks[1:] != ks[:-1]])
        end = np.r_[beg[1:], len(ks)]
        return Gs[(beg + end) // 2]

    def cloud(self, pts, stamp, frame=None, cols=None):
        hdr = Header(stamp=stamp, frame_id=frame or self.base)
        fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32,
                             count=1) for i, n in enumerate(('x', 'y', 'z'))]
        if cols is None:
            return pc2.create_cloud(
                hdr, fields, pts if len(pts) else np.zeros((0, 3), np.float32))
        fields.append(PointField(name='rgb', offset=12,
                                 datatype=PointField.FLOAT32, count=1))
        if not len(pts):
            return pc2.create_cloud(hdr, fields, np.zeros((0, 4), np.float32))
        c = cols.astype(np.uint32)
        packed = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
        rgb = np.ascontiguousarray(packed, np.uint32).view(np.float32)
        return pc2.create_cloud(
            hdr, fields,
            np.column_stack([pts.astype(np.float32), rgb]).astype(np.float32))

    def publish(self, pts, stamp):
        self.pub.publish(self.cloud(pts, stamp))

    @staticmethod
    def quat_to_mat(x, y, z, w):
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    rclpy.init()
    n = DropoffDetector()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
