#!/usr/bin/env python3
"""Редактор графа маршрутов поверх карты полигона.

Точки ставятся на полосы, рёбра задают разрешённые переезды. Проехав по
такому графу, робот не нарушает правил: встречные потоки разведены по
разным полосам, а развороты просто отсутствуют как рёбра.

    ./route_editor.py --map ../maps/city.yaml --graph ../maps/city_routes.yaml

Если файла графа нет, он генерируется из регламента и его можно править.

Управление
  ЛКМ по пустому         добавить точку (направление — текущее, см. N/S/E/W)
  ЛКМ по точке + тащить  двигать точку
  ПКМ по точке           удалить точку вместе с её рёбрами
  СКМ или Shift+ЛКМ      от точки к точке — создать ребро
  ПКМ по ребру           удалить ребро
  N S E W                направление для новых точек
  I                      показать/скрыть полные id точек
  T                      пометить/снять «стоп» у точки под курсором
  R                      перегенерировать граф из регламента с нуля
  U                      пересчитать нумерацию обходом графа
  C                      сбросить подсветку проезда

Симуляция без Gazebo и Nav2 — проверяется та же логика выбора точек:
  G                      назначить/снять цель (без цели — режим исследования)
  P                      поставить робота в точку под курсором
  пробел                 старт / пауза
  Backspace              сброс симуляции
  1 2 3                  повесить знак no_left / no_right / no_straight
  M                      расставить знаки полигона: направляющие плюс зоны
                         посадки («место остановки») и высадки («стоянка»)
  0                      снять все знаки
  Ctrl+S                 сохранить
  колесо                 масштаб, перетаскивание фона — панорама

Пока робот едет, редактор подсвечивает маршрут: зелёным — пройденные
точки, оранжевым — ту, куда едет сейчас, синим — конечную цель. Данные
берутся из /route_state, который публикует route_follower.
"""
import argparse
import copy
import math
import os
import random
import sys

from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import (QPainter, QPen, QBrush, QColor, QImage, QPolygonF,
                         QFont, QKeySequence)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import (QApplication, QWidget, QMainWindow, QFileDialog,
                             QLabel, QShortcut, QMessageBox, QPushButton,
                             QHBoxLayout, QVBoxLayout, QCheckBox)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from maze_nav.route_graph import (RouteGraph, Waypoint, RouteEdge, DIR_NAME,
                                  NAME_DIR, NORTH, SOUTH, EAST, WEST, right_of)

NODE_R = 13          # радиус точки на экране, пиксели
PICK_R = 16          # радиус попадания мышью

# Знаки полигона по Приложению Б регламента. Управляют выбором рёбер:
# предписывающие Б.1-Б.3 и запрещающие Б.4-Б.5. Запрета движения прямо
# в регламенте нет, поэтому случайно он не ставится, хотя граф его понимает.
SIGN_KINDS = ('only_straight', 'only_left', 'only_right',
              'no_left', 'no_right')
# Б.6 «место остановки» — зона посадки, стоим 2 секунды; Б.7 «место
# стоянки» — зона высадки и конец маршрута. Рёбра они не трогают.
SIGN_STOP = 'stop'
SIGN_PARKING = 'parking'
STOP_PAUSE_MS = 2000        # регламент: «остановился на 2 секунды»
SIGN_LETTER = {'only_straight': 'S', 'only_left': 'L', 'only_right': 'R',
               'no_straight': 'S', 'no_left': 'L', 'no_right': 'R',
               SIGN_STOP: 'O', SIGN_PARKING: 'P'}
SIGN_R = 11          # радиус кружка знака на экране

# При русской раскладке ev.key() возвращает код кириллической буквы, и ни
# одно латинское сочетание не срабатывает: нажатие M приходит как «ь».
# Приводим букву к латинице по физическому месту клавиши на ЙЦУКЕН.
RU_TO_EN = {'ь': 'M', 'т': 'N', 'ы': 'S', 'у': 'E', 'ц': 'W', 'п': 'G',
            'з': 'P', 'ш': 'I', 'е': 'T', 'к': 'R', 'г': 'U', 'с': 'C'}


def load_map(yaml_path):
    """Возвращает (QImage, resolution, origin_x, origin_y)."""
    import yaml
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(yaml_path), img_path)
    img = QImage(img_path)
    if img.isNull():
        raise RuntimeError(f"не удалось открыть {img_path}")
    ox, oy = meta['origin'][0], meta['origin'][1]
    return img, float(meta['resolution']), ox, oy


class RosBridge(QThread):
    """Связь с ROS: слушает состояние и позу робота, отправляет команды.

    Живёт в отдельном потоке: rclpy.spin блокирующий, а Qt должен рисовать.
    Сигналы доставляют данные в поток интерфейса, поэтому перерисовка
    происходит там, где положено.
    """
    got_state = pyqtSignal(str)
    got_pose = pyqtSignal(float, float, float)
    ready = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.node = None
        self.pub_start = None
        self.pub_sign = None

    def run(self):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
            from tf2_ros import Buffer, TransformListener
        except ImportError:
            self.ready.emit(False)
            return
        import math as _m
        rclpy.init()
        self.node = Node('route_editor')
        self.node.create_subscription(String, '/route_state',
                                      lambda m: self.got_state.emit(m.data), 10)
        self.pub_start = self.node.create_publisher(String, '/route_start', 10)
        self.pub_sign = self.node.create_publisher(String, '/route_sign', 10)

        buf = Buffer()
        TransformListener(buf, self.node)

        def tick():
            try:
                t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            except Exception:
                return
            q = t.transform.rotation
            self.got_pose.emit(t.transform.translation.x,
                               t.transform.translation.y,
                               _m.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))

        self.node.create_timer(0.1, tick)
        self.ready.emit(True)
        try:
            rclpy.spin(self.node)
        except Exception:
            pass
        finally:
            # Контекст мог быть уже закрыт при выходе из приложения —
            # тогда повторный shutdown бросает RCLError.
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def send_start(self, goal: str = ''):
        if self.pub_start is None:
            return False
        from std_msgs.msg import String
        m = String(); m.data = goal
        self.pub_start.publish(m)
        return True

    def send_sign(self, node_id: str, sign: str):
        if self.pub_sign is None:
            return False
        from std_msgs.msg import String
        m = String(); m.data = f"{node_id}:{sign}"
        self.pub_sign.publish(m)
        return True


class Canvas(QWidget):
    def __init__(self, graph, img, res, ox, oy, status):
        super().__init__()
        self.g = graph
        self.img = img
        self.res, self.ox, self.oy = res, ox, oy
        self.status = status
        self.scale = 120.0          # пикселей экрана на метр
        self.pan = QPointF(0, 0)
        self.new_dir = NORTH
        self.drag_node = None
        self.edge_from = None
        self.panning = False
        self.show_ids = False
        self.passed = set()      # точки, которые робот уже прошёл
        self.passed_edges = set() # рёбра, по которым проехали
        self.visits = {}         # сколько раз были в каждой точке
        self.current = None      # куда едет прямо сейчас
        self.goal = None         # конечная цель маршрута
        # Знаки, расставленные по полигону. На схеме они видны сразу, но на
        # граф действуют только после «распознавания» — когда робот приедет
        # в точку. Так проверяется вся цепочка целиком: увидели знак,
        # применили, рёбра отключились, маршрут пересчитался на ходу.
        self.placed_signs = {}    # id точки -> знак, ещё не распознан
        self.detected_signs = {}  # id точки -> знак, уже применён к графу
        self.parked = False       # доехали до зоны высадки, маршрут закончен
        # Симуляция: робот едет по графу без Nav2 и Gazebo — проверяется
        # ровно та же логика выбора точек, что и в route_follower.
        self.robot = None        # реальная поза из TF: (x, y, yaw)
        self.sim_at = None       # точка, где робот стоит
        self.sim_to = None       # куда едет
        self.sim_t = 0.0         # доля пути между ними, 0..1
        self.sim_path = []       # текущий план, для отрисовки
        self.sim_timer = QTimer(self)
        self.sim_timer.timeout.connect(self._sim_tick)
        self.sim_speed = 0.04    # доля отрезка за тик
        self.last_mouse = QPointF()
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

    # --- преобразования мир <-> экран ---
    def w2s(self, x, y):
        return QPointF(x * self.scale + self.pan.x() + self.width() / 2,
                       -y * self.scale + self.pan.y() + self.height() / 2)

    def s2w(self, p):
        return ((p.x() - self.pan.x() - self.width() / 2) / self.scale,
                -(p.y() - self.pan.y() - self.height() / 2) / self.scale)

    def node_at(self, p):
        for nid, n in self.g.nodes.items():
            if (self.w2s(n.x, n.y) - p).manhattanLength() < PICK_R * 1.5:
                return nid
        return None

    def edge_at(self, p):
        wx, wy = self.s2w(p)
        best, bd = None, 0.06     # порог в метрах
        for key, e in self.g.edges.items():
            a, b = self.g.nodes.get(e.src), self.g.nodes.get(e.dst)
            if not a or not b:
                continue
            # расстояние от точки до отрезка
            vx, vy = b.x - a.x, b.y - a.y
            L2 = vx * vx + vy * vy
            if L2 == 0:
                continue
            t = max(0.0, min(1.0, ((wx - a.x) * vx + (wy - a.y) * vy) / L2))
            d = math.hypot(wx - (a.x + t * vx), wy - (a.y + t * vy))
            if d < bd:
                best, bd = key, d
        return best

    # --- отрисовка ---
    def paintEvent(self, _):
        q = QPainter(self)
        q.setRenderHint(QPainter.Antialiasing)
        q.fillRect(self.rect(), QColor(60, 60, 64))

        # карта: origin — нижний левый угол картинки
        w_m = self.img.width() * self.res
        h_m = self.img.height() * self.res
        tl = self.w2s(self.ox, self.oy + h_m)
        q.drawImage(QRectF(tl.x(), tl.y(), w_m * self.scale, h_m * self.scale), self.img)

        # Рёбра одного цвета; цветом отмечается только то, что уже проехали,
        # иначе по схеме не видно, где робот побывал.
        for key, e in self.g.edges.items():
            a, b = self.g.nodes.get(e.src), self.g.nodes.get(e.dst)
            if not a or not b:
                continue
            if not e.enabled:
                col, w = QColor(110, 110, 110), 1        # отключено знаком
            elif key in self.passed_edges:
                col, w = QColor(60, 200, 90), 3          # проехали
            else:
                col, w = QColor(150, 160, 175), 2        # ещё нет
            self._arrow(q, self.w2s(a.x, a.y), self.w2s(b.x, b.y), col, w,
                        dashed=not e.enabled)

        # точки
        q.setFont(QFont('DejaVu Sans', 12, QFont.Bold))
        # Рисуем n.index — номер из обхода графа, тот же, что в YAML и на
        # схемах. Раньше здесь был порядок в словаре, и номера в редакторе
        # расходились с номерами везде ещё.
        for nid, n in sorted(self.g.nodes.items(), key=lambda kv: kv[1].index):
            i = n.index
            p = self.w2s(n.x, n.y)
            # Точки одного цвета: цветом показывается только проезд, иначе
            # раскраска по типу мешает следить за роботом.
            fill = QColor(250, 250, 250)
            if nid in self.passed:
                fill = QColor(120, 210, 120)        # уже проехали
            if nid == self.current:
                fill = QColor(255, 165, 40)         # едем сюда сейчас
            if nid == self.goal:
                fill = QColor(120, 180, 255)        # конечная цель
            if n.stop:
                fill = QColor(255, 70, 70)
            q.setPen(QPen(QColor(20, 20, 20), 2))
            q.setBrush(QBrush(fill))
            q.drawEllipse(p, NODE_R, NODE_R)

            # стрелка направления; у перекрёстка направления нет
            if n.direction:
                d = n.direction
                tip = QPointF(p.x() + d[1] * NODE_R * 2.0, p.y() - d[0] * NODE_R * 2.0)
                self._arrow(q, p, tip, QColor(20, 20, 20), 2, head=6)

            # номер по порядку и, при наведении, полный id
            q.setPen(QColor(10, 10, 10))
            q.drawText(QRectF(p.x() - NODE_R, p.y() - NODE_R,
                              NODE_R * 2, NODE_R * 2), Qt.AlignCenter, str(i))
            if self.show_ids:
                q.setPen(QColor(255, 255, 120))
                q.drawText(QPointF(p.x() + NODE_R + 3, p.y() - NODE_R - 2), nid)

        # знаки. Пока не распознан — тусклый серый кружок: знак стоит, но
        # робот его ещё не видел и граф не тронут. После распознавания
        # кружок загорается: красный ободок у запрещающих, синяя заливка
        # у предписывающих — как на настоящих дорожных знаках.
        for nid, sign in self.placed_signs.items():
            n = self.g.nodes.get(nid)
            if not n:
                continue
            p = self.w2s(n.x, n.y)
            c = QPointF(p.x() + NODE_R + SIGN_R - 2, p.y() - NODE_R - SIGN_R + 2)
            # Синие — предписывающие и информационные (остановка, стоянка),
            # с красным ободком — запрещающие. Как в ГОСТ Р 52289.
            blue = sign.startswith('only_') or sign in (SIGN_STOP, SIGN_PARKING)
            if nid not in self.detected_signs:
                ring, fill, ink = (QColor(120, 120, 125), QColor(70, 70, 75),
                                   QColor(170, 170, 175))
            elif blue:
                ring, fill, ink = (QColor(10, 10, 10), QColor(30, 90, 200),
                                   QColor(255, 255, 255))
            else:
                ring, fill, ink = (QColor(220, 40, 40), QColor(255, 255, 255),
                                   QColor(20, 20, 20))
            q.setPen(QPen(ring, 3))
            q.setBrush(QBrush(fill))
            q.drawEllipse(c, SIGN_R, SIGN_R)
            q.setPen(ink)
            q.setFont(QFont('DejaVu Sans', 9, QFont.Bold))
            q.drawText(QRectF(c.x() - SIGN_R, c.y() - SIGN_R, SIGN_R * 2, SIGN_R * 2),
                       Qt.AlignCenter, SIGN_LETTER.get(sign, '?'))

        # план симуляции — толстой полупрозрачной линией
        if self.sim_path:
            q.setPen(QPen(QColor(255, 0, 200, 90), 8))
            for a, b in zip(self.sim_path, self.sim_path[1:]):
                na, nb = self.g.nodes.get(a), self.g.nodes.get(b)
                if na and nb:
                    q.drawLine(self.w2s(na.x, na.y), self.w2s(nb.x, nb.y))

        # создаваемое ребро
        if self.edge_from and self.edge_from in self.g.nodes:
            n = self.g.nodes[self.edge_from]
            q.setPen(QPen(QColor(255, 0, 200), 2, Qt.DashLine))
            q.drawLine(self.w2s(n.x, n.y), self.last_mouse)

        # реальный робот из TF — синий треугольник
        if self.robot:
            rx, ry, rang = self.robot
            p = self.w2s(rx, ry)
            q.setPen(QPen(QColor(0, 0, 0), 2))
            q.setBrush(QBrush(QColor(60, 130, 255)))
            pts = [QPointF(p.x() + r * math.cos(rang + da),
                           p.y() - r * math.sin(rang + da))
                   for da, r in ((0, 18), (2.5, 11), (-2.5, 11))]
            q.drawPolygon(QPolygonF(pts))

        # робот в симуляции — красный
        if self.sim_at:
            a = self.g.nodes[self.sim_at]
            if self.sim_to:
                b = self.g.nodes[self.sim_to]
                rx = a.x + (b.x - a.x) * self.sim_t
                ry = a.y + (b.y - a.y) * self.sim_t
                ang = math.atan2(b.y - a.y, b.x - a.x)
            else:
                rx, ry, ang = a.x, a.y, a.yaw
            p = self.w2s(rx, ry)
            q.setPen(QPen(QColor(0, 0, 0), 2))
            q.setBrush(QBrush(QColor(255, 60, 60)))
            # треугольник носом по курсу
            pts = []
            for da, r in ((0, 16), (2.5, 10), (-2.5, 10)):
                pts.append(QPointF(p.x() + r * math.cos(ang + da),
                                   p.y() - r * math.sin(ang + da)))
            q.drawPolygon(QPolygonF(pts))

        # легенда
        q.setPen(QColor(240, 240, 240))
        q.setFont(QFont('DejaVu Sans', 10))
        q.drawText(10, 18, f"направление новых точек: {DIR_NAME[self.new_dir]}   "
                           f"точек: {len(self.g.nodes)}   рёбер: {len(self.g.edges)}")
        q.drawText(10, 34, "рёбра: серые не пройдены, зелёные пройдены, "
                           "пунктир — отключены знаком")
        q.drawText(10, 50, "точки: зелёные пройдены, оранжевая текущая, "
                           "синяя цель, красные стоп")
        q.drawText(10, 66, "I — id, T — стоп, R — перегенерировать, Ctrl+S — сохранить")
        q.drawText(10, 82, "симуляция: G цель, P поставить робота, пробел старт/пауза, "
                           "1/2/3 знаки, M случайные знаки, 0 снять, Backspace сброс")
        q.drawText(10, 98, "красный треугольник — симуляция, синий — реальный робот из TF")
        q.drawText(10, 114, "знаки: серый — не распознан, красный ободок — запрет, "
                            "синий — предписание; L/R/S направление, "
                            "O остановка (2 с), P стоянка (финиш)")

    def _arrow(self, q, a, b, color, width, head=9, dashed=False):
        pen = QPen(color, width)
        if dashed:
            pen.setStyle(Qt.DashLine)
        q.setPen(pen)
        q.setBrush(QBrush(color))
        dx, dy = b.x() - a.x(), b.y() - a.y()
        L = math.hypot(dx, dy)
        if L < 1e-6:
            return
        ux, uy = dx / L, dy / L
        # укорачиваем, чтобы стрелка не влезала в кружок точки
        a2 = QPointF(a.x() + ux * NODE_R, a.y() + uy * NODE_R)
        b2 = QPointF(b.x() - ux * NODE_R, b.y() - uy * NODE_R)
        q.drawLine(a2, b2)
        ang = math.atan2(uy, ux)
        p1 = QPointF(b2.x() - head * math.cos(ang - 0.45), b2.y() - head * math.sin(ang - 0.45))
        p2 = QPointF(b2.x() - head * math.cos(ang + 0.45), b2.y() - head * math.sin(ang + 0.45))
        q.drawPolygon(QPolygonF([b2, p1, p2]))

    # --- мышь ---
    def mousePressEvent(self, ev):
        self.last_mouse = ev.pos()
        nid = self.node_at(ev.pos())
        if ev.button() == Qt.LeftButton:
            if ev.modifiers() & Qt.ShiftModifier:
                if nid:
                    self.edge_from = nid
            elif nid:
                self.drag_node = nid
            else:
                self._add_node(ev.pos())
        elif ev.button() == Qt.MiddleButton:
            if nid:
                self.edge_from = nid
            else:
                self.panning = True
        elif ev.button() == Qt.RightButton:
            if nid:
                self._del_node(nid)
            else:
                key = self.edge_at(ev.pos())
                if key:
                    del self.g.edges[key]
                    self.status.setText(f"удалено ребро {key[0]} -> {key[1]}")
        self.update()

    def mouseMoveEvent(self, ev):
        if self.drag_node:
            x, y = self.s2w(ev.pos())
            n = self.g.nodes[self.drag_node]
            n.x, n.y = x, y
        elif self.panning:
            self.pan += ev.pos() - self.last_mouse
        self.last_mouse = ev.pos()
        self.update()

    def mouseReleaseEvent(self, ev):
        if self.edge_from:
            nid = self.node_at(ev.pos())
            if nid and nid != self.edge_from:
                self._add_edge(self.edge_from, nid)
            self.edge_from = None
        self.drag_node = None
        self.panning = False
        self.update()

    def wheelEvent(self, ev):
        f = 1.15 if ev.angleDelta().y() > 0 else 1 / 1.15
        self.scale *= f
        self.update()

    # --- клавиши ---
    def keyPressEvent(self, ev):
        k = ev.key()
        letter = RU_TO_EN.get(ev.text().lower())
        if letter:
            k = getattr(Qt, 'Key_' + letter)
        for key, d in ((Qt.Key_N, NORTH), (Qt.Key_S, SOUTH),
                       (Qt.Key_E, EAST), (Qt.Key_W, WEST)):
            if k == key and not (ev.modifiers() & Qt.ControlModifier):
                self.new_dir = d
                self.status.setText(f"направление новых точек: {DIR_NAME[d]}")
        if k == Qt.Key_G:
            nid = self.node_at(self.last_mouse)
            if nid:
                if self.goal == nid:
                    self.goal = None
                    self.status.setText("цель снята — режим исследования")
                else:
                    self.goal = nid
                    self.status.setText(f"цель: {nid} (№{self.g.nodes[nid].index})")
        elif k == Qt.Key_Space:
            if self.sim_timer.isActive():
                self.sim_stop()
            else:
                self.sim_start()
        elif k == Qt.Key_Backspace:
            self.sim_reset()
        elif k == Qt.Key_P:
            nid = self.node_at(self.last_mouse)
            if nid:
                self.sim_at = nid
                self.sim_to = None
                self.sim_t = 0.0
                self.passed = {nid}
                self.status.setText(f"робот поставлен в {nid}")
        elif k in (Qt.Key_1, Qt.Key_2, Qt.Key_3):
            nid = self.node_at(self.last_mouse)
            if nid:
                sign = {Qt.Key_1: 'no_left', Qt.Key_2: 'no_right',
                        Qt.Key_3: 'no_straight'}[k]
                ch = self.g.apply_sign(nid, sign)
                self.status.setText(f"{nid}: знак {sign}, отключено рёбер {len(ch)}")
                # тот же знак уходит узлу, чтобы живой робот перестроился
                w = self.window()
                if hasattr(w, 'bridge'):
                    w.bridge.send_sign(nid, sign)
        elif k == Qt.Key_0:
            self.clear_signs()
            self.status.setText("все знаки сняты, рёбра восстановлены")
        elif k == Qt.Key_M:
            self.place_random_signs()
        elif k == Qt.Key_C:
            self.on_route_state('reset')
            self.status.setText("подсветка проезда сброшена")
        elif k == Qt.Key_I:
            self.show_ids = not self.show_ids
            self.status.setText(f"показ id: {'вкл' if self.show_ids else 'выкл'}")
        elif k == Qt.Key_T:
            nid = self.node_at(self.last_mouse)
            if nid:
                n = self.g.nodes[nid]
                n.stop = not n.stop
                self.status.setText(f"{nid}: стоп = {n.stop}")
        elif k == Qt.Key_R:
            self.g.nodes.clear()
            self.g.edges.clear()
            fresh = RouteGraph.build_default()
            self.g.nodes.update(fresh.nodes)
            self.g.edges.update(fresh.edges)
            self.status.setText("граф перегенерирован из регламента")
        elif k == Qt.Key_U:
            self.g.renumber()
            self.status.setText("нумерация пересчитана обходом графа")
        self.update()

    # --- правки графа ---
    # ---------------- знаки ----------------
    def _sign_options(self, nid):
        """Знаки, осмысленные в этой точке.

        Условий два. Из точки должен быть выбор — вешать запрет там, где и
        так один выезд, нечего. И знак не должен гасить все выезды разом,
        иначе робот окажется заперт в точке, а симуляция просто встанет.
        """
        out = [e for (s, _), e in self.g.edges.items() if s == nid and e.enabled]
        if len(out) < 2:
            return []
        kinds = {e.kind for e in out}
        opts = []
        for sign in SIGN_KINDS:
            if sign.startswith('no_'):
                target = sign[3:]
                left = [e for e in out if e.kind != target]
            else:
                target = sign[5:]
                left = [e for e in out if e.kind == target]
            if target in kinds and left:
                opts.append(sign)
        return opts

    def _signs_survivable(self, plan):
        """Проверить расклад на копии графа, не трогая настоящий.

        Знаки применяются к probe, и если после них хоть из одной точки
        некуда выехать или пропал путь до цели, расклад бракуется.
        """
        probe = copy.deepcopy(self.g)
        for nid, sign in plan.items():
            probe.apply_sign(nid, sign)
        out = dict.fromkeys(probe.nodes, 0)
        for (src, _), e in probe.edges.items():
            if e.enabled:
                out[src] += 1
        if any(v == 0 for v in out.values()):
            return False
        if self.goal:
            start = self.sim_at or min(probe.nodes.items(),
                                       key=lambda kv: kv[1].index)[0]
            if not probe.find_path(start, self.goal):
                return False
        return True

    def place_random_signs(self, count=8, stops=2):
        """Расставить знаки полигона: направляющие, остановки и стоянку.

        На граф они сразу не действуют: рёбра отключатся, когда робот
        приедет в точку и знак «распознается». Расклад перебирается, пока
        не найдётся такой, при котором полигон остаётся проезжим.

        Состав взят из регламента: count направляющих знаков, stops зон
        посадки («место остановки») и ровно одна зона высадки («место
        стоянки»). Если цель задана, стоянка садится прямо на неё — по
        заданию высадка и есть конец маршрута.
        """
        self.clear_signs()
        start = self.sim_at or min(self.g.nodes.items(),
                                   key=lambda kv: kv[1].index)[0]
        # Зона высадки выбирается первой и выводится из кандидатов: иначе
        # на неё сядет направляющий знак, и стоянке места уже не останется.
        park = self.goal or random.choice([n for n in self.g.nodes if n != start])
        cands = [nid for nid in self.g.nodes
                 if nid != park and self._sign_options(nid)]
        if not cands:
            self.status.setText("нет точек с развилкой — знаки вешать некуда")
            return
        k = min(count, len(cands))
        # Знаки, раскиданные совсем случайно, чаще всего оказываются в
        # стороне от маршрута и ни на что не влияют. Поэтому половину
        # ставим прямо на текущий план: тогда видно, как робот упирается
        # в запрет и перекладывает путь.
        on_route = []
        if self.goal:
            path = self.g.find_path(start, self.goal) or []
            on_route = [nid for nid in path if nid in cands]
        for _ in range(40):
            picked = []
            if on_route:
                picked = random.sample(on_route, min((k + 1) // 2, len(on_route)))
            rest = [nid for nid in cands if nid not in picked]
            picked += random.sample(rest, min(k - len(picked), len(rest)))
            plan = {nid: random.choice(self._sign_options(nid))
                    for nid in picked}
            if self._signs_survivable(plan):
                self._add_zone_signs(plan, park, start, stops)
                self.placed_signs = plan
                where = ', '.join(
                    f"{self.g.nodes[n].index}:{sign}"
                    for n, sign in sorted(plan.items(),
                                          key=lambda kv: self.g.nodes[kv[0]].index))
                self.status.setText(f"знаков расставлено {len(plan)} — {where}")
                self.update()
                return
        self.status.setText("не удалось расставить знаки, не заперев полигон")

    def _add_zone_signs(self, plan, park, start, stops):
        """Добавить к раскладу зоны посадки и высадки.

        Рёбер они не трогают, поэтому связность от них не страдает и
        ставить их можно в любую точку — лишь бы не поверх другого знака.
        """
        plan[park] = SIGN_PARKING

        # Зоны посадки ставим по пути к высадке, иначе робот до них просто
        # не доедет и выдержку проверить будет негде.
        route = self.g.find_path(start, park) or [] if self.goal else []
        free = [n for n in route[1:] if n not in plan] or \
               [n for n in self.g.nodes if n not in plan and n != start]
        for nid in random.sample(free, min(stops, len(free))):
            plan[nid] = SIGN_STOP

    def clear_signs(self):
        self.parked = False
        self.g.reset_signs()
        self.placed_signs.clear()
        self.detected_signs.clear()
        self.update()

    def _detect_sign(self, nid):
        """Робот приехал в точку и увидел знак, если тот здесь стоит.

        Здесь и происходит то, что на живом роботе сделает связка камеры с
        детектором: знак превращается в вызов apply_sign, тот гасит рёбра,
        а маршрут пересчитывается сам — тем же find_path на следующем шаге.
        """
        sign = self.placed_signs.get(nid)
        if sign is None or nid in self.detected_signs:
            return
        changed = self.g.apply_sign(nid, sign)
        self.detected_signs[nid] = sign
        self.status.setText(f"точка {self.g.nodes[nid].index}: распознан знак "
                            f"{sign}, отключено рёбер {len(changed)}")

    # ---------------- симуляция ----------------
    def sim_start(self):
        """Поехали. С целью — по маршруту, без цели — в режиме исследования.

        Пока цели нет (её увидит камера, которой ещё нет), робот просто
        катается по полигону, предпочитая ещё не пройденные направления.
        Так видно, как работает приоритизация на развилках.
        """
        if self.sim_at is None:
            # стартуем с точки номер 1 — она у въезда на полигон
            first = min(self.g.nodes.items(), key=lambda kv: kv[1].index)[0]
            self.sim_at = first
            self.passed = {first}
        if self.parked:
            self.status.setText("робот в зоне высадки, маршрут выполнен — "
                                "Backspace для нового заезда")
            return
        # Знак в самой стартовой точке тоже надо увидеть до первого хода.
        self._detect_sign(self.sim_at)
        self.sim_timer.start(30)
        self.status.setText(f"симуляция: {self.sim_at} -> {self.goal}")

    def _resume_after_stop(self):
        """Продолжить после выдержки у зоны посадки.

        Проверка нужна: за две секунды пользователь мог нажать паузу или
        сбросить симуляцию, и просто так перезапускать таймер нельзя.
        """
        if self.sim_at and not self.parked:
            self.sim_timer.start(30)

    def sim_stop(self):
        self.sim_timer.stop()
        self.status.setText("симуляция на паузе")

    def sim_reset(self):
        self.sim_timer.stop()
        # Знаки остаются на местах, но снова считаются нераспознанными, и
        # рёбра восстанавливаются: один и тот же расклад можно проиграть
        # несколько раз подряд. Снимаются при этом и пометки «стоп»,
        # расставленные вручную клавишей T.
        self.g.reset_signs()
        self.detected_signs.clear()
        self.parked = False
        self.sim_at = self.sim_to = None
        self.sim_t = 0.0
        self.sim_path = []
        self.passed.clear()
        self.passed_edges.clear()
        self.visits.clear()
        self.current = None
        self.status.setText("симуляция сброшена")
        self.update()

    def _sim_tick(self):
        # едем по текущему отрезку
        if self.sim_to:
            self.sim_t += self.sim_speed
            if self.sim_t < 1.0:
                self.update()
                return
            # приехали в точку — дальше решаем заново, как настоящий узел
            self.passed.add(self.sim_at)
            self.visits[self.sim_to] = self.visits.get(self.sim_to, 0) + 1
            self.sim_at, self.sim_to, self.sim_t = self.sim_to, None, 0.0
            # «Камера» смотрит знаки ровно в момент приезда в точку — так же,
            # как это будет делать route_follower по сервису захвата кадра.
            # Дальше по этой же итерации find_path пересчитает маршрут, уже
            # с учётом погашенных рёбер.
            self._detect_sign(self.sim_at)
            node = self.g.nodes[self.sim_at]
            # Зона высадки — конец задания: дальше робот никуда не едет.
            if node.parking:
                self.sim_timer.stop()
                self.parked = True
                self.passed.add(self.sim_at)
                self.current = None
                self.sim_path = []
                self.status.setText(
                    f"точка {node.index}: знак «место стоянки» — зона высадки, "
                    f"маршрут выполнен")
                self.update()
                return
            # Зона посадки: регламент требует постоять ровно 2 секунды.
            if node.stop:
                self.status.setText(
                    f"точка {node.index}: знак «место остановки», "
                    f"стою {STOP_PAUSE_MS / 1000:.0f} с — посадка")
                self.sim_timer.stop()
                QTimer.singleShot(STOP_PAUSE_MS, self._resume_after_stop)
                self.update()
                return

        if self.sim_at == self.goal:
            self.sim_timer.stop()
            self.passed.add(self.sim_at)
            self.current = None
            self.sim_path = []
            self.status.setText(f"цель {self.goal} достигнута, "
                                f"пройдено точек: {len(self.passed)}")
            self.update()
            return

        if self.goal:
            path = self.g.find_path(self.sim_at, self.goal)
            if not path or len(path) < 2:
                self.sim_timer.stop()
                self.sim_path = []
                self.status.setText(f"маршрут {self.sim_at} -> {self.goal} не найден")
                self.update()
                return
            self.sim_path = path
            self.sim_to = path[1]
            self.status.setText(
                f"{self.g.nodes[self.sim_at].index} -> {self.g.nodes[self.sim_to].index}"
                f"   осталось точек: {len(path) - 1}")
        else:
            nxt, why = self._explore_next(self.sim_at)
            if nxt is None:
                self.sim_timer.stop()
                self.status.setText(f"из {self.sim_at} ехать некуда")
                self.update()
                return
            self.sim_path = [self.sim_at, nxt]
            self.sim_to = nxt
            self.status.setText(
                f"{self.g.nodes[self.sim_at].index} -> {self.g.nodes[nxt].index}   "
                f"{why}   пройдено рёбер {len(self.passed_edges)} из "
                f"{sum(1 for e in self.g.edges.values() if e.enabled)}")

        self.current = self.sim_to
        self.passed_edges.add((self.sim_at, self.sim_to))
        self.update()

    def _explore_next(self, nid):
        """Куда ехать без цели.

        Приоритет такой: сначала рёбра, по которым ещё не ездили, среди них
        прямо, потом направо, потом налево. Если всё объезжено — идём туда,
        где были реже всего. Правило простое и на развилке ведёт себя
        предсказуемо, что и нужно, чтобы смотреть приоритизацию.

        ДОРАБОТАТЬ: когда появится распознавание, здесь же решается, куда
        ехать «пока не увидим цель» — правило можно заменить на любое,
        например держаться правой стороны или обходить по кольцу.
        """
        order = {'straight': 0, 'right': 1, 'left': 2}
        outs = [(b, e) for (a, b), e in self.g.edges.items()
                if a == nid and e.enabled]
        if not outs:
            return None, ''
        fresh = [(b, e) for b, e in outs if (nid, b) not in self.passed_edges]
        if fresh:
            b, e = min(fresh, key=lambda p: order.get(p[1].kind, 3))
            return b, f"новое ребро, {e.kind}"
        b, e = min(outs, key=lambda p: (self.visits.get(p[0], 0),
                                        order.get(p[1].kind, 3)))
        return b, f"повтор, {e.kind}"

    def on_robot_pose(self, x, y, yaw):
        self.robot = (x, y, yaw)
        self.update()

    def on_route_state(self, text: str):
        """Состояние из /route_state: 'A->B', 'goal:<id>', 'done', 'reset'."""
        if text in ('done', 'unreachable', 'failed'):
            if self.current:
                self.passed.add(self.current)
            self.current = None
        elif text == 'reset':
            self.passed.clear()
            self.current = self.goal = None
        elif text.startswith('goal:'):
            self.goal = text.split(':', 1)[1]
            self.passed.clear()
        elif '->' in text:
            a, b = text.split('->')
            self.passed.add(a)
            self.current = b
        self.update()

    def _add_node(self, pos):
        x, y = self.s2w(pos)
        from maze_nav.route_graph import CELL_SIZE, HALF
        col = int((x + HALF) // CELL_SIZE)
        row = int((y + HALF) // CELL_SIZE)
        wp = Waypoint(row=row, col=col, direction=self.new_dir, x=x, y=y,
                      yaw=math.atan2(self.new_dir[0], self.new_dir[1]))
        nid = wp.id
        k = 1
        while nid in self.g.nodes:          # в одной клетке может быть несколько точек
            nid = f"{wp.id}_{k}"
            k += 1
        self.g.nodes[nid] = wp
        self.status.setText(f"добавлена точка {nid} ({x:+.2f}, {y:+.2f})")

    def _del_node(self, nid):
        self.g.nodes.pop(nid, None)
        for key in [k for k in self.g.edges if nid in k]:
            del self.g.edges[key]
        self.status.setText(f"удалена точка {nid}")

    def _add_edge(self, src, dst):
        a, b = self.g.nodes[src], self.g.nodes[dst]
        cost = math.hypot(b.x - a.x, b.y - a.y)
        kind = 'straight'
        if a.direction != b.direction:
            kind = 'right' if b.direction == right_of(a.direction) else 'left'
        self.g.edges[(src, dst)] = RouteEdge(src, dst, cost, kind=kind)
        self.status.setText(f"ребро {src} -> {dst} ({kind})")


class Main(QMainWindow):
    def __init__(self, map_yaml, graph_path):
        super().__init__()
        self.graph_path = graph_path
        img, res, ox, oy = load_map(map_yaml)

        if os.path.exists(graph_path):
            g = RouteGraph.load(graph_path)
            msg = f"загружен {graph_path}"
        else:
            g = RouteGraph.build_default()
            msg = "граф сгенерирован из регламента (файла не было)"

        self.status = QLabel(msg)
        self.canvas = Canvas(g, img, res, ox, oy, self.status)

        # Панель управления. Кнопки дублируют клавиши, но кнопка нагляднее,
        # когда нужно просто «поехали» и смотреть на карту.
        self.btn_start = QPushButton('Старт с точки 1')
        self.btn_start.setMinimumHeight(34)
        self.btn_start.clicked.connect(self.on_start)
        self.btn_sim = QPushButton('Симуляция (пробел)')
        self.btn_sim.setMinimumHeight(34)
        self.btn_sim.clicked.connect(self.on_sim)
        self.btn_reset = QPushButton('Сброс')
        self.btn_reset.setMinimumHeight(34)
        self.btn_reset.clicked.connect(self.canvas.sim_reset)
        self.chk_live = QCheckBox('вести робота по ROS')
        self.chk_live.setChecked(True)
        self.chk_live.setToolTip(
            'снято — кнопка «Старт» только запускает симуляцию, '
            'робот не поедет')

        bar = QHBoxLayout()
        for w in (self.btn_start, self.btn_sim, self.btn_reset, self.chk_live):
            # Нажатая кнопка забирает фокус себе, и дальше все клавиши уходят
            # ей, а не холсту: знаки по M не ставятся, а пробел вместо
            # старта симуляции повторно жмёт саму кнопку.
            w.setFocusPolicy(Qt.NoFocus)
            bar.addWidget(w)
        bar.addStretch(1)

        box = QVBoxLayout()
        box.setContentsMargins(6, 6, 6, 0)
        box.addLayout(bar)
        box.addWidget(self.canvas, 1)
        holder = QWidget()
        holder.setLayout(box)
        self.setCentralWidget(holder)

        self.statusBar().addWidget(self.status)
        self.setWindowTitle(f"Редактор маршрутов — {os.path.basename(graph_path)}")
        self.resize(1150, 980)

        QShortcut(QKeySequence('Ctrl+S'), self, self.save)

        # Подписка на состояние и позу. Если ROS недоступен, поток тихо
        # завершится и редактор останется offline-инструментом.
        self.bridge = RosBridge()
        self.bridge.got_state.connect(self.canvas.on_route_state)
        self.bridge.got_pose.connect(self.canvas.on_robot_pose)
        self.bridge.ready.connect(self.on_ros_ready)
        self.bridge.start()

    def keyPressEvent(self, ev):
        """Страховка: клавиши работают, даже если фокус ушёл с холста."""
        self.canvas.keyPressEvent(ev)

    def on_ros_ready(self, ok):
        self.chk_live.setEnabled(ok)
        if not ok:
            self.chk_live.setChecked(False)
            self.status.setText("ROS недоступен — только редактирование и симуляция")

    def on_start(self):
        """Поехали с точки 1. Цель, если назначена, иначе исследование."""
        goal = self.canvas.goal or ''
        self.canvas.sim_reset()
        first = min(self.canvas.g.nodes.items(), key=lambda kv: kv[1].index)[0]
        self.canvas.sim_at = first
        self.canvas.passed = {first}

        if self.chk_live.isChecked() and self.bridge.send_start(goal):
            self.status.setText(
                f"старт по ROS с точки 1"
                + (f", цель {goal}" if goal else " в режиме исследования"))
        else:
            self.canvas.sim_start()

    def on_sim(self):
        if self.canvas.sim_timer.isActive():
            self.canvas.sim_stop()
        else:
            self.canvas.sim_start()

    def save(self):
        self.canvas.g.save(self.graph_path)
        self.status.setText(f"сохранено: {self.graph_path} "
                            f"({len(self.canvas.g.nodes)} точек, "
                            f"{len(self.canvas.g.edges)} рёбер)")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--map', required=True, help='yaml карты (например city.yaml)')
    p.add_argument('--graph', required=True, help='yaml графа маршрутов; создастся, если нет')
    # parse_known_args, а не parse_args: launch_ros подмешивает свои
    # --ros-args, и строгий разбор на них падает ещё до создания окна.
    a, _ = p.parse_known_args()

    app = QApplication(sys.argv)
    w = Main(a.map, a.graph)
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
