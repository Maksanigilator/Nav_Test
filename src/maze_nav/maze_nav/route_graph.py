#!/usr/bin/env python3
"""Ориентированный граф маршрутов по полосам полигона «Город РТК».

Отличие от road_graph.py из регламента: там узлы стоят в ЦЕНТРАХ клеток,
и рёбра не различают полосы. Здесь узел — это точка на конкретной полосе,
а ребро существует только если переход между ними не нарушает правил.
Проехав по такому графу, робот автоматически едет по своей стороне.

Геометрия берётся из регламента (клетка 0.8 м, полоса смещена на 0.20 м
от оси дороги), но модуль самодостаточен — ни IsaacLab, ни файлов
регламента для работы не нужно.

    from maze_nav.route_graph import RouteGraph
    g = RouteGraph.build_default()
    path = g.find_path('n_4_1_E', 'n_0_3_W')

Граф сохраняется в YAML и правится в редакторе (route_editor.py),
поэтому сгенерированный вариант — только отправная точка.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

# --- геометрия полигона, совпадает с city_builder.py регламента ---
CELL_SIZE = 0.8
GRID_N = 5
HALF = CELL_SIZE * GRID_N / 2.0          # 2.0 м
LANE_OFFSET = 0.20                        # смещение центра полосы от оси дороги
BUILDING_CELLS = {(1, 1), (1, 3), (3, 1), (3, 3)}
# Классификация из регламента (lane_overlay.py): по типу клетки решается,
# сколько в ней точек и как они расставлены.
CORNER_CELLS = {(0, 0), (0, 4), (4, 0), (4, 4)}
INTERSECTION_CELLS = {(0, 2), (2, 0), (2, 2), (2, 4), (4, 2)}
CENTER_CELL = (2, 2)          # единственный полноценный перекрёсток
# Въезд на полигон — проём в нижней стене у правого угла (см. gen_city_map.py,
# параметр --gate south:4). Отсюда начинается нумерация: робот заезжает
# здесь, и номера растут по ходу его движения.
ENTRY_XY = (1.6, -2.0)
# Боковые перекрёстки — T-образные: сквозная дорога вдоль края плюс
# ответвление внутрь. Точки на них ставятся как на прямых, по одной на
# полосу, но выезжать с них можно и в ответвление.


def cell_kind(row: int, col: int) -> str:
    if (row, col) in BUILDING_CELLS:
        return 'building'
    if (row, col) in CORNER_CELLS:
        return 'corner'
    if (row, col) in INTERSECTION_CELLS:
        return 'intersection'
    return 'straight'


def straight_axis(row: int, col: int) -> tuple[tuple[int, int], tuple[int, int]]:
    """Пара направлений вдоль дороги для прямой клетки."""
    if row in (0, 4) or (row == 2 and col in (1, 3)):
        return (EAST, WEST)          # горизонтальный участок
    return (NORTH, SOUTH)            # вертикальный

# Направления как (drow, dcol); row растёт на север, col — на восток
NORTH, SOUTH, EAST, WEST = (1, 0), (-1, 0), (0, 1), (0, -1)
DIR_NAME = {NORTH: 'N', SOUTH: 'S', EAST: 'E', WEST: 'W'}
NAME_DIR = {v: k for k, v in DIR_NAME.items()}
OPPOSITE = {NORTH: SOUTH, SOUTH: NORTH, EAST: WEST, WEST: EAST}


def cell_center(row: int, col: int) -> tuple[float, float]:
    return (-HALF + CELL_SIZE * col + CELL_SIZE / 2.0,
            -HALF + CELL_SIZE * row + CELL_SIZE / 2.0)


def right_of(d: tuple[int, int]) -> tuple[int, int]:
    """Поворот направо: (dr, dc) -> (-dc, dr)."""
    return (-d[1], d[0])


def left_of(d: tuple[int, int]) -> tuple[int, int]:
    return (d[1], -d[0])


@dataclass
class Waypoint:
    """Точка на полосе: центр клетки плюс смещение вправо по ходу движения."""
    row: int
    col: int
    # Направление движения в этой точке. У перекрёстка его нет: туда въезжают
    # и выезжают в разные стороны, поэтому None, а курс для Nav2 считается
    # по следующей точке маршрута.
    direction: tuple[int, int] | None
    x: float
    y: float
    yaw: float
    # Знак Б.6 «Место остановки»: зона посадки, регламент требует постоять
    # там 2 секунды. Поле называется stop по историческим причинам.
    stop: bool = False
    # Знак Б.7 «Место стоянки»: зона высадки и конец маршрута. Робот должен
    # остановиться перед ней, дальше ехать некуда — задание выполнено.
    parking: bool = False
    kind: str = 'straight'         # straight | corner_in | corner_out | intersection
    # Направление въезда. Нужно углам: точка поворота принимает робота только
    # с одной стороны, иначе граф разрешил бы въехать в неё против движения.
    in_dir: tuple[int, int] | None = None
    # Порядковый номер: не порядок создания, а обход графа по направлению
    # движения от внешней полосы. Так соседние по маршруту точки получают
    # соседние номера, и по карте удобно диктовать задание.
    index: int = 0

    @property
    def id(self) -> str:
        tag = DIR_NAME[self.direction] if self.direction else 'X'
        return f"n_{self.row}_{self.col}_{tag}"


@dataclass
class RouteEdge:
    src: str
    dst: str
    cost: float = CELL_SIZE
    enabled: bool = True
    kind: str = 'straight'         # straight | left | right — для реакции на знаки


@dataclass
class RouteGraph:
    nodes: dict[str, Waypoint] = field(default_factory=dict)
    edges: dict[tuple[str, str], RouteEdge] = field(default_factory=dict)

    # ---------------- построение ----------------
    @classmethod
    def build_default(cls) -> RouteGraph:
        """Точки расставляются по типу клетки, а не по четыре на каждую.

        прямая клетка   две точки: по одной на каждую полосу движения
        угол            две точки: внутренний радиус (поворот направо) и
                        внешний (поворот налево). Точка лежит на пересечении
                        полосы въезда и полосы выезда, поэтому смещения
                        складываются по двум осям.
        перекрёсток     одна точка в центре, без фиксированного направления
        """
        g = cls()
        for row in range(GRID_N):
            for col in range(GRID_N):
                kind = cell_kind(row, col)
                if kind == 'building':
                    continue
                cx, cy = cell_center(row, col)

                if kind == 'straight':
                    for d in straight_axis(row, col):
                        rd = right_of(d)
                        g._add(Waypoint(row, col, d,
                                        cx + rd[1] * LANE_OFFSET,
                                        cy + rd[0] * LANE_OFFSET,
                                        math.atan2(d[0], d[1]), kind='straight'))

                elif kind == 'corner':
                    # Направления «внутрь поля» от этого угла
                    v = NORTH if row == 0 else SOUTH
                    h = EAST if col == 0 else WEST
                    # Два прохода: въезд по одной оси, выезд по другой
                    for d_in, d_out in ((OPPOSITE[h], v), (OPPOSITE[v], h)):
                        r_in, r_out = right_of(d_in), right_of(d_out)
                        x = cx + (r_in[1] + r_out[1]) * LANE_OFFSET
                        y = cy + (r_in[0] + r_out[0]) * LANE_OFFSET
                        # внутренний радиус — тот, что ближе к центру поля
                        inner = math.hypot(x, y) < math.hypot(cx, cy)
                        g._add(Waypoint(row, col, d_out, x, y,
                                        math.atan2(d_out[0], d_out[1]),
                                        kind='corner_in' if inner else 'corner_out',
                                        in_dir=d_in))

                elif (row, col) == CENTER_CELL:
                    g._add(Waypoint(row, col, None, cx, cy, 0.0, kind='intersection'))

                else:  # боковой T-образный перекрёсток
                    for d in straight_axis(row, col):
                        rd = right_of(d)
                        g._add(Waypoint(row, col, d,
                                        cx + rd[1] * LANE_OFFSET,
                                        cy + rd[0] * LANE_OFFSET,
                                        math.atan2(d[0], d[1]), kind='t_junction'))

        g._link()
        g._link_bypass()
        g.renumber()
        return g

    def renumber(self, start: str | None = None) -> None:
        """Пронумеровать точки обходом графа от въезда на полигон.

        Первой становится ближайшая к воротам точка внешней полосы — робот
        заезжает оттуда, и номера растут по ходу движения. Дальше идём по
        направленным рёбрам, предпочитая ехать прямо: обход накручивает
        внешний контур, а потом уходит внутрь. Точки, до которых не
        добрались, нумеруются следом.
        """
        if not self.nodes:
            return
        if start is None:
            ex, ey = ENTRY_XY
            # только внешние полосы: у ворот робот оказывается именно на них
            outer = [i for i, n in self.nodes.items()
                     if max(abs(n.x), abs(n.y)) > 1.6]
            pool = outer or list(self.nodes)
            start = min(pool, key=lambda i: math.hypot(self.nodes[i].x - ex,
                                                       self.nodes[i].y - ey))
        order, seen = [], set()
        stack = [start]
        while stack:
            nid = stack.pop()
            if nid in seen:
                continue
            seen.add(nid)
            order.append(nid)
            # прямые рёбра первыми, поэтому в стек кладём их последними
            nxt = sorted(
                (e for (a, _), e in self.edges.items() if a == nid and e.enabled),
                key=lambda e: (e.kind == 'straight', -e.cost))
            for e in nxt:
                if e.dst not in seen:
                    stack.append(e.dst)

        for nid in self.nodes:
            if nid not in seen:
                order.append(nid)
        for i, nid in enumerate(order, start=1):
            self.nodes[nid].index = i

    def _add(self, wp: Waypoint) -> None:
        nid = wp.id
        k = 1
        while nid in self.nodes:
            nid = f"{wp.id}_{k}"
            k += 1
        self.nodes[nid] = wp

    def _link_bypass(self) -> None:
        """Повороты через боковой перекрёсток — напрямую, минуя его точку.

        Раньше поворот шёл «точка до -> точка перекрёстка -> точка после»,
        и робот заезжал в центр клетки, а потом резко доворачивал. Теперь
        ребро соединяет точку в клетке ДО перекрёстка с точкой в клетке
        ПОСЛЕ: получается одна плавная дуга в обход центра.

        ДОРАБОТАТЬ: у центрального перекрёстка (2,2) точка одна и повороты
        по-прежнему идут через неё. Если робот будет там срезать — добавить
        такие же обходы и для него.
        """
        for cell in INTERSECTION_CELLS:
            if cell == CENTER_CELL:
                continue
            r, c = cell
            # точки в клетках вокруг перекрёстка
            around = {}
            for d in (NORTH, SOUTH, EAST, WEST):
                nr, nc = r + d[0], c + d[1]
                around[d] = [(nid, n) for nid, n in self.nodes.items()
                             if (n.row, n.col) == (nr, nc)]

            for d_in, ins in around.items():
                for d_out, outs in around.items():
                    if d_out == d_in or d_out == OPPOSITE[d_in]:
                        continue                      # это проезд прямо
                    for src_id, src in ins:
                        # въезжаем в перекрёсток, значит движемся против d_in
                        if src.direction != OPPOSITE[d_in]:
                            continue
                        for dst_id, dst in outs:
                            if dst.direction != d_out:
                                continue
                            if dst.direction != right_of(src.direction):
                                # Налево — только через точку перекрёстка:
                                # такой поворот пересекает встречную полосу,
                                # срезать его нельзя. Мимо идут лишь правые,
                                # они короткие и жмутся к своему краю.
                                continue
                            kind = 'right'
                            cost = math.hypot(dst.x - src.x, dst.y - src.y) * 1.5
                            self.edges[(src_id, dst_id)] = RouteEdge(
                                src_id, dst_id, cost, kind=kind)

    def _link(self) -> None:
        """Рёбра между точками соседних клеток.

        Переход разрешён, если робот выезжает из клетки в направлении
        соседней и попадает там в точку, совместимую с движением: либо
        с тем же направлением, либо на перекрёсток, либо в угол, чей
        выезд продолжает путь.
        """
        by_cell: dict[tuple[int, int], list[str]] = {}
        for nid, n in self.nodes.items():
            by_cell.setdefault((n.row, n.col), []).append(nid)

        for nid, n in self.nodes.items():
            # Из какой клетки и куда уезжаем. С полосы прямого участка —
            # только вперёд; с перекрёстка — во все стороны, кроме обратной
            # (разворот на месте правилами не предусмотрен).
            if n.direction is None:
                dirs = [NORTH, SOUTH, EAST, WEST]
            elif n.kind == 't_junction':
                # Прямо и налево. Правые повороты идут в обход этой точки
                # (см. _link_bypass): они короткие, и заезжать ради них в
                # центр клетки незачем.
                dirs = [n.direction, left_of(n.direction)]
            else:
                dirs = [n.direction]
            for d in dirs:
                nr, nc = n.row + d[0], n.col + d[1]
                if not (0 <= nr < GRID_N and 0 <= nc < GRID_N):
                    continue
                for did in by_cell.get((nr, nc), []):
                    m = self.nodes[did]
                    # въезд допустим, если точка ждёт того же направления,
                    # либо это перекрёсток, либо угол с таким въездом
                    if m.kind == 'straight' and m.direction != d:
                        continue
                    if m.kind == 't_junction' and m.direction not in (d, left_of(d)):
                        # Въезд прямо по своей оси либо под левый поворот.
                        # Правые манёвры сюда не заходят — для них есть
                        # обходные рёбра, иначе на один поворот получились
                        # бы две разные траектории.
                        continue
                    if m.kind.startswith('corner') and m.in_dir is not None:
                        # в угол пускаем только с той стороны, на которую он рассчитан
                        if m.in_dir != d:
                            continue
                    kind = 'straight'
                    if n.direction and m.direction and n.direction != m.direction:
                        kind = 'right' if m.direction == right_of(n.direction) else 'left'
                    cost = math.hypot(m.x - n.x, m.y - n.y)
                    if kind != 'straight':
                        cost *= 1.5
                    self.edges[(nid, did)] = RouteEdge(nid, did, cost, kind=kind)

    # ---------------- знаки ----------------
    def apply_sign(self, node_id: str, sign: str) -> list[str]:
        """Учесть знак, замеченный в точке node_id.

        sign: no_left | no_right | no_straight | only_left | only_right |
              only_straight | stop | parking

        Регламент «Города РТК» использует семь знаков: предписывающие
        «движение прямо / налево / направо», запрещающие «поворот налево
        запрещён» и «поворот направо запрещён», плюс «место остановки» и
        «место стоянки». Запрета движения прямо среди них нет, но граф его
        понимает — пригодится для собственных тестов.
        Возвращает список изменённых рёбер — удобно логировать, что именно
        поменялось после распознавания.

        ДОРАБОТАТЬ: знаки сейчас применяются навсегда. Для полигона, где
        робот проезжает перекрёсток несколько раз, может понадобиться
        сбрасывать их при выходе из клетки либо хранить срок действия.
        """
        changed = []
        if node_id not in self.nodes:
            return changed
        if sign == 'stop':
            self.nodes[node_id].stop = True
            return [node_id]
        if sign == 'parking':
            self.nodes[node_id].parking = True
            return [node_id]

        forbid = {'no_left': 'left', 'no_right': 'right', 'no_straight': 'straight'}
        allow = {'only_left': 'left', 'only_right': 'right', 'only_straight': 'straight'}

        for (s, d), e in self.edges.items():
            if s != node_id:
                continue
            if sign in forbid and e.kind == forbid[sign] and e.enabled:
                e.enabled = False
                changed.append(f"{s}->{d}")
            elif sign in allow and e.kind != allow[sign] and e.enabled:
                e.enabled = False
                changed.append(f"{s}->{d}")
        return changed

    def reset_signs(self) -> None:
        for e in self.edges.values():
            e.enabled = True
        for n in self.nodes.values():
            n.stop = False
            n.parking = False

    # ---------------- поиск пути ----------------
    def find_path(self, start: str, goal: str) -> list[str] | None:
        """A* по точкам полос. Возвращает список id или None.

        Разворот запрещён: нельзя вернуться в клетку, из которой только что
        приехали. Без этого маршрут вида «въехал на перекрёсток и ушёл
        обратно по встречной полосе» формально допустим — обе точки лежат на
        своих полосах, — но правилами это разворот.
        """
        if start not in self.nodes or goal not in self.nodes:
            return None
        gx, gy = self.nodes[goal].x, self.nodes[goal].y

        def h(nid):
            n = self.nodes[nid]
            return abs(n.x - gx) + abs(n.y - gy)

        def cell(nid):
            n = self.nodes[nid]
            return (n.row, n.col)

        # Состояние поиска — пара (куда пришли, откуда пришли): та же точка,
        # достигнутая с разных сторон, даёт разные допустимые продолжения.
        start_state = (start, None)
        open_set = [(h(start), start_state)]
        came: dict = {}
        gs = {start_state: 0.0}

        while open_set:
            _, st = heapq.heappop(open_set)
            cur, prev_cell = st
            if cur == goal:
                path, s2 = [cur], st
                while s2 in came:
                    s2 = came[s2]
                    path.append(s2[0])
                return path[::-1]

            for (a, b), e in self.edges.items():
                if a != cur or not e.enabled:
                    continue
                if prev_cell is not None and cell(b) == prev_cell:
                    continue                      # это и есть разворот
                nxt = (b, cell(cur))
                t = gs[st] + e.cost
                if t < gs.get(nxt, float('inf')):
                    came[nxt] = st
                    gs[nxt] = t
                    heapq.heappush(open_set, (t + h(b), nxt))
        return None

    def nearest_node(self, x: float, y: float, direction=None) -> str | None:
        """Ближайшая точка к координате. Если задано направление — только с ним."""
        best, bd = None, float('inf')
        for nid, n in self.nodes.items():
            if direction is not None and n.direction != direction:
                continue
            d = math.hypot(n.x - x, n.y - y)
            if d < bd:
                best, bd = nid, d
        return best

    # ---------------- сохранение ----------------
    def to_dict(self) -> dict:
        return {
            'nodes': [
                {'id': n.id, 'row': n.row, 'col': n.col,
                 'dir': DIR_NAME[n.direction] if n.direction else 'X',
                 'x': round(n.x, 4), 'y': round(n.y, 4),
                 'yaw': round(n.yaw, 4), 'stop': n.stop,
                 'parking': n.parking, 'kind': n.kind,
                 'in_dir': DIR_NAME[n.in_dir] if n.in_dir else None,
                 'index': n.index}
                for n in self.nodes.values()
            ],
            'edges': [
                {'src': e.src, 'dst': e.dst, 'cost': round(e.cost, 3),
                 'kind': e.kind, 'enabled': e.enabled}
                for e in self.edges.values()
            ],
        }

    @classmethod
    def from_dict(cls, data: dict) -> RouteGraph:
        g = cls()
        for n in data.get('nodes', []):
            d = NAME_DIR.get(n['dir'])          # 'X' -> None, перекрёсток
            g.nodes[n['id']] = Waypoint(
                row=n['row'], col=n['col'], direction=d,
                x=n['x'], y=n['y'], yaw=n['yaw'], stop=n.get('stop', False),
                parking=n.get('parking', False),
                kind=n.get('kind', 'straight'),
                in_dir=NAME_DIR.get(n['in_dir']) if n.get('in_dir') else None,
                index=n.get('index', 0))
        for e in data.get('edges', []):
            g.edges[(e['src'], e['dst'])] = RouteEdge(
                e['src'], e['dst'], e.get('cost', CELL_SIZE),
                e.get('enabled', True), e.get('kind', 'straight'))
        return g

    def save(self, path: str) -> None:
        import yaml
        with open(path, 'w') as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True, sort_keys=False)

    @classmethod
    def load(cls, path: str) -> RouteGraph:
        import yaml
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))


if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-o', '--out', help='сохранить граф в YAML')
    a = p.parse_args()

    g = RouteGraph.build_default()
    print(f"точек: {len(g.nodes)}, рёбер: {len(g.edges)}")
    ids = list(g.nodes)
    print("по типам: " + ", ".join(
        f"{k}={sum(1 for n in g.nodes.values() if n.kind == k)}"
        for k in ('straight', 'corner_in', 'corner_out', 't_junction', 'intersection')))
    src, dst = 'n_0_1_E', 'n_4_1_E'
    path = g.find_path(src, dst)
    print(f"пример маршрута {src} -> {dst}: {len(path) if path else 0} точек")
    if path:
        print("  " + " -> ".join(f"{i}({g.nodes[i].index})" for i in path))
        print("  в скобках — номера точек на карте")
    else:
        print("  СВЯЗНОСТЬ НАРУШЕНА")
    if a.out:
        g.save(a.out)
        print(f"сохранено в {a.out}")
