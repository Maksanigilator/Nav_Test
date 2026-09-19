#!/usr/bin/env python3
"""Сцена заезда на рельсы: фанерная платформа и трубы-рельсы.

    python3 src/maze_nav/scripts/gen_rails_world.py -o src/maze_nav/worlds/rails.world

Воспроизводится стенд, снятый на видео для датасета YOLO (проект Roshack):
две трубы, соединённые на ближнем конце П-образной перемычкой, лежат на
листе фанеры, а дальше идут на опорах.

Все размеры ИЗМЕРЕНЫ по кадру с глубиной из того стенда, а не назначены.
Метод: по облаку точек подогнана плоскость фанеры, кадр перепроецирован в
ортофото с масштабом 1 пиксель = 1 мм, размеченный контур рельса из
датасета спроецирован на ту же плоскость. Проверок масштаба две, обе
сошлись сами: колея получилась 545 мм при роликах робота на 543, а толщина
перемычки на контуре — 52 мм при трубе 51.

Форма ближнего конца — именно П, а не полукруг: скруглённый угол, прямой
участок, скруглённый угол. Радиус считается однозначно из того, что у П
прямой участок внешней грани ровно такой же длины, как у осевой линии.

    колея                     545 мм
    прямая перемычка          255 мм
    радиус угла по оси        145 мм   (2.8 диаметра трубы)
    глубина П вдоль рельсов   170 мм   (радиус + полтрубы)
    вылет на фанеру          ≈210 мм   от кромки до внешней грани П

Отсюда следует, что прямые рельсы начинаются практически у самой кромки
фанеры: П занимает почти весь свой вылет.

Геометрия заезда держится на совпадении, которое стоит понимать, прежде
чем что-то здесь менять:

    меканум-колесо       R = 101.4 мм
    рельсовый ролик      R =  50.0 мм
    разница                   51.4 мм,  а труба ровно 51 мм

Разница радиусов равна диаметру трубы. Пока труба лежит НА той же
поверхности, по которой катятся меканум-колёса, а не утоплена в неё, робот
при передаче опоры опускается на 0.4 мм — и высота платформы на это не
влияет. Утопите трубу заподлицо, и заезд станет невозможен.
"""
import argparse
import math

# ═══ Труба ═══
PIPE_OD = 0.051
PIPE_WALL = 0.002
STEEL = 7850.0

# ═══ Замеры стенда ═══
GAUGE = 0.545             # по осям труб
BEND_R = 0.145            # радиус угла П по оси трубы
DECK_OVERRUN = 0.210      # от кромки фанеры до внешней грани П

# ═══ Платформа ═══
# Настил набран из листов фанеры метр на метр — так он и сделан в жизни.
# Двух листов вдоль хватает, чтобы робот (колёсная база 636 мм, корпус
# 1.2 м) стоял на настиле целиком и имел место разогнаться до рельсов.
SHEET = 1.0
SHEET_NX, SHEET_NY = 2, 2
DECK_H = 0.120
# Листы кладём ВСТЫК. Зазор между ними — это не косметика, а сквозная
# щель до пола: идеальная глубина симулятора видит её насквозь и детектор
# края принимает за обрыв. Настоящий D435 такую щель не разрешит никогда,
# так что зазор делал сцену труднее реальности, а не честнее.
# Листы различаются поворотом текстуры, этого достаточно, чтобы видеть швы.
DECK_GAP = 0.0

# ═══ Рельсы за платформой ═══
RAIL_END_X = 8.0
SUPPORT_SPACING = 1.25
SUPPORT_PLATE = (0.72, 0.06, 0.006)
SUPPORT_POST = 0.030

# ═══ Комната вокруг стенда ═══
# Нужна не для вида, а чтобы визуальной одометрии было за что держаться.
# Камера наклонена на 30° вниз, низ кадра всегда занимает пол — на голой
# плоскости признаков нет вообще. Плитки пола кладём текстурой и КРУТИМ
# каждую случайно: текстура бесшовная, но повторяется с шагом плитки, а
# периодический рисунок для одометрии хуже однотонного — она находит
# ложные соответствия и уезжает, вместо того чтобы честно потеряться.
TILE = 2.0
WALL_H = 2.5
WALL_T = 0.10
# Сколько пола оставить ПОЗАДИ настила. Нужно не для вида: роботу надо
# куда-то отъехать, чтобы подойти к рельсам заново после неудачи, и надо
# видеть стену позади — иначе при осмотре половина кадров смотрит в пустоту.
ROOM_BACK = 2.0

# ═══ Робот ═══ (ведомые значения, менять в gen_robot_urdf.py)
WHEEL_R = 0.1014
ROLLER_R = 0.050

ARC_SEGMENTS = 14         # звеньев на четверть окружности


def emit_room(A, a, x_edge, deck_x0):
    """Комната: текстурированный пол плитками, стены и предметы."""
    import random
    rx, ry = a.room
    rnd = random.Random(7)          # фиксируем, чтобы сцена не менялась между прогонами
    # Комнату располагаем от ЗАДНЕЙ кромки настила: позади него остаётся
    # ROOM_BACK метров, остальное уходит вперёд, вдоль рельсов. Так размер
    # комнаты по X задаёт, сколько места за дальним концом рельсов.
    cx = deck_x0 - ROOM_BACK + rx / 2

    A('    <!-- Пол плитками. Текстура бесшовная, но повторяется с шагом')
    A('         плитки, поэтому каждую поворачиваем случайно на 0/90/180/270°')
    A('         — иначе получается регулярный узор, а он для визуальной')
    A('         одометрии хуже, чем однотонный пол. -->')
    A('    <model name="floor">')
    A('      <static>true</static>')
    A('      <link name="link">')
    nx, ny = int(rx / TILE) + 1, int(ry / TILE) + 1
    for i in range(nx):
        for j in range(ny):
            tx = cx - rx / 2 + (i + 0.5) * TILE
            ty = -ry / 2 + (j + 0.5) * TILE
            yaw = rnd.choice((0.0, math.pi / 2, math.pi, 3 * math.pi / 2))
            A(f'        <visual name="t_{i}_{j}">')
            A(f'          <pose>{tx:.3f} {ty:.3f} -0.005 0 0 {yaw:.4f}</pose>')
            A(f'          <geometry><box><size>{TILE} {TILE} 0.01</size></box></geometry>')
            A('          <material><diffuse>1 1 1 1</diffuse>')
            A('            <specular>0.1 0.1 0.1 1</specular>')
            A('            <pbr><metal><albedo_map>'
              'model://maze_nav/meshes/floor.png</albedo_map>'
              '<roughness>0.9</roughness><metalness>0</metalness>'
              '</metal></pbr></material>')
            A('        </visual>')
    A('      </link>')
    A('    </model>')
    A('')

    A('    <!-- Стены. Текстура на них мельче и бледнее пола: яркая стена')
    A('         даёт слишком жирные признаки, и одометрия в симуляторе')
    A('         выглядела бы лучше, чем окажется на железе. -->')
    A('    <model name="walls">')
    A('      <static>true</static>')
    A('      <link name="link">')
    walls = (
        (cx, +ry / 2, rx + WALL_T, WALL_T),
        (cx, -ry / 2, rx + WALL_T, WALL_T),
        (cx - rx / 2, 0.0, WALL_T, ry),
        (cx + rx / 2, 0.0, WALL_T, ry),
    )
    for n, (wx, wy, sx, sy) in enumerate(walls):
        for tag in ('collision', 'visual'):
            A(f'        <{tag} name="w{n}_{tag}">')
            A(f'          <pose>{wx:.3f} {wy:.3f} {WALL_H / 2:.3f} 0 0 0</pose>')
            A(f'          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H}'
              '</size></box></geometry>')
            if tag == 'visual':
                A('          <material><diffuse>1 1 1 1</diffuse>')
                A('            <pbr><metal><albedo_map>'
                  'model://maze_nav/meshes/wall.png</albedo_map>'
                  '<roughness>0.95</roughness><metalness>0</metalness>'
                  '</metal></pbr></material>')
            A(f'        </{tag}>')
    A('      </link>')
    A('    </model>')
    A('')

    A('    <!-- Предметы. Стоят по краям, чтобы не мешать заезду, но')
    A('         попадали в кадр при осмотре: вертикальные грани дают')
    A('         признаки, которых нет ни на полу, ни на гладкой стене. -->')
    objs = [
        ('ящик_1',   cx - rx / 2 + 0.9, +ry / 2 - 0.8, 0.8, 1.2, 0.9, (0.55, 0.40, 0.25)),
        ('ящик_2',   cx - rx / 2 + 0.9, -ry / 2 + 0.9, 1.0, 0.8, 0.6, (0.30, 0.42, 0.55)),
        ('ящик_3',   cx + rx / 2 - 1.0, +ry / 2 - 1.2, 0.7, 0.7, 1.4, (0.45, 0.45, 0.48)),
        ('стеллаж',  cx + rx / 2 - 0.6, -ry / 2 + 1.6, 0.5, 2.4, 2.0, (0.35, 0.35, 0.38)),
        ('поддон',   cx - 1.2, -ry / 2 + 1.0, 1.2, 0.8, 0.15, (0.60, 0.48, 0.30)),
    ]
    for n, (nm, ox, oy, sx, sy, sz, col) in enumerate(objs):
        A(f'    <model name="obj_{n}">')
        A('      <static>true</static>')
        A('      <link name="link">')
        for tag in ('collision', 'visual'):
            A(f'        <{tag} name="{tag}">')
            A(f'          <pose>{ox:.3f} {oy:.3f} {sz / 2:.3f} 0 0 0</pose>')
            A(f'          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>')
            if tag == 'visual':
                A(f'          <material><ambient>{col[0]:.2f} {col[1]:.2f}'
                  f' {col[2]:.2f} 1</ambient>')
                A(f'            <diffuse>{col[0]*1.4:.2f} {col[1]*1.4:.2f}'
                  f' {col[2]*1.4:.2f} 1</diffuse></material>')
            A(f'        </{tag}>')
        A('      </link>')
        A('    </model>')
    for n, (ox, oy, rr, hh) in enumerate(((cx + 1.8, +ry / 2 - 0.9, 0.28, 0.85),
                                          (cx + 2.6, +ry / 2 - 0.9, 0.28, 0.85))):
        A(f'    <model name="drum_{n}">')
        A('      <static>true</static>')
        A('      <link name="link">')
        for tag in ('collision', 'visual'):
            A(f'        <{tag} name="{tag}">')
            A(f'          <pose>{ox:.3f} {oy:.3f} {hh / 2:.3f} 0 0 0</pose>')
            A(f'          <geometry><cylinder><radius>{rr}</radius>'
              f'<length>{hh}</length></cylinder></geometry>')
            if tag == 'visual':
                A('          <material><ambient>0.20 0.30 0.22 1</ambient>')
                A('            <diffuse>0.32 0.48 0.35 1</diffuse></material>')
            A(f'        </{tag}>')
        A('      </link>')
        A('    </model>')
    A('')


def pipe_mass(length):
    r_o, r_i = PIPE_OD / 2, PIPE_OD / 2 - PIPE_WALL
    return STEEL * math.pi * (r_o ** 2 - r_i ** 2) * length


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--gauge', type=float, default=GAUGE)
    ap.add_argument('--bend-radius', type=float, default=BEND_R)
    ap.add_argument('--sheets-x', type=int, default=SHEET_NX,
                    help='сколько листов фанеры вдоль рельсов')
    ap.add_argument('--sheets-y', type=int, default=SHEET_NY,
                    help='сколько листов поперёк, симметрично относительно рельсов')
    ap.add_argument('--sheets-left', type=int, default=0,
                    help='сколько листов добавить СЛЕВА (в сторону +Y). '
                         'Настил от этого перестаёт быть симметричным, '
                         'рельсы остаются на месте')
    ap.add_argument('--sheet', type=float, default=SHEET,
                    help='сторона одного листа, м')
    ap.add_argument('--deck-height', type=float, default=DECK_H)
    ap.add_argument('--pipe-od', type=float, default=PIPE_OD)
    ap.add_argument('--room', nargs=2, type=float, metavar=('X', 'Y'),
                    default=None,
                    help='размеры комнаты вокруг стенда, м; без этого ключа '
                         'получается голая площадка')
    a = ap.parse_args()

    L, A = [], lambda s: L.append(s)
    r = a.pipe_od / 2
    pz = a.deck_height + r                    # ось трубы над полом
    y_rail = a.gauge / 2
    # Кромка фанеры, с которой рельсы уходят на опоры, — начало координат по X.
    x_edge = 0.0
    x_face = x_edge - DECK_OVERRUN            # внешняя грань П
    x_cross = x_face + r                      # ось перемычки
    x_tan = x_cross + a.bend_radius           # где кончается угол
    y_cross = y_rail - a.bend_radius          # докуда идёт прямая перемычка
    straight = 2 * y_cross
    deck_x = a.sheets_x * a.sheet
    # Листы слева расширяют настил только в +Y, рельсы остаются в y=0.
    deck_y = (a.sheets_y + a.sheets_left) * a.sheet
    deck_y0 = -a.sheets_y * a.sheet / 2                  # правая кромка
    deck_y1 = deck_y0 + deck_y                           # левая кромка
    deck_x0, deck_x1 = x_edge - deck_x, x_edge

    A('<?xml version="1.0" ?>')
    A('<!-- СГЕНЕРИРОВАНО scripts/gen_rails_world.py, руками не править. -->')
    A('<!-- Заезд на рельсы: фанера метр на метр и трубы с П-образным концом. -->')
    A('<sdf version="1.9">')
    A('  <world name="rails">')
    A('    <!-- Шаг 1 мс: у робота 48 отдельных тел роликов, и к ним')
    A('         добавился контакт бочки ролика с крутым бортом трубы. -->')
    A('    <physics name="1ms" type="dart">')
    A('      <max_step_size>0.001</max_step_size>')
    A('      <real_time_factor>1.0</real_time_factor>')
    A('    </physics>')
    for p in ('physics-system" name="gz::sim::systems::Physics',
              'user-commands-system" name="gz::sim::systems::UserCommands',
              'scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster',
              'imu-system" name="gz::sim::systems::Imu'):
        A(f'    <plugin filename="gz-sim-{p}"/>')
    A('    <plugin filename="gz-sim-sensors-system"')
    A('            name="gz::sim::systems::Sensors">')
    A('      <render_engine>ogre2</render_engine>')
    A('    </plugin>')
    A('')
    A('    <light type="directional" name="sun">')
    A('      <cast_shadows>true</cast_shadows>')
    A('      <pose>0 0 10 0 0 0</pose>')
    A('      <diffuse>0.85 0.85 0.85 1</diffuse>')
    A('      <specular>0.25 0.25 0.25 1</specular>')
    A('      <direction>-0.4 0.2 -0.9</direction>')
    A('    </light>')
    A('')
    A('    <model name="ground_plane">')
    A('      <static>true</static>')
    A('      <link name="link">')
    A('        <collision name="collision">')
    A('          <geometry><plane><normal>0 0 1</normal>'
      '<size>60 60</size></plane></geometry>')
    A('          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2>'
      '</ode></friction></surface>')
    A('        </collision>')
    if not a.room:
        A('        <visual name="visual">')
        A('          <geometry><plane><normal>0 0 1</normal>'
          '<size>60 60</size></plane></geometry>')
        A('          <material><ambient>0.32 0.32 0.34 1</ambient>')
        A('            <diffuse>0.42 0.42 0.45 1</diffuse></material>')
        A('        </visual>')
    A('      </link>')
    A('    </model>')
    A('')
    if a.room:
        emit_room(A, a, x_edge, deck_x0)
    A('')
    A(f'    <!-- Настил {a.sheets_x}x{a.sheets_y} листа по'
      f' {a.sheet:.2f} м, верх на {a.deck_height * 1000:.0f} мм.')
    A('         Кромка, с которой рельсы уходят на опоры, лежит в x=0.')
    A('         Листы рисуются отдельными визуалами со швом, а коллизия')
    A('         одна на весь настил: швы физически ничего не значат. -->')
    A('    <model name="deck">')
    A('      <static>true</static>')
    A('      <link name="link">')
    A('        <collision name="collision">')
    A(f'          <pose>{(deck_x0 + deck_x1) / 2:.4f}'
      f' {(deck_y0 + deck_y1) / 2:.4f} {a.deck_height / 2:.4f} 0 0 0</pose>')
    A(f'          <geometry><box><size>{deck_x:.3f} {deck_y:.3f}'
      f' {a.deck_height:.4f}</size></box></geometry>')
    A('          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2>'
      '</ode></friction></surface>')
    A('        </collision>')
    for i in range(a.sheets_x):
        for j in range(a.sheets_y + a.sheets_left):
            sx = deck_x0 + (i + 0.5) * a.sheet
            sy = deck_y0 + (j + 0.5) * a.sheet
            A(f'        <visual name="sheet_{i}_{j}">')
            # Листы поворачиваем через один: волокно фанеры, идущее у всех
            # одинаково, даёт на стыках регулярный рисунок — ту же беду,
            # что и неповёрнутые плитки пола.
            syaw = (math.pi / 2) if (i + j) % 2 else 0.0
            A(f'          <pose>{sx:.4f} {sy:.4f} {a.deck_height / 2:.4f}'
              f' 0 0 {syaw:.4f}</pose>')
            A(f'          <geometry><box><size>{a.sheet - DECK_GAP:.4f}'
              f' {a.sheet - DECK_GAP:.4f} {a.deck_height:.4f}'
              '</size></box></geometry>')
            A('          <material><diffuse>1 1 1 1</diffuse>')
            A('            <pbr><metal><albedo_map>'
              'model://maze_nav/meshes/plywood.png</albedo_map>'
              '<roughness>0.85</roughness><metalness>0</metalness>'
              '</metal></pbr></material>')
            A('        </visual>')
    A('      </link>')
    A('    </model>')
    A('')
    A(f'    <!-- Рельсы: труба {a.pipe_od * 1000:.0f}x{PIPE_WALL * 1000:.0f} мм,')
    A(f'         колея {a.gauge * 1000:.0f} мм, П-образный конец с радиусом')
    A(f'         {a.bend_radius * 1000:.0f} мм и прямой перемычкой'
      f' {straight * 1000:.0f} мм.')
    A(f'         Масса прямых участков {pipe_mass(RAIL_END_X - x_tan) * 2:.0f} кг. -->')
    A('    <model name="rails">')
    A('      <static>true</static>')
    A('      <link name="link">')

    def cyl(name, pose, length, radius=None):
        rr = radius if radius else r
        for tag in ('collision', 'visual'):
            A(f'        <{tag} name="{tag}_{name}">')
            A(f'          <pose>{pose}</pose>')
            A(f'          <geometry><cylinder><radius>{rr:.5f}</radius>'
              f'<length>{length:.4f}</length></cylinder></geometry>')
            if tag == 'visual':
                A('          <material><ambient>0.30 0.30 0.33 1</ambient>')
                A('            <diffuse>0.55 0.56 0.60 1</diffuse>')
                A('            <specular>0.45 0.45 0.45 1</specular></material>')
            else:
                A('          <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2>'
                  '</ode></friction></surface>')
            A(f'        </{tag}>')

    # прямые рельсы
    for side, sgn in (('l', 1), ('r', -1)):
        ln = RAIL_END_X - x_tan
        cyl(f'rail_{side}',
            f'{x_tan + ln / 2:.4f} {sgn * y_rail:.4f} {pz:.4f} 0 {math.pi / 2:.6f} 0',
            ln)
    # прямая перемычка
    cyl('cross', f'{x_cross:.4f} 0 {pz:.4f} {math.pi / 2:.6f} 0 0', straight)
    # углы: дуга набирается короткими цилиндрами — тора в SDF нет
    A('        <!-- Углы П: дуга из коротких цилиндров, тора в SDF нет.')
    A('             Цилиндр ставится ВДОЛЬ КАСАТЕЛЬНОЙ к дуге, а не по углу')
    A('             самой дуги — это разные вещи, они расходятся ровно на 90°,')
    A('             и если перепутать, звенья встают поперёк и дуга')
    A('             разваливается на ёлочку. -->')
    for side, sgn in (('l', 1), ('r', -1)):
        cx, cy = x_tan, sgn * y_cross
        dth = (math.pi / 2) / ARC_SEGMENTS
        # Длина звена — хорда с небольшим нахлёстом, чтобы между соседними
        # не оставалось щели: 2R*tan(dth/2) даёт стык ровно встык.
        seg = 2 * a.bend_radius * math.tan(dth / 2) * 1.15
        for i in range(ARC_SEGMENTS):
            th = (i + 0.5) * dth
            px = cx - a.bend_radius * math.cos(th)
            py = cy + sgn * a.bend_radius * math.sin(th)
            # касательная к дуге в этой точке
            tx, ty = math.sin(th), sgn * math.cos(th)
            yaw = math.atan2(ty, tx)
            cyl(f'arc_{side}_{i}',
                f'{px:.4f} {py:.4f} {pz:.4f} 0 {math.pi / 2:.6f} {yaw:.6f}',
                seg)
    A('      </link>')
    A('    </model>')
    A('')
    A(f'    <!-- Опоры за фанерой, шаг {SUPPORT_SPACING} м. Стойки строго под')
    A('         трубами: меканум-колёса проходят снаружи них. -->')
    xs, k = [], 0
    while True:
        x = x_edge + 0.25 + k * SUPPORT_SPACING
        if x > RAIL_END_X - 0.1:
            break
        xs.append(x)
        k += 1
    pw, pd, pt = SUPPORT_PLATE
    for n, x in enumerate(xs):
        A(f'    <model name="support_{n}">')
        A('      <static>true</static>')
        A('      <link name="link">')
        for tag in ('visual', 'collision'):
            A(f'        <{tag} name="plate_{tag}">')
            A(f'          <pose>{x:.3f} 0 {pt / 2:.4f} 0 0 0</pose>')
            A(f'          <geometry><box><size>{pd:.3f} {pw:.3f} {pt:.4f}'
              '</size></box></geometry>')
            if tag == 'visual':
                A('          <material><ambient>0.26 0.26 0.28 1</ambient>')
                A('            <diffuse>0.46 0.47 0.50 1</diffuse></material>')
            A(f'        </{tag}>')
        for side, sgn in (('l', 1), ('r', -1)):
            h = a.deck_height - pt
            for tag in ('visual', 'collision'):
                A(f'        <{tag} name="post_{side}_{tag}">')
                A(f'          <pose>{x:.3f} {sgn * y_rail:.4f}'
                  f' {pt + h / 2:.4f} 0 0 0</pose>')
                A(f'          <geometry><box><size>{SUPPORT_POST:.3f}'
                  f' {SUPPORT_POST:.3f} {h:.4f}</size></box></geometry>')
                if tag == 'visual':
                    A('          <material><ambient>0.26 0.26 0.28 1</ambient>')
                    A('            <diffuse>0.46 0.47 0.50 1</diffuse></material>')
                A(f'        </{tag}>')
        A('      </link>')
        A('    </model>')
    A('  </world>')
    A('</sdf>')

    open(a.out, 'w').write('\n'.join(L) + '\n')
    axle_deck = a.deck_height + WHEEL_R
    axle_rail = a.deck_height + a.pipe_od + ROLLER_R
    print(f'записано {a.out}')
    print(f'  настил {deck_x:.2f}x{deck_y:.2f} м (y от {deck_y0:+.2f} до'
          f' {deck_y1:+.2f}) из листов по {a.sheet:.2f} м,'
          f' x от {deck_x0:.2f} до {deck_x1:.2f}, верх'
          f' {a.deck_height * 1000:.0f} мм')
    print(f'  робот ставить в x={deck_x0 + deck_x / 2:.2f}, z={a.deck_height + 0.01:.2f}')
    print(f'  П: внешняя грань x={x_face:.3f}, ось перемычки x={x_cross:.3f},'
          f' конец угла x={x_tan:.3f}')
    print(f'  прямые рельсы от x={x_tan:.3f} до x={RAIL_END_X}, колея'
          f' {a.gauge * 1000:.0f} мм, радиус угла {a.bend_radius * 1000:.0f} мм')
    print(f'  вылет П на фанеру {(x_edge - x_face) * 1000:.0f} мм,'
          f' прямой перемычки {straight * 1000:.0f} мм')
    print(f'  ступенька при передаче опоры {(axle_rail - axle_deck) * 1000:+.1f} мм')
    print(f'  опор за фанерой: {len(xs)}')


if __name__ == '__main__':
    main()
