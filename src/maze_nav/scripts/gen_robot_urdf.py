#!/usr/bin/env python3
"""Генератор URDF робота из данных, извлечённых прямо из CAD-модели.

Зачем генератор, а не рукописный xacro. Расстановку роликов меканума
я пробовал вывести формулой и дважды ошибся: сначала развернул оси
по радиусу вместо касательной, потом промахнулся с положением плоскости
колеса. При этом в CAD всё это задано точно. Поэтому: читаем geometry
из модели, а URDF собираем по ней.

Что оказалось неверным в "выведенных" числах и верным в измеренных:

    колея              610 -> 710.8 мм
    база               640 -> 635.6 мм
    радиус кольца      83.2 -> 86.0 мм
    ориентация роликов радиальная -> касательная

Вход: JSON от скрипта извлечения (положения и оси роликов в системе колеса).
Выход: .urdf.xacro с явными числами.

    ./gen_robot_urdf.py wheels.json -o agro_robot.urdf.xacro
"""
import argparse
import json
import math
import pathlib
import sys

# Массы: заданы пользователем, в CAD их нет
M_TOTAL = 120.0
M_WHEEL_ASSY = 5.0
M_ROLLER = 0.040

# Рельсовый ролик. Профиль промерен 21 сечением вдоль оси и оказался
# железнодорожным: цилиндрическая дорожка качения радиусом 50 мм, затем
# вогнутая галтель и реборда радиусом 75 мм. Реборда держит от схода,
# катится колесо по дорожке — значит контактный радиус именно 50, а не 81,
# как я считал по габариту.
# ═══ Профиль рельсового ролика ═══
# Ступенчатая замена вогнутой канавки: (радиус, от, до) по оси колеса,
# считая от плоскости колеса внутрь. Невыпуклую канавку движки одним телом
# не считают, поэтому набираем её цилиндрами.
#
# Числа ИЗМЕРЕНЫ по meshes/wheel_hub_l.stl, а не назначены:
#   python3 -c "import numpy,struct; ..." — см. RUN.md, этап 5.
# Дорожка качения — ровный цилиндр R=50 шириной 40 мм, дальше галтель
# 51.8 -> 54 -> 57 -> 60.6 -> 64.8 и реборда 75.
#
# Радиус в каждой полосе взят по ВЕРХНЕЙ границе измеренного: так тело
# никогда не уже настоящего, и труба не проваливается внутрь.
#
# Зачем так подробно. Раньше здесь была пара цилиндров, дорожка и реборда,
# и реборда R=75 доходила до |y|=62.6 — то есть наезжала на край трубы
# Ø51. Робот вставал на рельсы на 11.9 мм выше расчётного и ехал не по
# дорожке, а заклиненным в прямом углу между цилиндрами. В симуляторе это
# выглядело как загадочный подъём на 12 мм при заезде.
RAIL_PROFILE = (
    (0.0500, 0.0650, 0.1049),   # дорожка качения — ею едем по трубе
    (0.0520, 0.0600, 0.0650),
    (0.0550, 0.0560, 0.0600),
    (0.0580, 0.0530, 0.0560),
    (0.0620, 0.0500, 0.0530),
    (0.0680, 0.0470, 0.0500),
    (0.0750, 0.0399, 0.0470),   # реборда — держит от схода вбок
)

RAIL_TREAD_R = 0.050        # дорожка качения: ЭТИМ радиусом едем по рельсе
RAIL_TREAD_Y = (0.0626, 0.1049)   # её протяжённость вдоль оси, от оси колеса
RAIL_FLANGE_R = 0.075       # реборда
RAIL_FLANGE_Y = (0.0399, 0.0626)

# ДВЕ РАЗНЫЕ высоты, которые легко перепутать (я и перепутал — робот висел
# над землёй на 2 мм):
#
#   CAD_AXLE_Z — на какой высоте ось колеса лежит В КООРДИНАТАХ CAD.
#                Нужна, чтобы правильно сместить меши относительно base_link.
#   WHEEL_R    — внешний радиус меканума. Именно на эту высоту base_link
#                поднят над base_footprint, потому что base_footprint по
#                соглашению стоит в ТОЧКЕ КАСАНИЯ колеса с землёй.
#
# Они не совпадают: в CAD низ колеса приходится на Z = 103.5 − 101.4 = 2.1 мм,
# то есть нулевая отметка модели проходит на 2 мм ниже земли.
CAD_AXLE_Z = 0.1035
# Радиус промерен по вершинам роликов, а не по габаритной коробке:
# у наклонённой бочки габарит зависит от её фазы на окружности и давал
# ложный разброс 104.7 против 111.8 мм на разных колёсах.
# ═══ Цвета ═══
# STL не несёт материалов, поэтому без этих строк всё белое: на светлом полу
# робот превращается в плоский силуэт, по которому не видно ни граней корпуса,
# ни того, крутятся ли ролики. Ролики намеренно светлее корпуса — иначе на
# тёмном колесе не разглядеть вращение.
COLORS = {
    'chassis': (0.22, 0.23, 0.25),
    'hub':     (0.15, 0.15, 0.17),
    'roller':  (0.38, 0.39, 0.42),
}

# ═══ Камера ═══
# RealSense D435 на штативе 10 см, привинченном к переднему профилю крыши
# по центру. Профиль по обмеру меша корпуса стоит на CAD Y=+382 мм, верх
# крыши на CAD Z=740.8 мм — это и есть числа ниже, пересчитанные в систему
# base_link.
#
# Наклон 30° вниз — компромисс, и стоит понимать, между чем.
# Круче вниз лучше видно рельсы, но кадр забивает пол: фанера и ковролин
# почти без текстуры, и визуальной одометрии не за что держаться. Положе —
# наоборот. При 30° и высоте 0.839 м над полом нижний луч (30+29°) бьёт в
# землю в 0.50 м перед камерой, то есть с x=0.87 от центра робота, а
# верхний уходит почти в горизонт и оставляет дальний план для SLAM.
#
# Мёртвая зона перед бампером (бампер на x=0.60) получается 0.27 м. Этого
# достаточно: рельсы меряются с 1.0-1.3 м, а последние полметра робот
# едет вслепую — запас по геометрии 40 мм, снос за полметра около 5 мм.
# Кронштейн крепится к ПЕРЕДНЕМУ ПРОФИЛЮ крыши и выносит камеру вперёд
# на 10 см, а не поднимает вверх.
# Профиль по обмеру меша: CAD Y 395..410 (передняя грань 410.5), Z 641..718.
# Берём его верх — CAD Z=718.2, это base_link z=0.6147. Верхняя плоскость
# крыши на 23 мм выше (CAD 740.8), но это уже другой элемент, дальше назад.
CAM_MOUNT_XYZ = (0.400, 0.0, 0.6147)
CAM_ARM = 0.10
CAM_PITCH = math.radians(30)
# D435: глубина 87x58°, 848x480. Горизонтальный угол задаём, вертикальный
# Gazebo выводит из соотношения сторон: 2*atan(tan(87/2)*480/848) = 56.5°.
#
# Разрешение и частота — рычаг производительности, а не мелочь.
# Замерено: 848x480 при 30 Гц роняют темп симуляции с 1.0 до 0.29. В
# МОДЕЛЬНОМ времени камера при этом честные 30 Гц и алгоритм видит то же,
# что увидит на железе, — проседает только скорость прогонов.
# D435 умеет и 640x480, и 15 кадр/с, так что снижать можно без вранья.
CAM_W, CAM_H = 640, 480
CAM_RATE = 15
CAM_HFOV = math.radians(87.0)
CAM_NEAR, CAM_FAR = 0.28, 10.0      # min-Z у D435 на 848x480

MEC_ROLLER_R = 0.01545
MEC_ROLLER_L = 0.0669
WHEEL_R = 0.1014
MEC_R = WHEEL_R
ROLLER_R = MEC_ROLLER_R


def rpy_from_axis(a):
    """Углы Эйлера, переводящие локальную ось Z цилиндра в направление a.

    URDF собирает поворот как Rz(yaw)·Ry(pitch)·Rx(roll). При yaw=0
    орт (0,0,1) переходит в (cos r·sin p, −sin r, cos r·cos p).
    Отсюда обратные формулы. Цилиндр симметричен вокруг своей оси,
    поэтому свободу по yaw можно занулить.
    """
    ax, ay, az = a
    roll = math.asin(max(-1.0, min(1.0, -ay)))
    c = math.cos(roll)
    pitch = math.atan2(ax / c, az / c) if abs(c) > 1e-9 else 0.0
    return roll, pitch, 0.0


def inertia_cyl(m, r, h, axis):
    """Моменты инерции цилиндра. axis: 'x', 'y' или 'z' — вдоль чего ось."""
    along = m * r * r / 2
    across = m * (3 * r * r + h * h) / 12
    return {'x': (along, across, across),
            'y': (across, along, across),
            'z': (across, across, along)}[axis]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('data', type=pathlib.Path)
    ap.add_argument('-o', '--out', type=pathlib.Path, required=True)
    ap.add_argument('--roller-layout', choices=('standard', 'cad'),
                    default='standard',
                    help='ориентация роликов меканума. cad — ровно как в STEP; '
                         'standard — зеркально к STEP, общепринятая раскладка. '
                         'Влияет на то, способен ли робот разворачиваться '
                         'качением; см. комментарий в коде')
    ap.add_argument('--roller-collision', choices=('mesh', 'cylinder'),
                    default='mesh',
                    help='чем ролик меканума сталкивается с полом; cylinder — '
                         'запасной вариант, если 48 мешей тормозят симулятор')
    ap.add_argument('--controllers',
                    default='$(find maze_nav)/params/agro_controllers.yaml',
                    help='путь к yaml контроллеров. По умолчанию подстановка '
                         'xacro: плагину Gazebo нужен абсолютный путь, а $(find) '
                         'развернётся в него на той машине, где запускают')
    args = ap.parse_args()

    d = json.load(open(args.data))
    wheels = {k: v for k, v in d.items() if v}
    if len(wheels) != 4:
        print(f'ожидалось 4 колеса, найдено {len(wheels)}', file=sys.stderr)
        return 1

    n_roll = len(next(iter(wheels.values()))['rollers'])
    m_hub = M_WHEEL_ASSY - n_roll * M_ROLLER
    m_base = M_TOTAL - 4 * M_WHEEL_ASSY

    L = []
    A = L.append
    A('<?xml version="1.0"?>')
    A('<!-- СГЕНЕРИРОВАНО scripts/gen_robot_urdf.py — руками не править.')
    A('     Геометрия колёс прочитана из CAD-модели, а не выведена формулой:')
    A('     положения и оси роликов взяты как есть, включая направление')
    A('     наклона для каждого колеса. Пересобрать после изменения модели. -->')
    A('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="agro_robot">')
    A('')
    # Объявляем ОДИН раз и ниже ссылаемся по имени. Повторять цвет в каждом
    # визуале нельзя: urdfdom ругается на неуникальный материал.
    for cname, (cr, cg, cb) in COLORS.items():
        A(f'  <material name="{cname}"><color rgba="{cr} {cg} {cb} 1"/></material>')
    A('')
    A('  <link name="base_footprint"/>')
    A('  <joint name="base_joint" type="fixed">')
    A('    <parent link="base_footprint"/><child link="base_link"/>')
    A(f'    <origin xyz="0 0 {WHEEL_R}"/>')
    A('  </joint>')
    A('')
    A('  <link name="base_link">')
    A('    <inertial>')
    A('      <origin xyz="0 0 0.22"/>')
    A(f'      <mass value="{m_base:.3f}"/>')
    A('      <inertia ixx="11.0" iyy="16.6" izz="18.5" ixy="0" ixz="0" iyz="0"/>')
    A('    </inertial>')
    A('    <visual>')
    A(f'      <origin xyz="0 0 {-CAD_AXLE_Z}" rpy="0 0 {-math.pi/2:.6f}"/>')
    A('      <geometry><mesh filename="package://maze_nav/meshes/chassis.stl"')
    A('              scale="0.001 0.001 0.001"/></geometry>')
    A('      <material name="chassis"/>')
    A('    </visual>')
    A('    <!-- Коллизия примитивом: меш в 680 тыс. граней убил бы физику -->')
    A('    <collision>')
    A('      <origin xyz="0 0 0.319"/>')
    A('      <geometry><box size="1.2016 0.9228 0.6375"/></geometry>')
    A('    </collision>')
    A('  </link>')

    # Имя колеса назначаем ПО ФАКТИЧЕСКОМУ положению в ROS, а не наследуем
    # из исходных данных: ярлыки в CAD-выгрузке были привязаны к прикидочным
    # центрам в координатах CAD, и после поворота CAD->ROS стороны менялись
    # местами (то, что звалось fl, оказывалось справа).
    placed = []
    for w in wheels.values():
        px, py = w['plane_x_cad'] / 1000.0, w['axis_y_cad'] / 1000.0
        wx, wy = py, -px          # ROS X вперёд = CAD Y; ROS Y влево = -CAD X
        nm = ('f' if wx > 0 else 'r') + ('l' if wy > 0 else 'r')
        placed.append((nm, wx, wy, w))
    placed.sort(key=lambda t: t[0])

    for name, wx, wy, w in placed:

        ixx, iyy, izz = inertia_cyl(m_hub, MEC_R, 0.065, 'y')
        A('')
        A(f'  <!-- колесо {name}: CAD плоскость X={w["plane_x_cad"]}, '
          f'ось Y={w["axis_y_cad"]}, кольцо роликов R={w["ring_mm"]} мм -->')
        A(f'  <link name="{name}_wheel">')
        A('    <inertial>')
        A(f'      <mass value="{m_hub:.3f}"/>')
        A(f'      <inertia ixx="{ixx:.6f}" iyy="{iyy:.6f}" izz="{izz:.6f}"'
          ' ixy="0" ixz="0" iyz="0"/>')
        A('    </inertial>')
        side = 'l' if wy > 0 else 'r'
        rd = -1.0 if wy > 0 else 1.0     # рельсовый ролик смотрит ВНУТРЬ робота
        A('    <!-- Визуал — геометрия узла из CAD как есть: ступица, диски')
        A('         и рельсовый ролик с настоящим профилем. Меш уже развёрнут')
        A('         в систему звена при извлечении, поправок не нужно.')
        A('         Ролики меканума сюда НЕ входят: они вращаются отдельно. -->')
        A(f'    <visual><geometry><mesh')
        A(f'      filename="package://maze_nav/meshes/wheel_hub_{side}.stl"')
        A('      scale="0.001 0.001 0.001"/></geometry>')
        A('      <material name="hub"/></visual>')
        A('    <!-- Коллизия рельсового ролика: набор цилиндров по измеренному')
        A('         профилю (см. RAIL_PROFILE). Канавка вогнутая, одним телом')
        A('         её не посчитать — движки берут только выпуклые формы. -->')
        for rr_, y0, y1 in RAIL_PROFILE:
            yc = rd * (y0 + y1) / 2
            A(f'    <collision><!-- R={rr_*1000:.1f} мм -->')
            A(f'      <origin xyz="0 {yc:.4f} 0" rpy="{math.pi/2:.6f} 0 0"/>')
            A(f'      <geometry><cylinder radius="{rr_}" length="{y1-y0:.4f}"/></geometry>')
            A('    </collision>')
        A('  </link>')
        A(f'  <joint name="{name}_wheel_joint" type="continuous">')
        A(f'    <parent link="base_link"/><child link="{name}_wheel"/>')
        A(f'    <origin xyz="{wx:.4f} {wy:.4f} 0"/>')
        A('    <axis xyz="0 1 0"/>')
        A('    <dynamics damping="0.05" friction="0.0"/>')
        A('  </joint>')

        rixx, riyy, rizz = inertia_cyl(M_ROLLER, ROLLER_R, 0.065, 'z')
        for i, r in enumerate(w['rollers']):
            axis = r['axis']
            if args.roller_layout == 'standard':
                # Зеркалим руку ролика: меняем знак составляющей ВДОЛЬ оси
                # колеса, тангенциальную оставляем. Ролик под +45° становится
                # ролик под -45°, положение не двигается.
                #
                # Зачем. Плечо, с которым колесо создаёт момент разворота,
                # для меканума равно (полубаза ± полуколея): знак задаётся
                # именно рукой ролика. В STEP рука такая, что плечо равно
                # РАЗНОСТИ: |0.3178 - 0.3554| = 0.0376 м против 0.6732 м у
                # общепринятой раскладки — в 18 раз меньше. Робот с такими
                # колёсами разворачивается не качением, а юзом, и колёсная
                # одометрия по развороту врёт в разы (замерено: 2.58 рад
                # против 1.85 рад по колёсам).
                #
                # Это почти наверняка огрех модели: в CAD на все четыре угла
                # поставлено колесо одной руки. На железе надо посмотреть на
                # верхние ролики: у исправного меканума пары по диагонали
                # наклонены одинаково, а сами диагонали — по-разному.
                # Если окажется, что железо собрано как в CAD, генерировать
                # с --roller-layout cad и переставить перёд/зад в
                # params/agro_controllers.yaml (там об этом написано).
                axis = [axis[0], -axis[1], axis[2]]
            roll, pitch, yaw = rpy_from_axis(axis)
            px_, py_, pz_ = r['pos']
            A(f'  <link name="{name}_roller_{i}">')
            A('    <inertial>')
            A(f'      <mass value="{M_ROLLER}"/>')
            A(f'      <inertia ixx="{rixx:.8f}" iyy="{riyy:.8f}" izz="{rizz:.8f}"'
              ' ixy="0" ixz="0" iyz="0"/>')
            A('    </inertial>')
            A('    <visual><geometry><mesh'
              ' filename="package://maze_nav/meshes/mec_roller.stl"'
              ' scale="0.001 0.001 0.001"/></geometry>'
              '<material name="roller"/></visual>')
            # Ролик — бочка, а не цилиндр: радиус максимален в середине и
            # спадает к краям. Прямой цилиндр даёт контакт линией длиной 67 мм
            # под 45°, тогда как настоящая бочка касается пола точкой. Бочка
            # выпуклая, поэтому выпуклая оболочка меша совпадает с ней точно
            # и стоит недорого — беру геометрию из CAD.
            if args.roller_collision == 'mesh':
                A('    <collision><geometry><mesh'
                  ' filename="package://maze_nav/meshes/mec_roller.stl"'
                  ' scale="0.001 0.001 0.001"/></geometry></collision>')
            else:
                A(f'    <collision><geometry><cylinder radius="{MEC_ROLLER_R}"'
                  f' length="{MEC_ROLLER_L}"/></geometry></collision>')
            A('  </link>')
            A(f'  <joint name="{name}_roller_{i}_joint" type="continuous">')
            A(f'    <parent link="{name}_wheel"/><child link="{name}_roller_{i}"/>')
            A(f'    <origin xyz="{px_:.5f} {py_:.5f} {pz_:.5f}"'
              f' rpy="{roll:.6f} {pitch:.6f} {yaw:.6f}"/>')
            A('    <axis xyz="0 0 1"/>')
            A('    <dynamics damping="0.0001" friction="0.0"/>')
            A('  </joint>')

    # ═══ ros2_control ═══
    A('')
    A('  <!-- Приводы получают ТОЛЬКО четыре шарнира колёс.')
    A('       48 роликов меканума остаются свободными: их вращает контакт')
    A('       с полом, и именно поэтому боковое движение возникает')
    A('       из физики, а не рисуется кинематикой. -->')
    A('  <ros2_control name="GazeboSystem" type="system">')
    A('    <hardware>')
    A('      <plugin>gz_ros2_control/GazeboSimSystem</plugin>')
    A('    </hardware>')
    for nm, _, _, _ in placed:
        A(f'    <joint name="{nm}_wheel_joint">')
        A('      <!-- Мотор-колесо с редуктором управляется скоростью,')
        A('           поэтому командный интерфейс velocity, а не effort:')
        A('           иначе поведение на проскальзывании будет непохожим. -->')
        A('      <command_interface name="velocity">')
        A('        <param name="min">-20.0</param>')
        A('        <param name="max">20.0</param>')
        A('      </command_interface>')
        A('      <state_interface name="position"/>')
        A('      <state_interface name="velocity"/>')
        A('    </joint>')
    A('  </ros2_control>')
    A('')
    A('  <gazebo>')
    A('    <plugin filename="gz_ros2_control-system"')
    A('            name="gz_ros2_control::GazeboSimROS2ControlPlugin">')
    A('      <parameters>CONTROLLERS_YAML</parameters>')
    A('    </plugin>')
    A('  </gazebo>')
    # ═══ Трение ═══
    # Без этих тегов sdformat ставит mu=1 всем подряд. Роликам нужно
    # сцепление резины по бетону, а ступице с роликом рельсы — тоже, но
    # корпусу лучше скользить: если он ляжет на препятствие, робот должен
    # соскользнуть, а не прилипнуть.
    for nm, _, _, _ in placed:
        for i in range(n_roll):
            A(f'  <gazebo reference="{nm}_roller_{i}">')
            A('    <mu1>1.0</mu1><mu2>1.0</mu2>')
            A('  </gazebo>')
        A(f'  <gazebo reference="{nm}_wheel">')
        A('    <mu1>1.0</mu1><mu2>1.0</mu2>')
        A('  </gazebo>')
    A('  <gazebo reference="base_link">')
    A('    <mu1>0.2</mu1><mu2>0.2</mu2>')
    A('  </gazebo>')
    A('')
    # ═══ Камера ═══
    A('')
    A('  <!-- Кронштейн: от профиля крыши ВПЕРЁД, горизонтально.')
    A('       Наклон живёт отдельно, в camera_link, чтобы менять его одной')
    A('       константой, не трогая вынос. -->')
    A('  <link name="camera_mount_link">')
    A(f'    <visual><origin xyz="{-CAM_ARM/2:.4f} 0 0" rpy="0 {math.pi/2:.6f} 0"/>')
    A(f'      <geometry><cylinder radius="0.012" length="{CAM_ARM}"/></geometry>')
    A('      <material name="hub"/></visual>')
    A('  </link>')
    A('  <joint name="camera_mount_joint" type="fixed">')
    A('    <parent link="base_link"/><child link="camera_mount_link"/>')
    A(f'    <origin xyz="{CAM_MOUNT_XYZ[0] + CAM_ARM:.4f} {CAM_MOUNT_XYZ[1]}'
      f' {CAM_MOUNT_XYZ[2]}"/>')
    A('  </joint>')
    A('')
    A(f'  <!-- D435, наклон {math.degrees(CAM_PITCH):.0f}° вниз -->')
    A('  <link name="camera_link">')
    A('    <visual><geometry><box size="0.025 0.090 0.025"/></geometry>')
    A('      <material name="hub"/></visual>')
    A('  </link>')
    A('  <joint name="camera_joint" type="fixed">')
    A('    <parent link="camera_mount_link"/><child link="camera_link"/>')
    A(f'    <origin rpy="0 {CAM_PITCH:.6f} 0"/>')
    A('  </joint>')
    A('  <!-- Оптический кадр: z вперёд, x вправо, y вниз — как принято в ROS.')
    A('       Именно его имя должно стоять в gz_frame_id ниже, иначе Gazebo')
    A('       проштампует снимки составным именем вида model/link/sensor,')
    A('       которого нет в TF, и SLAM молча развалится. -->')
    A('  <link name="camera_depth_optical_frame"/>')
    A('  <joint name="camera_optical_joint" type="fixed">')
    A('    <parent link="camera_link"/><child link="camera_depth_optical_frame"/>')
    A(f'    <origin rpy="{-math.pi/2:.6f} 0 {-math.pi/2:.6f}"/>')
    A('  </joint>')
    A('')
    A('  <gazebo reference="camera_link">')
    A('    <sensor name="d435" type="rgbd_camera">')
    A(f'      <update_rate>{CAM_RATE}</update_rate>')
    A('      <always_on>1</always_on>')
    A('      <topic>camera</topic>')
    A('      <gz_frame_id>camera_depth_optical_frame</gz_frame_id>')
    A('      <camera>')
    A(f'        <horizontal_fov>{CAM_HFOV:.6f}</horizontal_fov>')
    A(f'        <image><width>{CAM_W}</width><height>{CAM_H}</height>')
    A('          <format>R8G8B8</format></image>')
    A(f'        <clip><near>{CAM_NEAR}</near><far>{CAM_FAR}</far></clip>')
    A('        <optical_frame_id>camera_depth_optical_frame</optical_frame_id>')
    A('      </camera>')
    A('    </sensor>')
    A('  </gazebo>')
    # ═══ IMU ═══
    A('')
    A('  <!-- Стоит в начале base_link: центр колёсной базы на высоте оси.')
    A('       Место выбрано не для удобства. Гироскоп от положения не')
    A('       зависит вовсе — угловая скорость у твёрдого тела одинакова')
    A('       везде. А вот акселерометр, смещённый на r от центра вращения,')
    A('       ловит при повороте центростремительное w^2*r и тангенциальное')
    A('       a*r. При r=0.5 м и w=0.5 рад/с это 0.125 м/с2 — вектор')
    A('       гравитации наклоняется на 0.7°, а из него берутся крен и')
    A('       тангаж. robot_localization поворачивает данные по TF, но')
    A('       вынос плеча НЕ компенсирует, так что одним объявлением')
    A('       положения тут не отделаться.')
    A('       Нам крен нужен чистым: по нему отличается правильная посадка')
    A('       на дорожку качения от заезда на галтель. -->')
    A('  <link name="imu_link"/>')
    A('  <joint name="imu_joint" type="fixed">')
    A('    <parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0"/>')
    A('  </joint>')
    A('  <gazebo reference="imu_link">')
    A('    <sensor name="imu" type="imu">')
    A('      <update_rate>100</update_rate>')
    A('      <always_on>1</always_on>')
    A('      <topic>imu</topic>')
    A('      <gz_frame_id>imu_link</gz_frame_id>')
    A('    </sensor>')
    A('  </gazebo>')
    A('</robot>')

    args.out.write_text('\n'.join(L).replace('CONTROLLERS_YAML', args.controllers) + '\n')
    total = m_base + 4 * (m_hub + n_roll * M_ROLLER)
    print(f'записано {args.out}')
    print(f'  колёс 4, роликов на колесо {n_roll}, масса {total:.2f} кг')
    print(f'  раскладка роликов: {args.roller_layout}'
          + (' (зеркально к STEP)' if args.roller_layout == 'standard'
             else ' (как в STEP)'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
