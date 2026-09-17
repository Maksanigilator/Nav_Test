#!/usr/bin/env python3
"""Сцена этапа 3: TurtleBot3 в Isaac Sim, управляемый из контейнера.

Первый срез: робот стоит на полу, публикует /clock и едет от /cmd_vel.
Камеры здесь ещё нет намеренно — сначала надо убедиться, что физика и привод
работают, иначе неподвижный робот будет выглядеть как поломка камеры.

Роль этой сцены в этапе 3. Робот — КОНТРОЛЬ: тот же самый TurtleBot3, что
в этапе 2 на Gazebo, импортированный из того же URDF, по которому
robot_state_publisher в контейнере строит дерево TF. Переменной должно быть
только окружение. Иначе разрыв ATE между этапами 2 и 3 будет мерить смесь
«другой рендер» и «другой робот», а нужен чистый ответ про реализм.

    source isaac/env.sh
    "$ISAAC_PYTHON" isaac/tb3_sim.py            # headless
    "$ISAAC_PYTHON" isaac/tb3_sim.py --gui      # с окном

Проверка из контейнера:

    ./run.sh ros2 topic hz /clock
    ./run.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard

ВАЖНО про /cmd_vel. Узел ROS2SubscribeTwist слушает geometry_msgs/Twist,
а Nav2 в этой сборке настроен на TwistStamped (enable_stamped_cmd_vel: true
в пяти местах maze_params_rgbd.yaml — так устроен мост TB3 в Jazzy).
Для teleop это неважно, он шлёт обычный Twist. Для Nav2 понадобится снять
stamped на стороне контейнера через RewrittenYaml в launch — приём в проекте
уже применяется для путей к деревьям поведения. Трогать сам yaml нельзя:
он общий с гейзебовским этапом 2.
"""
import argparse
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
ROBOT_USD = HERE / 'assets' / 'turtlebot3_waffle' / 'turtlebot3_waffle.usd'

# Геометрия привода — из того же URDF, а не по памяти:
#   wheel_left_joint origin y = 0.144  ->  база 0.288 м
#   collision cylinder radius          ->  0.033 м
WHEEL_DISTANCE = 0.288
WHEEL_RADIUS = 0.033

# Пределы самого TurtleBot3. Выше, чем разрешает Nav2 (velocity_smoother
# режет до 0.26 / 1.0), и так и надо: ограничителем должен быть планировщик,
# а не симулятор.
MAX_LINEAR = 0.26
MAX_ANGULAR = 1.82

# ── Окружение ─────────────────────────────────────────────────────────────
# Робот в этапе 3 — КОНТРОЛЬ, окружение — ПЕРЕМЕННАЯ. Робот тот же, что
# в Gazebo на этапе 2, а сцену меняем ради текстур, бликов и честного шума
# глубины: именно их разница и должна проявиться в ATE.
#
# Пути относительно корня ассетов, который резолвится через
# get_assets_root_path() (по умолчанию сервер NVIDIA, см. настройку
# persistent.isaac.asset_root.default).
ENVIRONMENTS = {
    # Перегородки, столы и коридоры — много углов в ближней зоне. Ближе всего
    # по духу к turtlebot3_house этапа 2, поэтому годится для сравнения ATE.
    'office': 'Isaac/Environments/Office/office.usd',
    # Замкнутая комната с мебелью и паркетом, всё в пределах дальности камеры.
    'room': 'Isaac/Environments/Simple_Room/simple_room.usd',
    # Длинные пустые проходы. Проверено: в середине прохода камере не за что
    # зацепиться — пол однороден, дальняя стена за пределами clip far = 10 м,
    # и одометрия срывается на "Not enough features".
    'warehouse': 'Isaac/Environments/Simple_Warehouse/warehouse.usd',
    # Тот же склад, но со стеллажами: вертикальная структура в ближней зоне.
    'shelves': 'Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd',
}

PHYSICS_DT = 1.0 / 60.0

# Рендер вдвое реже физики: камера этапа 2 стоит на 30 Гц (update_rate в
# model.sdf), и держать её выше незачем — это вдвое лишняя нагрузка на GPU,
# а на 10 ГБ VRAM он тут не бездонный.
#
# Важнее другое: узлы камеры срабатывают по тику воспроизведения, и если
# рендер идёт чаще, чем публикуются кадры, camera_info успевает выйти на
# тиках, где нового изображения ещё нет. Замерено при рендере 60 Гц:
# camera_info ровно 50.3 Гц с разбросом 0.0006, а цвет и глубина 40-43 Гц
# с провалами до 0.081 с. Расходимость ровно того вида, из-за которой
# rgbd_odometry не может собрать синхронную тройку цвет+глубина+интринсики —
# на мосте Gazebo это уже стоило этапу 2 отдельной отладки.
RENDER_DT = 1.0 / 30.0

ROBOT_PRIM = '/World/turtlebot3'
BASE_FRAME_ID = 'base_footprint'

# ── Камера ────────────────────────────────────────────────────────────────
# Интринсики — те же, что у модели этапа 2 (models/turtlebot3_waffle_d435/
# model.sdf: horizontal_fov 1.2113 рад = 69.4 град, 848x480, clip 0.28..10).
# Менять их нельзя: критерий этапа 3 — сравнить ATE с этапом 2, а при другой
# оптике сравнение недействительно.
#
# Isaac описывает камеру фокусным расстоянием и апертурой, а не углом обзора.
# Пересчёт: horizontal_aperture / (2 * focal_length) = tan(hfov / 2).
# Взяв стандартную апертуру 20.955, получаем focal_length 15.1308.
# Вертикальную апертуру берём по соотношению сторон: 20.955 * 480/848.
# Обратная проверка сошлась с конфигом костмапа: вертикальный обзор выходит
# 0.7471 рад, а в maze_params_rgbd.yaml у обоих слоёв STVL стоит
# vertical_fov_angle: 0.747. То есть модель Gazebo, камера Isaac и костмап
# описывают одну и ту же геометрию.
CAMERA_WIDTH, CAMERA_HEIGHT = 848, 480
CAMERA_FOCAL_LENGTH = 15.1308
# Фокус в ПИКСЕЛЯХ: f = W / (2*tan(hfov/2)). Совпадает с k[0] из camera_info
# (проверено на живом топике: 612.30821553). Нужен модели стереодатчика.
CAMERA_FOCAL_PIXEL = 612.31
CAMERA_H_APERTURE = 20.955
CAMERA_V_APERTURE = 11.8613
CAMERA_CLIP = (0.28, 10.0)

# ── Модель стереодатчика D435 ─────────────────────────────────────────────
# Isaac умеет отдавать не идеальную глубину из буфера рендера, а результат
# СТЕРЕОКОНВЕЙЕРА: с базой, диспаратностью, шумом и порогом уверенности.
# Это принципиально честнее, чем красить готовую глубину случайным шумом.
#
# Почему шум задаётся в диспаратности, а не в метрах. Глубина считается как
# z = f*B/d, поэтому ПОСТОЯННАЯ ошибка диспаратности даёт ошибку глубины,
# растущую как z^2. Это не подгонка, а геометрия стерео — и именно так ведёт
# себя настоящий D435.
#
# Все числа из паспорта камеры и наших же интринсик, ни одного подогнанного:
#
#   база 50 мм           — паспорт D435
#   фокус 612.31 пикс    — наш, сверен с полем k в camera_info
#   maxDisparity 110     — даёт минимальную дальность f*B/d = 0.278 м,
#                          у D435 паспортные ~0.28 м
#   noiseSigma 0.306     — решение обратной задачи под известные из литературы
#                          ~4 см СКО на 2 м (Ahn и др., анализ шума D435).
#                          Даёт 1 см на 1 м, 16 см на 4 м, 1 м на 10 м —
#                          отсюда и правило не доверять глубине вдаль.
#
# Чего модель НЕ даёт, и это надо помнить: она осевая, без бокового шума
# и зависимости от угла падения; и она не моделирует ИК-проектор. Настоящий
# D435 подсвечивает сцену точечным узором и на гладкой стене глубину как раз
# получает, а здесь уверенность считается по обычной картинке. То есть дыры
# появятся там, где реальная камера справилась бы, — в помещении симулятор
# будет пессимистичен. Сторона ошибки безопасная, но знать надо.
DEPTH_SENSOR = {
    'baselineMM': 50.0,
    'focalLengthPixel': CAMERA_FOCAL_PIXEL,
    'maxDisparityPixel': 110.0,
    'noiseMean': 0.25,        # значение Isaac по умолчанию
    'noiseSigma': 0.306,
    'minDistance': CAMERA_CLIP[0],
    'maxDistance': CAMERA_CLIP[1],
    'confidenceThreshold': 0.95,
}

# Фрейм, в котором живут кадры. Связь с базой публикует robot_state_publisher
# в КОНТЕЙНЕРЕ по тому же URDF — Isaac в /tf не пишет ничего.
CAMERA_FRAME = 'camera_rgb_optical_frame'

# Имена не произвольные: ровно эти константы прошиты в
# src/maze_nav/launch/rgbd_sim.launch.py, и стек в контейнере ждёт их.
RGB_TOPIC = '/camera/image_raw'
# ВРЕМЕННО обратно в рабочий топик: узел шума глубины отключён, см. большой
# комментарий в его шапке (src/maze_nav/scripts/depth_noise.py). Когда он
# вернётся, сюда снова надо поставить '/camera/depth/ideal', а узел встанет
# между сценой и стеком.
DEPTH_TOPIC = '/camera/depth/image_raw'
INFO_TOPIC = '/camera/camera_info'

# ── IMU ───────────────────────────────────────────────────────────────────
# Публикуем ИДЕАЛЬНЫЙ поток, а шум добавляет отдельный узел в контейнере
# (scripts/imu_noise.py). Причина: у Isaac класс IMUSensor не имеет ни одного
# параметра шума, тогда как у Gazebo он задан в модели (гироскоп 2e-4 рад/с,
# акселерометр 1.7e-2 м/с^2). Без добавки инерциалка в Isaac оказалась бы
# ЛУЧШЕ, чем на этапе 2, и симулятор стал бы легче реальности.
IMU_TOPIC = '/imu/ideal'
IMU_FRAME = 'imu_link'

# ── Истинная поза ─────────────────────────────────────────────────────────
# Нужна, чтобы посчитать ATE, а не оценивать траекторию на глаз. На этапе 2
# ту же роль играл /world/default/dynamic_pose/info из Gazebo.
#
# Публикуется ОТДЕЛЬНЫМ топиком и НИКОГДА не попадает в /tf: попади оно туда,
# у base_footprint оказалось бы два родителя — истинная поза и одометрия, —
# и дерево сломалось бы молча. Узел ROS2PublishOdometry сам в TF не пишет,
# но имена фреймов всё равно берём отличные от рабочих, чтобы никто случайно
# не подхватил их как источник трансформа.
GT_TOPIC = '/ground_truth/odom'
GT_ODOM_FRAME = 'ground_truth'

# argparse до SimulationApp: тот поднимает Kit и разбирает argv сам.
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--gui', action='store_true', help='окно Isaac')
parser.add_argument('--env', default='office',
                    choices=['office', 'room', 'warehouse', 'shelves', 'none'],
                    help='окружение. warehouse и room тянутся с сервера ассетов '
                         'NVIDIA при первом запуске: office ~706 МБ, '
                         'room ~111 МБ, warehouse и shelves ~513 МБ на двоих. '
                         'none — голая площадка, годится '
                         'только для отладки физики: текстуры там нет вовсе '
                         'и визуальная одометрия слепа (quality=0)')
parser.add_argument('--x', type=float, default=0.0, help='точка спавна, м')
parser.add_argument('--y', type=float, default=0.0)
parser.add_argument('--debug', action='store_true',
                    help='печатать значения на узлах графа: видно, '
                         'на каком звене рвётся cmd_vel -> колёса')
args = parser.parse_args()

if not ROBOT_USD.is_file():
    raise SystemExit(f'нет {ROBOT_USD}\n'
                     'сначала: ./isaac/export_robot.sh и isaac/import_robot.py')

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({'headless': not args.gui})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import GroundPlane  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

enable_extension('isaacsim.ros2.bridge')
simulation_app.update()

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT)

stage = omni.usd.get_context().get_stage()

if args.env == 'none':
    # Голая площадка для отладки физики. Пол строим процедурно, а НЕ через
    # world.scene.add_default_ground_plane(): тот тянет ассет с сервера,
    # и сцена могла бы повиснуть на сети. GroundPlane внутри зовёт
    # PhysicsSchemaTools.addGroundPlane — чистая геометрия, без загрузок.
    GroundPlane(prim_path='/World/GroundPlane', z_position=0.0)
    light = UsdLux.DistantLight.Define(stage, '/World/DistantLight')
    light.CreateIntensityAttr(3000.0)
    print('[tb3_sim] окружение: голая площадка. Текстуры нет — '
          'визуальная одометрия работать НЕ будет', flush=True)
else:
    from isaacsim.core.utils.nucleus import get_assets_root_path  # noqa: E402

    _root = get_assets_root_path()
    if _root is None:
        raise SystemExit('не удалось получить корень ассетов: нет сети '
                         'или сервер недоступен. Для отладки физики без '
                         'окружения: --env none')
    # Локальная копия имеет приоритет. Без неё Isaac тянет сцену с сервера
    # NVIDIA во время загрузки, файл за файлом: офис это 706 МБ, из них 698 МБ
    # текстуры, и первый запуск выглядит как зависание — между "Startup
    # Complete" и появлением робота проходят минуты. Скачать заранее:
    #     ./isaac/fetch_env.py office
    _rel = ENVIRONMENTS[args.env]
    _local = HERE / 'assets' / 'environments' / _rel.split('Environments/')[1]
    if _local.is_file():
        _env_usd = str(_local)
        print(f'[tb3_sim] окружение (локально): {_env_usd}', flush=True)
    else:
        _env_usd = f'{_root}/{_rel}'
        print(f'[tb3_sim] окружение (с сервера): {_env_usd}', flush=True)
        print('[tb3_sim] тянется по сети — это минуты, не зависание. '
              'Скачать заранее: ./isaac/fetch_env.py ' + args.env, flush=True)
    add_reference_to_stage(usd_path=_env_usd, prim_path='/World/Environment')
    # Своего света НЕ добавляем: у готовых окружений он есть, и лишний
    # источник только пересветил бы сцену. Если картинка окажется тёмной,
    # это видно по среднему уровню кадра.

add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_PRIM)

# Точку спавна ставим на ССЫЛАЮЩЕМСЯ приме через USD напрямую: так не зависим
# от имён обёрток, которые от версии к версии переезжают. ClearXformOpOrder
# нужен, чтобы не наслоить второй translate поверх пришедшего из ассета.
_xf = UsdGeom.Xformable(stage.GetPrimAtPath(ROBOT_PRIM))
_xf.ClearXformOpOrder()
_xf.AddTranslateOp().Set(Gf.Vec3d(args.x, args.y, 0.02))

# Корень артикуляции ИЩЕМ, а не задаём константой, и вот почему.
# Импортёр URDF вешает ArticulationRootAPI не на верхний прим ассета,
# а на базовый линк — здесь это .../base_footprint. Если подсунуть
# IsaacArticulationController путь верхнего прима, он не найдёт артикуляцию
# и МОЛЧА ничего не сделает: ни ошибки в логе, ни исключения — робот просто
# стоит, будто команда не дошла. Именно на это я наступил с первой попытки,
# причём подписка на /cmd_vel при этом была живой, что сбивает с толку.
ARTICULATION_ROOT = None
for _p in stage.GetPrimAtPath(ROBOT_PRIM).GetChildren() + [stage.GetPrimAtPath(ROBOT_PRIM)]:
    if _p.HasAPI(UsdPhysics.ArticulationRootAPI):
        ARTICULATION_ROOT = str(_p.GetPath())
        break
if ARTICULATION_ROOT is None:
    for _p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
        if _p.HasAPI(UsdPhysics.ArticulationRootAPI):
            ARTICULATION_ROOT = str(_p.GetPath())
            break
if ARTICULATION_ROOT is None:
    raise SystemExit(f'не найден ArticulationRootAPI под {ROBOT_PRIM}')

robot = world.scene.add(Robot(prim_path=ARTICULATION_ROOT, name='tb3'))

# ── Камера ────────────────────────────────────────────────────────────────
# Прим камеры цепляем к camera_rgb_optical_frame — тому самому фрейму, ради
# которого при импорте пришлось сохранять неподвижные суставы.
_optical = None
for _p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
    if _p.GetName() == CAMERA_FRAME:
        _optical = str(_p.GetPath())
        break
if _optical is None:
    raise SystemExit(f'не найден {CAMERA_FRAME} под {ROBOT_PRIM}')

CAMERA_PATH = f'{_optical}/d435_rgb'
_cam = UsdGeom.Camera.Define(stage, CAMERA_PATH)

# ПОВОРОТ НА 180 ГРАДУСОВ ВОКРУГ X — не косметика, а перевод конвенций.
# USD-камера смотрит вдоль -Z, вверх у неё +Y. Оптический фрейм ROS задан
# иначе: +Z вперёд, +Y вниз. Поворот на pi вокруг X переводит одно в другое
# (X->X, Y->-Y, Z->-Z), и тогда -Z камеры совпадает с +Z оптического фрейма.
#
# Ошибка здесь НИЧЕГО не роняет: RTAB-Map примет перевёрнутый кадр и зеркальное
# облако и будет строить карту наизнанку. Проверять только замером — поставить
# перед роботом заметный объект и убедиться, что в облаке он спереди.
_cam.AddOrientOp().Set(Gf.Quatf(0.0, 1.0, 0.0, 0.0))
_cam.CreateFocalLengthAttr().Set(CAMERA_FOCAL_LENGTH)
_cam.CreateHorizontalApertureAttr().Set(CAMERA_H_APERTURE)
_cam.CreateVerticalApertureAttr().Set(CAMERA_V_APERTURE)
_cam.CreateClippingRangeAttr().Set(Gf.Vec2f(*CAMERA_CLIP))

# Render product — то, через что узлы моста получают кадры. Он же и есть
# основная нагрузка на GPU, поэтому разрешение ровно рабочее, не больше.
import omni.replicator.core as rep  # noqa: E402

_rp = rep.create.render_product(CAMERA_PATH, (CAMERA_WIDTH, CAMERA_HEIGHT))
RENDER_PRODUCT = _rp.path if hasattr(_rp, 'path') else str(_rp)

# ── Про встроенную модель стереодатчика Isaac ─────────────────────────────
# Isaac умеет генерировать глубину настоящим стереоконвейером: схема
# OmniSensorDepthSensorSingleViewAPI на render product даёт базу, диспаратность,
# шум и порог уверенности (см. omni:rtx:post:depthSensor:*). Я это пробовал:
# схема применяется, все атрибуты принимаются, параметры D435 ложатся ровно
# (база 50 мм, фокус 612.31 пикс, maxDisparityPixel 110 -> минимальная
# дальность 0.278 м при паспортных 0.28).
#
# НО до ROS её результат не доходит. Узел ROS2CameraHelper с type='depth'
# жёстко привязан к переменной рендера DistanceToImagePlane, а стереодатчик
# пишет в отдельный аннотатор DepthSensorDistance. Список допустимых значений
# type закрыт (allowedTokens), подменить нельзя. Проверено замером: прогон
# с включённой схемой и прогон с --ideal-depth дают побитово одинаковую
# глубину — 65.6% валидных пикселей и та же шероховатость по всем полосам
# дальности.
#
# Поэтому ту же физику считает узел в контейнере, scripts/depth_noise.py:
# шум задаётся в диспаратности (отсюда рост ошибки как z^2), дыры ставятся
# по нехватке текстуры и по границам дальности. Параметры при этом на виду
# и воспроизводимы по зерну, а не спрятаны в рендере.
#
# Сцена публикует ИДЕАЛЬНУЮ глубину в /camera/depth/ideal, узел выдаёт
# /camera/depth/image_raw — ровно та же схема, что у инерциалки.

# ── IMU ───────────────────────────────────────────────────────────────────
# Датчик вешаем на imu_link — тот самый фрейм, что сохранился благодаря
# merge_fixed_joints=False и которому при импорте пришлось чинить вырожденную
# инерцию. Прим датчика обязан быть потомком линка с RigidBodyAPI.
enable_extension('isaacsim.sensors.physics')
simulation_app.update()
from isaacsim.sensors.physics import IMUSensor  # noqa: E402

_imu_link = None
for _p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM)):
    if _p.GetName() == IMU_FRAME:
        _imu_link = str(_p.GetPath())
        break
if _imu_link is None:
    raise SystemExit(f'не найден {IMU_FRAME} под {ROBOT_PRIM}')

IMU_PRIM = f'{_imu_link}/imu_sensor'
IMUSensor(prim_path=IMU_PRIM, name='tb3_imu')

# ── Граф ──────────────────────────────────────────────────────────────────
# Три ветки от одного тика: часы, приём команды, отдача её в привод.
#
# Про типы портов, на которых легко ошибиться: ROS2SubscribeTwist отдаёт
# linearVelocity и angularVelocity как vectord[3], а DifferentialController
# ждёт double. Напрямую не соединить — между ними BreakVector3, из которого
# берём X линейной скорости и Z угловой. Это дифдрайв: боковая составляющая
# и крен с тангажом ему не нужны.
keys = og.Controller.Keys
og.Controller.edit(
    {'graph_path': '/ActionGraph', 'evaluator_name': 'execution'},
    {
        keys.CREATE_NODES: [
            ('OnTick', 'omni.graph.action.OnPlaybackTick'),
            ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ('SubscribeTwist', 'isaacsim.ros2.bridge.ROS2SubscribeTwist'),
            ('BreakLinear', 'omni.graph.nodes.BreakVector3'),
            ('BreakAngular', 'omni.graph.nodes.BreakVector3'),
            ('DiffController', 'isaacsim.robot.wheeled_robots.DifferentialController'),
            ('ArtController', 'isaacsim.core.nodes.IsaacArticulationController'),
            ('CameraRGB', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
            ('CameraDepth', 'isaacsim.ros2.bridge.ROS2CameraHelper'),
            ('CameraInfo', 'isaacsim.ros2.bridge.ROS2CameraInfoHelper'),
            ('ComputeGT', 'isaacsim.core.nodes.IsaacComputeOdometry'),
            ('PublishGT', 'isaacsim.ros2.bridge.ROS2PublishOdometry'),
        ],
        keys.CONNECT: [
            ('OnTick.outputs:tick', 'PublishClock.inputs:execIn'),
            ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),

            ('OnTick.outputs:tick', 'SubscribeTwist.inputs:execIn'),
            ('SubscribeTwist.outputs:linearVelocity', 'BreakLinear.inputs:tuple'),
            ('SubscribeTwist.outputs:angularVelocity', 'BreakAngular.inputs:tuple'),

            ('OnTick.outputs:tick', 'DiffController.inputs:execIn'),
            ('BreakLinear.outputs:x', 'DiffController.inputs:linearVelocity'),
            ('BreakAngular.outputs:z', 'DiffController.inputs:angularVelocity'),

            ('OnTick.outputs:tick', 'ArtController.inputs:execIn'),
            ('DiffController.outputs:velocityCommand',
             'ArtController.inputs:velocityCommand'),

            ('OnTick.outputs:tick', 'CameraRGB.inputs:execIn'),
            ('OnTick.outputs:tick', 'CameraDepth.inputs:execIn'),
            ('OnTick.outputs:tick', 'CameraInfo.inputs:execIn'),

            ('OnTick.outputs:tick', 'ComputeGT.inputs:execIn'),
            ('ComputeGT.outputs:execOut', 'PublishGT.inputs:execIn'),
            ('ComputeGT.outputs:position', 'PublishGT.inputs:position'),
            ('ComputeGT.outputs:orientation', 'PublishGT.inputs:orientation'),
            ('ComputeGT.outputs:linearVelocity', 'PublishGT.inputs:linearVelocity'),
            ('ComputeGT.outputs:angularVelocity', 'PublishGT.inputs:angularVelocity'),
            ('ReadSimTime.outputs:simulationTime', 'PublishGT.inputs:timeStamp'),
        ],
        keys.SET_VALUES: [
            ('PublishClock.inputs:topicName', 'clock'),
            ('SubscribeTwist.inputs:topicName', 'cmd_vel'),
            ('DiffController.inputs:wheelDistance', WHEEL_DISTANCE),
            ('DiffController.inputs:wheelRadius', WHEEL_RADIUS),
            ('DiffController.inputs:maxLinearSpeed', MAX_LINEAR),
            ('DiffController.inputs:maxAngularSpeed', MAX_ANGULAR),
            ('DiffController.inputs:dt', PHYSICS_DT),
            # robotPath строкой, а не targetPrim: он имеет приоритет
            # и задаётся обычным значением, тогда как targetPrim —
            # это связь USD, её через SET_VALUES не выставить.
            ('ArtController.inputs:robotPath', ARTICULATION_ROOT),
            ('ArtController.inputs:jointNames',
             ['wheel_left_joint', 'wheel_right_joint']),

            ('CameraRGB.inputs:renderProductPath', RENDER_PRODUCT),
            ('CameraRGB.inputs:type', 'rgb'),
            ('CameraRGB.inputs:topicName', RGB_TOPIC),
            ('CameraRGB.inputs:frameId', CAMERA_FRAME),

            # depth, а НЕ depth_pcl: облако точек из Isaac не публикуем.
            # На мосте Gazebo облако весило 9.77 МБ на сообщение и роняло
            # частоту ВСЕХ топиков до 4-5 Гц. Считает его point_cloud_xyz
            # в контейнере, с прореживанием и отсечкой по дальности.
            ('CameraDepth.inputs:renderProductPath', RENDER_PRODUCT),
            ('CameraDepth.inputs:type', 'depth'),
            ('CameraDepth.inputs:topicName', DEPTH_TOPIC),
            ('CameraDepth.inputs:frameId', CAMERA_FRAME),

            ('CameraInfo.inputs:renderProductPath', RENDER_PRODUCT),
            ('CameraInfo.inputs:topicName', INFO_TOPIC),
            ('CameraInfo.inputs:frameId', CAMERA_FRAME),

            # Прим шасси — значением типа Sdf.Path: inputs:chassisPrim
            # объявлен как target, обычной строкой его не задать. Если
            # не задать вовсе, узел ругается "No chassis (target) prim found
            # at path" с нечитаемым путём в кавычках, топик не появляется,
            # и больше нигде об этом не сообщается.
            ('ComputeGT.inputs:chassisPrim', Sdf.Path(ARTICULATION_ROOT)),
            ('PublishGT.inputs:topicName', GT_TOPIC),
            ('PublishGT.inputs:odomFrameId', GT_ODOM_FRAME),
            ('PublishGT.inputs:chassisFrameId', BASE_FRAME_ID),
        ],
    },
)

# ── Граф IMU: отдельный и ПО ЗАПРОСУ ──────────────────────────────────────
# OnPhysicsStep нельзя класть в общий граф. Isaac ругается прямым текстом:
# "Physics OnSimulationStep node detected in a non on-demand Graph. Node will
# only trigger events if the parent Graph is set to compute on-demand" —
# и узел молча не срабатывает, топик не появляется вовсе.
#
# Решает это НЕ evaluator_name, как можно подумать, а отдельный ключ
# pipeline_stage; evaluator_name при этом не задаётся вообще. Образец —
# штатный тест самого Isaac, isaacsim/core/nodes/tests/test_physics_step.py.
#
# Отсюда же и своё чтение времени симуляции: узел из чужого графа не подключить.
og.Controller.edit(
    {'graph_path': '/ImuGraph',
     'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND},
    {
        keys.CREATE_NODES: [
            ('OnPhysics', 'isaacsim.core.nodes.OnPhysicsStep'),
            ('ImuSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ('ReadIMU', 'isaacsim.sensors.physics.IsaacReadIMU'),
            ('PublishImu', 'isaacsim.ros2.bridge.ROS2PublishImu'),
        ],
        keys.CONNECT: [
            ('OnPhysics.outputs:step', 'ReadIMU.inputs:execIn'),
            ('ReadIMU.outputs:execOut', 'PublishImu.inputs:execIn'),
            ('ReadIMU.outputs:angVel', 'PublishImu.inputs:angularVelocity'),
            ('ReadIMU.outputs:linAcc', 'PublishImu.inputs:linearAcceleration'),
            ('ImuSimTime.outputs:simulationTime', 'PublishImu.inputs:timeStamp'),
        ],
        keys.SET_VALUES: [
            # Прим датчика передаётся значением типа Sdf.Path — так это
            # делает штатный тест Isaac. Через обычную строку не выйдет:
            # inputs:imuPrim объявлен как target.
            ('ReadIMU.inputs:imuPrim', Sdf.Path(IMU_PRIM)),
            ('PublishImu.inputs:topicName', IMU_TOPIC),
            ('PublishImu.inputs:frameId', IMU_FRAME),
            ('PublishImu.inputs:publishAngularVelocity', True),
            ('PublishImu.inputs:publishLinearAcceleration', True),
            # Ориентацию НЕ публикуем, хотя IsaacReadIMU её отдаёт. Это
            # истинное значение из симулятора, оракул: на реальном роботе его
            # взяться неоткуда, там кватернион выводит imu_filter_madgwick
            # из гироскопа и акселерометра. Приняв подарок, мы сделали бы
            # симулятор легче реальности — против смысла этапа 3.
            ('PublishImu.inputs:publishOrientation', False),
        ],
    },
)

world.reset()
omni.timeline.get_timeline_interface().play()

# flush=True на КАЖДОМ print: Isaac идёт с --/app/fastShutdown=True, и при
# закрытии процесс завершается жёстко, не сбрасывая буферы stdio.
print(f'[tb3_sim] робот в ({args.x}, {args.y}), база {WHEEL_DISTANCE} м, '
      f'колесо {WHEEL_RADIUS} м', flush=True)
print(f'[tb3_sim] корень артикуляции: {ARTICULATION_ROOT}', flush=True)
print('[tb3_sim] публикую /clock, слушаю /cmd_vel (geometry_msgs/Twist)',
      flush=True)
print(f'[tb3_sim] камера {CAMERA_WIDTH}x{CAMERA_HEIGHT} на {CAMERA_PATH}',
      flush=True)
print(f'[tb3_sim] топики: {RGB_TOPIC}, {DEPTH_TOPIC}, {INFO_TOPIC}',
      flush=True)
print(f'[tb3_sim] IMU {IMU_TOPIC} на {IMU_PRIM} (идеальный, шум в контейнере)',
      flush=True)
print(f'[tb3_sim] истинная поза: {GT_TOPIC} (для ATE, в /tf НЕ идёт)',
      flush=True)

step = 0
try:
    while simulation_app.is_running():
        world.step(render=True)
        step += 1
        if step % 180 == 0:
            pos, _ = robot.get_world_pose()
            lin = robot.get_linear_velocity()
            print(f'[tb3_sim] t={world.current_time:6.1f} c  '
                  f'поза=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})  '
                  f'|v|={np.linalg.norm(lin[:2]):.3f} м/с', flush=True)
            if args.debug:
                def _og(path):
                    try:
                        return og.Controller.get(og.Controller.attribute(path))
                    except Exception as exc:
                        return '<%s: %s>' % (type(exc).__name__, exc)
                for label, path in [
                        ('Twist.linear ', 'SubscribeTwist.outputs:linearVelocity'),
                        ('Twist.angular', 'SubscribeTwist.outputs:angularVelocity'),
                        ('Break.x      ', 'BreakLinear.outputs:x'),
                        ('Diff.linearIn', 'DiffController.inputs:linearVelocity'),
                        ('Diff.velCmd  ', 'DiffController.outputs:velocityCommand'),
                        ('Art.velCmd   ', 'ArtController.inputs:velocityCommand')]:
                    print('    %s = %s' % (label, _og('/ActionGraph/' + path)),
                          flush=True)
                print('    joint vel     = %s' % (robot.get_joint_velocities(),),
                      flush=True)
except KeyboardInterrupt:
    pass
finally:
    omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
