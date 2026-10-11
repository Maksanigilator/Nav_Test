#!/usr/bin/env python3
"""Импорт TurtleBot3 из URDF в USD для Isaac Sim.

Разовый шаг: получить из описания робота ассет, который дальше грузит сцена.
URDF берётся не откуда попало, а из образа nav2:jazzy — тот же самый, по которому
robot_state_publisher в контейнере строит дерево TF. Достаёт его
isaac/export_robot.sh, запускать надо сначала его.

Почему робот именно из URDF, а не из ассетов Isaac. Критерий этапа 3 по PLAN.md —
сравнить ATE с этапом 2 и увидеть, сколько стоит реализм. Если вместе с картинкой
сменить ещё и робота, разрыв будет мерить смесь двух причин. Поэтому робот здесь
контроль и остаётся тем же, а переменной становится ОКРУЖЕНИЕ — вот его и надо
брать из ассетов Isaac, ради текстур, бликов и честного шума глубины.

    source isaac/env.sh
    "$ISAAC_PYTHON" isaac/import_robot.py

Результат: isaac/assets/turtlebot3_waffle/turtlebot3_waffle.usd
"""
import argparse
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_URDF = HERE / 'assets' / 'turtlebot3_waffle' / 'turtlebot3_waffle.urdf'
DEFAULT_USD = HERE / 'assets' / 'turtlebot3_waffle' / 'turtlebot3_waffle.usd'

# argparse до SimulationApp: тот поднимает Kit и разбирает argv сам.
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf', default=str(DEFAULT_URDF))
parser.add_argument('--usd', default=str(DEFAULT_USD))
parser.add_argument('--gui', action='store_true')
args = parser.parse_args()

if not pathlib.Path(args.urdf).is_file():
    raise SystemExit(f'нет файла {args.urdf}\nсначала запусти isaac/export_robot.sh')

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({'headless': not args.gui})

import omni.kit.commands  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension('isaacsim.asset.importer.urdf')
simulation_app.update()

from isaacsim.asset.importer.urdf import _urdf  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

status, cfg = omni.kit.commands.execute('URDFCreateImportConfig')

# merge_fixed_joints=False — КРИТИЧНО и не для красоты. Слияние неподвижных
# суставов схлопнуло бы линки, соединённые fixed-суставами, в родителя, а именно
# так в URDF TurtleBot3 подвешены camera_link -> camera_rgb_frame ->
# camera_rgb_optical_frame и imu_link. Ровно к этим фреймам нам и надо цеплять
# камеру и IMU, причём в тех же позах, что в этапе 2. Схлопнув их, мы потеряли
# бы точку крепления и сравнимость с Gazebo.
cfg.merge_fixed_joints = False

# База свободна: робот должен ездить. True нужен манипуляторам, привинченным
# к полу, — с ним TurtleBot3 просто повис бы в воздухе.
cfg.fix_base = False

cfg.make_default_prim = True
cfg.import_inertia_tensor = True

# Физическую сцену заводит World в сценарии сцены. Если запечь её ещё и внутрь
# ассета робота, на стадии окажется две, и параметры будут спорить.
cfg.create_physics_scene = False

# URDF в метрах, стадию тоже держим в метрах (stage_units_in_meters=1.0).
cfg.distance_scale = 1.0

# Колёса дифдрайва должны слушаться заданий по СКОРОСТИ: их крутит связка
# DifferentialController -> IsaacArticulationController, которая пишет именно
# velocity targets. Имя члена перечисления зависит от версии, поэтому ищем его,
# а не зашиваем константу.
drive_types = [n for n in dir(_urdf.UrdfJointTargetType) if n.startswith('JOINT_DRIVE_')]
print(f'[import_robot] доступные типы привода: {drive_types}', flush=True)
if 'JOINT_DRIVE_VELOCITY' in drive_types:
    cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
    print('[import_robot] привод суставов: VELOCITY', flush=True)
else:
    print('[import_robot] VELOCITY не найден, оставляю значение по умолчанию',
          flush=True)

print(f'[import_robot] импортирую {args.urdf}', flush=True)

# dest_path НЕ используем, и это важно. С ним импортёр раскладывает результат
# на четыре слоя (base/physics/robot/sensor), и ссылки между ними ломаются:
# в base появляются reference на @..._physics.usd@</visuals/...> и
# @...@</collisions/...>, которых в том файле нет. Снаружи всё выглядит
# успешным — команда возвращает (True, '/turtlebot3_waffle'), суставы, массы
# и приводы на месте, — но Xform-ы visuals и collisions остаются ПУСТЫМИ.
# Робот получается вообще без коллайдеров: сцепления с полом нет, колёса
# крутятся вхолостую под приводом, и он стоит на месте при полностью
# исправном графе управления. Диагноз занимает время именно потому, что
# ошибок нигде не печатается.
# Поэтому импортируем в текущую стадию и выгружаем её ОДНИМ плоским файлом.
result = omni.kit.commands.execute(
    'URDFParseAndImportFile',
    urdf_path=args.urdf,
    import_config=cfg,
)
print(f'[import_robot] результат: {result}', flush=True)

simulation_app.update()

# ── Починка коллайдеров ───────────────────────────────────────────────────
# Импортёр этой версии складывает геометрию столкновений и отрисовки
# в ОТДЕЛЬНЫЕ ветки в корне стадии (/colliders/<линк>/mesh_0/... и
# /visuals/<линк>/...), а под самими линками оставляет пустые Xform-ы
# collisions и visuals. Ссылки, которые должны были их связать, не
# складываются — ни в многофайловой раскладке через dest_path, ни при
# импорте в текущую стадию.
#
# Для PhysX это означает, что робота физически НЕ СУЩЕСТВУЕТ: коллайдер
# учитывается только если он потомок прима с RigidBodyAPI, а лежащий
# в /colliders — не потомок. Внешне всё исправно: импорт возвращает True,
# суставы, массы и приводы на месте, колёса послушно крутятся с заданной
# скоростью. Но сцепления с полом нет, и робот просто висит, не двигаясь.
# Ошибок при этом не печатается нигде — отсюда и цена диагностики.
#
# Переносим геометрию на место сами. Трансформы у mesh_0 локальные
# относительно линка (проверено: у base_link стоит ровно origin из URDF),
# поэтому после переподчинения они встают правильно.
from pxr import Sdf  # noqa: E402

stage = omni.usd.get_context().get_stage()
robot_root = str(stage.GetDefaultPrim().GetPath())
layer = stage.GetRootLayer()

moved = 0
for scope, child_name in (('/colliders', 'collisions'), ('/visuals', 'visuals')):
    scope_prim = stage.GetPrimAtPath(scope)
    if not scope_prim.IsValid():
        continue
    for link_prim in scope_prim.GetChildren():
        dst_parent = f'{robot_root}/{link_prim.GetName()}/{child_name}'
        if not stage.GetPrimAtPath(dst_parent).IsValid():
            print(f'[import_robot] пропускаю {link_prim.GetName()}: '
                  f'нет {dst_parent}', flush=True)
            continue
        for geom in link_prim.GetChildren():
            dst = Sdf.Path(f'{dst_parent}/{geom.GetName()}')
            if Sdf.CopySpec(layer, geom.GetPath(), layer, dst):
                moved += 1
    # Исходную ветку убираем: иначе та же геометрия останется на сцене
    # вторым экземпляром, видимым и висящим в воздухе у начала координат.
    stage.RemovePrim(scope)

print(f'[import_robot] перенесено геометрий под линки: {moved}', flush=True)

# ── Починка вырожденных тел ───────────────────────────────────────────────
# Каждый линк, у которого в URDF нет <inertial>, импортёр превращает в тело
# с массой 0, центром масс (-inf, -inf, -inf) и НУЛЕВЫМ кватернионом главных
# осей (0,0,0,0) — то есть невалидным. У TurtleBot3 таких семь: base_footprint,
# imu_link и пять камерных фреймов. Это чистые фреймы, и в URDF так и задумано,
# но мы СОХРАНЯЕМ их через merge_fixed_joints=False — иначе не к чему цеплять
# камеру и инерциалку.
#
# Расплата: семь вырожденных тел в одной артикуляции. PhysX считает по ним
# мусор, и робот опрокидывается на нос за первую секунду — характерный признак
# кватернион вида [0.72, 0, 0.69, 0], то есть тангаж под 90 градусов.
# Симптом при этом выглядит как «не едет»: колёса исправно крутятся
# с заданной скоростью, а робот лежит.
#
# Даём им крошечные, но ВАЛИДНЫЕ свойства. Суммарно 7 грамм на робота
# массой около 1.6 кг — это 0.4%, на динамику не влияет.
from pxr import Gf  # noqa: E402

FRAME_MASS = 1e-3       # кг
FRAME_INERTIA = 1e-6    # кг*м^2

fixed = []
for prim in stage.Traverse():
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        continue
    mapi = UsdPhysics.MassAPI(prim)
    mass = mapi.GetMassAttr().Get()
    com = mapi.GetCenterOfMassAttr().Get()
    inertia = mapi.GetDiagonalInertiaAttr().Get()
    degenerate = (
        mass is None or mass <= 0.0
        or com is None or any(abs(c) > 1e6 for c in com)
        or inertia is None or any(i <= 0.0 for i in inertia))
    if not degenerate:
        continue
    mapi.CreateMassAttr().Set(FRAME_MASS)
    mapi.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    mapi.CreateDiagonalInertiaAttr().Set(
        Gf.Vec3f(FRAME_INERTIA, FRAME_INERTIA, FRAME_INERTIA))
    # Кватернион главных осей тоже приводим к единичному: импортёр оставляет
    # (0,0,0,0), а это не поворот вообще, и инерция по нему не считается.
    mapi.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    fixed.append(prim.GetName())

print(f'[import_robot] выправлено вырожденных тел: {len(fixed)} -> {fixed}',
      flush=True)
simulation_app.update()

omni.usd.get_context().get_stage().Export(args.usd)
print(f'[import_robot] записан {args.usd}', flush=True)

# Проверка идёт по ПЕРЕЧИТАННОМУ файлу, а не по стадии в памяти: проверять
# надо именно то, что достанется сцене.
#
# ВНИМАНИЕ: flush=True обязателен на КАЖДОМ print. Isaac стартует с ключом
# --/app/fastShutdown=True, поэтому simulation_app.close() завершает процесс
# жёстко, не сбрасывая буферы stdio. Без flush вывод просто исчезает, причём
# код отрабатывает полностью и возвращает 0 — выглядит так, будто блок
# не выполнялся вовсе.
omni.usd.get_context().open_stage(args.usd)
simulation_app.update()
stage = omni.usd.get_context().get_stage()

joints = [(p.GetName(), str(p.GetTypeName())) for p in stage.Traverse()
          if 'Joint' in str(p.GetTypeName())]
names = {p.GetName() for p in stage.Traverse()}

print(f'\n[import_robot] примов всего: {len(list(stage.Traverse()))}', flush=True)
print(f'[import_robot] суставов: {len(joints)}', flush=True)
for n, t in joints:
    print(f'    {n:26s} {t}', flush=True)

# Коллайдеры — то, на чём импорт уже один раз молча развалился.
# Без них робот физически не существует для пола.
print('\n[import_robot] коллайдеры по линкам:', flush=True)
expect_colliders = ['base_link', 'wheel_left_link', 'wheel_right_link',
                    'caster_back_left_link', 'caster_back_right_link']
bad = []
for link in expect_colliders:
    prim = next((p for p in stage.Traverse()
                 if p.GetName() == link and p.GetTypeName() == 'Xform'), None)
    found = []
    if prim is not None:
        found = [c.GetName() for c in Usd.PrimRange(prim)
                 if c.HasAPI(UsdPhysics.CollisionAPI)]
    if not found:
        bad.append(link)
    print(f'    {"OK " if found else "НЕТ"} {link:24s} {found}', flush=True)

required = ['base_footprint', 'base_link', 'camera_rgb_optical_frame',
            'imu_link', 'wheel_left_joint', 'wheel_right_joint']
missing = [r for r in required if r not in names]
print('\n[import_robot] обязательные имена:', flush=True)
for r in required:
    print(f'    {"OK " if r in names else "НЕТ"} {r}', flush=True)

if missing or bad:
    print(f'\n[import_robot] ПРОБЛЕМЫ: имён нет {missing}, '
          f'без коллайдеров {bad}', flush=True)
else:
    print('\n[import_robot] робот собран корректно', flush=True)

simulation_app.close()
