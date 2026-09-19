"""Робот в Gazebo с меканум-контроллером: проверка, что он вообще едет.

    ros2 launch maze_nav drive_robot.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # в другом терминале

Без карты, без SLAM и без Nav2 — только физика и привод. Смысл запуска в
том, чтобы увидеть три движения, которые отличают меканум от обычной
тележки: вперёд, боком и разворот на месте.

Что здесь чьё: gz_ros2_control, controller_manager, mecanum_drive_controller
и joint_state_broadcaster — штатные пакеты ROS 2. Своего кода тут два
файла: описание робота (urdf/agro_robot.urdf.xacro, сгенерировано из CAD
скриптом scripts/gen_robot_urdf.py) и переходник scripts/twist_to_stamped.py.

Одометрия по колёсам публикуется контроллером в
/mecanum_drive_controller/odometry, но TF из неё НЕ идёт: enable_odom_tf
выключен в params/agro_controllers.yaml. У этого робота меканум-колёса
проскальзывают, а привод не откалиброван, поэтому odom->base_link даёт
RTAB-Map по картинке. Колёсную одометрию оставляем как отдельный источник —
сравнивать с визуальной и, позже, сливать в robot_localization.
"""
import math
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


# Где ставить робота в каждом мире. Без этого приходится помнить, что для
# сцены с рельсами нужно x:=-1 z:=0.13, иначе робот появляется у кромки
# настила и наполовину мимо него. Значения можно перебить аргументами.
SPAWN = {
    'rails.world': ('-1.0', '0', '0.13', '0'),   # середина настила 120 мм
    'room.world':  ('-1.0', '0', '0.13', '0'),   # тот же настил в комнате
}
SPAWN_DEFAULT = ('0', '0', '0.01', '0')

# Отдельная точка для отладки детектора края: робот стоит на левом листе
# настила и смотрит ВБОК, так что рельсы в кадр не попадают вовсе. Нужна,
# чтобы отличать дрожание самой кромки от помех, которые вносят рельсы.
# Настил тянется по Y до +2.0, камера видит землю с 0.9 м перед роботом.
# Ставим в y=0.9, чтобы кромка оказалась в 1.1 м — с запасом внутри обзора,
# а не в мёртвой зоне, как было при y=1.4.
SPAWN_EDGE_ONLY = ('-1.0', '0.9', '0.13', '1.5708')



def random_deck_pose(world, seed, robot_r=0.775, cam_reach=2.4):
    """Случайная поза на настиле, пригодная для старта заезда.

    Условия ставил не я, а постановка задачи: робот стоит на настиле в
    произвольной позе, но так, чтобы он мог безопасно прокрутиться на месте
    и чтобы кромка попадала в поле зрения камеры.

    Поэтому центр робота отодвигается от каждой кромки на ОПИСАННЫЙ радиус
    0.775 м. Не на вписанный: при развороте корпус заметает круг именно по
    описанному радиусу, и меньший отступ означал бы, что робот сваливается с
    настила посреди осмотра.

    Геометрия настила читается из самого мира, чтобы не держать второе
    описание тех же размеров.
    """
    import random
    import re
    m = re.search(r'<model name="deck">(.*?)</model>', open(world).read(), re.S)
    if not m:
        return None
    pose = [float(v) for v in
            re.search(r'<pose>([^<]+)</pose>', m.group(1)).group(1).split()]
    size = [float(v) for v in
            re.search(r'<size>([^<]+)</size>', m.group(1)).group(1).split()]
    x0, x1 = pose[0] - size[0] / 2 + robot_r, pose[0] + size[0] / 2 - robot_r
    y0, y1 = pose[1] - size[1] / 2 + robot_r, pose[1] + size[1] / 2 - robot_r
    if x1 <= x0 or y1 <= y0:
        return None
    rnd = random.Random(seed)
    for _ in range(200):
        x, y = rnd.uniform(x0, x1), rnd.uniform(y0, y1)
        yaw = rnd.uniform(-math.pi, math.pi)
        # Кромка должна быть видна. Камера смотрит вперёд и вниз, поле
        # зрения 87°, достаёт примерно от 0.6 до 2.4 м. Проверяем весь
        # периметр настила точками через 10 см, а не четыре ближайшие
        # проекции: с четырьмя проверка получалась узкой и сбивала все
        # позы в один угол настила вместо равномерного разброса.
        seen = False
        per = []
        n = int(size[0] / 0.1) + 1
        for i in range(n):
            ex = pose[0] - size[0] / 2 + i * size[0] / (n - 1)
            per += [(ex, pose[1] - size[1] / 2), (ex, pose[1] + size[1] / 2)]
        n = int(size[1] / 0.1) + 1
        for i in range(n):
            ey = pose[1] - size[1] / 2 + i * size[1] / (n - 1)
            per += [(pose[0] - size[0] / 2, ey), (pose[0] + size[0] / 2, ey)]
        for ex, ey in per:
            d = math.hypot(ex - x, ey - y)
            if not (0.6 < d < cam_reach):
                continue
            if abs(math.atan2(math.sin(math.atan2(ey - y, ex - x) - yaw),
                              math.cos(math.atan2(ey - y, ex - x) - yaw))) < 0.72:
                seen = True
                break
        if seen:
            return f'{x:.3f}', f'{y:.3f}', f'{pose[2] + size[2] / 2 + 0.01:.3f}', \
                   f'{yaw:.4f}'
    return None


def launch_setup(context, *args, **kwargs):
    """Собирает узлы, когда значения аргументов уже известны.

    Через OpaqueFunction, а не напрямую, из-за командной строки Gazebo:
    ключ -s либо есть, либо его нет, а подставить «пустой аргумент» нельзя —
    gz воспримет пустую строку как имя мира и не найдёт его.
    """
    arg = lambda n: LaunchConfiguration(n).perform(context)
    pkg = get_package_share_directory('maze_nav')
    world_name = arg('world')
    world = os.path.join(pkg, 'worlds', world_name)
    headless = arg('gui') in ('false', 'False', '0')

    # 'auto' означает «возьми из таблицы для этого мира»
    if arg('edge_only') not in ('false', 'False', '0'):
        preset = SPAWN_EDGE_ONLY
    elif arg('random_spawn') not in ('false', 'False', '0'):
        seed = None if arg('seed') in ('', 'none') else int(arg('seed'))
        preset = random_deck_pose(world, seed) or SPAWN.get(world_name,
                                                            SPAWN_DEFAULT)
        print(f'[drive_robot] случайная поза на настиле: {preset}')
    else:
        preset = SPAWN.get(world_name, SPAWN_DEFAULT)
    spawn_xyz = [v if arg(k) != 'auto' else dflt
                 for k, v, dflt in (('x', arg('x'), preset[0]),
                                    ('y', arg('y'), preset[1]),
                                    ('z', arg('z'), preset[2]),
                                    ('yaw', arg('yaw'), preset[3]))]

    # Решаем заранее: от этого зависит, куда мост кладёт глубину.
    add_noise = arg('depth_noise') not in ('false', 'False', '0')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'agro_robot.urdf.xacro')]),
        value_type=str)

    # ═══ Gazebo ═══
    # Запускается НАПРЯМУЮ, а не через ros_gz_sim/gz_sim.launch.py.
    #
    # Тот launch собирает процесс с shell=True, то есть на самом деле поднимает
    # /bin/sh -c "ruby .../gz sim ...". Сигнал от launch достаётся оболочке,
    # а детям она его не передаёт. Дальше по журналу: launch ждёт 5 секунд,
    # не дожидается и шлёт SIGTERM, от которого умирает оболочка — а сервер
    # и окно остаются СИРОТАМИ. Launch отчитывается о завершении, окно
    # Gazebo продолжает висеть на экране.
    #
    # Без оболочки сигнал приходит самой обёртке gz, а у неё в совмещённом
    # режиме (сервер и окно одной командой) есть ловушки INT/TERM, гасящие
    # обоих детей. Замерено после исправления: всё закрывается меньше чем
    # за 3 секунды, без эскалации до SIGTERM.
    #
    # Разделять на `gz sim -s` и `gz sim -g` НЕЛЬЗЯ: в одиночных режимах
    # ruby зовёт C++ прямо через FFI, ловушек там нет вовсе.
    #
    # -r запускает мир сразу, без нажатия «play». -v 3 оставляет
    # предупреждения видимыми: молчащий Gazebo, который не загрузил плагин,
    # выглядит точно так же, как исправно работающий — именно на этом уровне
    # видна строка "Failed to load system plugin [gz_ros2_control-system]",
    # без которой робот просто стоит и не отзывается на команды.
    cmd = ['ruby', shutil.which('gz'), 'sim', '-r', '-v', '3']
    if headless:
        cmd.append('-s')
    cmd.append(world)

    # Два пути, которые Gazebo обязан знать, и оба легко потерять.
    #
    # SYSTEM_PLUGIN_PATH — где лежит libgz_ros2_control-system.so. Без него
    # симулятор поднимется как ни в чём не бывало, но controller_manager
    # внутри него не появится, и робот останется неподвижным. Берём из
    # LD_LIBRARY_PATH, как это делает ros_gz_sim.
    #
    # RESOURCE_PATH — где искать меши. Конвертер URDF->SDF переписывает
    # package://maze_nav/... в model://maze_nav/..., и дальше их ищет уже gz.
    # Без этого пути меши роликов молча не грузятся, единственная коллизия,
    # которая касается пола, исчезает, и робот едет на ребордах рельсовых
    # роликов: вперёд ползёт, вбок и вокруг оси — никак. В логе про это
    # всего одно предупреждение Wrn, ошибки нет.
    def joined(var, extra):
        return os.pathsep.join(filter(None, (os.environ.get(var, ''), extra)))

    gz = ExecuteProcess(
        name='gazebo', output='screen', cmd=cmd,
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': joined('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                                os.environ.get('LD_LIBRARY_PATH', '')),
            'GZ_SIM_RESOURCE_PATH': joined('GZ_SIM_RESOURCE_PATH',
                                           os.path.dirname(pkg)),
        },
        on_exit=[Shutdown(reason='Gazebo закрылся')])

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}])

    # Спавн берёт описание из топика, а не из файла: так в симулятор попадает
    # ровно то, что уже разобрал robot_state_publisher, и разойтись они не могут.
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description',
                            '-name', 'agro_robot',
                            '-x', spawn_xyz[0], '-y', spawn_xyz[1],
                            '-z', spawn_xyz[2], '-Y', spawn_xyz[3]])

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    # Камера. Облако точек НЕ мостим: кадр 848x480 это около 10 МБ на
    # сообщение и под 300 МБ/с — мерено на втором этапе, мост от этого
    # ложится. Облако при надобности считается у потребителя из глубины.
    camera = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  output='screen', parameters=[{'use_sim_time': True}],
                  arguments=[
                      '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                      '/camera/camera_info@sensor_msgs/msg/CameraInfo'
                      '[gz.msgs.CameraInfo',
                      '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                  remappings=[('/camera/image', '/camera/color/image_raw'),
                              ('/camera/depth_image',
                               '/camera/depth/ideal' if add_noise
                               else '/camera/depth/image_raw'),
                              ('/camera/camera_info', '/camera/color/camera_info'),
                              ('/imu', '/imu/data')])

    relay = Node(package='maze_nav', executable='twist_to_stamped.py',
                 output='screen',
                 parameters=[{'use_sim_time': True}],
                 remappings=[('in', '/cmd_vel'),
                             ('out', '/mecanum_drive_controller/reference')])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    output='screen',
                    arguments=[name, '--param-file',
                               os.path.join(pkg, 'params', 'agro_controllers.yaml'),
                               # Штатные 5 секунд не выдерживаются, когда
                               # симуляция идёт вполовину реального времени
                               # и рядом работают SLAM с шумом глубины.
                               # Спавнер падает, публикатор состояний не
                               # стартует, углов колёс нет — и RViz сваливает
                               # все колёсные звенья в одну точку под роботом.
                               # В Gazebo при этом всё верно: там физика
                               # знает углы сама, и расхождение видно только
                               # в RViz.
                               '--controller-manager-timeout', '60',
                               '--switch-timeout', '30'])

    jsb = spawner('joint_state_broadcaster')
    mec = spawner('mecanum_drive_controller')

    # Шум глубины. Симулятор отдаёт идеальную карту глубины, и на ней
    # видны вещи, которых настоящий D435 не разрешит никогда — например
    # миллиметровые щели между листами фанеры. Отлаживать перцепцию на
    # такой картинке значит обманывать себя: в симуляторе всё работает,
    # на железе разваливается. Узел добавляет шум В ДИСПАРАТНОСТИ, отчего
    # ошибка растёт как квадрат дальности, как у настоящей камеры.
    noise = [Node(package='maze_nav', executable='depth_noise.py',
                 output='screen',
                 parameters=[{'use_sim_time': True,
                              'input_topic': '/camera/depth/ideal',
                              'output_topic': '/camera/depth/image_raw',
                              'rgb_topic': '/camera/color/image_raw',
                              'info_topic': '/camera/color/camera_info'}])] \
        if add_noise else []

    # Контроль заезда: сравнивает колёсную одометрию с визуальной.
    # На рельсах меканум-колёса висят в воздухе, везут рельсовые ролики,
    # и передаточное число меняется ровно вдвое — это и есть признак.
    entry = Node(package='maze_nav', executable='rail_entry_monitor.py',
                 output='screen',
                 condition=IfCondition(LaunchConfiguration('entry_monitor')),
                 parameters=[{'use_sim_time': True}],
                 remappings=[('visual_odom', '/odom'),
                             ('odom_info', '/odom_info'),
                             ('cmd_vel', '/cmd_vel'),
                             ('state', '/rail_entry/state')])

    # Детектор обрыва. Публикует край платформы облаком точек НА ВЫСОТЕ,
    # то есть подделывает отрицательное препятствие под обычное — иначе
    # ни костмап, ни сетка RTAB-Map его не увидят вовсе.
    dropoff = Node(package='maze_nav', executable='dropoff_detector.py',
                   output='screen',
                   condition=IfCondition(LaunchConfiguration('dropoff')),
                   parameters=[{'use_sim_time': True}],
                   remappings=[('depth', '/camera/depth/image_raw'),
                               ('camera_info', '/camera/color/camera_info'),
                               ('dropoff', '/dropoff/points'),
                               ('grid_cloud', '/dropoff/grid_cloud'),
                               ('rails', '/rails/points'),
                               ('rails/mask', '/rails/mask')])

    # Идеальная имитация сегментации рельсов: маску даёт не модель, а
    # истинная геометрия из файла мира. Нужна, чтобы отлаживать логику
    # заезда, не дожидаясь настоящего распознавания. Позже эта нода
    # заменяется на модель CV, а потребитель маски не меняется.
    rail_oracle = Node(package='maze_nav', executable='rail_oracle.py',
                       output='screen',
                       condition=IfCondition(LaunchConfiguration('rail_oracle')),
                       parameters=[{'use_sim_time': True,
                                    'world_file': world}],
                       remappings=[('depth', '/camera/depth/image_raw'),
                                   ('camera_info',
                                    '/camera/color/camera_info'),
                                   ('odom', '/odom'),
                                   ('rails/mask', '/rails/mask')])

    # ═══ Nav2 ═══
    # Только навигация: ни AMCL, ни map_server не нужны, карту и связку
    # map->odom публикует RTAB-Map. Скорости уходят в /cmd_vel, тот же
    # топик, что и у ручного управления, — переходник до контроллера один.
    #
    # Цепочка скоростей у Nav2 длинная: контроллер шлёт в /cmd_vel_nav,
    # сглаживатель публикует /cmd_vel_smoothed, и последним стоит монитор
    # столкновений. Он же и переименовывает выход в /cmd_vel, который
    # слушает робот, — см. cmd_vel_out_topic в agro_nav2.yaml. Переименовать
    # топик снаружи нельзя: имя cmd_vel_smoothed принадлежит сразу двум
    # узлам, и общее переименование замкнуло бы монитор сам на себя.
    nav2 = GroupAction(
        condition=IfCondition(LaunchConfiguration('nav2')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch',
                    'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'true',
                    'params_file': os.path.join(pkg, 'params',
                                                'agro_nav2.yaml'),
                    'use_composition': 'False',
                }.items()),
        ])

    # ═══ SLAM ═══
    # Параметры RTAB-Map передаются ТОЛЬКО строками. Библиотека разбирает
    # их сама, а не через rclcpp, и число или bool она просто не примет.
    slam = IfCondition(LaunchConfiguration('slam'))
    rtabmap_topics = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('scan_cloud', '/dropoff/grid_cloud'),
        # Узлы RTAB-Map слушают топик с именем 'imu'. Без этой строки они
        # подписаны на несуществующий /imu и работают на одном зрении —
        # ровно та беда, из-за которой одометрия срывалась на разворотах.
        ('imu', '/imu/data'),
    ]
    rtabmap_common = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'use_sim_time': True,
        'subscribe_depth': True,
        # Мост публикует цвет и глубину разными сообщениями, и хотя метки
        # времени у них из одного кадра Gazebo, точная синхронизация иногда
        # промахивается. Приблизительная надёжнее и ничего не стоит.
        'approx_sync': True,
    }
    vo = Node(package='rtabmap_odom', executable='rgbd_odometry',
              output='screen', condition=slam,
              parameters=[rtabmap_common, {
                  'Odom/Strategy': '0',
                  'Vis/MaxFeatures': '1000',
                  'OdomF2M/MaxSize': '2000',
                  # САМОЕ ВАЖНОЕ ЗДЕСЬ. По умолчанию у RTAB-Map стоит 0, и
                  # это значит «потерявшись, не восстанавливаться никогда».
                  # Одного срыва на быстром развороте хватает, чтобы
                  # одометрия умерла до конца прогона: /odom продолжает
                  # идти, но с нулевой позой и ковариацией 9999, TF
                  # odom->base_footprint пропадает, и карта перестаёт
                  # строиться. Со стороны выглядит как «SLAM сломался».
                  # 1 означает переинициализацию после первого же
                  # потерянного кадра.
                  'Odom/ResetCountdown': '1',
              }, {
                  # Ждём IMU перед стартом: он даёт начальную ориентацию по
                  # гравитации, иначе крен и тангаж копятся от нуля и никак
                  # не поправляются.
                  'wait_imu_to_init': True,
              }],
              remappings=rtabmap_topics)
    slam_node = Node(package='rtabmap_slam', executable='rtabmap',
                     output='screen', condition=slam,
                     arguments=['--delete_db_on_start'],
                     parameters=[rtabmap_common, {
                         'subscribe_scan': False,
                         'Reg/Force3DoF': 'true',
                         'Optimizer/Slam2D': 'true',
                         # Сетку занятости строим по облаку, а не по лучу:
                         # лидара нет, а глубина даёт полноценный 3D.
                         # Сетку строим НЕ по глубине напрямую, а по облаку
                         # от детектора обрыва: в нём точки ниже плоскости
                         # уже подняты на 30 см. Так обрыв попадает на карту
                         # препятствием, а RTAB-Map про обрывы ничего знать
                         # не обязан.
                         'Grid/FromDepth': 'false',
                         'subscribe_scan_cloud': True,

                         # Пол делим по ВЫСОТЕ, а не по нормалям.
                         # По умолчанию RTAB-Map ищет пол по нормалям
                         # поверхности. На ровном полу, снятом под острым
                         # углом с трёх-четырёх метров, шум дальномера
                         # накреняет нормали, и куски пола перещёлкиваются
                         # в препятствия — это и есть та крупа, которой
                         # засыпана вся карта. Геометрию мы знаем точно,
                         # поэтому порог по высоте надёжнее.
                         'Grid/NormalsSegmentation': 'false',

                         # Порог ниже высоты рельса. Было 0.06, а труба
                         # поднимается на 0.051 — она попадала в «пол» и
                         # не появлялась на карте вовсе.
                         'Grid/MaxGroundHeight': '0.03',

                         # Ближайшее к обработке отрицательных препятствий,
                         # что есть у RTAB-Map. Точки НИЖЕ этой отметки не
                         # считаются ни полом, ни препятствием — они просто
                         # отбрасываются. Когда робот стоит на настиле,
                         # окружающий пол лежит на -0.12, и без этого он
                         # размечался как свободное место: обрыв выглядел
                         # вровень с настилом. Теперь за кромкой остаётся
                         # НЕИЗВЕСТНО, и планировщик туда не поедет.
                         'Grid/MinGroundHeight': '-0.05',

                         # Дальше трёх метров пол виден под слишком острым
                         # углом, чтобы верить его высоте.
                         'Grid/RangeMax': '3.0',
                         'Grid/MaxObstacleHeight': '1.8',
                         # Прочерчивание лучей ОБЯЗАТЕЛЬНО в этой схеме.
                         # Приняв облако за лазерный скан, RTAB-Map считает
                         # свободным только то, что прочерчено лучом от
                         # сенсора до отсчёта. Без трассировки свободного
                         # места не остаётся вовсе: замерено 1625 занятых
                         # клеток против 72 свободных.
                         #
                         # Раньше трассировка стирала границу настила —
                         # луч до дальней стены проходил НАД пониженным
                         # полом. Теперь это безопасно: детектор поднял
                         # точки обрыва на 30 см, они стали препятствием,
                         # и луч останавливается ровно на кромке.
                         'Grid/RayTracing': 'true',
                         # Гравитация как связь в графе: не даёт карте
                         # заваливаться по крену и тангажу при накоплении
                         # ошибки. Для наземного робота это почти даровая
                         # точность.
                         'Optimizer/GravitySigma': '0.3',
                     }, {
                         'wait_imu_to_init': True,
                     }],
                     remappings=rtabmap_topics)

    return [
        gz, rsp, clock, camera, relay, *noise, dropoff, rail_oracle, entry, spawn,

        # Контроллеры грузятся только после того, как модель оказалась в мире:
        # controller_manager живёт внутри плагина Gazebo и до спавна не существует.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        # Nav2 поднимается ПОСЛЕ контроллеров, а не вместе со всем сразу.
        #
        # Его менеджер жизненного цикла начинает подъём сразу после старта и
        # ждёт ответа серверов ограниченное время. Пока параллельно
        # поднимаются Gazebo, RTAB-Map и контроллеры, машина загружена,
        # ответа он не дожидается и бросает подъём на полпути. Снаружи это
        # выглядит так: узлы Nav2 в графе есть, но остаются unconfigured, а
        # действия /navigate_to_pose нет вовсе — автомат заезда доходит до
        # ПОДХОДА и говорит «Nav2 не отвечает».
        #
        # Привязка к выходу спавнера контроллеров надёжнее задержки на
        # таймере: она ждёт ровно того события, после которого машина
        # освобождается, и не зависит от того, насколько быстра эта машина.
        RegisterEventHandler(OnProcessExit(target_action=jsb,
                                           on_exit=[mec, nav2])),

        vo, slam_node,

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(pkg, 'rviz',
                                           LaunchConfiguration('rviz_config')
                                           .perform(context))]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat.world',
                              description='имя файла в share/maze_nav/worlds/'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='окно Gazebo; false — только сервер'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value='nav.rviz'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='RTAB-Map: визуальная одометрия и карта'),
        DeclareLaunchArgument('random_spawn', default_value='false',
                              description='случайная поза на настиле, '
                                          'пригодная для осмотра и заезда'),
        DeclareLaunchArgument('seed', default_value='none',
                              description='зерно случайной позы, чтобы '
                                          'прогон можно было повторить'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='планировщик Nav2 для подхода '
                                          'к точке заезда'),
        DeclareLaunchArgument('rail_oracle', default_value='true',
                              description='идеальная маска рельсов из мира '
                                          'вместо распознавания'),
        DeclareLaunchArgument('dropoff', default_value='true',
                              description='детектор края платформы'),
        DeclareLaunchArgument('entry_monitor', default_value='true',
                              description='контроль заезда по двум одометриям'),
        DeclareLaunchArgument('depth_noise', default_value='true',
                              description='приводить глубину к поведению '
                                          'настоящего D435'),
        # Точка появления робота. 'auto' — взять подходящую для выбранного
        # мира (см. SPAWN выше). По высоте робот ставится чуть ВЫШЕ опоры:
        # если посадить ровно вровень, ролики оказываются в контакте уже в
        # первом шаге решателя, получают выталкивающий импульс, и робот
        # подпрыгивает на старте.
        DeclareLaunchArgument('x', default_value='auto'),
        DeclareLaunchArgument('y', default_value='auto'),
        DeclareLaunchArgument('z', default_value='auto'),
        DeclareLaunchArgument('yaw', default_value='auto'),
        DeclareLaunchArgument('edge_only', default_value='false',
                              description='поставить робота боком, чтобы '
                                          'рельсы не попадали в кадр'),

        OpaqueFunction(function=launch_setup),
    ])
