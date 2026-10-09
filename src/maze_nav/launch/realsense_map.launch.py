#!/usr/bin/env python3
"""Карта с рук: RealSense в руках, ноутбук на плече, никакого робота.

Нужно, чтобы проверить связку камера + ИНС отдельно от стенда: качество
одометрии, дальность, поведение на бликах и однотонных стенах — всё то, что
симулятор не воспроизводит.

Отличия от стенда принципиальные, а не косметические:

* нет колёсной базы и нет base_footprint. Опорным кадром служит сам корпус
  камеры, camera_link, и одометрию ведёт только зрение с ИНС;
* нет детектора обрыва и нет маски рельсов. Здесь строится обычная карта,
  а не карта площадки с кромкой;
* глубина выравнивается по цвету драйвером (align_depth), а не сводится
  вручную, — RTAB-Map ждёт кадры в одном кадре координат.

Про ИНС. У D435 её НЕТ, она есть только у D435i. Если камера без ИНС,
запускать надо с imu:=false, иначе одометрия будет ждать инициализации,
которой не дождётся, и карта не начнёт строиться вовсе. Проверить модель
можно так:  rs-enumerate-devices | grep -i "Name\\|Firmware"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    imu = LaunchConfiguration('imu')
    prof = LaunchConfiguration('profile')
    db = LaunchConfiguration('db')

    src = LaunchConfiguration('imu_source')
    use_pico = PythonExpression(["'", src, "' == 'pico'"])
    use_cam_imu = PythonExpression(
        ["'", src, "' == 'camera' and '", imu, "'.lower() in ('true','1')"])


    # Топики драйвера. camera_namespace пустой, иначе они уезжают в
    # /camera/camera/..., и каждый раз приходится гадать, сколько там слоёв.
    rgb = '/camera/color/image_raw'
    info = '/camera/color/camera_info'
    depth = '/camera/aligned_depth_to_color/image_raw'

    # ДВЕ КАМЕРЫ НА ВЫБОР, аргумент camera:=realsense|zed. Остальной
    # запуск знает только три имени топиков выше и про камеру не
    # осведомлён: ветка ZED переименовывает свои в эти же.
    use_rs = PythonExpression(
        ["'", LaunchConfiguration('camera'), "' != 'zed'"])
    use_zed = PythonExpression(
        ["'", LaunchConfiguration('camera'), "' == 'zed'"])

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch',
            'rs_launch.py')),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            # Глубину выравниваем по цвету. Без этого у цвета и глубины
            # разные кадры координат и разные матрицы, и RTAB-Map строит
            # облако со смещением.
            'align_depth.enable': 'true',
            # Жёсткая синхронизация цвета и глубины. С рук камера движется
            # быстрее, чем на роботе, и рассинхрон в полкадра даёт смазанное
            # облако.
            'enable_sync': 'true',
            'depth_module.depth_profile': prof,
            'rgb_camera.color_profile': prof,
            # Потоки ИНС камеры поднимаем, только если она и есть
            # источник: при imu_source:=pico они лишние и мешают, потому
            # что их открытие падает на занятом ядром модуле движения.
            'enable_gyro': use_cam_imu,
            'enable_accel': use_cam_imu,
            # 2 — линейная интерполяция: гироскоп и акселерометр сводятся в
            # один топик /camera/imu. Без этого они идут раздельно и
            # фильтру ориентации их нечем связать.
            'unite_imu_method': '2',
            # Сброс камеры при запуске по умолчанию ВЫКЛЮЧЕН. Он помогает,
            # когда устройство осталось занятым после убитого процесса, но
            # сам способен уронить камеру с шины: она исчезает из lsusb, а
            # драйвер сыплет «No such device» на каждом кадре, и лечится
            # это только физическим переподключением. Включать осознанно.
            'initial_reset': LaunchConfiguration('reset'),
        }.items(),
        condition=IfCondition(use_rs))

    # ═══ Ветка ZED ═══
    #
    # Имена топиков сняты с живого узла обёртки 5.5 — в прежних выпусках
    # они были другими. Если после обновления камера перестанет доходить
    # до RTAB-Map, сверить с разделом «PUBLISHED TOPICS» в её логе.
    #
    # Собственную привязку ZED выключаем: одометрию здесь считает
    # rgbd_odometry, а два источника одного преобразования рвут дерево TF.
    zed = GroupAction(
        condition=IfCondition(use_zed),
        actions=[
            SetRemap('/zed/zed_node/rgb/color/rect/image', rgb),
            SetRemap('/zed/zed_node/rgb/color/rect/camera_info', info),
            SetRemap('/zed/zed_node/depth/depth_registered', depth),
            # ИНС ZED отдаёт УЖЕ ГОТОВУЮ ориентацию: кватернион считает
            # сам SDK, сводя гироскоп с акселерометром на 800 Гц. Поэтому
            # её поток идёт прямо в /imu/data, минуя фильтр Маджвика —
            # тот ниже отключается, когда выбрана эта камера.
            #
            # Нужна она для выравнивания карты по тяжести. Без неё
            # горизонтом становится то, как камера была наклонена в
            # первое мгновение, и вся карта наследует этот перекос.
            SetRemap('/zed/zed_node/imu/data', '/imu/data'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('zed_wrapper'), 'launch',
                    'zed_camera.launch.py')),
                launch_arguments={
                    'camera_model': 'zedm',
                    'camera_name': 'zed',
                    # РАЗРЕШЕНИЕ И ТЕМП СНИЖЕНЫ НАМЕРЕННО.
                    #
                    # По умолчанию ZED отдаёт 1920x1080 на тридцати
                    # герцах. Замерено, что с этим связка не справляется:
                    # обработка кадра в RTAB-Map занимала до 1.2 с при
                    # темпе 0.5 с, задержки доходили до 1.6 с, одометрия
                    # шла 11 Гц при кадрах 27 — то есть между двумя
                    # обработанными кадрами камера успевала уехать, и
                    # сопоставление валилось с «Not enough inliers 0/15».
                    # Каждый такой провал сбрасывает одометрию, а сброс
                    # начинает НОВУЮ карту: в рабочей памяти набралось 799
                    # узлов, не связанных между собой, и единой карты не
                    # получалось вовсе.
                    #
                    # HD720 это 1280x720 против 848x480 у RealSense —
                    # всё ещё вдвое больше, но уже посильно. Если и этого
                    # много, следующая ступень VGA (672x376).
                    'param_overrides': PythonExpression([
                        "';'.join([x for x in [",
                        "'general.grab_resolution:=",
                        LaunchConfiguration('zed_res'), "',",
                        "'general.pub_frame_rate:=",
                        LaunchConfiguration('zed_fps'), "',",
                        "'depth.depth_mode:=",
                        LaunchConfiguration('zed_depth_mode'), "',",
                        "'depth.max_depth:=",
                        LaunchConfiguration('zed_max_depth'), "',",
                        "'depth.depth_confidence:=",
                        LaunchConfiguration('zed_conf'), "',",
                        # СОБСТВЕННАЯ ПРИВЯЗКА ZED ВЫКЛЮЧЕНА ЦЕЛИКОМ.
                        # Раньше я выключил только публикацию её
                        # преобразований, а сам расчёт продолжал идти: в
                        # логе видно «Positional tracking: TRUE, mode
                        # GEN 3, Area Memory: TRUE» — это отдельный SLAM
                        # со своей картой местности внутри SDK. Нам он не
                        # нужен, одометрию ведёт rgbd_odometry, карту
                        # RTAB-Map. Замерено: узел камеры съедал 119%
                        # процессора, и заметная часть уходила сюда.
                        "'pos_tracking.pos_tracking_enabled:=false',",
                        "'", LaunchConfiguration('zed_extra'), "'",
                        "] if x])"]),
                    'publish_tf': 'false',
                    'publish_map_tf': 'false',
                    # А ВОТ СВЯЗКУ ИНС С КОРПУСОМ ПУБЛИКОВАТЬ НАДО.
                    # Это отдельный ключ, и по умолчанию он false. Без
                    # него кадра zed_imu_link в дереве нет, хотя
                    # сообщения ИНС на него ссылаются, и одометрия
                    # отказывается её принимать:
                    #   «Dropping imu data! A valid TF between
                    #    camera_link and zed_imu_link is required»
                    # Сам узел при этом молча пишет в лог
                    # «Broadcast IMU TF: FALSE» — заметить легко только
                    # задним числом.
                    'publish_imu_tf': 'true',
                    # publish_urdf ОСТАЁТСЯ ВКЛЮЧЁННЫМ, и это не
                    # недосмотр. Сперва я его выключил, рассудив, что
                    # описание камеры нам ни к чему: своё есть. Узел
                    # после этого вставал на «Starting Positional
                    # Tracking / Waiting for valid static
                    # transformations» и кадры не публиковал вовсе —
                    # привязка ждёт преобразований, которые публикует
                    # именно этот издатель.
                    #
                    # Выключены только publish_tf и publish_map_tf:
                    # odom->base и map->odom дают rgbd_odometry и
                    # RTAB-Map, а два источника одного преобразования
                    # рвут дерево TF.
                    'publish_urdf': 'true',
                }.items()),
            # Опорный кадр здесь camera_link, а глубина приходит в
            # zed_left_camera_frame_optical. Цепочка кадров ZED висит
            # отдельным деревом, и без этой связки RTAB-Map не сможет
            # пересчитать глубину и откажется работать.
            #
            # Нули означают: камера там же, где корпус RealSense. Для
            # съёмки с рук это и не важно — карта строится относительно
            # самой камеры.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='zed_mount', output='log',
                 arguments=['--frame-id', 'camera_link',
                            '--child-frame-id', 'zed_camera_link',
                            '--x', '0', '--y', '0', '--z', '0',
                            '--roll', '0', '--pitch', '0', '--yaw', '0']),
        ])

    # ИСТОЧНИК ИНС: камера или отдельная плата.
    #
    # У D435i ИНС есть, но ядро Linux забирает её себе модулями hid_sensor_*,
    # и librealsense тогда не может открыть модуль движения. Лечится
    # выгрузкой этих модулей на ХОСТЕ, но не везде это допустимо. Поэтому
    # есть второй путь: MPU6050 на Pico, поток по USB CDC. Прошивка лежит
    # в корне проекта, pico_imu.py.
    pico = Node(
        package='maze_nav', executable='pico_imu_node.py', output='screen',
        condition=IfCondition(use_pico),
        parameters=[{'port': LaunchConfiguration('imu_port'),
                     'frame_id': 'imu_link',
                     'axes': LaunchConfiguration('imu_axes')}],
        remappings=[('imu/data_raw', '/imu/data_raw')])

    # Связка камеры с платой ИНС. По умолчанию единичная: как датчик
    # реально повёрнут относительно камеры, знает только тот, кто его
    # прикручивал. Если он стоит криво, ориентация приедет с постоянным
    # наклоном, и это будет видно как заваленный горизонт в карте.
    pico_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        condition=IfCondition(use_pico),
        arguments=['--frame-id', 'camera_link', '--child-frame-id',
                   'imu_link'])

    # Сырые гироскоп с акселерометром — это не ориентация. Фильтр Маджвика
    # сводит их в кватернион, который и ждёт RTAB-Map.
    imu_filter = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter', output='screen',
        # ТОЛЬКО ДЛЯ СЫРЫХ ИСТОЧНИКОВ. У RealSense и у платы на Pico
        # наружу идут отдельно гироскоп и акселерометр, их надо свести в
        # ориентацию. ZED делает это внутри себя, и второй фильтр поверх
        # готового кватерниона только навредил бы.
        condition=IfCondition(PythonExpression([
            "'", imu, "'.lower() in ('true','1') and '",
            LaunchConfiguration('camera'), "' != 'zed'"])),
        parameters=[{'use_mag': False, 'world_frame': 'enu',
                     # TF от фильтра не нужен: дерево кадров ведёт драйвер
                     # камеры, и вторая рука в нём всё только испортит.
                     'publish_tf': False}],
        # Откуда брать сырые данные, решает imu_source: у камеры они в
        # /camera/imu, у платы — в /imu/data_raw, который и так наш.
        remappings=[('imu/data_raw', PythonExpression(
                        ["'/imu/data_raw' if '", src,
                         "' == 'pico' else '/camera/imu'"])),
                    ('imu/data', '/imu/data')])

    # Одометрия. Кадр опоры — корпус камеры: другого твёрдого тела здесь
    # нет. Два варианта узла отличаются только ожиданием ИНС.
    odom_common = dict(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        remappings=[('rgb/image', rgb), ('rgb/camera_info', info),
                    ('depth/image', depth), ('imu', '/imu/data')])
    odom_params = {
        'frame_id': 'camera_link',
        'approx_sync': True,
        'publish_tf': True,
        'Odom/ResetCountdown': '1',
        # С рук камеру ведут рывками, и потеря слежения неизбежна. Пусть
        # узел сам перезапускается, а не встаёт до конца прогона.
        'Odom/Strategy': '0',
        'Vis/MinInliers': '15',
        # НЕ ПРОВЕРЯТЬ СВЯЗЬ С ИНС НА КАЖДОЕ СООБЩЕНИЕ.
        #
        # Связь камеры с её ИНС неподвижна — это железо внутри корпуса.
        # Но узел по умолчанию запрашивает её под метку времени каждого
        # сообщения, а ИНС идёт 800 Гц и обгоняет публикацию
        # преобразований на миллисекунды. Получался сплошной поток
        # «Lookup would require extrapolation into the future» и
        # «Could not transform IMU msg», по два предупреждения на кадр.
        #
        # Сам узел в этом же предупреждении и советует: если связь
        # статична, проверку можно выключить. Она статична.
        'always_check_imu_tf': False,
    }
    odom_imu = Node(**odom_common, condition=IfCondition(imu),
                    parameters=[dict(odom_params, wait_imu_to_init=True)])
    odom_plain = Node(**odom_common, condition=UnlessCondition(imu),
                      parameters=[dict(odom_params, wait_imu_to_init=False)])

    # Стирать базу при старте — поведение по умолчанию у RTAB-Map, и для
    # стенда оно верное: там каждый прогон начинается с чистого листа.
    # Здесь же база это результат работы, ради которого человек ходил с
    # камерой полчаса, и потерять её из-за следующего запуска нельзя.
    # Поэтому по умолчанию НЕ стираем, а дописываем; стереть можно явно.
    #
    # Два узла вместо одного с вычисляемым аргументом: в ветке «не стирать»
    # пришлось бы подставлять пустую строку в argv, а это грязно и зависит
    # от того, как разбирает аргументы сама RTAB-Map.
    slam_params = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'database_path': db,
        # Сетку занятости строим по глубине: детектора обрыва здесь нет,
        # и подставлять вместо него нечего.
        'Grid/FromDepth': 'true',
        'Grid/RangeMax': '4.0',
        # Пол ищем по нормалям, а не по высоте: камеру держат в руке под
        # произвольным наклоном, и постоянный порог высоты бессмыслен.
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundAngle': '45',
        'Grid/RayTracing': 'true',
        'Rtabmap/DetectionRate': '2.0',
    }]
    slam_remap = [('rgb/image', rgb), ('rgb/camera_info', info),
                  ('depth/image', depth), ('imu', '/imu/data')]
    reset_db = LaunchConfiguration('reset_db')
    slam_fresh = Node(package='rtabmap_slam', executable='rtabmap',
                      output='screen', condition=IfCondition(reset_db),
                      arguments=['--delete_db_on_start'],
                      parameters=slam_params, remappings=slam_remap)
    slam_keep = Node(package='rtabmap_slam', executable='rtabmap',
                     output='screen', condition=UnlessCondition(reset_db),
                     parameters=slam_params, remappings=slam_remap)

    viz = Node(package='rtabmap_viz', executable='rtabmap_viz',
               output='screen',
               condition=IfCondition(LaunchConfiguration('viz')),
               parameters=[{'frame_id': 'camera_link',
                            'subscribe_depth': True, 'subscribe_rgb': True,
                            'approx_sync': True}],
               remappings=[('rgb/image', rgb), ('rgb/camera_info', info),
                           ('depth/image', depth)])

    # RViz с нашим конфигом — РЯДОМ с rtabmap_viz, а не вместо него.
    # Окно rtabmap_viz показывает внутреннюю кухню SLAM: замыкания
    # петель, одометрию, граф. А плотное текстурное облако всей карты
    # даёт слой «Текстурная карта RTAB-Map» в RViz, и он подписан на
    # /mapData. В конфиге он ВЫКЛЮЧЕН по умолчанию: поток тяжёлый, и
    # нужен он не всегда — включается галкой в списке слоёв.
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', os.path.join(
                    get_package_share_directory('maze_nav'),
                    'rviz', 'nav.rviz')])

    return LaunchDescription([
        DeclareLaunchArgument('imu', default_value='true',
                              description='есть ли ИНС в камере: у D435i '
                                          'есть, у обычного D435 нет'),
        DeclareLaunchArgument('imu_source', default_value='camera',
                              description="откуда брать ИНС: 'camera' "
                                          "(D435i) или 'pico' (MPU6050 на "
                                          "плате, см. pico_imu.py)"),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyACM0',
                              description='порт платы с MPU6050'),
        DeclareLaunchArgument('imu_axes', default_value='x y z',
                              description='разворот осей датчика в оси ROS, '
                                          "например '-y x z'"),
        DeclareLaunchArgument('profile', default_value='848x480x30',
                              description='разрешение и частота потоков'),
        DeclareLaunchArgument('db', default_value='/datasets/handheld.db',
                              description='куда класть базу карты; /datasets '
                                          'смонтирован с хоста, поэтому файл '
                                          'переживёт контейнер'),
        DeclareLaunchArgument('reset_db', default_value='false',
                              description='стереть базу карты при старте; '
                                          'по умолчанию съёмка дописывается '
                                          'в существующую'),
        DeclareLaunchArgument('reset', default_value='false',
                              description='сбросить камеру при запуске; '
                                          'может уронить её с шины, '
                                          'включать только осознанно'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='окно rtabmap_viz'),
        # ГЛУБИНА ОБРЕЗАНА ЧЕТЫРЬМЯ МЕТРАМИ, и это главная настройка.
        # У ZED Mini стереобаза 62.9 мм (из её калибровки), а ошибка
        # глубины растёт как квадрат дальности и обратно базе:
        #
        #    1 м -> 0.9 см    4 м -> 14 см
        #    2 м -> 3.5 см    5 м -> 22 см
        #    3 м -> 8   см   10 м -> 89 см
        #
        # По умолчанию SDK принимает всё до десяти метров, и точки с
        # ошибкой под метр ложились в карту наравне с ближними, размазывая
        # её. Четыре метра — граница, за которой ошибка превышает 14 см.
        # ВОЗВРАЩЕНО К ДЕСЯТИ МЕТРАМ. Обрезка до четырёх улучшала бы
        # карту, но ломала одометрию: ей нужны точки С ГЛУБИНОЙ как
        # опорные признаки, и в просторном помещении четыре метра
        # оставляли её почти без материала — траектория запуталась.
        #
        # Правильное место для обрезки не здесь, а на стороне карты:
        # камера пусть отдаёт всё, что видит, одометрия пользуется
        # дальними точками, а в КАРТУ дальний шум не пускаем отдельно.
        # В nav.rviz у слоя текстурной карты для этого уже стоит
        # «Cloud max depth: 4.0», а у RTAB-Map есть Grid/RangeMax.
        DeclareLaunchArgument('zed_max_depth', default_value='10.0',
                              description='дальше этого глубина ZED не '
                                          'используется, м'),
        # Порог доверия: чем МЕНЬШЕ, тем строже отбор. По умолчанию 95,
        # то есть пропускается почти всё, включая края предметов, блики и
        # однотонные пятна, где стерео угадывает. 50 оставляет только
        # уверенные пиксели: карта реже, но чище.
        DeclareLaunchArgument('zed_conf', default_value='50',
                              description='порог доверия к глубине ZED, '
                                          '0..100, меньше — строже'),
        DeclareLaunchArgument('zed_depth_mode', default_value='NEURAL_LIGHT',
                              description='NEURAL_LIGHT, NEURAL, NEURAL_PLUS: '
                                          'точнее и тяжелее по возрастанию'),
        DeclareLaunchArgument('zed_extra', default_value='',
                              description='любые ещё параметры ZED через '
                                          'точку с запятой, вида '
                                          'раздел.имя:=значение'),
        DeclareLaunchArgument('zed_res', default_value='HD720',
                              description='разрешение ZED: HD2K, HD1080, '
                                          'HD720, VGA'),
        DeclareLaunchArgument('zed_fps', default_value='15.0',
                              description='темп публикации кадров ZED, Гц'),
        DeclareLaunchArgument('camera', default_value='realsense',
                              description='какая камера: realsense или zed'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='окно RViz с нашим конфигом: в нём '
                                          'живёт слой плотного текстурного '
                                          'облака всей карты'),
        realsense, zed, imu_filter, pico, pico_tf, odom_imu, odom_plain, slam_fresh, slam_keep, viz, rviz,
    ])
