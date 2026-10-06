#!/usr/bin/env python3
"""Навигация на НАСТОЯЩЕМ роботе: карта по RGB-D и езда по ней.

Всё считается на ноутбуке, который едет на роботе: к нему воткнуты
реалсенс и плата с MPU6050. На робота по сети уходит только скорость.

РАЗДЕЛЕНИЕ МАШИН. На роботе стоит ROS 2 Humble, здесь Jazzy. Через
границу ходит одно сообщение — geometry_msgs/Twist в топике /cmd_nav, и
этот тип между дистрибутивами не менялся. Проверено передачей: значения
доходят без искажений. Собственные типы RTAB-Map границу бы не прошли, но
им и не нужно: весь SLAM на одной стороне.

КУДА ПУБЛИКУЕТСЯ СКОРОСТЬ. Только в /cmd_nav, никогда в /cmd_vel.
На роботе стоит мультиплексор: он принимает /cmd_vel_teleop с геймпада и
/cmd_nav отсюда, а в /cmd_vel пишет сам. Режим переключается кнопкой X на
геймпаде, программного пути нет — и это правильно, кнопка остаётся
аварийным выключателем. Если писать в /cmd_vel напрямую, мы обойдём и
мультиплексор, и сторожа по геймпаду.

ЧЕГО ЗДЕСЬ НЕТ. Колёсной одометрии: на роботе /wheel_odometry имеет тип
Twist, то есть это скорость, а не поза, и одометрией служить не может в
принципе. Положение ведёт только зрение с ИНС.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from typing import List
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('maze_nav')
    imu = LaunchConfiguration('imu')
    prof = LaunchConfiguration('profile')

    rgb = '/camera/color/image_raw'
    info = '/camera/color/camera_info'
    depth = '/camera/aligned_depth_to_color/image_raw'

    # ═══ Описание робота ═══
    # Берём описание для ЖЕЛЕЗА: без блоков Gazebo и без оптического кадра
    # камеры — его ведёт драйвер реалсенса. Отсюда же берётся связка
    # base_footprint -> camera_link, без которой одометрия не сможет
    # пересчитать движение камеры в движение робота.
    urdf = os.path.join(pkg, 'urdf', 'agro_robot_real.urdf.xacro')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': ParameterValue(
                   Command(['xacro ', urdf]), value_type=str)}])

    # ═══ Камера ═══
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch',
            'rs_launch.py')),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            # Живое плотное облако с текстурой. По умолчанию ВЫКЛЮЧЕНО:
            # 848x480 точек с цветом тридцать раз в секунду — это заметный
            # поток, а нужен он только чтобы посмотреть глазами, что под
            # роботом прямо сейчас.
            #
            # Карта такой текстуры не даст и не должна: мы кормим RTAB-Map
            # СВОИМ облаком, где цвет означает смысл (серое — пол, зелёное
            # — рельсы, красное — кромка), а не вид поверхности. Это
            # сознательный размен разметки на текстуру.
            'pointcloud.enable': LaunchConfiguration('cloud'),
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': prof,
            'rgb_camera.color_profile': prof,
            # ИНС камеры НЕ трогаем: её забирает ядро модулями hid_sensor_*,
            # и попытка открыть модуль движения роняет весь узел драйвера.
            # Угловую скорость даёт отдельная плата.
            # СОБСТВЕННАЯ ИНС КАМЕРЫ. По умолчанию выключена: для карты
            # она не нужна, а в ядре её разбирают модули hid_sensor_*, из-за
            # чего драйвер натыкается на «Permission denied».
            #
            # Включается ради калибровки угла крепления: акселерометр
            # камеры меряет тяжесть В ЕЁ СОБСТВЕННОМ кадре, то есть даёт
            # наклон камеры напрямую, без посредников и без URDF.
            #
            # Если драйвер ругнётся на доступ — выгрузить модули ядра,
            # которые забрали прибор себе: sudo modprobe -r hid_sensor_accel_3d
            'enable_gyro': LaunchConfiguration('cam_imu'),
            'enable_accel': LaunchConfiguration('cam_imu'),
            'initial_reset': 'false',
        }.items())

    # ═══ ИНС ═══
    pico = Node(package='maze_nav', executable='pico_imu_node.py',
                name='pico_imu', output='screen',
                condition=IfCondition(imu),
                parameters=[{'port': LaunchConfiguration('imu_port'),
                             'frame_id': 'imu_link',
                             'axes': LaunchConfiguration('imu_axes'),
                             'gyro_bias': ParameterValue(
                                 LaunchConfiguration('gyro_bias'),
                                 value_type=List[float]),
                             # ЧУВСТВИТЕЛЬНОСТЬ АКСЕЛЕРОМЕТРА ЗАДАНА ЯВНО.
                             #
                             # Узел умеет выбирать её по формату кадра и
                             # для подлинного MPU6050 выбрал бы верно:
                             # регистр диапазона у нас читается 0x08, что
                             # по документации означает +-4 g и 8192
                             # отсчёта на g.
                             #
                             # Но чип на плате НЕ ПОДЛИННЫЙ: WHO_AM_I
                             # отвечает 0x70 вместо 0x68. У клонов
                             # раскладка битов диапазона бывает сдвинута,
                             # и здесь именно так. Замерено на неподвижном
                             # роботе: тяжесть выходила 4.92 м/с² вместо
                             # 9.81, то есть ровно вдвое меньше. Значит
                             # чувствительность 4096, как у +-8 g.
                             #
                             # Проверено чтением регистра: запись проходит,
                             # чип отдаёт обратно ровно то, что записали.
                             # То есть виновата не прошивка, а раскладка
                             # конкретного экземпляра.
                             #
                             # Проверка после правки: imu_level.py должен
                             # показать |a| около 9.81. Если выйдет 19.6 —
                             # поставили не тому чипу, вернуть 0 (выбор по
                             # формату кадра).
                             'accel_lsb_per_g': 4096.0,
                             'accel_bias': ParameterValue(
                                 LaunchConfiguration('accel_bias'),
                                 value_type=List[float])}],
                remappings=[('imu/data_raw', '/imu/data_raw')])
    madgwick = Node(package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter', output='screen',
                    condition=IfCondition(imu),
                    parameters=[{'use_mag': False, 'world_frame': 'enu',
                                 'publish_tf': False}],
                    remappings=[('imu/data_raw', '/imu/data_raw'),
                                ('imu/data', '/imu/data')])

    # ═══ Одометрия ═══
    # frame_id = base_footprint, а не камера: узел меряет движение камеры,
    # но обязан отдать движение РОБОТА, и пересчёт идёт через TF. Поэтому
    # ошибка в креплении камеры превращается в систематическую ошибку
    # одометрии, а не в шум.
    odom_common = dict(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        remappings=[('rgb/image', rgb), ('rgb/camera_info', info),
                    ('depth/image', depth), ('imu', '/imu/data')])
    # ТРИ СТЕПЕНИ СВОБОДЫ ВМЕСТО ШЕСТИ, по флагу force_2d.
    #
    # Робот ездит по ровному настилу, и физически у него три степени
    # свободы: x, y и курс. Зрение же оценивает все шесть, и три лишние —
    # шум, который НАКАПЛИВАЕТСЯ. Замерено в симуляторе: крен 3.5 и тангаж
    # 5.6 градуса за сорок секунд езды по ровному полу, при истинных нулях.
    #
    # На живом роботе это проявилось иначе и больнее. Порог пола у
    # RTAB-Map — три сантиметра (Grid/MaxGroundHeight ниже). Когда карта
    # наклоняется, дальние части ровного пола поднимаются выше порога и
    # становятся ПРЕПЯТСТВИЯМИ: костмап закрашивался непроезжим там, где
    # робот только что проехал. Заодно переставала выделяться кромка — она
    # публикуется приподнятой, но на фоне поднявшегося пола терялась.
    #
    # Расплата: настоящие наклоны корпуса (подвеска при разгоне, ступенька
    # на рельсах) в оценку не попадут. Для заезда это не опасно — там крен
    # проверяется ПО ИНС напрямую, а не по одометрии.
    force_2d = ParameterValue(LaunchConfiguration('force_2d'), value_type=str)

    odom_params = {
        'Reg/Force3DoF': force_2d,
        'frame_id': 'base_footprint',
        'approx_sync': True,
        'publish_tf': True,
        'Odom/ResetCountdown': '1',
        'Vis/MinInliers': '15',
    }
    odom_imu = Node(**odom_common, condition=IfCondition(imu),
                    parameters=[dict(odom_params, wait_imu_to_init=True)])
    odom_plain = Node(**odom_common, condition=UnlessCondition(imu),
                      parameters=[dict(odom_params, wait_imu_to_init=False)])

    # ═══ SLAM ═══
    # ═══ SLAM ═══
    # Два варианта, и разница принципиальная.
    #
    # Без детектора обрыва RTAB-Map строит сетку занятости сам из глубины.
    # Кромку площадки он при этом НЕ увидит: обрыв — это отрицательное
    # препятствие, пола там просто нет, и для сетки это неотличимо от
    # неизвестности.
    #
    # С детектором RTAB-Map перестаёт смотреть на глубину и принимает
    # готовое облако от него (subscribe_scan_cloud). В этом облаке кромка
    # уже подделана под обычное препятствие — поднята на виртуальную
    # высоту, — и попадает в карту красным. Ровно так это и работало в
    # симуляторе.
    # Стирать ли базу на старте.
    #
    # По умолчанию да: при отладке почти всегда нужна чистая карта, а
    # дописывание в старую даёт путаницу, которую потом не распутать —
    # робот стоит в одном месте, а карта помнит прошлую поездку.
    #
    # Но когда карту собирают НАРОЧНО, чтобы потом по ней ездить, стирать
    # её нельзя. Для этого delete_db:=false: тогда база, указанная в db,
    # переживёт запуск и её можно будет открыть снова.
    #
    # ВАЖНО: RTAB-Map дописывает базу по ходу, но закрывает её корректно
    # только при штатном завершении. Останавливать запуск надо Ctrl-C и
    # дождаться, пока узлы погаснут сами. Убийство через kill -9 оставит
    # базу недописанной.
    slam_common = dict(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        arguments=[PythonExpression(
            ["'--delete_db_on_start' if '",
             LaunchConfiguration('delete_db'), "'.lower() in "
             "('true', '1', 'yes') else ''"])])
    slam_base = {
        # Та же мера для графа карты, см. force_2d выше. Slam2D держит
        # оптимизатор в плоскости: без него граф остаётся трёхмерным, и
        # замыкания петель снова внесут крен с тангажом, уже исправленные
        # в одометрии.
        'Reg/Force3DoF': force_2d,
        'Optimizer/Slam2D': force_2d,
        # ОЧЕРЕДИ СИНХРОНИЗАЦИИ подняты с умолчания 10.
        #
        # RTAB-Map ждёт пять входов с совпадающими метками: одометрию,
        # цвет, глубину, калибровку и облако от детектора. Облако приходит
        # позже остальных — детектор его считает, — а одометрия, как видно
        # в логе, отдаёт результат с задержкой до 0.4 с. Очередь в 10
        # кадров на 30 Гц держит всего 333 мс истории, и набор просто не
        # успевал сойтись: узел работал, но не публиковал НИЧЕГО, включая
        # преобразование map->odom. Снаружи это выглядело как «нет кадра
        # map» и неактивный Nav2, хотя причина была здесь.
        #
        # Тридцать кадров — это секунда истории, с запасом на наблюдаемые
        # задержки. Платим памятью на буферы, её достаточно.
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'database_path': LaunchConfiguration('db'),
        'Grid/RayTracing': 'true',
        'Rtabmap/DetectionRate': '2.0',
    }
    slam_remap = [('rgb/image', rgb), ('rgb/camera_info', info),
                  ('depth/image', depth), ('imu', '/imu/data')]

    slam_plain = Node(
        **slam_common, condition=UnlessCondition(LaunchConfiguration('dropoff')),
        parameters=[dict(slam_base,
                         **{'Grid/FromDepth': 'true',
                            'Grid/RangeMax': '4.0',
                            # Пол по нормалям: камера наклонена, и
                            # постоянный порог высоты отрезал бы дальнюю
                            # часть пола вместе с препятствиями.
                            'Grid/NormalsSegmentation': 'true',
                            'Grid/MaxGroundAngle': '45'})],
        remappings=slam_remap)

    slam_edge = Node(
        **slam_common, condition=IfCondition(LaunchConfiguration('dropoff')),
        parameters=[dict(slam_base,
                         **{'subscribe_scan_cloud': True,
                            # Сетку строим ТОЛЬКО из облака детектора.
                            'Grid/FromDepth': 'false',
                            # Облако уже в кадре камеры и уже разобрано на
                            # пол и препятствия, поэтому сегментация по
                            # нормалям тут лишняя и только мешает.
                            'Grid/NormalsSegmentation': 'false',
                            # ПОРОГ ПОЛА ПОДНЯТ С 3 ДО 15 СМ.
                            #
                            # Всё выше порога RTAB-Map считает
                            # препятствием. Три сантиметра означали, что
                            # малейший перекос карты делает дальний край
                            # ровного пола непроезжим: при остаточном
                            # наклоне 1.6 градуса пол на трёх метрах
                            # поднимается на 8 см, и костмап закрашивался
                            # фиолетовым там, где робот только что ехал.
                            #
                            # Нам от сетки нужно немногое: знать, где
                            # КРОМКА, и не пускать в неизвестность. Кромку
                            # детектор публикует приподнятой на 30 см
                            # (virtual_height), так что порог 15 см её
                            # по-прежнему ловит с двукратным запасом, а
                            # шум пола и наклон — уже нет.
                            #
                            # Расплата: настоящее препятствие ниже 15 см
                            # на настиле сеткой не заметится. Рельсы в том
                            # числе — но они и не должны быть
                            # препятствием, робот на них заезжает, а
                            # показываются они своим слоем.
                            'Grid/MaxGroundHeight': '0.15',
                            'Grid/MinGroundHeight': '-0.05',
                            'Grid/RangeMax': '3.0',
                            'Grid/Sensor': '0'})],
        remappings=slam_remap + [('scan_cloud', '/dropoff/grid_cloud')])

    # ═══ Nav2 ═══
    # МОНИТОР СТОЛКНОВЕНИЙ ВЫВЕДЕН ИЗ ЦЕПОЧКИ СКОРОСТЕЙ.
    #
    # Штатно цепочка такая: контроллер -> сглаживатель -> монитор -> робот.
    # Монитор требует живой источник облака точек и, не получив его,
    # ОСТАНАВЛИВАЕТ робота. В настройках там стоит /dropoff/grid_cloud от
    # детектора обрыва, которого на роботе может не быть вовсе, и монитор
    # глушил скорость наглухо — снаружи это выглядело как «Nav2 работает,
    # путь построен, а робот стоит».
    #
    # Поэтому монитор обходим: вход переводим на несуществующий топик,
    # выход в тупик, а сглаживатель публикует прямо в /cmd_nav. Теряем
    # автоматический останов перед препятствием, которое Nav2 проглядел;
    # последняя защита — кнопка X на геймпаде.
    nav_params = RewrittenYaml(
        source_file=os.path.join(pkg, 'params', 'agro_nav2.yaml'),
        root_key='', convert_types=True,
        param_rewrites={'cmd_vel_in_topic': '/collision_monitor_unused_in',
                        'cmd_vel_out_topic': '/collision_monitor_unused_out'})
    nav2 = GroupAction(
        condition=IfCondition(LaunchConfiguration('nav2')),
        actions=[
            SetRemap('cmd_vel_smoothed', '/cmd_nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch',
                    'navigation_launch.py')),
                launch_arguments={'use_sim_time': 'false',
                                  'params_file': nav_params,
                                  'use_composition': 'False'}.items()),
        ])

    # Детектор обрыва: кромка платформы по глубине. На железе рельсов
    # может не быть вовсе — маску рельсов он берёт по желанию, и без неё
    # просто не красит ничего зелёным, а кромку находит как обычно.
    dropoff = Node(package='maze_nav', executable='dropoff_detector.py',
                   output='screen',
                   condition=IfCondition(LaunchConfiguration('dropoff')),
                   remappings=[('depth', depth),
                               ('camera_info', info),
                               ('dropoff', '/dropoff/points'),
                               ('grid_cloud', '/dropoff/grid_cloud'),
                               ('rails/mask', '/rails/mask')])

    # Автомат заезда. По умолчанию выключен: он СРАЗУ начинает крутить
    # робота на месте, а потом едет, и включать его вместе с картой надо
    # осознанно. Рельсы берёт из накопленного слоя, а не из карты
    # препятствий: при сегментации в 2 Гц зелёного в карте почти не
    # остаётся, и вписывание колеи садится мимо — проверено в симуляторе,
    # колея выходила 261 мм вместо 545.
    entry = Node(package='maze_nav', executable='rail_entry.py',
                 output='screen',
                 condition=IfCondition(LaunchConfiguration('entry')),
                 remappings=[('odom', '/odom'), ('imu', '/imu/data'),
                             ('obstacles', '/rails/map')])

    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')])

    return LaunchDescription([
        DeclareLaunchArgument('imu', default_value='true',
                              description='брать ИНС с платы на Pico'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('imu_axes', default_value='x y z',
                              description='разворот осей датчика; подобрать '
                                          'помощником imu_axes.py'),
        DeclareLaunchArgument('gyro_bias', default_value='[0.0, 0.0, 0.0]',
                              description='смещение нуля гироскопа, рад/с; '
                                          'замеряется imu_bias.py и без него '
                                          'курс уходит на градусы в минуту'),
        DeclareLaunchArgument('accel_bias', default_value='[0.0, 0.0, 0.0]',
                              description='смещение нуля акселерометра, м/с²'),
        DeclareLaunchArgument('profile', default_value='848x480x30',
                              description='на порту USB2 нужно 640x480x15'),
        DeclareLaunchArgument('db', default_value='/datasets/robot_map.db',
                              description='файл карты; /datasets смонтирован '
                                          'из datasets/ в корне проекта'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='стирать карту на старте; false — '
                                          'продолжать в уже существующей'),
        DeclareLaunchArgument('dropoff', default_value='false',
                              description='искать кромку площадки по '
                                          'глубине; нужен наклон камеры '
                                          '30-35 градусов'),
        DeclareLaunchArgument('cam_imu', default_value='false',
                              description='включить собственную ИНС D435i; '
                                          'нужна для калибровки угла камеры'),
        DeclareLaunchArgument('cloud', default_value='false',
                              description='живое плотное облако с камеры '
                                          'для просмотра в RViz; поток '
                                          'немалый, включать по нужде'),
        DeclareLaunchArgument('force_2d', default_value='false',
                              description='держать одометрию и карту в трёх '
                                          'степенях свободы: x, y, курс. '
                                          'Крен, тангаж и высота обнуляются — '
                                          'для ровного настила это правда, а '
                                          'не упрощение'),
        DeclareLaunchArgument('entry', default_value='false',
                              description='автомат заезда на рельсы; '
                                          'ОСТОРОЖНО: сразу начинает крутить '
                                          'робота и ехать'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='планировщик; скорость уходит '
                                          'в /cmd_nav'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rsp, camera, pico, madgwick, odom_imu, odom_plain, entry,
        slam_plain, slam_edge, dropoff, nav2, rviz,
    ])
