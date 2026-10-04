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
from launch.substitutions import Command, LaunchConfiguration
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
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': prof,
            'rgb_camera.color_profile': prof,
            # ИНС камеры НЕ трогаем: её забирает ядро модулями hid_sensor_*,
            # и попытка открыть модуль движения роняет весь узел драйвера.
            # Угловую скорость даёт отдельная плата.
            'enable_gyro': 'false',
            'enable_accel': 'false',
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
    odom_params = {
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
    slam_common = dict(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        arguments=['--delete_db_on_start'])
    slam_base = {
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
                            'Grid/MaxGroundHeight': '0.03',
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
        DeclareLaunchArgument('db', default_value='/datasets/robot_map.db'),
        DeclareLaunchArgument('dropoff', default_value='false',
                              description='искать кромку площадки по '
                                          'глубине; нужен наклон камеры '
                                          '30-35 градусов'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='планировщик; скорость уходит '
                                          'в /cmd_nav'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rsp, camera, pico, madgwick, odom_imu, odom_plain,
        slam_plain, slam_edge, dropoff, nav2, rviz,
    ])
