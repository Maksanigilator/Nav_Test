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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
        }.items())

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
        condition=IfCondition(imu),
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
        realsense, imu_filter, pico, pico_tf, odom_imu, odom_plain, slam_fresh, slam_keep, viz,
    ])
