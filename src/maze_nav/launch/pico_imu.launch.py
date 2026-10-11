#!/usr/bin/env python3
"""Только ИНС на Pico: поток с MPU6050 в топики, без камеры и без карты.

Нужен, чтобы проверить сам датчик отдельно от всего остального: видно ли
его, с какой частотой идут выборки, куда смотрят оси, не врёт ли масштаб.
Смешивать эту проверку со съёмкой карты нельзя — при отказе непонятно, кто
виноват, камера или плата.

Публикуется два топика:

* ``/imu/data_raw`` — сырые ускорения и угловые скорости прямо с датчика;
* ``/imu/data`` — то же плюс ориентация, посчитанная фильтром Маджвика.

Ориентация нужна RTAB-Map, а сырые данные — чтобы понять, исправен ли
датчик: по кватерниону этого уже не видно.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from typing import List


def generate_launch_description():
    port = LaunchConfiguration('port')
    axes = LaunchConfiguration('axes')
    filt = LaunchConfiguration('filter')

    reader = Node(
        package='maze_nav', executable='pico_imu_node.py',
        name='pico_imu', output='screen',
        parameters=[{'port': port,
                     'axes': axes,
                     'frame': LaunchConfiguration('frame'),
                     'frame_id': LaunchConfiguration('frame_id'),
                     'gyro_bias': ParameterValue(
                         LaunchConfiguration('gyro_bias'),
                         value_type=List[float]),
                     'accel_bias': ParameterValue(
                         LaunchConfiguration('accel_bias'),
                         value_type=List[float])}],
        remappings=[('imu/data_raw', '/imu/data_raw')])

    # Фильтр ориентации. Отключаемый: если на плате прошивка без
    # гироскопа, считать по ней ориентацию нечем, и фильтр только
    # намусорит в топик.
    madgwick = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
        name='imu_filter', output='screen',
        condition=IfCondition(filt),
        parameters=[{'use_mag': False, 'world_frame': 'enu',
                     'publish_tf': False}],
        remappings=[('imu/data_raw', '/imu/data_raw'),
                    ('imu/data', '/imu/data')])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('axes', default_value='x y z',
                              description="разворот осей датчика в оси ROS, "
                                          "например '-y x z'"),
        DeclareLaunchArgument('frame', default_value='auto',
                              description='формат кадра: auto, full '
                                          '(акселерометр и гироскоп) или '
                                          'accel (только акселерометр)'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('gyro_bias', default_value='[0.0, 0.0, 0.0]',
                              description='смещение нуля гироскопа, рад/с; '
                                          'замеряется imu_bias.py'),
        DeclareLaunchArgument('accel_bias', default_value='[0.0, 0.0, 0.0]',
                              description='смещение нуля акселерометра, м/с²'),
        DeclareLaunchArgument('filter', default_value='true',
                              description='считать ориентацию фильтром '
                                          'Маджвика'),
        reader, madgwick,
    ])
