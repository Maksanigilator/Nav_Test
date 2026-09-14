"""Nav2 для реального робота Frob. Симулятор не поднимается вообще.

ВНИМАНИЕ: запускать только с ноутбука. Этот файл ходит в nav2_bringup
ради ветки slam:=true (построение карты через slam_toolbox), а в образе
робота ни того, ни другого пакета нет — они тянут за собой Gazebo.
На борту робота — city.launch.py, он поднимает Nav2 через свой
nav2_frob.launch.py без этих зависимостей.

    # построить карту (slam_toolbox), робот ездит с клавиатуры
    ros2 launch maze_nav frob.launch.py slam:=true

    # ездить по готовой карте (AMCL)
    ros2 launch maze_nav frob.launch.py map:=/root/ros_ws/src/maze_nav/maps/frob_room.yaml

Данные приходят с самого робота — там должен быть поднят его bringup:

    ros2 launch frob_bringup bringup.launch.py

Он даёт /scan, /scan/filtered, /odom, /odometry/filtered, TF от base_footprint
и приёмник /cmd_vel (geometry_msgs/Twist). Контейнер и робот должны совпадать
по ROS_DOMAIN_ID и RMW_IMPLEMENTATION — иначе узлы не увидят друг друга.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')

    # nav2_bringup считает slam питоновским выражением, и 'true' в нижнем
    # регистре роняет запуск с "name 'true' is not defined". Приводим любое
    # написание (true/True/1/yes) к тому, что понимает Python.
    slam_bool = PythonExpression(
        ['"True" if "', slam, '".lower() in ("true", "1", "yes") else "False"'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_maze, 'params', 'frob_params.yaml'),
            description='конфиг Nav2 под реального робота'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='путь к .yaml карты; при slam:=true не нужен'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true — строить карту slam_toolbox вместо локализации по готовой'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='окно RViz: им же ставится начальная поза и цель'),

        # Время системное: /clock на реальном роботе никто не публикует,
        # поэтому use_sim_time здесь всегда False — в отличие от maze.launch.py.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'slam': slam_bool,
                'params_file': params_file,
                'use_sim_time': 'False',
                'autostart': 'True',
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(rviz),
            output='screen'),
    ])
