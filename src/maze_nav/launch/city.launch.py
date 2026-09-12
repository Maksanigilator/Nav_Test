"""Полигон «Город РТК» целиком: Nav2 по карте, RViz, маршруты и редактор.

    ros2 launch maze_nav city.launch.py

Что поднимается:
  * Nav2 с картой полигона (city.yaml) и AMCL — обычная навигация;
  * RViz — видно карту, скан, костмапы и текущую цель Nav2;
  * route_follower — ведёт робота по графу точка за точкой;
  * route_editor — окно с точками, кнопкой «Старт» и живой отрисовкой.

Робот при этом должен быть поднят своим bringup:
    ros2 launch frob_bringup bringup.launch.py

Начальную позу задавать руками не нужно: по кнопке «Старт» узел сам
публикует её в точке номер 1, у въезда на полигон.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')
    graph = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')
    editor = LaunchConfiguration('editor')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_maze, 'maps', 'city.yaml'),
            description='карта полигона'),
        DeclareLaunchArgument(
            'graph', default_value=os.path.join(pkg_maze, 'maps', 'city_routes.yaml'),
            description='граф маршрутов по полосам'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_maze, 'params', 'frob_params.yaml'),
            description='конфиг Nav2 под реального робота'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('editor', default_value='true'),

        # Nav2 по готовой карте. use_sim_time False — время системное,
        # /clock на реальном роботе никто не публикует.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'slam': 'False',
                'params_file': params_file,
                'use_sim_time': 'False',
                'autostart': 'True',
            }.items()),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(rviz), output='screen'),

        Node(
            package='maze_nav', executable='route_follower.py', name='route_follower',
            arguments=['--graph', graph], output='screen'),

        # Редактор — обычное окно Qt, а не ROS-узел: ROS-часть живёт у него
        # в отдельном потоке и сама заводит узел. Поэтому имя здесь не
        # задаём — launch_ros иначе подмешал бы --ros-args, на которых
        # разбор аргументов спотыкается.
        Node(
            package='maze_nav', executable='route_editor.py',
            arguments=['--map', map_yaml, '--graph', graph],
            condition=IfCondition(editor), output='screen'),
    ])
