"""Только построение карты: slam_toolbox + RViz, без Nav2.

    ros2 launch maze_nav frob_slam.launch.py

Отличие от frob.launch.py slam:=true — там поднимается весь стек Nav2
(полтора десятка узлов, костмапы, планировщики), и весь их DDS-обмен идёт
по тому же Wi-Fi, что и данные с робота. Канал забивается, сканы теряются
("Message Filter dropping message ... queue is full"), и объекты внутри
комнаты прорабатываются заметно хуже стен.

При сборе карты Nav2 не нужен: робот едет телеопом. Здесь работают только
slam_toolbox и RViz.

Карта сохраняется как обычно, из соседнего терминала:
    ros2 run nav2_map_server map_saver_cli -f <путь>/frob_room
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
    pkg_slam = get_package_share_directory('slam_toolbox')

    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_maze, 'params', 'frob_params.yaml'),
            description='конфиг с секцией slam_toolbox'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='окно RViz для наблюдения за картой'),

        # Штатный launch самого slam_toolbox, а не голая Node: в Jazzy это
        # lifecycle-узел, и его нужно явно перевести в configure, затем в
        # activate. Запущенный как обычная Node он остаётся unconfigured —
        # не подписывается на скан и не публикует ни карту, ни map->odom.
        # Синхронный вариант выбран намеренно: он не пропускает сканы ради
        # темпа, а это важно для проработки мелких объектов.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_sync_launch.py')),
            launch_arguments={
                'slam_params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # Свой конфиг, а не nav2_default_view.rviz: тот тянет панель Nav2
            # и без запущенных controller_server/planner_server уходит в
            # бесконечное "Failed to load plugins. Retrying..."
            arguments=['-d', os.path.join(pkg_maze, 'rviz', 'frob_slam.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(rviz),
            output='screen'),
    ])
