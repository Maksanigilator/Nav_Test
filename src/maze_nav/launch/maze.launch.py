"""Запуск TurtleBot3 в произвольном лабиринте (Gazebo Harmonic / gz sim).

    ros2 launch maze_nav maze.launch.py
    ros2 launch maze_nav maze.launch.py world:=maze2.world
    ros2 launch maze_nav maze.launch.py world:=maze2.world x_pose:=-3.75 y_pose:=-3.75

Имя мира ищется в share/maze_nav/worlds/ — то есть в том, что установил
colcon build. Можно передать и абсолютный путь.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Полный путь к миру: если передали просто имя — ищем в worlds/ пакета
    world_path = PathJoinSubstitution([pkg_maze, 'worlds', world])

    # gz sim резолвит model:// только через GZ_SIM_RESOURCE_PATH.
    # Меши робота лежат в моделях turtlebot3_gazebo, поэтому путь нужен
    # даже когда сам робот спавнится по абсолютному пути к model.sdf.
    gz_resources = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg_tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg_maze, 'models')),
    ]

    return LaunchDescription(gz_resources + [
        DeclareLaunchArgument('world', default_value='maze1.world',
                              description='имя .world в share/maze_nav/worlds/'),
        # Стартовая клетка лабиринта 6x6 с клеткой 1.5 м — левый нижний угол
        DeclareLaunchArgument('x_pose', default_value='-3.75',
                              description='координата X точки спавна робота'),
        DeclareLaunchArgument('y_pose', default_value='-3.75',
                              description='координата Y точки спавна робота'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='запускать ли окно Gazebo (gz sim -g)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='брать время из симулятора, а не системное'),

        # Физика симуляции. Раньше это был gzserver с подгрузкой
        # libgazebo_ros_init/_factory; в gz sim плагины объявлены внутри
        # .world, а связь с ROS даёт ros_gz_bridge (поднимается ниже,
        # вместе со спавном робота).
        #   -r запустить сразу, не на паузе; -s только сервер; -v2 уровень логов
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r -s -v2 ', world_path],
                              'on_exit_shutdown': 'true'}.items()),

        # Окно симулятора — отключается gui:=false
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 '}.items(),
            condition=IfCondition(gui)),

        # Публикация TF по URDF робота
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # Появление робота в мире + мост gz<->ROS (/clock, /scan, /odom, /cmd_vel, /tf)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ])
