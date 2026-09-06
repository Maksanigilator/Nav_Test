"""Запуск TurtleBot3 в произвольном лабиринте.

    ros2 launch maze_nav maze.launch.py
    ros2 launch maze_nav maze.launch.py world:=maze2.world
    ros2 launch maze_nav maze.launch.py world:=maze2.world x_pose:=-3.75 y_pose:=-3.75

Имя мира ищется в share/maze_nav/worlds/ — то есть в том, что установил
colcon build. Можно передать и абсолютный путь.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Полный путь к миру: если передали просто имя — ищем в worlds/ пакета
    world_path = PathJoinSubstitution([pkg_maze, 'worlds', world])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze1.world',
                              description='имя .world в share/maze_nav/worlds/'),
        # Стартовая клетка лабиринта 6x6 с клеткой 1.5 м — левый нижний угол
        DeclareLaunchArgument('x_pose', default_value='-3.75',
                              description='координата X точки спавна робота'),
        DeclareLaunchArgument('y_pose', default_value='-3.75',
                              description='координата Y точки спавна робота'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='запускать ли окно Gazebo (gzclient)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='брать время из симулятора, а не системное'),

        # Физика симуляции
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),

        # Окно симулятора — отключается gui:=false
        ExecuteProcess(
            cmd=['gzclient'],
            condition=IfCondition(gui),
            output='screen'),

        # Публикация TF по URDF робота
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        # Появление робота в мире
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ])
