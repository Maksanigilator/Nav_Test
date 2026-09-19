"""Просмотр URDF робота в RViz — без симулятора и без физики.

Пакета urdf_launch в образе нет, поэтому launch свой. Нужен минимум:
xacro разворачивает описание, robot_state_publisher строит TF по нему,
joint_state_publisher выдаёт нулевые углы для всех шарниров (иначе TF
для вращательных не построится), rviz2 показывает.

    ros2 launch maze_nav display_robot.launch.py
    ros2 launch maze_nav display_robot.launch.py model:=другой.urdf.xacro

joint_state_publisher_gui намеренно не используется: у этого робота
52 вращательных шарнира (4 привода плюс 48 роликов меканума), и панель
из 52 ползунков бесполезна. Если понадобится покрутить колесо руками,
проще опубликовать /joint_states вручную.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('maze_nav')
    model = LaunchConfiguration('model')
    rviz_cfg = os.path.join(pkg, 'rviz', 'robot.rviz')

    # ParameterValue(..., value_type=str) обязателен: без него launch пытается
    # угадать тип результата Command и портит XML описания.
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', model])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='agro_robot.urdf.xacro',
                              description='имя файла в share/maze_nav/urdf/'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        # Без него вращательные шарниры остаются без углов, и TF обрывается
        # на base_link — робот в RViz будет без колёс.
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             output='screen'),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_cfg]),
    ])
