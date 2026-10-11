from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('rail_stage1'))/'config/rails.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('database', default_value='/datasets/rail_registry/rails_platform.json'),
        DeclareLaunchArgument('map_id', default_value='platform'),
        DeclareLaunchArgument('approach_distance', default_value='0.90'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package='rail_stage1', executable='rail_finder', name='rail_finder',
             parameters=[LaunchConfiguration('config'),
                         {'database_path': LaunchConfiguration('database'),
                          'map_id': LaunchConfiguration('map_id'),
                          'approach_distance': ParameterValue(LaunchConfiguration('approach_distance'), value_type=float),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='rail_stage1', executable='rail_approach', name='rail_approach',
             parameters=[LaunchConfiguration('config'),
                         {'approach_distance': ParameterValue(LaunchConfiguration('approach_distance'), value_type=float),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(Path(get_package_share_directory('rail_stage1'))/'config/rails.rviz')],
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
