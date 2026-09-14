"""Стек Nav2 для Frob — без `nav2_bringup`.

    ros2 launch maze_nav nav2_frob.launch.py map:=.../city.yaml

Зачем свой launch вместо штатного `bringup_launch.py`. Пакет
`nav2_bringup` по зависимостям тянет за собой симулятор целиком:
`nav2-minimal-tb3-sim`, `nav2-minimal-tb4-sim`, `ros-gz-sim`,
`ros-gz-bridge`, `slam-toolbox` и куски ros2_control. Метапакет
`navigation2` вдобавок тянет `nav2-rviz-plugins`, а с ними весь X-стек.
На роботе нет ни экрана, ни симулятора, и образ из-за этого распухал
вчетверо. Здесь узлы перечислены руками, поэтому в образ ставятся
только те пакеты Nav2, которые реально запускаются.

Отличия от штатного bringup, кроме отсутствия зависимости:
  * нет `route_server` и `docking_server` — ими не пользуемся, а в
    lifecycle-менеджере они лишние: он ждёт от них ответа при старте;
  * нет ветки с composition: узлы всегда отдельными процессами. Разница
    в паре сотен мегабайт ОЗУ, которых на Pi 5 хватает, зато падение
    одного узла видно сразу и по имени процесса;
  * нет namespace и remapping'ов /tf -> tf: они нужны только при
    запуске нескольких роботов в одном домене, у нас робот один;
  * параметры отдаются узлам как есть, без `RewrittenYaml` — в
    frob_params.yaml нет ни подстановок `$(var ...)`, ни ключа
    autostart, переписывать в нём нечего. Заодно не нужен `nav2_common`.

Цепочка скоростей та же, что в bringup, и менять её нельзя — на ней
завязаны параметры:
    controller_server -> /cmd_vel_nav -> velocity_smoother
        -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel -> робот
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Порядок важен: lifecycle-менеджер поднимает узлы именно в нём, и
# bt_navigator должен активироваться после серверов, к которым он ходит.
LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    def nav2_node(package, executable, name, extra_remaps=()):
        """Узел Nav2 с общим для всех набором настроек."""
        return Node(
            package=package, executable=executable, name=name,
            output='screen',
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=list(extra_remaps))

    return LaunchDescription([
        # Без этого вывод узлов идёт блоками по мере заполнения буфера,
        # и в логе непонятно, на чём именно всё встало.
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_maze, 'maps', 'city.yaml'),
            description='карта для map_server'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_maze, 'params', 'frob_params.yaml'),
            description='общий конфиг всех узлов Nav2'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='поднимать ли lifecycle-узлы сразу до active'),
        DeclareLaunchArgument('log_level', default_value='info'),

        # ── Локализация ───────────────────────────────────────────────
        # Путь к карте отдаётся отдельным параметром, а не правкой yaml:
        # так одна и та же строка params_file работает с любой картой.
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml}],
            arguments=['--ros-args', '--log-level', log_level]),
        nav2_node('nav2_amcl', 'amcl', 'amcl'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'autostart': autostart, 'node_names': LOCALIZATION_NODES}],
            arguments=['--ros-args', '--log-level', log_level]),

        # ── Навигация ─────────────────────────────────────────────────
        # cmd_vel -> cmd_vel_nav у трёх узлов не опечатка: сырую команду
        # контроллера нельзя пускать на робота мимо сглаживателя и
        # монитора столкновений, поэтому она уходит в промежуточный топик.
        nav2_node('nav2_controller', 'controller_server', 'controller_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
                  [('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor', 'collision_monitor'),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'autostart': autostart, 'node_names': NAVIGATION_NODES}],
            arguments=['--ros-args', '--log-level', log_level]),
    ])
