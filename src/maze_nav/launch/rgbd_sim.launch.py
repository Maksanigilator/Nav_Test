"""TurtleBot3 с камерой D435 в Gazebo: навигация БЕЗ лидара и БЕЗ колёсной одометрии.

Целевая архитектура этапа 2. Что кто публикует:

    gz rgbd_camera ──> rtabmap_odom/rgbd_odometry ──> odom->base_footprint
           │                        │
           │                        └──> /odom
           ├──> imu_filter_madgwick ──> /imu/data ──┐
           └──> rtabmap_slam/rtabmap <──────────────┘
                        ├──> /map (OccupancyGrid) ──> static_layer костмапа
                        └──> map->odom

AMCL и map_server не запускаются вовсе: карту и локализацию даёт RTAB-Map.
Колёсной одометрии нет — в мосте намеренно отсутствуют "odom" и "/tf"
от плагина DiffDrive, см. params/waffle_d435_bridge.yaml.

    ros2 launch maze_nav rgbd_sim.launch.py
    ros2 launch maze_nav rgbd_sim.launch.py gui:=false viz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

# Фрейм, в котором едут кадры камеры. Оптическая конвенция: Z вперёд, X вправо,
# Y вниз. Связь с base_footprint публикует robot_state_publisher по URDF.
CAMERA_OPTICAL_FRAME = 'camera_rgb_optical_frame'
BASE_FRAME = 'base_footprint'

RGB_TOPIC = '/camera/image_raw'
RGB_INFO_TOPIC = '/camera/camera_info'
DEPTH_TOPIC = '/camera/depth/image_raw'


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_ros_gz = get_package_share_directory('ros_gz_sim')

    nav2 = LaunchConfiguration('nav2')
    rviz = LaunchConfiguration('rviz')
    delete_db = LaunchConfiguration('delete_db')
    localization = LaunchConfiguration('localization')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    viz = LaunchConfiguration('viz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    database_path = LaunchConfiguration('database_path')

    bridge_config = os.path.join(pkg_maze, 'params', 'waffle_d435_bridge.yaml')

    # bt_navigator принимает только абсолютный путь к дереву, а в yaml его
    # не записать: каталог install зависит от машины. Поэтому подставляем
    # через RewrittenYaml — штатный для Nav2 приём.
    bt_dir = os.path.join(pkg_maze, 'behavior_trees')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_maze, 'params', 'maze_params_rgbd.yaml'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml':
                os.path.join(bt_dir, 'navigate_to_pose_rgbd.xml'),
            'default_nav_through_poses_bt_xml':
                os.path.join(bt_dir, 'navigate_through_poses_rgbd.xml'),
        },
        convert_types=True)
    robot_sdf = os.path.join(pkg_maze, 'models', 'turtlebot3_waffle_d435', 'model.sdf')

    # Общие параметры RTAB-Map для одометрии и SLAM.
    # Параметры самой библиотеки задаются ТОЛЬКО строками, даже числа и флаги:
    # она разбирает их своим конфигом, а не через rclcpp.
    rtabmap_common = {
        'frame_id': BASE_FRAME,
        'subscribe_depth': True,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        # Робот наземный и камера закреплена жёстко — крен и тангаж запрещены.
        # Это заметно стабилизирует решение по сравнению с полными 6 DoF.
        'Reg/Force3DoF': 'true',
        # Гравитация от IMU даёт приор на ориентацию: без неё при потере
        # признаков одометрия уплывает по крену.
        'Optimizer/GravitySigma': '0.3',
    }

    remappings = [
        ('rgb/image', RGB_TOPIC),
        ('rgb/camera_info', RGB_INFO_TOPIC),
        ('depth/image', DEPTH_TOPIC),
        ('imu', '/imu/data'),
    ]

    gz_resources = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg_tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(pkg_maze, 'models')),
    ]

    return LaunchDescription(gz_resources + [
        DeclareLaunchArgument('world', default_value='turtlebot3_house.world',
                              description='мир; house выбран потому, что у него '
                                          'есть настоящие текстуры — в сером '
                                          'лабиринте визуальный SLAM слеп'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('viz', default_value='false',
                              description='GUI rtabmap_viz (нужен X11)'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='0.5'),
        DeclareLaunchArgument('database_path',
                              default_value='/datasets/rtabmap_sim.db'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='поднимать ли стек Nav2'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz с 3D-картой и целью навигации'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='ехать по УЖЕ СОБРАННОЙ карте вместо её построения. '
                        'Карта не достраивается, RTAB-Map только определяет '
                        'в ней своё место. База при этом не стирается.'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='стирать базу RTAB-Map при старте. '
                                          'true для чистого прогона, false '
                                          'чтобы продолжить накопленную карту'),

        SetParameter(name='use_sim_time', value=True),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r -s -v2 ',
                            PathJoinSubstitution([pkg_tb3, 'worlds', world])],
                'on_exit_shutdown': 'true'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 '}.items(),
            condition=IfCondition(gui)),

        # TF по URDF: base_footprint -> ... -> camera_rgb_optical_frame.
        # Именно отсюда RTAB-Map узнаёт, где стоит камера относительно базы.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),

        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', 'turtlebot3_waffle_d435',
                        '-file', robot_sdf,
                        '-x', x_pose, '-y', y_pose, '-z', '0.01']),

        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': bridge_config}]),

        # Сырой IMU из gz -> ориентация. RTAB-Map ждёт кватернион,
        # а gz отдаёт только угловые скорости и ускорения.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False,
                          'world_frame': 'enu'}],
             remappings=[('imu/data_raw', '/imu')]),

        Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
             parameters=[rtabmap_common, {'wait_imu_to_init': True}],
             remappings=remappings),

        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=[rtabmap_common, {
                 'database_path': database_path,
                 'subscribe_odom_info': True,
                 # Сетку занятости строим из глубины — она пойдёт в static_layer
                 # костмапа вместо карты из map_server.
                 'Grid/FromDepth': 'true',
                 'Grid/RayTracing': 'true',
                 'Grid/MaxGroundHeight': '0.05',
                 'Grid/MaxObstacleHeight': '1.0',
                 'RGBD/CreateOccupancyGrid': 'true',
                 # Режим локализации. Параметры RTAB-Map принимаются только
                 # строками, поэтому подставляем 'true'/'false' текстом.
                 #   IncrementalMemory=false — не добавлять новые узлы в карту;
                 #   InitWMWithAllNodes=true — загрузить всю карту в рабочую
                 #     память сразу, иначе робот узнает только те места,
                 #     куда дойдёт по графу от точки старта.
                 # ParameterValue(..., value_type=str) обязателен. Без него
                 # launch видит строку "false" и САМ приводит её к bool, а
                 # RTAB-Map объявляет эти параметры строковыми и падает с
                 # InvalidParameterTypeException ещё до старта.
                 'Mem/IncrementalMemory': ParameterValue(
                     PythonExpression(
                         ["'false' if '", localization,
                          "'.lower() == 'true' else 'true'"]),
                     value_type=str),
                 'Mem/InitWMWithAllNodes': ParameterValue(
                     PythonExpression(
                         ["'true' if '", localization,
                          "'.lower() == 'true' else 'false'"]),
                     value_type=str),
             }],
             # Флаг -d стирает базу при старте. Он нужен, чтобы прогоны
             # не наслаивались, но по умолчанию молча уничтожает накопленную
             # карту при следующем запуске. Поэтому вынесен в аргумент:
             #   delete_db:=false — продолжить накопленную карту.
             # Сохранить карту до перезапуска можно сервисом /rtabmap/backup.
             remappings=remappings,
             arguments=[PythonExpression(
                 ["'-d' if '", delete_db, "'.lower() == 'true' and '",
                  localization, "'.lower() != 'true' else ''"])]),

        Node(package='rtabmap_viz', executable='rtabmap_viz', output='screen',
             parameters=[rtabmap_common], remappings=remappings,
             condition=IfCondition(viz)),

        # Облако точек для костмапов. Считается здесь, а НЕ мостится из gz:
        # штатный camera/points весит 9.77 МБ на сообщение и даёт 142 МБ/с,
        # на чём мост захлёбывается и роняет частоту всех остальных топиков.
        # Здесь же прореживание вчетверо и отсечка по дальности делают облако
        # на два порядка легче, а костмапу большего и не нужно.
        Node(package='rtabmap_util', executable='point_cloud_xyz', output='screen',
             parameters=[{'approx_sync': True, 'decimation': 4,
                          'max_depth': 4.0, 'voxel_size': 0.05}],
             remappings=[('depth/image', DEPTH_TOPIC),
                         ('depth/camera_info', RGB_INFO_TOPIC)]),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg_maze, 'rviz', 'rgbd_nav.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(rviz)),

        # Nav2. map_server и amcl НЕ поднимаются: карту даёт RTAB-Map,
        # локализацию — он же через map->odom.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'),
                             'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
            }.items(),
            condition=IfCondition(nav2)),
    ])
