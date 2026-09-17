"""Контейнерная половина этапа 3: стек навигации для сцены в Isaac Sim.

Симулятор здесь НЕ запускается — он живёт на хосте отдельным процессом
(isaac/tb3_sim.py) и общается с контейнером по DDS. Этот файл поднимает только
потребителей данных:

    Isaac (хост)                         контейнер (этот launch)
    ─────────────                        ────────────────────────
    /clock                          ──>  всё на use_sim_time
    /camera/image_raw                ─┐
    /camera/depth/image_raw          ─┼─> rtabmap_odom/rgbd_odometry ──> odom->base
    /camera/camera_info              ─┘            │
    /imu/ideal  ──> imu_noise ──> /imu ──> madgwick ──> /imu/data ──┤
                                                                   v
                                                        rtabmap_slam/rtabmap
                                                          ├──> /map
                                                          └──> map->odom
    <──  /cmd_vel                        Nav2

Почему отдельный файл, а не аргумент у rgbd_sim.launch.py. Тот поднимает
Gazebo, мост ros_gz и спавнит робота — в варианте с Isaac ничего этого нет.
Разделение оставляет гейзебовский этап 2 нетронутым и повторяемым: он остаётся
опорной точкой, с которой сравнивается ATE этапа 3, и ломать его нельзя.

Дерево TF даёт robot_state_publisher по URDF TurtleBot3 — тому же самому, из
которого импортирован робот в Isaac (проверено: имена линков и origin суставов
в turtlebot3_gazebo и turtlebot3_description совпадают). Isaac в /tf не пишет
НИЧЕГО: два источника одного трансформа ломают дерево молча, и проект на этом
уже спотыкался дважды.

    ros2 launch maze_nav isaac_nav.launch.py
    ros2 launch maze_nav isaac_nav.launch.py nav2:=true rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

BASE_FRAME = 'base_footprint'

# Имена ровно те, что публикует сцена (isaac/tb3_sim.py). Совпадают с теми,
# что давал мост Gazebo, поэтому ремаппинги ниже одинаковы для обоих стендов.
RGB_TOPIC = '/camera/image_raw'
RGB_INFO_TOPIC = '/camera/camera_info'
# Сцена публикует ИДЕАЛЬНУЮ глубину, а поведение настоящего D435 ей придаёт
# scripts/depth_noise.py: шум в диспаратности (отсюда рост ошибки как z^2),
# дыры по нехватке текстуры и по границам дальности. Встроенная модель
# стереодатчика у Isaac есть, но её результат не доходит до ROS —
# ROS2CameraHelper жёстко читает переменную DistanceToImagePlane,
# подробности в комментарии в isaac/tb3_sim.py.
DEPTH_IDEAL_TOPIC = '/camera/depth/ideal'
DEPTH_TOPIC = '/camera/depth/image_raw'

# Isaac публикует ИДЕАЛЬНУЮ инерциалку: у его класса IMUSensor нет ни одного
# параметра шума, тогда как в модели Gazebo шум задан (гироскоп 2e-4 рад/с,
# акселерометр 1.7e-2 м/с^2). Шум добавляет scripts/imu_noise.py, иначе
# симулятор оказался бы легче реальности — против смысла этапа.
IMU_IDEAL_TOPIC = '/imu/ideal'
IMU_TOPIC = '/imu'


def generate_launch_description():
    pkg_maze = get_package_share_directory('maze_nav')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')

    # Тот же URDF, из которого импортирован робот в Isaac. В версии
    # turtlebot3_gazebo подстановки ${namespace} уже раскрыты, а геометрия
    # совпадает с turtlebot3_description до последнего origin — проверено
    # сравнением файлов.
    urdf_path = os.path.join(pkg_tb3, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_path, 'r') as f:
        ROBOT_DESCRIPTION = f.read()

    nav2 = LaunchConfiguration('nav2')
    rviz = LaunchConfiguration('rviz')
    viz = LaunchConfiguration('viz')
    localization = LaunchConfiguration('localization')
    delete_db = LaunchConfiguration('delete_db')
    database_path = LaunchConfiguration('database_path')

    bt_dir = os.path.join(pkg_maze, 'behavior_trees')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_maze, 'params', 'maze_params_rgbd.yaml'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml':
                os.path.join(bt_dir, 'navigate_to_pose_rgbd.xml'),
            'default_nav_through_poses_bt_xml':
                os.path.join(bt_dir, 'navigate_through_poses_rgbd.xml'),
            # Nav2 в этой сборке настроен на TwistStamped — так устроен мост
            # TurtleBot3 в Jazzy, и для этапа 2 это правильно. А узел
            # ROS2SubscribeTwist в Isaac принимает обычный Twist: выделенного
            # узла под stamped в мосте Isaac нет вовсе. Снимаем stamped ЗДЕСЬ,
            # а не в yaml: файл общий с этапом 2, и правка в нём сломала бы
            # гейзебовский стенд.
            'enable_stamped_cmd_vel': 'false',
        },
        convert_types=True)

    # Параметры RTAB-Map — те же, что в rgbd_sim.launch.py. Дублирование
    # намеренное: связывать два стенда общим кодом значит рисковать сломать
    # опорный этап 2 правкой ради этапа 3.
    rtabmap_common = {
        'frame_id': BASE_FRAME,
        'subscribe_depth': True,
        'approx_sync': True,
        'wait_for_transform': 0.5,
        'Reg/Force3DoF': 'true',
        'Optimizer/GravitySigma': '0.3',
    }

    remappings = [
        ('rgb/image', RGB_TOPIC),
        ('rgb/camera_info', RGB_INFO_TOPIC),
        ('depth/image', DEPTH_TOPIC),
        ('imu', '/imu/data'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('nav2', default_value='false',
                              description='поднимать ли стек Nav2'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('viz', default_value='false',
                              description='GUI rtabmap_viz (нужен X11)'),
        DeclareLaunchArgument('database_path',
                              default_value='/datasets/rtabmap_isaac.db'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='ехать по УЖЕ СОБРАННОЙ карте'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='стирать базу RTAB-Map при старте'),

        # Обязательно: время берётся из /clock, который публикует Isaac.
        # Симулятор идёт быстрее реального времени, и на системных часах
        # ничего не сойдётся.
        SetParameter(name='use_sim_time', value=True),

        # Дерево TF по URDF. Единственный источник base_footprint -> всё
        # остальное, включая camera_rgb_optical_frame и imu_link.
        #
        # Узел поднимаем САМИ, а не через штатный
        # turtlebot3_gazebo/launch/robot_state_publisher.launch.py, и вот
        # почему. Там frame_prefix собирается выражением
        #     PythonExpression(["'", frame_prefix, "/'"])
        # и при пустом префиксе (значение по умолчанию) даёт '/'. Этот слэш
        # приклеивается ко ВСЕМ именам, и в /tf_static уезжают
        # "/base_footprint", "/base_link" и так далее. RTAB-Map запрашивает
        # "base_footprint" без слэша и падает с
        #     "base_footprint" passed to lookupTransform ... does not exist
        # причём одометрия при этом молчит, а кадры идут — симптом,
        # неотличимый от сломанной камеры.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': ROBOT_DESCRIPTION,
                          'frame_prefix': ''}]),

        # ── Узел шума глубины ОТКЛЮЧЁН ────────────────────────────────
        # Модель воспроизводит паспортный разброс D435 верно по величине
        # (проверено замером: 1 см на 1 м, 6 см на 2.5 м, 32 см на 6 м —
        # совпадает с теорией по всему диапазону), но НЕВЕРНО по временной
        # структуре, и из-за этого ломает одометрию сильнее, чем ломала бы
        # реальная камера. Подробный разбор — в шапке scripts/depth_noise.py.
        #
        # Замерено на прогоне в офисе: 15 потерь одометрии, при отказах
        # выживает 4.7 инлайера из 71 совпадения (7%). Признаки при этом
        # сопоставляются прекрасно — рушится именно геометрия.
        #
        # Пока не исправлено, сцена отдаёт глубину прямо в DEPTH_TOPIC,
        # а этот узел не поднимается. Чтобы вернуть: раскомментировать блок
        # ниже и поставить DEPTH_TOPIC = '/camera/depth/ideal'
        # в isaac/tb3_sim.py.
        #
        # Node(package='maze_nav', executable='depth_noise.py', output='screen',
        #      parameters=[{'input_topic': DEPTH_IDEAL_TOPIC,
        #                   'output_topic': DEPTH_TOPIC,
        #                   'rgb_topic': RGB_TOPIC,
        #                   'info_topic': RGB_INFO_TOPIC}]),

        # Шум в инерциалку: /imu/ideal -> /imu. Дальше по течению всё так же,
        # как на этапе 2, где /imu приходил уже зашумлённым из модели Gazebo.
        Node(package='maze_nav', executable='imu_noise.py', output='screen',
             parameters=[{'input_topic': IMU_IDEAL_TOPIC,
                          'output_topic': IMU_TOPIC}]),

        # Кватернион ориентации. Isaac умеет отдать его готовым, но это
        # истинное значение из симулятора: на роботе его взяться неоткуда,
        # там ориентацию выводит именно этот фильтр.
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False,
                          'world_frame': 'enu'}],
             remappings=[('imu/data_raw', IMU_TOPIC)]),

        Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
             parameters=[rtabmap_common, {
                 'wait_imu_to_init': True,
                 # Автосброс одометрии после N подряд неудачных регистраций.
                 # По умолчанию параметр равен нулю, то есть сброс ВЫКЛЮЧЕН:
                 # сорвавшись однажды, визуальная одометрия остаётся мёртвой
                 # навсегда, даже когда текстура вернулась в кадр. Проверено:
                 # робот упёрся в гладкую стену склада, отъехал, снова видит
                 # резкость 85 и валидную глубину — а quality остаётся 0.
                 #
                 # На этапе 2 в turtlebot3_house этого не всплыло: там потерь
                 # было ноль за четыре прогона, и параметр не понадобился.
                 # На складе гладкие стены есть, и без сброса любой заезд
                 # в тупик хоронит прогон целиком.
                 #
                 # 15 кадров при 30 Гц — это полсекунды отказов до сброса.
                 # Значение строкой: RTAB-Map разбирает свои параметры
                 # собственным конфигом, число вместо строки роняет узел.
                 'Odom/ResetCountdown': '15',
             }],
             remappings=remappings),

        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=[rtabmap_common, {
                 'database_path': database_path,
                 'subscribe_odom_info': True,
                 'Grid/FromDepth': 'true',
                 'Grid/RayTracing': 'true',
                 'Grid/MaxGroundHeight': '0.05',
                 'Grid/MaxObstacleHeight': '1.0',
                 'RGBD/CreateOccupancyGrid': 'true',
                 # Параметры RTAB-Map принимаются только строками, поэтому
                 # value_type=str обязателен: иначе launch сам приведёт
                 # "false" к bool и узел упадёт на InvalidParameterTypeException.
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
             remappings=remappings,
             arguments=[PythonExpression(
                 ["'-d' if '", delete_db, "'.lower() == 'true' and '",
                  localization, "'.lower() != 'true' else ''"])]),

        Node(package='rtabmap_viz', executable='rtabmap_viz', output='screen',
             parameters=[rtabmap_common], remappings=remappings,
             condition=IfCondition(viz)),

        # Облако для костмапов считаем ЗДЕСЬ, а не публикуем из Isaac.
        # На мосте Gazebo облако весило 9.77 МБ на сообщение и роняло частоту
        # всех топиков; в Isaac мы по той же причине берём type=depth,
        # а не depth_pcl.
        Node(package='rtabmap_util', executable='point_cloud_xyz', output='screen',
             parameters=[{'approx_sync': True, 'decimation': 4,
                          'max_depth': 4.0, 'voxel_size': 0.05}],
             remappings=[('depth/image', DEPTH_TOPIC),
                         ('depth/camera_info', RGB_INFO_TOPIC)]),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg_maze, 'rviz', 'rgbd_nav.rviz')],
             condition=IfCondition(rviz)),

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
