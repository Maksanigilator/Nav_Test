"""RTAB-Map на записи с реальной RealSense.

Вход — ROS 2 bag, полученный из родной записи librealsense скриптом
scripts/realsense_bag_to_ros2.py. Напрямую .bag от librealsense сюда не годится:
внутри у него структура /device_0/sensor_N/..., а не ROS-топики.

Почему не через `realsense2_camera rosbag_filename`, хотя так было бы короче:
с включённым выравниванием глубины обёртка выдаёт ~1 Гц вместо 30 и при этом
играет запись в реальном времени, то есть молча теряет кадры. Между обработанными
набегает по два метра движения, и одометрия перестаёт сходиться. Подробности
и замеры — в RTABMAP.md.

    ros2 launch maze_nav rtabmap_realsense_bag.launch.py \
        bag:=/datasets/rail_run play:=true rate:=0.5 viz:=true

Ground truth в таких записях нет, поэтому ошибку траектории посчитать не с чем:
оценка только качественная — держится ли одометрия, срабатывают ли замыкания,
не разъезжается ли карта.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

# Топики, которые пишет наш конвертер. Глубина выровнена в кадр цвета,
# поэтому делит с ним и интринсики, и frame_id — отдельный camera_info не нужен.
RGB_TOPIC = '/camera/color/image_raw'
RGB_INFO_TOPIC = '/camera/color/camera_info'
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'

BASE_FRAME = 'camera_link'
OPTICAL_FRAME = 'camera_color_optical_frame'


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    viz = LaunchConfiguration('viz')
    play = LaunchConfiguration('play')
    rate = LaunchConfiguration('rate')
    database_path = LaunchConfiguration('database_path')

    params = [{
        'frame_id': BASE_FRAME,
        'subscribe_depth': True,
        'approx_sync': True,
        # Конвертер штампует цвет и глубину одним и тем же временем, поэтому
        # рассинхрона между ними быть не может. Порог узкий, чтобы поймать
        # ошибку в данных, а не молча склеить неподходящие кадры.
        'approx_sync_max_interval': 0.01,
        'wait_for_transform': 0.5,
        # Параметры библиотеки RTAB-Map принимаются ТОЛЬКО строками, даже
        # числа и флаги: она разбирает их своим конфигом, а не через rclcpp.
        'Odom/Strategy': '0',
        'Odom/ResetCountdown': '15',
        # Камеру несли в руках, крен и тангаж реальны — запрещать их нельзя.
        # На роботе с жёстким креплением это станет 'true'.
        'Reg/Force3DoF': 'false',
        'RGBD/CreateOccupancyGrid': 'true',
        'Grid/FromDepth': 'true',
        'Grid/RayTracing': 'true',
    }]

    remappings = [
        ('rgb/image', RGB_TOPIC),
        ('rgb/camera_info', RGB_INFO_TOPIC),
        ('depth/image', DEPTH_TOPIC),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='каталог ROS 2 bag с записью'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='GUI rtabmap_viz (нужен X11)'),
        DeclareLaunchArgument('play', default_value='false',
                              description='проигрывать bag самим, не во втором терминале'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='скорость проигрывания'),
        DeclareLaunchArgument('database_path',
                              default_value='/datasets/rtabmap_realsense.db',
                              description='куда класть базу RTAB-Map'),

        SetParameter(name='use_sim_time', value=True),

        # Камера в записи одна и неподвижна относительно базы, так что связь
        # статическая. Поворот -90° по двум осям — стандартный переход от
        # "X вперёд" (конвенция ROS) к "Z вперёд" (оптическая конвенция).
        Node(package='tf2_ros', executable='static_transform_publisher',
             output='screen',
             arguments=['0.0', '0.0', '0.0',
                        '-1.57079632679', '0.0', '-1.57079632679',
                        BASE_FRAME, OPTICAL_FRAME]),

        Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
             parameters=params, remappings=remappings),

        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=params + [{'database_path': database_path}],
             remappings=remappings, arguments=['-d']),

        Node(package='rtabmap_viz', executable='rtabmap_viz', output='screen',
             parameters=params, remappings=remappings,
             condition=IfCondition(viz)),

        # Задержка нужна: узлы поднимаются не мгновенно, а сообщения, пришедшие
        # до подписки, пропадут — и начало записи выпадет из карты.
        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
                output='screen')],
            condition=IfCondition(play)),
    ])
