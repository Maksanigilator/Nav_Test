"""RTAB-Map на записи RealSense с одометрией по ГЕОМЕТРИИ, а не по картинке.

Зачем отдельный вариант. На записи `rail_20260613` визуальная одометрия
не работает: 91% цветных кадров смазаны (медиана резкости 45 при пороге ~100),
признаки между кадрами не повторяются, `quality=0`. При этом **глубина цела** —
измерено по всей записи: валидных пикселей 84% медианно, и корреляция качества
глубины со смазом цвета равна -0.09, то есть отсутствует. Смаз бьёт по цветному
сенсору (роллинговый затвор), а глубина считается с пары ИК-сенсоров
с глобальным затвором.

Отсюда идея: не искать особые точки в смазанной картинке, а совмещать облака
точек по форме. Этим занимается `rtabmap_odom/icp_odometry` — она изображение
не использует вообще.

Ограничение, о котором надо помнить: ICP требует, чтобы в поле зрения была
геометрия, задающая все шесть степеней свободы. На участках записи, где в кадре
только ровный пол (кадры около 700 и 1000), облако вырождается в плоскость,
и решение «скользит» вдоль неё. Там ICP тоже поплывёт, но по другой причине,
чем визуальная одометрия.

    ros2 launch maze_nav rtabmap_realsense_icp.launch.py \\
        bag:=/datasets/rail_20260613 play:=true rate:=0.5 viz:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/color/camera_info'
RGB_TOPIC = '/camera/color/image_raw'
BASE_FRAME = 'camera_link'
OPTICAL_FRAME = 'camera_color_optical_frame'


def generate_launch_description():
    bag = LaunchConfiguration('bag')
    viz = LaunchConfiguration('viz')
    play = LaunchConfiguration('play')
    rate = LaunchConfiguration('rate')
    database_path = LaunchConfiguration('database_path')

    # Параметры RTAB-Map — только строками, это требование библиотеки.
    icp_params = [{
        'frame_id': BASE_FRAME,
        'wait_for_transform': 0.5,
        # Прореживание облака: без него ICP захлебнётся на 300 тысячах точек.
        'Icp/VoxelSize': '0.05',
        # Точка-к-плоскости сходится заметно лучше на сценах с полом и стенами,
        # чем классическое точка-к-точке.
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        # Правило из официального примера lidar3d.launch.py: радиус поиска
        # соответствий = ДЕСЯТЬ размеров вокселя. Первая попытка со значением
        # 0.1 м дала `cor=0` — ноль соответствий на каждом кадре: при быстром
        # движении смещение между кадрами больше десяти сантиметров, и точки
        # предыдущего облака просто не попадают в радиус поиска.
        'Icp/MaxCorrespondenceDistance': '0.5',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
        'Icp/MaxTranslation': '3',
        # Для ОДОМЕТРИИ порог доли совпавших точек делают очень мягким (0.01):
        # строгий порог отвергает кадр целиком, а потерянный кадр хуже,
        # чем неточный. Для замыканий петли порог, наоборот, высокий.
        'Icp/CorrespondenceRatio': '0.01',
        'Odom/Strategy': '0',
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': '0.05',
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
    }]

    slam_params = [{
        'frame_id': BASE_FRAME,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'approx_sync_max_interval': 0.01,
        'wait_for_transform': 0.5,
        'Reg/Force3DoF': 'false',
        'RGBD/CreateOccupancyGrid': 'true',
        'Grid/FromDepth': 'true',
    }]
    slam_remaps = [
        ('rgb/image', RGB_TOPIC),
        ('rgb/camera_info', INFO_TOPIC),
        ('depth/image', DEPTH_TOPIC),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='каталог ROS 2 bag с записью'),
        DeclareLaunchArgument('viz', default_value='true'),
        DeclareLaunchArgument('play', default_value='false'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('database_path',
                              default_value='/datasets/rtabmap_icp.db'),

        SetParameter(name='use_sim_time', value=True),

        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['0.0', '0.0', '0.0',
                        '-1.57079632679', '0.0', '-1.57079632679',
                        BASE_FRAME, OPTICAL_FRAME]),

        # Карта глубины -> облако точек. Оно и есть вход ICP.
        Node(package='rtabmap_util', executable='point_cloud_xyz', output='screen',
             parameters=[{'approx_sync': True, 'decimation': 4,
                          'max_depth': 4.0, 'voxel_size': 0.05}],
             remappings=[('depth/image', DEPTH_TOPIC),
                         ('depth/camera_info', INFO_TOPIC)]),

        Node(package='rtabmap_odom', executable='icp_odometry', output='screen',
             parameters=icp_params,
             remappings=[('scan_cloud', '/cloud')]),

        # SLAM продолжает работать по RGB-D: карта и замыкания петли ему нужны
        # цветные, а позу он теперь берёт от ICP-одометрии.
        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=slam_params + [{'database_path': database_path}],
             remappings=slam_remaps, arguments=['-d']),

        Node(package='rtabmap_viz', executable='rtabmap_viz', output='screen',
             parameters=slam_params, remappings=slam_remaps,
             condition=IfCondition(viz)),

        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
                output='screen')],
            condition=IfCondition(play)),
    ])
