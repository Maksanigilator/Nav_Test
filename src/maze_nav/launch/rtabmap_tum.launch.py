"""RTAB-Map на датасете TUM RGB-D — опорная точка перед симулятором.

Смысл запуска: убедиться, что RTAB-Map собран и настроен правильно, на данных
с ground truth. Если здесь ошибка траектории в пределах нормы, то все дальнейшие
провалы в Gazebo и на роботе можно списывать на данные и на камеру, а не на сам
алгоритм. Без такой опорной точки отлаживать визуальный SLAM вслепую бессмысленно.

Сделан по мотивам rtabmap_examples/launch/rgbdslam_datasets.launch.py, но с двумя
отличиями: rtabmap_viz по умолчанию выключен (в автоматическом прогоне GUI не нужен
и требует X11), а пути и параметры вынесены в аргументы.

Запуск (bag должен быть уже сконвертирован scripts/tum_bag_to_ros2.py):

    ros2 launch maze_nav rtabmap_tum.launch.py
    # во втором терминале:
    ros2 bag play /datasets/rgbd_dataset_freiburg3_long_office_household --clock

После прогона:

    rtabmap-report ~/.ros/rtabmap.db
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

# Кадр камеры в датасете. Ground truth приезжает как world->kinect_gt: суффикс
# _gt навешивает наш конвертер, иначе ground truth и визуальная одометрия
# спорили бы за родителя фрейма kinect и дерево TF разваливалось бы.
FRAME_ID = 'kinect'
GROUND_TRUTH_FRAME = 'world'
GROUND_TRUTH_BASE = 'kinect_gt'


def generate_launch_description():
    viz = LaunchConfiguration('viz')
    database_path = LaunchConfiguration('database_path')
    publish_camera_tf = LaunchConfiguration('publish_camera_tf')
    bag = LaunchConfiguration('bag')
    play = LaunchConfiguration('play')
    rate = LaunchConfiguration('rate')

    # Параметры RTAB-Map передаются строками — это требование самой библиотеки,
    # а не прихоть: она парсит их своим конфигом, а не через rclcpp.
    odom_parameters = [{
        'frame_id': FRAME_ID,
        # ground truth тут нужен только чтобы совместить начало траектории
        # одометрии с началом эталонной
        'ground_truth_frame_id': GROUND_TRUTH_FRAME,
        'ground_truth_base_frame_id': GROUND_TRUTH_BASE,
        'keep_color': True,
        'wait_for_transform': 0.5,
        'Odom/Strategy': '0',           # 0 = Frame-to-Map
        'Odom/ResetCountdown': '15',    # сколько кадров потери терпеть до сброса
        'Odom/GuessSmoothingDelay': '0',
    }]

    slam_parameters = [{
        'frame_id': FRAME_ID,
        'database_path': database_path,
        # Записываем ground truth в базу — без этого rtabmap-report
        # не посчитает ошибку траектории, а весь смысл этапа в ней.
        'ground_truth_frame_id': GROUND_TRUTH_FRAME,
        'ground_truth_base_frame_id': GROUND_TRUTH_BASE,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_rgbd': True,
        'subscribe_odom_info': True,
        'Mem/UseOdomFeatures': 'true',
        'Rtabmap/StartNewMapOnLoopClosure': 'true',
        'RGBD/CreateOccupancyGrid': 'false',
        'Rtabmap/CreateIntermediateNodes': 'true',
        # 0 = обрабатывать каждый кадр, не прореживать по смещению.
        # На датасете нам нужна вся траектория, а не экономия узлов.
        'RGBD/LinearUpdate': '0',
        'RGBD/AngularUpdate': '0',
    }]

    odom_remappings = [
        ('rgb/image', '/camera/rgb/image_color'),
        ('rgb/camera_info', '/camera/rgb/camera_info'),
        ('depth/image', '/camera/depth/image'),
    ]
    # SLAM берёт готовый RGBD-кадр с выхода одометрии, чтобы не выделять
    # признаки на тех же картинках второй раз.
    slam_remappings = [('rgbd_image', 'odom_rgbd_image')]

    return LaunchDescription([
        DeclareLaunchArgument('viz', default_value='false',
                              description='запускать ли GUI rtabmap_viz (нужен X11)'),
        DeclareLaunchArgument('database_path', default_value='/datasets/rtabmap_tum.db',
                              description='куда класть базу RTAB-Map для rtabmap-report'),
        DeclareLaunchArgument('publish_camera_tf', default_value='false',
                              description='публиковать статику kinect->optical_frame '
                                          '(нужно только если в бэге нет /tf)'),
        DeclareLaunchArgument(
            'bag',
            default_value='/datasets/rgbd_dataset_freiburg3_long_office_household',
            description='каталог ROS 2 bag с датасетом'),
        DeclareLaunchArgument('play', default_value='false',
                              description='проигрывать bag самим, не во втором терминале'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='скорость проигрывания: <1 медленнее, >1 быстрее'),

        # Время берём из bag: он проигрывается с --clock
        SetParameter(name='use_sim_time', value=True),

        Node(package='rtabmap_odom', executable='rgbd_odometry', output='screen',
             parameters=odom_parameters, remappings=odom_remappings),

        # -d стирает базу при старте, иначе прогоны наслаиваются друг на друга
        # и отчёт считается по мешанине из нескольких запусков.
        Node(package='rtabmap_slam', executable='rtabmap', output='screen',
             parameters=slam_parameters, remappings=slam_remappings,
             arguments=['-d']),

        Node(package='rtabmap_viz', executable='rtabmap_viz', output='screen',
             parameters=slam_parameters, remappings=slam_remappings,
             condition=IfCondition(viz)),

        # Проигрывание бэга прямо отсюда — чтобы смотреть демо одной командой,
        # а не жонглировать двумя терминалами. Задержка нужна: узлы поднимаются
        # не мгновенно, а сообщения, пришедшие до подписки, просто пропадут,
        # и первые секунды датасета выпадут из карты.
        TimerAction(
            period=8.0,
            actions=[ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
                output='screen')],
            condition=IfCondition(play)),

        # ВАЖНО: по умолчанию выключено, и это отличие от штатного примера RTAB-Map.
        # Там эта статика нужна, потому что их способ конвертации терял /tf.
        # Наш конвертер /tf сохраняет полностью: в бэге живёт цепочка
        # kinect -> openni_camera -> openni_rgb_frame -> openni_rgb_optical_frame
        # (проверено, публикуется на 10 Гц весь прогон). Если поверх неё поднять
        # ещё и kinect -> openni_rgb_optical_frame, у фрейма окажется два
        # родителя и дерево TF развалится.
        # Оставлено аргументом на случай датасета, где /tf действительно пуст.
        Node(package='tf2_ros', executable='static_transform_publisher', output='screen',
             arguments=['0.0', '0.0', '0.0',
                        '-1.57079632679', '0.0', '-1.57079632679',
                        FRAME_ID, 'openni_rgb_optical_frame'],
             condition=IfCondition(publish_camera_tf)),
    ])
