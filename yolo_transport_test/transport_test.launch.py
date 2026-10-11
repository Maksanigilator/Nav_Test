"""Стенд передачи: издатель кадра и RViz одной командой.

    ros2 launch /root/ros_ws/yolo_transport_test/transport_test.launch.py

Узел сегментации сюда НЕ входит и входить не может: он живёт в другом
контейнере, с видеокартой и ultralytics. Его поднимают отдельно:

    cd ~/roshack/Roshack-yolodocker && docker compose up rail_mask

Файл лежит рядом со скриптами, а не в пакете maze_nav: к сборке ROS
стенд отношения не имеет, и держать его отдельно проще. Поэтому издатель
запускается через ExecuteProcess — Node умеет только то, что установлено
в пакет.

ПРО ЧАСТОТУ. По умолчанию 1 Гц: сперва надо убедиться, что кадр вообще
доходит. Разгонять есть смысл до 2-3 Гц и не выше — RTAB-Map обрабатывает
2 Гц (Rtabmap/DetectionRate в robot_nav.launch.py), и всё, что приходит
чаще, он отбрасывает. Большие частоты имеют смысл только как нагрузочная
проба транспорта, а не как рабочий режим.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    rate = LaunchConfiguration('rate')

    pub = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'frame_pub.py'),
             '--ros-args',
             '-p', ['rate:=', rate],
             '-p', ['image:=', LaunchConfiguration('image')],
             '-p', ['report_every:=', LaunchConfiguration('report_every')]],
        output='screen',
        # Без emulate_tty вывод питона уходит в канал и буферизуется
        # блоками — сводка не появлялась бы до конца прогона.
        emulate_tty=True)

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(HERE, 'transport_test.rviz')])

    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='частота отправки кадра, Гц'),
        DeclareLaunchArgument('image',
                              default_value=os.path.join(HERE,
                                                         'rail_frame.png'),
                              description='какой кадр слать'),
        DeclareLaunchArgument('report_every', default_value='10.0',
                              description='как часто печатать сводку, с'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='показывать окно с двумя панелями'),
        pub, rviz,
    ])
