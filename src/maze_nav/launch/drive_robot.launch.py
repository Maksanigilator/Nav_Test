"""Робот в Gazebo с меканум-контроллером: проверка, что он вообще едет.

    ros2 launch maze_nav drive_robot.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard   # в другом терминале

Без карты, без SLAM и без Nav2 — только физика и привод. Смысл запуска в
том, чтобы увидеть три движения, которые отличают меканум от обычной
тележки: вперёд, боком и разворот на месте.

Что здесь чьё: gz_ros2_control, controller_manager, mecanum_drive_controller
и joint_state_broadcaster — штатные пакеты ROS 2. Своего кода тут два
файла: описание робота (urdf/agro_robot.urdf.xacro, сгенерировано из CAD
скриптом scripts/gen_robot_urdf.py) и переходник scripts/twist_to_stamped.py.

Одометрия по колёсам публикуется контроллером в
/mecanum_drive_controller/odometry, но TF из неё НЕ идёт: enable_odom_tf
выключен в params/agro_controllers.yaml. У этого робота меканум-колёса
проскальзывают, а привод не откалиброван, поэтому odom->base_link даёт
RTAB-Map по картинке. Колёсную одометрию оставляем как отдельный источник —
сравнивать с визуальной и, позже, сливать в robot_localization.
"""
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    """Собирает узлы, когда значения аргументов уже известны.

    Через OpaqueFunction, а не напрямую, из-за командной строки Gazebo:
    ключ -s либо есть, либо его нет, а подставить «пустой аргумент» нельзя —
    gz воспримет пустую строку как имя мира и не найдёт его.
    """
    arg = lambda n: LaunchConfiguration(n).perform(context)
    pkg = get_package_share_directory('maze_nav')
    world = os.path.join(pkg, 'worlds', arg('world'))
    headless = arg('gui') in ('false', 'False', '0')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'agro_robot.urdf.xacro')]),
        value_type=str)

    # ═══ Gazebo ═══
    # Запускается НАПРЯМУЮ, а не через ros_gz_sim/gz_sim.launch.py.
    #
    # Тот launch собирает процесс с shell=True, то есть на самом деле поднимает
    # /bin/sh -c "ruby .../gz sim ...". Сигнал от launch достаётся оболочке,
    # а детям она его не передаёт. Дальше по журналу: launch ждёт 5 секунд,
    # не дожидается и шлёт SIGTERM, от которого умирает оболочка — а сервер
    # и окно остаются СИРОТАМИ. Launch отчитывается о завершении, окно
    # Gazebo продолжает висеть на экране.
    #
    # Без оболочки сигнал приходит самой обёртке gz, а у неё в совмещённом
    # режиме (сервер и окно одной командой) есть ловушки INT/TERM, гасящие
    # обоих детей. Замерено после исправления: всё закрывается меньше чем
    # за 3 секунды, без эскалации до SIGTERM.
    #
    # Разделять на `gz sim -s` и `gz sim -g` НЕЛЬЗЯ: в одиночных режимах
    # ruby зовёт C++ прямо через FFI, ловушек там нет вовсе.
    #
    # -r запускает мир сразу, без нажатия «play». -v 3 оставляет
    # предупреждения видимыми: молчащий Gazebo, который не загрузил плагин,
    # выглядит точно так же, как исправно работающий — именно на этом уровне
    # видна строка "Failed to load system plugin [gz_ros2_control-system]",
    # без которой робот просто стоит и не отзывается на команды.
    cmd = ['ruby', shutil.which('gz'), 'sim', '-r', '-v', '3']
    if headless:
        cmd.append('-s')
    cmd.append(world)

    # Два пути, которые Gazebo обязан знать, и оба легко потерять.
    #
    # SYSTEM_PLUGIN_PATH — где лежит libgz_ros2_control-system.so. Без него
    # симулятор поднимется как ни в чём не бывало, но controller_manager
    # внутри него не появится, и робот останется неподвижным. Берём из
    # LD_LIBRARY_PATH, как это делает ros_gz_sim.
    #
    # RESOURCE_PATH — где искать меши. Конвертер URDF->SDF переписывает
    # package://maze_nav/... в model://maze_nav/..., и дальше их ищет уже gz.
    # Без этого пути меши роликов молча не грузятся, единственная коллизия,
    # которая касается пола, исчезает, и робот едет на ребордах рельсовых
    # роликов: вперёд ползёт, вбок и вокруг оси — никак. В логе про это
    # всего одно предупреждение Wrn, ошибки нет.
    def joined(var, extra):
        return os.pathsep.join(filter(None, (os.environ.get(var, ''), extra)))

    gz = ExecuteProcess(
        name='gazebo', output='screen', cmd=cmd,
        additional_env={
            'GZ_SIM_SYSTEM_PLUGIN_PATH': joined('GZ_SIM_SYSTEM_PLUGIN_PATH',
                                                os.environ.get('LD_LIBRARY_PATH', '')),
            'GZ_SIM_RESOURCE_PATH': joined('GZ_SIM_RESOURCE_PATH',
                                           os.path.dirname(pkg)),
        },
        on_exit=[Shutdown(reason='Gazebo закрылся')])

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': robot_description,
                            'use_sim_time': True}])

    # Спавн берёт описание из топика, а не из файла: так в симулятор попадает
    # ровно то, что уже разобрал robot_state_publisher, и разойтись они не могут.
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description',
                            '-name', 'agro_robot',
                            '-x', '0', '-y', '0', '-z', arg('z')])

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 output='screen',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    relay = Node(package='maze_nav', executable='twist_to_stamped.py',
                 output='screen',
                 parameters=[{'use_sim_time': True}],
                 remappings=[('in', '/cmd_vel'),
                             ('out', '/mecanum_drive_controller/reference')])

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    output='screen',
                    arguments=[name, '--param-file',
                               os.path.join(pkg, 'params', 'agro_controllers.yaml')])

    jsb = spawner('joint_state_broadcaster')
    mec = spawner('mecanum_drive_controller')

    return [
        gz, rsp, clock, relay, spawn,

        # Контроллеры грузятся только после того, как модель оказалась в мире:
        # controller_manager живёт внутри плагина Gazebo и до спавна не существует.
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[mec])),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(pkg, 'rviz', 'robot.rviz')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat.world',
                              description='имя файла в share/maze_nav/worlds/'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='окно Gazebo; false — только сервер'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # Робот ставится на сантиметр выше пола: если посадить ровно в ноль,
        # ролики оказываются в контакте уже в первом шаге решателя и получают
        # выталкивающий импульс — робот подпрыгивает на старте.
        DeclareLaunchArgument('z', default_value='0.01'),

        OpaqueFunction(function=launch_setup),
    ])
