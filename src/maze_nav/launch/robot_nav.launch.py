#!/usr/bin/env python3
"""Навигация на НАСТОЯЩЕМ роботе: карта по RGB-D и езда по ней.

Всё считается на ноутбуке, который едет на роботе: к нему воткнуты
реалсенс и плата с MPU6050. На робота по сети уходит только скорость.

РАЗДЕЛЕНИЕ МАШИН. На роботе стоит ROS 2 Humble, здесь Jazzy. Через
границу ходит одно сообщение — geometry_msgs/Twist в топике /cmd_nav, и
этот тип между дистрибутивами не менялся. Проверено передачей: значения
доходят без искажений. Собственные типы RTAB-Map границу бы не прошли, но
им и не нужно: весь SLAM на одной стороне.

КУДА ПУБЛИКУЕТСЯ СКОРОСТЬ. Только в /cmd_nav, никогда в /cmd_vel.
На роботе стоит мультиплексор: он принимает /cmd_vel_teleop с геймпада и
/cmd_nav отсюда, а в /cmd_vel пишет сам. Режим переключается кнопкой X на
геймпаде, программного пути нет — и это правильно, кнопка остаётся
аварийным выключателем. Если писать в /cmd_vel напрямую, мы обойдём и
мультиплексор, и сторожа по геймпаду.

ЧЕГО ЗДЕСЬ НЕТ. Колёсной одометрии: на роботе /wheel_odometry имеет тип
Twist, то есть это скорость, а не поза, и одометрией служить не может в
принципе. Положение ведёт только зрение с ИНС.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, Command, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from typing import List
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('maze_nav')
    imu = LaunchConfiguration('imu')
    prof = LaunchConfiguration('profile')

    rgb = '/camera/color/image_raw'
    info = '/camera/color/camera_info'
    depth = '/camera/aligned_depth_to_color/image_raw'

    # ═══ Описание робота ═══
    # Берём описание для ЖЕЛЕЗА: без блоков Gazebo и без оптического кадра
    # камеры — его ведёт драйвер реалсенса. Отсюда же берётся связка
    # base_footprint -> camera_link, без которой одометрия не сможет
    # пересчитать движение камеры в движение робота.
    urdf = os.path.join(pkg, 'urdf', 'agro_robot_real.urdf.xacro')
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': ParameterValue(
                   Command(['xacro ', urdf]), value_type=str)}])

    # ═══ Камера ═══
    #
    # ДВЕ КАМЕРЫ НА ВЫБОР, аргумент camera:=realsense|zed. Весь стек ниже
    # знает ТОЛЬКО три имени топиков, объявленных выше (rgb, info, depth),
    # и про камеру не осведомлён вовсе. Ветка ZED переименовывает свои
    # топики в эти же три, поэтому ни RTAB-Map, ни детектор кромки, ни
    # сегментация рельсов при смене камеры не меняются ни на строку.
    #
    # Форматы глубины у них разные — RealSense отдаёт 16UC1 в миллиметрах,
    # ZED 32FC1 в метрах, — но и это уже учтено: детектор различает их по
    # полю encoding, а отбор отсчётов идёт через isfinite, что ловит и
    # NaN от ZED, и нули от RealSense.
    use_rs = PythonExpression(
        ["'", LaunchConfiguration('camera'), "' != 'zed'"])
    use_zed = PythonExpression(
        ["'", LaunchConfiguration('camera'), "' == 'zed'"])

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch',
            'rs_launch.py')),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': '',
            # Живое плотное облако с текстурой. По умолчанию ВЫКЛЮЧЕНО:
            # 848x480 точек с цветом тридцать раз в секунду — это заметный
            # поток, а нужен он только чтобы посмотреть глазами, что под
            # роботом прямо сейчас.
            #
            # Карта такой текстуры не даст и не должна: мы кормим RTAB-Map
            # СВОИМ облаком, где цвет означает смысл (серое — пол, зелёное
            # — рельсы, красное — кромка), а не вид поверхности. Это
            # сознательный размен разметки на текстуру.
            'pointcloud.enable': LaunchConfiguration('cloud'),
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.depth_profile': prof,
            'rgb_camera.color_profile': prof,
            # ИНС камеры НЕ трогаем: её забирает ядро модулями hid_sensor_*,
            # и попытка открыть модуль движения роняет весь узел драйвера.
            # Угловую скорость даёт отдельная плата.
            # СОБСТВЕННАЯ ИНС КАМЕРЫ. По умолчанию выключена: для карты
            # она не нужна, а в ядре её разбирают модули hid_sensor_*, из-за
            # чего драйвер натыкается на «Permission denied».
            #
            # Включается ради калибровки угла крепления: акселерометр
            # камеры меряет тяжесть В ЕЁ СОБСТВЕННОМ кадре, то есть даёт
            # наклон камеры напрямую, без посредников и без URDF.
            #
            # Если драйвер ругнётся на доступ — выгрузить модули ядра,
            # которые забрали прибор себе: sudo modprobe -r hid_sensor_accel_3d
            'enable_gyro': LaunchConfiguration('cam_imu'),
            'enable_accel': LaunchConfiguration('cam_imu'),
            'initial_reset': 'false',
        }.items(),
        condition=IfCondition(use_rs))

    # ═══ Камера: ветка ZED ═══
    #
    # Переименование делает SetRemap в группе: обёртка ZED публикует под
    # своими именами, а наружу они выходят теми же тремя, что и у
    # RealSense. Глубина у ZED уже совмещена с цветом, отдельного
    # выравнивания (align_depth) не требуется.
    #
    # СВОИ ПРЕОБРАЗОВАНИЯ И ПРИВЯЗКУ ВЫКЛЮЧАЕМ. ZED умеет считать
    # собственную одометрию и публиковать map->odom, но этим у нас
    # занимается RTAB-Map. Два источника одного преобразования дают
    # рваное дерево TF и карту, которая дёргается, — мы это уже проходили
    # с двумя robot_state_publisher.
    zed = GroupAction(
        condition=IfCondition(use_zed),
        actions=[
            # ИМЕНА СНЯТЫ С ЖИВОГО УЗЛА, а не взяты из памяти. В обёртке
            # 5.5 они не такие, как в прежних выпусках: цветной кадр
            # теперь rgb/color/rect/image, а не rgb/image_rect_color.
            # Если камера перестанет доходить до RTAB-Map после
            # обновления обёртки — первым делом сверить этот список с
            # разделом «PUBLISHED TOPICS» в её логе при старте.
            SetRemap('/zed/zed_node/rgb/color/rect/image', rgb),
            SetRemap('/zed/zed_node/rgb/color/rect/camera_info', info),
            SetRemap('/zed/zed_node/depth/depth_registered', depth),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('zed_wrapper'), 'launch',
                    'zed_camera.launch.py')),
                launch_arguments={
                    'camera_model': 'zedm',
                    'camera_name': 'zed',
                    # РАЗРЕШЕНИЕ И ТЕМП СНИЖЕНЫ НАМЕРЕННО.
                    #
                    # По умолчанию ZED отдаёт 1920x1080 на тридцати
                    # герцах. Замерено, что с этим связка не справляется:
                    # обработка кадра в RTAB-Map занимала до 1.2 с при
                    # темпе 0.5 с, задержки доходили до 1.6 с, одометрия
                    # шла 11 Гц при кадрах 27 — то есть между двумя
                    # обработанными кадрами камера успевала уехать, и
                    # сопоставление валилось с «Not enough inliers 0/15».
                    # Каждый такой провал сбрасывает одометрию, а сброс
                    # начинает НОВУЮ карту: в рабочей памяти набралось 799
                    # узлов, не связанных между собой, и единой карты не
                    # получалось вовсе.
                    #
                    # HD720 это 1280x720 против 848x480 у RealSense —
                    # всё ещё вдвое больше, но уже посильно. Если и этого
                    # много, следующая ступень VGA (672x376).
                    'param_overrides': PythonExpression([
                        "'general.grab_resolution:=' + '",
                        LaunchConfiguration('zed_res'),
                        "' + ';general.pub_frame_rate:=' + '",
                        LaunchConfiguration('zed_fps'), "'"]),
                    'publish_tf': 'false',
                    'publish_map_tf': 'false',
                    # А ВОТ СВЯЗКУ ИНС С КОРПУСОМ ПУБЛИКОВАТЬ НАДО.
                    # Это отдельный ключ, и по умолчанию он false. Без
                    # него кадра zed_imu_link в дереве нет, хотя
                    # сообщения ИНС на него ссылаются, и одометрия
                    # отказывается её принимать:
                    #   «Dropping imu data! A valid TF between
                    #    camera_link and zed_imu_link is required»
                    # Сам узел при этом молча пишет в лог
                    # «Broadcast IMU TF: FALSE» — заметить легко только
                    # задним числом.
                    'publish_imu_tf': 'true',
                    # publish_urdf ОСТАЁТСЯ ВКЛЮЧЁННЫМ, и это не
                    # недосмотр. Сперва я его выключил, рассудив, что
                    # описание камеры нам ни к чему: своё есть. Узел
                    # после этого вставал на «Starting Positional
                    # Tracking / Waiting for valid static
                    # transformations» и кадры не публиковал вовсе —
                    # привязка ждёт преобразований, которые публикует
                    # именно этот издатель.
                    #
                    # Выключены только publish_tf и publish_map_tf:
                    # odom->base и map->odom дают rgbd_odometry и
                    # RTAB-Map, а два источника одного преобразования
                    # рвут дерево TF.
                    'publish_urdf': 'true',
                }.items()),
            # СВЯЗКА С РОБОТОМ. Узел ZED публикует свою цепочку кадров
            # (zed_camera_link -> ... -> zed_left_camera_frame_optical), но
            # её корень ни к чему не прицеплен: для TF это отдельное
            # дерево. RTAB-Map не смог бы пересчитать глубину в кадр
            # робота и отказался бы работать вовсе.
            #
            # ГЕОМЕТРИЯ ЗДЕСЬ ЗАГЛУШКА — нули. ZED встаёт ровно туда, где
            # в URDF стоит RealSense, вместе с его наклоном 50.1° и
            # высотой 723 мм. Это заведомо неверно: у камер разные
            # габариты и разное положение оптического центра. Для
            # проверки «есть ли картинка и облако» годится, ездить по
            # такой карте НЕЛЬЗЯ.
            #
            # Когда крепление измерят, числа ставятся сюда: смещение в
            # метрах и поворот в радианах от camera_link к zed_camera_link.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='zed_mount', output='log',
                 arguments=['--frame-id', 'camera_link',
                            '--child-frame-id', 'zed_camera_link',
                            '--x', '0', '--y', '0', '--z', '0',
                            '--roll', '0', '--pitch', '0', '--yaw', '0']),
        ])

    # ═══ ИНС ═══
    pico = Node(package='maze_nav', executable='pico_imu_node.py',
                name='pico_imu', output='screen',
                condition=IfCondition(imu),
                parameters=[{'port': LaunchConfiguration('imu_port'),
                             'frame_id': 'imu_link',
                             'axes': LaunchConfiguration('imu_axes'),
                             'gyro_bias': ParameterValue(
                                 LaunchConfiguration('gyro_bias'),
                                 value_type=List[float]),
                             # ЧУВСТВИТЕЛЬНОСТЬ АКСЕЛЕРОМЕТРА ЗАДАНА ЯВНО.
                             #
                             # Узел умеет выбирать её по формату кадра и
                             # для подлинного MPU6050 выбрал бы верно:
                             # регистр диапазона у нас читается 0x08, что
                             # по документации означает +-4 g и 8192
                             # отсчёта на g.
                             #
                             # Но чип на плате НЕ ПОДЛИННЫЙ: WHO_AM_I
                             # отвечает 0x70 вместо 0x68. У клонов
                             # раскладка битов диапазона бывает сдвинута,
                             # и здесь именно так. Замерено на неподвижном
                             # роботе: тяжесть выходила 4.92 м/с² вместо
                             # 9.81, то есть ровно вдвое меньше. Значит
                             # чувствительность 4096, как у +-8 g.
                             #
                             # Проверено чтением регистра: запись проходит,
                             # чип отдаёт обратно ровно то, что записали.
                             # То есть виновата не прошивка, а раскладка
                             # конкретного экземпляра.
                             #
                             # Проверка после правки: imu_level.py должен
                             # показать |a| около 9.81. Если выйдет 19.6 —
                             # поставили не тому чипу, вернуть 0 (выбор по
                             # формату кадра).
                             'accel_lsb_per_g': 4096.0,
                             'accel_bias': ParameterValue(
                                 LaunchConfiguration('accel_bias'),
                                 value_type=List[float])}],
                remappings=[('imu/data_raw', '/imu/data_raw')])
    madgwick = Node(package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter', output='screen',
                    condition=IfCondition(imu),
                    parameters=[{'use_mag': False, 'world_frame': 'enu',
                                 'publish_tf': False}],
                    remappings=[('imu/data_raw', '/imu/data_raw'),
                                ('imu/data', '/imu/data')])

    # ═══ Одометрия ═══
    # frame_id = base_footprint, а не камера: узел меряет движение камеры,
    # но обязан отдать движение РОБОТА, и пересчёт идёт через TF. Поэтому
    # ошибка в креплении камеры превращается в систематическую ошибку
    # одометрии, а не в шум.
    odom_common = dict(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        remappings=[('rgb/image', rgb), ('rgb/camera_info', info),
                    ('depth/image', depth), ('imu', '/imu/data')])
    # ТРИ СТЕПЕНИ СВОБОДЫ ВМЕСТО ШЕСТИ, по флагу force_2d.
    #
    # Робот ездит по ровному настилу, и физически у него три степени
    # свободы: x, y и курс. Зрение же оценивает все шесть, и три лишние —
    # шум, который НАКАПЛИВАЕТСЯ. Замерено в симуляторе: крен 3.5 и тангаж
    # 5.6 градуса за сорок секунд езды по ровному полу, при истинных нулях.
    #
    # На живом роботе это проявилось иначе и больнее. Порог пола у
    # RTAB-Map — три сантиметра (Grid/MaxGroundHeight ниже). Когда карта
    # наклоняется, дальние части ровного пола поднимаются выше порога и
    # становятся ПРЕПЯТСТВИЯМИ: костмап закрашивался непроезжим там, где
    # робот только что проехал. Заодно переставала выделяться кромка — она
    # публикуется приподнятой, но на фоне поднявшегося пола терялась.
    #
    # Расплата: настоящие наклоны корпуса (подвеска при разгоне, ступенька
    # на рельсах) в оценку не попадут. Для заезда это не опасно — там крен
    # проверяется ПО ИНС напрямую, а не по одометрии.
    force_2d = ParameterValue(LaunchConfiguration('force_2d'), value_type=str)

    odom_params = {
        'Reg/Force3DoF': force_2d,
        'frame_id': 'base_footprint',
        'approx_sync': True,
        'publish_tf': True,
        'Odom/ResetCountdown': '1',
        'Vis/MinInliers': '15',
    }
    odom_imu = Node(**odom_common, condition=IfCondition(imu),
                    parameters=[dict(odom_params, wait_imu_to_init=True)])
    odom_plain = Node(**odom_common, condition=UnlessCondition(imu),
                      parameters=[dict(odom_params, wait_imu_to_init=False)])

    # ═══ SLAM ═══
    # ═══ SLAM ═══
    # Два варианта, и разница принципиальная.
    #
    # Без детектора обрыва RTAB-Map строит сетку занятости сам из глубины.
    # Кромку площадки он при этом НЕ увидит: обрыв — это отрицательное
    # препятствие, пола там просто нет, и для сетки это неотличимо от
    # неизвестности.
    #
    # С детектором RTAB-Map перестаёт смотреть на глубину и принимает
    # готовое облако от него (subscribe_scan_cloud). В этом облаке кромка
    # уже подделана под обычное препятствие — поднята на виртуальную
    # высоту, — и попадает в карту красным. Ровно так это и работало в
    # симуляторе.
    # Стирать ли базу на старте.
    #
    # По умолчанию да: при отладке почти всегда нужна чистая карта, а
    # дописывание в старую даёт путаницу, которую потом не распутать —
    # робот стоит в одном месте, а карта помнит прошлую поездку.
    #
    # Но когда карту собирают НАРОЧНО, чтобы потом по ней ездить, стирать
    # её нельзя. Для этого delete_db:=false: тогда база, указанная в db,
    # переживёт запуск и её можно будет открыть снова.
    #
    # ВАЖНО: RTAB-Map дописывает базу по ходу, но закрывает её корректно
    # только при штатном завершении. Останавливать запуск надо Ctrl-C и
    # дождаться, пока узлы погаснут сами. Убийство через kill -9 оставит
    # базу недописанной.
    slam_common = dict(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        arguments=[PythonExpression(
            ["'--delete_db_on_start' if '",
             LaunchConfiguration('delete_db'), "'.lower() in "
             "('true', '1', 'yes') else ''"])])
    slam_base = {
        # Та же мера для графа карты, см. force_2d выше. Slam2D держит
        # оптимизатор в плоскости: без него граф остаётся трёхмерным, и
        # замыкания петель снова внесут крен с тангажом, уже исправленные
        # в одометрии.
        'Reg/Force3DoF': force_2d,
        'Optimizer/Slam2D': force_2d,
        # ОЧЕРЕДИ СИНХРОНИЗАЦИИ подняты с умолчания 10.
        #
        # RTAB-Map ждёт пять входов с совпадающими метками: одометрию,
        # цвет, глубину, калибровку и облако от детектора. Облако приходит
        # позже остальных — детектор его считает, — а одометрия, как видно
        # в логе, отдаёт результат с задержкой до 0.4 с. Очередь в 10
        # кадров на 30 Гц держит всего 333 мс истории, и набор просто не
        # успевал сойтись: узел работал, но не публиковал НИЧЕГО, включая
        # преобразование map->odom. Снаружи это выглядело как «нет кадра
        # map» и неактивный Nav2, хотя причина была здесь.
        #
        # Тридцать кадров — это секунда истории, с запасом на наблюдаемые
        # задержки. Платим памятью на буферы, её достаточно.
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'approx_sync': True,
        'database_path': LaunchConfiguration('db'),
        'Grid/RayTracing': 'true',
        'Rtabmap/DetectionRate': '2.0',
    }
    slam_remap = [('rgb/image', rgb), ('rgb/camera_info', info),
                  ('depth/image', depth), ('imu', '/imu/data')]

    slam_plain = Node(
        **slam_common, condition=UnlessCondition(LaunchConfiguration('dropoff')),
        parameters=[dict(slam_base,
                         **{'Grid/FromDepth': 'true',
                            'Grid/RangeMax': '4.0',
                            # Пол по нормалям: камера наклонена, и
                            # постоянный порог высоты отрезал бы дальнюю
                            # часть пола вместе с препятствиями.
                            'Grid/NormalsSegmentation': 'true',
                            'Grid/MaxGroundAngle': '45'})],
        remappings=slam_remap)

    slam_edge = Node(
        **slam_common, condition=IfCondition(LaunchConfiguration('dropoff')),
        parameters=[dict(slam_base,
                         **{'subscribe_scan_cloud': True,
                            # Сетку строим ТОЛЬКО из облака детектора.
                            'Grid/FromDepth': 'false',
                            # Облако уже в кадре камеры и уже разобрано на
                            # пол и препятствия, поэтому сегментация по
                            # нормалям тут лишняя и только мешает.
                            'Grid/NormalsSegmentation': 'false',
                            # ПОРОГ ПОЛА ПОДНЯТ С 3 ДО 15 СМ.
                            #
                            # Всё выше порога RTAB-Map считает
                            # препятствием. Три сантиметра означали, что
                            # малейший перекос карты делает дальний край
                            # ровного пола непроезжим: при остаточном
                            # наклоне 1.6 градуса пол на трёх метрах
                            # поднимается на 8 см, и костмап закрашивался
                            # фиолетовым там, где робот только что ехал.
                            #
                            # Нам от сетки нужно немногое: знать, где
                            # КРОМКА, и не пускать в неизвестность. Кромку
                            # детектор публикует приподнятой на 30 см
                            # (virtual_height), так что порог 15 см её
                            # по-прежнему ловит с двукратным запасом, а
                            # шум пола и наклон — уже нет.
                            #
                            # Расплата: настоящее препятствие ниже 15 см
                            # на настиле сеткой не заметится. Рельсы в том
                            # числе — но они и не должны быть
                            # препятствием, робот на них заезжает, а
                            # показываются они своим слоем.
                            'Grid/MaxGroundHeight': '0.15',
                            'Grid/MinGroundHeight': '-0.05',
                            'Grid/RangeMax': '3.0',
                            'Grid/Sensor': '0'})],
        remappings=slam_remap + [('scan_cloud', '/dropoff/grid_cloud')])

    # ═══ Nav2 ═══
    # МОНИТОР СТОЛКНОВЕНИЙ ВЫВЕДЕН ИЗ ЦЕПОЧКИ СКОРОСТЕЙ.
    #
    # Штатно цепочка такая: контроллер -> сглаживатель -> монитор -> робот.
    # Монитор требует живой источник облака точек и, не получив его,
    # ОСТАНАВЛИВАЕТ робота. В настройках там стоит /dropoff/grid_cloud от
    # детектора обрыва, которого на роботе может не быть вовсе, и монитор
    # глушил скорость наглухо — снаружи это выглядело как «Nav2 работает,
    # путь построен, а робот стоит».
    #
    # Поэтому монитор обходим: вход переводим на несуществующий топик,
    # выход в тупик, а сглаживатель публикует прямо в /cmd_nav. Теряем
    # автоматический останов перед препятствием, которое Nav2 проглядел;
    # последняя защита — кнопка X на геймпаде.
    nav_params = RewrittenYaml(
        source_file=os.path.join(pkg, 'params', 'agro_nav2.yaml'),
        root_key='', convert_types=True,
        param_rewrites={'cmd_vel_in_topic': '/collision_monitor_unused_in',
                        'cmd_vel_out_topic': '/collision_monitor_unused_out'})
    nav2 = GroupAction(
        condition=IfCondition(LaunchConfiguration('nav2')),
        actions=[
            SetRemap('cmd_vel_smoothed', '/cmd_nav_raw'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch',
                    'navigation_launch.py')),
                # Глушим ОДИН логгер. computeCircumscribedCost сыпет
                # ошибкой каждый такт, потому что раздувание 0.55 меньше
                # описанного радиуса 0.772 (кузов 1.22 x 0.90 плюс
                # footprint_padding 0.01). Это не поломка: Nav2 лишь
                # сообщает, что не может срезать проверку столкновений по
                # полю стоимости и проверяет весь полигон. Он и так его
                # проверяет — у CostCritic стоит consider_footprint, — а
                # поднять раздувание до 0.772 нельзя: площадка около двух
                # метров, и свободного места не осталось бы вовсе
                # (разбор у inflation_radius в agro_nav2.yaml).
                # Остальные логгеры остаются на info.
                launch_arguments={'use_sim_time': 'false',
                                  'params_file': nav_params,
                                  'use_composition': 'False',
                                  'log_level':
                                      'computeCircumscribedCost:=fatal'
                                  }.items()),
        ])

    # Детектор обрыва: кромка платформы по глубине. На железе рельсов
    # может не быть вовсе — маску рельсов он берёт по желанию, и без неё
    # просто не красит ничего зелёным, а кромку находит как обычно.
    dropoff = Node(package='maze_nav', executable='dropoff_detector.py',
                   output='screen',
                   condition=IfCondition(LaunchConfiguration('dropoff')),
                   remappings=[('depth', depth),
                               ('camera_info', info),
                               ('dropoff', '/dropoff/points'),
                               ('grid_cloud', '/dropoff/grid_cloud'),
                               ('rails/mask', '/rails/mask')])

    # Автомат заезда. По умолчанию выключен: он СРАЗУ начинает крутить
    # робота на месте, а потом едет, и включать его вместе с картой надо
    # осознанно. Рельсы берёт из накопленного слоя, а не из карты
    # препятствий: при сегментации в 2 Гц зелёного в карте почти не
    # остаётся, и вписывание колеи садится мимо — проверено в симуляторе,
    # колея выходила 261 мм вместо 545.
    # ПОСЛЕДНИЙ УЗЕЛ ПЕРЕД РОБОТОМ. Разворачивает знак поворота и гасит
    # скорость при пропаже команд. Подробности и дата замера — в самом
    # cmd_sign.py. Поднимается всегда, когда что-то может поехать: и при
    # nav2, и при entry, потому что пишут в /cmd_nav_raw оба.
    sign = Node(package='maze_nav', executable='cmd_sign.py',
                name='cmd_sign', output='screen',
                parameters=[{'angular_z_sign': ParameterValue(
                    LaunchConfiguration('wz_sign'), value_type=float)}],
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('nav2'), "' == 'true' or '",
                    LaunchConfiguration('entry'), "' == 'true'"])),
                remappings=[('cmd_nav_raw', '/cmd_nav_raw'),
                            ('cmd_nav', '/cmd_nav')])

    entry = Node(package='maze_nav', executable='rail_entry.py',
                 output='screen',
                 condition=IfCondition(LaunchConfiguration('entry')),
                 remappings=[('odom', '/odom'), ('imu', '/imu/data'),
                             ('obstacles', '/rails/map')])

    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')])

    return LaunchDescription([
        DeclareLaunchArgument('imu', default_value='true',
                              description='брать ИНС с платы на Pico'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('imu_axes', default_value='x y z',
                              description='разворот осей датчика; подобрать '
                                          'помощником imu_axes.py'),
        DeclareLaunchArgument('gyro_bias', default_value='[0.0, 0.0, 0.0]',
                              description='смещение нуля гироскопа, рад/с; '
                                          'замеряется imu_bias.py и без него '
                                          'курс уходит на градусы в минуту'),
        DeclareLaunchArgument('accel_bias', default_value='[0.0, 0.0, 0.0]',
                              description='смещение нуля акселерометра, м/с²'),
        DeclareLaunchArgument('zed_res', default_value='HD720',
                              description='разрешение ZED: HD2K, HD1080, '
                                          'HD720, VGA'),
        DeclareLaunchArgument('zed_fps', default_value='15.0',
                              description='темп публикации кадров ZED, Гц'),
        DeclareLaunchArgument('camera', default_value='realsense',
                              description='какая камера: realsense или zed'),
        DeclareLaunchArgument('profile', default_value='848x480x30',
                              description='на порту USB2 нужно 640x480x15'),
        DeclareLaunchArgument('db', default_value='/datasets/robot_map.db',
                              description='файл карты; /datasets смонтирован '
                                          'из datasets/ в корне проекта'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='стирать карту на старте; false — '
                                          'продолжать в уже существующей'),
        DeclareLaunchArgument('dropoff', default_value='false',
                              description='искать кромку площадки по '
                                          'глубине; заодно переводит SLAM '
                                          'на ветку с кромкой. Отлажен на '
                                          'стенде при наклоне камеры около '
                                          '50 градусов — нынешние 49 в '
                                          'URDF как раз в этом диапазоне'),
        DeclareLaunchArgument('cam_imu', default_value='false',
                              description='включить собственную ИНС D435i; '
                                          'нужна для калибровки угла камеры'),
        DeclareLaunchArgument('cloud', default_value='false',
                              description='живое плотное облако с камеры '
                                          'для просмотра в RViz; поток '
                                          'немалый, включать по нужде'),
        DeclareLaunchArgument('force_2d', default_value='false',
                              description='держать одометрию и карту в трёх '
                                          'степенях свободы: x, y, курс. '
                                          'Крен, тангаж и высота обнуляются — '
                                          'для ровного настила это правда, а '
                                          'не упрощение'),
        DeclareLaunchArgument('entry', default_value='false',
                              description='автомат заезда на рельсы; '
                                          'ОСТОРОЖНО: сразу начинает крутить '
                                          'робота и ехать'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='планировщик; скорость уходит '
                                          'в /cmd_nav'),
        DeclareLaunchArgument('wz_sign', default_value='1.0',
                              description='знак поворота перед роботом. '
                              '1.0 — без разворота: 9 октября 2026 в репе '
                              'робота кинематика приведена к REP-103 '
                              '(fl = vx - vy - lw*wz), и +wz даёт левое '
                              'вращение. -1.0 — для прошивки до того '
                              'исправления, иначе инвертор развернёт '
                              'уже верный знак обратно'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rsp, camera, zed, pico, madgwick, odom_imu, odom_plain, entry,
        slam_plain, slam_edge, dropoff, nav2, sign, rviz,
    ])
