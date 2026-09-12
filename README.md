# ROS 2 Jazzy + Nav2 + Gazebo Harmonic

1. Сборка

cd ~/Nav_Test
docker build -f docker/Dockerfile -t nav2:jazzy .
часть	смысл
-f docker/Dockerfile	где лежит Dockerfile
.	контекст сборки — от него считается путь в COPY docker/entrypoint.sh
-t nav2:jazzy	имя и тег образа
Именно поэтому запускать надо из Nav_Test/, а не из docker/ — иначе COPY не найдёт файл.

⏱ Первый раз — минут десять: 1.2 ГБ базы плюс Nav2 со всеми зависимостями. Дальше пересборки будут быстрыми за счёт кэша слоёв.

2. Запуск

Проще всего — ./run.sh: он же подключается вторым терминалом к уже поднятому контейнеру.
Полный эквивалент вручную:

docker run -it --rm \
  --name nav2 \
  --network host \
  --ipc host \
  --gpus all \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models:/root/ros_ws/src/maze_nav/models \
  -v "$XAUTHORITY":/tmp/.docker.xauth:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  -v ~/Nav_Test/src:/root/ros_ws/src \
  nav2:jazzy
Разбор флагов
флаг	зачем
--network host	DDS discovery работает без настройки; без этого узлы не найдут друг друга
--ipc host	разделяемая память для DDS. Без него /dev/shm = 64 МБ, и большие топики (карты, облака точек) молча теряются
--gpus all	доступ к RTX 3080
-e NVIDIA_DRIVER_CAPABILITIES=all	критично: без него дадут CUDA, но не OpenGL, и Gazebo откроется чёрным окном
-e GZ_SIM_RESOURCE_PATH	где gz sim ищет модели и миры. Заменяет пару GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH из Gazebo Classic
-v "$XAUTHORITY":/tmp/.docker.xauth:ro	X11-cookie. У тебя он в /run/user/1000/gdm/, а не в ~/.Xauthority — рецепты из интернета тут не сработают
-v /tmp/.X11-unix	сокет X-сервера
--device /dev/dri	прямой рендеринг
-v ~/Nav_Test/src:...	твой код с хоста внутрь контейнера
--rm	удалить контейнер при выходе
Про --rm: контейнер исчезает, когда закроешь этот терминал. Всё, что ты доставил внутрь руками, пропадёт — но src/ смонтирован с хоста и сохранится. Это нормальный режим для разработки. Если захочешь, чтобы контейнер жил дольше, убери --rm и добавь -d.

3. Симулятор

ros2 launch maze_nav maze.launch.py            # лабиринт maze1 + TurtleBot3
ros2 launch maze_nav maze.launch.py gui:=false # без окна, только физика

4. Чем Jazzy отличается от Humble

Gazebo Classic 11 в Jazzy нет вообще: он EOL и не собирается под Ubuntu 24.04.
Симулятор здесь — Gazebo Harmonic (команда gz sim), а связь с ROS даёт ros_gz_bridge
вместо плагинов libgazebo_ros_*. Что из этого следует на практике:

старое (Humble)	новое (Jazzy)
gzserver / gzclient	gz sim -s / gz sim -g (через ros_gz_sim/gz_sim.launch.py)
пакет gazebo_ros	пакеты ros_gz_sim, ros_gz_bridge
GAZEBO_MODEL_PATH + GAZEBO_RESOURCE_PATH	один GZ_SIM_RESOURCE_PATH
кэш в ~/.gazebo	кэш в ~/.gz
model://ground_plane из базы моделей	земля и солнце описаны прямо в .world (иначе gz полез бы в онлайновую Fuel)
.world в формате SDF 1.6	SDF 1.10 + системные плагины Physics/SceneBroadcaster/Sensors/Imu внутри мира

Геометрия лабиринта при переезде не менялась — maze1.world перегенерирован тем же
seed (gen_maze.py --cells 6 --seed 1), так что карта maps/maze1.pgm остаётся валидной.

5. Навигация реального робота (Frob)

Контейнер крутит Nav2, робот только отдаёт данные и принимает команды.
Код робота живёт отдельно: https://github.com/dark516/Frob_robot

На роботе (Raspberry, по ssh) поднять всё разом:

ros2 launch frob_bringup bringup.launch.py

Именно launch, а не голый arduino_bridge: bringup поднимает ещё лидар,
lidar_filter, IMU, robot_state_publisher (TF по URDF) и одометрию
(энкодерная нода + EKF robot_localization). Один arduino_bridge даёт только
/cmd_vel и сырые тики энкодеров — без /odom и TF Nav2 не тронется с места.

В контейнере:

./run.sh
ros2 launch maze_nav frob.launch.py slam:=true          # построить карту
ros2 launch maze_nav frob.launch.py map:=/путь/к/map.yaml # ехать по готовой

Что должно совпадать у контейнера и робота, иначе они друг друга не увидят:
ROS_DOMAIN_ID (в образе 42) и RMW_IMPLEMENTATION (в образе rmw_fastrtps_cpp).
Проверка связи — ros2 topic list внутри контейнера: должны быть видны
/scan, /odometry/filtered и /cmd_vel.

Цепочка команд скорости в Nav2 Jazzy:
controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> /cmd_vel, и уже его слушает arduino_bridge.
Тип сообщения — geometry_msgs/Twist, как и ждёт мост; поэтому в
frob_params.yaml enable_stamped_cmd_vel не включён (в отличие от maze_params*,
где мост TurtleBot3 требует TwistStamped).

Опорные числа взяты из URDF робота и ros2_arduino_bridge: корпус — цилиндр
R=0.11 м, база колёс 0.18 м, потолок моста 0.4863 м/с и 5.4 рад/с. Базовый
фрейм — base_footprint; фрейма base_link у Frob нет вовсе.

6. Грабли перехода Humble -> Jazzy в конфигах Nav2

Их стоит знать, если будешь править params руками:

* имена плагинов только через "::". Форма nav2_navfn_planner/NavfnPlanner
  в Jazzy не существует, planner_server падает в FATAL и обрывает весь bringup;
* collision_monitor теперь входит в nav2_bringup по умолчанию и без своей
  секции параметров валит запуск на "observation_sources is not initialized";
* behavior_server больше не знает costmap_topic/footprint_topic — вместо них
  local_costmap_topic, global_costmap_topic, local_footprint_topic,
  global_footprint_topic, а фреймов стало два: local_frame и global_frame.
