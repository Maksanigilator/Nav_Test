#!/usr/bin/env bash
# Запуск контейнера с ROS 2 Jazzy + Nav2 + Gazebo Harmonic.
#
#   ./run.sh                    первый терминал — поднять контейнер
#   ./run.sh                    второй/третий — подключиться к работающему
#   ./run.sh ros2 topic list    выполнить разовую команду
set -euo pipefail

IMAGE="nav2:jazzy"

# Домен DDS и имя контейнера. Две сессии в ОДНОМ домене видят узлы друг друга
# и склеиваются в один ROS-граф, даже если это разные контейнеры: у обоих
# --network host. Симптом — два источника odom->base_footprint, разорванное
# дерево TF и "зависшая" карта при полностью свободных CPU и GPU.
# Поэтому вторую сессию поднимать так:
#     ROS_DOMAIN_ID=43 ./run.sh
# Имя контейнера тогда тоже меняется, и они не конфликтуют.
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
NAME="nav2${ROS_DOMAIN_ID:+_$ROS_DOMAIN_ID}"
[ "$ROS_DOMAIN_ID" = "42" ] && NAME="nav2"
WS="$HOME/Nav_Test"
PKG="/root/ros_ws/src/maze_nav"

# X11-cookie. Под GDM он лежит в /run/user/1000/gdm/Xauthority,
# а НЕ в ~/.Xauthority, как пишут в большинстве рецептов.
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

# Контейнер уже работает -> подключаемся к нему, а не запускаем второй
# с тем же именем. Якоря ^$ обязательны, иначе фильтр поймает и nav2_test.
if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "Контейнер ${NAME} уже запущен — подключаюсь."
    exec docker exec -it "${NAME}" "${@:-zsh}"
fi

mkdir -p "${WS}/src" "${WS}/datasets"

exec docker run -it --rm \
    --name "${NAME}" \
    --network host \
    --ipc host \
    --gpus all \
    -e DISPLAY="${DISPLAY}" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    `# Гибридная графика: без этих двух переменных OpenGL уходит на встроенную` \
    `# Intel, а не на дискретную NVIDIA. Проверяется через glxinfo -B:` \
    `# renderer должен быть NVIDIA, а не "Mesa Intel".` \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    `# Один путь вместо пары GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH:` \
    `# Gazebo Harmonic ищет и модели, и миры в GZ_SIM_RESOURCE_PATH.` \
    `# Модели TurtleBot3 — первыми: model.sdf тянет меши через` \
    `# model://turtlebot3_common. Исходники пакета — следом:` \
    `# правки в моделях и мирах видны без пересборки.` \
    -e GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${PKG}/models:${PKG}/worlds" \
    -v "${XAUTH}:/tmp/.docker.xauth:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri:/dev/dri \
    `# код и данные пакета` \
    -v "${WS}/src:/root/ros_ws/src" \
    `# датасеты (TUM RGB-D и пр.) — гигабайты, в git не лежат, см. .gitignore` \
    -v "${WS}/datasets:/datasets" \
    `# артефакты сборки и кэш Gazebo — в томах, на хост не выносятся` \
    -v nav2_build:/root/ros_ws/build \
    -v nav2_install:/root/ros_ws/install \
    -v nav2_gz:/root/.gz \
    "${IMAGE}" "${@:-zsh}"
