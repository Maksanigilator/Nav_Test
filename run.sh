#!/usr/bin/env bash
# Запуск контейнера с ROS 2 Jazzy + Nav2 + Gazebo Harmonic.
#
#   ./run.sh                    первый терминал — поднять контейнер
#   ./run.sh                    второй/третий — подключиться к работающему
#   ./run.sh ros2 topic list    выполнить разовую команду
set -euo pipefail

IMAGE="nav2:jazzy"
NAME="nav2"
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

mkdir -p "${WS}/src"

exec docker run -it --rm \
    --name "${NAME}" \
    --network host \
    --ipc host \
    --gpus all \
    -e DISPLAY="${DISPLAY}" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    `# Один путь вместо пары GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH:` \
    `# Gazebo Harmonic ищет и модели, и миры в GZ_SIM_RESOURCE_PATH.` \
    `# Модели TurtleBot3 — первыми: model.sdf тянет меши через` \
    `# model://turtlebot3_common. Исходники пакета — следом:` \
    `# правки в моделях и мирах видны без пересборки.` \
    -e GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${PKG}/models:${PKG}/worlds" \
    -v "${XAUTH}:/tmp/.docker.xauth:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri:/dev/dri \
    `# единственный bind-mount: весь код и данные лежат в пакете` \
    -v "${WS}/src:/root/ros_ws/src" \
    `# артефакты сборки и кэш Gazebo — в томах, на хост не выносятся` \
    -v nav2_build:/root/ros_ws/build \
    -v nav2_install:/root/ros_ws/install \
    -v nav2_gz:/root/.gz \
    "${IMAGE}" "${@:-zsh}"
