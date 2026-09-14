#!/usr/bin/env bash
# Контейнер с YOLO: детектор знаков по запросу.
#
#   ./run_yolo.sh                 первый терминал — поднять контейнер
#   ./run_yolo.sh                 второй — подключиться к работающему
#   ./run_yolo.sh python3 ...     выполнить разовую команду
#
# Образ yolo:latest собран отдельным проектом (копия его Dockerfile лежит
# в src/yolo/docker/). Здесь он только запускается: torch с CUDA и
# ultralytics в нём уже есть, ставить ничего не нужно.
#
# Домен и RMW обязаны совпадать с контейнером Nav2 и с роботом, иначе
# сервис /detect_signs просто не будет виден с той стороны.
set -euo pipefail

IMAGE="yolo:latest"
NAME="yolo_signs"
WS="$HOME/Nav_Test"

XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "Контейнер ${NAME} уже запущен — подключаюсь."
    exec docker exec -it "${NAME}" "${@:-zsh}"
fi

exec docker run -it --rm \
    --name "${NAME}" \
    --network host \
    --ipc host \
    --gpus all \
    -e DISPLAY="${DISPLAY}" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -v "${XAUTH}:/tmp/.docker.xauth:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri:/dev/dri \
    `# весь репозиторий целиком: ноде нужны и свой код в src/yolo/,` \
    `# и веса из src/yolo — второй копии модели заводить незачем` \
    -v "${WS}:/root/ros_ws" \
    "${IMAGE}" "${@:-zsh}"
