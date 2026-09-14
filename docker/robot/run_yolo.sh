#!/usr/bin/env bash
# Детектор дорожных знаков на самом роботе.
#
#   ./run_yolo.sh                 запустить детектор (камера + CPU)
#   ./run_yolo.sh bash            зайти внутрь контейнера
#   CAM=/dev/video2 ./run_yolo.sh взять другую камеру
#
# Отдельный контейнер от навигации: образы независимы, и детектор можно
# не возить на робота вовсе, если знаки в заезде не нужны.
#
# Кадры берутся прямо с USB-камеры, никакой сети. Поэтому sign_camera.py
# (MJPEG-сервер для ноутбука) одновременно с этим работать НЕ может:
# /dev/video0 отдаётся только одному процессу.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="yolo:jazzy-arm64"
NAME="yolo_signs"
TAR="${HERE}/yolo-arm64.tar.gz"
CAM="${CAM:-/dev/video0}"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    if [ ! -f "${TAR}" ]; then
        echo "Нет ни образа ${IMAGE}, ни архива ${TAR}." >&2
        echo "Собери на ноутбуке: ./docker/robot/build.sh --only yolo" >&2
        exit 1
    fi
    echo "==> первый запуск: загружаю образ из $(basename "${TAR}")"
    docker load -i "${TAR}"
fi

if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "Контейнер ${NAME} уже запущен — подключаюсь."
    exec docker exec -it "${NAME}" "${@:-bash}"
fi

if [ ! -e "$CAM" ]; then
    echo "Камеры ${CAM} нет. Список: $(ls /dev/video* 2>/dev/null || echo 'пусто')" >&2
    echo "Задать другую: CAM=/dev/videoN ./run_yolo.sh" >&2
    exit 1
fi

# Без аргументов сразу поднимаем детектор — больше этому контейнеру
# делать нечего. device:=cpu обязателен: параметр по умолчанию cuda:0,
# он рассчитан на ноутбук с видеокартой, и здесь узел бы упал на старте.
if [ $# -gt 0 ]; then
    CMD=("$@")
else
    CMD=(python3 /root/ros_ws/src/yolo/sign_detector.py --ros-args
         -p source:=camera
         -p device:=cpu
         -p camera_device:="${CAM}")
fi

exec docker run -it --rm \
    --name "${NAME}" \
    --network host \
    --ipc host \
    --device "${CAM}:${CAM}" \
    `# исходники: код детектора и веса dataset_v3_180.pt лежат в src/yolo` \
    -v "${HERE}/src:/root/ros_ws/src" \
    "${IMAGE}" "${CMD[@]}"
