#!/usr/bin/env bash
# Запуск контейнера Nav2 на самом роботе. Лежит в той же папке, что и
# образ с исходниками, и работает от неё — где эту папку положили на
# Raspberry, неважно.
#
#   ./run.sh                   поднять контейнер (или войти в работающий)
#   ./run.sh ros2 topic list   выполнить разовую команду внутри
#
# Окна (RViz, редактор маршрута) открываются здесь же, а показываются на
# ноутбуке. Для этого заходить на робота надо с пробросом X:
#
#   ssh -X pi@10.0.0.5
#
# Скрипт сам подхватит DISPLAY и cookie и передаст их внутрь контейнера.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="nav2:jazzy-arm64"
NAME="nav2"
TAR="${HERE}/nav2-arm64.tar.gz"

# ── Образ ─────────────────────────────────────────────────────────────
# Ещё не загружен — берём из архива рядом. Ради этого папка и собирается
# целиком: на роботе не нужен ни интернет, ни реестр образов.
if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    if [ ! -f "${TAR}" ]; then
        echo "Нет ни образа ${IMAGE}, ни архива ${TAR}." >&2
        echo "Собери на ноутбуке: ./docker/robot/build.sh" >&2
        exit 1
    fi
    echo "==> первый запуск: загружаю образ из $(basename "${TAR}") (минуту-другую)"
    docker load -i "${TAR}"
fi

if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "Контейнер ${NAME} уже запущен — подключаюсь."
    exec docker exec -it -e DISPLAY="${DISPLAY:-}" "${NAME}" "${@:-bash}"
fi

# ── Проброс X ─────────────────────────────────────────────────────────
# DISPLAY бывает двух видов, и они требуют разного:
#   localhost:10.0  — пришли по ssh -X. Икс-трафик идёт по TCP на сам
#                     Raspberry, и с --network host контейнер до него
#                     достучится. Нужен только cookie из ~/.Xauthority.
#   :0              — монитор воткнут прямо в плату. Тогда связь идёт
#                     через сокет в /tmp/.X11-unix, его и монтируем.
X_ARGS=()
if [ -n "${DISPLAY:-}" ]; then
    X_ARGS+=(-e "DISPLAY=${DISPLAY}")

    XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -f "$XAUTH" ]; then
        X_ARGS+=(-e XAUTHORITY=/tmp/.docker.xauth -v "${XAUTH}:/tmp/.docker.xauth:ro")
    else
        echo "Предупреждение: не нашёл cookie ${XAUTH}." >&2
        echo "На роботе должен стоять xauth (sudo apt install xauth), иначе" >&2
        echo "ssh -X не создаёт файл и окна не откроются." >&2
    fi

    case "$DISPLAY" in
        :*) [ -d /tmp/.X11-unix ] && X_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw) ;;
    esac

    # GPU платы, если он есть. RViz его почти наверняка не возьмёт — Ogre
    # просит OpenGL 3.3, а V3D столько не даёт, — и упадёт на llvmpipe из
    # образа. Пробрасываем на случай, если возьмёт: софтверный рендер
    # заметно медленнее.
    [ -d /dev/dri ] && X_ARGS+=(--device /dev/dri:/dev/dri)
else
    echo "DISPLAY пуст: окна не откроются. Заходи через ssh -X, если нужны" >&2
    echo "RViz и редактор маршрута. Навигация без них работает." >&2
fi

# --network host и --ipc host обязательны для DDS: в отдельной сети
# контейнер и нативные узлы робота увидят друг друга по именам, но
# данные ходить не будут. ROS_DOMAIN_ID зашит в образ (42).
exec docker run -it --rm \
    --name "${NAME}" \
    --network host \
    --ipc host \
    "${X_ARGS[@]}" \
    `# исходники проекта — копия, разложенная build.sh рядом` \
    -v "${HERE}/src:/root/ros_ws/src" \
    `# артефакты colcon — в томах: пересборка после перезапуска не нужна` \
    -v nav2_build:/root/ros_ws/build \
    -v nav2_install:/root/ros_ws/install \
    "${IMAGE}" "${@:-bash}"
