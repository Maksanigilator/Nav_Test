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
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
NAME="nav2${ROS_DOMAIN_ID:+_$ROS_DOMAIN_ID}"
[ "$ROS_DOMAIN_ID" = "0" ] && NAME="nav2"
# Корень проекта — КАТАЛОГ ЭТОГО СКРИПТА, а не фиксированный путь.
#
# Раньше здесь стояло "$HOME/Nav_Test", и это молча ломалось при клоне в
# любое другое место: mkdir -p ниже создавал пустой ~/Nav_Test/src,
# монтировал его, и контейнер поднимался БЕЗ КОДА. Ни ошибки, ни
# предупреждения — просто пакета maze_nav не оказывалось.
#
# readlink -f разворачивает симлинк на сам run.sh: его удобно класть в
# PATH, и тогда $0 указывает на ссылку, а не на проект. pwd -P убирает
# симлинки уже из пути каталога — докеру нужен ФИЗИЧЕСКИЙ путь хоста,
# по ссылке он смонтирует не то.
#
# Известный предел: имя контейнера по-прежнему зависит только от
# ROS_DOMAIN_ID. Два клона на ОДНОЙ машине в одном домене будут
# считать своим один и тот же контейнер "nav2", и второй запуск
# подключится к первому. Разводятся они так же, как две сессии:
#     ROS_DOMAIN_ID=43 ./run.sh
WS="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
PKG="/root/ros_ws/src/maze_nav"

# X11-cookie. Под GDM он лежит в /run/user/1000/gdm/Xauthority,
# а НЕ в ~/.Xauthority, как пишут в большинстве рецептов.
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

# Терминал Cursor и ssh не получают DISPLAY. Своего экрана нет —
# поднимаем виртуальный :99 и noVNC, RViz открывается в браузере.
# Пакеты Xvfb/x11vnc/novnc должны быть в образе (docker/Dockerfile.novnc).
DISPLAY="${DISPLAY:-}"
USE_NOVNC=0
if [ -z "${DISPLAY}" ]; then
    USE_NOVNC=1
    DISPLAY=":99"
    echo "Своего экрана нет — RViz пойдёт через noVNC, дисплей ${DISPLAY}."
fi

# Контейнер уже работает -> подключаемся к нему, а не запускаем второй
# с тем же именем. Якоря ^$ обязательны, иначе фильтр поймает и nav2_test.
if [ -n "$(docker ps -q -f name="^${NAME}$")" ]; then
    echo "Контейнер ${NAME} уже запущен — подключаюсь."
    # ЧЕРЕЗ ИНТЕРАКТИВНЫЙ zsh, А НЕ НАПРЯМУЮ. docker exec не проходит через
    # точку входа, и ROS в PATH не попадает: `./run.sh rviz2` отвечал
    # «executable file not found», хотя rviz2 в образе есть. Оболочка с
    # -i читает ~/.zshrc, а он подключает и /opt/ros, и оба наших
    # пространства.
    #
    # Идиома `zsh -ic 'exec "$@"' zsh "$@"` передаёт слова как есть, не
    # склеивая их в строку: иначе развалились бы аргументы с пробелами и
    # кавычками, вроде gyro_bias:='[0.13, 0.004, 0.05]'.
    if [ $# -eq 0 ]; then
        exec docker exec -it "${NAME}" zsh
    fi
    exec docker exec -it "${NAME}" zsh -ic 'exec "$@"' zsh "$@"
fi

mkdir -p "${WS}/src" "${WS}/datasets"

# noVNC стартует в том же контейнере, до команды пользователя, и отдаёт
# ей уже поднятый DISPLAY. Отдельный процесс на хосте не нужен: сеть host,
# порт 6080 слушается прямо здесь.
if [ "${USE_NOVNC}" = 1 ]; then
    if [ $# -eq 0 ]; then
        set -- /opt/nav2/novnc.sh zsh
    else
        set -- /opt/nav2/novnc.sh "$@"
    fi
fi

exec docker run -it --rm \
    --name "${NAME}" \
    --network host \
    `# --ipc host обязателен: без него у контейнера СВОЙ /dev/shm, и` \
    `# разделяемая память между ним и хостом не заработает ни при каких правах.` \
    --ipc host \
    `# Тот же UID, что у хоста. Ради разделяемой памяти Fast DDS: она` \
    `# доставляет данные записью в файлы /dev/shm, созданные с правами 0644` \
    `# и владельцем-создателем, поэтому при разных пользователях доставка` \
    `# с хоста в контейнер запрещена — узлы и топики видны, данных нет.` \
    `# Подробности и замеры — в docker/Dockerfile.` \
    --user "$(id -u):$(id -g)" \
    `# /root внутри образа открыт на запись (см. Dockerfile), а HOME нужен` \
    `# явно: под --user он не подставляется, и оболочка не найдёт ни .zshrc,` \
    `# ни настройки ros.` \
    -e HOME=/root \
    `# Группы video и render — доступ к /dev/dri. На этой машине их заменяет` \
    `# ACL, который logind вешает на устройства для активной сессии, но ACL` \
    `# есть не всегда (headless, другая машина), а лишние группы безвредны.` \
    $(getent group video  >/dev/null && echo --group-add "$(getent group video  | cut -d: -f3)") \
    $(getent group render >/dev/null && echo --group-add "$(getent group render | cut -d: -f3)") \
    `# Видеокарта NVIDIA. Ключ безусловным быть НЕ МОЖЕТ: на машине без` \
    `# NVIDIA докер отвечает "could not select device driver" и контейнер` \
    `# не стартует вовсе. Переменная NO_NVIDIA ставится из run_no_nvidia.sh.` \
    $([ -z "${NO_NVIDIA:-}" ] && echo --gpus all) \
    -e DISPLAY="${DISPLAY}" \
    -e XAUTHORITY=/tmp/.docker.xauth \
    $([ -z "${NO_NVIDIA:-}" ] && printf "%s\\n" -e NVIDIA_DRIVER_CAPABILITIES=all) \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    `# Транспорт Fast DDS НЕ навязываем: контейнер работает под тем же UID,` \
    `# что и процессы на хосте (см. --user ниже), поэтому разделяемая память` \
    `# доступна обеим сторонам и выбирается автоматически. Это важно для` \
    `# тяжёлых потоков: цвет плюс глубина с камеры 848x480 это ~136 МБ/с,` \
    `# а UDP по петле столько не тянет — замерено, глубина теряет кадры.` \
    `# Принудить UDP можно так:  FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ./run.sh` \
    ${FASTDDS_BUILTIN_TRANSPORTS:+-e FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS"} \
    `# На настоящем экране OpenGL уводим на NVIDIA, иначе RViz садится` \
    `# на встроенную Intel. На виртуальном дисплее noVNC драйвер NVIDIA` \
    `# не к чему привязать — там программный рендерер, иначе окно чёрное.` \
    $(if [ "${USE_NOVNC}" = 1 ]; then
         printf '%s\n' -e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe \
             -e QT_QPA_PLATFORM=xcb -e QT_X11_NO_MITSHM=1 \
             -e MESA_GL_VERSION_OVERRIDE=3.3
     elif [ -z "${NO_NVIDIA:-}" ]; then
         printf '%s\n' -e __NV_PRIME_RENDER_OFFLOAD=1 \
             -e __GLX_VENDOR_LIBRARY_NAME=nvidia
     fi) \
    `# Третьего случая раньше не было, а он есть: НАСТОЯЩИЙ экран БЕЗ` \
    `# NVIDIA. Тогда не задаём НИЧЕГО — Mesa сама возьмёт встроенную` \
    `# видеокарту через /dev/dri, который уже смонтирован ниже. Ветку` \
    `# noVNC сюда переиспользовать нельзя: там LIBGL_ALWAYS_SOFTWARE=1,` \
    `# и на Iris Xe с настоящим аппаратным GL это только замедлит RViz.` \
    `# Один путь вместо пары GAZEBO_MODEL_PATH/GAZEBO_RESOURCE_PATH:` \
    `# Gazebo Harmonic ищет и модели, и миры в GZ_SIM_RESOURCE_PATH.` \
    `# Модели TurtleBot3 — первыми: model.sdf тянет меши через` \
    `# model://turtlebot3_common. Исходники пакета — следом:` \
    `# правки в моделях и мирах видны без пересборки.` \
    -e GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${PKG}/models:${PKG}/worlds" \
    `# Файла cookie может не быть: терминал без графической сессии.` \
    `# Монтировать отсутствующий путь нельзя — docker создаст на его` \
    `# месте каталог и затрёт ~/.Xauthority.` \
    $([ -f "${XAUTH}" ] && echo "-v${XAUTH}:/tmp/.docker.xauth:ro") \
    -v "${WS}/docker/novnc.sh:/opt/nav2/novnc.sh:ro" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/dri:/dev/dri \
    `# Доступ к RealSense. Камера — обычное USB-устройство, но librealsense` \
    `# переоткрывает его при смене режима, поэтому пробросить один` \
    `# /dev/bus/usb/NNN/MMM нельзя: после переоткрытия номер другой.` \
    `# Отсюда весь /dev плюс правила cgroup на классы устройств:` \
    `# 81 — video4linux (потоки камеры), 189 — USB-устройства.` \
    -v /dev:/dev \
    `# /sys НА ЗАПИСЬ — ради ИНС у D435i. Гироскоп с акселерометром` \
    `# подключены как HID-датчики ядра и управляются через sysfs:` \
    `# librealsense включает каналы записью в` \
    `# /sys/bus/iio/devices/iio:deviceN/scan_elements/in_anglvel_*_en.` \
    `# Docker монтирует /sys только на чтение, и драйвер падает с` \
    `# "Read-only file system" и "Hid device is busy" — камера при этом` \
    `# видна и перечисляется, но ни один топик не публикуется.` \
    `# Права на сами файлы уже нужные: правила udev от librealsense` \
    `# выставляют им 0666, так что достаточно перемонтировать на запись.` \
    -v /sys:/sys \
    --device-cgroup-rule "c 81:* rmw" \
    --device-cgroup-rule "c 189:* rmw" \
    `# 166 — последовательные порты USB CDC (/dev/ttyACM*). Сюда попадают` \
    `# платы с собственным USB: микроконтроллеры, ИНС, отладчики.` \
    --device-cgroup-rule "c 166:* rmw" \
    `# 188 — последовательные порты через переходник USB (/dev/ttyUSB*).` \
    `# Через такой подключена ИНС WT901C-485: у неё промышленный RS485, и` \
    `# она висит за переходником CH340, а не напрямую по USB CDC.` \
    `#` \
    `# Без этого правила порт выглядит исправным — файл на месте, права` \
    `# 0666, — но открытие даёт «Operation not permitted». Запрещает его` \
    `# не владелец файла, а cgroup устройств самого контейнера, и по` \
    `# сообщению об ошибке это не прочитать.` \
    --device-cgroup-rule "c 188:* rmw" \
    `# Группа dialout владеет /dev/ttyACM*, без неё порт виден, но не` \
    `# открывается под непривилегированным пользователем.` \
    $(getent group dialout >/dev/null && echo --group-add "$(getent group dialout | cut -d: -f3)") \
    `# Правила udev от librealsense дают доступ группе plugdev. Без неё` \
    `# под непривилегированным пользователем камера видна, но не читается.` \
    $(getent group plugdev >/dev/null && echo --group-add "$(getent group plugdev | cut -d: -f3)") \
    `# код и данные пакета` \
    -v "${WS}/src:/root/ros_ws/src" \
    `# стенд проверки передачи кадров в контейнер с YOLO: лежит в корне` \
    `# проекта, а не в пакете, потому что к сборке ROS отношения не имеет` \
    -v "${WS}/yolo_transport_test:/root/ros_ws/yolo_transport_test" \
    `# Профиль транспорта Fast DDS: поднимает сегмент разделяемой памяти` \
    `# с дефолтных ~512 КБ до 64 МБ. Кадр камеры 1.22 МБ в дефолтный сегмент` \
    `# не влезает, и разделяемая память для него не работает — замерено,` \
    `# выходит 22 Гц вместо 30, хуже чем по UDP. Подробности в самом файле.` \
    -v "${WS}/fastdds_profiles.xml:/root/fastdds_profiles.xml:ro" \
    -e FASTDDS_DEFAULT_PROFILES_FILE=/root/fastdds_profiles.xml \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_profiles.xml \
    `# датасеты (TUM RGB-D и пр.) — гигабайты, в git не лежат, см. .gitignore` \
    -v "${WS}/datasets:/datasets" \
    `# артефакты сборки и кэш Gazebo — в томах, на хост не выносятся` \
    -v nav2_build:/root/ros_ws/build \
    -v nav2_install:/root/ros_ws/install \
    -v nav2_gz:/root/.gz \
    "${IMAGE}" "${@:-zsh}"
