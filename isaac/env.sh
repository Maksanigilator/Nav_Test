# Окружение для запуска ROS 2 на ХОСТЕ библиотеками, встроенными в Isaac Sim.
#
#     source isaac/env.sh
#
# Зачем. На хосте Ubuntu 22.04, где нативного ROS 2 Jazzy нет и быть не может
# (Jazzy собирается под 24.04). Но он и не нужен: пакет isaacsim привозит с собой
# полный комплект библиотек Jazzy для своего ros2_bridge, и они работают как
# обычный ROS 2 — из них поднимается нода в голом python, без Isaac вообще.
# Именно на этом строится вся отладка: транспорт и топики проверяются за секунды,
# а не за минуты старта симулятора.
#
# Расширение bridge везёт ДВА дистрибутива — jazzy и humble. Берём jazzy:
# контейнер nav2:jazzy тоже Jazzy, а разные дистрибутивы по DDS не разговаривают.
#
# Настройка ros_distro у самого расширения по умолчанию "system_default" —
# то есть "взять то, что засорсено в терминале". Раз мы сорсим отсюда jazzy,
# Isaac подхватит его же, и отдельно переключать расширение не нужно.

ISAAC_ENV="$HOME/miniconda3/envs/env_isaaclab"
ROS2_BRIDGE="$ISAAC_ENV/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge"

if [ ! -d "$ROS2_BRIDGE/jazzy" ]; then
    echo "isaac/env.sh: не найден $ROS2_BRIDGE/jazzy" >&2
    return 1 2>/dev/null || exit 1
fi

export LD_LIBRARY_PATH="$ROS2_BRIDGE/jazzy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$ROS2_BRIDGE/jazzy/rclpy:${PYTHONPATH:-}"

# ROS_DISTRO — ИМЕННО по ней ros2_bridge выбирает, какой из двух встроенных
# дистрибутивов грузить. Без неё пути к библиотекам не помогают: Isaac запускает
# пробник isaacsim.ros2.bridge.check, тот падает (malloc assertion, core dumped,
# и на это уходит 80 секунд старта), мост сваливается в резервный вариант,
# печатает "Using backup internal ROS2 humble distro" и не поднимается вовсе —
# "ROS2 Bridge startup failed". Узлов isaacsim.ros2.bridge.* при этом не
# существует, и построение любого графа OmniGraph падает на неизвестном типе.
# Для голого rclpy (как в dds_probe.py) переменная не нужна — только для моста.
export ROS_DISTRO=jazzy

# Fast DDS — дефолт Jazzy и то, что стоит в образе nav2:jazzy (см. Dockerfile).
# Cyclone тут ставить НЕЛЬЗЯ по той же причине, что и в контейнере: стороны
# увидят имена нод, но не увидят топиков.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Транспорт НЕ навязываем — работает разделяемая память, и это важно.
#
# Сначала здесь стоял FASTDDS_BUILTIN_TRANSPORTS=UDPv4. Он был нужен, пока
# контейнер работал под root, а Isaac на хосте — под обычным пользователем:
# Fast DDS доставляет данные записью в файлы /dev/shm, созданные с правами
# 0644 и владельцем-создателем (принудительно, umask игнорируется), поэтому
# запись с хоста в root-овский сегмент была запрещена. Топики и ноды при этом
# видны с обеих сторон, а данных нет вообще.
#
# Теперь контейнер запускается под тем же UID (--user в run.sh), препятствие
# снято, и разделяемая память выбирается сама. Так и надо: UDP по петле режет
# кадр 1.2 МБ на ~1100 датаграмм, и на потоке камеры (цвет плюс глубина,
# ~136 МБ/с) начинает терять кадры. Замерено: 36.8 МБ/с проходит идеально,
# на 136 МБ/с глубина расходится с цветом — совпадало 85% штампов вместо 100%.
#
# Вернуть UDP, если понадобится сравнить:
#     export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
# Тогда пригодится и поднятие приёмных буферов ядра, иначе фрагменты теряются:
#     sudo sysctl -w net.core.rmem_max=16777216 net.core.rmem_default=16777216
# (rmem_default тоже обязателен: UDP-транспорт Fast DDS запрашивает системный
# дефолт, а не потолок.)

# Тот же домен, что у контейнера по умолчанию (см. run.sh). Isaac на хосте
# и один контейнер в одном домене — это норма и то, что нам нужно.
# Предупреждение из run.sh про конфликт доменов касается ДВУХ КОНТЕЙНЕРОВ,
# каждый из которых поднимает свой odom->base_footprint.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Python из conda-окружения: bundled rclpy собран под 3.11, системный
# python3 на 22.04 — 3.10, и модули расширения туда не загрузятся.
# Профиль транспорта Fast DDS — тот же файл, что монтируется контейнеру
# в run.sh. Поднимает сегмент разделяемой памяти с дефолтных ~512 КБ до 64 МБ:
# кадр камеры 1.22 МБ в дефолтный не помещается, и разделяемая память для него
# просто не задействуется. Обе стороны обязаны читать ОДИН профиль, иначе
# договориться о транспорте они не смогут.
_PROFILES="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)/fastdds_profiles.xml"
if [ -f "$_PROFILES" ]; then
    export FASTDDS_DEFAULT_PROFILES_FILE="$_PROFILES"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_PROFILES"
else
    echo "isaac/env.sh: профиль $_PROFILES не найден, транспорт по умолчанию" >&2
fi

export ISAAC_PYTHON="$ISAAC_ENV/bin/python"

echo "ROS 2 Jazzy (bundled Isaac Sim), domain ${ROS_DOMAIN_ID}, ${RMW_IMPLEMENTATION}"
echo "python: \$ISAAC_PYTHON = ${ISAAC_PYTHON}"
