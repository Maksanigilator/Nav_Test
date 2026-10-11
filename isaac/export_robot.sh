#!/usr/bin/env bash
# Достаёт описание TurtleBot3 из контейнера в вид, пригодный для импортёра USD.
#
#     ./isaac/export_robot.sh
#
# Зачем это нужно. Робот в Isaac должен быть ТОТ ЖЕ, что в этапе 2 на Gazebo:
# критерий этапа 3 — сравнение ATE с этапом 2, и если сменить заодно геометрию
# и положение камеры, разрыв будет мерить смесь реализма с другим роботом.
# Поэтому берём не готовый ассет Isaac, а штатный URDF TurtleBot3 из образа.
# Из ассетов Isaac будет ОКРУЖЕНИЕ — вот его менять и надо, ради текстур,
# бликов и честного шума глубины.
#
# Две вещи, из-за которых это не сводится к "скопировать файл":
#
# 1. turtlebot3_waffle.urdf — не чистый URDF, а xacro: имена всех линков
#    и суставов записаны как ${namespace}base_link. Импортёр получил бы
#    линки с долларами в именах. Разрешаем пустым namespace.
# 2. Меши подключены через package://turtlebot3_description/..., а эту схему
#    умеет разворачивать только ROS. На хосте, где работает Isaac, ROS нет
#    и не будет — значит переписываем пути на относительные и кладём меши рядом.

set -euo pipefail

IMAGE="nav2:jazzy"
OUT_DIR="$(cd "$(dirname "$0")" && pwd)/assets/turtlebot3_waffle"
SHARE=/opt/ros/jazzy/share/turtlebot3_description

mkdir -p "$OUT_DIR"

docker run --rm -v "$OUT_DIR:/out" "$IMAGE" bash -c "
# Без -u намеренно: setup.bash от ROS обращается к необъявленным переменным
# (AMENT_TRACE_SETUP_FILES), и под 'set -u' сорсинг падает на первой же строке.
set -eo pipefail
source /opt/ros/jazzy/setup.bash

# namespace:= с пустым значением — так подстановки исчезают, а имена линков
# становятся обычными (base_link, camera_rgb_optical_frame и так далее).
xacro $SHARE/urdf/turtlebot3_waffle.urdf namespace:= > /tmp/waffle.urdf

# package:// -> относительный путь. Импортёр USD ищет меши от места,
# где лежит сам .urdf, поэтому структура meshes/ сохраняется как есть.
sed -i 's|package://turtlebot3_description/|./|g' /tmp/waffle.urdf

cp /tmp/waffle.urdf /out/turtlebot3_waffle.urdf
cp -r $SHARE/meshes /out/
chmod -R a+rX /out
"

echo "готово: $OUT_DIR"
ls -la "$OUT_DIR"
