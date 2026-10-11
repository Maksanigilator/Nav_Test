#!/usr/bin/env zsh
set -e

# ROS сам не подхватывается — setup нужно source-ить в каждой новой сессии
source /opt/ros/jazzy/setup.zsh

# install/ появляется только после первого colcon build,
# поэтому под проверкой [ -f ] — иначе свежий контейнер падал бы при старте
# Обёртка ZED собрана отдельно, в /opt/zed_ws. Подключаем ДО нашего
# пространства, чтобы наши пакеты при совпадении имён были главнее.
[ -f /opt/zed_ws/install/setup.zsh ] && source /opt/zed_ws/install/setup.zsh

[ -f /root/ros_ws/install/setup.zsh ] && source /root/ros_ws/install/setup.zsh

exec "$@"
#заменяет текущий процесс оболочки (скрипта) на программу, переданную в качестве аргументов, сохраняя при этом все исходные переданные параметры