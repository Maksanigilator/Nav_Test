#!/usr/bin/env zsh
set -e

# ROS сам не подхватывается — setup нужно source-ить в каждой новой сессии
source /opt/ros/jazzy/setup.zsh

# install/ появляется только после первого colcon build,
# поэтому под проверкой [ -f ] — иначе свежий контейнер падал бы при старте
# Обёртка ZED собиралась в /opt/zed_ws и подключалась ЗДЕСЬ, до нашего
# пространства. Слой ZED выключен, см. комментарий в docker/Dockerfile:
# [ -f /opt/zed_ws/install/setup.zsh ] && source /opt/zed_ws/install/setup.zsh

[ -f /root/ros_ws/install/setup.zsh ] && source /root/ros_ws/install/setup.zsh

exec "$@"
#заменяет текущий процесс оболочки (скрипта) на программу, переданную в качестве аргументов, сохраняя при этом все исходные переданные параметры