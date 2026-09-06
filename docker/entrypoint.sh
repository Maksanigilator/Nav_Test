#!/usr/bin/env zsh
set -e

# ROS сам не подхватывается — setup нужно source-ить в каждой новой сессии
source /opt/ros/humble/setup.zsh

# install/ появляется только после первого colcon build,
# поэтому под проверкой [ -f ] — иначе свежий контейнер падал бы при старте
[ -f /root/ros_ws/install/setup.zsh ] && source /root/ros_ws/install/setup.zsh

exec "$@"
#заменяет текущий процесс оболочки (скрипта) на программу, переданную в качестве аргументов, сохраняя при этом все исходные переданные параметры