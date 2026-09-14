#!/usr/bin/env bash
set -e

# Отдельный от docker/entrypoint.sh файл, и это не дублирование ради
# симметрии: тот написан на zsh, а в образе робота zsh нет — база
# ros-base голая, и ставить оболочку ради одного source незачем.
source /opt/ros/jazzy/setup.bash

# install/ появляется только после первого colcon build, поэтому под
# проверкой: иначе свежезагруженный образ падал бы при первом запуске.
[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash

exec "$@"
