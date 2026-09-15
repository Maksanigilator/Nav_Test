ARG BASE_IMAGE=osrf/ros:jazzy-desktop
FROM $BASE_IMAGE 
#базовый образ задается через переменную
#desktop, а не desktop-full: в Jazzy full всё равно не тянет симулятор,
#Gazebo Harmonic приезжает отдельным пакетом ros-jazzy-ros-gz ниже

RUN apt-get update && apt-get install -y \
    zsh \
    curl \
    mesa-utils \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/* \
    && chsh -s "$(which zsh)" root 2>/dev/null || true
#Пакеты для системы


# apt-get upgrade обязателен: база osrf/ros:jazzy-desktop заморожена на
# январских пакетах (ros-jazzy-fastcdr 2.2.5), а navigation2 из репозитория
# собран под 2.2.7. Без обновления Nav2 стартует и падает на
# undefined symbol в libnav2_msgs__rosidl_typesupport_fastrtps_cpp.so.
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-slam-toolbox \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*
#Рос пакеты.
#ros-jazzy-ros-gz = Gazebo Harmonic + мост gz<->ROS. Классического
#gazebo_ros в Jazzy нет: Gazebo Classic 11 не собирается под Ubuntu 24.04.


# ═══ Слой RGB-D: навигация по RealSense вместо лидара ═══
# Отдельным слоем, чтобы правки здесь не роняли кэш всего Nav2 выше.
# Все пакеты есть бинарниками под Jazzy — собирать из исходников нечего.
RUN apt-get update && apt-get install -y \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    ros-jazzy-spatio-temporal-voxel-layer \
    ros-jazzy-depth-image-proc \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-pointcloud-to-laserscan \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
#rtabmap-ros = RGB-D SLAM: даёт /map и map->odom вместо связки map_server+AMCL,
#  плюс rtabmap_odom — визуальная одометрия вместо колёсной.
#spatio-temporal-voxel-layer — костмап-слой под depth-камеру: разреженные воксели
#  с затуханием и frustum-очисткой. Обычный VoxelLayer на плотном облаке жрёт CPU
#  и не умеет забывать препятствия, а при FOV 65° у D415 забывание обязательно.
#realsense2-camera нужен только на реальном роботе, но пусть лежит в образе:
#  так один и тот же образ годится и для симулятора, и для железа.
#imu-filter-madgwick + robot-localization — IMU и слияние источников одометрии.
#image-transport-plugins — сжатие depth/RGB: сырое облако 848x480 это ~390 МБ/с,
#  по сети такое не ходит, гоняем сжатые картинки и считаем облако у потребителя.

# rosbags — конвертер ROS1 bag -> ROS2 (датасеты TUM RGB-D лежат в формате ROS1).
# evo — подсчёт ATE траектории против ground truth.
# --break-system-packages: в Ubuntu 24.04 системный python помечен externally-managed
# (PEP 668), а заводить venv внутри одноразового контейнера смысла нет.
# pyrealsense2 нужен не для работы с живой камерой (её ведёт realsense2_camera),
# а чтобы конвертировать записи .bag от librealsense. У ROS-обёртки нет режима
# невещественного времени, из-за чего при выравнивании глубины она теряет кадры;
# см. scripts/realsense_bag_to_ros2.py и RTABMAP.md.
RUN pip3 install --no-cache-dir --break-system-packages rosbags evo pyrealsense2

RUN sh -c "$(curl -fsSL https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -t bira \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions

RUN for c in register-python-argcomplete3 register-python-argcomplete; do \
      echo "if command -v $c >/dev/null 2>&1; then eval \"\$($c ros2)\"; eval \"\$($c colcon)\"; fi" >> ~/.zshrc; \
    done
#Приколы для удобства. В Jazzy бинарь называется register-python-argcomplete
#(без «3»), поэтому проверяем оба имени.

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo '[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash' >> ~/.bashrc

RUN echo "source /opt/ros/jazzy/setup.zsh" >> ~/.zshrc && \
    echo '[ -f /root/ros_ws/install/setup.zsh ] && source /root/ros_ws/install/setup.zsh' >> ~/.zshrc
#Автоматический сорс при подключении через exec (при run срабатывет энтрипоинт?)




# ═══ Переменные времени выполнения ═══
# Fast DDS — дефолт Jazzy и то, на чём говорит реальный робот.
# Cyclone здесь стоять НЕ должен: с ним контейнер видит имена нод робота,
# но не видит их топиков, и данные не ходят вообще.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV TURTLEBOT3_MODEL=waffle
ENV QT_QPA_PLATFORM=xcb
ENV ROS_DOMAIN_ID=42
#ENV QT_QPA_PLATFORM=xcb настраивает x11
#GAZEBO_MODEL_DATABASE_URI больше не нужен: у gz своя Fuel, а мир лабиринта
#её не трогает — земля и солнце описаны прямо в .world

WORKDIR /root/ros_ws


COPY docker/entrypoint.sh /tmp/entrypoint.sh
RUN chmod +x /tmp/entrypoint.sh

ENTRYPOINT ["/tmp/entrypoint.sh"]
CMD ["zsh"]
