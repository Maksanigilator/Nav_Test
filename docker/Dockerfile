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
