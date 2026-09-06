ARG BASE_IMAGE=osrf/ros:humble-desktop-full
FROM $BASE_IMAGE 
#базовый образ задается через переменную

RUN apt-get update && apt-get install -y \
    zsh \
    mesa-utils \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/* \
    && chsh -s "$(which zsh)" root 2>/dev/null || true
#Пакеты для системы


RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-simulations \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
#Рос пакеты

RUN sh -c "$(curl -fsSL https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -t bira \
    -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions

RUN echo 'if command -v register-python-argcomplete3 >/dev/null 2>&1; then eval "$(register-python-argcomplete3 ros2)"; fi' >> ~/.zshrc && \
    echo 'if command -v register-python-argcomplete3 >/dev/null 2>&1; then eval "$(register-python-argcomplete3 colcon)"; fi' >> ~/.zshrc
#Приколы для удобства

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo '[ -f /root/ros_ws/install/setup.bash ] && source /root/ros_ws/install/setup.bash' >> ~/.bashrc

RUN echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc && \
    echo '[ -f /root/ros_ws/install/setup.zsh ] && source /root/ros_ws/install/setup.zsh' >> ~/.zshrc
#Автоматический сорс при подключении через exec (при run срабатывет энтрипоинт?)




# ═══ Переменные времени выполнения ═══
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV GAZEBO_MODEL_DATABASE_URI=""
ENV TURTLEBOT3_MODEL=waffle
ENV QT_QPA_PLATFORM=xcb
ENV ROS_DOMAIN_ID=42
#ENV QT_QPA_PLATFORM=xcb настраивает x11

WORKDIR /root/ros_ws


COPY docker/entrypoint.sh /tmp/entrypoint.sh
RUN chmod +x /tmp/entrypoint.sh

ENTRYPOINT ["/tmp/entrypoint.sh"]
CMD ["zsh"]