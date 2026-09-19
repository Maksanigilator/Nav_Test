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
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-mecanum-drive-controller \
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

# Скачиваем В ФАЙЛ, а не в подстановку команды.
#
# Канонический рецепт `sh -c "$(curl -fsSL ...)"` молча проглатывает сбой
# сети: curl с -f отдаёт пустоту, sh послушно выполняет пустой скрипт и
# возвращает НОЛЬ, сборка идёт дальше как ни в чём не бывало. Именно так
# один раз и вышло: образ собрался «успешно», но oh-my-zsh в нём не было
# вовсе, и в контейнере встречал голый промпт вместо темы bira.
# Отдельным -o файлом curl падает с ненулевым кодом, и сборка встаёт.
#
# Проверка после установки — не паранойя: скрипт может отработать
# наполовину (тема скачалась, плагины нет), и это тоже надо ловить здесь,
# а не обнаруживать потом в живом контейнере.
RUN curl -fsSL -o /tmp/zsh-in-docker.sh \
      https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh \
    && sh /tmp/zsh-in-docker.sh \
        -t bira \
        -p git \
        -p https://github.com/zsh-users/zsh-autosuggestions \
        -p https://github.com/zsh-users/zsh-completions \
    && rm /tmp/zsh-in-docker.sh \
    && test -d /root/.oh-my-zsh \
    && grep -q 'ZSH_THEME="bira"' /root/.zshrc

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




# ═══ Запуск под пользователем хоста ═══
# Контейнер запускается с --user $(id -u):$(id -g), см. run.sh.
#
# Причина не в безопасности, а в РАЗДЕЛЯЕМОЙ ПАМЯТИ. Fast DDS доставляет
# сообщение, записывая в «почтовый ящик» получателя в /dev/shm, и создаёт эти
# файлы с правами 0644 и владельцем-создателем, причём выставляет их
# ПРИНУДИТЕЛЬНО, игнорируя umask (проверено). Пока контейнер работал под root,
# а процесс на хосте — под обычным пользователем, запись с хоста в root-овский
# ящик была запрещена, и данные не шли ВООБЩЕ. Симптом крайне обманчив:
# обнаружение идёт не через разделяемую память, поэтому `ros2 node list` и
# `ros2 topic list` показывают чужую сторону целиком, а `ros2 topic hz` молчит.
#
# Обойти это можно принудительным UDP (FASTDDS_BUILTIN_TRANSPORTS=UDPv4), и так
# и было сделано сначала. Но у UDP по петле есть потолок: кадр 1.2 МБ режется
# на ~1100 датаграмм, и при потере одного фрагмента теряется весь кадр.
# Замерено: 36.8 МБ/с проходит идеально, а цвет плюс глубина с камеры просят
# 136 МБ/с — и глубина начинает терять кадры.
#
# Совпадение UID убирает причину. Побочно исчезает вторая неприятность: файлы,
# которые контейнер пишет в смонтированные src/ и datasets/, перестают
# появляться с владельцем root.
#
# Каталоги создаём ЗАРАНЕЕ и с открытыми правами не для красоты: именованный
# том наследует владельца и права из образа в момент первого создания. Если
# build/install/.gz появятся root-овскими, том окажется недоступен на запись,
# и colcon build упадёт с отказом в доступе.
RUN mkdir -p /root/ros_ws/src /root/ros_ws/build /root/ros_ws/install /root/.gz \
    && chmod -R a+rwX /root \
    `# А вот каталоги дополнений zsh открывать всем НЕЛЬЗЯ. Строкой выше` \
    `# они тоже стали доступны на запись, и zsh при каждом входе спрашивает` \
    `# "insecure directories ... abort compinit [n]?". Ответа он не получает,` \
    `# дополнения не поднимаются, а следом падает register-python-argcomplete` \
    `# с "command not found: compdef" — то есть пропадает и дополнение ros2.` \
    `# Писать в них некому: под рабочим пользователем они только читаются.` \
    `# Владельцем оставляем root — так compaudit доволен при ЛЮБОМ UID,` \
    `# а не только при 1000.` \
    && chmod -R go-w /root/.oh-my-zsh
#/root по умолчанию 700 — под другим UID туда не попасть вовсе, а там лежат
#и рабочее пространство, и настройки zsh с oh-my-zsh, которые оболочка читает
#при каждом входе. HOME=/root контейнеру задаёт run.sh.


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
