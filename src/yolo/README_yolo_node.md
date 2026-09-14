# Пакет `yolo` — детекции + поза робота в момент кадра

Узел читает **MJPEG-поток** (его отдаёт `ffmpeg` на Raspberry Pi), прогоняет кадры
через YOLO и публикует **одно сообщение на кадр**: рамки + поза робота ровно на
момент получения этого кадра. Потребителю не нужно самому сводить два топика.

```
ffmpeg на Pi :8080/stream ──► yolo_client ──► /yolo/frame_detections
                                   ▲
                        TF (map→base_footprint) или топик позы
```

`maze_nav` этот пакет не трогает: это отдельный ament-пакет в `src/yolo`.

## Быстрый старт

```bash
# --- на хосте ---
cd ~/Nav_Test
docker build --network host -f docker/Dockerfile -t nav2:jazzy .
./run.sh                      # второй запуск подключается к работающему контейнеру

# --- внутри контейнера ---
colcon build --packages-select yolo
source /root/ros_ws/install/setup.zsh

ros2 run yolo yolo_client --ros-args \
  -p stream_url:=http://10.10.10.5:8080/stream \
  -p pose_parent_frame:=map -p pose_child_frame:=base_footprint
```

Проверка в другом терминале (`./run.sh` из корня репозитория):

```bash
ros2 topic hz /yolo/frame_detections
ros2 topic echo /yolo/frame_detections --once
ros2 topic echo /yolo/frame_detections --field pose_valid
```

## Что где лежит

| Файл | Что это |
|---|---|
| `yolo_node.py` | Узел. Ставится как `ros2 run yolo yolo_client`. |
| `msg/FrameDetections.msg` | Формат выходного сообщения. |
| `CMakeLists.txt`, `package.xml` | Сборка пакета (`ament_cmake` + `rosidl`). |
| `dataset_v3_180.pt` | Модель по умолчанию, ставится в `share/yolo/`. |
| `yolo_client_node.py` | **Старый пример**, не собирается и не устанавливается. |
| `notes.md`, `notes2.md` | Старые заметки под прежнюю схему — **устарели**. |
| `dockerfile` | Старый фрагмент с зависимостями; актуальные — в `docker/Dockerfile`. |

## Топики

| Топик | Тип | Когда есть |
|---|---|---|
| `/yolo/frame_detections` | `yolo/msg/FrameDetections` | всегда |
| `/yolo/detection_image` | `sensor_msgs/Image` (`bgr8`) | только при `publish_annotated:=true` |

```
std_msgs/Header header                     # stamp = момент получения кадра
vision_msgs/Detection2D[] detections       # рамки в пикселях кадра
geometry_msgs/PoseWithCovarianceStamped robot_pose
bool pose_valid                            # читать robot_pose только если true
```

## Параметры

Все читаются **один раз при старте** — после `ros2 param set` ничего не изменится,
нужен перезапуск узла.

### Поток и модель

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `stream_url` | `http://10.10.10.5:8080/stream` | Адрес MJPEG-потока. |
| `model_path` | `share/yolo/dataset_v3_180.pt` | Веса; свой файл подставить сюда. |
| `conf` | `0.5` | Порог уверенности. |
| `imgsz` | `640` | Размер входа сети. |
| `device` | `cpu` | `cpu` или `cuda:0`. |
| `publish_annotated` | `false` | Публиковать картинку с рамками (тяжёлая, по Wi-Fi не гонять). |

### Поза

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `pose_source` | `tf` | `tf` — искать в TF; `topic` — брать последнее сообщение из топика. |
| `pose_parent_frame` | `map` | Родительский фрейм для TF. |
| `pose_child_frame` | `base_footprint` | Фрейм робота для TF. |
| `pose_topic` | `/odometry/filtered` | Топик позы для `pose_source:=topic`. |
| `pose_topic_type` | `odometry` | `odometry` = `nav_msgs/Odometry`, `pose` = `PoseWithCovarianceStamped`. |
| `pose_max_age` | `0.5` | Сообщения старше этого (секунды) считаются отсутствующими. |

### Прочее

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `camera_frame` | `camera` | `frame_id` детекций (статичная строка, TF не ищется). |
| `output_topic` | `/yolo/frame_detections` | Куда публиковать. |

### Файл параметров вместо длинной команды

```yaml
# ~/yolo_params.yaml — путь должен существовать ВНУТРИ контейнера
/yolo_client:
  ros__parameters:
    stream_url: "http://10.10.10.5:8080/stream"
    pose_parent_frame: "map"
    pose_child_frame: "base_footprint"
    conf: 0.4
```

```bash
ros2 run yolo yolo_client --ros-args --params-file ~/yolo_params.yaml
```

Имя узла — `yolo_client`, поэтому ключ в YAML именно `/yolo_client`.

## Как адаптировать при подключении

### 1. Адрес потока

На Pi узнать IP и проверить, что ffmpeg слушает порт:

```bash
hostname -I
ss -tlnp | grep 8080
```

С хоста убедиться, что поток реально идёт (3 секунды достаточно):

```bash
curl -s --max-time 3 http://10.10.10.5:8080/stream -o /tmp/frame.mjpg
ls -la /tmp/frame.mjpg        # должен быть ненулевой размер
```

Если размер 0 или соединение висит — дело не в узле, а в ffmpeg/сети.

### 2. Фреймы позы

| Что за робот | `pose_parent_frame` | `pose_child_frame` |
|---|---|---|
| Реальный Frob | `map` | `base_footprint` |
| Симуляция TurtleBot3 | `map` | `base_link` |

У Frob **нет фрейма `base_link`** вообще. Проверить TF до запуска:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Если TF недоступен, а поза нужна сейчас — переключиться на топик:

```bash
ros2 run yolo yolo_client --ros-args \
  -p stream_url:=http://10.10.10.5:8080/stream \
  -p pose_source:=topic -p pose_topic:=/odometry/filtered \
  -p pose_topic_type:=odometry
```

Какие именно данные брать — вопрос открытый, поэтому и сделан параметр: код менять
не нужно, меняются только аргументы запуска.

### 3. Своя модель

```bash
-p model_path:=/root/ros_ws/src/yolo/best.pt
```

Классы берутся из самой модели (`results.names`), отдельный список не нужен.

### 4. Связь с роботом

`ROS_DOMAIN_ID` и `RMW_IMPLEMENTATION` должны совпадать с Pi (в образе `42` и
`rmw_fastrtps_cpp`). Проверка — `ros2 topic list` в контейнере должен видеть
топики робота.

## Диагностика

| Симптом | Что делать |
|---|---|
| `Ошибка потока: ... Переподключение через 2 с` в цикле | ffmpeg на Pi не отдаёт поток: проверить IP, `ss -tlnp \| grep 8080`. |
| `Поза недоступна (map -> base_footprint)` каждые 5 с | Нет TF. Проверить `tf2_echo` или уйти на `pose_source:=topic`. |
| `pose_valid: false` в каждом кадре | То же самое; `robot_pose` в сообщении — мусор, игнорировать. |
| Детекций нет вообще | Слишком высокий `conf`, либо модель не знает нужные классы. |
| Топик не видно из другого контейнера/машины | Разошлись `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION`. |
| Низкий FPS | `device:=cpu` и `imgsz:=640` — это дорого; уменьшить `imgsz`, смотреть строку `FPS:` в логе. |
| Позу «ведёт» относительно картинки | Не синхронизировано время Pi и контейнера (`timedatectl status`, NTP). |
| `ros2 run yolo yolo_client` не найден | Не сделан `colcon build --packages-select yolo` или не засорсен `install/setup.zsh`. |

Полезное:

```bash
ros2 param list /yolo_client
ros2 node info /yolo_client
ros2 interface show yolo/msg/FrameDetections
```

## Зависимости

`vision_msgs`, `cv_bridge` и `python3-opencv` уже есть в базовом образе;
`docker/Dockerfile` доставляет только `python3-pip` и pip-пакеты
(`ultralytics`, `requests`, `opencv-python`). При изменении зависимостей образ
надо пересобрать:

```bash
docker build --network host -f docker/Dockerfile -t nav2:jazzy .
```
