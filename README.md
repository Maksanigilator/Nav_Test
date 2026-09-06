1. Сборка

cd ~/Nav_Test
docker build -f docker/Dockerfile -t nav2:humble .
часть	смысл
-f docker/Dockerfile	где лежит Dockerfile
.	контекст сборки — от него считается путь в COPY docker/entrypoint.sh
-t nav2:humble	имя и тег образа
Именно поэтому запускать надо из Nav_Test/, а не из docker/ — иначе COPY не найдёт файл.

⏱ Первый раз — минут десять: 1.2 ГБ базы плюс Nav2 со всеми зависимостями. Дальше пересборки будут быстрыми за счёт кэша слоёв.

2. Запуск

docker run -it --rm \
  --name nav2 \
  --network host \
  --ipc host \
  --gpus all \
  -e DISPLAY="$DISPLAY" \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v "$XAUTHORITY":/tmp/.docker.xauth:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri:/dev/dri \
  -v ~/Nav_Test/src:/root/ros_ws/src \
  nav2:humble
Разбор флагов
флаг	зачем
--network host	DDS discovery работает без настройки; без этого узлы не найдут друг друга
--ipc host	разделяемая память для DDS. Без него /dev/shm = 64 МБ, и большие топики (карты, облака точек) молча теряются
--gpus all	доступ к RTX 3080
-e NVIDIA_DRIVER_CAPABILITIES=all	критично: без него дадут CUDA, но не OpenGL, и Gazebo откроется чёрным окном
-v "$XAUTHORITY":/tmp/.docker.xauth:ro	X11-cookie. У тебя он в /run/user/1000/gdm/, а не в ~/.Xauthority — рецепты из интернета тут не сработают
-v /tmp/.X11-unix	сокет X-сервера
--device /dev/dri	прямой рендеринг
-v ~/Nav_Test/src:...	твой код с хоста внутрь контейнера
--rm	удалить контейнер при выходе
Про --rm: контейнер исчезает, когда закроешь этот терминал. Всё, что ты доставил внутрь руками, пропадёт — но src/ смонтирован с хоста и сохранится. Это нормальный режим для разработки. Если захочешь, чтобы контейнер жил дольше, убери --rm и добавь -d.




