#!/usr/bin/env python3
"""Детектор дорожных знаков по запросу.

Работает в двух местах, один и тот же файл:

  * на ноутбуке, в образе yolo:latest — инференс на видеокарте, кадры
    забираются с робота по HTTP (source: snapshot или stream);
  * на самом роботе, в образе yolo:jazzy-arm64 — инференс на процессоре
    Pi, кадры берутся прямо с /dev/video0 (source: camera). Сеть в этом
    случае не участвует вообще, и детектор продолжает работать, даже
    если Wi-Fi отвалился.

Живёт в отдельном контейнере, а не рядом с Nav2: torch с ultralytics
весят больше, чем вся навигация, и таскать их в образ, который
пересобирается от каждой правки YAML, ни к чему.

Работает НЕ потоком, а по запросу. Пока никто не спрашивает, к роботу не
идёт ни одного байта — это принципиально: Wi-Fi у нас делят лидар, TF и
одометрия, и постоянный видеопоток их душит.

Один вызов сервиса — один цикл:

    1. запросить у робота несколько отдельных снимков (GET /snapshot);
    2. прогнать их через YOLO;
    3. свести детекции в один ответ и опубликовать его.

Именно отдельные снимки, а не кусок видеопотока. Открытие MJPEG-потока
ради нескольких кадров оказалось хрупким: приходилось ждать, пока
накопится нужное число кадров на частоте камеры, и на загруженном канале
это регулярно заканчивалось "Read timed out" или "успели взять 0 кадров
из 5". Короткий GET за одной картинкой либо проходит, либо нет — и не
держит соединение открытым всё это время.

Режим потока остался в параметре source на случай, если понадобится
серия строго подряд идущих кадров.

Кадров берём несколько, а не один: одиночный снимок легко ловит смаз или
засветку, и робот примет неверное решение о маршруте. Мы в этот момент
стоим, лишние полсекунды ничего не стоят.

Запуск в контейнере yolo:latest:

    python3 /root/ros_ws/src/yolo/sign_detector.py --ros-args \
        -p stream_url:=http://192.168.1.99:8080/stream \
        -p device:=cuda:0

Вызов из контейнера с Nav2 (или откуда угодно в том же домене):

    ros2 service call /detect_signs std_srvs/srv/Trigger

Выход:
    /sign_detected    std_msgs/String — имя знака, понятное route_graph,
                      пустая строка означает «ничего не увидели»;
    /sign_detections  std_msgs/String — тот же результат в JSON, с
                      уверенностью и рамками, для отладки и логов;
    /sign_image       sensor_msgs/Image — кадр с нарисованными рамками,
                      чтобы глазами убедиться, что видит детектор.

Тот же кадр пишется файлом (параметр annotated_path), это удобнее RViz:
файл лежит в примонтированном репозитории и открывается с хоста обычной
смотрелкой, RViz для разовой проверки поднимать не нужно.

Имя точки графа сюда НЕ подставляется: где стоит робот, знает
route_follower, он и соберёт строку «точка:знак» для /route_sign.
"""
import json
import time

import cv2
import numpy as np
import requests
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ultralytics import YOLO


# Классы модели dataset_v3_180.pt названы цифрами и идут подряд по
# Приложению Б регламента «Города РТК». Имена справа — те, что понимает
# RouteGraph.apply_sign в пакете maze_nav.
SIGN_BY_CLASS = {
    '1': 'only_straight',   # Б.1 движение прямо
    '2': 'only_left',       # Б.2 движение налево
    '3': 'only_right',      # Б.3 движение направо
    '4': 'no_left',         # Б.4 поворот налево запрещён
    '5': 'no_right',        # Б.5 поворот направо запрещён
    '6': 'stop',            # Б.6 место остановки
    '7': 'parking',         # Б.7 место стоянки
    # Б.8 «опасный объект» маршрут не меняет: ограждение и так видит лидар,
    # костмапа объедет его сама. Класс логируем, но знаком не считаем.
    '8': None,
}


def make_session():
    """HTTP-сессия, не заглядывающая в переменные окружения.

    trust_env=False: requests по умолчанию читает http_proxy, и запрос к
    роботу в локальной сети уходит в прокси VPN, который отвечает 502.
    Робот рядом, прокси для него не нужен ни при каких обстоятельствах.
    """
    session = requests.Session()
    session.trust_env = False
    return session


def fetch_snapshots(url, count, timeout, gap, logger):
    """Забрать count отдельных снимков короткими запросами.

    Каждый снимок — самостоятельный GET: соединение открылось, картинка
    пришла, соединение закрылось. В промежутках к роботу не уходит ничего.

    Неудачный снимок не отменяет остальные: канал может моргнуть на одном
    запросе и быть в порядке на следующем, а для решения достаточно и
    двух кадров из трёх.
    """
    frames = []
    session = make_session()
    for i in range(count):
        if i and gap > 0:
            time.sleep(gap)
        try:
            r = session.get(url, timeout=timeout)
            r.raise_for_status()
            img = cv2.imdecode(np.frombuffer(r.content, np.uint8),
                               cv2.IMREAD_COLOR)
            if img is not None:
                frames.append(img)
        except Exception as exc:
            logger.warn(f'снимок {i + 1}/{count} не получен: {exc}')
    return frames


def read_mjpeg_frames(url, count, timeout, logger):
    """Забрать count кадров из MJPEG-потока и закрыть соединение.

    Кадры в MJPEG идут подряд, каждый обрамлён маркерами JPEG: начало
    FFD8, конец FFD9. Разбираем поток по ним, а не по HTTP-границам —
    boundary у разных источников называется по-разному, а маркеры одни.
    """
    frames = []
    buf = b''
    deadline = time.monotonic() + timeout
    with make_session().get(url, stream=True, timeout=(timeout, timeout)) as r:
        r.raise_for_status()
        for chunk in r.iter_content(chunk_size=8192):
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b'\xff\xd8')
                end = buf.find(b'\xff\xd9', start + 2)
                if start < 0 or end < 0:
                    break
                jpg = buf[start:end + 2]
                buf = buf[end + 2:]
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    frames.append(img)
            if len(frames) >= count:
                return frames
            if time.monotonic() > deadline:
                logger.warn(f'таймаут: успели взять {len(frames)} кадров из {count}')
                return frames
    return frames


def grab_camera_frames(device, count, gap, width, height, logger):
    """Забрать count кадров прямо с камеры, без всякой сети.

    Режим для случая, когда детектор работает на самом роботе: камера
    воткнута сюда же в USB, и гонять кадры через HTTP-сервер на том же
    железе было бы странно. Заодно исчезают и прокси, и таймауты, и
    разбор MJPEG — половина причин, по которым детектор раньше падал.

    Камера открывается на время запроса и тут же освобождается. Держать
    её постоянно нельзя: тот же /dev/video0 может понадобиться
    sign_camera.py, когда картинку хотят посмотреть с ноутбука.
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'камера {device} не открылась: занята или её нет')
    try:
        # MJPG, а не сырой YUYV: по USB 2.0 несжатый кадр 640x480 идёт
        # дольше, чем его успевает отдать матрица, и fps проваливается.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if width:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # Очередь в один кадр. Иначе read() отдаёт то, что накопилось в
        # буфере V4L2, и при паузе между снимками мы получаем прошлое.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Первые кадры идут с неустоявшейся экспозицией и балансом белого:
        # камера только что включилась. Знак на них уходит в засветку.
        for _ in range(5):
            cap.read()

        frames = []
        for i in range(count):
            if i and gap > 0:
                time.sleep(gap)
            ok, img = cap.read()
            if ok and img is not None:
                frames.append(img)
            else:
                logger.warn(f'кадр {i + 1}/{count} с камеры не прочитан')
        return frames
    finally:
        cap.release()


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')

        # snapshot | stream — забрать кадры по HTTP с робота (детектор
        # на ноутбуке); camera — взять их прямо с /dev/video0 (детектор
        # на самом роботе, сеть не участвует вовсе).
        self.declare_parameter('source', 'snapshot')
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('stream_url', 'http://192.168.1.99:8080/stream')
        # Пустая строка — вывести из stream_url заменой /stream на /snapshot,
        # чтобы не задавать два почти одинаковых адреса руками.
        self.declare_parameter('snapshot_url', '')
        # Пауза между снимками: камера отдаёт 5 кадров в секунду, и три
        # запроса подряд вернули бы один и тот же кадр три раза.
        self.declare_parameter('snapshot_gap', 0.25)
        self.declare_parameter('model_path',
                               '/root/ros_ws/src/yolo/dataset_v3_180.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('burst', 3)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('service_name', '/detect_signs')
        self.declare_parameter('annotated_topic', '/sign_image')
        self.declare_parameter('annotated_path',
                               '/root/ros_ws/src/yolo/last_detection.jpg')

        g = self.get_parameter
        self.source = g('source').value
        self.stream_url = g('stream_url').value
        self.snapshot_url = (g('snapshot_url').value
                             or self.stream_url.replace('/stream', '/snapshot'))
        self.snapshot_gap = float(g('snapshot_gap').value)
        self.camera_device = g('camera_device').value
        self.camera_width = int(g('camera_width').value)
        self.camera_height = int(g('camera_height').value)
        self.device = g('device').value
        self.conf = float(g('conf').value)
        self.imgsz = int(g('imgsz').value)
        self.burst = int(g('burst').value)
        self.timeout = float(g('timeout').value)

        self.pub_sign = self.create_publisher(String, '/sign_detected', 10)
        self.pub_raw = self.create_publisher(String, '/sign_detections', 10)
        self.pub_img = self.create_publisher(Image, g('annotated_topic').value, 1)
        self.annotated_path = g('annotated_path').value

        model_path = g('model_path').value
        self.get_logger().info(f'загружаю модель {model_path} на {self.device}')
        self.model = YOLO(model_path)

        # Прогрев. Первый инференс тянет за собой инициализацию CUDA и
        # занимает секунду-полторы — получить эту задержку на первом же
        # знаке посреди полигона было бы обидно.
        self.model(np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8),
                   device=self.device, verbose=False)
        self.get_logger().info(f'модель прогрета, классы: {self.model.names}')

        # Сервис и инференс в реентерабельной группе: запрос блокирующий,
        # без этого узел на время запроса переставал отвечать вообще.
        self.srv = self.create_service(
            Trigger, g('service_name').value, self.on_request,
            callback_group=ReentrantCallbackGroup())
        src = {'camera': self.camera_device,
               'stream': self.stream_url}.get(self.source, self.snapshot_url)
        self.get_logger().info(
            f'готов. сервис {g("service_name").value}, '
            f'источник {self.source}: {src}, кадров за запрос: {self.burst}')

    def on_request(self, _req, resp):
        t0 = time.monotonic()
        try:
            if self.source == 'camera':
                frames = grab_camera_frames(
                    self.camera_device, self.burst, self.snapshot_gap,
                    self.camera_width, self.camera_height, self.get_logger())
            elif self.source == 'stream':
                frames = read_mjpeg_frames(self.stream_url, self.burst,
                                           self.timeout, self.get_logger())
            else:
                frames = fetch_snapshots(self.snapshot_url, self.burst,
                                         self.timeout, self.snapshot_gap,
                                         self.get_logger())
        except Exception as exc:                       # сеть, камера, что угодно
            resp.success = False
            resp.message = f'камера недоступна: {exc}'
            self.get_logger().error(resp.message)
            self._publish('', [])
            return resp

        if not frames:
            resp.success = False
            resp.message = 'кадры не получены'
            self.get_logger().error(resp.message)
            self._publish('', [])
            return resp

        # По каждому классу берём лучшую уверенность и считаем, в скольких
        # кадрах он вообще встретился: одиночное срабатывание на одном
        # кадре из пяти — обычно шум, а не знак.
        best = {}
        # Кадр, на котором детектор был увереннее всего, — его и показываем.
        # Брать первый попавшийся нельзя: именно на нём знака может не быть.
        top_score, annotated = -1.0, frames[0]
        for img in frames:
            r = self.model(img, conf=self.conf, imgsz=self.imgsz,
                           device=self.device, verbose=False)[0]
            if r.boxes is not None and len(r.boxes):
                s_max = max(float(b.conf[0]) for b in r.boxes)
                if s_max > top_score:
                    # plot() возвращает копию кадра с рамками и подписями,
                    # уже в BGR — ровно то, что нужно и файлу, и топику.
                    top_score, annotated = s_max, r.plot()
            seen = set()
            for box in (r.boxes or []):
                cls = str(r.names[int(box.cls[0])])
                score = float(box.conf[0])
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].tolist())
                rec = best.setdefault(cls, {'class_id': cls, 'score': 0.0,
                                            'frames': 0, 'bbox': None})
                if score > rec['score']:
                    rec['score'] = score
                    rec['bbox'] = [x1, y1, x2, y2]
                if cls not in seen:
                    rec['frames'] += 1
                    seen.add(cls)

        detections = sorted(best.values(), key=lambda d: -d['score'])
        for d in detections:
            d['sign'] = SIGN_BY_CLASS.get(d['class_id'])

        # Знаком считаем самый уверенный класс, у которого есть имя:
        # «опасный объект» маршрут не меняет и победителем быть не может.
        sign = next((d['sign'] for d in detections if d['sign']), '')

        dt = time.monotonic() - t0
        self._publish(sign, detections, frames=len(frames), seconds=dt)
        self._publish_image(annotated)

        resp.success = True
        if sign:
            top = next(d for d in detections if d['sign'] == sign)
            resp.message = (f"{sign} (класс {top['class_id']}, "
                            f"уверенность {top['score']:.2f}, "
                            f"в {top['frames']} из {len(frames)} кадров, "
                            f"{dt:.2f} с)")
        else:
            resp.message = (f'знаков не найдено: {len(frames)} кадров, '
                            f'{dt:.2f} с')
        self.get_logger().info(resp.message)
        return resp

    def _publish_image(self, img):
        """Показать, что увидел детектор: файлом и топиком.

        cv_bridge в этом образе нет, но Image собирается вручную — полей
        всего шесть, и лишняя зависимость того не стоит.
        """
        try:
            cv2.imwrite(self.annotated_path, img)
        except Exception as exc:
            self.get_logger().warn(f'не удалось записать {self.annotated_path}: {exc}')

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

    def _publish(self, sign, detections, frames=0, seconds=0.0):
        self.pub_sign.publish(String(data=sign))
        self.pub_raw.publish(String(data=json.dumps({
            'sign': sign,
            'frames': frames,
            'seconds': round(seconds, 3),
            'detections': detections,
        }, ensure_ascii=False)))


def main():
    rclpy.init()
    node = SignDetector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
