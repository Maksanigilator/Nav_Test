#!/usr/bin/env python3
"""
YOLO-узел: читает MJPEG-поток (его отдаёт ffmpeg на Raspberry Pi) и публикует
yolo/FrameDetections — детекции вместе с позой робота в момент получения кадра.

Один кадр -> одно сообщение:
  * header.stamp  — ROS-время ПОЛУЧЕНИЯ кадра (не момент старта инференса);
  * detections[]  — рамки и классы (vision_msgs/Detection2D), пиксели кадра;
  * robot_pose    — поза робота ровно на это время;
  * pose_valid    — false, если позу получить не удалось.

Источник позы выбирается параметром pose_source:
  * 'tf'    (по умолчанию) — lookup_transform(pose_parent_frame -> pose_child_frame)
              на время кадра, с откатом на последнюю доступную трансформацию;
  * 'topic' — последнее сообщение из pose_topic, тип задаёт pose_topic_type:
              'odometry' = nav_msgs/Odometry, 'pose' = geometry_msgs/PoseWithCovarianceStamped.
              Сообщения старше pose_max_age секунд считаются устаревшими.

Запуск:
    ros2 run yolo yolo_client --ros-args \
        -p stream_url:=http://10.10.10.5:8080/stream \
        -p pose_parent_frame:=map -p pose_child_frame:=base_footprint

Топики:
    /yolo/frame_detections  (yolo/FrameDetections) — основной выход;
    /yolo/detection_image   (sensor_msgs/Image)    — только при publish_annotated:=true,
                                                     тяжёлый, по Wi-Fi не гонять.
"""

import os
import threading
import time
from copy import deepcopy

import cv2
import numpy as np
import requests

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import tf2_ros

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
from ultralytics import YOLO

from yolo.msg import FrameDetections


def default_model_path():
    """dataset_v3_180.pt из share/yolo.

    Через share, а не через __file__: скрипт ставится в lib/yolo, а модель
    лежит в share/yolo. Если пакет ещё не собран — падаем обратно на папку
    скрипта, чтобы узел можно было запустить и напрямую из исходников.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('yolo'), 'dataset_v3_180.pt')
    except Exception:
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'dataset_v3_180.pt')


# ============================================================
# Чтение MJPEG
# ============================================================
class MJPEGReader:
    """Читает MJPEG в фоновом потоке и всегда отдаёт последний кадр.

    Вместе с кадром хранит монотонную метку времени — по ней узел потом
    считает ROS-время получения кадра.
    """

    def __init__(self, url, logger):
        self.url = url
        self.logger = logger
        self.connected = False
        self._frame = None
        self._stamp_mono = 0.0
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        buf = b''
        while self._running:
            try:
                self.logger.info(f'Подключаюсь к {self.url} ...')
                response = requests.get(self.url, stream=True, timeout=10)
                self.connected = True
                self.logger.info('Поток открыт')
                buf = b''
                for chunk in response.iter_content(chunk_size=16384):
                    if not self._running:
                        return
                    if not chunk:
                        continue
                    buf += chunk
                    while True:
                        # Кадр MJPEG — это JPEG между маркерами SOI и EOI.
                        start = buf.find(b'\xff\xd8')
                        if start == -1:
                            buf = b''
                            break
                        end = buf.find(b'\xff\xd9', start + 2)
                        if end == -1:
                            buf = buf[start:]
                            break
                        jpg = buf[start:end + 2]
                        buf = buf[end + 2:]
                        img = cv2.imdecode(
                            np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                        if img is not None:
                            with self._lock:
                                self._frame = img
                                self._stamp_mono = time.monotonic()
            except Exception as exc:
                self.connected = False
                self.logger.warning(
                    f'Ошибка потока: {exc}. Переподключение через 2 с')
                time.sleep(2.0)

    def read(self):
        """Возвращает (кадр BGR | None, монотонное время получения)."""
        with self._lock:
            return self._frame, self._stamp_mono

    def stop(self):
        self._running = False


# ============================================================
# Поза робота
# ============================================================
class PoseProvider:
    """Отдаёт PoseWithCovarianceStamped на заданный момент — из TF или топика."""

    def __init__(self, node, mode, parent_frame, child_frame,
                 topic, topic_type, max_age):
        self.logger = node.get_logger()
        self.mode = mode
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.max_age = max_age
        self._lock = threading.Lock()
        self._last = None
        self._last_mono = 0.0
        self._buffer = None
        self._listener = None
        self._sub = None

        if mode == 'tf':
            self._buffer = tf2_ros.Buffer()
            self._listener = tf2_ros.TransformListener(self._buffer, node)
            self.logger.info(f'Поза: TF {parent_frame} -> {child_frame}')
        elif mode == 'topic':
            cls = Odometry if topic_type == 'odometry' else PoseWithCovarianceStamped
            self._sub = node.create_subscription(cls, topic, self._on_msg, 10)
            self.logger.info(f'Поза: топик {topic} ({cls.__name__})')
        else:
            raise ValueError(
                f"pose_source должен быть 'tf' или 'topic', а не {mode!r}")

    def _on_msg(self, msg):
        pose = PoseWithCovarianceStamped()
        if isinstance(msg, Odometry):
            pose.header = msg.header
            pose.pose = msg.pose
        else:
            pose = deepcopy(msg)
        with self._lock:
            self._last = pose
            self._last_mono = time.monotonic()

    def get(self, frame_time, frame_mono):
        """Поза на момент кадра (rclpy Time) или None, если её нет."""
        if self.mode == 'topic':
            with self._lock:
                if self._last is None:
                    return None
                if frame_mono - self._last_mono > self.max_age:
                    return None
                pose = deepcopy(self._last)
            pose.header.stamp = frame_time.to_msg()
            return pose

        # TF. Сначала честно на время кадра; если трансформация ещё не
        # догнала это время — берём последнюю доступную.
        buffer = self._buffer
        if buffer is None:
            return None
        try:
            tr = buffer.lookup_transform(
                self.parent_frame, self.child_frame, frame_time)
        except Exception:
            try:
                tr = buffer.lookup_transform(
                    self.parent_frame, self.child_frame, Time())
            except Exception as exc:
                self.logger.warning(
                    f'Поза недоступна ({self.parent_frame} -> '
                    f'{self.child_frame}): {exc}',
                    throttle_duration_sec=5.0)
                return None

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = frame_time.to_msg()
        pose.header.frame_id = self.parent_frame
        pose.pose.pose.position.x = tr.transform.translation.x
        pose.pose.pose.position.y = tr.transform.translation.y
        pose.pose.pose.position.z = tr.transform.translation.z
        pose.pose.pose.orientation = tr.transform.rotation
        return pose


# ============================================================
# ROS-узел
# ============================================================
class YoloClientNode(Node):

    def __init__(self):
        super().__init__('yolo_client')

        # --- Параметры ---
        self.declare_parameter('stream_url', 'http://10.10.10.5:8080/stream')
        self.declare_parameter('model_path', default_model_path())
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('output_topic', '/yolo/frame_detections')
        self.declare_parameter('publish_annotated', False)

        self.declare_parameter('pose_source', 'tf')
        self.declare_parameter('pose_parent_frame', 'map')
        self.declare_parameter('pose_child_frame', 'base_footprint')
        self.declare_parameter('pose_topic', '/odometry/filtered')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('pose_max_age', 0.5)

        self.stream_url = self.get_parameter('stream_url').value
        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        output_topic = self.get_parameter('output_topic').value

        self.get_logger().info(f'Загружаю модель: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'Модель готова, device={self.device}')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Публикации ---
        self.pub_det = self.create_publisher(FrameDetections, output_topic, 10)
        self.pub_img = None
        if self.publish_annotated:
            self.pub_img = self.create_publisher(
                Image, '/yolo/detection_image', qos_sensor)

        # --- Поза ---
        self.pose_provider = PoseProvider(
            self,
            self.get_parameter('pose_source').value,
            self.get_parameter('pose_parent_frame').value,
            self.get_parameter('pose_child_frame').value,
            self.get_parameter('pose_topic').value,
            self.get_parameter('pose_topic_type').value,
            self.get_parameter('pose_max_age').value,
        )

        # --- Поток + инференс ---
        self.reader = MJPEGReader(self.stream_url, self.get_logger())
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        # --- Статистика ---
        self.fps_t0 = time.time()
        self.fps_count = 0
        self.pose_ok = 0
        self.create_timer(5.0, self._report_stats)

        self.get_logger().info(f'Готово. Публикую {output_topic}')

    def _loop(self):
        while self.running and rclpy.ok():
            frame, frame_mono = self.reader.read()
            if frame is None:
                time.sleep(0.01)
                continue

            # Время кадра = часы ROS минус возраст кадра по монотонным часам.
            # Так stamp — момент ПОЛУЧЕНИЯ кадра, а не старта инференса.
            now = self.get_clock().now()
            frame_time = now - Duration(seconds=time.monotonic() - frame_mono)

            # Позу берём до инференса: она нужна на время кадра, а не на
            # время, когда YOLO закончит считать.
            pose = self.pose_provider.get(frame_time, frame_mono)

            results = self.model(
                frame,
                conf=self.conf,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )[0]

            msg = FrameDetections()
            msg.header.stamp = frame_time.to_msg()
            msg.header.frame_id = self.camera_frame

            for box in (results.boxes or []):
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = (x1 + x2) / 2.0
                det.bbox.center.position.y = (y1 + y2) / 2.0
                det.bbox.center.theta = 0.0
                det.bbox.size_x = x2 - x1
                det.bbox.size_y = y2 - y1

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(results.names[int(box.cls[0])])
                hyp.hypothesis.score = float(box.conf[0])
                det.results.append(hyp)

                msg.detections.append(det)

            msg.pose_valid = pose is not None
            if pose is not None:
                msg.robot_pose = pose

            self.pub_det.publish(msg)

            # --- Картинка с рамками (опционально) ---
            if self.pub_img is not None:
                annotated = np.ascontiguousarray(results.plot())
                height, width = annotated.shape[:2]
                img = Image()
                img.header = msg.header
                img.height = int(height)
                img.width = int(width)
                img.encoding = 'bgr8'
                img.is_bigendian = 0
                img.step = int(width) * 3
                img.data = annotated.tobytes()
                self.pub_img.publish(img)

            self.fps_count += 1
            if pose is not None:
                self.pose_ok += 1

    def _report_stats(self):
        dt = time.time() - self.fps_t0
        if dt > 0 and self.fps_count > 0:
            self.get_logger().info(
                f'FPS: {self.fps_count / dt:.1f} | '
                f'поток: {"ок" if self.reader.connected else "нет"} | '
                f'поза: {100.0 * self.pose_ok / self.fps_count:.0f}%')
        self.fps_t0 = time.time()
        self.fps_count = 0
        self.pose_ok = 0

    def destroy_node(self):
        self.running = False
        self.reader.stop()
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
