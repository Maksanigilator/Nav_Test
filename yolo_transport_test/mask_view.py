#!/usr/bin/env python3
"""Окно просмотра сегментации: кадр в полный размер, маска поверх него.

ЗАЧЕМ ОТДЕЛЬНОЕ ОКНО, А НЕ RViz. В RViz центральная область — это всегда
трёхмерная сцена, а Image-дисплеи бывают только панелями по краям и
получаются мелкими. Здесь кадр занимает всё окно, и маска лежит прямо на
нём, так что видно не «есть маска или нет», а ПО ЧЕМУ она легла: по
трубам или по стене. Для оценки сегментации это и есть главный вопрос.

ОБА ИЗОБРАЖЕНИЯ БЕРУТСЯ ИЗ ТОПИКОВ, а не рисуются из памяти. Локальный
предпросмотр выглядел бы правильно даже при полностью сломанной передаче:
ни сериализация, ни транспорт, ни совместимость QoS им не проверяются.

Маска сопоставляется с кадром ПО МЕТКЕ ВРЕМЕНИ, а не берётся последняя.
При сегментации в 2 Гц и подвижном роботе последняя маска отстаёт от
кадра до полусекунды, и наложение показывало бы несуществующее
расхождение — трубы «уехали» бы относительно зелёного просто из-за
задержки.

Клавиши: пробел — показать только маску или вернуть наложение,
    s — сохранить снимок, q или Esc — выход.
"""
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

WIN = 'segmentation'


def to_bgr(m):
    """Image -> BGR с учётом шага строки: он не обязан равняться ширине."""
    if m.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a[:, :, ::-1] if m.encoding == 'rgb8' else a
    if m.encoding == 'mono8':
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)
        return a[:, :m.width]
    return None


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class View(Node):
    def __init__(self):
        super().__init__('mask_view')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('mask_topic', '/rails/mask')
        # Предельное расхождение кадра и маски. Полсекунды — период
        # сегментации на 2 Гц: соседняя маска ещё годится, пропуск уже нет.
        p('max_age', 0.5)
        p('scale', 1.6)
        p('save_dir', '/tmp')

        sub = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.frames = []
        self.mask = None
        self.mask_ns = 0
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self.on_image, sub)
        self.create_subscription(
            Image, str(self.get_parameter('mask_topic').value),
            self.on_mask, sub)
        self.only_mask = False
        self.shown = None
        self.t0 = time.time()
        self.n_img = self.n_mask = 0
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        self.create_timer(0.05, self.draw)
        self.create_timer(5.0, self.report)

    def on_image(self, m):
        self.n_img += 1
        self.frames.append((ns(m.header.stamp), m))
        del self.frames[:-30]

    def on_mask(self, m):
        self.n_mask += 1
        self.mask, self.mask_ns = m, ns(m.header.stamp)

    def draw(self):
        if not self.frames:
            return
        # Кадр подбираем ПОД МАСКУ, а не наоборот: иначе на 2 Гц зелёное
        # отставало бы от картинки и выглядело бы промахом сегментации.
        if self.mask is not None:
            fns, img_msg = min(self.frames, key=lambda f: abs(f[0] - self.mask_ns))
            stale = abs(fns - self.mask_ns) * 1e-9 > float(
                self.get_parameter('max_age').value)
        else:
            img_msg, stale = self.frames[-1][1], True

        img = to_bgr(img_msg)
        if img is None or img.ndim != 3:
            return
        out = img.copy()
        note = 'маски нет'
        if self.mask is not None and not stale:
            mk = to_bgr(self.mask)
            if mk is not None and mk.shape[:2] == img.shape[:2]:
                area = int(np.count_nonzero(mk))
                if self.only_mask:
                    out = cv2.cvtColor(mk, cv2.COLOR_GRAY2BGR)
                else:
                    paint = out.copy()
                    paint[mk > 0] = (0, 255, 0)
                    out = cv2.addWeighted(out, 0.55, paint, 0.45, 0)
                note = (f'маска {area} px ({100.0 * area / mk.size:.1f}%)'
                        if area else 'маска пустая — рельсов не нашлось')
        elif self.mask is not None:
            note = 'маска устарела'

        k = float(self.get_parameter('scale').value)
        if k != 1.0:
            out = cv2.resize(out, None, fx=k, fy=k,
                             interpolation=cv2.INTER_NEAREST)
        cv2.putText(out, note, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out, note, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (80, 255, 80), 1, cv2.LINE_AA)
        self.shown = out
        cv2.imshow(WIN, out)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            raise KeyboardInterrupt
        if key == ord(' '):
            self.only_mask = not self.only_mask
        if key == ord('s') and self.shown is not None:
            path = os.path.join(str(self.get_parameter('save_dir').value),
                                f'mask_{int(time.time())}.png')
            cv2.imwrite(path, self.shown)
            self.get_logger().info(f'снимок сохранён: {path}')

    def report(self):
        dt = max(1e-6, time.time() - self.t0)
        self.get_logger().info(
            f'кадров {self.n_img / dt:.1f} Гц, масок {self.n_mask / dt:.1f} Гц')
        self.t0, self.n_img, self.n_mask = time.time(), 0, 0


def main():
    rclpy.init()
    n = View()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        n.destroy_node()


if __name__ == '__main__':
    main()
