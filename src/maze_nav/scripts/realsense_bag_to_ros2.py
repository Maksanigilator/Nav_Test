#!/usr/bin/env python3
"""Конвертер записи RealSense (.bag от librealsense) в ROS 2 bag.

Зачем это нужно, хотя есть штатный путь. Узел `realsense2_camera` умеет открывать
такой файл как виртуальное устройство (`rosbag_filename`), и для живого просмотра
этого достаточно. Но с включённым выравниванием глубины он выдаёт ~1 Гц вместо 30
(замерено на D435: color 1.2 Гц, aligned depth 0.9 Гц против 28 Гц без выравнивания).
Проигрывание при этом идёт в реальном времени, поэтому узел не тормозит запись,
а **теряет кадры**: между обработанными набегает по два метра движения, и визуальная
одометрия перестаёт сходиться — `Not enough inliers 0/20` при живых сопоставлениях.

Само выравнивание тут ни при чём: через pyrealsense2 те же кадры обрабатываются
со скоростью ~290 кадр/с. Дело в режиме воспроизведения. В librealsense есть
`set_real_time(False)` — проигрывание ждёт потребителя и не роняет ни одного кадра,
но наружу из ROS-обёртки этот режим не выставлен.

Поэтому: конвертируем один раз офлайн, с уже выровненной глубиной, и дальше
работаем обычным `ros2 bag play` на любой скорости.

Выравнивание обязательно: у D435 depth 848x480, color 640x480, у них разные
интринсики и физически разнесённые объективы, а RTAB-Map считает, что пиксель
(u,v) в обоих кадрах — одна и та же точка сцены.

    ./realsense_bag_to_ros2.py /data/20260613_142750.bag -o /datasets/rail_run

Дальше:

    ros2 launch maze_nav rtabmap_realsense_bag.launch.py \
        bag:=/datasets/rail_run play:=true
"""
import argparse
import pathlib
import shutil
import sys

import numpy as np
import pyrealsense2 as rs
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

# Кадр публикуемых изображений. Оптическая конвенция (Z вперёд, X вправо):
# именно её ждёт RTAB-Map от камеры. Связь с base_link публикует launch.
OPTICAL_FRAME = 'camera_color_optical_frame'

RGB_TOPIC = '/camera/color/image_raw'
INFO_TOPIC = '/camera/color/camera_info'
# Глубина выровнена в кадр цвета, поэтому делит с ним и интринсики, и frame_id.
DEPTH_TOPIC = '/camera/aligned_depth_to_color/image_raw'


def make_header(ts, stamp_ns, frame_id):
    Header = ts.types['std_msgs/msg/Header']
    Time = ts.types['builtin_interfaces/msg/Time']
    return Header(stamp=Time(sec=int(stamp_ns // 10**9),
                             nanosec=int(stamp_ns % 10**9)),
                  frame_id=frame_id)


def make_camera_info(ts, intr, stamp_ns):
    CameraInfo = ts.types['sensor_msgs/msg/CameraInfo']
    RegionOfInterest = ts.types['sensor_msgs/msg/RegionOfInterest']
    k = np.array([intr.fx, 0.0, intr.ppx,
                  0.0, intr.fy, intr.ppy,
                  0.0, 0.0, 1.0], dtype=np.float64)
    p = np.array([intr.fx, 0.0, intr.ppx, 0.0,
                  0.0, intr.fy, intr.ppy, 0.0,
                  0.0, 0.0, 1.0, 0.0], dtype=np.float64)
    return CameraInfo(
        header=make_header(ts, stamp_ns, OPTICAL_FRAME),
        height=intr.height, width=intr.width,
        distortion_model='plumb_bob',
        # Кадры из RealSense уже ректифицированы драйвером, поэтому
        # коэффициенты дисторсии нулевые — иначе их применили бы дважды.
        d=np.zeros(5, dtype=np.float64),
        k=k, r=np.eye(3, dtype=np.float64).ravel(), p=p,
        binning_x=0, binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0,
                             do_rectify=False))


def make_image(ts, stamp_ns, arr, encoding):
    Image = ts.types['sensor_msgs/msg/Image']
    h, w = arr.shape[:2]
    return Image(header=make_header(ts, stamp_ns, OPTICAL_FRAME),
                 height=h, width=w, encoding=encoding,
                 is_bigendian=0, step=arr.strides[0],
                 data=arr.reshape(-1).view(np.uint8))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', type=pathlib.Path, help='запись .bag от librealsense')
    ap.add_argument('-o', '--dst', type=pathlib.Path, default=None,
                    help='каталог ROS 2 bag (по умолчанию — имя src без .bag)')
    ap.add_argument('-f', '--force', action='store_true',
                    help='перезаписать каталог назначения')
    ap.add_argument('--limit', type=int, default=0,
                    help='обработать только N первых кадров (для быстрой проверки)')
    args = ap.parse_args()

    dst = args.dst or args.src.with_suffix('')
    if dst.exists():
        if not args.force:
            print(f'{dst} уже существует — запусти с --force', file=sys.stderr)
            return 1
        shutil.rmtree(dst)

    cfg = rs.config()
    cfg.enable_device_from_file(str(args.src), repeat_playback=False)
    pipe = rs.pipeline()
    profile = pipe.start(cfg)

    # Ключевой момент: без этого проигрывание идёт в реальном времени и роняет
    # кадры, когда потребитель не успевает. Нам нужны все кадры, а не темп.
    profile.get_device().as_playback().set_real_time(False)

    align = rs.align(rs.stream.color)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
    print(f'масштаб глубины: {depth_scale} м на единицу')

    typestore = get_typestore(Stores.ROS2_JAZZY)
    n = 0
    t_first = None

    with Writer(dst, version=Writer.VERSION_LATEST) as writer:
        c_rgb = writer.add_connection(RGB_TOPIC, 'sensor_msgs/msg/Image',
                                      typestore=typestore)
        c_depth = writer.add_connection(DEPTH_TOPIC, 'sensor_msgs/msg/Image',
                                        typestore=typestore)
        c_info = writer.add_connection(INFO_TOPIC, 'sensor_msgs/msg/CameraInfo',
                                       typestore=typestore)
        while True:
            ok, frames = pipe.try_wait_for_frames(3000)
            if not ok:
                break
            frames = align.process(frames)
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue

            # get_timestamp() отдаёт миллисекунды. Приводим к наносекундам и
            # сдвигаем так, чтобы запись начиналась не с нуля: нулевое время
            # в tf2 означает «последний доступный», и на нём всё ломается.
            stamp_ns = int(color.get_timestamp() * 1e6)
            if t_first is None:
                t_first = stamp_ns
                intr = color.profile.as_video_stream_profile().intrinsics
                print(f'цвет {intr.width}x{intr.height} '
                      f'fx={intr.fx:.1f} fy={intr.fy:.1f} '
                      f'cx={intr.ppx:.1f} cy={intr.ppy:.1f}')
            stamp_ns = 10**18 + (stamp_ns - t_first)

            rgb = np.asanyarray(color.get_data())
            # RealSense отдаёт глубину в единицах depth_scale, ROS ждёт 16UC1
            # в миллиметрах. Для 0.001 это тождество, но у других профилей нет.
            d_mm = (np.asanyarray(depth.get_data()).astype(np.float64)
                    * depth_scale * 1000.0)
            d_mm = np.clip(d_mm, 0, 65535).astype(np.uint16)

            writer.write(c_rgb, stamp_ns,
                         typestore.serialize_cdr(
                             make_image(typestore, stamp_ns, rgb, 'rgb8'),
                             'sensor_msgs/msg/Image'))
            writer.write(c_depth, stamp_ns,
                         typestore.serialize_cdr(
                             make_image(typestore, stamp_ns, d_mm, '16UC1'),
                             'sensor_msgs/msg/Image'))
            writer.write(c_info, stamp_ns,
                         typestore.serialize_cdr(
                             make_camera_info(typestore, intr, stamp_ns),
                             'sensor_msgs/msg/CameraInfo'))
            n += 1
            if n % 200 == 0:
                print(f'  ...{n} кадров')
            if args.limit and n >= args.limit:
                break

    pipe.stop()
    print(f'готово: {n} кадров -> {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
