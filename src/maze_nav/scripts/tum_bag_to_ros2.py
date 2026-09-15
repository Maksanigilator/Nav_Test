#!/usr/bin/env python3
"""Конвертер ROS 1 bag датасета TUM RGB-D в ROS 2 bag, пригодный для RTAB-Map.

Заменяет цепочку из шапки rtabmap_examples/launch/rgbdslam_datasets.launch.py,
которой нужен установленный ROS 1: rosbag decompress, roscore и два ROS1-скрипта
(tum_rename_world_kinect_frame.py и change_frame_id.py). ROS 1 у нас нет и
ставить его незачем.

Работа идёт в два прохода, и это важно:

  1. rosbags.convert — честный перевод ROS 1 -> ROS 2. Своими руками это делать
     нельзя: у сообщений реально разные поля. Например, в ROS 1 у CameraInfo
     матрицы назывались D/K/R/P, а в ROS 2 — d/k/r/p; у Header исчез seq;
     тип tf/tfMessage заменён на tf2_msgs/TFMessage. Библиотека знает все такие
     различия, поэтому маппинг типов отдан ей.

  2. Правка фреймов уже внутри ROS 2 — здесь читаем и пишем одним типовым
     хранилищем, так что никаких сюрпризов с полями. Делается ровно то, ради
     чего в оригинале нужны были ROS1-скрипты:

     * убирается ведущий "/" из frame_id и child_frame_id. В ROS 1 фреймы
       писались как "/world", ROS 2 такие имена отвергает;

     * потомки фрейма "world" в /tf переименовываются в "<имя>_gt". Без этого
       ground truth публикует world->kinect, визуальная одометрия — odom->kinect,
       и два родителя у одного фрейма ломают дерево TF. После переименования
       ground truth живёт отдельной веткой world->kinect_gt, и RTAB-Map может
       считать по ней ошибку траектории (rtabmap-report).

Пример:

    ./tum_bag_to_ros2.py /datasets/rgbd_dataset_freiburg3_long_office_household.bag

Результат — каталог с тем же именем без .bag, рядом с исходником.
"""
import argparse
import pathlib
import shutil
import sys

from rosbags.convert import convert
from rosbags.rosbag2 import Reader as Rosbag2Reader
from rosbags.rosbag2 import Writer as Rosbag2Writer
from rosbags.typesys import Stores, get_typestore

GROUND_TRUTH_PARENT = 'world'
GROUND_TRUTH_SUFFIX = '_gt'
TF_TYPE = 'tf2_msgs/msg/TFMessage'

# Маркеры системы захвата движения: RTAB-Map их не читает, а весят они треть бэга.
SKIP_TOPICS_DEFAULT = ('/cortex_marker_array',)


def strip_slash(name: str) -> str:
    """"/world" -> "world". В ROS 2 ведущий слэш во фрейме недопустим."""
    return name[1:] if name.startswith('/') else name


def fix_tf(msg) -> int:
    """Чинит имена фреймов и уводит ground truth в отдельную ветку."""
    renamed = 0
    for tr in msg.transforms:
        tr.header.frame_id = strip_slash(tr.header.frame_id)
        tr.child_frame_id = strip_slash(tr.child_frame_id)
        if (tr.header.frame_id == GROUND_TRUTH_PARENT
                and not tr.child_frame_id.endswith(GROUND_TRUTH_SUFFIX)):
            tr.child_frame_id += GROUND_TRUTH_SUFFIX
            renamed += 1
    return renamed


def fix_frames(src: pathlib.Path, dst: pathlib.Path, typestore) -> None:
    """Второй проход: ROS 2 -> ROS 2 с починенными фреймами."""
    total = renamed = 0
    with Rosbag2Reader(src) as reader, \
         Rosbag2Writer(dst, version=Rosbag2Writer.VERSION_LATEST) as writer:
        conns = {
            conn.id: writer.add_connection(conn.topic, conn.msgtype,
                                           typestore=typestore)
            for conn in reader.connections
        }
        for conn, timestamp, raw in reader.messages():
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            if conn.msgtype == TF_TYPE:
                renamed += fix_tf(msg)
            elif hasattr(msg, 'header'):
                msg.header.frame_id = strip_slash(msg.header.frame_id)
            writer.write(conns[conn.id], timestamp,
                         typestore.serialize_cdr(msg, conn.msgtype))
            total += 1

    print(f'  сообщений записано: {total}')
    print(f'  ground-truth трансформов уведено в *{GROUND_TRUTH_SUFFIX}: {renamed}')
    if renamed == 0:
        print(f'  ВНИМАНИЕ: не найдено ни одного трансформа от "{GROUND_TRUTH_PARENT}" — '
              'rtabmap-report не сможет посчитать ошибку траектории', file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('src', type=pathlib.Path, help='исходный ROS 1 .bag')
    ap.add_argument('-o', '--dst', type=pathlib.Path, default=None,
                    help='каталог для ROS 2 bag (по умолчанию — имя src без .bag)')
    ap.add_argument('-f', '--force', action='store_true',
                    help='перезаписать каталог назначения, если он уже есть')
    ap.add_argument('--keep-all-topics', action='store_true',
                    help=f'не выбрасывать {", ".join(SKIP_TOPICS_DEFAULT)}')
    args = ap.parse_args()

    dst = args.dst or args.src.with_suffix('')
    tmp = dst.with_name(dst.name + '.raw')

    for path in (dst, tmp):
        if path.exists():
            if not args.force:
                print(f'{path} уже существует — запусти с --force или укажи другой -o',
                      file=sys.stderr)
                return 1
            shutil.rmtree(path)

    typestore = get_typestore(Stores.ROS2_JAZZY)
    skip = () if args.keep_all_topics else SKIP_TOPICS_DEFAULT

    print(f'[1/2] ROS 1 -> ROS 2 ({args.src.name})')
    if skip:
        print(f'  выбрасываем топики: {", ".join(skip)}')
    convert(
        srcs=[args.src],
        dst=tmp,
        dst_storage='sqlite3',
        dst_version=Rosbag2Writer.VERSION_LATEST,
        compress=None,
        compress_mode='none',
        default_typestore=get_typestore(Stores.ROS1_NOETIC),
        typestore=typestore,
        exclude_topics=skip,
        include_topics=[],
        exclude_msgtypes=[],
        include_msgtypes=[],
    )

    print('[2/2] правка фреймов')
    fix_frames(tmp, dst, typestore)
    shutil.rmtree(tmp)

    print(f'готово: {dst}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
