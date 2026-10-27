import json, time, os
from pathlib import Path
from datetime import datetime, timezone
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

def stamp(m):
    return m.header.stamp.sec * 1000000000 + m.header.stamp.nanosec

def array(m):
    dtype = np.uint16 if m.encoding == '16UC1' else np.float32 if m.encoding == '32FC1' else np.uint8
    a = np.frombuffer(m.data, dtype).reshape(m.height, m.step // np.dtype(dtype).itemsize)
    if m.encoding in ('rgb8', 'bgr8'):
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return a[:, :, ::-1].copy() if m.encoding == 'rgb8' else a.copy()
    return a[:, :m.width].copy()

rclpy.init()
n = Node('rail_snapshot_capture')
frames, masks, depths, infos = {}, {}, {}, {}
subs = []
qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
def receive(store, m):
    store[stamp(m)] = m
    while len(store) > 90:
        del store[next(iter(store))]
for topic, store in [('/camera/color/image_raw', frames), ('/rails/mask', masks), ('/camera/aligned_depth_to_color/image_raw', depths)]:
    subs.append(n.create_subscription(Image, topic, lambda m, s=store: receive(s, m), qos))
subs.append(n.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: infos.update(color=m), qos))
result = None
t0 = time.monotonic()
try:
    while time.monotonic() - t0 < 25:
        rclpy.spin_once(n, timeout_sec=0.1)
        for ts in sorted(frames.keys() & masks.keys(), reverse=True):
            mask = array(masks[ts])
            if not np.any(mask) or not depths or not infos:
                continue
            ds = min(depths, key=lambda x: abs(x-ts))
            if abs(ds-ts) > 80000000:
                continue
            rgb = array(frames[ts])
            if rgb.shape[:2] != mask.shape:
                continue
            out = Path(os.environ.get('SNAPSHOT_ROOT', '/tmp/aida_nav_vlad_snapshots')) / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
            out.mkdir(parents=True, exist_ok=False)
            paint = rgb.copy()
            paint[mask > 0] = (0, 255, 0)
            overlay = cv2.addWeighted(rgb, 0.55, paint, 0.45, 0)
            for name, a in [('rgb.png', rgb), ('mask.png', mask), ('overlay.png', overlay)]:
                if not cv2.imwrite(str(out/name), a):
                    raise RuntimeError('Failed to save '+name)
            depth = array(depths[ds])
            np.save(out/'depth.npy', depth)
            if depths[ds].encoding == '16UC1':
                cv2.imwrite(str(out/'depth_mm.png'), depth)
            ci = infos['color']
            meta = dict(rgb_stamp_ns=ts, mask_stamp_ns=stamp(masks[ts]), depth_stamp_ns=ds,
                        depth_delta_ms=(ds-ts)/1e6, frame_id=frames[ts].header.frame_id,
                        width=frames[ts].width, height=frames[ts].height,
                        mask_nonzero=int(np.count_nonzero(mask)), depth_encoding=depths[ds].encoding,
                        camera_info=dict(width=ci.width, height=ci.height, distortion_model=ci.distortion_model,
                                         k=list(ci.k), d=list(ci.d), r=list(ci.r), p=list(ci.p)),
                        topics=dict(rgb='/camera/color/image_raw', mask='/rails/mask', depth='/camera/aligned_depth_to_color/image_raw'))
            (out/'metadata.json').write_text(json.dumps(meta, ensure_ascii=False, indent=2))
            result = dict(path=str(out), **meta)
            break
        if result:
            break
    if not result:
        raise RuntimeError(f'No matched nonempty snapshot: rgb={len(frames)} masks={len(masks)} depth={len(depths)} info={len(infos)}')
    print(json.dumps(result, ensure_ascii=False))
finally:
    n.destroy_node()
    rclpy.shutdown()
