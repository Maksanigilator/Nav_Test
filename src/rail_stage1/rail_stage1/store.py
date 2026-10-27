"""Persistent, map-scoped rail entries. No ROS dependency."""
import json
import math
import os
from pathlib import Path
import tempfile
import numpy as np


def angle_distance(a, b):
    return abs((a-b+math.pi) % (2*math.pi)-math.pi)


class RailStore:
    def __init__(self, path, map_id, cfg):
        self.path = Path(path).expanduser()
        self.map_id = map_id
        self.cfg = cfg
        self.rails = {}
        self.pending = []
        if self.path.exists():
            data = json.loads(self.path.read_text())
            if data['frame'] != 'map' or data['map_id'] != map_id:
                raise ValueError('rail database belongs to a different map')
            self.rails = {row['id']: row for row in data['rails']}

    def save(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = dict(version=1, frame='map', map_id=self.map_id, rails=list(self.rails.values()))
        fd, name = tempfile.mkstemp(prefix='.rails-', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f, indent=2, allow_nan=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(name, self.path)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    def observe(self, point, yaw, stamp_ns, now):
        self.pending = [p for p in self.pending if now-p['last_time'] < self.cfg['confirmation_timeout']]
        matched = next((p for p in self.pending
                        if np.linalg.norm(np.array(p['point'])[:2]-point[:2]) < self.cfg['confirmation_position']
                        and angle_distance(p['yaw'], yaw) < math.radians(self.cfg['confirmation_angle_deg'])), None)
        if matched is None:
            matched = dict(point=np.array(point), yaw=yaw, count=0, last_stamp=-1, last_time=now)
            self.pending.append(matched)
        if stamp_ns <= matched['last_stamp']:
            return None
        count = matched['count']
        matched['point'] = (matched['point']*count+point)/(count+1)
        matched['yaw'] += ((yaw-matched['yaw']+math.pi) % (2*math.pi)-math.pi)/(count+1)
        matched.update(count=count+1, last_stamp=stamp_ns, last_time=now)
        if matched['count'] < self.cfg['confirmation_frames']:
            return None
        point, yaw = matched['point'], matched['yaw']
        self.pending = [p for p in self.pending if p is not matched]
        nearby = [rail for rail in self.rails.values()
                  if np.linalg.norm(np.array(rail['point'])[:2]-point[:2]) < self.cfg['match_distance']]
        rail = next((rail for rail in nearby
                     if angle_distance(rail['yaw'], yaw) < math.radians(self.cfg['match_angle_deg'])), None)
        if rail is None and nearby:
            # Do not count a flipped or conflicting estimate as another rail.
            return None
        if rail is None:
            rail_id = f"rail_{max([int(k.split('_')[-1]) for k in self.rails] or [0])+1:03d}"
            rail = dict(id=rail_id, point=point.tolist(), yaw=float(yaw), confirmations=0)
        else:
            # Update only using a fully confirmed batch of independent frames.
            weight = min(rail['confirmations'], 10)
            rail['point'] = ((np.array(rail['point'])*weight+point)/(weight+1)).tolist()
            rail['yaw'] += ((yaw-rail['yaw']+math.pi) % (2*math.pi)-math.pi)/(weight+1)
        rail['confirmations'] += 1
        rail['last_observation_ns'] = int(stamp_ns)
        self.rails[rail['id']] = rail
        self.save()
        return rail


def approach_pose(rail, distance):
    yaw = rail['yaw']
    point = np.array(rail['point'], dtype=float)
    point[:2] -= distance*np.array([math.cos(yaw), math.sin(yaw)])
    return point, yaw
