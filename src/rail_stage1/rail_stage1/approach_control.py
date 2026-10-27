"""Coarse pre-entry pose confirmation, independent of Nav2 global settings."""
import math


class ApproachGate:
    def __init__(self, xy_tolerance, yaw_tolerance_deg, dwell_sec):
        if not all(math.isfinite(v) and v > 0 for v in
                   (xy_tolerance, yaw_tolerance_deg, dwell_sec)):
            raise ValueError('approach tolerances and dwell must be finite and positive')
        self.xy = xy_tolerance
        self.yaw = math.radians(yaw_tolerance_deg)
        self.dwell = dwell_sec
        self.reset()

    def reset(self):
        self.since = None
        self.last_stamp = -1
        self.count = 0

    def contains(self, distance, yaw_error):
        return (math.isfinite(distance) and math.isfinite(yaw_error) and
                0 <= distance <= self.xy and
                abs(math.atan2(math.sin(yaw_error), math.cos(yaw_error))) <= self.yaw)

    def observe(self, distance, yaw_error, stamp, now):
        if stamp < self.last_stamp or not self.contains(distance, yaw_error):
            self.reset()
            return False
        if stamp == self.last_stamp:
            return False
        self.last_stamp = stamp
        if self.since is None:
            self.since = now
        self.count += 1
        return self.count >= 2 and now-self.since >= self.dwell
