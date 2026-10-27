"""IBVS and straight-drive logic without ROS, hardware or OpenCV dependencies."""
from dataclasses import dataclass
import math
import numpy as np


def target_pixel(width, height, bottom_offset):
    if width < 2 or height < 2 or not 0 <= bottom_offset < height-1:
        raise ValueError('target bottom offset outside image')
    return np.array([(width-1)/2., height-1-bottom_offset], dtype=float)


def point_jacobian(camera_point, base_from_camera_rotation, camera_origin_base, omni=False):
    """Normalized image velocity for [body forward velocity, body yaw rate].

    A stationary world point has Pdot=-v_camera-w_camera cross P.
    Camera translation includes the lever arm w_body cross camera_origin.
    """
    p = np.asarray(camera_point, dtype=float)
    r = np.asarray(base_from_camera_rotation, dtype=float)
    origin = np.asarray(camera_origin_base, dtype=float)
    if p.shape != (3,) or r.shape != (3, 3) or origin.shape != (3,):
        raise ValueError('invalid camera geometry shape')
    if not all(np.isfinite(a).all() for a in (p, r, origin)) or p[2] <= .05:
        raise ValueError('invalid camera geometry or depth')
    projection = np.array([[1/p[2], 0, -p[0]/p[2]**2],
                           [0, 1/p[2], -p[1]/p[2]**2]])
    ez = np.array([0., 0., 1.])
    forward_motion = -(r.T @ np.array([1., 0., 0.]))
    yaw_motion = -(r.T @ np.cross(ez, origin)) - np.cross(r.T @ ez, p)
    if omni:
        lateral_motion = -(r.T @ np.array([0., 1., 0.]))
        return projection @ np.column_stack((forward_motion, lateral_motion, yaw_motion))
    return projection @ np.column_stack((forward_motion, yaw_motion))


def rail_heading(direction_base):
    direction = np.asarray(direction_base, dtype=float)
    if direction.shape != (3,) or not np.isfinite(direction).all() or np.linalg.norm(direction[:2]) < .1:
        raise ValueError('invalid directed rail axis')
    return math.atan2(direction[1], direction[0])


def omni_ibvs_command(camera_point, desired_normalized, heading_error, r, origin,
                      gain, heading_gain, limits, minimums, max_condition=100., max_reverse=None):
    """Image point + visually estimated directed floor-axis angle -> vx, vy, wz.

    Under planar base motion the axis angle derivative is -wz. Point rows
    include camera lever arm and lateral translation, compensating yaw motion.
    Inputs are task errors after their deadbands, not odometry/map yaw.
    """
    desired = np.asarray(desired_normalized, dtype=float)
    limits, minimums = np.asarray(limits, dtype=float), np.asarray(minimums, dtype=float)
    if desired.shape != (2,) or limits.shape != (3,) or minimums.shape != (3,):
        raise ValueError('invalid omni settings shape')
    if (not np.isfinite(np.r_[desired, limits, minimums, heading_error, gain, heading_gain]).all()
            or min(gain, heading_gain) <= 0 or np.any(limits <= 0)
            or np.any(minimums < 0) or np.any(minimums > limits)):
        raise ValueError('invalid omni gains or velocity bounds')
    jacobian = np.vstack((point_jacobian(camera_point, r, origin, omni=True), [0., 0., -1.]))
    if np.linalg.cond(jacobian) > max_condition:
        raise ValueError('omni image Jacobian is poorly conditioned')
    point = np.asarray(camera_point)
    angle = math.atan2(math.sin(heading_error), math.cos(heading_error))
    error = np.r_[point[:2]/point[2]-desired, angle]
    weighted = error*np.array([gain, gain, heading_gain])
    velocity = np.linalg.solve(jacobian, -weighted)
    if max_reverse is not None:
        if not math.isfinite(max_reverse) or max_reverse < minimums[0] or max_reverse <= 0:
            raise ValueError('invalid reverse velocity limit')
        limits = limits.copy()
        if velocity[0] < 0:
            limits[0] = max_reverse
    bounded = velocity*min(1., float(np.min(limits/np.maximum(np.abs(velocity), 1e-12))))
    active = np.abs(bounded) > 1e-8
    floored = np.where(active, np.sign(bounded)*np.maximum(np.abs(bounded), minimums), 0.)
    # Per-axis stiction compensation can spoil coupled image error descent.
    # Keep it only when descending; otherwise amplify all axes together.
    if weighted @ (jacobian @ floored) < 0 or not active.any():
        return floored
    boost = max(1., float(np.min(minimums[active]/np.abs(bounded[active]))))
    boosted = bounded*boost
    return boosted*min(1., float(np.min(limits/np.maximum(np.abs(boosted), 1e-12))))


class VelocitySlew:
    def __init__(self, linear_acceleration, yaw_acceleration):
        if not all(math.isfinite(v) and v > 0 for v in (linear_acceleration, yaw_acceleration)):
            raise ValueError('invalid alignment acceleration limits')
        self.acceleration = np.array([linear_acceleration, linear_acceleration, yaw_acceleration])
        self.reset()

    def reset(self):
        self.velocity = np.zeros(3)
        self.last_time = None

    def step(self, velocity, now):
        requested = np.asarray(velocity, dtype=float)
        if requested.shape != (3,) or not np.isfinite(requested).all() or not math.isfinite(now):
            raise ValueError('invalid requested velocity')
        dt = 0. if self.last_time is None else max(0., min(.1, now-self.last_time))
        delta = self.acceleration*dt
        self.velocity += np.clip(requested-self.velocity, -delta, delta)
        self.last_time = now
        return self.velocity.copy()


def ibvs_command(camera_point, desired_normalized, r, origin, gain,
                 max_forward, max_reverse, max_yaw, max_condition=100.):
    desired = np.asarray(desired_normalized, dtype=float)
    if desired.shape != (2,) or not np.isfinite(desired).all():
        raise ValueError('invalid target feature')
    if min(gain, max_forward, max_reverse, max_yaw) <= 0:
        raise ValueError('IBVS gain and velocity limits must be positive')
    jacobian = point_jacobian(camera_point, r, origin)
    if np.linalg.cond(jacobian) > max_condition:
        raise ValueError('image Jacobian is poorly conditioned')
    point = np.asarray(camera_point)
    error = point[:2]/point[2]-desired
    velocity = np.linalg.solve(jacobian, -gain*error)
    limits = np.array([max_forward if velocity[0] >= 0 else max_reverse, max_yaw])
    # Common scaling preserves the intended direction of image error reduction.
    scale = min(1., float(np.min(limits/np.maximum(np.abs(velocity), 1e-12))))
    return velocity*scale


class AlignmentGate:
    def __init__(self, tolerance_px, frames, dwell_sec, heading_tolerance_deg=None):
        if tolerance_px <= 0 or frames < 1 or dwell_sec < 0:
            raise ValueError('invalid alignment confirmation settings')
        self.tolerance = tolerance_px
        self.frames = frames
        self.dwell = dwell_sec
        self.heading_tolerance = (None if heading_tolerance_deg is None else math.radians(heading_tolerance_deg))
        if self.heading_tolerance is not None and (not math.isfinite(self.heading_tolerance) or self.heading_tolerance <= 0):
            raise ValueError('invalid heading confirmation tolerance')
        self.reset()

    def reset(self):
        self.count = 0
        self.first_time = None
        self.last_stamp = -1

    def observe(self, error_px, stamp_ns, now, heading_error=None):
        if stamp_ns <= self.last_stamp:
            return False
        self.last_stamp = stamp_ns
        error = np.asarray(error_px)
        heading_ok = self.heading_tolerance is None or (
            heading_error is not None and math.isfinite(heading_error) and
            abs(math.atan2(math.sin(heading_error), math.cos(heading_error))) <= self.heading_tolerance)
        if error.shape != (2,) or not np.isfinite(error).all() or np.max(np.abs(error)) > self.tolerance or not heading_ok:
            self.count, self.first_time = 0, None
            return False
        if self.first_time is None:
            self.first_time = now
        self.count += 1
        return self.count >= self.frames and now-self.first_time >= self.dwell


@dataclass(frozen=True)
class Localization:
    x: float
    y: float
    yaw: float
    frame: str
    stamp_ns: int
    received_at: float


@dataclass(frozen=True)
class DriveStep:
    state: str
    vx: float
    distance: float
    time_remaining: float
    reason: str = ''


class StraightDrive:
    """Constant body vx; progress is localization projected on initial heading."""
    def __init__(self, start, distance, speed, now, localization_timeout=.5,
                 jump_allowance=.15, max_localization_speed=1., max_yaw_error_deg=20.):
        if not np.isfinite([distance, speed, now]).all() or distance <= 0 or speed <= 0:
            raise ValueError('distance and speed must be finite and positive')
        self.start = self.last = start
        self.distance, self.speed = distance, speed
        self.direction = np.array([math.cos(start.yaw), math.sin(start.yaw)])
        self.deadline = now+2*distance/speed
        self.localization_timeout = localization_timeout
        self.jump_allowance = jump_allowance
        self.max_localization_speed = max_localization_speed
        self.max_yaw_error = math.radians(max_yaw_error_deg)
        self.terminal = None
        self.progress = 0.

    def step(self, pose, now, canceled=False):
        remaining = max(0., self.deadline-now)
        if self.terminal:
            return self.terminal
        reason = ''
        if canceled:
            reason = 'canceled'
        elif now >= self.deadline:
            reason = 'drive timeout (2 * distance / speed)'
        elif pose is None or now-pose.received_at > self.localization_timeout:
            reason = 'localization unavailable or stale'
        elif not np.isfinite([pose.x, pose.y, pose.yaw]).all():
            reason = 'invalid localization'
        elif pose.frame != self.start.frame:
            reason = 'localization frame changed'
        elif pose.stamp_ns < self.last.stamp_ns:
            reason = 'localization timestamp moved backwards'
        elif abs((pose.yaw-self.start.yaw+math.pi) % (2*math.pi)-math.pi) > self.max_yaw_error:
            reason = 'heading deviation exceeds limit'
        elif pose.stamp_ns > self.last.stamp_ns:
            dt = (pose.stamp_ns-self.last.stamp_ns)*1e-9
            jump = math.hypot(pose.x-self.last.x, pose.y-self.last.y)
            if jump > self.jump_allowance+self.max_localization_speed*dt:
                reason = 'localization jump'
            else:
                self.last = pose
                self.progress = max(0., float(np.array([pose.x-self.start.x, pose.y-self.start.y]) @ self.direction))
        if reason:
            self.terminal = DriveStep('CANCELED' if canceled else 'FAILED', 0., self.progress, remaining, reason)
        elif self.progress >= self.distance:
            self.terminal = DriveStep('SUCCEEDED', 0., self.progress, remaining, 'distance reached')
        if self.terminal:
            return self.terminal
        return DriveStep('DRIVE', self.speed, self.progress, remaining)
