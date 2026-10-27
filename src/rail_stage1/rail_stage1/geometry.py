"""Pure geometry: mask -> three straight sections -> approximate entry on floor."""
from itertools import combinations
import math
import cv2
import numpy as np


def rotation(q):
    x, y, z, w = q
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError('invalid TF quaternion')
    x, y, z, w = np.array(q) / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def thin(mask):
    a = np.pad((mask > 0).astype(np.uint8), 1)
    while True:
        changed = False
        for phase in (0, 1):
            p = a[1:-1, 1:-1]
            nb = [a[:-2, 1:-1], a[:-2, 2:], a[1:-1, 2:], a[2:, 2:],
                  a[2:, 1:-1], a[2:, :-2], a[1:-1, :-2], a[:-2, :-2]]
            count = sum(nb)
            transitions = sum((nb[i] == 0) & (nb[(i+1) % 8] == 1) for i in range(8))
            c1 = nb[0]*nb[2]*(nb[4] if phase == 0 else nb[6])
            c2 = (nb[2] if phase == 0 else nb[0])*nb[4]*nb[6]
            remove = (p == 1) & (count >= 2) & (count <= 6) & (transitions == 1) & (c1 == 0) & (c2 == 0)
            if remove.any():
                p[remove] = 0
                changed = True
        if not changed:
            return a[1:-1, 1:-1]*255


def floor_plane(depth, mask, k, distortion, r, camera_origin, cfg):
    if cfg['plane_mode'] == 'fixed':
        return np.array([0., 0., 1.]), -cfg['floor_z']
    if cfg['plane_mode'] != 'depth':
        raise ValueError('plane_mode must be depth or fixed')
    ys, xs = np.mgrid[0:depth.shape[0]:8, 0:depth.shape[1]:8]
    d = depth[ys, xs]
    excluded = cv2.dilate(mask, np.ones((15, 15), np.uint8)) > 0
    valid = np.isfinite(d) & (d > .15) & (d < cfg['max_range']) & ~excluded[ys, xs]
    pixels = np.column_stack((xs[valid], ys[valid])).astype(np.float64)
    if len(pixels) < cfg['plane_min_points']:
        raise ValueError('not enough floor depth points')
    rays = cv2.undistortPoints(pixels[:, None], k, distortion).reshape(-1, 2)
    pts = np.column_stack((rays, np.ones(len(rays)))) * d[valid, None]
    pts = pts @ r.T + camera_origin
    pts = pts[(pts[:, 0] > .1) & (np.abs(pts[:, 1]) < 2.) &
              (np.abs(pts[:, 2]-cfg['floor_z']) < cfg['plane_height_tolerance'])]
    if len(pts) < cfg['plane_min_points']:
        raise ValueError('not enough floor depth points')
    rng = np.random.default_rng(17)
    best = np.zeros(len(pts), dtype=bool)
    for _ in range(80):
        p = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(p[1]-p[0], p[2]-p[0])
        if np.linalg.norm(n) < 1e-8:
            continue
        n /= np.linalg.norm(n)
        if abs(n[2]) < math.cos(math.radians(cfg['plane_max_tilt_deg'])):
            continue
        inliers = np.abs((pts-p[0]) @ n) < cfg['plane_residual']
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < cfg['plane_min_points'] or best.mean() < cfg['plane_min_fraction']:
        raise ValueError('floor plane not confirmed')
    center = pts[best].mean(0)
    _, _, vt = np.linalg.svd(pts[best]-center, full_matrices=False)
    n = vt[-1]
    if n[2] < 0:
        n = -n
    if n[2] < math.cos(math.radians(cfg['plane_max_tilt_deg'])):
        raise ValueError('floor tilt exceeds configured limit')
    return n, -float(n @ center)


def bird_view(mask, k, distortion, r, camera_origin, normal, offset, cfg):
    origin = -offset * normal
    ex = np.array([1., 0., 0.]) - normal[0]*normal
    ex /= np.linalg.norm(ex)
    ey = np.cross(normal, ex)
    resolution = cfg['bev_resolution']
    xs = np.arange(cfg['bev_x_min'], cfg['bev_x_max'], resolution)
    ys = np.arange(-cfg['bev_half_width'], cfg['bev_half_width'], resolution)
    xx, yy = np.meshgrid(xs, ys)
    points = origin + xx[..., None]*ex + yy[..., None]*ey + cfg['feature_height']*normal
    camera = (points-camera_origin) @ r
    pixels, _ = cv2.projectPoints(camera.reshape(-1, 3), np.zeros(3), np.zeros(3), k, distortion)
    pixels = pixels.reshape(*xx.shape, 2).astype(np.float32)
    pixels[camera[..., 2] <= .05] = -1
    bev = cv2.remap(mask, pixels[..., 0], pixels[..., 1], cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT)
    return bev, origin, ex, ey


def line_candidates(skeleton, cfg):
    res = cfg['bev_resolution']
    raw = cv2.HoughLinesP(skeleton, 1, np.pi/720, cfg['hough_votes'],
                         minLineLength=cfg['min_line_length']/res,
                         maxLineGap=cfg['max_line_gap']/res)
    if raw is None:
        return []
    yy, xx = np.nonzero(skeleton)
    points = np.column_stack((xx*res+cfg['bev_x_min'], yy*res-cfg['bev_half_width']))
    result = []
    for x1, y1, x2, y2 in np.asarray(raw).reshape(-1, 4):
        a = np.array([x1*res+cfg['bev_x_min'], y1*res-cfg['bev_half_width']])
        b = np.array([x2*res+cfg['bev_x_min'], y2*res-cfg['bev_half_width']])
        u = (b-a)/np.linalg.norm(b-a)
        n = np.array([-u[1], u[0]])
        near = points[np.abs((points-a) @ n) < cfg['line_support_band']]
        if len(near) < cfg['hough_votes']:
            continue
        t = (near-a) @ u
        lo, hi = np.quantile(t, [.02, .98])
        if hi-lo < cfg['min_line_length']:
            continue
        result.append(dict(a=a+lo*u, b=a+hi*u, u=u, n=n, support=len(near)))
    result.sort(key=lambda l: l['support'], reverse=True)
    kept = []
    for line in result:
        if any(abs(line['u'] @ old['u']) > math.cos(math.radians(4)) and
               abs((line['a']-old['a']) @ old['n']) < .04 for old in kept):
            continue
        kept.append(line)
        if len(kept) >= cfg['max_candidates']:
            break
    return kept


def intersection(a, b):
    matrix = np.column_stack((a['u'], -b['u']))
    if abs(np.linalg.det(matrix)) < 1e-6:
        return None
    t = np.linalg.solve(matrix, b['a']-a['a'])[0]
    return a['a'] + t*a['u']


def select_three(lines, cfg):
    choices = []
    parallel = math.cos(math.radians(cfg['parallel_tolerance_deg']))
    perpendicular = math.sin(math.radians(cfg['perpendicular_tolerance_deg']))
    for left, right in combinations(lines, 2):
        dot = left['u'] @ right['u']
        if abs(dot) < parallel:
            continue
        u = left['u'] + (right['u'] if dot >= 0 else -right['u'])
        u /= np.linalg.norm(u)
        n = np.array([-u[1], u[0]])
        gauge = abs(((left['a']+left['b'])/2-(right['a']+right['b'])/2) @ n)
        if abs(gauge-cfg['rail_gauge']) > cfg['gauge_tolerance']:
            continue
        for bar in lines:
            if bar is left or bar is right or abs(bar['u'] @ u) > perpendicular:
                continue
            il, ir = intersection(left, bar), intersection(right, bar)
            if il is None or ir is None:
                continue
            entry = (il+ir)/2
            # The observed crossbar must reach both arms, allowing rounded corners.
            bt = (np.array([bar['a'], bar['b']])-entry) @ bar['u']
            it = (np.array([il, ir])-entry) @ bar['u']
            if it.min() < bt.min()-cfg['corner_allowance'] or it.max() > bt.max()+cfg['corner_allowance']:
                continue
            arms = np.array([left['a']-il, left['b']-il, right['a']-ir, right['b']-ir]) @ u
            if arms.mean() < 0:
                u = -u
                arms = -arms
            if (arms.min() < -cfg['corner_allowance'] or
                    min(arms[:2].max(), arms[2:].max()) < cfg['min_arm_extent']):
                continue
            score = left['support']+right['support']+bar['support']
            choices.append((score, entry, u, [left, right, bar]))
    if not choices:
        raise ValueError('no geometrically consistent three-line U')
    choices.sort(key=lambda c: c[0], reverse=True)
    best = choices[0]
    for other in choices[1:]:
        if other[0] < best[0]*cfg['ambiguity_score_ratio']:
            break
        if (np.linalg.norm(other[1]-best[1]) > cfg['ambiguity_position'] or
                other[2] @ best[2] < math.cos(math.radians(cfg['ambiguity_angle_deg']))):
            raise ValueError('multiple competing three-line poses')
    return best[1], best[2], best[3]


def detect(mask, depth, k, distortion, r, camera_origin, cfg, debug=None, return_feature=False):
    if np.count_nonzero(mask) < cfg['min_mask_area_px']:
        raise ValueError('mask below minimum area')
    normal, offset = floor_plane(depth, mask, k, distortion, r, camera_origin, cfg)
    bev, origin, ex, ey = bird_view(mask, k, distortion, r, camera_origin, normal, offset, cfg)
    skeleton = thin(bev)
    lines = line_candidates(skeleton, cfg)
    try:
        entry, u, selected = select_three(lines, cfg)
    except ValueError:
        if debug is not None:
            debug(bev, [])
        raise
    if debug is not None:
        debug(bev, selected)
    point = origin + entry[0]*ex + entry[1]*ey
    direction = u[0]*ex + u[1]*ey
    if return_feature:
        return point, direction, bev, selected, point+cfg['feature_height']*normal
    return point, direction, bev, selected
