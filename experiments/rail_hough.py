"""Offline Hough experiment on one saved rail mask. No ROS or motion commands."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def thin(mask):
    """Zhang-Suen thinning preserves the U shape and yields one-pixel axes."""
    a = np.pad((mask > 0).astype(np.uint8), 1)
    iterations = 0
    while True:
        changed = False
        for phase in (0, 1):
            p = a[1:-1, 1:-1]
            nb = [a[:-2, 1:-1], a[:-2, 2:], a[1:-1, 2:], a[2:, 2:],
                  a[2:, 1:-1], a[2:, :-2], a[1:-1, :-2], a[:-2, :-2]]
            count = sum(nb)
            transitions = sum((nb[i] == 0) & (nb[(i+1) % 8] == 1) for i in range(8))
            if phase == 0:
                c1, c2 = nb[0]*nb[2]*nb[4], nb[2]*nb[4]*nb[6]
            else:
                c1, c2 = nb[0]*nb[2]*nb[6], nb[0]*nb[4]*nb[6]
            remove = (p == 1) & (count >= 2) & (count <= 6) & (transitions == 1) & (c1 == 0) & (c2 == 0)
            if remove.any():
                p[remove] = 0
                changed = True
        iterations += 1
        if not changed:
            return a[1:-1, 1:-1] * 255, iterations


def candidates(image, threshold=60):
    lines = cv2.HoughLines(image, 1, np.pi/720, threshold)
    if lines is None:
        return []
    y, x = np.nonzero(image)
    pts = np.column_stack((x, y)).astype(float)
    result = []
    for rho, theta in lines[:, 0, :]:
        normal = np.array([np.cos(theta), np.sin(theta)])
        direction = np.array([-np.sin(theta), np.cos(theta)])
        selected = pts[np.abs(pts @ normal - rho) <= 3.0]
        if len(selected) < 30:
            continue
        t = selected @ direction
        low, high = np.quantile(t, [0.02, 0.98])
        result.append(dict(rho=float(rho), theta=float(theta),
                           angle_deg=float(np.degrees(np.arctan2(direction[1], direction[0]))),
                           support=int(len(selected)), length_px=float(high-low),
                           endpoints=[(rho*normal+low*direction).tolist(),
                                      (rho*normal+high*direction).tolist()]))
    return sorted(result, key=lambda line: line['support'], reverse=True)


def select_three(lines, width):
    # Require two upright arms and one transverse bar. No metric parallelism:
    # parallel 3D pipes can converge in a perspective image.
    groups = {'left': [], 'right': [], 'crossbar': []}
    for line in lines:
        dx, dy = np.subtract(*line['endpoints'])
        if abs(dx) > abs(dy):
            groups['crossbar'].append(line)
        else:
            # Classify the leg by where its infinite line crosses image y=100.
            x100 = (line['rho'] - 100*np.sin(line['theta']))/np.cos(line['theta'])
            groups['left' if x100 < width/2 else 'right'].append(line)
    chosen = []
    for role, options in groups.items():
        if not options:
            raise RuntimeError(f'No line detected for {role}')
        chosen.append(dict(role=role, **options[0]))
    return chosen


def draw_lines(image, lines, colors, label=False):
    out = image.copy()
    for i, line in enumerate(lines):
        p, q = (tuple(np.round(v).astype(int)) for v in line['endpoints'])
        color = colors[i % len(colors)]
        cv2.line(out, p, q, color, 2, cv2.LINE_AA)
        if label:
            center = tuple(np.round((np.array(p)+q)/2).astype(int))
            cv2.putText(out, line['role'], center, cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(out, line['role'], center, cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, color, 1, cv2.LINE_AA)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('snapshot', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.snapshot
    out = args.output or root/'hough_step1'
    out.mkdir(parents=True, exist_ok=True)
    mask = cv2.imread(str(root/'mask.png'), cv2.IMREAD_GRAYSCALE)
    rgb = cv2.imread(str(root/'rgb.png'))
    if mask is None or rgb is None:
        raise RuntimeError('Missing rgb.png or mask.png')
    skeleton, iterations = thin(mask)
    edges = cv2.Canny(mask, 50, 150)
    raw = candidates(edges)
    sk = candidates(skeleton)
    chosen = select_three(sk, mask.shape[1])
    colors = [(255, 180, 0), (255, 0, 255), (0, 0, 255)]
    cv2.imwrite(str(out/'skeleton.png'), skeleton)
    cv2.imwrite(str(out/'edges.png'), edges)
    cv2.imwrite(str(out/'raw_hough.png'), draw_lines(rgb, raw[:40], [(0, 220, 255)]))
    final = draw_lines(rgb, chosen, colors, label=True)
    cv2.imwrite(str(out/'three_lines.png'), final)
    skview = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(str(out/'three_lines_on_skeleton.png'), draw_lines(skview, chosen, colors))
    # Intersections of the extended fitted lines are geometric corners,
    # not literal tube endpoints: the actual object has rounded bends.
    def intersect(a, b):
        A = np.array([[np.cos(v['theta']), np.sin(v['theta'])] for v in (a,b)])
        return np.linalg.solve(A, [a['rho'], b['rho']]).tolist()
    byrole = {line['role']: line for line in chosen}
    corners = {role: intersect(byrole[role], byrole['crossbar']) for role in ('left','right')}
    report = dict(method='Canny+Hough vs Zhang-Suen skeleton+Hough',
                  hough=dict(rho_px=1, theta_step_deg=0.25, vote_threshold=60),
                  image_size=[int(mask.shape[1]), int(mask.shape[0])],
                  mask_pixels=int(np.count_nonzero(mask)), skeleton_pixels=int(np.count_nonzero(skeleton)),
                  thinning_iterations=iterations, raw_candidates=len(raw), skeleton_candidates=len(sk),
                  selected=chosen, virtual_corners_px=corners,
                  threshold_sensitivity={str(t): select_three(candidates(skeleton,t), mask.shape[1]) for t in (40,60,80,100)})
    (out/'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
