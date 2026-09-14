"""Final goal points on straight road cells (competition polygon).

Each *straight* road cell (not corner / intersection / building) has **two**
lanes.  We place one goal at the centre of each lane.

Road axis:
  * **Horizontal** strips (lanes run east–west): offset along north–south.
  * **Vertical** strips (lanes run north–south): offset along west–east.

Numbering: cells in scan order left → right, top → bottom (north row first);
within each cell: **left** lane (relative to travel direction along the road),
then **right** lane.

No IsaacLab dependency.
"""

from __future__ import annotations

import math

from lane_overlay import (
    BUILDING_CELLS,
    GRID,
    LANE_CENTER,
    LANE_W,
    NORTH,
    EAST,
    _INTERSECTION_SET,
    _CORNER_SET,
    _RIGHT_OF,
    _cell_xy,
)

_GOAL_Z = 0.018

# ---------------------------------------------------------------------------
# Straight road cells (full lane segments, not corners / crossings)
# ---------------------------------------------------------------------------

def _straight_road_cells() -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    for row in range(GRID):
        for col in range(GRID):
            c = (row, col)
            if c in BUILDING_CELLS:
                continue
            if c in _CORNER_SET:
                continue
            if c in _INTERSECTION_SET:
                continue
            out.append(c)
    return out


def _travel_dir_for_straight_cell(row: int, col: int) -> tuple[int, int]:
    """Reference travel direction along the road (defines left/right lanes)."""
    # Horizontal strips (lanes E–W): lateral offset along N/S
    if row in (0, 4) or (col in (1, 3) and row == 2):
        return EAST
    # Vertical strips (lanes N–S): lateral offset along E/W
    if col in (0, 4) or (row in (1, 3) and col == 2):
        return NORTH
    raise ValueError(f"not a straight-classified cell: {(row, col)}")


def _lane_centers(row: int, col: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return (left_lane_xyz, right_lane_xyz) relative to travel direction."""
    cx, cy = _cell_xy(row, col)
    travel = _travel_dir_for_straight_cell(row, col)
    rd = _RIGHT_OF[travel]
    # Right lane = toward curb on the right side of travel; left = opposite
    rx = cx + rd[1] * LANE_CENTER
    ry = cy + rd[0] * LANE_CENTER
    lx = cx - rd[1] * LANE_CENTER
    ly = cy - rd[0] * LANE_CENTER
    return (lx, ly, _GOAL_Z), (rx, ry, _GOAL_Z)


def build_final_goal_points() -> list[dict]:
    """Ordered list of goal dicts: id (1-based), row, col, lane, x, y, z."""
    cells = _straight_road_cells()
    cells.sort(key=lambda rc: (-rc[0], rc[1]))
    goals: list[dict] = []
    gid = 1
    for row, col in cells:
        left_xyz, right_xyz = _lane_centers(row, col)
        for lane, (x, y, z) in (("left", left_xyz), ("right", right_xyz)):
            goals.append({
                "id": gid,
                "row": row,
                "col": col,
                "lane": lane,
                "x": x,
                "y": y,
                "z": z,
            })
            gid += 1
    return goals


FINAL_GOAL_POINTS: list[dict] = build_final_goal_points()


# ---------------------------------------------------------------------------
# 7-segment digit lines (for debug draw — no text API)
# ---------------------------------------------------------------------------
# Segments a..g in local (u,v): u right, v up.  Box width 1, height 2.
#   a: top    b: upper-right  c: lower-right  d: bottom
#   e: lower-left  f: upper-left  g: middle

_SEG_PATTERNS: dict[int, tuple[int, ...]] = {
    0: (1, 1, 1, 1, 1, 1, 0),
    1: (0, 1, 1, 0, 0, 0, 0),
    2: (1, 1, 0, 1, 1, 0, 1),
    3: (1, 1, 1, 1, 0, 0, 1),
    4: (0, 1, 1, 0, 0, 1, 1),
    5: (1, 0, 1, 1, 0, 1, 1),
    6: (1, 0, 1, 1, 1, 1, 1),
    7: (1, 1, 1, 0, 0, 0, 0),
    8: (1, 1, 1, 1, 1, 1, 1),
    9: (1, 1, 1, 1, 0, 1, 1),
}


def _segment_endpoints(scale: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Seven segments as ((u0,v0),(u1,v1)) in local coords, height 2*scale."""
    s = scale
    # u in [0,s], v in [0,2s], origin bottom-left of digit
    return [
        ((0, 2 * s), (s, 2 * s)),       # a top
        ((s, 2 * s), (s, s)),          # b upper right
        ((s, s), (s, 0)),              # c lower right
        ((0, 0), (s, 0)),              # d bottom
        ((0, s), (0, 0)),              # e lower left
        ((0, 2 * s), (0, s)),          # f upper left
        ((0, s), (s, s)),              # g middle
    ]


def append_goal_marker_lines(
    gx: float, gy: float, gz: float,
    goal_id: int,
    lines_s: list,
    lines_e: list,
    lines_col: list,
    lines_sz: list,
    *,
    color: tuple[float, float, float, float] = (1.0, 0.85, 0.1, 1.0),
    digit_scale: float = 0.07,
    gap: float = 0.02,
) -> None:
    """Append debug-draw line segments for a goal point + its number.

    Digits are drawn in the XY plane near the point (offset +X,+Y).
    """
    # Offset label so it does not cover the point
    ox = gx + 0.12
    oy = gy + 0.12
    oz = gz

    s = digit_scale
    segs = _segment_endpoints(s)
    patterns = _SEG_PATTERNS

    def draw_digit(d: int, base_u: float, base_v: float) -> None:
        pat = patterns[d]
        for on, ((u0, v0), (u1, v1)) in zip(pat, segs):
            if not on:
                continue
            lines_s.append((ox + base_u + u0, oy + base_v + v0, oz))
            lines_e.append((ox + base_u + u1, oy + base_v + v1, oz))
            lines_col.append(color)
            lines_sz.append(2)

    if goal_id < 10:
        draw_digit(goal_id, 0.0, 0.0)
    else:
        draw_digit(goal_id // 10, 0.0, 0.0)
        w = digit_scale + gap
        draw_digit(goal_id % 10, w, 0.0)


def append_goal_circle_lines(
    gx: float, gy: float, gz: float,
    lines_s: list,
    lines_e: list,
    lines_col: list,
    lines_sz: list,
    *,
    color: tuple[float, float, float, float] = (1.0, 0.85, 0.1, 0.95),
    segments: int = 32,
    line_size: int = 2,
) -> None:
    """Append a circular outline centred at goal point.

    Circle diameter equals one lane width.
    """
    r = LANE_W / 2.0
    n = max(12, int(segments))
    pts = []
    for i in range(n):
        a = 2.0 * 3.141592653589793 * i / n
        pts.append((gx + r * math.cos(a), gy + r * math.sin(a), gz))
    for i in range(n):
        j = (i + 1) % n
        lines_s.append(pts[i])
        lines_e.append(pts[j])
        lines_col.append(color)
        lines_sz.append(line_size)
