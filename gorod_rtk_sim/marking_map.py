"""World-space marking geometry for the city polygon.

Provides all road markings as simple 2D primitives (segments, arcs, rectangles)
with no IsaacLab dependency. Used by the camera projection module.

Three marking classes:
    DASHED    — lane guide lines (rendered as continuous coloured lines in projection)
    SOLID     — stop lines before crosswalks
    CROSSWALK — zebra zones (rendered as filled rectangles in projection)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

CELL_SIZE = 0.8
GRID_N = 5
_HALF = CELL_SIZE * GRID_N / 2.0

CORNER_R = 0.30
MW = 0.04
DASH_LEN = 0.20
# Gap between the inner (intersection-facing) cell edge and crosswalk stripe.
CROSSWALK_EDGE_GAP_INNER = 0.02
# Crosswalk centre offset from cell centre toward the field centre.
CROSSWALK_SHIFT_IN = CELL_SIZE / 2.0 - CROSSWALK_EDGE_GAP_INNER - DASH_LEN / 2.0
STOP_LINE_DEPTH = 0.028
CURB_W = 0.02
CROSSWALK_LATERAL = CELL_SIZE - 2.0 * CURB_W
ARM_JOIN_EPS = 0.07

# Projection colours (BGR for OpenCV)
COLOR_DASHED_BGR = (0, 200, 255)     # yellow
COLOR_SOLID_BGR = (255, 255, 255)    # white
COLOR_CROSSWALK_BGR = (255, 180, 80) # light blue


def _cell_xy(row: int, col: int) -> tuple[float, float]:
    x = -_HALF + CELL_SIZE * col + CELL_SIZE / 2.0
    y = -_HALF + CELL_SIZE * row + CELL_SIZE / 2.0
    return (x, y)


def _outer_ring_midline():
    m = _HALF - CELL_SIZE / 2.0
    return (-m, m, -m, m)


# -----------------------------------------------------------------------
# Primitives
# -----------------------------------------------------------------------

@dataclass
class Segment:
    """Straight line from (x0,y0) to (x1,y1) in world frame."""
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass
class Arc:
    """Circular arc: centre (cx,cy), radius r, from ang0 to ang1 (radians, CCW)."""
    cx: float
    cy: float
    r: float
    ang0: float
    ang1: float


@dataclass
class Rect:
    """Axis-aligned rectangle: centre (cx,cy), half-extents (hx,hy). rotation=0 means aligned with world axes."""
    cx: float
    cy: float
    hx: float
    hy: float
    angle: float = 0.0


@dataclass
class MarkingMap:
    """All road markings as world-space primitives, grouped by type."""
    dashed_segments: list[Segment] = field(default_factory=list)
    dashed_arcs: list[Arc] = field(default_factory=list)
    solid_rects: list[Rect] = field(default_factory=list)
    crosswalk_rects: list[Rect] = field(default_factory=list)


def build_marking_map() -> MarkingMap:
    m = MarkingMap()
    xl, xr, yb, yt = _outer_ring_midline()
    r = min(CORNER_R, (xr - xl) / 2 - 0.02, (yt - yb) / 2 - 0.02)

    # --- Perimeter straight edges ---
    m.dashed_segments.append(Segment(xl + r, yb, xr - r, yb))  # bottom
    m.dashed_segments.append(Segment(xl + r, yt, xr - r, yt))  # top
    m.dashed_segments.append(Segment(xl, yb + r, xl, yt - r))  # left
    m.dashed_segments.append(Segment(xr, yb + r, xr, yt - r))  # right

    # --- Corner arcs ---
    m.dashed_arcs.append(Arc(xl + r, yb + r, r, math.pi, 1.5 * math.pi))      # BL
    m.dashed_arcs.append(Arc(xr - r, yb + r, r, -math.pi / 2, 0.0))           # BR
    m.dashed_arcs.append(Arc(xr - r, yt - r, r, 0.0, math.pi / 2))            # TR
    m.dashed_arcs.append(Arc(xl + r, yt - r, r, math.pi / 2, math.pi))         # TL

    # --- Extensions to field edges ---
    edge = _HALF
    eps = 0.02
    m.dashed_segments.append(Segment(0.0, -edge + eps, 0.0, yb - eps))   # bottom ext
    m.dashed_segments.append(Segment(0.0, yt + eps, 0.0, edge - eps))    # top ext
    m.dashed_segments.append(Segment(-edge + eps, 0.0, xl - eps, 0.0))   # left ext
    m.dashed_segments.append(Segment(xr + eps, 0.0, edge - eps, 0.0))    # right ext

    # --- Radial arms (stop before crosswalks) ---
    s = CROSSWALK_SHIFT_IN
    xw_margin = 0.04
    _, cy12 = _cell_xy(1, 2)
    _, cy32 = _cell_xy(3, 2)
    cx21, _ = _cell_xy(2, 1)
    cx23, _ = _cell_xy(2, 3)
    half_xw = DASH_LEN / 2

    arm_s_end = (cy12 + s) - half_xw - xw_margin
    m.dashed_segments.append(Segment(0.0, yb + ARM_JOIN_EPS, 0.0, arm_s_end))

    arm_n_start = (cy32 - s) + half_xw + xw_margin
    m.dashed_segments.append(Segment(0.0, arm_n_start, 0.0, yt - ARM_JOIN_EPS))

    arm_w_end = (cx21 + s) - half_xw - xw_margin
    m.dashed_segments.append(Segment(xl + ARM_JOIN_EPS, 0.0, arm_w_end, 0.0))

    arm_e_start = (cx23 - s) + half_xw + xw_margin
    m.dashed_segments.append(Segment(arm_e_start, 0.0, xr - ARM_JOIN_EPS, 0.0))

    # --- Crosswalk rectangles ---
    half_lat = CROSSWALK_LATERAL / 2

    cx21w, cy21w = _cell_xy(2, 1)
    cx21w += s
    m.crosswalk_rects.append(Rect(cx21w, cy21w, DASH_LEN / 2, half_lat))

    cx23w, cy23w = _cell_xy(2, 3)
    cx23w -= s
    m.crosswalk_rects.append(Rect(cx23w, cy23w, DASH_LEN / 2, half_lat))

    cx12w, cy12w = _cell_xy(1, 2)
    cy12w += s
    m.crosswalk_rects.append(Rect(cx12w, cy12w, half_lat, DASH_LEN / 2))

    cx32w, cy32w = _cell_xy(3, 2)
    cy32w -= s
    m.crosswalk_rects.append(Rect(cx32w, cy32w, half_lat, DASH_LEN / 2))

    # --- Solid stop lines ---
    stop_tail = STOP_LINE_DEPTH / 2 + 0.034
    stop_half = DASH_LEN / 2 + stop_tail

    m.solid_rects.append(Rect(cx21w - stop_half, cy21w, STOP_LINE_DEPTH / 2, half_lat))
    m.solid_rects.append(Rect(cx23w + stop_half, cy23w, STOP_LINE_DEPTH / 2, half_lat))
    m.solid_rects.append(Rect(cx12w, cy12w - stop_half, half_lat, STOP_LINE_DEPTH / 2))
    m.solid_rects.append(Rect(cx32w, cy32w + stop_half, half_lat, STOP_LINE_DEPTH / 2))

    return m
