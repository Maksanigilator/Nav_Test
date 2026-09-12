"""Lane overlay for the 5×5 city polygon.

Each road cell has two lanes (right-hand traffic). When the robot enters a
cell through the correct lane band on the entry edge, that lane is highlighted
with green debug-draw lines until the robot leaves the cell.

Geometry
--------
The dashed centre-line runs through the cell centre. Each lane occupies half
the roadway between the centre-line and the curb/building/fence.

For corner cells the lane is an arc (following the rounded dashed line,
offset inward/outward) with straight extensions to the cell edges so the
lane band visually reaches the full cell boundary.

No IsaacLab dependency.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# Grid constants (must match city_builder / marking_map)
# ---------------------------------------------------------------------------
CELL  = 0.80
GRID  = 5
_HALF = CELL * GRID / 2.0   # 2.0

CURB_W   = 0.02
MARK_W   = 0.04   # centre dashed-line width
CORNER_R = 0.30   # dashed-line corner radius

BUILDING_CELLS = {(1, 1), (1, 3), (3, 1), (3, 3)}

# Lane geometry
# road half-width from centre-line edge to curb
HALF_ROAD   = CELL / 2.0 - CURB_W - MARK_W / 2.0   # 0.36
LANE_W      = HALF_ROAD                              # 0.36 — full width to curb, no margin
LANE_CENTER = MARK_W / 2.0 + LANE_W / 2.0           # 0.20

_Z     = 0.009       # height of overlay lines
_ARC_N = 20          # segments per arc

# ---------------------------------------------------------------------------
# Direction helpers
# ---------------------------------------------------------------------------
NORTH = (1, 0)
SOUTH = (-1, 0)
EAST  = (0, 1)
WEST  = (0, -1)

_ALL_DIRS = (NORTH, SOUTH, EAST, WEST)

_RIGHT_OF = {
    NORTH: EAST,
    SOUTH: WEST,
    EAST:  SOUTH,
    WEST:  NORTH,
}


def _cell_xy(row: int, col: int) -> tuple[float, float]:
    x = -_HALF + CELL * col + CELL / 2.0
    y = -_HALF + CELL * row + CELL / 2.0
    return x, y


def _cell_of(wx: float, wy: float) -> tuple[int, int] | None:
    col = int((wx + _HALF) / CELL)
    row = int((wy + _HALF) / CELL)
    if 0 <= row < GRID and 0 <= col < GRID:
        return (row, col)
    return None


# ---------------------------------------------------------------------------
# Lane entry band on a cell edge
# ---------------------------------------------------------------------------

_BAND_TOLERANCE = 0.04   # allow slight overlap with centre-line

def _lane_band_on_edge(
    cell: tuple[int, int],
    entry_dir: tuple[int, int],
) -> tuple[float, float]:
    """Return (coord_min, coord_max) of the right-hand lane band
    on the entry edge, in the *lateral* axis of that edge.

    For corner cells the centre-line position is taken from the arc's
    tangent point on that edge (which sits at the midline of the outer loop),
    not the cell geometric centre.

    For NORTH/SOUTH entry the lateral axis is X.
    For EAST/WEST entry the lateral axis is Y.
    """
    # Centre-line coordinate on the entry edge.
    # For outer-ring cells (row 0/4, col 0/4) the dashed marking runs along
    # the midline at ±(_HALF - CELL/2).  For inner cells it runs through
    # the cell centre.  In practice _cell_xy already gives that value.
    cx, cy = _cell_xy(*cell)

    right_dir = _RIGHT_OF[entry_dir]

    # Inner boundary: start from centre-line minus a small tolerance so
    # the robot sitting right on the line is still accepted.
    inner_offset = -_BAND_TOLERANCE
    outer_offset = CELL / 2.0   # accept all the way to cell edge

    if entry_dir in (NORTH, SOUTH):
        sign = right_dir[1]
        if sign > 0:
            return (cx + inner_offset, cx + outer_offset)
        else:
            return (cx - outer_offset, cx - inner_offset)
    else:
        sign = right_dir[0]
        if sign > 0:
            return (cy + inner_offset, cy + outer_offset)
        else:
            return (cy - outer_offset, cy - inner_offset)


def _robot_in_lane_band(
    rx: float, ry: float,
    cell: tuple[int, int],
    entry_dir: tuple[int, int],
) -> bool:
    """Check if robot position falls within the right lane band on the entry edge."""
    lo, hi = _lane_band_on_edge(cell, entry_dir)
    if entry_dir in (NORTH, SOUTH):
        return lo <= rx <= hi
    else:
        return lo <= ry <= hi


# ---------------------------------------------------------------------------
# Corner cell definitions
# ---------------------------------------------------------------------------

@dataclass
class _CornerDef:
    row: int
    col: int
    arc_cx: float
    arc_cy: float
    ang0: float
    ang1: float
    entries: tuple[tuple[int, int], tuple[int, int]]

_CORNER_DEFS: list[_CornerDef] = []

def _build_corner_defs():
    mx = _HALF - CELL / 2.0   # 1.6
    r  = CORNER_R
    # entries = (entry_dir_a, entry_dir_b) where entry_dir is the delta
    # (prev_cell → this_cell), i.e. the direction the robot travels INTO
    # the corner.
    #
    # For right-hand traffic on the outer CW loop:
    #   BL (0,0): entered from (0,1) going WEST, or from (1,0) going SOUTH
    #   BR (0,4): entered from (1,4) going SOUTH, or from (0,3) going EAST
    #   TR (4,4): entered from (4,3) going EAST, or from (3,4) going NORTH
    #   TL (4,0): entered from (3,0) going NORTH, or from (4,1) going WEST
    #
    # entry_a → inner lane (tighter turn), entry_b → outer lane (wider turn)
    _CORNER_DEFS.append(_CornerDef(0, 0, -mx + r, -mx + r,
                                    math.pi, 1.5 * math.pi,
                                    entries=(WEST, SOUTH)))
    _CORNER_DEFS.append(_CornerDef(0, 4, mx - r, -mx + r,
                                    -math.pi / 2, 0.0,
                                    entries=(SOUTH, EAST)))
    _CORNER_DEFS.append(_CornerDef(4, 4, mx - r, mx - r,
                                    0.0, math.pi / 2,
                                    entries=(EAST, NORTH)))
    _CORNER_DEFS.append(_CornerDef(4, 0, -mx + r, mx - r,
                                    math.pi / 2, math.pi,
                                    entries=(NORTH, WEST)))

_build_corner_defs()

_CORNER_MAP = {(cd.row, cd.col): cd for cd in _CORNER_DEFS}
_CORNER_SET = set(_CORNER_MAP.keys())

# ---------------------------------------------------------------------------
# Intersection cell definitions
# ---------------------------------------------------------------------------
# Maps (row, col) → set of blocked directions (wall/fence sides).
# A maneuver whose exit direction is in the blocked set is impossible.
INTERSECTION_CELLS: dict[tuple[int, int], set[tuple[int, int]]] = {
    (2, 2): set(),       # 4-way cross, open on all sides
    (0, 2): {SOUTH},     # T-junction, fence on south
    (4, 2): {NORTH},     # T-junction, fence on north
    (2, 0): {WEST},      # T-junction, fence on west
    (2, 4): {EAST},      # T-junction, fence on east
}
_INTERSECTION_SET = set(INTERSECTION_CELLS.keys())

_LEFT_OF = {
    NORTH: WEST,
    SOUTH: EAST,
    EAST:  NORTH,
    WEST:  SOUTH,
}

_OPPOSITE = {
    NORTH: SOUTH,
    SOUTH: NORTH,
    EAST:  WEST,
    WEST:  EAST,
}


def _arc_polyline(cx: float, cy: float, r: float,
                  a0: float, a1: float, n: int = _ARC_N
                  ) -> list[tuple[float, float, float]]:
    pts = []
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a), _Z))
    return pts


# ---------------------------------------------------------------------------
# Straight lane geometry
# ---------------------------------------------------------------------------

def _straight_lane_lines(
    cx: float, cy: float,
    travel_dir: tuple[int, int],
) -> tuple[list[tuple], list[tuple]]:
    """Return (inner_edge, outer_edge) polylines spanning the full cell."""
    dr, dc = travel_dir
    right_dir = _RIGHT_OF[travel_dir]

    half_lane = LANE_W / 2.0
    inner_off_x = right_dir[1] * (LANE_CENTER - half_lane)
    inner_off_y = right_dir[0] * (LANE_CENTER - half_lane)
    outer_off_x = right_dir[1] * (LANE_CENTER + half_lane)
    outer_off_y = right_dir[0] * (LANE_CENTER + half_lane)

    tvx = float(dc)
    tvy = float(dr)
    half_cell = CELL / 2.0

    s_x = cx - tvx * half_cell
    s_y = cy - tvy * half_cell
    e_x = cx + tvx * half_cell
    e_y = cy + tvy * half_cell

    inner = [
        (s_x + inner_off_x, s_y + inner_off_y, _Z),
        (e_x + inner_off_x, e_y + inner_off_y, _Z),
    ]
    outer = [
        (s_x + outer_off_x, s_y + outer_off_y, _Z),
        (e_x + outer_off_x, e_y + outer_off_y, _Z),
    ]
    return inner, outer


# ---------------------------------------------------------------------------
# Corner lane geometry — arc clipped to cell boundaries
# ---------------------------------------------------------------------------

def _corner_lane_full(
    cdef: _CornerDef, entry_dir: tuple[int, int]
) -> tuple[list[tuple], list[tuple]]:
    """Return (inner, outer) polylines for corner lane clipped to cell edges.

    At ang0 and ang1 the arc endpoints sit on an axis-aligned line.
    We extend each endpoint along that line to the cell edge.

    At ang0 = k*π/2 the radius is axis-aligned:
      cos(ang)=±1, sin=0  → point offset in X → road is vertical → extend Y
      cos=0, sin(ang)=±1  → point offset in Y → road is horizontal → extend X

    The extension goes AWAY from the arc centre (toward the cell interior).
    """
    row, col = cdef.row, cdef.col
    cell_x0 = -_HALF + col * CELL
    cell_x1 = cell_x0 + CELL
    cell_y0 = -_HALF + row * CELL
    cell_y1 = cell_y0 + CELL

    acx, acy = cdef.arc_cx, cdef.arc_cy

    # ext edges: toward the arc centre (away from polygon corner)
    ext_x = cell_x1 if acx > (cell_x0 + cell_x1) / 2 else cell_x0
    ext_y = cell_y1 if acy > (cell_y0 + cell_y1) / 2 else cell_y0
    # corner point: the polygon corner (opposite to arc centre)
    corner_x = cell_x0 if ext_x == cell_x1 else cell_x1
    corner_y = cell_y0 if ext_y == cell_y1 else cell_y1

    def _extend_to_edge(r, ang):
        px = acx + r * math.cos(ang)
        py = acy + r * math.sin(ang)
        ca = round(math.cos(ang))
        if ca != 0:
            return (px, ext_y, _Z)
        else:
            return (ext_x, py, _Z)

    if entry_dir == cdef.entries[0]:
        # Inner lane: outer edge = dashed centre-line arc,
        # inner edge = two straights meeting at the polygon corner.
        r_outer = CORNER_R - MARK_W / 2.0

        outer_arc = _arc_polyline(acx, acy, r_outer, cdef.ang0, cdef.ang1)
        outer = [_extend_to_edge(r_outer, cdef.ang0)] + outer_arc + \
                [_extend_to_edge(r_outer, cdef.ang1)]

        # Inner edge: two straights meeting at the inner road corner
        # (ext_x, ext_y) — the cell corner on the arc-centre side.
        p_ang0 = _extend_to_edge(0.0, cdef.ang0)
        p_ang1 = _extend_to_edge(0.0, cdef.ang1)
        inner = [p_ang0, (ext_x, ext_y, _Z), p_ang1]
    else:
        # Outer lane: inner edge = dashed centre-line
        r_inner = CORNER_R + MARK_W / 2.0
        r_outer = r_inner + LANE_W

        inner_arc = _arc_polyline(acx, acy, r_inner, cdef.ang0, cdef.ang1)
        outer_arc = _arc_polyline(acx, acy, r_outer, cdef.ang0, cdef.ang1)

        inner = [_extend_to_edge(r_inner, cdef.ang0)] + inner_arc + \
                [_extend_to_edge(r_inner, cdef.ang1)]
        outer = [_extend_to_edge(r_outer, cdef.ang0)] + outer_arc + \
                [_extend_to_edge(r_outer, cdef.ang1)]

    return inner, outer


# ---------------------------------------------------------------------------
# Intersection maneuver geometry
# ---------------------------------------------------------------------------
# At an intersection the two centre-lines cross at the cell centre (cx, cy).
# Turns use the same CORNER_R as corner cells but the arc centre is placed
# relative to (cx, cy).
#
# Direction convention for arcs:
#   entry_dir  = direction the robot travels INTO the cell (delta prev→this)
#   exit_dir   = direction the robot leaves the cell
#   right_dir  = _RIGHT_OF[entry_dir]
#   left_dir   = _LEFT_OF[entry_dir]
#
# travel vector: (dc, dr) in world XY for direction (dr, dc) in grid coords.

def _dir_vec(d: tuple[int, int]) -> tuple[float, float]:
    """Convert grid direction (dr, dc) to world XY unit vector."""
    return (float(d[1]), float(d[0]))


# Pre-computed arc parameters for intersection turns.
# Key: (entry_dir, exit_dir)
# Value: (sign_x, sign_y, ang0, ang1)
#   arc centre = (cx + sign_x * MARK_W/2, cy + sign_y * MARK_W/2)
#   ang0 = angle of the radius arm on the entry side
#   ang1 = angle of the radius arm on the exit side
_RIGHT_TURN_ARC: dict[tuple, tuple] = {
    (EAST,  SOUTH): (+1, -1, math.pi,        1.5 * math.pi),
    (SOUTH, WEST):  (-1, -1, math.pi / 2,    math.pi),
    (WEST,  NORTH): (-1, +1, 0.0,            math.pi / 2),
    (NORTH, EAST):  (+1, +1, -math.pi / 2,   0.0),
}

_LEFT_TURN_ARC: dict[tuple, tuple] = {
    (EAST,  NORTH): (+1, +1, math.pi,        math.pi / 2),
    (NORTH, WEST):  (-1, +1, -math.pi / 2,   -math.pi),
    (WEST,  SOUTH): (-1, -1, 0.0,            -math.pi / 2),
    (SOUTH, EAST):  (+1, -1, math.pi / 2,    0.0),
}


def _intersection_turn(
    cx: float, cy: float,
    cell: tuple[int, int],
    entry_dir: tuple[int, int],
    exit_dir: tuple[int, int],
    is_right: bool,
) -> tuple[list[tuple], list[tuple]]:
    """Turn at intersection (right or left).

    Right turn → inner lane geometry (tight arc, triangle inner edge).
    Left turn  → outer lane geometry (wide arc, both edges are arcs).

    Extensions go along the road arms from arc endpoints to cell edges.
    """
    table = _RIGHT_TURN_ARC if is_right else _LEFT_TURN_ARC
    sx, sy, ang0, ang1 = table[(entry_dir, exit_dir)]

    acx = cx + sx * MARK_W / 2.0
    acy = cy + sy * MARK_W / 2.0

    row, col = cell
    cell_x0 = -_HALF + col * CELL
    cell_x1 = cell_x0 + CELL
    cell_y0 = -_HALF + row * CELL
    cell_y1 = cell_y0 + CELL

    # Entry and exit travel vectors in world XY
    entry_vx, entry_vy = _dir_vec(entry_dir)
    exit_vx,  exit_vy  = _dir_vec(exit_dir)
    half_cell = CELL / 2.0

    # Entry cell edge coordinate along travel axis
    entry_edge_x = cx - entry_vx * half_cell
    entry_edge_y = cy - entry_vy * half_cell
    # Exit cell edge coordinate along travel axis
    exit_edge_x = cx + exit_vx * half_cell
    exit_edge_y = cy + exit_vy * half_cell

    def _extend_entry(r):
        """Extend arc point at ang0 back to the entry cell edge."""
        px = acx + r * math.cos(ang0)
        py = acy + r * math.sin(ang0)
        if abs(entry_vx) > 0.5:
            return (entry_edge_x, py, _Z)
        else:
            return (px, entry_edge_y, _Z)

    def _extend_exit(r):
        """Extend arc point at ang1 out to the exit cell edge."""
        px = acx + r * math.cos(ang1)
        py = acy + r * math.sin(ang1)
        if abs(exit_vx) > 0.5:
            return (exit_edge_x, py, _Z)
        else:
            return (px, exit_edge_y, _Z)

    if is_right:
        r_outer = CORNER_R - MARK_W / 2.0
        outer_arc = _arc_polyline(acx, acy, r_outer, ang0, ang1)
        outer = [_extend_entry(r_outer)] + outer_arc + [_extend_exit(r_outer)]

        p0 = _extend_entry(0.0)
        p1 = _extend_exit(0.0)
        # Inner corner: the cell corner in the turn quadrant
        corner_x = cx + sx * half_cell
        corner_y = cy + sy * half_cell
        inner = [p0, (corner_x, corner_y, _Z), p1]
    else:
        r_inner = CORNER_R + MARK_W / 2.0
        r_outer = r_inner + LANE_W

        inner_arc = _arc_polyline(acx, acy, r_inner, ang0, ang1)
        outer_arc = _arc_polyline(acx, acy, r_outer, ang0, ang1)

        inner = [_extend_entry(r_inner)] + inner_arc + [_extend_exit(r_inner)]
        outer = [_extend_entry(r_outer)] + outer_arc + [_extend_exit(r_outer)]

    return inner, outer


def _intersection_uturn(
    cx: float, cy: float,
    entry_dir: tuple[int, int],
) -> tuple[list[tuple], list[tuple]]:
    """U-turn at intersection — 180deg semicircle in the right lane.

    The robot enters on the right lane, drives to the centre-line, makes a
    semicircle, and exits on the opposite lane of the same road (now heading
    the opposite direction).

    The semicircle centre sits on the centre-line perpendicular to travel,
    at the cell centre.  Inner radius = MARK_W/2 (hugs the centre-line),
    outer radius = MARK_W/2 + LANE_W.
    """
    right_dir = _RIGHT_OF[entry_dir]
    evx, evy = _dir_vec(entry_dir)
    rvx, rvy = _dir_vec(right_dir)

    # Semicircle centre: on the lateral centre-line, at cell centre
    scx = cx
    scy = cy

    # The semicircle spans from entry heading to opposite heading (180deg)
    # in the right-hand half of the road.
    # Start angle: pointing in the right direction (toward the entry lane)
    ang_start = math.atan2(rvy, rvx)
    # End angle: pointing in the opposite-right direction (toward the exit lane)
    ang_end = ang_start - math.pi  # CW sweep of 180deg

    r_inner = MARK_W / 2.0
    r_outer = MARK_W / 2.0 + LANE_W

    inner_arc = _arc_polyline(scx, scy, r_inner, ang_start, ang_end)
    outer_arc = _arc_polyline(scx, scy, r_outer, ang_start, ang_end)

    half_cell = CELL / 2.0
    half_lane = LANE_W / 2.0
    entry_edge_x = cx - evx * half_cell
    entry_edge_y = cy - evy * half_cell

    # Inner edge endpoints: centre-line side of entry and exit lanes
    entry_inner_x = entry_edge_x + rvx * (LANE_CENTER - half_lane)
    entry_inner_y = entry_edge_y + rvy * (LANE_CENTER - half_lane)
    exit_inner_x  = entry_edge_x - rvx * (LANE_CENTER - half_lane)
    exit_inner_y  = entry_edge_y - rvy * (LANE_CENTER - half_lane)

    # Outer edge endpoints: curb side of entry and exit lanes
    entry_outer_x = entry_edge_x + rvx * (LANE_CENTER + half_lane)
    entry_outer_y = entry_edge_y + rvy * (LANE_CENTER + half_lane)
    exit_outer_x  = entry_edge_x - rvx * (LANE_CENTER + half_lane)
    exit_outer_y  = entry_edge_y - rvy * (LANE_CENTER + half_lane)

    inner = ([(entry_inner_x, entry_inner_y, _Z)] +
             inner_arc +
             [(exit_inner_x, exit_inner_y, _Z)])

    outer = ([(entry_outer_x, entry_outer_y, _Z)] +
             outer_arc +
             [(exit_outer_x, exit_outer_y, _Z)])

    return inner, outer


def compute_intersection_maneuvers(
    cell: tuple[int, int],
    entry_dir: tuple[int, int],
) -> list[str]:
    """Return names of valid maneuvers at an intersection for given entry.

    Returns a list of maneuver names: 'straight', 'right', 'left', 'uturn'.
    """
    walls = INTERSECTION_CELLS.get(cell, set())
    result: list[str] = []

    if _OPPOSITE[entry_dir] not in walls:
        result.append("straight")
    if _RIGHT_OF[entry_dir] not in walls:
        result.append("right")
    if _LEFT_OF[entry_dir] not in walls:
        result.append("left")
    result.append("uturn")

    return result


def _intersection_union_outline(
    cell: tuple[int, int],
    entry_dir: tuple[int, int],
) -> tuple[list[tuple], list[tuple]]:
    """Build the union outline of all valid maneuvers at an intersection.

    Returns (inner, outer) polylines that together enclose the reachable area.

    "outer" walks CW along the cell boundary/curb from the entry right-curb
    to the entry left-curb.  On blocked edges it follows the cell corner
    and then the centre-line back to where the area stops.

    "inner" runs along the centre-line from entry (right side) through a
    U-turn semicircle to entry (left side).
    """
    walls = INTERSECTION_CELLS.get(cell, set())
    cx, cy = _cell_xy(*cell)
    h = CELL / 2.0

    can_right    = _RIGHT_OF[entry_dir] not in walls
    can_straight = _OPPOSITE[entry_dir] not in walls
    can_left     = _LEFT_OF[entry_dir]  not in walls

    evx, evy = _dir_vec(entry_dir)
    rvx, rvy = _dir_vec(_RIGHT_OF[entry_dir])
    mo = MARK_W / 2.0

    def _p(fx: float, fy: float) -> tuple:
        return (cx + fx, cy + fy, _Z)

    # Outer polyline: walks CW around the cell perimeter from entry
    # right-curb, through all reachable exits, to entry left-curb.
    # On each edge:
    #   valid exit → full curb-to-curb across the exit lane
    #   blocked    → cut at centre-line (robot can't cross there)
    outer: list[tuple] = []

    # Start: entry edge, right-curb
    outer.append(_p(-evx * h + rvx * h, -evy * h + rvy * h))

    # -- Between entry-right-curb and fwd-right corner --
    if can_right:
        # Right exit valid: the area extends to the curb on the right edge.
        # Walk entry-right corner → along right edge (back curb → fwd curb)
        outer.append(_p(rvx * h - evx * h, rvy * h - evy * h))
        outer.append(_p(rvx * h + evx * h, rvy * h + evy * h))
    else:
        # Right blocked: area stops at centre-line on right side.
        # Walk to centre-line on right edge, then along edge to fwd corner.
        outer.append(_p(rvx * h - evx * mo, rvy * h - evy * mo))
        outer.append(_p(rvx * h + evx * mo, rvy * h + evy * mo))
        outer.append(_p(rvx * h + evx * h, rvy * h + evy * h))

    # -- Forward edge --
    if can_straight:
        outer.append(_p(evx * h + rvx * h, evy * h + rvy * h))
        outer.append(_p(evx * h - rvx * h, evy * h - rvy * h))
    else:
        outer.append(_p(evx * h + rvx * mo, evy * h + rvy * mo))
        outer.append(_p(evx * h - rvx * mo, evy * h - rvy * mo))
        outer.append(_p(evx * h - rvx * h, evy * h - rvy * h))

    # -- Left edge --
    if can_left:
        outer.append(_p(-rvx * h + evx * h, -rvy * h + evy * h))
        outer.append(_p(-rvx * h - evx * h, -rvy * h - evy * h))
    else:
        outer.append(_p(-rvx * h + evx * mo, -rvy * h + evy * mo))
        outer.append(_p(-rvx * h - evx * mo, -rvy * h - evy * mo))
        outer.append(_p(-rvx * h - evx * h, -rvy * h - evy * h))

    # End: entry edge, left-curb
    outer.append(_p(-evx * h - rvx * h, -evy * h - rvy * h))

    # Inner polyline: centre-line from entry right to U-turn to entry left.
    inner: list[tuple] = []
    inner.append(_p(-evx * h + rvx * mo, -evy * h + rvy * mo))
    ang_s = math.atan2(rvy, rvx)
    inner.extend(_arc_polyline(cx, cy, mo, ang_s, ang_s - math.pi, n=10))
    inner.append(_p(-evx * h - rvx * mo, -evy * h - rvy * mo))

    return inner, outer


# ---------------------------------------------------------------------------
# Lane tracker
# ---------------------------------------------------------------------------

class LaneTracker:
    """Tracks which cell the robot is in and which lane to highlight.

    The lane is only shown if the robot entered the cell within the correct
    lane band on the entry edge. The highlight persists until the robot leaves.
    """

    def __init__(self):
        self._current_cell: tuple[int, int] | None = None
        self._travel_dir: tuple[int, int] | None = None
        self._active: bool = False   # True if robot entered via correct lane band

    def update(self, rx: float, ry: float, ryaw: float
               ) -> list[tuple[list[tuple], list[tuple]]]:
        cell = _cell_of(rx, ry)
        if cell is None or cell in BUILDING_CELLS:
            self._current_cell = None
            self._travel_dir = None
            self._active = False
            return []

        if cell != self._current_cell:
            # Cell transition
            if self._current_cell is not None:
                dr = cell[0] - self._current_cell[0]
                dc = cell[1] - self._current_cell[1]
                if (dr, dc) in _ALL_DIRS:
                    entry_dir = (dr, dc)
                else:
                    entry_dir = _yaw_to_dir(ryaw)
                self._current_cell = cell
                self._travel_dir = entry_dir
                self._active = _robot_in_lane_band(rx, ry, cell, entry_dir)
            else:
                # First frame / spawn — detect which lane the robot is actually in.
                entry_dir = _detect_spawn_lane(rx, ry, ryaw, cell)
                self._current_cell = cell
                self._travel_dir = entry_dir
                self._active = entry_dir is not None

        if not self._active:
            return []

        if self._travel_dir is None:
            return []

        cx, cy = _cell_xy(cell[0], cell[1])

        if cell in _INTERSECTION_SET:
            inner, outer = _intersection_union_outline(cell, self._travel_dir)
            return [(inner, outer)]

        if cell in _CORNER_SET:
            cdef = _CORNER_MAP[cell]
            if self._travel_dir in cdef.entries:
                inner, outer = _corner_lane_full(cdef, self._travel_dir)
            else:
                return []
        else:
            inner, outer = _straight_lane_lines(cx, cy, self._travel_dir)

        return [(inner, outer)]


def _detect_spawn_lane(
    rx: float, ry: float, ryaw: float, cell: tuple[int, int]
) -> tuple[int, int] | None:
    """At spawn, detect which lane the robot is actually in.

    Tries the yaw-derived direction first. If that doesn't match,
    iterates over all valid directions for the cell and picks one
    where the robot falls within the right-hand lane band.
    Returns None if no valid lane found.
    """
    yaw_dir = _yaw_to_dir(ryaw)

    if cell in _CORNER_SET:
        candidates = list(_CORNER_MAP[cell].entries)
    else:
        candidates = list(_ALL_DIRS)

    # Prioritise the yaw direction
    if yaw_dir in candidates:
        candidates.remove(yaw_dir)
        candidates.insert(0, yaw_dir)

    for d in candidates:
        if _robot_in_lane_band(rx, ry, cell, d):
            return d
    return None


def _yaw_to_dir(yaw: float) -> tuple[int, int]:
    if -math.pi / 4 <= yaw < math.pi / 4:
        return EAST
    elif math.pi / 4 <= yaw < 3 * math.pi / 4:
        return NORTH
    elif -3 * math.pi / 4 <= yaw < -math.pi / 4:
        return SOUTH
    else:
        return WEST
