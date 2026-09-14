"""Procedural generation of the 'City Driving' competition polygon for IsaacLab.

Grid layout (5x5, row 0 = bottom, col 0 = left):

    (4,0) (4,1) (4,2) (4,3) (4,4)
    (3,0) [3,1] (3,2) [3,3] (3,4)
    (2,0) (2,1) (2,2) (2,3) (2,4)
    (1,0) [1,1] (1,2) [1,3] (1,4)
    (0,0) (0,1) (0,2) (0,3) (0,4)

    [x,y] = building cells on pedestals
    (x,y) = road cells

Marking types (distinct colours for camera / colour projection):
    SOLID     — stop lines before crosswalks
    DASHED    — outer rounded loop + radial arms toward the centre
    CROSSWALK — zebra stripes between blocks

Marking layout:
    - DASHED: outer loop on the **centre of the perimeter road lane** (not on
      the field edge), plus four radial arms to the inner ring.
    - CROSSWALK: shifted toward (2,2); stripes **⊥ pedestrian path** between facing
      pedestals (not along the auto corridor only).
    - SOLID: one line **perpendicular to the lane**, **before** the zebra when
      driving toward the central intersection.
"""

from __future__ import annotations

import math

import isaaclab.sim as sim_utils
from isaaclab.sim import schemas
from isaaclab.assets import AssetBaseCfg

# ---------------------------------------------------------------------------
# Dimensions (metres) from competition regulations
# ---------------------------------------------------------------------------
CELL_SIZE = 0.8
GRID_N = 5
POLYGON_SIZE = CELL_SIZE * GRID_N  # 4.0 m
_HALF = POLYGON_SIZE / 2.0         # 2.0 m

ROAD_THICKNESS = 0.005
ROAD_COLOR = (0.22, 0.22, 0.22)

MW = 0.04       # marking line width
MT = 0.001      # marking thickness (visual only)

# Three distinct colours for segmentation
COLOR_SOLID     = (0.95, 0.95, 0.95)   # white — solid centre-line
COLOR_DASHED    = (0.95, 0.95, 0.60)   # pale yellow — dashed perimeter
COLOR_CROSSWALK = (0.60, 0.85, 0.95)   # pale blue — crosswalk bars

DASH_LEN = 0.20
DASH_GAP = 0.14
DASH_PITCH = DASH_LEN + DASH_GAP

# Rounded corners on outer loop (metres)
CORNER_R = 0.30

# Gap between inner (intersection-facing) cell edge and crosswalk stripe.
CROSSWALK_EDGE_GAP_INNER = 0.02
# Shift zebra toward central junction (toward +x / −x / +y / −y as appropriate)
CROSSWALK_SHIFT_IN = CELL_SIZE / 2.0 - CROSSWALK_EDGE_GAP_INNER - DASH_LEN / 2.0


def _outer_ring_midline() -> tuple[float, float, float, float]:
    """Axis coordinates of the outer lane **centreline** (mid of row 0 / 4 / col 0 / 4)."""
    m = _HALF - CELL_SIZE / 2.0
    return (-m, m, -m, m)


def _cell_edge_bot(row: int) -> float:
    return -_HALF + CELL_SIZE * row


def _cell_edge_top(row: int) -> float:
    return -_HALF + CELL_SIZE * (row + 1)


def _cell_edge_left(col: int) -> float:
    return -_HALF + CELL_SIZE * col


def _cell_edge_right(col: int) -> float:
    return -_HALF + CELL_SIZE * (col + 1)

# Zebra: thick bars; pack spans **lateral** (curb-to-curb).
CROSSWALK_BAR_W = 0.045
CROSSWALK_N = 7

# Solid stop lines before crossings
STOP_LINE_DEPTH = 0.028

# Tiny gap so radial dashes do not z-fight with perimeter at side midpoints
ARM_JOIN_EPS = 0.07

PEDESTAL_H = 0.08
PEDESTAL_COLOR = (0.30, 0.55, 0.25)

# --- Simple rectangle footprint (top-right building, cell (3,3)) ---
# Used as the width/depth of the single cuboid for the «обычный дом».
BUILDING_BASE = 0.40
BUILDING_H = 0.50
BUILDING_COLOR = (0.55, 0.45, 0.35)

# Regulation-style RGB tints (П / прямоугольник / крест / Г)
COL_BLD_P = (0.10, 0.32, 0.14)       # NW (3,1) — буква П (тёмно-зелёный)
COL_BLD_RECT = (0.40, 0.40, 0.44)    # NE (3,3) — прямоугольник (размеры: BUILDING_BASE × BUILDING_BASE × BUILDING_H)
COL_BLD_PLUS = (0.92, 0.88, 0.62)  # SW (1,1) — центр + выступы
COL_BLD_G = (0.30, 0.62, 0.58)      # SE (1,3) — буква Г

CURB_H = 0.025
CURB_W = 0.02
CURB_COLOR = (0.65, 0.65, 0.60)

# Lateral pack (between inner curbs); outer stripe centres span this width.
CROSSWALK_LATERAL = CELL_SIZE - 2.0 * CURB_W

FENCE_H = 0.30
FENCE_THICKNESS = 0.005
FENCE_COLOR = (0.90, 0.90, 0.88)

SIGN_POLE_R = 0.008
SIGN_POLE_H = 0.15
SIGN_DISC_R = 0.05
SIGN_COLORS = {
    "straight": (0.15, 0.35, 0.85),
    "left":     (0.15, 0.35, 0.85),
    "right":    (0.15, 0.35, 0.85),
    "no_left":  (0.85, 0.15, 0.15),
    "no_right": (0.85, 0.15, 0.15),
    "stop":     (0.85, 0.15, 0.15),
    "parking":  (0.15, 0.35, 0.85),
    "danger":   (0.85, 0.65, 0.05),
}

BUILDING_CELLS = [(1, 1), (1, 3), (3, 1), (3, 3)]
_BUILDING_SET = set(BUILDING_CELLS)


def _cell_xy(row: int, col: int) -> tuple[float, float]:
    """World XY of cell centre. Row 0 / col 0 is bottom-left."""
    x = -_HALF + CELL_SIZE * col + CELL_SIZE / 2.0
    y = -_HALF + CELL_SIZE * row + CELL_SIZE / 2.0
    return (x, y)


def _mat(color, roughness=0.8):
    return sim_utils.PreviewSurfaceCfg(
        diffuse_color=color, roughness=roughness, metallic=0.0,
    )


def _quat_z(angle_rad: float) -> tuple[float, float, float, float]:
    c = math.cos(angle_rad / 2)
    s = math.sin(angle_rad / 2)
    return (c, 0.0, 0.0, s)


def _cuboid(bp, name, size, pos, color, roughness=0.8, rot=None, physical=False):
    state = AssetBaseCfg.InitialStateCfg(pos=pos)
    if rot is not None:
        state = AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot)
    spawn_kwargs = dict(
        size=size,
        visual_material=_mat(color, roughness),
    )
    if physical:
        spawn_kwargs["rigid_props"] = schemas.RigidBodyPropertiesCfg(
            kinematic_enabled=True,
        )
        spawn_kwargs["collision_props"] = schemas.CollisionPropertiesCfg(
            collision_enabled=True,
        )
    # Global collision group: shared city prims must hit every env's robot while
    # InteractiveScene filter_collisions disables robot↔robot across env clones.
    acfg = dict(
        prim_path=f"{bp}/{name}",
        spawn=sim_utils.CuboidCfg(**spawn_kwargs),
        init_state=state,
    )
    if physical:
        acfg["collision_group"] = -1
    return AssetBaseCfg(**acfg)


def _bld_z_center() -> float:
    return ROAD_THICKNESS + PEDESTAL_H + BUILDING_H / 2.0


def _append_regulation_building(
    a: dict,
    bp: str,
    idx: int,
    r: int,
    c: int,
    cx: float,
    cy: float,
) -> None:
    """Spawn regulation footprints from composite cuboids (one cell each).

    BUILDING_CELLS order: (1,1) SW plus, (1,3) SE Г, (3,1) NW П, (3,3) NE rectangle.
    """
    z = _bld_z_center()
    bh = BUILDING_H

    def part(name: str, size: tuple[float, float, float], px: float, py: float, color: tuple[float, float, float]):
        a[f"bld_{idx}_{name}"] = _cuboid(
            bp, f"bld_{idx}_{name}",
            size=size,
            pos=(px, py, z),
            color=color, roughness=0.78, physical=True,
        )

    # NE (3,3): прямоугольник — только BUILDING_BASE / BUILDING_H (см. константы выше)
    if (r, c) == (3, 3):
        part("rect", (BUILDING_BASE, BUILDING_BASE, bh), cx, cy, COL_BLD_RECT)
        return

    # NW (3,1): П — толстая «крышка» (перекладина по +Y) + ноги к перекрёстку (−Y)
    if (r, c) == (3, 1):
        t_top = 0.17  # толщина перекладины по Y (была ~0.10)
        t_leg = 0.10
        part("p_top", (0.38, t_top, bh), cx, cy + 0.15, COL_BLD_P)
        part("p_leg_l", (t_leg, 0.24, bh), cx - 0.135, cy - 0.04, COL_BLD_P)
        part("p_leg_r", (t_leg, 0.24, bh), cx + 0.135, cy - 0.04, COL_BLD_P)
        return

    # SW (1,1): крест — горизонтальная палка (E–W) длиннее поперечника: толщина N–S бруса
    # по X = 80% длины горизонтали, чтобы боковые концы горизонтали чуть выступали.
    if (r, c) == (1, 1):
        arm_t = 0.10
        span_ew = 0.36
        thick_ns = 0.8 * span_ew
        part("x_bar_h", (span_ew, arm_t, bh), cx, cy, COL_BLD_PLUS)
        part("x_bar_v", (thick_ns, span_ew, bh), cx, cy, COL_BLD_PLUS)
        return

    # SE (1,3): Г — поворот на 180° относительно варианта «стойка слева, перекладина сверху»
    if (r, c) == (1, 3):
        part("g_vert", (0.12, 0.30, bh), cx + 0.12, cy - 0.02, COL_BLD_G)
        part("g_horz", (0.28, 0.12, bh), cx - 0.05, cy - 0.15, COL_BLD_G)
        return


# ===================================================================
# Marking helpers
# ===================================================================

def _add_dashes_h(a, bp, tag, x0, x1, cy, mz):
    """Horizontal dashed line from x0 to x1 (increasing x)."""
    if x1 < x0:
        x0, x1 = x1, x0
    length = x1 - x0
    if length < 0.02:
        return
    pitch = DASH_PITCH
    n = max(1, int(length / pitch))
    s0 = x0 + (length - (n - 1) * pitch) / 2
    for k in range(n):
        dx = s0 + k * pitch
        a[f"dsh_{tag}_{k}"] = _cuboid(
            bp, f"dsh_{tag}_{k}",
            size=(min(DASH_LEN, length * 0.45), MW, MT), pos=(dx, cy, mz),
            color=COLOR_DASHED, roughness=0.9,
        )


def _add_dashes_v(a, bp, tag, cx, y0, y1, mz):
    """Vertical dashed line from y0 to y1 (increasing y)."""
    if y1 < y0:
        y0, y1 = y1, y0
    length = y1 - y0
    if length < 0.02:
        return
    pitch = DASH_PITCH
    n = max(1, int(length / pitch))
    s0 = y0 + (length - (n - 1) * pitch) / 2
    for k in range(n):
        dy = s0 + k * pitch
        a[f"dsh_{tag}_{k}"] = _cuboid(
            bp, f"dsh_{tag}_{k}",
            size=(MW, min(DASH_LEN, length * 0.45), MT), pos=(cx, dy, mz),
            color=COLOR_DASHED, roughness=0.9,
        )


def _add_dashes_arc(
    a,
    bp,
    tag: str,
    cx: float,
    cy: float,
    radius: float,
    ang0: float,
    ang1: float,
    mz: float,
) -> None:
    """Dashes along a circular arc from ang0 to ang1 (radians, CCW)."""
    span = ang1 - ang0
    if span <= 0:
        return
    arc_len = radius * span
    n = max(3, int(arc_len / DASH_PITCH))
    delta_theta = span / n
    for k in range(n):
        theta = ang0 + (k + 0.5) * delta_theta
        px = cx + radius * math.cos(theta)
        py = cy + radius * math.sin(theta)
        tangent = theta + math.pi / 2
        seg_len = max(0.02, 2 * radius * math.sin(delta_theta / 2))
        a[f"dsh_{tag}_a{k}"] = _cuboid(
            bp, f"dsh_{tag}_a{k}",
            size=(seg_len, MW, MT),
            pos=(px, py, mz),
            color=COLOR_DASHED,
            roughness=0.9,
            rot=_quat_z(tangent),
        )


def _rounded_rect_perimeter(a, bp, mz: float) -> None:
    """Dashed loop along mid-perimeter lane + rounded corners + extensions to field edges."""
    xl, xr, yb, yt = _outer_ring_midline()
    r = min(CORNER_R, (xr - xl) / 2 - 0.02, (yt - yb) / 2 - 0.02)

    # Bottom edge (left → right)
    _add_dashes_h(a, bp, "perim_bot", xl + r, xr - r, yb, mz)
    # Top edge
    _add_dashes_h(a, bp, "perim_top", xl + r, xr - r, yt, mz)
    # Left edge (bottom → top)
    _add_dashes_v(a, bp, "perim_left", xl, yb + r, yt - r, mz)
    # Right edge
    _add_dashes_v(a, bp, "perim_right", xr, yb + r, yt - r, mz)

    # Corners: arc centres inset by r from outer rectangle
    _add_dashes_arc(a, bp, "perim_cbl", xl + r, yb + r, r, math.pi, 1.5 * math.pi, mz)
    _add_dashes_arc(a, bp, "perim_cbr", xr - r, yb + r, r, -math.pi / 2, 0.0, mz)
    _add_dashes_arc(a, bp, "perim_ctr", xr - r, yt - r, r, 0.0, math.pi / 2, mz)
    _add_dashes_arc(a, bp, "perim_ctl", xl + r, yt - r, r, math.pi / 2, math.pi, mz)

    # Extensions from the midpoint of each perimeter side out to the field edge.
    # These represent the T-junction arms at midpoints of each side.
    edge = _HALF  # field boundary
    eps = 0.02
    # Bottom side midpoint → field bottom edge
    _add_dashes_v(a, bp, "ext_bot", 0.0, -edge + eps, yb - eps, mz)
    # Top side midpoint → field top edge
    _add_dashes_v(a, bp, "ext_top", 0.0, yt + eps, edge - eps, mz)
    # Left side midpoint → field left edge
    _add_dashes_h(a, bp, "ext_left", -edge + eps, xl - eps, 0.0, mz)
    # Right side midpoint → field right edge
    _add_dashes_h(a, bp, "ext_right", xr + eps, edge - eps, 0.0, mz)


def _radial_arms(a, bp, mz: float) -> None:
    """Four dashed spokes from mid-perimeter lane toward inner ring, stopping before crosswalks."""
    xl, xr, yb, yt = _outer_ring_midline()

    # Crosswalk zone boundaries: the crosswalks are shifted by CROSSWALK_SHIFT_IN
    # toward center and have depth DASH_LEN. The arms must stop before entering
    # the crosswalk zone.
    s = CROSSWALK_SHIFT_IN
    xw_margin = 0.04  # small gap before crosswalk

    # Cell centres for crosswalk cells
    _, cy12 = _cell_xy(1, 2)  # south crosswalk (horizontal)
    _, cy32 = _cell_xy(3, 2)  # north crosswalk (horizontal)
    cx21, _ = _cell_xy(2, 1)  # west crosswalk (vertical)
    cx23, _ = _cell_xy(2, 3)  # east crosswalk (vertical)

    # Crosswalk depth along the vehicle axis = DASH_LEN
    half_xw = DASH_LEN / 2

    # South arm: from perimeter bottom inward, stop before south crosswalk
    arm_s_end = (cy12 + s) - half_xw - xw_margin
    _add_dashes_v(a, bp, "arm_s", 0.0, yb + ARM_JOIN_EPS, arm_s_end, mz)

    # North arm: from north crosswalk end outward to perimeter top
    arm_n_start = (cy32 - s) + half_xw + xw_margin
    _add_dashes_v(a, bp, "arm_n", 0.0, arm_n_start, yt - ARM_JOIN_EPS, mz)

    # West arm: from perimeter left inward, stop before west crosswalk
    arm_w_end = (cx21 + s) - half_xw - xw_margin
    _add_dashes_h(a, bp, "arm_w", xl + ARM_JOIN_EPS, arm_w_end, 0.0, mz)

    # East arm: from east crosswalk end outward to perimeter right
    arm_e_start = (cx23 - s) + half_xw + xw_margin
    _add_dashes_h(a, bp, "arm_e", arm_e_start, xr - ARM_JOIN_EPS, 0.0, mz)


def _add_stop_line_h(a, bp, tag, cx, cy, half_width: float, mz):
    """Stop segment at constant y, long along x (for N–S carriageway)."""
    a[f"stp_{tag}"] = _cuboid(
        bp, f"stp_{tag}",
        size=(2 * half_width, STOP_LINE_DEPTH, MT),
        pos=(cx, cy, mz),
        color=COLOR_SOLID,
        roughness=0.9,
    )


def _add_stop_line_v(a, bp, tag, cx, cy, half_width: float, mz):
    """Stop segment at constant x, long along y (for E–W carriageway)."""
    a[f"stp_{tag}"] = _cuboid(
        bp, f"stp_{tag}",
        size=(STOP_LINE_DEPTH, 2 * half_width, MT),
        pos=(cx, cy, mz),
        color=COLOR_SOLID,
        roughness=0.9,
    )


def _crosswalk_pitch() -> float:
    """Centre-to-centre step so the stripe pack spans exactly CROSSWALK_LATERAL."""
    if CROSSWALK_N < 2:
        return 0.0
    return (CROSSWALK_LATERAL - CROSSWALK_BAR_W) / (CROSSWALK_N - 1)


def _add_crosswalk_h(a, bp, tag, cx, cy, mz):
    """Zebra between **west/east** pedestals: stroke length in y = DASH_LEN; pack along x over CROSSWALK_LATERAL."""
    pitch = _crosswalk_pitch()
    start_x = cx - (CROSSWALK_N - 1) * pitch / 2
    for k in range(CROSSWALK_N):
        bx = start_x + k * pitch
        a[f"xwk_{tag}_{k}"] = _cuboid(
            bp, f"xwk_{tag}_{k}",
            size=(CROSSWALK_BAR_W, DASH_LEN, MT),
            pos=(bx, cy, mz),
            color=COLOR_CROSSWALK, roughness=0.9,
        )


def _add_crosswalk_v(a, bp, tag, cx, cy, mz):
    """Zebra between **south/north** pedestals: stroke length in x = DASH_LEN; pack along y over CROSSWALK_LATERAL."""
    pitch = _crosswalk_pitch()
    along = DASH_LEN
    start_y = cy - (CROSSWALK_N - 1) * pitch / 2
    for k in range(CROSSWALK_N):
        by = start_y + k * pitch
        a[f"xwk_{tag}_{k}"] = _cuboid(
            bp, f"xwk_{tag}_{k}",
            size=(along, CROSSWALK_BAR_W, MT),
            pos=(cx, by, mz),
            color=COLOR_CROSSWALK, roughness=0.9,
        )


# ===================================================================
# Main polygon builder
# ===================================================================

def build_city_assets(base_path: str = "/World/City") -> dict[str, AssetBaseCfg]:
    """Build the full polygon: road, markings, curbs, buildings, fence."""
    a: dict[str, AssetBaseCfg] = {}
    bp = base_path
    mz = ROAD_THICKNESS + MT / 2

    # --- 1. Road surface ---
    a["road"] = _cuboid(
        bp, "road",
        size=(POLYGON_SIZE, POLYGON_SIZE, ROAD_THICKNESS),
        pos=(0.0, 0.0, ROAD_THICKNESS / 2),
        color=ROAD_COLOR,
    )

    # --- 2. Building pedestals and buildings ---
    for i, (r, c) in enumerate(BUILDING_CELLS):
        cx, cy = _cell_xy(r, c)
        a[f"ped_{i}"] = _cuboid(
            bp, f"ped_{i}",
            size=(CELL_SIZE, CELL_SIZE, PEDESTAL_H),
            pos=(cx, cy, ROAD_THICKNESS + PEDESTAL_H / 2),
            color=PEDESTAL_COLOR, physical=True,
        )
        _append_regulation_building(a, bp, i, r, c, cx, cy)

    # --- 3. Curbs around each building pedestal ---
    curb_z = ROAD_THICKNESS + CURB_H / 2
    for i, (r, c) in enumerate(BUILDING_CELLS):
        cx, cy = _cell_xy(r, c)
        hc = CELL_SIZE / 2
        for side, pos, size in [
            ("s", (cx, cy - hc, curb_z), (CELL_SIZE + CURB_W, CURB_W, CURB_H)),
            ("n", (cx, cy + hc, curb_z), (CELL_SIZE + CURB_W, CURB_W, CURB_H)),
            ("w", (cx - hc, cy, curb_z), (CURB_W, CELL_SIZE + CURB_W, CURB_H)),
            ("e", (cx + hc, cy, curb_z), (CURB_W, CELL_SIZE + CURB_W, CURB_H)),
        ]:
            name = f"curb_{i}_{side}"
            a[name] = _cuboid(bp, name, size=size, pos=pos, color=CURB_COLOR, roughness=0.7, physical=True)

    # --- 4. Lane markings (reference layout) ---
    _rounded_rect_perimeter(a, bp, mz)
    _radial_arms(a, bp, mz)

    # Slightly higher Z so stripes / stops draw above base dashed layer
    mz_xw = mz + 0.0006
    mz_st = mz + 0.0010

    half_across = CROSSWALK_LATERAL / 2
    stop_tail = STOP_LINE_DEPTH / 2 + 0.034
    # Stripe depth along vehicle axis equals DASH_LEN on all four approaches
    stop_half = DASH_LEN / 2 + stop_tail
    stop_off_ew = stop_half
    stop_off_ns = stop_half

    s = CROSSWALK_SHIFT_IN
    # (2,1)/(2,3): auto along ±x; zebra bridges **N/S** platforms (ped ⊥ y) → long bars along x
    cx21, cy21 = _cell_xy(2, 1)
    cx21 += s
    _add_crosswalk_v(a, bp, "xw_21", cx21, cy21, mz_xw)
    _add_stop_line_v(a, bp, "21", cx21 - stop_off_ew, cy21, half_across, mz_st)

    cx23, cy23 = _cell_xy(2, 3)
    cx23 -= s
    _add_crosswalk_v(a, bp, "xw_23", cx23, cy23, mz_xw)
    _add_stop_line_v(a, bp, "23", cx23 + stop_off_ew, cy23, half_across, mz_st)

    # (1,2)/(3,2): auto along ±y; zebra bridges **W/E** platforms (ped ⊥ x) → long bars along y
    cx12, cy12 = _cell_xy(1, 2)
    cy12 += s
    _add_crosswalk_h(a, bp, "xw_12", cx12, cy12, mz_xw)
    _add_stop_line_h(a, bp, "12", cx12, cy12 - stop_off_ns, half_across, mz_st)

    cx32, cy32 = _cell_xy(3, 2)
    cy32 -= s
    _add_crosswalk_h(a, bp, "xw_32", cx32, cy32, mz_xw)
    _add_stop_line_h(a, bp, "32", cx32, cy32 + stop_off_ns, half_across, mz_st)

    # --- 5. Perimeter fence ---
    fz = ROAD_THICKNESS + FENCE_H / 2
    fo = _HALF + FENCE_THICKNESS / 2
    for key, pos, size in [
        ("fence_s", (0.0, -fo, fz), (POLYGON_SIZE + 2 * FENCE_THICKNESS, FENCE_THICKNESS, FENCE_H)),
        ("fence_n", (0.0,  fo, fz), (POLYGON_SIZE + 2 * FENCE_THICKNESS, FENCE_THICKNESS, FENCE_H)),
        ("fence_w", (-fo, 0.0, fz), (FENCE_THICKNESS, POLYGON_SIZE, FENCE_H)),
        ("fence_e", ( fo, 0.0, fz), (FENCE_THICKNESS, POLYGON_SIZE, FENCE_H)),
    ]:
        a[key] = _cuboid(bp, key, size=size, pos=pos, color=FENCE_COLOR, roughness=0.6, physical=True)

    return a


# ===================================================================
# Signs — separate function so they can be reconfigured per run
# ===================================================================

_CORNER_OFFSETS = {
    "NE": ( CELL_SIZE / 2 - 0.05,  CELL_SIZE / 2 - 0.05),
    "NW": (-CELL_SIZE / 2 + 0.05,  CELL_SIZE / 2 - 0.05),
    "SE": ( CELL_SIZE / 2 - 0.05, -CELL_SIZE / 2 + 0.05),
    "SW": (-CELL_SIZE / 2 + 0.05, -CELL_SIZE / 2 + 0.05),
}

DEFAULT_SIGNS = [
    (0, "NE", "right",    0.0),
    (0, "SE", "straight", math.pi / 2),
    (1, "NW", "left",     math.pi),
    (1, "SW", "straight", -math.pi / 2),
    (2, "NE", "straight", math.pi / 2),
    (2, "NW", "stop",     math.pi),
    (3, "SE", "parking",  0.0),
    (3, "SW", "straight", -math.pi / 2),
]


def build_sign_assets(base_path: str = "/World/City") -> dict[str, AssetBaseCfg]:
    """Build traffic sign assets (pole + coloured disc each)."""
    a: dict[str, AssetBaseCfg] = {}
    bp = base_path

    for i, (bld_idx, corner, stype, facing) in enumerate(DEFAULT_SIGNS):
        r, c = BUILDING_CELLS[bld_idx]
        cx, cy = _cell_xy(r, c)
        dx, dy = _CORNER_OFFSETS[corner]
        sx, sy = cx + dx, cy + dy
        base_z = ROAD_THICKNESS + PEDESTAL_H

        a[f"spole_{i}"] = AssetBaseCfg(
            prim_path=f"{bp}/spole_{i}",
            spawn=sim_utils.CylinderCfg(
                radius=SIGN_POLE_R,
                height=SIGN_POLE_H,
                visual_material=_mat((0.5, 0.5, 0.5), roughness=0.3),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(sx, sy, base_z + SIGN_POLE_H / 2),
            ),
        )

        color = SIGN_COLORS.get(stype, (0.5, 0.5, 0.5))
        a[f"sdisc_{i}"] = AssetBaseCfg(
            prim_path=f"{bp}/sdisc_{i}",
            spawn=sim_utils.SphereCfg(
                radius=SIGN_DISC_R,
                visual_material=_mat(color, roughness=0.4),
            ),
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(sx, sy, base_z + SIGN_POLE_H + SIGN_DISC_R),
                rot=_quat_z(facing),
            ),
        )

    return a
