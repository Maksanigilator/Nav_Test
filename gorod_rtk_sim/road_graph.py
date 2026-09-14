"""Road graph for the 5x5 city polygon.

The polygon is a 5x5 grid where 4 cells are buildings and 21 are roads.
Nodes are placed at the centres of road cells.  Edges connect adjacent road
cells (4-connected).  Each edge is directed and can be disabled when a
traffic sign forbids that direction.

Coordinate system: origin at polygon centre, X right, Y up.
Cell (row=0, col=0) is the bottom-left corner.

This module is self-contained (no IsaacLab dependency) so it can be used
both inside the simulator and on the real robot.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Constants (must match city_builder.py)
# ---------------------------------------------------------------------------
CELL_SIZE = 0.8
GRID_N = 5
_HALF = CELL_SIZE * GRID_N / 2.0

BUILDING_CELLS = {(1, 1), (1, 3), (3, 1), (3, 3)}


def cell_center(row: int, col: int) -> tuple[float, float]:
    x = -_HALF + CELL_SIZE * col + CELL_SIZE / 2.0
    y = -_HALF + CELL_SIZE * row + CELL_SIZE / 2.0
    return (x, y)


# ---------------------------------------------------------------------------
# Direction helpers
# ---------------------------------------------------------------------------
# Directions as (drow, dcol) — "north" means row+1
NORTH = (1, 0)
SOUTH = (-1, 0)
EAST = (0, 1)
WEST = (0, -1)

DIRECTION_NAMES = {NORTH: "north", SOUTH: "south", EAST: "east", WEST: "west"}
OPPOSITE = {NORTH: SOUTH, SOUTH: NORTH, EAST: WEST, WEST: EAST}


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass
class Node:
    row: int
    col: int
    x: float
    y: float

    @property
    def id(self) -> tuple[int, int]:
        return (self.row, self.col)

    def __hash__(self):
        return hash(self.id)

    def __eq__(self, other):
        return self.id == other.id


@dataclass
class Edge:
    src: tuple[int, int]
    dst: tuple[int, int]
    cost: float = CELL_SIZE
    enabled: bool = True


@dataclass
class RoadGraph:
    """Directed graph of road cells with A* pathfinding."""

    nodes: dict[tuple[int, int], Node] = field(default_factory=dict)
    edges: dict[tuple[tuple[int, int], tuple[int, int]], Edge] = field(default_factory=dict)

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    @classmethod
    def build_default(cls) -> RoadGraph:
        """Build the full road graph for the standard 5x5 polygon."""
        g = cls()
        for r in range(GRID_N):
            for c in range(GRID_N):
                if (r, c) in BUILDING_CELLS:
                    continue
                x, y = cell_center(r, c)
                g.nodes[(r, c)] = Node(r, c, x, y)

        for nid in g.nodes:
            r, c = nid
            for dr, dc in (NORTH, SOUTH, EAST, WEST):
                nr, nc = r + dr, c + dc
                if (nr, nc) in g.nodes:
                    g.edges[(nid, (nr, nc))] = Edge(src=nid, dst=(nr, nc))
        return g

    # ------------------------------------------------------------------
    # Sign-based edge updates
    # ------------------------------------------------------------------
    def disable_edge(self, src: tuple[int, int], dst: tuple[int, int]) -> None:
        key = (src, dst)
        if key in self.edges:
            self.edges[key].enabled = False

    def enable_edge(self, src: tuple[int, int], dst: tuple[int, int]) -> None:
        key = (src, dst)
        if key in self.edges:
            self.edges[key].enabled = True

    def apply_sign(self, node_id: tuple[int, int], sign_type: str, facing_dir: tuple[int, int]) -> None:
        """Update edges based on a detected traffic sign.

        Args:
            node_id: The road cell where the sign is observed.
            sign_type: One of 'straight', 'left', 'right', 'no_left', 'no_right'.
            facing_dir: Direction the robot is traveling when it sees the sign (drow, dcol).
        """
        r, c = node_id
        # relative directions from the robot's perspective
        left = _turn_left(facing_dir)
        right = _turn_right(facing_dir)

        if sign_type == "straight":
            dst = (r + facing_dir[0], c + facing_dir[1])
            self.enable_edge(node_id, dst)
        elif sign_type == "left":
            dst = (r + left[0], c + left[1])
            self.enable_edge(node_id, dst)
        elif sign_type == "right":
            dst = (r + right[0], c + right[1])
            self.enable_edge(node_id, dst)
        elif sign_type == "no_left":
            dst = (r + left[0], c + left[1])
            self.disable_edge(node_id, dst)
        elif sign_type == "no_right":
            dst = (r + right[0], c + right[1])
            self.disable_edge(node_id, dst)

    # ------------------------------------------------------------------
    # A* pathfinding
    # ------------------------------------------------------------------
    def find_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]] | None:
        """A* shortest path from *start* to *goal*.

        Returns a list of node ids (including start and goal) or None if
        no path exists.
        """
        if start not in self.nodes or goal not in self.nodes:
            return None

        goal_node = self.nodes[goal]
        open_set: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return _reconstruct(came_from, current)

            for edge in self._outgoing(current):
                if not edge.enabled:
                    continue
                tentative = g_score[current] + edge.cost
                if tentative < g_score.get(edge.dst, float("inf")):
                    came_from[edge.dst] = current
                    g_score[edge.dst] = tentative
                    f = tentative + _heuristic(self.nodes[edge.dst], goal_node)
                    heapq.heappush(open_set, (f, edge.dst))

        return None

    def path_to_world_coords(self, path: list[tuple[int, int]]) -> list[tuple[float, float]]:
        """Convert a node-id path to a list of (x, y) world coordinates."""
        return [(self.nodes[nid].x, self.nodes[nid].y) for nid in path]

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _outgoing(self, node_id: tuple[int, int]) -> list[Edge]:
        return [e for (s, _), e in self.edges.items() if s == node_id]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _heuristic(a: Node, b: Node) -> float:
    return abs(a.x - b.x) + abs(a.y - b.y)


def _reconstruct(came_from: dict, current: tuple[int, int]) -> list[tuple[int, int]]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _turn_left(d: tuple[int, int]) -> tuple[int, int]:
    """90-degree left turn: (dr, dc) -> (dc, -dr)."""
    return (d[1], -d[0])


def _turn_right(d: tuple[int, int]) -> tuple[int, int]:
    """90-degree right turn: (dr, dc) -> (-dc, dr)."""
    return (-d[1], d[0])


# ---------------------------------------------------------------------------
# Quick self-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    g = RoadGraph.build_default()
    print(f"Road graph: {len(g.nodes)} nodes, {len(g.edges)} edges")

    start = (0, 0)
    goal = (4, 4)
    path = g.find_path(start, goal)
    if path:
        coords = g.path_to_world_coords(path)
        print(f"Path {start} -> {goal}: {path}")
        print(f"World coords: {coords}")
    else:
        print(f"No path from {start} to {goal}")

    # test sign application
    g.apply_sign((2, 2), "no_right", NORTH)
    path2 = g.find_path(start, goal)
    print(f"After blocking right at (2,2) facing north: {path2}")
