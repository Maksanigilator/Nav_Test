#!/usr/bin/env python3
"""Генератор лабиринтов для Gazebo Classic.

Строит случайный ИДЕАЛЬНЫЙ лабиринт (между любыми двумя точками
ровно один путь) алгоритмом recursive backtracker и сохраняет его
в формате SDF .world.

    ./gen_maze.py --cells 6 --seed 1 -o ../worlds/maze1.world

Стены складываются в ОДИН статический model с одним link и множеством
collision/visual. Так Gazebo обрабатывает их заметно быстрее, чем
сотню отдельных моделей.
"""
import argparse
import random

WALL_H = 0.6      # высота стен, м. Лидар TurtleBot3 на 0.15 — с запасом
WALL_T = 0.1      # толщина стен, м


def carve(n, rng):
    """Recursive backtracker. Возвращает наборы оставшихся стен.

    vw[(i,j)] — вертикальная стена слева от клетки (i,j), j = 0..n
    hw[(i,j)] — горизонтальная стена снизу от клетки (i,j), i = 0..n
    """
    vw = {(i, j) for i in range(n) for j in range(n + 1)}
    hw = {(i, j) for i in range(n + 1) for j in range(n)}

    visited = [[False] * n for _ in range(n)]
    stack = [(0, 0)]
    visited[0][0] = True

    while stack:
        i, j = stack[-1]
        nbrs = []
        if i > 0 and not visited[i - 1][j]:      nbrs.append((i - 1, j, 'S'))
        if i < n - 1 and not visited[i + 1][j]:  nbrs.append((i + 1, j, 'N'))
        if j > 0 and not visited[i][j - 1]:      nbrs.append((i, j - 1, 'W'))
        if j < n - 1 and not visited[i][j + 1]:  nbrs.append((i, j + 1, 'E'))

        if not nbrs:
            stack.pop()
            continue

        ni, nj, d = rng.choice(nbrs)
        # убираем стену между текущей клеткой и выбранной соседней
        if d == 'S':   hw.discard((i, j))
        elif d == 'N': hw.discard((i + 1, j))
        elif d == 'W': vw.discard((i, j))
        elif d == 'E': vw.discard((i, j + 1))

        visited[ni][nj] = True
        stack.append((ni, nj))

    return vw, hw


def segments(n, cell, vw, hw):
    """Переводит стены в отрезки (x, y, yaw, длина) с центром в начале координат."""
    off = n * cell / 2.0
    segs = []
    for i, j in sorted(vw):
        segs.append((j * cell - off, (i + 0.5) * cell - off, 1.5707963, cell))
    for i, j in sorted(hw):
        segs.append(((j + 0.5) * cell - off, i * cell - off, 0.0, cell))
    return segs


def to_sdf(segs, name):
    parts = [f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{name}">
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <model name="maze">
      <static>true</static>
      <link name="walls">''']

    for k, (x, y, yaw, ln) in enumerate(segs):
        # длина + толщина: чтобы углы стыковались без щелей
        size = f"{ln + WALL_T:.3f} {WALL_T:.3f} {WALL_H:.3f}"
        pose = f"{x:.3f} {y:.3f} {WALL_H / 2:.3f} 0 0 {yaw:.7f}"
        parts.append(f'''
        <collision name="c{k}">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="v{k}">
          <pose>{pose}</pose>
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.55 1</ambient>
            <diffuse>0.7 0.7 0.75 1</diffuse>
          </material>
        </visual>''')

    parts.append('''
      </link>
    </model>
  </world>
</sdf>
''')
    return ''.join(parts)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--cells', type=int, default=6, help='размер сетки, клеток (по умолчанию 6)')
    p.add_argument('--cell-size', type=float, default=1.5, help='сторона клетки, м (по умолчанию 1.5)')
    p.add_argument('--seed', type=int, default=None, help='зерно ГСЧ — один seed даёт один и тот же лабиринт')
    p.add_argument('-o', '--out', required=True, help='куда писать .world')
    a = p.parse_args()

    rng = random.Random(a.seed)
    vw, hw = carve(a.cells, rng)
    segs = segments(a.cells, a.cell_size, vw, hw)

    name = a.out.split('/')[-1].replace('.world', '')
    with open(a.out, 'w') as f:
        f.write(to_sdf(segs, name))

    side = a.cells * a.cell_size
    print(f"{a.out}: {a.cells}x{a.cells} клеток, {len(segs)} стен, поле {side:.1f}x{side:.1f} м")
    print(f"коридор {a.cell_size - WALL_T:.2f} м (робот 0.28 м)")
    print(f"стартовая клетка: x={-side/2 + a.cell_size/2:.2f} y={-side/2 + a.cell_size/2:.2f}")


if __name__ == '__main__':
    main()
