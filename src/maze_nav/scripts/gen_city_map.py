#!/usr/bin/env python3
"""Генератор карты Nav2 для полигона «Город РТК» по данным регламента.

Строит occupancy grid (.pgm + .yaml) напрямую из геометрии полигона,
без объезда лидаром. Источник размеров — scene/city_builder.py проекта
gorod_rtk, копия лежит в регламент/ этого репозитория.

    ./gen_city_map.py -o ../maps/city
    ./gen_city_map.py --gate south:4 -o ../maps/city   # въезд у правого угла
    ./gen_city_map.py --no-fence -o ../maps/city_open

Что попадает на карту и почему. Лидар робота стоит примерно на 6 см над
полом, поэтому в скан попадает не всё, что есть на полигоне:

  забор по периметру   высота 0.30 м   ВИДЕН   -> препятствие
  постаменты 0.8x0.8   высота 0.08 м   ВИДЕН   -> препятствие
  здания на постаментах основание 0.085 м      -> выше луча, НЕ видны
  бордюры              высота 0.025 м          -> ниже луча, НЕ видны
  дорожная разметка    плоская                 -> не видна

Поэтому здания рисовать не нужно: робот видит только площадки под ними.
"""
import argparse

# --- геометрия полигона (city_builder.py) ---
CELL_SIZE = 0.8            # м, сторона клетки, она же сторона постамента
GRID_N = 5                 # клеток в ряду
POLYGON_SIZE = CELL_SIZE * GRID_N          # 4.0 м
HALF = POLYGON_SIZE / 2.0                  # 2.0 м
BUILDING_CELLS = [(1, 1), (1, 3), (3, 1), (3, 3)]   # (row, col)
FENCE_THICKNESS = 0.005    # м, реальная толщина забора

# значения пикселей в формате PGM, как их понимает map_server
OCCUPIED, FREE, UNKNOWN = 0, 254, 205


def cell_center(row, col):
    """Центр клетки в метрах. Начало координат — центр полигона, X вправо, Y вверх."""
    x = -HALF + CELL_SIZE * col + CELL_SIZE / 2.0
    y = -HALF + CELL_SIZE * row + CELL_SIZE / 2.0
    return x, y


def build(res, margin, fence, fence_px, gate):
    # Карта охватывает полигон плюс поля по краям
    span = POLYGON_SIZE + 2 * margin
    n = int(round(span / res))
    origin = -span / 2.0

    # Пиксель (ix, iy): iy отсчитывается снизу, в файл пишется сверху вниз
    def to_px(m):
        return int(round((m - origin) / res))

    grid = [[UNKNOWN] * n for _ in range(n)]

    # Внутренность полигона свободна
    lo, hi = to_px(-HALF), to_px(HALF)
    for iy in range(lo, hi):
        for ix in range(lo, hi):
            grid[iy][ix] = FREE

    # Постаменты под зданиями
    for row, col in BUILDING_CELLS:
        cx, cy = cell_center(row, col)
        x0, x1 = to_px(cx - CELL_SIZE / 2), to_px(cx + CELL_SIZE / 2)
        y0, y1 = to_px(cy - CELL_SIZE / 2), to_px(cy + CELL_SIZE / 2)
        for iy in range(y0, y1):
            for ix in range(x0, x1):
                grid[iy][ix] = OCCUPIED

    # Забор по периметру. Реальная толщина 5 мм тоньше клетки карты, поэтому
    # рисуем минимум в fence_px пикселей — иначе AMCL не за что зацепиться.
    # Стены строятся ровными полосами наружу от границы полигона: раньше забор
    # наращивался слоями с расширением диапазона, и на углах вылезали ступеньки.
    if fence:
        t = max(fence_px, int(round(FENCE_THICKNESS / res)))

        # Проём (въезд) шириной в одну клетку.
        # gate = (сторона, индекс клетки 0..GRID_N-1); индекс считается слева
        # направо для south/north и снизу вверх для west/east.
        gap_lo = gap_hi = None
        gap_side = None
        if gate:
            gap_side, idx = gate
            a0 = -HALF + CELL_SIZE * idx
            gap_lo, gap_hi = to_px(a0), to_px(a0 + CELL_SIZE)

        def put(iy, ix, side, along):
            if gap_side == side and gap_lo <= along < gap_hi:
                return
            if 0 <= iy < n and 0 <= ix < n:
                grid[iy][ix] = OCCUPIED

        # горизонтальные стены идут во всю ширину, включая углы
        for k in range(t):
            for ix in range(lo - t, hi + t):
                put(lo - 1 - k, ix, 'south', ix)
                put(hi + k, ix, 'north', ix)
        # вертикальные — только между горизонтальными, чтобы углы не дублировались
        for k in range(t):
            for iy in range(lo, hi):
                put(iy, lo - 1 - k, 'west', iy)
                put(iy, hi + k, 'east', iy)

    return grid, n, origin


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('-o', '--out', required=True,
                   help='путь без расширения: создаст .pgm и .yaml')
    p.add_argument('--resolution', type=float, default=0.05, help='м/пиксель (по умолчанию 0.05)')
    p.add_argument('--margin', type=float, default=0.3,
                   help='поля вокруг полигона, м (по умолчанию 0.3)')
    p.add_argument('--no-fence', action='store_true',
                   help='не рисовать периметральный забор')
    p.add_argument('--fence-px', type=int, default=2,
                   help='толщина забора в пикселях (по умолчанию 2)')
    p.add_argument('--gate', default='south:0',
                   help='проём в заборе: сторона:клетка, например south:0 (по умолчанию) '
                        'или south:4. Клетки нумеруются 0..4 слева направо для south/north '
                        'и снизу вверх для west/east. Пустая строка — забор сплошной.')
    a = p.parse_args()

    gate = None
    if a.gate:
        side, idx = a.gate.split(':')
        gate = (side, int(idx))

    grid, n, origin = build(a.resolution, a.margin, not a.no_fence, a.fence_px, gate)

    pgm = a.out + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n')
        f.write(b'# Polygon "Gorod RTK", generated from regulation geometry\n')
        f.write(f'{n} {n}\n255\n'.encode())
        # PGM пишется сверху вниз, а наша сетка снизу вверх
        for iy in range(n - 1, -1, -1):
            f.write(bytes(grid[iy]))

    yaml = a.out + '.yaml'
    with open(yaml, 'w') as f:
        f.write(f"image: {pgm.split('/')[-1]}\n")
        f.write("mode: trinary\n")
        f.write(f"resolution: {a.resolution}\n")
        f.write(f"origin: [{origin:.3f}, {origin:.3f}, 0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    occ = sum(r.count(OCCUPIED) for r in grid)
    free = sum(r.count(FREE) for r in grid)
    print(f"{pgm}: {n}x{n} пикселей = {n*a.resolution:.1f}x{n*a.resolution:.1f} м "
          f"при {a.resolution} м/пиксель")
    print(f"занято {occ} ({occ/(n*n)*100:.1f}%), свободно {free} ({free/(n*n)*100:.1f}%)")
    print(f"полигон {POLYGON_SIZE}x{POLYGON_SIZE} м, начало координат в его центре")
    print(f"постаменты {CELL_SIZE}x{CELL_SIZE} м в клетках {BUILDING_CELLS}")
    print("центры постаментов: " + ", ".join(
        f"({cell_center(r,c)[0]:+.1f}, {cell_center(r,c)[1]:+.1f})" for r, c in BUILDING_CELLS))
    if gate:
        a0 = -HALF + CELL_SIZE * gate[1]
        print(f"проём в заборе: сторона {gate[0]}, клетка {gate[1]}, "
              f"от {a0:+.1f} до {a0 + CELL_SIZE:+.1f} м")


if __name__ == '__main__':
    main()
