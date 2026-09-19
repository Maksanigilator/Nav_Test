"""Меш корпуса из STEP: всё, что НЕ вращается вместе с колёсами.

    freecadcmd src/maze_nav/scripts/cad_chassis_mesh.py -- \
        "Модель прототипа 2025.step" src/maze_nav/urdf/wheels_from_cad.json \
        src/maze_nav/meshes/chassis.stl

Из модели выбрасывается всё, что попадает в оболочку колёсного узла:
ролики, ступица, диски, обод, оси роликов, рельсовый ролик. Рисовать их
— дело звеньев колеса, и если такая деталь останется ещё и в корпусе,
она будет висеть на месте неподвижно, пока колесо вращается.

Именно это однажды и случилось: первый вариант меша отбирал колёсные тела
по ПРИКИДОЧНЫМ центрам колёс, промахиваясь на 28-31 мм. Часть роликов двух
колёс не попала в радиус отсечения и осталась в корпусе. Пока ролики в URDF
стояли ровно как в CAD, лишние копии совпадали с ними и были незаметны;
стоило развернуть раскладку зеркально — и на двух колёсах стало видно по
два набора роликов. Поэтому центры здесь берутся ИЗМЕРЕННЫЕ, из того же
json, по которому генерируется URDF.

Координаты меша остаются как в CAD (мм, ось колеса вдоль X): разворот в
систему ROS делает сам URDF поворотом визуала на -90° вокруг Z. Меши колёс,
в отличие от корпуса, разворачиваются при извлечении — у них своё звено.
"""
import json
import math
import sys

import FreeCAD
import Mesh
import MeshPart
import Part

MAST_Z = 900.0          # всё выше — мачта с камерами, в симуляторе не нужна
LINEAR_DEFLECTION = 8.0     # грубее, чем у колёс: корпус разглядывают издали

# Оболочка колёсного узла: всё внутри неё вращается вместе с колесом и
# рисуется звеньями колеса, а не корпусом.
#
# Мерить надо от ИЗМЕРЕННОГО центра оси, а не от прикидочного. Именно на
# этом всё и сломалось: в первом варианте центры были взяты на глаз, с
# промахом 28-31 мм, часть роликов двух колёс не попала в отсечение и
# осталась в корпусе. Пока ролики в URDF стояли ровно как в CAD, лишние
# копии совпадали с ними и были не видны; стоило развернуть раскладку
# зеркально — и на двух колёсах стало по два набора роликов.
AXIAL_HALF = 150.0      # вдоль оси: хватает на рельсовый ролик, вынесенный внутрь
RADIAL = 115.0          # поперёк: ролики достают до 101.5, реборда до 81


def main(step_path, wheels_path, out_path):
    shp = Part.read(step_path)
    wheels = json.load(open(wheels_path))

    centres = [(w['plane_x_cad'], w['axis_y_cad'], w['axis_z_cad'])
               for w in wheels.values()]

    chassis, dropped = [], {'узлы колёс': 0, 'мачта': 0}
    for s in shp.Solids:
        bb = s.BoundBox
        if bb.ZMax > MAST_Z:
            dropped['мачта'] += 1
            continue
        cx = (bb.XMin + bb.XMax) / 2
        cy = (bb.YMin + bb.YMax) / 2
        cz = (bb.ZMin + bb.ZMax) / 2

        rotating = any(abs(cx - px) < AXIAL_HALF
                       and math.hypot(cy - wy, cz - wz) < RADIAL
                       for px, wy, wz in centres)
        kind = 'узлы колёс' if rotating else None

        if kind:
            dropped[kind] += 1
        else:
            chassis.append(s)

    for k, v in dropped.items():
        sys.stderr.write(f'  выброшено ({k}): {v} тел\n')
    sys.stderr.write(f'  в корпусе: {len(chassis)} тел\n')

    comp = Part.makeCompound(chassis)
    m = MeshPart.meshFromShape(Shape=comp, LinearDeflection=LINEAR_DEFLECTION,
                               AngularDeflection=0.6, Relative=False)
    Mesh.Mesh(m.Topology).write(out_path)
    bb = comp.BoundBox
    sys.stderr.write(f'{out_path}: {m.CountFacets} треуг., габарит '
                     f'{bb.XLength:.0f}x{bb.YLength:.0f}x{bb.ZLength:.0f} мм\n')


if __name__ == '__main__':
    a = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else sys.argv[1:]
    main(*a[:3])
