#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ResampleWithDataSet 测试模型生成脚本（标准 VTK 3.0 ASCII，格式与 iGameVTKReader 对齐）

产出（两个**类型不同**的网格，各自带点属性和单元属性）：

  Examples/Models/ResampleWithDataset_HexSource.vtk
      被采样网格（filter 的输入 0，对应 VTK 的 Source）：DATASET UNSTRUCTURED_GRID，
      [0,1]^3 上 2x2x2 六面体（27 点 / 8 单元）。
        POINT_DATA : temperature(线性) / velocity(VECTORS, 线性) / density(线性) / pressure(非线性)
        CELL_DATA  : material_id(int) / cell_porosity(float) / cell_center(VECTORS)

  Examples/Models/ResampleWithDataset_PlaneProbe.vtk
      采样点网格（filter 的输入 1，对应 VTK 的 Input）：DATASET POLYDATA（读入后是
      SurfaceMesh，与被采样网格的 UnstructuredMesh 类型不同），z = 0.25 的切片平面，
      x,y ∈ [-0.05, 1.05] 上 6x6 点 / 5x5 四边形；最外圈落在被采样网格之外，
      用来同时演示 validpointmask = 1 / 0 两种情况。
        POINT_DATA : probe_node_id(float)
        CELL_DATA  : probe_patch(int)

预期行为（与 filter 当前语义一致，见 doc/FilterUserNotice/ResampleWithDataSet使用说明.md）：
  * 输出类型/几何/拓扑 = 采样点网格的同类型深拷贝（这里是 SurfaceMesh，25 个四边形）；
  * 输出属性 = 被采样网格的**点数据插值**（temperature / velocity / density / pressure）
    + validpointmask；
  * 被采样网格的**单元数据**（material_id / cell_porosity / cell_center）不进入输出；
  * 采样点网格自身的数组（probe_node_id / probe_patch）也不进入输出；
  * 平面上 16 个内部点有效（temperature = 100 + 10x + 20y + 30z 是线性场，插值应精确），
    最外圈 20 个点在被采样网格之外，validpointmask = 0 且各属性填 0。

用法：python3 gen_resample_with_dataset_models.py
"""
import os

VTK_HEXAHEDRON = 12

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "Examples", "Models")


def fmt(v):
    """VTK ASCII 数值：整数不带小数点，浮点用最短表示。"""
    return ("%g" % v)


def write_lines(path, lines):
    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    print("wrote {} ({} bytes)".format(path, os.path.getsize(path)))


# ---------------------------------------------------------------------------
# 1) 被采样网格：2x2x2 六面体
# ---------------------------------------------------------------------------
def build_hex_source():
    nx = ny = nz = 3  # 每方向 3 个点 -> 2x2x2 个单元
    step = 0.5

    def idx(ix, iy, iz):
        return ix + nx * iy + nx * ny * iz

    points = []
    for iz in range(nz):
        for iy in range(ny):
            for ix in range(nx):
                points.append((ix * step, iy * step, iz * step))

    cells = []
    for iz in range(nz - 1):
        for iy in range(ny - 1):
            for ix in range(nx - 1):
                cells.append([
                    idx(ix, iy, iz), idx(ix + 1, iy, iz), idx(ix + 1, iy + 1, iz), idx(ix, iy + 1, iz),
                    idx(ix, iy, iz + 1), idx(ix + 1, iy, iz + 1), idx(ix + 1, iy + 1, iz + 1),
                    idx(ix, iy + 1, iz + 1),
                ])

    # 点数据：三个线性场（插值可精确核对）+ 一个非线性场
    temperature = [100.0 + 10.0 * x + 20.0 * y + 30.0 * z for (x, y, z) in points]
    velocity = [(1.0 + 2.0 * x, -1.0 + 3.0 * y, 0.5 + 4.0 * z) for (x, y, z) in points]
    density = [1.0 + 0.5 * x + 0.25 * y - 0.1 * z for (x, y, z) in points]
    pressure = [1.0 + x * x + y * z for (x, y, z) in points]

    # 单元数据：单元中心即六面体中心
    material_id = list(range(len(cells)))
    cell_porosity = [0.1 * (i + 1) for i in range(len(cells))]
    cell_center = []
    for cell in cells:
        cx = sum(points[p][0] for p in cell) / len(cell)
        cy = sum(points[p][1] for p in cell) / len(cell)
        cz = sum(points[p][2] for p in cell) / len(cell)
        cell_center.append((cx, cy, cz))

    lines = [
        "# vtk DataFile Version 3.0",
        "ResampleWithDataSet source mesh: 2x2x2 hexahedra over [0,1]^3, "
        "POINT_DATA temperature/velocity/density/pressure, "
        "CELL_DATA material_id/cell_porosity/cell_center",
        "ASCII",
        "DATASET UNSTRUCTURED_GRID",
        "POINTS {} float".format(len(points)),
    ]
    lines += [" ".join(fmt(c) for c in p) for p in points]

    conn = sum(len(c) + 1 for c in cells)
    lines.append("CELLS {} {}".format(len(cells), conn))
    lines += ["{} {}".format(len(c), " ".join(str(i) for i in c)) for c in cells]

    lines.append("CELL_TYPES {}".format(len(cells)))
    lines += [str(VTK_HEXAHEDRON)] * len(cells)

    lines.append("POINT_DATA {}".format(len(points)))
    for name, values in (("temperature", temperature), ("density", density), ("pressure", pressure)):
        lines.append("SCALARS {} float 1".format(name))
        lines.append("LOOKUP_TABLE default")
        lines += [fmt(v) for v in values]
    lines.append("VECTORS velocity float")
    lines += [" ".join(fmt(c) for c in v) for v in velocity]

    lines.append("CELL_DATA {}".format(len(cells)))
    for name, values in (("material_id", material_id), ("cell_porosity", cell_porosity)):
        dtype = "int" if name == "material_id" else "float"
        lines.append("SCALARS {} {} 1".format(name, dtype))
        lines.append("LOOKUP_TABLE default")
        lines += [fmt(v) for v in values]
    lines.append("VECTORS cell_center float")
    lines += [" ".join(fmt(c) for c in v) for v in cell_center]

    return lines


# ---------------------------------------------------------------------------
# 2) 采样点网格：POLYDATA 切片平面（最外圈有意伸出被采样网格之外）
# ---------------------------------------------------------------------------
def build_plane_probe():
    n = 6            # 每边点数
    lo, hi = -0.05, 1.05
    z = 0.25         # 落在被采样网格下层单元内部，避开 z = 0 / 0.5 / 1 的单元面
    step = (hi - lo) / (n - 1)

    def pid(i, j):
        return i + n * j

    points = [(lo + i * step, lo + j * step, z) for j in range(n) for i in range(n)]
    quads = [[pid(i, j), pid(i + 1, j), pid(i + 1, j + 1), pid(i, j + 1)]
             for j in range(n - 1) for i in range(n - 1)]

    probe_node_id = [float(i) for i in range(len(points))]
    probe_patch = list(range(len(quads)))

    lines = [
        "# vtk DataFile Version 3.0",
        "ResampleWithDataSet probe mesh: POLYDATA slice plane z=0.25 over [-0.05,1.05]^2 "
        "(outer ring lies outside the source mesh), POINT_DATA probe_node_id, CELL_DATA probe_patch",
        "ASCII",
        "DATASET POLYDATA",
        "POINTS {} float".format(len(points)),
    ]
    lines += [" ".join(fmt(c) for c in p) for p in points]

    conn = sum(len(q) + 1 for q in quads)
    lines.append("POLYGONS {} {}".format(len(quads), conn))
    lines += ["{} {}".format(len(q), " ".join(str(i) for i in q)) for q in quads]

    lines.append("POINT_DATA {}".format(len(points)))
    lines.append("SCALARS probe_node_id float 1")
    lines.append("LOOKUP_TABLE default")
    lines += [fmt(v) for v in probe_node_id]

    lines.append("CELL_DATA {}".format(len(quads)))
    lines.append("SCALARS probe_patch int 1")
    lines.append("LOOKUP_TABLE default")
    lines += [fmt(v) for v in probe_patch]

    return lines


if __name__ == "__main__":
    os.makedirs(MODELS, exist_ok=True)
    write_lines(os.path.join(MODELS, "ResampleWithDataset_HexSource.vtk"), build_hex_source())
    write_lines(os.path.join(MODELS, "ResampleWithDataset_PlaneProbe.vtk"), build_plane_probe())
