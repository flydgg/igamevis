// ============================================================================
// ProbeFilter — 见 iGameProbeFilter.h
//
// 算法说明（v3）:
//   1. 单元定位：遍历模型单元。面网格走面单元、体网格走体单元
//      （v1 仅三角形 / 四面体，其余类型未命中）；命中后按重心坐标对全部
//      点属性（标量 / 矢量等）做线性插值。
//   2. 结果原地写入查询点集：复用/新建同名点属性数组，另加
//      ValidPointMask（找到单元 = 1，未找到 = 0，未找到时插值属性填 0）。
//   3. 输出 0 与输入 1 为同一个对象，重复执行不新建点集。
// ============================================================================
#include "iGameProbeFilter.h"

#include <iGameThreadPool.h>
#include <iGameAttributeSet.h>
#include <iGamePolyLine.h>
#include <iGameCellArray.h>
#include <iGameCompositeDataObject.h>
#include <iGameFlatArray.h>
#include <iGameLagrangeHexahedron.h>
#include <iGameLagrangeLine.h>
#include <iGameLagrangePrism.h>
#include <iGameLagrangeQuadrilateral.h>
#include <iGameLagrangeTetrahedron.h>
#include <iGameLagrangeTriangle.h>
#include <iGameLagrangeUnstructuredMesh.h>
#include <iGameStructuredMesh.h>
#include <iGameSurfaceMesh.h>
#include <iGameUnstructuredMesh.h>
#include <iGameVolumeMesh.h>

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <random>
#include <string>
#include <vector>

IGAME_NAMESPACE_BEGIN

ProbeFilter::ProbeFilter() {
    SetNumberOfInputs(2);
    SetNumberOfOutputs(1);
}

void ProbeFilter::GenerateSpherePoints(PointSet::Pointer points, const Point& center,
                                       float radius, int count, unsigned seed) {
    // radius < 0 非法；radius == 0 时按退化球处理：全部点落在球心。
    if (points.IsNull() || count <= 0 || radius < 0.0f) return;
    auto pts = points->GetPoints();
    if (pts.IsNull()) return;

    std::mt19937 gen;
    if (seed == 0) {
        std::random_device rd;
        gen.seed(rd());
    } else {
        gen.seed(seed);
    }
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    constexpr double kTwoPi = 6.283185307179586476925286766559;

    // 球体体积均匀采样：r = R * u^(1/3)，θ 均匀，φ = acos(1 - 2u)
    pts->Reset();
    pts->Reserve(static_cast<IGsize>(count));
    for (int i = 0; i < count; ++i) {
        const double r = static_cast<double>(radius) * std::cbrt(u01(gen));
        const double theta = kTwoPi * u01(gen);
        const double phi = std::acos(1.0 - 2.0 * u01(gen));
        const double sp = std::sin(phi);
        pts->AddPoint(static_cast<float>(center[0] + r * sp * std::cos(theta)),
                      static_cast<float>(center[1] + r * sp * std::sin(theta)),
                      static_cast<float>(center[2] + r * std::cos(phi)));
    }
    points->Modified();
}

/* ================================================================== */
/* 采样内核：CSR 均匀网格 + 单元类型驱动的定位 + VTK 语义属性          */
/*                                                                    */
/* 与 vtkProbeFilter 的对应关系：                                      */
/*   - 空间索引 ≈ vtkStaticCellLocator（均匀桶，自动分辨率）；         */
/*   - 单元判定 = 单元类型自己的 EvaluatePosition（vtkCell 风格）；     */
/*   - 命中规则 = vtkDataSet::FindCell：在单元内，或 dist2 ≤ tol2；    */
/*   - 容差 = 源包围盒对角线 × 1e-3（VTK: 对角线² × 1e-6，平方比较）； */
/*   - 上一个命中单元作为提示（VTK 的 FindCell(x, lastCellId)）。       */
/* ================================================================== */
namespace {

/** VTK 默认容差系数：tol = 包围盒对角线 × 1e-3 */
constexpr double kVTKToleranceFactor = 1e-3;
constexpr igIndex kGridMinDim = 4;
constexpr igIndex kGridMaxDim = 96;
/** 目标“每体素单元数”，用于自动选择网格分辨率（vtkStaticCellLocator 的 NumberOfCellsPerNode 思路） */
constexpr double kCellsPerVoxelTarget = 4.0;

/** vtkDataSetAttributes 的 ghost 位：HIDDENCELL / DUPLICATECELL */
constexpr unsigned char kGhostHiddenCell = 32;
constexpr unsigned char kGhostDuplicateCell = 64;
constexpr unsigned char kGhostCellMask = static_cast<unsigned char>(kGhostHiddenCell | kGhostDuplicateCell);

// 单元对象工厂（定义在后面；BuildCellGrid 需要用它判断是否能并行）
IGenum LinearTypeFor(IGenum type);
Cell* NativeCellForType(IGenum type);
bool RequiresMeshCell(IGenum type);

/** CSR 均匀网格：扁平候选表 + 每体素区间（避免 vector<vector<>> 的大量小分配） */
struct CellGrid {
    igIndex nx{0}, ny{0}, nz{0};
    double origin[3]{0.0, 0.0, 0.0};
    double voxel[3]{1.0, 1.0, 1.0};
    std::vector<igIndex> offsets; // 长度 nx*ny*nz + 1
    std::vector<igIndex> cells;   // 扁平候选表
    std::vector<double> boxes;    // 每单元 6 个 double：minx,miny,minz,maxx,maxy,maxz

    bool Valid() const { return nx > 0 && ny > 0 && nz > 0; }
    size_t VoxelCount() const { return static_cast<size_t>(nx) * static_cast<size_t>(ny) * static_cast<size_t>(nz); }
    size_t VoxelIndex(igIndex ix, igIndex iy, igIndex iz) const {
        return static_cast<size_t>(ix) + static_cast<size_t>(iy) * nx +
               static_cast<size_t>(iz) * nx * ny;
    }
};

/** 取单元的包围盒（含容差外扩）写入 grid.boxes */
void ComputeCellBox(const UnstructuredMesh::Pointer& mesh, igIndex cellId, double tol, double* box) {
    double minv[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
    double maxv[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    igIndex ptIds[IGAME_CELL_MAX_SIZE];
    const int npts = mesh->GetCellPointIds(cellId, ptIds);
    for (int k = 0; k < npts; ++k) {
        const Point& p = mesh->GetPoint(ptIds[k]);
        for (int j = 0; j < 3; ++j) {
            const double v = static_cast<double>(p[j]);
            minv[j] = std::min(minv[j], v);
            maxv[j] = std::max(maxv[j], v);
        }
    }
    if (npts <= 0) {
        for (int j = 0; j < 3; ++j) { minv[j] = maxv[j] = 0.0; }
    }
    for (int j = 0; j < 3; ++j) {
        box[j] = minv[j] - tol;
        box[3 + j] = maxv[j] + tol;
    }
}

/** 建立 CSR 均匀网格（两遍：计数 → 前缀和 → 填充） */
void BuildCellGrid(const UnstructuredMesh::Pointer& mesh, const BoundingBox& bbox, double tol,
                   CellGrid& grid, bool& unsafeForParallel) {
    unsafeForParallel = false;
    const igIndex cellNum = static_cast<igIndex>(mesh->GetNumberOfCells());
    if (cellNum <= 0) { return; }

    double span[3];
    for (int j = 0; j < 3; ++j) {
        span[j] = static_cast<double>(bbox.max[j]) - static_cast<double>(bbox.min[j]);
    }
    const double maxSpan = std::max(span[0], std::max(span[1], span[2]));
    if (maxSpan <= 0.0) { return; }

    // 自动分辨率：cbrt(单元数 / 目标每体素单元数)，并按包围盒长宽比分配三轴
    const double base = std::cbrt(static_cast<double>(cellNum) / kCellsPerVoxelTarget);
    igIndex dim[3];
    for (int j = 0; j < 3; ++j) {
        const double ratio = (span[j] > 0.0) ? (span[j] / maxSpan) : 0.0;
        dim[j] = static_cast<igIndex>(base * std::max(ratio, 1.0 / 16.0) + 0.5);
        dim[j] = std::max<igIndex>(kGridMinDim, std::min<igIndex>(kGridMaxDim, dim[j]));
    }
    grid.nx = dim[0];
    grid.ny = dim[1];
    grid.nz = dim[2];
    for (int j = 0; j < 3; ++j) {
        grid.origin[j] = static_cast<double>(bbox.min[j]);
        grid.voxel[j] = span[j] / static_cast<double>(dim[j]);
        if (grid.voxel[j] < 1e-12) { grid.voxel[j] = 1.0; }
    }

    grid.boxes.assign(static_cast<size_t>(cellNum) * 6, 0.0);
    std::vector<igIndex> counts(grid.VoxelCount() + 1, 0);
    std::vector<igIndex> ranges(static_cast<size_t>(cellNum) * 6, 0); // 每单元的体素范围

    const auto clampIndex = [](igIndex v, igIndex hi) {
        return std::max<igIndex>(0, std::min<igIndex>(hi - 1, v));
    };

    for (igIndex cellId = 0; cellId < cellNum; ++cellId) {
        // 只能用 mesh 共享缓存单元对象的类型（PolyLine / Polyhedron 等）不适合并行
        if (RequiresMeshCell(mesh->GetCellType(cellId))) { unsafeForParallel = true; }
        double* box = &grid.boxes[static_cast<size_t>(cellId) * 6];
        ComputeCellBox(mesh, cellId, tol, box);
        igIndex* range = &ranges[static_cast<size_t>(cellId) * 6];
        for (int j = 0; j < 3; ++j) {
            const igIndex dimJ = dim[j];
            range[j] = clampIndex(static_cast<igIndex>((box[j] - grid.origin[j]) / grid.voxel[j]), dimJ);
            range[3 + j] = clampIndex(static_cast<igIndex>((box[3 + j] - grid.origin[j]) / grid.voxel[j]), dimJ);
        }
        for (igIndex iz = range[2]; iz <= range[5]; ++iz) {
            for (igIndex iy = range[1]; iy <= range[4]; ++iy) {
                for (igIndex ix = range[0]; ix <= range[3]; ++ix) {
                    ++counts[grid.VoxelIndex(ix, iy, iz)];
                }
            }
        }
    }

    grid.offsets.assign(grid.VoxelCount() + 1, 0);
    for (size_t v = 0; v < grid.VoxelCount(); ++v) {
        grid.offsets[v + 1] = grid.offsets[v] + counts[v];
    }
    grid.cells.assign(static_cast<size_t>(grid.offsets.back()), -1);
    std::vector<igIndex> cursor(grid.offsets.begin(), grid.offsets.end() - 1);
    for (igIndex cellId = 0; cellId < cellNum; ++cellId) {
        const igIndex* range = &ranges[static_cast<size_t>(cellId) * 6];
        for (igIndex iz = range[2]; iz <= range[5]; ++iz) {
            for (igIndex iy = range[1]; iy <= range[4]; ++iy) {
                for (igIndex ix = range[0]; ix <= range[3]; ++ix) {
                    const size_t v = grid.VoxelIndex(ix, iy, iz);
                    grid.cells[static_cast<size_t>(cursor[v]++)] = cellId;
                }
            }
        }
    }
}

/** 源网格上的 vtkGhostType（单元 ghost 数组），用于跳过 blanked 单元 */
ArrayObject* FindCellGhostArray(const AttributeSet* attrs) {
    if (attrs == nullptr) { return nullptr; }
    auto all = const_cast<AttributeSet*>(attrs)->GetAllAttributes();
    if (all == nullptr) { return nullptr; }
    for (IGsize i = 0; i < all->GetNumberOfElements(); ++i) {
        auto& attr = all->GetElement(i);
        if (attr.isDeleted || attr.pointer == nullptr) { continue; }
        if (attr.attachmentType != IG_CELL) { continue; }
        std::string name = attr.pointer->GetName();
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (name == "vtkghosttype") { return attr.pointer.get(); }
    }
    return nullptr;
}

bool IsBlankedCell(ArrayObject* ghost, igIndex cellId) {
    if (ghost == nullptr) { return false; }
    if (static_cast<IGsize>(cellId) >= ghost->GetNumberOfElements()) { return false; }
    const double v = ghost->GetElementValue(static_cast<IGsize>(cellId), 0);
    return (static_cast<unsigned char>(v) & kGhostCellMask) != 0;
}

/**
 * 未实现 VTK 插值的单元类型退化时所用的「角点线性」单元类型；IG_NONE = 不做退化。
 * 等价旧内核的 LinearCornerCount + 角点线性退化策略。
 */
IGenum LinearTypeFor(IGenum type) {
    switch (type) {
        case IG_QUADRATIC_EDGE:
            return IG_LINE;
        case IG_QUADRATIC_TRIANGLE:
        case IG_BIQUADRATIC_TRIANGLE:
        case IG_LAGRANGE_TRIANGLE:
            return IG_TRIANGLE;
        case IG_QUADRATIC_QUAD:
        case IG_BIQUADRATIC_QUAD:
        case IG_QUADRATIC_LINEAR_QUAD:
        case IG_QUADRATIC_POLYGON:
        case IG_LAGRANGE_QUADRILATERAL:
            return IG_QUAD;
        case IG_QUADRATIC_TETRA:
        case IG_LAGRANGE_TETRAHEDRON:
            return IG_TETRA;
        case IG_QUADRATIC_PRISM:
        case IG_QUADRATIC_LINEAR_WEDGE:
        case IG_BIQUADRATIC_QUADRATIC_WEDGE:
        case IG_LAGRANGE_PRISM:
            return IG_PRISM;
        case IG_QUADRATIC_PYRAMID:
        case IG_TRIQUADRATIC_PYRAMID:
        case IG_LAGRANGE_PYRAMID:
            return IG_PYRAMID;
        case IG_QUADRATIC_HEXAHEDRON:
        case IG_TRIQUADRATIC_HEXAHEDRON:
        case IG_BIQUADRATIC_QUADRATIC_HEXAHEDRON:
        case IG_LAGRANGE_HEXAHEDRON:
            return IG_HEXAHEDRON;
        default:
            return IG_NONE;
    }
}

/**
 * 按单元类型构造标准单元对象（每线程一份，避免与 mesh 内部共享缓存单元对象）。
 * 覆盖线性 10 类 + 二次 7 类；需要面拓扑的 Polyhedron（以及未实现类型）返回 nullptr。
 */
Cell* NativeCellForType(IGenum type) {
    thread_local struct {
        Cell::Pointer vertex, line, triangle, quad, polygon, tetra, hexahedron, prism, pyramid;
        Cell::Pointer quadraticLine, quadraticTriangle, quadraticQuad, quadraticTetra,
                quadraticHexahedron, quadraticPrism, quadraticPyramid;
        Cell::Pointer lagrangeLine, lagrangeTriangle, lagrangeQuadrilateral, lagrangeHexahedron;
        Cell::Pointer lagrangeTetrahedron, lagrangePrism;
    } cache;

    switch (type) {
        case IG_VERTEX:
            if (cache.vertex == nullptr) { cache.vertex = Vertex::New(); }
            return cache.vertex.get();
        case IG_LINE:
            if (cache.line == nullptr) { cache.line = Line::New(); }
            return cache.line.get();
        case IG_TRIANGLE:
            if (cache.triangle == nullptr) { cache.triangle = Triangle::New(); }
            return cache.triangle.get();
        case IG_QUAD:
            if (cache.quad == nullptr) { cache.quad = Quad::New(); }
            return cache.quad.get();
        case IG_POLYGON:
            if (cache.polygon == nullptr) { cache.polygon = Polygon::New(); }
            return cache.polygon.get();
        case IG_TETRA:
            if (cache.tetra == nullptr) { cache.tetra = Tetra::New(); }
            return cache.tetra.get();
        case IG_HEXAHEDRON:
            if (cache.hexahedron == nullptr) { cache.hexahedron = Hexahedron::New(); }
            return cache.hexahedron.get();
        case IG_PRISM:
            if (cache.prism == nullptr) { cache.prism = Prism::New(); }
            return cache.prism.get();
        case IG_PYRAMID:
            if (cache.pyramid == nullptr) { cache.pyramid = Pyramid::New(); }
            return cache.pyramid.get();
        case IG_QUADRATIC_EDGE:
            if (cache.quadraticLine == nullptr) { cache.quadraticLine = QuadraticLine::New(); }
            return cache.quadraticLine.get();
        case IG_QUADRATIC_TRIANGLE:
            if (cache.quadraticTriangle == nullptr) { cache.quadraticTriangle = QuadraticTriangle::New(); }
            return cache.quadraticTriangle.get();
        case IG_QUADRATIC_QUAD:
            if (cache.quadraticQuad == nullptr) { cache.quadraticQuad = QuadraticQuad::New(); }
            return cache.quadraticQuad.get();
        case IG_QUADRATIC_TETRA:
            if (cache.quadraticTetra == nullptr) { cache.quadraticTetra = QuadraticTetra::New(); }
            return cache.quadraticTetra.get();
        case IG_QUADRATIC_HEXAHEDRON:
            if (cache.quadraticHexahedron == nullptr) { cache.quadraticHexahedron = QuadraticHexahedron::New(); }
            return cache.quadraticHexahedron.get();
        case IG_QUADRATIC_PRISM:
            if (cache.quadraticPrism == nullptr) { cache.quadraticPrism = QuadraticPrism::New(); }
            return cache.quadraticPrism.get();
        case IG_QUADRATIC_PYRAMID:
            if (cache.quadraticPyramid == nullptr) { cache.quadraticPyramid = QuadraticPyramid::New(); }
            return cache.quadraticPyramid.get();
        // Lagrange：已按 VTK 语义实现插值的类型（其余 Lagrange 类型仍走角点线性退化）
        case IG_LAGRANGE_CURVE:
            if (cache.lagrangeLine == nullptr) { cache.lagrangeLine = LagrangeLine::New(); }
            return cache.lagrangeLine.get();
        case IG_LAGRANGE_TRIANGLE:
            if (cache.lagrangeTriangle == nullptr) { cache.lagrangeTriangle = LagrangeTriangle::New(); }
            return cache.lagrangeTriangle.get();
        case IG_LAGRANGE_QUADRILATERAL:
            if (cache.lagrangeQuadrilateral == nullptr) {
                cache.lagrangeQuadrilateral = LagrangeQuadrilateral::New();
            }
            return cache.lagrangeQuadrilateral.get();
        case IG_LAGRANGE_HEXAHEDRON:
            if (cache.lagrangeHexahedron == nullptr) {
                cache.lagrangeHexahedron = LagrangeHexahedron::New();
            }
            return cache.lagrangeHexahedron.get();
        case IG_LAGRANGE_TETRAHEDRON:
            if (cache.lagrangeTetrahedron == nullptr) {
                cache.lagrangeTetrahedron = LagrangeTetrahedron::New();
            }
            return cache.lagrangeTetrahedron.get();
        case IG_LAGRANGE_PRISM:
            if (cache.lagrangePrism == nullptr) { cache.lagrangePrism = LagrangePrism::New(); }
            return cache.lagrangePrism.get();
        default:
            return nullptr;
    }
}

/** 原生单元优先；未实现插值的类型退化为角点线性单元 */
Cell* MakeCellForType(IGenum type) {
    if (Cell* native = NativeCellForType(type)) { return native; }
    const IGenum linear = LinearTypeFor(type);
    return (linear == IG_NONE) ? nullptr : NativeCellForType(linear);
}

/**
 * 该类型是否必须用 mesh 内部构造的单元对象（共享缓存 → 不能并行）：
 * 原生类型或可退化为角点线性的类型都用内核自己的每线程单元对象，其余（PolyLine / Polyhedron 等）
 * 只能走 mesh->GetCell()。
 */
/** 采样点并行的最小样本数（与定位一致的阈值） */
constexpr IGsize kSampleParallelThreshold = 1024;

/**
 * 采样点按块并行：走线程池的任务队列（而不是每个分块起一个裸线程），
 * 并且分块数取「工作线程数 × 4」，让先完成的线程继续取后面的块 → 命中分布不均时也能均衡。
 */
template<typename Func>
void ParallelChunks(int sampleCount, Func&& process) {
    if (sampleCount <= 0) { return; }
    const int workers = std::max(1, ThreadPool::GetDefaultThreadCount());
    const int chunks = std::max(1, std::min(sampleCount, workers * 4));
    if (chunks <= 1) {
        process(0, sampleCount);
        return;
    }
    ThreadPool::parallelFor(0, sampleCount,
                            [&](int begin, int end) { process(begin, end); }, chunks);
}

bool RequiresMeshCell(IGenum type) {
    // 折线由内核的每线程 PolyLine 承担（点数由网格给定），不再依赖 mesh 的共享缓存单元
    if (type == IG_POLY_LINE) { return false; }
    if (NativeCellForType(type) != nullptr) { return false; }
    const IGenum linear = LinearTypeFor(type);
    return !(linear != IG_NONE && NativeCellForType(linear) != nullptr);
}

/** 用网格单元的点填充内核自己的每线程单元对象（pointCount 个点） */
void FillKernelCell(Cell* cell, const UnstructuredMesh::Pointer& mesh, const igIndex* ptIds, int pointCount) {
    cell->Reset();
    for (int i = 0; i < pointCount; ++i) {
        const Point& p = mesh->GetPoint(ptIds[i]);
        cell->m_Points->AddPoint(p[0], p[1], p[2]);
        cell->m_PointIds->AddId(ptIds[i]);
    }
}

/**
 * 取用于求值的单元对象：
 *   1) 单元类型自己的 VTK 插值实现（**内核自己的每线程单元对象**，不共享 mesh 内部缓存，
 *      因此可用在并行循环里）；
 *   2) 需要面拓扑的类型（Polyhedron）：只能用 mesh 提供的单元对象（共享缓存，故不并行）；
 *   3) 未实现插值的类型：退化为「前 N 个角点」的线性单元（等价旧内核的角点线性退化，
 *      并在 SampleLocation::degraded 标记）。nptsOut 返回实际参与求值的点数。
 *
 * 注意：插值阶段（InterpolateAttributes）必须用同一个函数再取一次单元，才能保证
 * 权重与全局点 id 一一对应。
 */
Cell* KernelCellFor(const UnstructuredMesh::Pointer& mesh, igIndex cellId, int& nptsOut, bool* degraded) {
    // 「上次填充好的单元」缓存：同一线程连续/重复请求同一个单元时直接复用，避免
    // Reset()+AddPoint() 重填与 GetCellPointIds 的拷贝。定位与随后的一次插值通常命中
    // 同一个单元，命中率很高。键包含 mesh 地址与 MTime，网格被修改后自动失效。
    struct FilledCell {
        const void* meshKey{nullptr};
        unsigned int meshMTime{0};
        igIndex cellId{-1};
        IGenum type{IG_NONE};
        Cell* cell{nullptr};
        int numPoints{0};
        bool degraded{false};
    };
    thread_local FilledCell s_filled;

    const void* meshKey = static_cast<const void*>(mesh.GetPointer());
    const unsigned int meshMTime = mesh->GetMTime();
    if (s_filled.cell != nullptr && s_filled.cellId == cellId && s_filled.meshKey == meshKey &&
        s_filled.meshMTime == meshMTime && s_filled.type == mesh->GetCellType(cellId)) {
        nptsOut = s_filled.numPoints;
        if (degraded != nullptr) { *degraded = s_filled.degraded; }
        return s_filled.cell;
    }
    const auto remember = [&](Cell* cell, int numPoints, bool isDegraded) {
        s_filled.meshKey = meshKey;
        s_filled.meshMTime = meshMTime;
        s_filled.cellId = cellId;
        s_filled.type = mesh->GetCellType(cellId);
        s_filled.cell = cell;
        s_filled.numPoints = numPoints;
        s_filled.degraded = isDegraded;
        return cell;
    };

    if (degraded != nullptr) { *degraded = false; }
    igIndex ptIds[IGAME_CELL_MAX_SIZE];
    const int npts = mesh->GetCellPointIds(cellId, ptIds);
    nptsOut = npts;
    if (npts <= 0) { return nullptr; }

    const IGenum type = mesh->GetCellType(cellId);

    // 1) 原生类型：用内核每线程单元对象（固定点数类型用其常量点数；Polygon 等动态类型用实际点数）
    if (Cell* native = NativeCellForType(type)) {
        const int need = (native->GetCellSize() > 0) ? native->GetCellSize() : npts;
        if (need > 0 && need <= npts) {
            FillKernelCell(native, mesh, ptIds, need);
            nptsOut = need;
            return remember(native, need, false);
        }
    }

    // 2) 折线：用内核每线程 PolyLine（点数由网格给定；等价 vtkPolyLine 的按段求值）
    if (type == IG_POLY_LINE) {
        thread_local PolyLine::Pointer s_polyLine = PolyLine::New();
        FillKernelCell(s_polyLine.GetPointer(), mesh, ptIds, npts);
        nptsOut = npts;
        return remember(s_polyLine.GetPointer(), npts, false);
    }

    // 3) 需要面拓扑的类型（Polyhedron）：沿用 mesh 内部构造好的单元对象
    if (Cell* meshCell = mesh->GetCell(cellId); meshCell != nullptr && meshCell->GetCellType() != IG_EMPTY_CELL) {
        return remember(meshCell, npts, false);
    }

    // 4) 退化：前 N 个角点的线性单元
    const IGenum linear = LinearTypeFor(type);
    Cell* fallback = (linear == IG_NONE) ? nullptr : NativeCellForType(linear);
    if (fallback == nullptr) { return mesh->GetCell(cellId); }

    const int corners = (fallback->GetCellSize() > 0) ? fallback->GetCellSize() : npts;
    if (corners <= 0 || corners > npts) { return mesh->GetCell(cellId); }

    FillKernelCell(fallback, mesh, ptIds, corners);
    nptsOut = corners;
    if (degraded != nullptr) { *degraded = true; }
    return remember(fallback, corners, true);
}

int EvaluateCellAt(const UnstructuredMesh::Pointer& mesh, igIndex cellId, const Point& p,
                   ProbeFilter::SampleLocation& loc) {
    int n = 0;
    bool degraded = false;
    Cell* cell = KernelCellFor(mesh, cellId, n, &degraded);
    if (cell == nullptr) { return 0; }
    if (n <= 0 || n > IGAME_CELL_MAX_SIZE) { return 0; }

    double pcoords[3] = {0.0, 0.0, 0.0};
    double dist2 = 0.0;
    Point closest;
    int subId = 0;
    const int rc = cell->EvaluatePosition(p, closest, subId, pcoords, dist2, loc.weights);
    if (rc < 0) { return -1; } // 退化单元

    loc.cellId = cellId;
    loc.subId = subId;
    loc.closestPoint = closest;
    loc.pcoords[0] = pcoords[0];
    loc.pcoords[1] = pcoords[1];
    loc.pcoords[2] = pcoords[2];
    loc.dist2 = dist2;
    loc.numWeights = n;
    loc.degraded = degraded;
    return (rc == 1) ? 1 : 0;
}


/** 结构化网格点索引：i 变化最快，其次 j，最后 k（与 iGameStructuredMesh 一致） */
inline igIndex StructuredPointId(const igIndex n[3], igIndex i, igIndex j, igIndex k) {
    return i + j * n[0] + k * n[0] * n[1];
}

/**
 * StructuredMesh → UnstructuredMesh
 *
 * 按 (ni, nj, nk) 中「尺寸大于 1 的维数」生成单元：
 *   0 维 → IG_VERTEX（单点）；1 维 → IG_LINE（曲线）；2 维 → IG_QUAD（平面）；3 维 → IG_HEXAHEDRON。
 * 节点序与 StructuredMesh::GenStructuredCellConnectivities() 保持一致。这里不修改输入网格，
 * 也不依赖其连接关系是否已经生成。
 */
UnstructuredMesh::Pointer MeshFromStructured(StructuredMesh* structured) {
    if (structured == nullptr || structured->GetPoints() == nullptr) { return nullptr; }

    igIndex n[3] = {1, 1, 1};
    igIndex* size = structured->GetDimensionSize();
    if (size != nullptr) {
        for (int a = 0; a < 3; ++a) { n[a] = std::max<igIndex>(1, size[a]); }
    }
    if (structured->GetNumberOfPoints() < n[0] * n[1] * n[2]) { return nullptr; }

    auto mesh = UnstructuredMesh::New();
    mesh->SetName(structured->GetName());
    mesh->SetPoints(structured->GetPoints());
    mesh->SetAttributeSet(structured->GetAttributeSet());

    auto cells = CellArray::New();
    auto types = UnsignedIntArray::New();
    const auto pid = [&n](igIndex i, igIndex j, igIndex k) { return StructuredPointId(n, i, j, k); };

    int varying[3] = {0, 0, 0};
    int nv = 0;
    for (int a = 0; a < 3; ++a) {
        if (n[a] > 1) { varying[nv++] = a; }
    }

    if (nv == 0) {
        // 仅一个点
        igIndex id = 0;
        cells->AddCellIds(&id, 1);
        types->AddValue(IG_VERTEX);
    } else if (nv == 1) {
        // 1D 曲线
        const int a = varying[0];
        for (igIndex t = 0; t + 1 < n[a]; ++t) {
            igIndex c[3] = {0, 0, 0};
            igIndex id[2];
            c[a] = t;
            id[0] = pid(c[0], c[1], c[2]);
            c[a] = t + 1;
            id[1] = pid(c[0], c[1], c[2]);
            cells->AddCellIds(id, 2);
            types->AddValue(IG_LINE);
        }
    } else if (nv == 2) {
        // 2D 平面（含 (1, nj, nk) 这类退化排布）
        const int a = varying[0], b = varying[1];
        for (igIndex tb = 0; tb + 1 < n[b]; ++tb) {
            for (igIndex ta = 0; ta + 1 < n[a]; ++ta) {
                igIndex c[3] = {0, 0, 0};
                igIndex id[4];
                c[a] = ta;
                c[b] = tb;
                id[0] = pid(c[0], c[1], c[2]);
                c[a] = ta + 1;
                id[1] = pid(c[0], c[1], c[2]);
                c[b] = tb + 1;
                id[2] = pid(c[0], c[1], c[2]);
                c[a] = ta;
                id[3] = pid(c[0], c[1], c[2]);
                cells->AddCellIds(id, 4);
                types->AddValue(IG_QUAD);
            }
        }
    } else {
        // 3D 六面体
        for (igIndex k = 0; k + 1 < n[2]; ++k) {
            for (igIndex j = 0; j + 1 < n[1]; ++j) {
                for (igIndex i = 0; i + 1 < n[0]; ++i) {
                    igIndex id[8];
                    id[0] = pid(i, j, k);
                    id[1] = pid(i + 1, j, k);
                    id[2] = pid(i + 1, j, k + 1);
                    id[3] = pid(i, j, k + 1);
                    id[4] = pid(i, j + 1, k);
                    id[5] = pid(i + 1, j + 1, k);
                    id[6] = pid(i + 1, j + 1, k + 1);
                    id[7] = pid(i, j + 1, k + 1);
                    cells->AddCellIds(id, 8);
                    types->AddValue(IG_HEXAHEDRON);
                }
            }
        }
    }

    mesh->SetCells(cells, types);
    return mesh;
}

/** PointSet（点云）→ UnstructuredMesh：每个点一个 IG_VERTEX 单元（等价 vtkPolyData 的 verts） */
UnstructuredMesh::Pointer MeshFromPointSet(PointSet* pointSet) {
    if (pointSet == nullptr || pointSet->GetPoints() == nullptr) { return nullptr; }

    auto mesh = UnstructuredMesh::New();
    mesh->SetName(pointSet->GetName());
    mesh->SetPoints(pointSet->GetPoints());
    mesh->SetAttributeSet(pointSet->GetAttributeSet());

    auto cells = CellArray::New();
    auto types = UnsignedIntArray::New();
    const IGsize pointNum = pointSet->GetNumberOfPoints();
    for (IGsize i = 0; i < pointNum; ++i) {
        igIndex id = static_cast<igIndex>(i);
        cells->AddCellIds(&id, 1);
        types->AddValue(IG_VERTEX);
    }
    mesh->SetCells(cells, types);
    return mesh;
}

/** LagrangeUnstructuredMesh → UnstructuredMesh：保留 IG_LAGRANGE_* 单元类型 */
UnstructuredMesh::Pointer MeshFromLagrange(LagrangeUnstructuredMesh* lagrange) {
    if (lagrange == nullptr || lagrange->GetPoints() == nullptr) { return nullptr; }

    auto mesh = UnstructuredMesh::New();
    mesh->SetName(lagrange->GetName());
    mesh->SetPoints(lagrange->GetPoints());
    mesh->SetAttributeSet(lagrange->GetAttributeSet());

    auto cells = CellArray::New();
    auto types = UnsignedIntArray::New();
    const IGsize cellNum = lagrange->GetNumberOfCells();
    for (IGsize c = 0; c < cellNum; ++c) {
        const igIndex* ids = nullptr;
        const int cnt = lagrange->GetCellPointIds(c, ids);
        // 即使单元退化也要占位，保证单元序号与 Cell Data 对齐
        cells->AddCellIds((cnt > 0) ? ids : nullptr, std::max(0, cnt));
        types->AddValue(lagrange->GetSpecificCellType(c));
    }
    mesh->SetCells(cells, types);
    return mesh;
}

/** 多块合并时需要的属性布局（同名 + 同附着类型视为同一个输出属性） */
struct SourceAttributeLayout {
    std::string name;
    IGenum type{IG_NONE};
    IGenum attachmentType{IG_POINT};
    IGenum arrayType{IG_FloatArray};
    int dimension{1};
};

void CollectSourceAttributeLayout(AttributeSet* set, std::vector<SourceAttributeLayout>& layouts) {
    if (set == nullptr) { return; }
    auto all = set->GetAllAttributes();
    if (all == nullptr) { return; }

    for (IGsize i = 0; i < all->GetNumberOfElements(); ++i) {
        auto& attr = all->GetElement(i);
        if (attr.isDeleted || attr.pointer == nullptr) { continue; }
        if (attr.attachmentType != IG_POINT && attr.attachmentType != IG_CELL) { continue; }
        const int dim = attr.pointer->GetDimension();
        if (dim <= 0) { continue; }

        bool found = false;
        for (auto& layout : layouts) {
            if (layout.name == attr.pointer->GetName() && layout.attachmentType == attr.attachmentType) {
                layout.dimension = std::max(layout.dimension, dim);
                found = true;
                break;
            }
        }
        if (found) { continue; }

        SourceAttributeLayout layout;
        layout.name = attr.pointer->GetName();
        layout.type = attr.type;
        layout.attachmentType = attr.attachmentType;
        layout.arrayType = attr.pointer->GetArrayType();
        layout.dimension = dim;
        layouts.push_back(std::move(layout));
    }
}

/**
 * 多块网格合并为一个 UnstructuredMesh（等价先做 Merge Blocks 再采样）：
 * 点 / 单元按块顺序追加并做序号偏移，属性按「名字 + 附着类型」取并集后逐块填写。
 * 采样点同时落在多个块中时，序号靠前的块先命中（LocateSample 按单元序号取第一个命中）。
 */
UnstructuredMesh::Pointer MergeSourceBlocks(const std::vector<UnstructuredMesh::Pointer>& blocks) {
    if (blocks.empty()) { return nullptr; }
    if (blocks.size() == 1) { return blocks[0]; }

    IGsize totalPoints = 0;
    IGsize totalCells = 0;
    std::vector<SourceAttributeLayout> layouts;
    for (const auto& block : blocks) {
        if (block == nullptr) { continue; }
        totalPoints += block->GetNumberOfPoints();
        totalCells += block->GetNumberOfCells();
        CollectSourceAttributeLayout(block->GetAttributeSet(), layouts);
    }

    auto mesh = UnstructuredMesh::New();
    for (const auto& block : blocks) {
        if (block != nullptr) {
            mesh->SetName(block->GetName());
            break;
        }
    }

    // 1) 合并点
    auto points = Points::New();
    for (const auto& block : blocks) {
        if (block == nullptr) { continue; }
        const IGsize pointNum = block->GetNumberOfPoints();
        for (IGsize i = 0; i < pointNum; ++i) { points->AddPoint(block->GetPoint(i)); }
    }
    mesh->SetPoints(points);

    // 2) 合并单元（点序号偏移）
    auto cells = CellArray::New();
    auto types = UnsignedIntArray::New();
    std::vector<igIndex> ids;
    igIndex pointOffset = 0;
    for (const auto& block : blocks) {
        if (block == nullptr) { continue; }
        const IGsize cellNum = block->GetNumberOfCells();
        for (IGsize c = 0; c < cellNum; ++c) {
            const igIndex* cellIds = nullptr;
            const int cnt = block->GetCellPointIds(c, cellIds);
            ids.resize(static_cast<size_t>(std::max(0, cnt)));
            for (int k = 0; k < cnt; ++k) { ids[static_cast<size_t>(k)] = cellIds[k] + pointOffset; }
            cells->AddCellIds((cnt > 0) ? ids.data() : nullptr, std::max(0, cnt));
            types->AddValue(block->GetCellType(c));
        }
        pointOffset += static_cast<igIndex>(block->GetNumberOfPoints());
    }
    mesh->SetCells(cells, types);

    // 3) 合并属性：先按总长度建数组（多出的位置由 Resize 置 0），再逐块填写
    AttributeSet::Pointer outSet = AttributeSet::New();
    std::vector<ArrayObject::Pointer> arrays;
    arrays.reserve(layouts.size());
    for (const auto& layout : layouts) {
        auto array = ProbeFilter::CreateArrayObject(layout.arrayType);
        if (array == nullptr) {
            arrays.push_back(nullptr);
            continue;
        }
        array->SetName(layout.name);
        array->SetDimension(layout.dimension);
        array->Resize(layout.attachmentType == IG_POINT ? totalPoints : totalCells);
        arrays.push_back(array);
    }

    std::vector<double> values;
    igIndex pointBase = 0;
    igIndex cellBase = 0;
    for (const auto& block : blocks) {
        if (block == nullptr) { continue; }
        AttributeSet* set = block->GetAttributeSet();
        const IGsize pointNum = block->GetNumberOfPoints();
        const IGsize cellNum = block->GetNumberOfCells();

        if (set != nullptr) {
            for (size_t a = 0; a < layouts.size(); ++a) {
                if (arrays[a] == nullptr) { continue; }
                const SourceAttributeLayout& layout = layouts[a];
                const bool isPoint = (layout.attachmentType == IG_POINT);
                const IGsize count = isPoint ? pointNum : cellNum;
                const igIndex base = isPoint ? pointBase : cellBase;

                ArrayObject* src = set->GetArrayPointer(IG_NONE, layout.attachmentType, layout.name);
                if (src == nullptr || src->GetDimension() <= 0) { continue; }

                values.assign(static_cast<size_t>(layout.dimension), 0.0);
                for (IGsize i = 0; i < count; ++i) {
                    std::fill(values.begin(), values.end(), 0.0);
                    src->GetElement(i, values.data());
                    arrays[a]->SetElement(base + i, values.data());
                }
            }
        }

        pointBase += static_cast<igIndex>(pointNum);
        cellBase += static_cast<igIndex>(cellNum);
    }

    for (size_t a = 0; a < layouts.size(); ++a) {
        if (arrays[a] == nullptr) { continue; }
        outSet->AddAttribute(layouts[a].type, layouts[a].attachmentType, arrays[a]);
    }
    mesh->SetAttributeSet(outSet);
    return mesh;
}

/** 单个叶子网格 → UnstructuredMesh；不支持的类型返回 nullptr */
UnstructuredMesh::Pointer MeshFromLeaf(DataObject* object) {
    switch (object->GetDataObjectType()) {
        case IG_UNSTRUCTURED_MESH:
            return DynamicCast<UnstructuredMesh>(object);
        case IG_LAGRANGE_UNSTRUCTURED_MESH:
            return MeshFromLagrange(DynamicCast<LagrangeUnstructuredMesh>(object));
        case IG_SURFACE_MESH: {
            auto surface = DynamicCast<SurfaceMesh>(object);
            if (surface == nullptr) { return nullptr; }
            auto mesh = UnstructuredMesh::New();
            return mesh->GenerateFromSurfaceMesh(surface) ? mesh : nullptr;
        }
        case IG_VOLUME_MESH: {
            auto volume = DynamicCast<VolumeMesh>(object);
            if (volume == nullptr) { return nullptr; }
            auto mesh = UnstructuredMesh::New();
            return mesh->GenerateFromVolumeMesh(volume) ? mesh : nullptr;
        }
        case IG_STRUCTURED_MESH:
            return MeshFromStructured(DynamicCast<StructuredMesh>(object));
        case IG_POINT_SET:
            return MeshFromPointSet(DynamicCast<PointSet>(object));
        default:
            return nullptr;
    }
}

/**
 * 递归收集输入中的所有网格块：
 *   - 目标是具体网格类型 → 直接转换；
 *   - 目标是容器（DrawObject / DataObject / CompositeDataObject 等）→ 遍历子数据对象，
 *     使多帧时间序列、多 zone 文件这类「父对象 + 多个子网格」的输入也能整体采样。
 */
void CollectSourceBlocks(DataObject* object, std::vector<UnstructuredMesh::Pointer>& blocks, int depth) {
    if (object == nullptr || depth > 8) { return; }

    switch (object->GetDataObjectType()) {
        case IG_UNSTRUCTURED_MESH:
        case IG_LAGRANGE_UNSTRUCTURED_MESH:
        case IG_SURFACE_MESH:
        case IG_VOLUME_MESH:
        case IG_STRUCTURED_MESH:
        case IG_POINT_SET: {
            auto mesh = MeshFromLeaf(object);
            if (mesh != nullptr) { blocks.push_back(mesh); }
            return;
        }
        default:
            break;
    }

    if (auto composite = DynamicCast<CompositeDataObject>(object)) {
        const unsigned int childNum = composite->GetNumberOfChildren();
        for (unsigned int i = 0; i < childNum; ++i) {
            CollectSourceBlocks(composite->GetChild(static_cast<int>(i)), blocks, depth + 1);
        }
        if (childNum > 0) { return; }
    }

    if (object->GetNumberOfSubDataObjects() > 0) {
        for (auto it = object->SubDataObjectIteratorBegin(); it != object->SubDataObjectIteratorEnd(); ++it) {
            CollectSourceBlocks(it->second.GetPointer(), blocks, depth + 1);
        }
    }
}
} // namespace

/**
 * 归一化入口：任意受支持网格 / 容器 → 一份 UnstructuredMesh。
 * 单块输入不做拷贝（直接返回转换结果）；blockCount 返回合并前的块数。
 */
UnstructuredMesh::Pointer ProbeFilter::BuildSourceMesh(DataObject* object, int* blockCount) {
    if (blockCount != nullptr) { *blockCount = 0; }
    if (object == nullptr) { return nullptr; }

    std::vector<UnstructuredMesh::Pointer> blocks;
    CollectSourceBlocks(object, blocks, 0);
    if (blockCount != nullptr) { *blockCount = static_cast<int>(blocks.size()); }
    if (blocks.empty()) { return nullptr; }
    return MergeSourceBlocks(blocks);
}



int ProbeFilter::EvaluateCell(IGenum cellType, const std::vector<Point>& cellPoints, const Point& x,
                              double* weights, int& numWeights, double pcoords[3],
                              Point& closestPoint, double& dist2) {
    numWeights = 0;
    pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
    dist2 = 0.0;
    const int n = static_cast<int>(cellPoints.size());
    if (weights == nullptr || n <= 0 || n > IGAME_CELL_MAX_SIZE) { return -1; }

    Cell* cell = MakeCellForType(cellType);
    if (cell == nullptr) { return -1; } // 需要面拓扑（Polyhedron）或未实现类型

    cell->Reset();
    // 固定点数类型返回常量（Hexahedron=8 等）；动态类型（Polygon）Reset 后为 0，跳过最小点数检查
    const int need = cell->GetCellSize();
    if (need > 0 && n < need) { return -1; }

    for (int i = 0; i < n; ++i) {
        cell->m_Points->AddPoint(cellPoints[i][0], cellPoints[i][1], cellPoints[i][2]);
        cell->m_PointIds->AddId(i);
    }
    numWeights = (cell->GetCellSize() > 0) ? cell->GetCellSize() : n;

    int subId = 0;
    return cell->EvaluatePosition(x, closestPoint, subId, pcoords, dist2, weights);
}

const char* ProbeFilter::SourceTypeName(IGenum type) {
    switch (type) {
        case IG_DATA_OBJECT: return "DataObject";
        case IG_COMPOSITE_DATA_OBJECT: return "CompositeDataObject";
        case IG_DRAW_OBJECT: return "DrawObject";
        case IG_POINT_SET: return "PointSet";
        case IG_SURFACE_MESH: return "SurfaceMesh";
        case IG_VOLUME_MESH: return "VolumeMesh";
        case IG_UNSTRUCTURED_MESH: return "UnstructuredMesh";
        case IG_STRUCTURED_MESH: return "StructuredMesh";
        case IG_MULTIBLOCK_MESH: return "MultiBlockMesh";
        case IG_SPLINE_GEOMETRY: return "SplineGeometry";
        case IG_LAGRANGE_UNSTRUCTURED_MESH: return "LagrangeUnstructuredMesh";
        default: return "Unknown";
    }
}

ArrayObject::Pointer ProbeFilter::CreateArrayObject(IGenum arrayType) {
    switch (arrayType) {
        case IG_FloatArray: return FloatArray::New();
        case IG_DoubleArray: return DoubleArray::New();
        case IG_IntArray: return IntArray::New();
        case IG_UnsignedIntArray: return UnsignedIntArray::New();
        case IG_CharArray: return CharArray::New();
        case IG_UnsignedCharArray: return UnsignedCharArray::New();
        case IG_ShortArray: return ShortArray::New();
        case IG_UnsignedShortArray: return UnsignedShortArray::New();
        case IG_LongLongArray: return LongLongArray::New();
        case IG_UnsignedLongLongArray: return UnsignedLongLongArray::New();
        default: return FloatArray::New();
    }
}

double ProbeFilter::DefaultTolerance(const UnstructuredMesh::Pointer& mesh) {
    // VTK：tol2 = 包围盒对角线² × 1e-6，等价于绝对容差 = 对角线 × 1e-3
    const double diag = (mesh.IsNull()) ? 1.0 : std::max(mesh->GetBoundingBox().diag(), 1e-12);
    return diag * kVTKToleranceFactor;
}

bool ProbeFilter::LocatePositions(const UnstructuredMesh::Pointer& mesh,
                                 const std::vector<Point>& positions,
                                 const ProbeOptions& options,
                                 std::vector<SampleLocation>& out,
                                 std::vector<unsigned char>* validMask) {
    out.assign(positions.size(), SampleLocation{});
    if (validMask != nullptr) { validMask->assign(positions.size(), 0); }
    return LocatePositionsRange(mesh, positions, 0, positions.size(), options, out, validMask);
}

bool ProbeFilter::LocatePositionsRange(const UnstructuredMesh::Pointer& mesh,
                                       const std::vector<Point>& positions,
                                       size_t begin,
                                       size_t end,
                                       const ProbeOptions& options,
                                       std::vector<SampleLocation>& out,
                                       std::vector<unsigned char>* validMask) {
    if (mesh.IsNull() || mesh->GetNumberOfCells() == 0) { return false; }
    if (positions.empty()) { return true; }
    // 调用方负责预分配；越界/非法范围直接视为无事可做
    if (out.size() < positions.size()) { return false; }
    if (validMask != nullptr && validMask->size() < positions.size()) { return false; }
    if (begin >= end || end > positions.size()) { return true; }

    const double tol = (options.tolerance < 0.0) ? DefaultTolerance(mesh) : options.tolerance;
    const double tol2 = tol * tol;
    const bool snapping = options.snapToCellWithClosestPoint;
    const double snapRadius2 =
            (options.snappingRadius > 0.0) ? (options.snappingRadius * options.snappingRadius) : -1.0;
    const BoundingBox& bbox = mesh->GetBoundingBox();

    // 空间索引缓存：同一份源网格 + 同一容差下只建一次（等价 VTK 把 locator 缓存在 dataset 上）。
    // 校验用「对象地址 + MTime + 点数/单元数 + 容差 + 包围盒」；MTime 是全局单调计数器，
    // 因此不会因为对象析构后地址被复用而误命中；缓存只保存地址与指纹，解引用始终用调用方传入的 mesh。
    struct GridCache {
        const void* meshKey{nullptr};
        unsigned int meshMTime{0};
        IGsize pointCount{0};
        IGsize cellCount{0};
        double tolerance{-1.0};
        double boundsMin[3]{0.0, 0.0, 0.0};
        double boundsMax[3]{0.0, 0.0, 0.0};
        bool valid{false};
        CellGrid data;
        bool unsafeForParallel{false};
    };
    thread_local GridCache s_gridCache;

    const IGsize meshPointCount = mesh->GetNumberOfPoints();
    const IGsize meshCellCount = mesh->GetNumberOfCells();
    const unsigned int meshMTime = mesh->GetMTime();
    const void* meshKey = static_cast<const void*>(mesh.GetPointer());

    GridCache& cache = s_gridCache;
    bool cacheHit = cache.valid && cache.meshKey == meshKey && cache.meshMTime == meshMTime &&
                    cache.pointCount == meshPointCount && cache.cellCount == meshCellCount &&
                    cache.tolerance == tol;
    if (cacheHit) {
        for (int j = 0; j < 3 && cacheHit; ++j) {
            cacheHit = cache.boundsMin[j] == static_cast<double>(bbox.min[j]) &&
                       cache.boundsMax[j] == static_cast<double>(bbox.max[j]);
        }
    }
    // 缓存内存上限：索引本身是 O(单元数)（每单元 6 个 double 的包围盒 + 桶内引用），
    // 超出阈值时不进缓存（改用局部索引，调用结束即释放），避免长期占用内存。
    constexpr IGsize kMaxCachedCells = 4000000; // 约 240 MB 量级
    const bool cacheable = meshCellCount <= kMaxCachedCells;

    CellGrid localGrid;
    bool localUnsafeForParallel = false;
    const CellGrid* gridPtr = nullptr;
    bool unsafeForParallel = false;

    if (cacheable && cacheHit) {
        gridPtr = &cache.data;
        unsafeForParallel = cache.unsafeForParallel;
    } else if (cacheable) {
        BuildCellGrid(mesh, bbox, tol, cache.data, cache.unsafeForParallel);
        cache.meshKey = meshKey;
        cache.meshMTime = meshMTime;
        cache.pointCount = meshPointCount;
        cache.cellCount = meshCellCount;
        cache.tolerance = tol;
        for (int j = 0; j < 3; ++j) {
            cache.boundsMin[j] = static_cast<double>(bbox.min[j]);
            cache.boundsMax[j] = static_cast<double>(bbox.max[j]);
        }
        cache.valid = true;
        gridPtr = &cache.data;
        unsafeForParallel = cache.unsafeForParallel;
    } else {
        BuildCellGrid(mesh, bbox, tol, localGrid, localUnsafeForParallel);
        gridPtr = &localGrid;
        unsafeForParallel = localUnsafeForParallel;
    }
    // 注意：热循环里必须用**普通指针**访问索引数据。若每次都直接访问 cache.data（thread_local），
    // TLS 间接寻址会让定位明显变慢。
    ArrayObject* ghost = FindCellGhostArray(mesh->GetAttributeSet());

    // 每个线程一份 scratch：
    //   · 候选去重集合：环形扩张时同一单元可能落在多个体素里（等价 VTK 定位器内的候选去重）。
    //     改成**定容开放寻址哈希 + 代计数器**，内存 O(1)，不再是原先 O(单元数) 的 stamp 数组
    //     （原先千万单元量级下每线程约 80 MB）。表满时允许重复求值——只影响性能，不影响结果。
    //   · lastCell 提示（VTK 的 FindCell(x, lastCellId)）
    struct LocateScratch {
        enum : int { kVisitedSlots = 1024 }; // 必须是 2 的幂
        struct Entry {
            igIndex cellId{-1};
            unsigned long long generation{0};
        };
        Entry visited[kVisitedSlots];
        unsigned long long generation{0};
        igIndex lastCellId{-1};

        void NextGeneration() { ++generation; }

        /**
         * 返回 true 表示该单元本点尚未见过（应继续求值）；false 表示已见过（跳过）。
         * 探测长度有上限：表很满时直接返回 true（允许重复求值），保证单次调用 O(1)，
         * 结果不受影响（重复求值只是多算一次，命中判定与取最近的过程都是幂等的）。
         *
         * 调用方应**先做便宜的单元包围盒预判**再调用本函数，否则"域内但不在任何单元里"的点
         * 会把整个网格的单元都塞进表里，导致探测退化（实测慢一个数量级）。
         */
        bool MarkVisited(igIndex cellId) {
            constexpr int kMaxProbe = 16;
            const size_t mask = static_cast<size_t>(kVisitedSlots) - 1;
            size_t slot = (static_cast<size_t>(cellId) * 2654435761u) & mask;
            for (int probe = 0; probe < kMaxProbe; ++probe) {
                Entry& e = visited[(slot + static_cast<size_t>(probe)) & mask];
                if (e.generation != generation) {
                    e.generation = generation;
                    e.cellId = cellId;
                    return true;
                }
                if (e.cellId == cellId) { return false; }
            }
            return true; // 探测过长：允许重复
        }
    };

    const igIndex cellNum = static_cast<igIndex>(mesh->GetNumberOfCells());

    // 源包围盒预判的余量：开启吸附时要按吸附半径放宽；吸附半径不限（<0）则不做预判
    const double bboxMargin = snapping
                                      ? ((options.snappingRadius > 0.0)
                                                 ? std::max(tol, options.snappingRadius)
                                                 : -1.0)
                                      : tol;

    // 单点定位：结果与线程调度无关（lastCell 只是提示；命中判定与容差均与顺序无关）
    const auto locateOne = [&](size_t i, LocateScratch& scratch) {
        const Point& p = positions[i];
        SampleLocation& loc = out[i];
        igIndex& lastCellId = scratch.lastCellId;
        scratch.NextGeneration(); // 本点的去重集合从“新的一代”开始，O(1) 失效

        // 源包围盒（含余量）预判：域外点直接判无效（吸附不限半径时不做预判）
        if (bboxMargin >= 0.0) {
            bool outside = false;
            for (int j = 0; j < 3 && !outside; ++j) {
                if (static_cast<double>(p[j]) < static_cast<double>(bbox.min[j]) - bboxMargin ||
                    static_cast<double>(p[j]) > static_cast<double>(bbox.max[j]) + bboxMargin) {
                    outside = true;
                }
            }
            if (outside) { return; } // lambda 顶层：域外直接返回（等价原来的 continue）
        }

        // 1) 上一个命中单元作为提示（VTK 的 FindCell(x, lastCellId)）
        if (lastCellId >= 0 && lastCellId < cellNum && !IsBlankedCell(ghost, lastCellId)) {
            SampleLocation hint;
            const int rc = EvaluateCellAt(mesh, lastCellId, p, hint);
            // 与 VTK 一致：0/1/2 维单元的「参数域内」不足以判定命中，距离还必须在容差内
            if (rc == 1 && hint.dist2 <= tol2) {
                loc = hint;
                if (validMask != nullptr) { (*validMask)[i] = 1; }
                return; // 提示命中
            }
        }

        igIndex foundCell = -1;
        SampleLocation best;
        double bestDist2 = DBL_MAX;
        bool accepted = false;

        if (!gridPtr->Valid()) {
            // 退化网格：退化为全单元扫描（保证正确性）
            for (igIndex cellId = 0; cellId < cellNum && !accepted; ++cellId) {
                if (IsBlankedCell(ghost, cellId)) { continue; }
                SampleLocation cand;
                const int rc = EvaluateCellAt(mesh, cellId, p, cand);
                if (rc == 1 && cand.dist2 <= tol2) { best = cand; accepted = true; }
            }
        } else {
            const igIndex ix = std::max<igIndex>(0, std::min<igIndex>(
                    gridPtr->nx - 1, static_cast<igIndex>((static_cast<double>(p[0]) - gridPtr->origin[0]) / gridPtr->voxel[0])));
            const igIndex iy = std::max<igIndex>(0, std::min<igIndex>(
                    gridPtr->ny - 1, static_cast<igIndex>((static_cast<double>(p[1]) - gridPtr->origin[1]) / gridPtr->voxel[1])));
            const igIndex iz = std::max<igIndex>(0, std::min<igIndex>(
                    gridPtr->nz - 1, static_cast<igIndex>((static_cast<double>(p[2]) - gridPtr->origin[2]) / gridPtr->voxel[2])));

            const igIndex maxRing = std::max<igIndex>(gridPtr->nx, std::max<igIndex>(gridPtr->ny, gridPtr->nz));
            // 吸附时只需要扫到吸附半径覆盖的环；否则扫满整个网格
            igIndex ringLimit = maxRing;
            if (snapping && options.snappingRadius > 0.0) {
                const double minVoxel = std::max(1e-12, std::min(gridPtr->voxel[0],
                                                                 std::min(gridPtr->voxel[1], gridPtr->voxel[2])));
                ringLimit = std::min<igIndex>(maxRing,
                                              static_cast<igIndex>(std::ceil(options.snappingRadius / minVoxel)) + 1);
            }
            // 吸附备选：半径内最近的单元（VTK SnapToCellWithClosestPoint）
            igIndex snapCellId = -1;
            Point snapClosest;
            double snapDist2 = DBL_MAX;

            for (igIndex r = 0; r <= ringLimit && !accepted; ++r) {
                // 目前最近距离已经小于本环可能的最小距离 → 后续环不可能更近，可提前结束
                if (snapping && !accepted && snapDist2 < DBL_MAX) {
                    const double minRingDist = (r > 0) ? (static_cast<double>(r) - 1.0) *
                                                         std::min(gridPtr->voxel[0], std::min(gridPtr->voxel[1], gridPtr->voxel[2]))
                                                       : 0.0;
                    if (minRingDist * minRingDist > snapDist2) { break; }
                }
                for (igIndex dx = -r; dx <= r && !accepted; ++dx) {
                    for (igIndex dy = -r; dy <= r && !accepted; ++dy) {
                        for (igIndex dz = -r; dz <= r && !accepted; ++dz) {
                            if (std::max(std::abs(dx), std::max(std::abs(dy), std::abs(dz))) != r) { continue; }
                            const igIndex cx = ix + dx, cy = iy + dy, cz = iz + dz;
                            if (cx < 0 || cx >= gridPtr->nx || cy < 0 || cy >= gridPtr->ny || cz < 0 || cz >= gridPtr->nz) { continue; }
                            const size_t v = gridPtr->VoxelIndex(cx, cy, cz);
                            for (igIndex k = gridPtr->offsets[v]; k < gridPtr->offsets[v + 1]; ++k) {
                                const igIndex cellId = gridPtr->cells[static_cast<size_t>(k)];
                                if (cellId < 0) { continue; }

                                // 1) 单元包围盒预判（最便宜，先做；吸附时按吸附半径外扩判定）
                                const double* box = &gridPtr->boxes[static_cast<size_t>(cellId) * 6];
                                const double pad = snapping ? std::max(0.0, (options.snappingRadius > 0.0 ?
                                                                              options.snappingRadius : tol))
                                                            : tol;
                                if (static_cast<double>(p[0]) < box[0] - pad || static_cast<double>(p[0]) > box[3] + pad ||
                                    static_cast<double>(p[1]) < box[1] - pad || static_cast<double>(p[1]) > box[4] + pad ||
                                    static_cast<double>(p[2]) < box[2] - pad || static_cast<double>(p[2]) > box[5] + pad) {
                                    continue;
                                }
                                // 2) 去重：只对通过包围盒预判（即可能真正求值）的单元去重
                                if (!scratch.MarkVisited(cellId)) { continue; }
                                if (IsBlankedCell(ghost, cellId)) { continue; }

                                SampleLocation cand;
                                const int rc = EvaluateCellAt(mesh, cellId, p, cand);
                                if (rc < 0) { continue; }
                                // 在单元内**且**距离在容差内：与 VTK 一致，直接命中
                                // （三维单元域内 dist2 == 0，退化为「是否在单元内」；0/1/2 维单元
                                //   还要看点到单元的距离，等价 vtkProbeFilter 的 dist2 ≤ tol2 判定）
                                if (rc == 1 && cand.dist2 <= tol2) {
                                    best = cand;
                                    foundCell = cellId;
                                    accepted = true;
                                    break;
                                }
                                // 其余（含「参数域内但超出容差」的 0/1/2 维单元）：按最近且 ≤ tol2 判定
                                if (cand.dist2 <= tol2 && cand.dist2 < bestDist2) {
                                    bestDist2 = cand.dist2;
                                    best = cand;
                                    foundCell = cellId;
                                }
                                // 吸附备选：半径内（半径 < 0 表示不限）最近的单元
                                if (snapping && cand.dist2 < snapDist2 &&
                                    (snapRadius2 < 0.0 || cand.dist2 <= snapRadius2)) {
                                    snapDist2 = cand.dist2;
                                    snapCellId = cellId;
                                    snapClosest = cand.closestPoint;
                                }
                            }
                        }
                    }
                }
            }

            // 吸附：把采样点吸附到最近单元的最近点，用最近点处的形函数取插值（VTK 的
            // FindClosestPointWithinRadius + 在最近点插值；本实现不移动采样点坐标）
            if (!accepted && snapping && snapCellId >= 0) {
                SampleLocation snapped;
                const int rc = EvaluateCellAt(mesh, snapCellId, snapClosest, snapped);
                if (rc >= 0) {
                    snapped.snapped = true;
                    snapped.dist2 = snapDist2; // 保留到原始采样点的距离，便于诊断
                    best = snapped;
                    foundCell = snapCellId;
                }
            }
        }

        if (foundCell >= 0) {
            loc = best;
            lastCellId = foundCell;
            if (validMask != nullptr) { (*validMask)[i] = 1; }
        } else {
            lastCellId = -1;
        }
    };

    // 单位置循环；只处理 [begin, end) 这一段（范围式 API 让调用方复用结果缓冲，
    // 分块上报进度时不再需要把每块的 SampleLocation 逐元素拷回总缓冲）。
    const size_t rangeCount = end - begin;
    const bool parallel = options.useParallel && !unsafeForParallel &&
                          rangeCount >= static_cast<size_t>(kSampleParallelThreshold);
    if (parallel) {
        ParallelChunks(static_cast<int>(rangeCount), [&](int chunkBegin, int chunkEnd) {
            LocateScratch scratch;
            for (int k = chunkBegin; k < chunkEnd; ++k) {
                locateOne(begin + static_cast<size_t>(k), scratch);
            }
        });
    } else {
        LocateScratch scratch;
        for (size_t i = begin; i < end; ++i) { locateOne(i, scratch); }
    }
    return true;
}

void ProbeFilter::InterpolateAttributes(const UnstructuredMesh::Pointer& mesh,
                                        AttributeSet::Pointer outSet,
                                        const std::vector<SampleLocation>& locations,
                                        const ProbeOptions& options) {
    if (mesh.IsNull() || outSet.IsNull()) { return; }
    AttributeSet* inAttrs = mesh->GetAttributeSet();
    if (inAttrs == nullptr) { return; }
    auto all = inAttrs->GetAllAttributes();
    if (all == nullptr) { return; }

    const int sampleNum = static_cast<int>(locations.size());

    std::vector<std::string> producedPointNames;

    if (options.interpolatePointData) {
        struct OutputArray {
            ArrayObject::Pointer src;
            ArrayObject::Pointer dst;
            int dimension{1};
        };
        std::vector<OutputArray> outs;
        for (IGsize i = 0; i < all->GetNumberOfElements(); ++i) {
            auto& attr = all->GetElement(i);
            if (attr.isDeleted || attr.pointer == nullptr) { continue; }
            if (attr.attachmentType != IG_POINT) { continue; }
            const int dim = attr.pointer->GetDimension();
            if (dim <= 0) { continue; }

            auto dst = CreateArrayObject(attr.pointer->GetArrayType()); // 保留源数组类型
            if (dst == nullptr) { continue; }
            dst->SetName(attr.pointer->GetName());
            dst->SetDimension(dim);
            dst->Resize(static_cast<IGsize>(sampleNum));
            outSet->AddAttribute(attr.type, IG_POINT, dst, attr.GetDataRange());

            producedPointNames.push_back(attr.pointer->GetName());
            outs.push_back({attr.pointer, dst, dim});
        }

        // 采样点外层、数组内层：每个采样点只取一次命中单元（等价 VTK worklet 的做法）。
        // 与定位同样并行：每线程独立缓冲、写 out 的互不相交下标；数组 SetElement 只写自身缓冲区，
        // 无共享状态。源网格含"只能用共享单元对象"的类型（PolyLine/Polyhedron）时退回串行。
        const auto interpolateOne = [&](int j, std::vector<double>& values, std::vector<double>& raw) {
            const SampleLocation& loc = locations[static_cast<size_t>(j)];
            // 与定位阶段用同一个 KernelCellFor：退化单元（EmptyCell → 角点线性）也能拿到正确的全局点 id
            int npts = 0;
            Cell* cell = (loc.cellId >= 0) ? KernelCellFor(mesh, loc.cellId, npts, nullptr) : nullptr;
            const bool valid = (cell != nullptr) && (npts > 0) && (loc.numWeights == npts);

            for (auto& out : outs) {
                values.assign(static_cast<size_t>(out.dimension), 0.0);
                if (valid) {
                    raw.resize(static_cast<size_t>(out.dimension));
                    for (int v = 0; v < npts; ++v) {
                        const double w = loc.weights[v];
                        if (w == 0.0) { continue; }
                        out.src->GetElement(static_cast<IGsize>(cell->GetPointId(v)), raw.data());
                        for (int c = 0; c < out.dimension; ++c) {
                            values[static_cast<size_t>(c)] += w * raw[static_cast<size_t>(c)];
                        }
                    }
                }
                out.dst->SetElement(static_cast<IGsize>(j), values.data());
            }
        };

        bool unsafeForParallel = false;
        if (options.useParallel && sampleNum >= static_cast<int>(kSampleParallelThreshold)) {
            const IGsize cellCount = mesh->GetNumberOfCells();
            for (IGsize cid = 0; cid < cellCount && !unsafeForParallel; ++cid) {
                unsafeForParallel = RequiresMeshCell(mesh->GetCellType(cid));
            }
        }
        const bool parallel = options.useParallel && !unsafeForParallel &&
                              sampleNum >= static_cast<int>(kSampleParallelThreshold);
        if (parallel) {
            ParallelChunks(sampleNum, [&](int begin, int end) {
                std::vector<double> values;
                std::vector<double> raw;
                for (int j = begin; j < end; ++j) { interpolateOne(j, values, raw); }
            });
        } else {
            std::vector<double> values;
            std::vector<double> raw;
            for (int j = 0; j < sampleNum; ++j) { interpolateOne(j, values, raw); }
        }
    }

    // 源单元数据 → 输出点数据（VTK 的 InputCellArrays；同名时点数据优先）
    if (options.copyCellData) {
        std::vector<double> values;
        for (IGsize i = 0; i < all->GetNumberOfElements(); ++i) {
            auto& attr = all->GetElement(i);
            if (attr.isDeleted || attr.pointer == nullptr) { continue; }
            if (attr.attachmentType != IG_CELL) { continue; }

            auto src = attr.pointer;
            const std::string name = src->GetName();
            if (std::find(producedPointNames.begin(), producedPointNames.end(), name) !=
                producedPointNames.end()) {
                continue; // 点数据优先
            }
            const int dim = src->GetDimension();
            if (dim <= 0) { continue; }

            auto dst = CreateArrayObject(src->GetArrayType());
            if (dst == nullptr) { continue; }
            dst->SetName(name);
            dst->SetDimension(dim);
            dst->Resize(static_cast<IGsize>(sampleNum));

            const IGsize cellNum = src->GetNumberOfElements();
            values.assign(static_cast<size_t>(dim), 0.0);
            for (int j = 0; j < sampleNum; ++j) {
                std::fill(values.begin(), values.end(), 0.0); // 无效采样点填 0
                const igIndex cellId = locations[static_cast<size_t>(j)].cellId;
                if (cellId >= 0 && static_cast<IGsize>(cellId) < cellNum) {
                    src->GetElement(static_cast<IGsize>(cellId), values.data());
                }
                dst->SetElement(static_cast<IGsize>(j), values.data());
            }
            outSet->AddAttribute(attr.type, IG_POINT, dst, attr.GetDataRange());
        }
    }

    // 掩膜：1 = 有效，0 = 无效（VTK 的 vtkValidPointMask；本工程默认名与类型见 ProbeOptions）
    if (!options.maskArrayName.empty()) {
        auto mask = UnsignedCharArray::New();
        mask->SetName(options.maskArrayName);
        mask->SetDimension(1);
        mask->Resize(static_cast<IGsize>(sampleNum));
        for (int j = 0; j < sampleNum; ++j) {
            const unsigned char v = (locations[static_cast<size_t>(j)].cellId >= 0) ? 1 : 0;
            mask->SetValue(static_cast<IGsize>(j), static_cast<double>(v));
        }
        outSet->AddAttribute(IG_SCALAR, IG_POINT, mask);
    }
}

bool ProbeFilter::Execute() {
    // ================= 取输入并校验 =================
    auto in = GetInput(0);
    auto query = DynamicCast<PointSet>(GetInput(1));
    if (in.IsNull() || query.IsNull()) return false;

    auto srcPoints = in->GetPoints();
    auto queryPoints = query->GetPoints();
    if (srcPoints.IsNull() || srcPoints->GetNumberOfPoints() == 0) return false;
    if (queryPoints.IsNull() || queryPoints->GetNumberOfPoints() == 0) return false;

    const IGsize numQuery = queryPoints->GetNumberOfPoints();

    // ================= 源网格归一化（采样内核输入）=================
    // 容差由采样内核按 VTK 语义处理：HasAutoTolerance 时为源包围盒对角线 × 1e-3
    UnstructuredMesh::Pointer mesh = DynamicCast<UnstructuredMesh>(in);
    if (mesh.IsNull()) { mesh = UnstructuredMesh::TransDataObjToUnstructuredMesh(in); }
    if (mesh.IsNull()) return false;

    // ================= 输出属性准备（原地写入查询点集）=================
    AttributeSet* inAttrs = mesh->GetAttributeSet();
    AttributeSet* outAttrs = query->GetAttributeSet();
    if (inAttrs == nullptr || outAttrs == nullptr) return false;

    struct OutAttribute {
        ArrayObject::Pointer inArray;
        FloatArray::Pointer outArray;
        int dimension;
    };
    std::vector<OutAttribute> outAttributes;

    {
        auto allAttributes = inAttrs->GetAllPointAttributes();
        for (IGsize i = 0; i < allAttributes->GetNumberOfElements(); ++i) {
            auto& attr = allAttributes->GetElement(i);
            if (attr.isDeleted || attr.pointer.IsNull()) continue;
            if (attr.attachmentType != IG_POINT) continue;  // 只插值点属性

            const std::string name = attr.pointer->GetName();
            const int dimension = std::max(1, attr.pointer->GetDimension());

            FloatArray::Pointer outArray = nullptr;
            const int existingIndex = outAttrs->GetAttributeIndex(name);
            if (existingIndex >= 0) {
                auto existing = DynamicCast<FloatArray>(
                    outAttrs->GetAttribute(existingIndex).pointer);
                if (!existing.IsNull()) {
                    outArray = existing;
                } else {
                    outAttrs->DeleteAttribute(existingIndex);  // 类型不符，删除重建
                }
            }
            if (outArray.IsNull()) {
                outArray = FloatArray::New();
                outArray->SetName(name);
                outArray->SetDimension(dimension);
                outArray->Resize(numQuery);
                outAttrs->AddAttribute(attr.type, IG_POINT, outArray,
                                       attr.GetDataRange());
            } else {
                if (outArray->GetDimension() != dimension) {
                    outArray->SetDimension(dimension);
                }
                outArray->Resize(numQuery);
            }
            outAttributes.push_back({attr.pointer, outArray, dimension});
        }
    }

    // ValidPointMask：找到单元 = 1，未找到 = 0
    IntArray::Pointer mask = nullptr;
    {
        const int maskIndex = outAttrs->GetAttributeIndex("ValidPointMask");
        if (maskIndex >= 0) {
            auto existing = DynamicCast<IntArray>(
                outAttrs->GetAttribute(maskIndex).pointer);
            if (!existing.IsNull()) {
                mask = existing;
            } else {
                outAttrs->DeleteAttribute(maskIndex);
            }
        }
        if (mask.IsNull()) {
            mask = IntArray::New();
            mask->SetName("ValidPointMask");
            mask->SetDimension(1);
            mask->Resize(numQuery);
            outAttrs->AddAttribute(IG_SCALAR, IG_POINT, mask);
        } else {
            mask->Resize(numQuery);
        }
    }

    // ================= 定位（采样内核：CSR 网格 + 单元类型驱动 + VTK 容差语义） =====
    std::vector<Point> positions(static_cast<size_t>(numQuery));
    for (IGsize i = 0; i < numQuery; ++i) { positions[static_cast<size_t>(i)] = queryPoints->GetPoint(i); }

    ProbeOptions options;
    options.tolerance = HasAutoTolerance() ? -1.0 : m_Tolerance;
    options.maskArrayName = "ValidPointMask"; // 保持 UI 既有命名（重采样滤波用默认 validpointmask）

    std::vector<SampleLocation> locations;
    std::vector<unsigned char> validMask;
    if (!LocatePositions(mesh, positions, options, locations, &validMask)) { return false; }

    // ================= 逐采样点写回（原地更新查询点集）=================
    double rawValues[IGAME_CELL_MAX_SIZE] = {};
    double interpolated[IGAME_CELL_MAX_SIZE] = {};
    for (IGsize qi = 0; qi < numQuery; ++qi) {
        const SampleLocation& loc = locations[static_cast<size_t>(qi)];
        Cell* cell = (loc.cellId >= 0) ? mesh->GetCell(loc.cellId) : nullptr;
        const int npts = (cell != nullptr) ? cell->GetCellSize() : 0;
        const bool found = (cell != nullptr) && (npts > 0) && (loc.numWeights == npts);

        if (found) {
            for (auto& out : outAttributes) {
                const int dimension = out.dimension;
                for (int c = 0; c < dimension; ++c) { interpolated[c] = 0.0; }
                for (int v = 0; v < npts; ++v) {
                    const double w = loc.weights[v];
                    if (w == 0.0) { continue; }
                    out.inArray->GetElement(cell->GetPointId(v), rawValues);
                    for (int c = 0; c < dimension; ++c) {
                        interpolated[c] += w * rawValues[c];
                    }
                }
                out.outArray->SetElement(qi, interpolated);
            }
        }

        mask->SetValue(qi, found ? 1 : 0);
        if (!found) {
            double zeros[IGAME_CELL_MAX_SIZE] = {};
            for (auto& out : outAttributes) {
                out.outArray->SetElement(qi, zeros);
            }
        }
    }

    query->Modified();

    // 输出 0 与输入 1 为同一个对象（原地更新，不新建点集）
    SetOutput(0, query);
    return true;
}

IGAME_NAMESPACE_END
