#include "iGameResampleToLine.h"

#include "Probe/iGameProbeFilter.h"
#include "iGameAttributeSet.h"
#include "iGameCellArray.h"
#include "iGameCompositeDataObject.h"
#include "iGameFlatArray.h"
#include "iGameLagrangeUnstructuredMesh.h"
#include "iGamePointSet.h"
#include "iGamePoints.h"
#include "iGameStructuredMesh.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

IGAME_NAMESPACE_BEGIN

namespace {

} // namespace

/* ------------------------------------------------------------------ */
/* 参数设置                                                            */
/* ------------------------------------------------------------------ */
void ResampleToLine::setOrigTarget(const Vector3d& p0, const Vector3d& p1, const int& x) {
    orig = Point(static_cast<float>(p0[0]), static_cast<float>(p0[1]), static_cast<float>(p0[2]));
    target = Point(static_cast<float>(p1[0]), static_cast<float>(p1[1]), static_cast<float>(p1[2]));
    n = x;
}

/* ================================================================== */
/* 输入归一化：任意网格 → UnstructuredMesh                             */
/*                                                                    */
/* 与 vtkProbeFilter 一致，采样前把输入统一成「点 + 带类型的单元」形式。 */
/* ================================================================== */
namespace {

/** 数据对象类型名，用于失败/结果消息 */
const char* DataObjectTypeName(IGenum type) {
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

} // namespace

/* ------------------------------------------------------------------ */
/* 主流程                                                              */
/* ------------------------------------------------------------------ */
bool ResampleToLine::Execute() {
    auto input = GetInput(0);
    if (input == nullptr) {
        m_Message = "未选择输入数据";
        return false;
    }
    if (n < 2) {
        m_Message = "n must be greater than 1";
        return false;
    }

    m_LastInputType = input->GetDataObjectType();
    m_LastInputBlockCount = 0;

    // 1. 输入归一化：任意网格（含多块 / 容器）→ UnstructuredMesh
    //    与 ProbeFilter / ResampleWithDataSet 共用同一份实现（ProbeFilter 是唯一采样内核）
    int inputBlockCount = 0;
    auto mesh = ProbeFilter::BuildSourceMesh(input.GetPointer(), &inputBlockCount);
    m_LastInputBlockCount = inputBlockCount;
    if (mesh == nullptr) {
        m_Message = std::string("unsupported input type: ") + ProbeFilter::SourceTypeName(m_LastInputType) +
                    " (supported: PointSet / SurfaceMesh / VolumeMesh / UnstructuredMesh / StructuredMesh / "
                    "LagrangeUnstructuredMesh, or a container of them)";
        return false;
    }
    if (mesh->GetNumberOfPoints() == 0 || mesh->GetNumberOfCells() == 0) {
        m_Message = "Mesh has no cells or points.";
        return false;
    }

    const BoundingBox& bbox = mesh->GetBoundingBox();
    const double diag = std::max(bbox.diag(), 1e-12);

    // 容差：自动 = VTK 默认（源包围盒对角线 × 1e-3，等价 vtkProbeFilter 的 tol2 = 对角线²×1e-6）；
    // 手动为「相对对角线的比例」（对外 API 语义不变）
    m_EffectiveTolerance = m_AutoTolerance ? ProbeFilter::DefaultTolerance(mesh) : (m_Tolerance * diag);

    m_UnsupportedQuadraticCellCount = 0;

    // 2. 沿线均匀生成采样点
    Points::Pointer samples = Points::New();
    std::vector<Point> positions(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(n - 1);
        const Point p(static_cast<float>((1.0 - t) * orig[0] + t * target[0]),
                      static_cast<float>((1.0 - t) * orig[1] + t * target[1]),
                      static_cast<float>((1.0 - t) * orig[2] + t * target[2]));
        samples->AddPoint(p);
        positions[static_cast<size_t>(i)] = p;
    }

    // 3. 定位 + 权重 + 属性：全部交给 ProbeFilter 采样内核
    //    （与 ProbeFilter / ResampleWithDataSet 共用同一份实现：CSR 空间索引 + 单元类型自己的
    //      VTK 插值；二次单元直接由 QuadraticXxx::EvaluatePosition 求值，未实现插值的类型
    //      退化为角点线性。沿线采样不做吸附。）
    ProbeFilter::ProbeOptions options;
    options.tolerance = m_EffectiveTolerance;             // 绝对长度
    options.maskArrayName = m_ValidPointMaskArrayName;    // 默认 validpointmask
    options.snapToCellWithClosestPoint = false;

    std::vector<ProbeFilter::SampleLocation> kernelLocations;
    std::vector<unsigned char> validMask;
    if (!ProbeFilter::LocatePositions(mesh, positions, options, kernelLocations, &validMask)) {
        m_Message = "failed to locate sample points on the input mesh";
        return false;
    }

    m_SampleCellIds.assign(static_cast<size_t>(n), -1);
    m_SampleValidMask.assign(static_cast<size_t>(n), 0);
    int validCount = 0;
    for (int i = 0; i < n; ++i) {
        const size_t si = static_cast<size_t>(i);
        m_SampleCellIds[si] = kernelLocations[si].cellId;
        m_SampleValidMask[si] = validMask[si];
        if (validMask[si] != 0) { ++validCount; }
        // 角点线性退化（该单元类型尚未实现 VTK 插值）
        if (kernelLocations[si].degraded) { ++m_UnsupportedQuadraticCellCount; }
    }

    // 4. 插值 Point Data / 复制 Cell Data / 有效点掩码（内核按 VTK 语义统一处理）
    AttributeSet::Pointer outAttr = AttributeSet::New();
    ProbeFilter::InterpolateAttributes(mesh, outAttr, kernelLocations, options);

    // 6. 生成折线输出
    BuildPolyLineOutputs(samples, outAttr, n);

    m_Message = std::string("ResampleToLine: input=") + DataObjectTypeName(m_LastInputType) +
                (m_LastInputBlockCount > 1 ? (", blocks=" + std::to_string(m_LastInputBlockCount)) : "") +
                ", cells=" + std::to_string(mesh->GetNumberOfCells()) + ", samples=" + std::to_string(n) +
                ", valid=" + std::to_string(validCount) + ", invalid=" + std::to_string(n - validCount) +
                ", tolerance=" + std::to_string(m_EffectiveTolerance) +
                (m_AutoTolerance ? " (auto = bboxDiag*1e-3, VTK default)" : " (manual)");
    if (m_UnsupportedQuadraticCellCount > 0) {
        m_Message += ", degraded cells=" + std::to_string(m_UnsupportedQuadraticCellCount);
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* 均匀网格                                                            */
/* ------------------------------------------------------------------ */
std::vector<igIndex> ResampleToLine::GetValidPoints() const {
    std::vector<igIndex> ids;
    ids.reserve(m_SampleValidMask.size());
    for (size_t i = 0; i < m_SampleValidMask.size(); ++i) {
        if (m_SampleValidMask[i] != 0) { ids.push_back(static_cast<igIndex>(i)); }
    }
    return ids;
}

/* ------------------------------------------------------------------ */
/* 折线输出                                                            */
/* ------------------------------------------------------------------ */
/** 复制一份属性条目（数组共享，属性条目独立），避免共享 AttributeSet 时宿主对象混乱 */
namespace {
AttributeSet::Pointer CloneAttributeSetEntries(const AttributeSet::Pointer& srcSet) {
    auto dstSet = AttributeSet::New();
    if (srcSet == nullptr) { return dstSet; }

    auto all = srcSet->GetAllAttributes();
    if (all == nullptr) { return dstSet; }

    for (IGsize i = 0; i < all->GetNumberOfElements(); ++i) {
        auto& attr = all->GetElement(i);
        if (attr.isDeleted || attr.pointer == nullptr) { continue; }
        dstSet->AddAttribute(attr.type, attr.attachmentType, attr.pointer, attr.dataRange);
    }
    return dstSet;
}
} // namespace

void ResampleToLine::BuildPolyLineOutputs(const Points::Pointer& samples, AttributeSet::Pointer attrSet,
                                          int sampleNum) {
    // (1) UnstructuredMesh 折线：IG_LINE 单元，保持与既有流程（菜单/示例）兼容
    auto lineMesh = UnstructuredMesh::New();
    lineMesh->SetName("resample_to_line");
    lineMesh->SetPoints(samples);

    auto cells = CellArray::New();
    auto types = UnsignedIntArray::New();
    for (int i = 0; i + 1 < sampleNum; ++i) {
        cells->AddCellId2(i, i + 1);
        types->AddValue(IG_LINE);
    }
    lineMesh->SetCells(cells, types);
    lineMesh->SetAttributeSet(CloneAttributeSetEntries(attrSet));
    lineMesh->SetViewStyle(IG_WIREFRAME);

    m_LineMesh = lineMesh;
    SetOutput(0, lineMesh);

    // (2) SurfaceMesh 折线：真正的折线数据（点 + 边），可直接渲染为折线
    auto poly = SurfaceMesh::New();
    poly->SetName("resample_to_line");
    poly->SetPoints(samples);

    // 先给出空的面片数组：SurfaceMesh 的拓扑/可绘制数据接口要求面片数组非空，
    // 且其时间戳要早于边数组，避免后续 RequestEditStatus() 依据面片重建边而丢掉折线
    poly->SetFaces(CellArray::New());

    auto edges = CellArray::New();
    for (int i = 0; i + 1 < sampleNum; ++i) { edges->AddCellId2(i, i + 1); }
    poly->SetEdges(edges);
    poly->SetAttributeSet(attrSet);
    poly->SetViewStyle(IG_WIREFRAME);
    // 折线没有面片，关闭抽壳/简化渲染，避免空面片网格参与简化
    poly->SetShellRenderingOption(false);

    m_PolyLine = poly;
    SetOutput(1, poly);
}

IGAME_NAMESPACE_END
