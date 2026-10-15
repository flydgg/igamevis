#include "iGameResampleWithDataSet.h"

#include "iGameAttributeSet.h"
#include "iGameCellArray.h"
#include "Probe/iGameProbeFilter.h"
#include "iGameCompositeDataObject.h"
#include "iGameLagrangeUnstructuredMesh.h"
#include "iGamePointSet.h"
#include "iGameStructuredMesh.h"
#include "iGameSurfaceMesh.h"
#include "iGameUnstructuredMesh.h"
#include "iGameVolumeMesh.h"
#include "iGameFlatArray.h"
#include "iGamePoints.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>

IGAME_NAMESPACE_BEGIN
namespace {
/** RAII：退出 Execute 时把进度推到 1.0（成功/失败/提前返回都一样），避免进度条卡在中间。
 *  顺带清空文案——Qt 进度条控件在文本为空时恢复默认并隐藏。 */
struct ResampleProgressGuard {
    explicit ResampleProgressGuard(Filter* filter) : m_Filter(filter) {}
    ~ResampleProgressGuard() {
        if (m_Filter != nullptr) { m_Filter->UpdateProgress(1.0); }
        ProgressObserver::Instance()->UpdateText(std::string());
    }
    Filter* m_Filter{nullptr};
};
} // namespace


namespace {

// 复制单元数组，避免 CellArray::DeepCopy 在混合单元尺寸下产生错误偏移
CellArray::Pointer DeepCopyCellArray(CellArray* source) {
    auto output = CellArray::New();
    if (!source) {
        return output;
    }

    for (IGsize i = 0; i < source->GetNumberOfCells(); ++i) {
        const igIndex* ids = nullptr;
        const int count = source->GetCellIds(i, ids);
        output->AddCellIds(ids, count);
    }

    return output;
}

// 复制 PointSet 共有的点坐标和属性
void DeepCopyPointSetBase(PointSet* output, PointSet* input, AttributeSet::Pointer attributes) {
    auto points = Points::New();
    if (input->GetPoints()) {
        points->DeepCopy(input->GetPoints());
    }
    output->SetPoints(points);

    if (attributes != nullptr) {
        // 调用方给定属性集：几何取自 input、属性来自别处
        output->SetAttributeSet(attributes);
    } else {
        auto copiedAttributes = AttributeSet::New();
        if (input->GetAttributeSet()) {
            copiedAttributes->DeepCopy(input->GetAttributeSet());
        }
        output->SetAttributeSet(copiedAttributes);
    }
    output->SetName(input->GetName());
}

// 按数据类型创建并深拷贝一个独立输出对象
DataObject::Pointer DeepCopyDataObject(DataObject::Pointer input, AttributeSet::Pointer attributes = nullptr) {
    if (input == nullptr) {
        return nullptr;
    }

    const IGenum type = input->GetDataObjectType();
    DataObject::Pointer output = DataObject::CreateDataObject(type);
    if (!output && type == IG_LAGRANGE_UNSTRUCTURED_MESH) {
        output = LagrangeUnstructuredMesh::New();
    }
    if (!output) {
        return nullptr;
    }

    auto inputPointSet = DynamicCast<PointSet>(input);
    auto outputPointSet = DynamicCast<PointSet>(output);
    if (!inputPointSet || !outputPointSet) {
        return nullptr;
    }
    DeepCopyPointSetBase(outputPointSet.get(), inputPointSet.get(), attributes);

    switch (type) {
        case IG_UNSTRUCTURED_MESH: {
            auto inputMesh = DynamicCast<UnstructuredMesh>(input);
            auto outputMesh = DynamicCast<UnstructuredMesh>(output);
            if (!inputMesh || !outputMesh) {
                return nullptr;
            }

            auto cellTypes = UnsignedIntArray::New();
            if (inputMesh->GetCellTypes()) {
                cellTypes->DeepCopy(inputMesh->GetCellTypes());
            }
            outputMesh->SetCells(DeepCopyCellArray(inputMesh->GetCells()), cellTypes);
            break;
        }
        case IG_SURFACE_MESH: {
            auto inputMesh = DynamicCast<SurfaceMesh>(input);
            auto outputMesh = DynamicCast<SurfaceMesh>(output);
            if (!inputMesh || !outputMesh) {
                return nullptr;
            }

            if (inputMesh->GetFaces()) {
                outputMesh->SetFaces(DeepCopyCellArray(inputMesh->GetFaces()));
            }
            if (inputMesh->GetEdges()) {
                outputMesh->SetEdges(DeepCopyCellArray(inputMesh->GetEdges()));
            }
            break;
        }
        case IG_VOLUME_MESH: {
            auto inputMesh = DynamicCast<VolumeMesh>(input);
            auto outputMesh = DynamicCast<VolumeMesh>(output);
            if (!inputMesh || !outputMesh) {
                return nullptr;
            }

            if (inputMesh->GetIsPolyhedronType()) {
                // 多面体网格需要同时重建面片和体-面索引
                auto faces = DeepCopyCellArray(inputMesh->GetFaces());
                auto volumeFaces = CellArray::New();
                for (IGsize i = 0; i < inputMesh->GetNumberOfVolumes(); ++i) {
                    std::vector<igIndex> faceIds(IGAME_CELL_MAX_SIZE);
                    const int faceCount = inputMesh->GetVolumeFaceIds(i, faceIds.data());
                    volumeFaces->AddCellIds(faceIds.data(), faceCount);
                }
                outputMesh->InitVolumesWithPolyhedron(faces, volumeFaces);
                break;
            }

            if (inputMesh->GetVolumes()) {
                outputMesh->SetVolumes(DeepCopyCellArray(inputMesh->GetVolumes()));
            }
            if (inputMesh->GetFaces()) {
                outputMesh->SetFaces(DeepCopyCellArray(inputMesh->GetFaces()));
            }
            if (inputMesh->GetEdges()) {
                outputMesh->SetEdges(DeepCopyCellArray(inputMesh->GetEdges()));
            }
            outputMesh->SetIsPolyhedronType(false);
            break;
        }
        case IG_STRUCTURED_MESH: {
            auto inputMesh = DynamicCast<StructuredMesh>(input);
            auto outputMesh = DynamicCast<StructuredMesh>(output);
            if (!inputMesh || !outputMesh) {
                return nullptr;
            }

            igIndex* dimensions = inputMesh->GetDimensionSize();
            if (dimensions) {
                igIndex copiedDimensions[3]{dimensions[0], dimensions[1], dimensions[2]};
                outputMesh->SetDimensionSize(copiedDimensions);
            }
            if (inputMesh->GetFaces()) {
                outputMesh->SetFaces(DeepCopyCellArray(inputMesh->GetFaces()));
            }
            if (inputMesh->GetEdges()) {
                outputMesh->SetEdges(DeepCopyCellArray(inputMesh->GetEdges()));
            }
            break;
        }
        case IG_LAGRANGE_UNSTRUCTURED_MESH: {
            auto inputMesh = DynamicCast<LagrangeUnstructuredMesh>(input);
            auto outputMesh = DynamicCast<LagrangeUnstructuredMesh>(output);
            if (!inputMesh || !outputMesh) {
                return nullptr;
            }

            for (IGsize i = 0; i < inputMesh->GetNumberOfCells(); ++i) {
                const igIndex* ids = nullptr;
                const int count = inputMesh->GetCellPointIds(i, ids);
                std::vector<igIndex> cellIds;
                if (count > 0 && ids) {
                    cellIds.assign(ids, ids + count);
                }
                outputMesh->AddCell(cellIds.data(), count,
                                    inputMesh->GetSpecificCellType(i),
                                    inputMesh->GetCellOrder(i));
            }
            break;
        }
        case IG_POINT_SET:
        default:
            break;
    }

    return output;
}

/** 3x3 行列式（等价于 c1 . (c2 x c3)） */
inline double Det3(const double c1[3], const double c2[3], const double c3[3]) {
    return c1[0] * c2[1] * c3[2] + c2[0] * c3[1] * c1[2] + c3[0] * c1[1] * c2[2] -
           c1[0] * c3[1] * c2[2] - c2[0] * c1[1] * c3[2] - c3[0] * c2[1] * c1[2];
}

/**
 * 单元插值类型：
 *   - 线性面单元 → 线性插值（重心坐标 / 双线性 / 扇形三角化）
 *   - 体单元     → 均值坐标（Mean Value Coordinates）
 *   - 二次单元   → 二次形函数（Newton 参数坐标反解）
 */
enum ShapeKind {
    SHAPE_NONE = 0,
    // 线性面单元
    SHAPE_LINEAR_TRIANGLE,
    SHAPE_LINEAR_QUAD,
    SHAPE_LINEAR_POLYGON,
    // 二次面单元
    SHAPE_QUADRATIC_TRIANGLE, // 6 节点
    SHAPE_QUADRATIC_QUAD,     // 8 节点
    // 体单元
    SHAPE_TETRA,              // 4 节点：解析重心坐标（等价于均值坐标，精确）
    SHAPE_MVC_VOLUME,         // 六面体/棱柱/金字塔/多面体：均值坐标
    // 二次体单元
    SHAPE_QUADRATIC_TETRA,    // 10 节点
    SHAPE_QUADRATIC_HEX,      // 20 节点
    SHAPE_QUADRATIC_PRISM,    // 15 节点（三棱柱 / 楔形）
    SHAPE_QUADRATIC_PYRAMID,  // 13 节点
    // 高次单元（节点序与形函数按 VTK 硬编码；VTK 中 pcoords 为 [0,1]，此处直接使用其 [-1,1] 内部形式）
    SHAPE_BIQUADRATIC_QUAD,       // 9 节点：0-3 角点、4-7 棱中点(0-1)(1-2)(2-3)(3-0)、8 面心
    SHAPE_QUADRATIC_LINEAR_QUAD,  // 6 节点：0-3 角点、4=棱(0,1)中点、5=棱(2,3)中点
    SHAPE_QUADRATIC_LINEAR_WEDGE, // 12 节点：0-5 角点、6-8 底面棱中点、9-11 顶面棱中点
    SHAPE_TRIQUADRATIC_HEX,       // 27 节点：0-7 角点、8-19 棱中点、20-25 面心、26 体心
};

/** 是否走「二次 / 高次形函数」插值路径 */
inline bool IsQuadraticKind(ShapeKind kind) {
    switch (kind) {
        case SHAPE_QUADRATIC_TRIANGLE:
        case SHAPE_QUADRATIC_QUAD:
        case SHAPE_QUADRATIC_TETRA:
        case SHAPE_QUADRATIC_HEX:
        case SHAPE_QUADRATIC_PRISM:
        case SHAPE_QUADRATIC_PYRAMID:
        case SHAPE_BIQUADRATIC_QUAD:
        case SHAPE_QUADRATIC_LINEAR_QUAD:
        case SHAPE_QUADRATIC_LINEAR_WEDGE:
        case SHAPE_TRIQUADRATIC_HEX:
            return true;
        default:
            return false;
    }
}

/** 二次「面」单元（走平面投影 + 2D Newton） */
inline bool IsQuadraticFaceKind(ShapeKind kind) {
    return kind == SHAPE_QUADRATIC_TRIANGLE || kind == SHAPE_QUADRATIC_QUAD || kind == SHAPE_BIQUADRATIC_QUAD ||
           kind == SHAPE_QUADRATIC_LINEAR_QUAD;
}

/* 源归一化（结构化网格 / 点云 / Lagrange / 容器合并）已迁到
   ProbeFilter::BuildSourceMesh，各重采样滤波共用同一份实现。 */
} // namespace

/* ------------------------------------------------------------------ */
/* 主流程                                                              */
/* ------------------------------------------------------------------ */
bool ResampleWithDataSet::Execute() {
    // 输入 0：被采样网格（数据来源，VTK 的 Source）
    // 输入 1：采样点网格（探针几何，输出是它的深拷贝；VTK 的 Input）
    auto source = GetInput(0);
    auto probe = GetInput(1);

    m_UnsupportedQuadraticCellCount = 0;
    m_SnappedPointCount = 0;
    m_Output = nullptr;
    SetOutput(0, nullptr);
    m_SampleCellIds.clear();
    m_SampleValidMask.clear();

    // 进度显示：Qt 的进度条控件订阅全局 ProgressObserver（跨线程会 marshal 回 GUI 线程），
    // 因此这里按阶段 + 分块上报即可，无需改动采样内核。
    ResampleProgressGuard progressGuard(this);
    auto reportProgress = [this](double value, const std::string& text) {
        m_ProgressObserver->UpdateText(text);
        this->UpdateProgress(value);
    };
    reportProgress(0.0, "重采样：准备被采样网格…");

    if (source == nullptr) {
        m_Message = "未选择「被采样网格」";
        return false;
    }
    if (probe == nullptr) {
        m_Message = "未选择「采样点网格」";
        return false;
    }

    auto probePoints = probe->GetPoints();
    if (probePoints == nullptr || probePoints->GetNumberOfPoints() == 0) {
        m_Message = std::string("「采样点网格」没有点，无法确定采样位置（数据类型：") +
                    ProbeFilter::SourceTypeName(probe->GetDataObjectType()) + "）";
        return false;
    }

    // 被采样网格：归一化为一份 UnstructuredMesh。
    // 对齐 ParaView 的 Resample With Dataset（任意 vtkDataSet + 复合数据集逐块展开）：
    //   UnstructuredMesh 直接用；SurfaceMesh / VolumeMesh 转换；StructuredMesh 按 (ni,nj,nk) 生成单元；
    //   PointSet 每点生成 IG_VERTEX；LagrangeUnstructuredMesh 保留原单元类型；
    //   容器（DrawObject / CompositeDataObject / 带子数据对象）递归展开后合并为一份网格。
    int sourceBlockCount = 0;
    auto mesh = ProbeFilter::BuildSourceMesh(source.GetPointer(), &sourceBlockCount);
    reportProgress(0.05, "重采样：定位采样点…");
    if (mesh == nullptr) {
        m_Message = std::string("「被采样网格」不支持的数据类型：") + ProbeFilter::SourceTypeName(source->GetDataObjectType()) +
                    "（支持 PointSet / SurfaceMesh / VolumeMesh / UnstructuredMesh / StructuredMesh / "
                    "LagrangeUnstructuredMesh，以及包含它们的容器网格）";
        return false;
    }
    if (mesh->GetNumberOfPoints() == 0 || mesh->GetNumberOfCells() == 0) {
        m_Message = "「被采样网格」没有单元或点";
        return false;
    }

    const BoundingBox& bbox = mesh->GetBoundingBox();
    const double diag = std::max(bbox.diag(), 1e-12);

    // 容差与吸附半径：对外是「相对包围盒对角线的比例」，内核用绝对长度。
    // 自动容差对齐 VTK 默认：源包围盒对角线 × 1e-3（vtkProbeFilter 的 tol2 = 对角线² × 1e-6）。
    m_EffectiveTolerance = m_AutoTolerance ? ProbeFilter::DefaultTolerance(mesh) : (m_Tolerance * diag);
    const bool snappingEnabled = (m_SnappingRadius > 0.0);
    m_EffectiveSnappingRadius = snappingEnabled ? (m_SnappingRadius * diag) : -1.0;

    // 采样位置就是「采样点网格」的所有点
    const int sampleNum = static_cast<int>(probePoints->GetNumberOfPoints());
    std::vector<Point> positions(static_cast<size_t>(sampleNum));
    for (int i = 0; i < sampleNum; ++i) {
        positions[static_cast<size_t>(i)] = probePoints->GetPoint(i);
    }

    // ================= 定位 + 权重：全部交给 ProbeFilter 采样内核 =================
    // （CSR 空间索引 + 单元类型自己的 EvaluatePosition + VTK 的 FindCell 容差语义，
    //   与 ProbeFilter / 其它重采样滤波共用同一份实现）
    ProbeFilter::ProbeOptions options;
    options.tolerance = m_EffectiveTolerance;                 // 绝对长度
    options.maskArrayName = m_ValidPointMaskArrayName;        // 掩膜名可配置（默认 validpointmask）
    options.snapToCellWithClosestPoint = snappingEnabled;     // VTK SnapToCellWithClosestPoint
    options.snappingRadius = m_EffectiveSnappingRadius;       // <0 = 不限
    options.interpolateArrays = m_InterpolateArrays;          // 空 = 全部插值（VTK 默认）

    std::vector<ProbeFilter::SampleLocation> kernelLocations(static_cast<size_t>(sampleNum));
    std::vector<unsigned char> validMask(static_cast<size_t>(sampleNum), 0);
    // 形函数权重池：stride = 源网格最大单元点数（六面体=8）。原来权重内嵌在
    // SampleLocation 里（2 KB/点，百万采样点即 2 GB），现在按 numWeights 个 double 写入。
    ProbeFilter::SampleWeights kernelWeights = ProbeFilter::MakeSampleWeights(mesh, static_cast<size_t>(sampleNum));

    // 分块调用内核对进度友好：每块内部仍然走并行（ParallelChunks），块之间上报一次进度。
    // 块数按采样点数自适应，样本少时只有 1 块，行为与单次调用完全一致。
    const int progressBlocks = std::min(32, std::max(1, sampleNum / 4096));
    for (int block = 0; block < progressBlocks; ++block) {
        const size_t begin = static_cast<size_t>(static_cast<long long>(sampleNum) * block / progressBlocks);
        const size_t end = static_cast<size_t>(static_cast<long long>(sampleNum) * (block + 1) / progressBlocks);
        // 范围式内核：结果直接写进预分配的总缓冲，不做逐元素回填（SampleLocation 约 2 KB/点）
        if (!ProbeFilter::LocatePositionsRange(mesh, positions, begin, end, options, kernelLocations, &validMask,
                                               &kernelWeights)) {
            m_Message = "「被采样网格」定位失败";
            return false;
        }
        reportProgress(0.05 + 0.60 * (block + 1) / progressBlocks,
                       "重采样：定位采样点 " + std::to_string(block + 1) + "/" +
                               std::to_string(progressBlocks));
    }

    m_SampleCellIds.assign(static_cast<size_t>(sampleNum), -1);
    m_SampleValidMask.assign(static_cast<size_t>(sampleNum), 0);
    int validCount = 0;
    for (int i = 0; i < sampleNum; ++i) {
        const size_t si = static_cast<size_t>(i);
        m_SampleCellIds[si] = kernelLocations[si].cellId;
        m_SampleValidMask[si] = validMask[si];
        if (validMask[si] != 0) { ++validCount; }
        if (kernelLocations[si].snapped) { ++m_SnappedPointCount; }
        // 角点线性退化：该单元类型尚未实现 VTK 插值（如部分高阶 / Lagrange）
        if (kernelLocations[si].degraded) { ++m_UnsupportedQuadraticCellCount; }
    }

    // 属性：① 被采样网格的点数据插值 ② 其单元数据按命中单元拷到输出点 ③ validpointmask
    // 与 vtkProbeFilter 一致：源单元数组进入**输出点数据**（VTK 内部的 InputCellArrays），
    // 同名时点数据优先；无效采样点各属性填 0。
    // 注意：对齐 vtkProbeFilter 的默认值（PassPointArrays = 0、PassCellArrays = 0），
    // 「采样点网格」自身的属性数组不复制到输出里（这两个开关针对的是探针输入，不是被采样网格）。
    AttributeSet::Pointer outAttr = AttributeSet::New();
    reportProgress(0.70, "重采样：插值属性…");
    ProbeFilter::InterpolateAttributes(mesh, outAttr, kernelLocations, options, &kernelWeights);
    reportProgress(0.95, "重采样：组装输出…");

    // 输出 = 采样点网格的同类型深拷贝（几何/拓扑独立），属性换成采样结果
    auto output = DeepCopyDataObject(probe, outAttr);
    if (output == nullptr) {
        m_Message = "输出构建失败：不支持「采样点网格」的数据类型";
        return false;
    }
    output->SetName(probe->GetName() + "_resample");
    m_Output = output;
    SetOutput(0, output);

    m_Message = "ResampleWithDataSet: samples=" + std::to_string(sampleNum) + ", valid=" + std::to_string(validCount) +
                ", invalid=" + std::to_string(sampleNum - validCount) +
                ", tolerance=" + std::to_string(m_EffectiveTolerance) +
                (m_AutoTolerance ? " (auto = bboxDiag*1e-3, VTK default)" : " (manual)");
    if (sourceBlockCount > 1) {
        m_Message += ", source blocks=" + std::to_string(sourceBlockCount);
    }
    if (m_SnappedPointCount > 0) {
        m_Message += ", snapped=" + std::to_string(m_SnappedPointCount) +
                     " (radius=" + std::to_string(m_EffectiveSnappingRadius) + ")";
    }
    if (m_UnsupportedQuadraticCellCount > 0) {
        m_Message += ", degraded cells=" + std::to_string(m_UnsupportedQuadraticCellCount);
    }
    return true;
}

/* ------------------------------------------------------------------ */
/* 属性插值                                                            */
/* ------------------------------------------------------------------ */
/** 有效采样点 id 列表（等价 VTK vtkProbeFilter::GetValidPoints()） */
std::vector<igIndex> ResampleWithDataSet::GetValidPoints() const {
    std::vector<igIndex> ids;
    ids.reserve(m_SampleValidMask.size());
    for (size_t i = 0; i < m_SampleValidMask.size(); ++i) {
        if (m_SampleValidMask[i] != 0) { ids.push_back(static_cast<igIndex>(i)); }
    }
    return ids;
}

IGAME_NAMESPACE_END
