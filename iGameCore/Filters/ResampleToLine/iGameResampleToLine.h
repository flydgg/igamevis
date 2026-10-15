#pragma once
#ifndef iGameResampleToLine_h
#define iGameResampleToLine_h

#include "iGameCellCenter.h"
#include "iGameDrawObject.h"
#include "iGameFilter.h"
#include "iGamePointSet.h"
#include "iGameSceneManager.h"
#include "iGameSurfaceMesh.h"
#include "iGameUnstructuredMesh.h"
#include "iGameVector.h"
#include "iGameVolumeMesh.h"

#include <string>
#include <vector>

IGAME_NAMESPACE_BEGIN

/**
 * @brief 重采样至直线（Resample to line）
 *
 * 沿给定线段均匀生成采样点，为每个采样点定位其所在的单元并插值；定位 + 插值的语义与
 * VTK / ParaView 的 vtkProbeFilter（Plot Over Line）一致：FindCell → InterpolatePoint，
 * 命中单元时对 Point Data 线性/形函数插值、对 Cell Data 复制命中单元的值，未命中任何
 * 单元的采样点用 "validpointmask" 标记为无效并把各属性填 0。
 *
 * 输入：任意类型的网格（等价 vtkProbeFilter 接受任意 vtkDataSet），内部统一转换为
 *      UnstructuredMesh 后采样：
 *   1) UnstructuredMesh：直接使用；
 *   2) SurfaceMesh / VolumeMesh：按其面 / 体单元连接关系转换；
 *   3) StructuredMesh：按 (ni, nj, nk) 尺寸生成 0D 顶点 / 1D 线 / 2D 四边形 / 3D 六面体
 *      单元，覆盖 1D 曲线、2D 平面（含 (1, nj, nk) 这类退化排布）与 3D 结构化网格；
 *   4) LagrangeUnstructuredMesh：保留 IG_LAGRANGE_* 单元类型转换；
 *   5) PointSet（点云）：每个点生成一个 IG_VERTEX 单元（等价 vtkPolyData 的 verts）；
 *   6) 多块输入：输入对象通过子数据对象组织多个网格块（例如 CGNS 多 zone 的父对象）
 *      时逐块转换并合并为一个网格（等价 ParaView 先 Merge Blocks 再 Plot Over Line），
 *      采样点命中多个块时取序号靠前的块的数据。
 *
 * 采样内核：定位 / 权重 / 点数据插值 / 单元数据拷贝 / 有效点掩膜**全部由 ProbeFilter 采样内核完成**
 * （ProbeFilter::BuildSourceMesh + LocatePositions + InterpolateAttributes）。本类只负责沿线生成
 * 采样点、把内核结果整理成本类的查询接口、以及折线输出的构造，不再自带几何判定或插值实现。
 *
 * 单元插值由各单元类型自己的 VTK 复刻实现（Core/CellModel）：
 *   Vertex / Line / Triangle / Quad / Polygon / Tetra / Hexahedron / Prism / Pyramid / Polyhedron，
 *   以及 QuadraticTriangle / QuadraticQuad / QuadraticTetra / QuadraticHexahedron /
 *   QuadraticPrism(15, vtkQuadraticWedge) / QuadraticPyramid(13)——各自的 EvaluatePosition /
 *   InterpolationFunctions / InterpolationDerivs 均逐行对照 vtkXxx。二次单元直接由
 *   QuadraticXxx::EvaluatePosition 求值，内核不再重写形函数。
 *
 * 尚未实现插值的类型（18 节点双二次-二次楔形、24 节点双二次-二次六面体、19 节点三二次金字塔、
 * 含棱中点的 QuadraticPolygon、Lagrange 金字塔——VTK 本身亦为 Not Implemented 占位）
 * 由内核退化为「前 N 个角点」的线性单元，
 * 数量见 GetUnsupportedQuadraticCellCount() 与 GetMessage() 中的 degraded cells=N。
 *
 * 采样点不在任何单元内（超出容差）时标记为无效：输出点数据属性 validpointmask
 * (UnsignedCharArray)，1 = 可插值，0 = 无效；数组名可用 SetValidPointMaskArrayName() 修改：
 * 默认 "validpointmask"（与本工程 ResampleWithDataSet 一致），设为 "vtkValidPointMask" 即与
 * VTK 默认命名一致。无效采样点各插值属性填 0（与 vtkProbeFilter 一致），有效点 id 用
 * GetValidPoints() 取出。
 *
 * 容差默认自动，与 vtkProbeFilter 一致：源包围盒对角线 × 1e-3（等价 tol2 = 对角线² × 1e-6，
 * 比较用平方距离）；也可用 SetTolerance() 手动指定（**相对**包围盒对角线的比例）。
 *
 * 输出：
 *   output 0 : UnstructuredMesh 折线（IG_LINE 单元），与既有流程兼容;
 *   output 1 : SurfaceMesh 折线（点 + 边），真正的折线数据，可直接渲染为折线。
 */
class ResampleToLine : public Filter {
public:
    I_OBJECT(ResampleToLine);
    static Pointer New() { return new ResampleToLine; }

    bool Execute() override;

    /* ---------------- 参数设置 ---------------- */

    /** 设置采样线段：起点 p0、终点 p1、采样点数量 x */
    void setOrigTarget(const Point& p0, const Point& p1, const int& x) {
        orig = p0;
        target = p1;
        n = x;
    }
    void setOrigTarget(const Vector3d& p0, const Vector3d& p1, const int& x);
    void SetOrigTarget(const Point& p0, const Point& p1) {
        orig = p0;
        target = p1;
    }
    void SetSampleNumber(int x) { n = x; }
    int GetSampleNumber() const { return n; }

    /** 是否使用自动容差（默认开启） */
    void SetAutoTolerance(bool flag) { m_AutoTolerance = flag; }
    bool IsAutoTolerance() const { return m_AutoTolerance; }

    /**
     * 手动指定容差（相对于模型包围盒对角线长度）。
     * t <= 0 时恢复自动容差，对齐 vtkProbeFilter 的默认值（包围盒对角线 × 1e-3，
     * 即 VTK 的 tol2 = 对角线² × 1e-6）。
     */
    void SetTolerance(double t) {
        if (t > 0.0) {
            m_AutoTolerance = false;
            m_Tolerance = t;
        } else {
            m_AutoTolerance = true;
            m_Tolerance = -1.0;
        }
    }
    /** 手动容差；自动容差时返回 -1 */
    double GetTolerance() const { return m_AutoTolerance ? -1.0 : m_Tolerance; }

    /**
     * 只插值这些数组（按名字；点/单元数组共用一份名单）。
     * 空名单 = 全部插值，保持 vtkProbeFilter 的默认语义。
     */
    void SetInterpolateArrays(std::vector<std::string> names) { m_InterpolateArrays = std::move(names); }
    const std::vector<std::string>& GetInterpolateArrays() const { return m_InterpolateArrays; }
    /** 上一次 Execute() 实际使用的容差（绝对长度） */
    double GetEffectiveTolerance() const { return m_EffectiveTolerance; }

    std::string GetMessage() const { return m_Message; }

    /* ---------------- 结果查询 ---------------- */

    /** output 1: 折线（SurfaceMesh，点 + 边） */
    SurfaceMesh::Pointer GetPolyLine() const { return m_PolyLine; }
    /** output 0: 折线（UnstructuredMesh，IG_LINE 单元） */
    UnstructuredMesh::Pointer GetLineMesh() const { return m_LineMesh; }
    /** 每个采样点命中的单元 id，-1 表示无效（不在任何单元内） */
    const std::vector<igIndex>& GetSampleCellIds() const { return m_SampleCellIds; }
    /** 每个采样点是否有效（1 = 可插值，0 = 无效），与 validpointmask 属性一致 */
    const std::vector<unsigned char>& GetSampleValidMask() const { return m_SampleValidMask; }
    /** 有效采样点 id 列表（等价 VTK vtkProbeFilter::GetValidPoints()） */
    std::vector<igIndex> GetValidPoints() const;
    /** validpointmask 数组名；传空串恢复默认 "validpointmask" */
    void SetValidPointMaskArrayName(const std::string& maskName) {
        m_ValidPointMaskArrayName = maskName.empty() ? std::string("validpointmask") : maskName;
    }
    const std::string& GetValidPointMaskArrayName() const { return m_ValidPointMaskArrayName; }
    /** 本次执行中退化为线性角点处理的二次/高次单元数量 */
    IGsize GetUnsupportedQuadraticCellCount() const { return m_UnsupportedQuadraticCellCount; }
    /** 上一次 Execute() 的输入数据对象类型（IG_*，见 iGameType.h） */
    IGenum GetLastInputType() const { return m_LastInputType; }
    /** 上一次 Execute() 实际参与采样的网格块数量（多块输入 > 1，等价 Merge Blocks 后的块数） */
    int GetLastInputBlockCount() const { return m_LastInputBlockCount; }

private:
    // 定位 / 权重 / 属性插值全部由 ProbeFilter 采样内核完成（见 iGameProbeFilter.h）：
    //   ProbeFilter::BuildSourceMesh / LocatePositions / InterpolateAttributes
    // 本类只负责：沿线生成采样点、把内核结果整理成本类的查询接口、以及折线输出的构造。

    /* ---------------- 输出构建 ---------------- */
    void BuildPolyLineOutputs(const Points::Pointer& samples, AttributeSet::Pointer attrSet, int sampleNum);

    ResampleToLine() {
        SetNumberOfInputs(1);
        SetNumberOfOutputs(2);
    }
    ~ResampleToLine() override = default;

    Point orig{-1.0f, -0.983795f, -0.35714f};
    Point target{1.0f, 0.983795f, 0.35714f};
    int n{40};

    // 容差：默认自动（包围盒对角线 × kAutoToleranceRatio）
    bool m_AutoTolerance{true};
    double m_Tolerance{-1.0};
    std::vector<std::string> m_InterpolateArrays{}; // 空 = 全部插值
    double m_EffectiveTolerance{0.0};

    std::string m_Message{"Not Executed !"};
    IGsize m_UnsupportedQuadraticCellCount{0};
    IGenum m_LastInputType{IG_NONE};
    int m_LastInputBlockCount{0};

    SurfaceMesh::Pointer m_PolyLine{};
    UnstructuredMesh::Pointer m_LineMesh{};
    std::vector<igIndex> m_SampleCellIds;
    std::vector<unsigned char> m_SampleValidMask;
    std::string m_ValidPointMaskArrayName{"validpointmask"};
};

IGAME_NAMESPACE_END
#endif
