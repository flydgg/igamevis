// ============================================================================
// ProbeFilter — 在指定位置探测数据（点定位 + 插值）
//
// 数据流（v3 设计）:
//   输入 0：模型网格（面网格 / 体网格），提供单元与点属性；
//   输入 1：查询点点集（PointSet），用于点定位 + 插值；
//   输出 0：与输入 1 为同一个对象 —— 在查询点集上原地添加插值属性与
//           ValidPointMask，不新建输出点集，重复执行不会产生新对象。
//
// 用法:
//   auto f = ProbeFilter::New();
//   f->SetInput(model);        // 模型网格
//   f->SetInput(1, queryPts);  // 查询点点集（Execute 后原地带上插值属性）
//   f->SetTolerance(0.001);   // 可选：**绝对**距离容差；不设时用 VTK 默认
//                             //（源包围盒对角线 × 1e-3，内部平方比较）
//   f->Execute();
//   auto result = f->GetOutput();  // 与 queryPts 为同一对象
//
// 算法（v1）:
//   1. 对每个查询点遍历模型单元（面网格只查面单元、体网格只查体单元），
//      调用 EvaluatePosition 判定；v1 仅支持三角形 / 四面体，其余类型未命中；
//   2. 命中后按重心坐标对模型全部点属性做线性插值，写入查询点；
//   3. 新增点属性 ValidPointMask：找到单元 = 1，未找到 = 0；
//      未找到单元时插值属性填 0；
//   4. 模型无属性 / 无单元不报错，流程照常执行。
//
// 球体随机采样（交互层使用）:
//   GenerateSpherePoints(points, center, radius, n) 在以 center 为球心、
//   radius 为半径的球体内均匀随机采样 n 个点，原地写入 points。
//   radius == 0 时按退化球处理：n 个点全部生成在 center 位置。
// ============================================================================
#pragma once
#include <iGameDataObject.h>
#include <iGameFilter.h>
#include <iGameFlatArray.h>
#include <iGamePointSet.h>
#include <iGamePoints.h>
#include <iGameUnstructuredMesh.h>

IGAME_NAMESPACE_BEGIN
class ProbeFilter : public Filter {
public:
    I_OBJECT(ProbeFilter);
    static Pointer New() { return new ProbeFilter; }

    bool Execute() override;

    // ---- 容差（相对值，默认自动）----
    void SetTolerance(double value) { m_Tolerance = value; }
    double GetTolerance() const { return m_Tolerance; }
    void SetAutoTolerance() { m_Tolerance = -1.0; }
    bool HasAutoTolerance() const { return m_Tolerance < 0.0; }

    // ---- 球体随机采样（原地修改 points，供 UI / 测试复用）----
    // radius == 0 时所有点落在 center 位置；radius < 0 时不生成任何点。
    // seed == 0 时使用随机种子；否则使用固定种子（便于复现）。
    static void GenerateSpherePoints(PointSet::Pointer points, const Point& center,
                                     float radius, int count, unsigned seed = 0);

    /* ==================================================================
     * 采样内核（ProbeFilter 作为唯一探针内核，供各重采样滤波复用）
     *
     * 语义与 vtkProbeFilter / vtkDataSet::FindCell 对齐：
     *   - 定位用单元类型自己的 EvaluatePosition（Cell 的 VTK 风格接口），
     *     而不是在滤波里重写几何判定；
     *   - 判定规则：EvaluatePosition 判为“在单元内”，或最近点距离平方
     *     dist2 ≤ tol2（VTK 的 FindCell 容差语义）；
     *   - 默认容差 = 源包围盒对角线 × 1e-3（VTK: 对角线² × 1e-6，平方比较）；
     *   - 源单元数据按命中单元整段拷到**输出点数据**（VTK 的 InputCellArrays），
     *     同名时点数据优先；无效采样点各属性填 0。
     * ================================================================== */

    /** 单个采样点的定位结果（等价 vtkProbeFilter 每个点的 FindCell 结果） */
    struct SampleLocation {
        igIndex cellId{-1};                     // 命中单元；-1 = 未命中
        int subId{0};                           // 命中的子单元 id（等价 vtkCell::EvaluatePosition 的 subId）
        Point closestPoint{0.f, 0.f, 0.f};      // 单元上离采样点最近的点
        double pcoords[3]{0.0, 0.0, 0.0};       // 参数坐标（可超出 [0,1]）
        double dist2{0.0};                      // 到最近点的平方距离
        int numWeights{0};                      // 形函数权重个数（= 单元点数）
        double weights[IGAME_CELL_MAX_SIZE]{};  // pcoords 处的形函数值
        bool snapped{false};                    // 是否靠“吸附到最近单元”才命中（VTK SnapToCellWithClosestPoint）
        bool degraded{false};                   // 是否走“角点线性退化”（该单元类型尚未实现 VTK 插值）
    };

    /** 采样选项（与 vtkProbeFilter 的属性逐项对应） */
    struct ProbeOptions {
        double tolerance{-1.0};          // 距离容差；<0 = VTK 默认（包围盒对角线 × 1e-3）
        bool interpolatePointData{true}; // 源点数据插值到采样点
        bool copyCellData{true};         // 源单元数据按命中单元拷到输出点数据（VTK InputCellArrays）
        bool skipBlankedCells{true};     // 跳过 vtkGhostType 中 HIDDENCELL/DUPLICATECELL 的单元
        bool snapToCellWithClosestPoint{false}; // 未命中时吸附到最近单元的最近点（VTK 默认关）
        double snappingRadius{-1.0};     // 吸附半径（绝对长度）；<0 = 不限（VTK 默认正无穷）
        std::string maskArrayName{"validpointmask"}; // 掩膜数组名；空串 = 不输出掩膜
    };

    /** 数据对象类型名（用于失败 / 结果消息） */
    static const char* SourceTypeName(IGenum type);

    /**
     * 源网格归一化：任意受支持网格 / 容器 → 一份 UnstructuredMesh（等价 VTK 接受任意
     * vtkDataSet 并逐块展开复合数据集）：PointSet（每点一个顶点单元）/ SurfaceMesh /
     * VolumeMesh / UnstructuredMesh / StructuredMesh / LagrangeUnstructuredMesh 直接转换，
     * 容器（DrawObject / CompositeDataObject / 带子数据对象）递归展开并按序合并。
     * 单块输入不拷贝；*blockCount 返回合并前的块数。
     */
    static UnstructuredMesh::Pointer BuildSourceMesh(DataObject* object, int* blockCount = nullptr);

    /** 按数组类型创建同类型数组（保留数组数据类型，供各重采样滤波复用） */
    static ArrayObject::Pointer CreateArrayObject(IGenum arrayType);

    /** VTK 默认容差（绝对距离）：源包围盒对角线 × 1e-3 */
    static double DefaultTolerance(const UnstructuredMesh::Pointer& mesh);

    /**
     * 批量定位：只建一次空间索引，把 positions 逐个定位到 mesh 上。
     * options.tolerance < 0 时用 DefaultTolerance(mesh)；options.snapToCellWithClosestPoint
     * 打开时，未命中的采样点会吸附到 snappingRadius（<0 = 不限）内最近单元的最近点取值。
     * validMask（可空）返回 1/0。返回 false 仅表示输入非法（mesh 为空 / 无单元）。
     */
    static bool LocatePositions(const UnstructuredMesh::Pointer& mesh,
                                const std::vector<Point>& positions,
                                const ProbeOptions& options,
                                std::vector<SampleLocation>& out,
                                std::vector<unsigned char>* validMask = nullptr);

    /**
     * 批量属性（把数组追加到 outSet，VTK 语义）：
     *   - 源点数据插值（保留源数组类型；无效采样点填 0）；
     *   - 源单元数据按命中单元整段拷到**输出点数据**（同名时点数据优先；无效点填 0）；
     *   - 掩膜数组（UnsignedCharArray，1 = 有效 / 0 = 无效）。
     */
    static void InterpolateAttributes(const UnstructuredMesh::Pointer& mesh,
                                      AttributeSet::Pointer outSet,
                                      const std::vector<SampleLocation>& locations,
                                      const ProbeOptions& options);

protected:
    ProbeFilter();
    ~ProbeFilter() override = default;

private:
    double m_Tolerance{-1.0};  // < 0 表示自动计算
};
IGAME_NAMESPACE_END
