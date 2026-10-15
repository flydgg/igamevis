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
// 采样内核（本类同时也是所有重采样滤波共用的唯一内核）:
//   空间索引   CSR 均匀网格（按包围盒长宽比与单元数自动定分辨率，扁平候选表 + 每体素区间）
//   定位       单元类型自己的 VTK 复刻 EvaluatePosition（复刻 vtkCell 与各 vtkXxx）；
//              上一个命中单元作为提示（等价 vtkProbeFilter 的 FindCell(x, lastCellId)）；
//              环形候选 + 去重戳 + 单元 AABB 预判 + 源包围盒预判
//   命中判定   等价 vtkDataSet::FindCell(x, tol2)：参数域内**且** dist2 ≤ tol2
//              （三维单元域内 dist2 = 0，于是退化为「是否在单元内」；0/1/2 维单元还要看距离）
//   容差       默认 = 源包围盒对角线 × 1e-3（等价 vtkProbeFilter 的 tol2 = 对角线² × 1e-6）
//   吸附       可选 SnapToCellWithClosestPoint + SnappingRadius（VTK 语义）
//   退化       尚未实现插值的类型（部分高阶 / Lagrange / QuadraticPolygon）退化为
//              「前 N 个角点」的线性单元，并在 SampleLocation::degraded 标记
//   ghost      跳过 vtkGhostType 中 HIDDENCELL | DUPLICATECELL 的单元
//   属性       源点数据插值（保留数组类型）+ 源单元数据按命中单元拷到输出点数据
//              （VTK InputCellArrays，同名点数据优先）+ 掩膜；无效点各属性填 0
//   并行       样本数 ≥ 1024 且源网格不含「只能用共享单元对象」的类型时按线程分块定位
//              （结果与串行 bitwise 一致；lastCell 只是提示，命中判定与顺序无关）
//
//   对外批量 API（重采样滤波直接调用，避免各自重写定位/插值）:
//     BuildSourceMesh / LocatePositions / InterpolateAttributes / EvaluateCell /
//     DefaultTolerance / CreateArrayObject / SourceTypeName
//
// Execute() 的行为（保持交互层不变的约定）:
//   1. 定位查询点（上述内核）；
//   2. 命中后对模型全部点属性做线性插值，写回查询点集（原地，GetOutput() == 输入 1）；
//   3. 点属性 ValidPointMask（**IntArray**，供 igQtProbeWidget 使用）：命中 = 1，未命中 = 0，
//      未命中时插值属性填 0；
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
        bool snapped{false};                    // 是否靠“吸附到最近单元”才命中（VTK SnapToCellWithClosestPoint）
        bool degraded{false};                   // 是否走“角点线性退化”（该单元类型尚未实现 VTK 插值）
    };

    /**
     * 形函数权重池：每个采样点占 stride 个槽（stride = 源网格的最大单元点数）。
     * 原先权重内嵌在 SampleLocation 里（double[256] = 2 KB/点，100 万采样点即 2 GB）；
     * 拆成扁平池后单点结果约 64 B，且权重只写 numWeights 个 double（六面体仅 64 B）。
     * stride 由 MakeSampleWeights 按源网格算出；调用方复用它给定位与插值。
     */
    struct SampleWeights {
        std::vector<double> values;  // 大小 = stride * 采样点数
        int stride{0};               // 每个采样点的槽数

        const double* At(size_t i) const {
            return values.data() + static_cast<size_t>(i) * static_cast<size_t>(stride);
        }
        double* At(size_t i) {
            return values.data() + static_cast<size_t>(i) * static_cast<size_t>(stride);
        }
    };

    /** 源网格的最大单元点数（等长单元 O(1)，混合单元 O(单元数)）。用于给权重池定 stride。 */
    static int MaxCellPointCount(const UnstructuredMesh::Pointer& mesh);

    /** 按源网格分配权重池（stride = MaxCellPointCount，内容清零）。 */
    static SampleWeights MakeSampleWeights(const UnstructuredMesh::Pointer& mesh, size_t sampleCount);

    /** 采样选项（与 vtkProbeFilter 的属性逐项对应） */
    struct ProbeOptions {
        double tolerance{-1.0};          // 距离容差；<0 = VTK 默认（包围盒对角线 × 1e-3）
        bool interpolatePointData{true}; // 源点数据插值到采样点
        bool copyCellData{true};         // 源单元数据按命中单元拷到输出点数据（VTK InputCellArrays）
        bool skipBlankedCells{true};     // 跳过 vtkGhostType 中 HIDDENCELL/DUPLICATECELL 的单元
        // 只插值这些数组（按名字匹配；空 = 全部，保持 VTK vtkProbeFilter 的默认语义）。
        // 真实 CFD 结果常带几十条数组，全插一遍往往是最大的单项开销；
        // 只要显示某几条时把名字传进来即可（点/单元数组共用这一份名单）。
        std::vector<std::string> interpolateArrays;
        bool snapToCellWithClosestPoint{false}; // 未命中时吸附到最近单元的最近点（VTK 默认关）
        double snappingRadius{-1.0};     // 吸附半径（绝对长度）；<0 = 不限（VTK 默认正无穷）
        std::string maskArrayName{"validpointmask"}; // 掩膜数组名；空串 = 不输出掩膜
        // 采样点较多时按线程分块定位（等价 vtkProbeFilter 的 vtkSMPTools::For）。
        // 结果与串行完全一致（lastCell 只是提示，命中判定与顺序无关）；源网格含需要面拓扑的
        // Polyhedron 时自动退回串行。默认开启，样本数 < 1024 时仍走串行。
        bool useParallel{true};
    };

    /**
     * 按「单元类型 + 点表」求值（等价 vtkCell::EvaluatePosition）：定位 + 权重由单元类型自己的
     * VTK 复刻实现给出，未实现插值的类型退化为角点线性。供拿不到 UnstructuredMesh 的「单元定向」
     * 重采样（如 ResampleToImage 的网格适配输入）复用同一套单元求值，避免各自重写形函数。
     * numWeights 返回权重个数（退化时 < cellPoints.size()）；Polyhedron 等需要面拓扑的类型返回 -1。
     * 返回 1 = 在单元内 / 0 = 不在 / -1 = 不可用或退化单元。
     */
    static int EvaluateCell(IGenum cellType, const std::vector<Point>& cellPoints, const Point& x,
                            double* weights, int& numWeights, double pcoords[3],
                            Point& closestPoint, double& dist2);

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
                                std::vector<unsigned char>* validMask = nullptr,
                                SampleWeights* weights = nullptr);

    /**
     * 范围式批量定位：out/validMask 由调用方按 positions.size() 预分配（并已完成初始化），
     * 内核只写 [begin, end) 这一段，不产生任何逐元素拷贝。
     * 供「分块上报进度」「多次复用同一结果缓冲」的调用方使用（LocatePositions 即其特例）。
     */
    static bool LocatePositionsRange(const UnstructuredMesh::Pointer& mesh,
                                     const std::vector<Point>& positions,
                                     size_t begin,
                                     size_t end,
                                     const ProbeOptions& options,
                                     std::vector<SampleLocation>& out,
                                     std::vector<unsigned char>* validMask = nullptr,
                                     SampleWeights* weights = nullptr);

    /**
     * 批量属性（把数组追加到 outSet，VTK 语义）：
     *   - 源点数据插值（保留源数组类型；无效采样点填 0）；
     *   - 源单元数据按命中单元整段拷到**输出点数据**（同名时点数据优先；无效点填 0）；
     *   - 掩膜数组（UnsignedCharArray，1 = 有效 / 0 = 无效）。
     */
    static void InterpolateAttributes(const UnstructuredMesh::Pointer& mesh,
                                      AttributeSet::Pointer outSet,
                                      const std::vector<SampleLocation>& locations,
                                      const ProbeOptions& options,
                                      const SampleWeights* weights = nullptr);

protected:
    ProbeFilter();
    ~ProbeFilter() override = default;

private:
    double m_Tolerance{-1.0};  // < 0 表示自动计算
};
IGAME_NAMESPACE_END
