
#ifndef iGameLagrangeLine_h
#define iGameLagrangeLine_h

#include "iGameCell.h"

IGAME_NAMESPACE_BEGIN

class LagrangeLine : public Cell {
public:
    I_OBJECT(LagrangeLine);
    static Pointer New() { return new LagrangeLine; }

    // 设置/获取单元的阶数
    void SetOrder(int order) { m_Order = order; }
    int GetOrder() const { return m_Order; }

    // 统一使用 IG_QUADRATIC_EDGE 代表所有高阶边
    IGenum GetCellType() const noexcept override { return IG_QUADRATIC_EDGE; }

    // 顶点数由阶数动态决定
    int GetNumberOfPoints() override { return m_Order + 1; }

    /* ---------------- VTK 风格插值接口（对照 vtkLagrangeCurve / vtkHigherOrderCurve） ---------------- */
    int GetParametricDimension() const noexcept override { return 1; }

    // 等价 vtkHigherOrderCurve::GetParametricCenter：pcoords = 0.5
    void GetParametricCenter(double pcoords[3]) const override {
        pcoords[0] = 0.5;
        pcoords[1] = 0.0;
        pcoords[2] = 0.0;
    }

    // 权重个数 = 实际点数（阶数 = 点数 - 1）。**尚未填充点时必须返回 0**，
    // 这样内核/调用方才知道"点数由网格给定"，会把网格给的全部点填进来。
    int GetCellSize() const noexcept override {
        const int n = static_cast<int>(this->m_Points->GetNumberOfPoints());
        if (n > 0) { return n; }
        return (m_Order > 0) ? (m_Order + 1) : 0;
    }

    int EffectiveOrder() const {
        const int n = this->GetCellSize();
        return (n > 1) ? (n - 1) : m_Order;
    }

    void InterpolationFunctions(const double pcoords[3], double* weights) const override {
        const int order = this->EffectiveOrder();
        if (order < 1 || weights == nullptr) { return; }
        double derivs[64];
        LagrangeTensor1(order, pcoords[0], weights, derivs);
    }

    void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
        const int order = this->EffectiveOrder();
        if (order < 1 || derivs == nullptr) { return; }
        double shape[64];
        LagrangeTensor1(order, pcoords[0], shape, derivs);
    }

    int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
                         double& dist2, double* weights) const override {
        // 通用形函数 Newton 反解（等价 vtkCell 的高阶 Newton 路径；VTK 曲线用线性子段近似，
        // 这里用与本类形函数严格一致的 Newton，结果在容差内一致）
        return this->EvaluatePositionNewton(x, closestPoint, subId, pcoords, dist2, weights);
    }

    // 1D单元没有边和面
    int GetNumberOfEdges() override { return 0; }
    int GetNumberOfFaces() override { return 0; }
    Cell* GetEdge(const int) override { return nullptr; }
    Cell* GetFace(const int) override { return nullptr; }

protected:
    LagrangeLine() : m_Order(0) {}
    ~LagrangeLine() override = default;

    int m_Order;
};

IGAME_NAMESPACE_END
#endif