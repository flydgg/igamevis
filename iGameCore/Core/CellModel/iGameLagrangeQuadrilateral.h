#ifndef iGameLagrangeQuadrilateral_h
#define iGameLagrangeQuadrilateral_h

#include "iGameLagrangeFace.h"

IGAME_NAMESPACE_BEGIN

class LagrangeQuadrilateral : public LagrangeFace {
public:
    I_OBJECT(LagrangeQuadrilateral);
    static Pointer New() { return new LagrangeQuadrilateral; }

    IGenum GetCellType() const noexcept override { return IG_LAGRANGE_QUADRILATERAL; }

    // 使用通用公式，支持任意阶
    int GetNumberOfPoints() override {
        if (m_Order <= 0) return 0;
        return (m_Order + 1) * (m_Order + 1);
    }
    int GetNumberOfEdges() override { return 4; }

    /* ---------------- VTK 风格插值接口（对照 vtkLagrangeQuadrilateral / vtkHigherOrderQuadrilateral） -------- */
    int GetParametricDimension() const noexcept override { return 2; }

    // 等价 vtkHigherOrderQuadrilateral::GetParametricCenter：pcoords = (0.5, 0.5)
    void GetParametricCenter(double pcoords[3]) const override {
        pcoords[0] = 0.5;
        pcoords[1] = 0.5;
        pcoords[2] = 0.0;
    }

    int GetCellSize() const noexcept override {
        const int n = static_cast<int>(this->m_Points->GetNumberOfPoints());
        if (n > 0) { return n; }
        return (m_Order > 0) ? (m_Order + 1) * (m_Order + 1) : 0;
    }

    int EffectiveOrder() const {
        const int order = QuadOrderFromPointCount(this->GetCellSize());
        return (order > 0) ? order : m_Order;
    }

    void InterpolationFunctions(const double pcoords[3], double* weights) const override {
        const int order = this->EffectiveOrder();
        if (order < 1 || weights == nullptr) { return; }
        double derivs[512];
        LagrangeTensor2(order, order, pcoords[0], pcoords[1], weights, derivs);
    }

    void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
        const int order = this->EffectiveOrder();
        if (order < 1 || derivs == nullptr) { return; }
        double shape[256];
        LagrangeTensor2(order, order, pcoords[0], pcoords[1], shape, derivs);
    }

    int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
                         double& dist2, double* weights) const override {
        return this->EvaluatePositionNewton(x, closestPoint, subId, pcoords, dist2, weights);
    }

    int GetEdgePointIds(const int edgeId, const igIndex*& ptIds) override {
        if (edgeId < 0 || edgeId >= 4) return 0;
        switch (m_Order) {
            case 1: { // 线性 (2节点边) - 与 iGameQuad.h 对齐
                static const igIndex linearEdges[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
                ptIds = linearEdges[edgeId];
                return 2;
            }
            case 2: { // 二次 (3节点边)
                static const igIndex quadraticEdges[4][3] = {{0, 1, 4}, {1, 2, 5}, {2, 3, 6}, {3, 0, 7}};
                ptIds = quadraticEdges[edgeId];
                return 3;
            }
            case 3: { // 三次 (4节点边)
                static const igIndex cubicEdges[4][4] = {{0, 1, 4, 5}, {1, 2, 6, 7}, {2, 3, 8, 9}, {3, 0, 10, 11}};
                ptIds = cubicEdges[edgeId];
                return 4;
            }
            case 4: { // 四次 (5节点边)
                static const igIndex quarticEdges[4][5] = {{0, 1, 4, 5, 6},
                                                           {1, 2, 7, 8, 9},
                                                           {2, 3, 10, 11, 12},
                                                           {3, 0, 13, 14, 15}};
                ptIds = quarticEdges[edgeId];
                return 5;
            }
            default:
                return 0;
        }
    }

protected:
    LagrangeQuadrilateral() = default;
    ~LagrangeQuadrilateral() override = default;
};

IGAME_NAMESPACE_END
#endif