#ifndef iGameLagrangeTriangle_h
#define iGameLagrangeTriangle_h

#include "iGameLagrangeFace.h"

IGAME_NAMESPACE_BEGIN

class LagrangeTriangle : public LagrangeFace {
public:
    I_OBJECT(LagrangeTriangle);
    static Pointer New() { return new LagrangeTriangle; }

    IGenum GetCellType() const noexcept override { return IG_LAGRANGE_TRIANGLE; }

    // 使用通用公式，支持任意阶
    int GetNumberOfPoints() override {
        if (m_Order <= 0) return 0;
        return (m_Order + 1) * (m_Order + 2) / 2;
    }
    int GetNumberOfEdges() override { return 3; }

    /* ---------------- VTK 风格插值接口（对照 vtkLagrangeTriangle / vtkHigherOrderTriangle） ------- */
    int GetParametricDimension() const noexcept override { return 2; }

    // 等价 vtkHigherOrderTriangle::GetParametricCenter：pcoords = (1/3, 1/3)
    void GetParametricCenter(double pcoords[3]) const override {
        pcoords[0] = 1.0 / 3.0;
        pcoords[1] = 1.0 / 3.0;
        pcoords[2] = 0.0;
    }

    int GetCellSize() const noexcept override {
        const int n = static_cast<int>(this->m_Points->GetNumberOfPoints());
        if (n > 0) { return n; }
        return (m_Order > 0) ? ((m_Order + 1) * (m_Order + 2) / 2) : 0;
    }

    int EffectiveOrder() const {
        const int order = TriangleOrderFromPointCount(this->GetCellSize());
        return (order > 0) ? order : m_Order;
    }

    void InterpolationFunctions(const double pcoords[3], double* weights) const override {
        const int order = this->EffectiveOrder();
        if (order < 1 || weights == nullptr) { return; }
        // 重心坐标 tau = (r, s, 1 - r - s)；等价 VTK 的 InterpolateFunctions
        const double tau[3] = {pcoords[0], pcoords[1], 1.0 - pcoords[0] - pcoords[1]};
        if (order == 1) {
            weights[0] = tau[2];
            weights[1] = tau[0];
            weights[2] = tau[1];
            return;
        }
        if (order == 2) {
            weights[0] = tau[2] * (2.0 * tau[2] - 1.0);
            weights[1] = tau[0] * (2.0 * tau[0] - 1.0);
            weights[2] = tau[1] * (2.0 * tau[1] - 1.0);
            weights[3] = 4.0 * tau[0] * tau[2];
            weights[4] = 4.0 * tau[0] * tau[1];
            weights[5] = 4.0 * tau[1] * tau[2];
            return;
        }
        const int n = this->GetCellSize();
        for (int idx = 0; idx < n; ++idx) {
            int lambda[3] = {0, 0, 0};
            SimplexBarycentricIndex(idx, lambda, order);
            weights[idx] = SimplexEta(order, lambda[0], tau[0]) *
                           SimplexEta(order, lambda[1], tau[1]) *
                           SimplexEta(order, lambda[2], tau[2]);
        }
    }

    void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
        const int order = this->EffectiveOrder();
        if (order < 1 || derivs == nullptr) { return; }
        const double tau[3] = {pcoords[0], pcoords[1], 1.0 - pcoords[0] - pcoords[1]};
        if (order == 1) {
            derivs[0] = -1.0; derivs[1] = 1.0; derivs[2] = 0.0;
            derivs[3] = -1.0; derivs[4] = 0.0; derivs[5] = 1.0;
            return;
        }
        if (order == 2) {
            derivs[0] = 1.0 - 4.0 * tau[2];
            derivs[1] = 4.0 * tau[0] - 1.0;
            derivs[2] = 0.0;
            derivs[3] = 4.0 * (tau[2] - tau[0]);
            derivs[4] = 4.0 * tau[1];
            derivs[5] = -4.0 * tau[1];
            derivs[6] = 1.0 - 4.0 * tau[2];
            derivs[7] = 0.0;
            derivs[8] = 4.0 * tau[1] - 1.0;
            derivs[9] = -4.0 * tau[0];
            derivs[10] = 4.0 * tau[0];
            derivs[11] = 4.0 * (tau[2] - tau[1]);
            return;
        }
        const int n = this->GetCellSize();
        for (int idx = 0; idx < n; ++idx) {
            int lambda[3] = {0, 0, 0};
            SimplexBarycentricIndex(idx, lambda, order);
            const double ea = SimplexEta(order, lambda[0], tau[0]);
            const double eb = SimplexEta(order, lambda[1], tau[1]);
            const double eg = SimplexEta(order, lambda[2], tau[2]);
            const double da = SimplexDeta(order, lambda[0], tau[0]);
            const double db = SimplexDeta(order, lambda[1], tau[1]);
            const double dg = SimplexDeta(order, lambda[2], tau[2]);
            derivs[idx] = da * eb * eg - ea * eb * dg;      // ∂N/∂r
            derivs[n + idx] = ea * db * eg - ea * eb * dg;  // ∂N/∂s
        }
    }

    int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
                         double& dist2, double* weights) const override {
        // 通用形函数 Newton 反解（VTK 用子三角形闭式解，二者在容差内一致）
        return this->EvaluatePositionNewton(x, closestPoint, subId, pcoords, dist2, weights);
    }

    int GetEdgePointIds(const int edgeId, const igIndex*& ptIds) override {
        if (edgeId < 0 || edgeId >= 3) return 0;
        switch (m_Order) {
            case 1: { // 线性 (2节点边)
                static const igIndex linearEdges[3][2] = {{0, 1}, {1, 2}, {2, 0}};
                ptIds = linearEdges[edgeId];
                return 2;
            }
            case 2: { // 二次 (3节点边)
                static const igIndex quadraticEdges[3][3] = {{0, 1, 3}, {1, 2, 4}, {2, 0, 5}};
                ptIds = quadraticEdges[edgeId];
                return 3;
            }
            case 3: { // 三次 (4节点边)
                static const igIndex cubicEdges[3][4] = {{0, 1, 3, 4}, {1, 2, 5, 6}, {2, 0, 7, 8}};
                ptIds = cubicEdges[edgeId];
                return 4;
            }
            case 4: { // 四次 (5节点边)
                static const igIndex quarticEdges[3][5] = {{0, 1, 3, 4, 5}, {1, 2, 6, 7, 8}, {2, 0, 9, 10, 11}};
                ptIds = quarticEdges[edgeId];
                return 5;
            }
            default:
                return 0;
        }
    }

protected:
    LagrangeTriangle() = default;
    ~LagrangeTriangle() override = default;
};

IGAME_NAMESPACE_END
#endif