#ifndef iGameLagrangeTetrahedron_h
#define iGameLagrangeTetrahedron_h

#include "iGameLagrangeVolume.h"
#include "iGameLagrangeTriangle.h"
#include "iGameLagrangeQuadrilateral.h"
#include "iGameLagrangeLine.h"

IGAME_NAMESPACE_BEGIN

class LagrangeTetrahedron : public LagrangeVolume {
public:
    I_OBJECT(LagrangeTetrahedron);
    static Pointer New() { return new LagrangeTetrahedron; }

    IGenum GetCellType() const noexcept override { return IG_LAGRANGE_TETRAHEDRON; }
    int GetNumberOfEdges() override { return 6; }
    int GetNumberOfFaces() override { return 4; }

    int GetNumberOfPoints() override {
        if (m_Order <= 0) return 0;
        return (m_Order + 1) * (m_Order + 2) * (m_Order + 3) / 6;
    }

    Cell* GetEdge(const int edgeId) override {
        const igIndex* ptIds = nullptr;
        int numEdgePoints = GetEdgePointIds(edgeId, ptIds);
        if (numEdgePoints == 0) return nullptr;

        m_Edge->SetOrder(numEdgePoints - 1);
        m_Edge->Reset();
        m_Edge->m_PointIds->SetNumberOfIds(numEdgePoints);
        m_Edge->m_Points->SetNumberOfPoints(numEdgePoints);

        for (int i = 0; i < numEdgePoints; ++i) {
            m_Edge->m_PointIds->SetId(i, this->m_PointIds->GetId(ptIds[i]));
            m_Edge->m_Points->SetPoint(i, this->m_Points->GetPoint(ptIds[i]));
        }
        return m_Edge.get();
    }

    Cell* GetFace(const int faceId) override {
        const igIndex* ptIds = nullptr;
        int numFacePoints = GetFacePointIds(faceId, ptIds);
        if (numFacePoints == 0) return nullptr;

        // 四面体的面都是三角形
        LagrangeTriangle* face = m_TriFace.get();

        face->SetOrder(m_Order);
        face->Reset();
        face->m_PointIds->SetNumberOfIds(numFacePoints);
        face->m_Points->SetNumberOfPoints(numFacePoints);

        for (int i = 0; i < numFacePoints; ++i) {
            face->m_PointIds->SetId(i, this->m_PointIds->GetId(ptIds[i]));
            face->m_Points->SetPoint(i, this->m_Points->GetPoint(ptIds[i]));
        }
        return face;
    }
    /* ---------------- VTK 风格插值接口（对照 vtkLagrangeTetra / vtkHigherOrderTetra） ------------ */
    int GetParametricDimension() const noexcept override { return 3; }

    // 等价 vtkHigherOrderTetra::GetParametricCenter：重心坐标各 1/4
    void GetParametricCenter(double pcoords[3]) const override {
        pcoords[0] = 0.25;
        pcoords[1] = 0.25;
        pcoords[2] = 0.25;
    }

    int GetCellSize() const noexcept override {
        const int n = static_cast<int>(this->m_Points->GetNumberOfPoints());
        if (n > 0) { return n; }
        return (m_Order > 0) ? ((m_Order + 1) * (m_Order + 2) * (m_Order + 3) / 6) : 0;
    }

    int EffectiveOrder() const {
        const int order = TetraOrderFromPointCount(this->GetCellSize());
        return (order > 0) ? order : m_Order;
    }

    // 说明：VTK 的 InterpolateFunctions 对 n==2 且 15 点有专门分支，且其 10 点展开分支的
    // 棱中点顺序与本仓库 GetEdgePointIds 的表**不一致**（VTK 自身的问题）；这里统一走
    // 「Eta/Deta 直积 + 重心索引」的通用路径，与本仓库的边表（标准 vtkTetra 棱序）严格一致。
    void InterpolationFunctions(const double pcoords[3], double* weights) const override {
        const int order = this->EffectiveOrder();
        const int n = this->GetCellSize();
        if (order < 1 || weights == nullptr || n <= 0) { return; }
        const double tau[4] = {pcoords[0], pcoords[1], pcoords[2],
                               1.0 - pcoords[0] - pcoords[1] - pcoords[2]};
        for (int idx = 0; idx < n; ++idx) {
            int lambda[4] = {0, 0, 0, 0};
            TetraBarycentricIndex(idx, lambda, order);
            weights[idx] = SimplexEta(order, lambda[0], tau[0]) *
                           SimplexEta(order, lambda[1], tau[1]) *
                           SimplexEta(order, lambda[2], tau[2]) *
                           SimplexEta(order, lambda[3], tau[3]);
        }
    }

    void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
        const int order = this->EffectiveOrder();
        const int n = this->GetCellSize();
        if (order < 1 || derivs == nullptr || n <= 0) { return; }
        const double tau[4] = {pcoords[0], pcoords[1], pcoords[2],
                               1.0 - pcoords[0] - pcoords[1] - pcoords[2]};
        for (int idx = 0; idx < n; ++idx) {
            int lambda[4] = {0, 0, 0, 0};
            TetraBarycentricIndex(idx, lambda, order);
            const double e[4] = {SimplexEta(order, lambda[0], tau[0]),
                                 SimplexEta(order, lambda[1], tau[1]),
                                 SimplexEta(order, lambda[2], tau[2]),
                                 SimplexEta(order, lambda[3], tau[3])};
            const double d[4] = {SimplexDeta(order, lambda[0], tau[0]),
                                 SimplexDeta(order, lambda[1], tau[1]),
                                 SimplexDeta(order, lambda[2], tau[2]),
                                 SimplexDeta(order, lambda[3], tau[3])};
            const double common = e[0] * e[1] * e[2] * d[3];
            derivs[idx] = d[0] * e[1] * e[2] * e[3] - common;             // ∂N/∂r
            derivs[n + idx] = e[0] * d[1] * e[2] * e[3] - common;         // ∂N/∂s
            derivs[2 * n + idx] = e[0] * e[1] * d[2] * e[3] - common;     // ∂N/∂t
        }
    }

    int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
                         double& dist2, double* weights) const override {
        return this->EvaluatePositionNewton(x, closestPoint, subId, pcoords, dist2, weights);
    }

    int GetEdgePointIds(const int edgeId, const igIndex*& ptIds) override {
        if (edgeId < 0 || edgeId >= 6) return 0;
        switch (m_Order) {
            case 1: { // 线性 (2节点) 边
                static const igIndex edges[6][2] = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {1, 3}, {2, 3}};
                ptIds = edges[edgeId];
                return 2;
            }
            case 2: { // 二次 (3节点) 边
                // 节点顺序: [角点1, 角点2, 边中点]
                static const igIndex edges[6][3] = {{0, 1, 4}, {1, 2, 5}, {2, 0, 6}, {0, 3, 7}, {1, 3, 8}, {2, 3, 9}};
                ptIds = edges[edgeId];
                return 3;
            }
            case 3: { // 三次 (4节点) 边
                // 节点顺序: [角点1, 角点2, 边内部点1, 边内部点2]
                static const igIndex edges[6][4] = {{0, 1, 4, 5},   {1, 2, 6, 7},   {2, 0, 8, 9},
                                                    {0, 3, 10, 11}, {1, 3, 12, 13}, {2, 3, 14, 15}};
                ptIds = edges[edgeId];
                return 4;
            }
            case 4: { // 四次 (5节点) 边
                // 节点顺序: [角点1, 角点2, 边内部点1, 边内部点2, 边内部点3]
                static const igIndex edges[6][5] = {{0, 1, 4, 5, 6},    {1, 2, 7, 8, 9},    {2, 0, 10, 11, 12},
                                                    {0, 3, 13, 14, 15}, {1, 3, 16, 17, 18}, {2, 3, 19, 20, 21}};
                ptIds = edges[edgeId];
                return 5;
            }
            default:
                return 0;
        }
    }

    int GetFacePointIds(const int faceId, const igIndex*& ptIds) override {
        if (faceId < 0 || faceId >= 4) return 0;
        switch (m_Order) {
            case 1: { // 线性面 (3节点)
                static const igIndex linearFaces[4][3] = {{0, 1, 2}, {0, 3, 1}, {1, 3, 2}, {2, 3, 0}};
                ptIds = linearFaces[faceId];
                return 3;
            }
            case 2: { // 二次面 (6节点)
                static const igIndex quadraticFaces[4][6] = {{0, 1, 3, 4, 8, 7},
                                                             {1, 2, 3, 5, 9, 8},
                                                             {2, 0, 3, 6, 7, 9},
                                                             {0, 2, 1, 6, 5, 4}};
                ptIds = quadraticFaces[faceId];
                return 6;
            }
            case 3: {                                     
                static const igIndex cubicFaces[4][10] = {
                                                          
                                                          {0, 1, 3, 4, 5, 12, 13, 11, 10, 16},

                                                          {1, 2, 3, 6, 7, 14, 15, 13, 12, 17},

                                                          {2, 0, 3, 8, 9, 10, 11, 15, 14, 18},

                                                          {0, 2, 1, 9, 8, 7, 6, 5, 4, 19}};
                ptIds = cubicFaces[faceId];
                return 10;
            }
            case 4: { // Quartic (15 nodes)
                static const igIndex quarticFaces[4][15] = {

                        {0, 1, 3, 4, 5, 6, 16, 17, 18, 15, 14, 13, 22, 23, 24},


                        {1, 2, 3, 7, 8, 9, 19, 20, 21, 18, 17, 16, 27, 25, 26},

                        {2, 0, 3, 10, 11, 12, 13, 14, 15, 21, 20, 19, 30, 28, 29},

        
                        {0, 2, 1, 12, 11, 10, 9, 8, 7, 6, 5, 4, 31, 32, 33}};
                ptIds = quarticFaces[faceId];
                return 15;
            }
            default:
                return 0;
        }
    }

protected:
    LagrangeTetrahedron()
    {
        m_Edge = LagrangeLine::New();
        m_TriFace = LagrangeTriangle::New();
    }
    ~LagrangeTetrahedron() override = default;

private:
    LagrangeLine::Pointer m_Edge;
    LagrangeTriangle::Pointer m_TriFace;
};

IGAME_NAMESPACE_END
#endif