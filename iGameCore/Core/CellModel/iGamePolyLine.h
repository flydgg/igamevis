#ifndef iGamePolyLine_h
#define iGamePolyLine_h

#include "iGameCell.h"

IGAME_NAMESPACE_BEGIN
class PolyLine : public Cell {
public:
    I_OBJECT(PolyLine);
    static Pointer New() { return new PolyLine; }

	IGenum GetCellType() const noexcept override { return IG_POLY_LINE; }
    int GetCellSize() const noexcept override { return 2; }
    int GetNumberOfEdges() override { return 0; }
    int GetNumberOfFaces() override { return 0; }
    Cell* GetEdge(const int) override { return nullptr; }
    Cell* GetFace(const int) override { return nullptr; }

    /* ---------------- VTK 风格插值接口（对照 vtkPolyLine） ---------------- */
    // 折线单元：按段遍历，取最近的一段（段内用 vtkLine 的 EvaluatePosition 语义），
    // 权重只落在该段的两个端点上并补零（等价 vtkPolyLine::EvaluatePosition）。
    int GetParametricDimension() const noexcept override { return 1; }

    // vtkPolyLine::GetParametricCenter 返回的是「中间段的编号」，这里只取参数坐标
    void GetParametricCenter(double pcoords[3]) const override {
        pcoords[0] = 0.5;
        pcoords[1] = 0.0;
        pcoords[2] = 0.0;
    }

    int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
                         double& minDist2, double* weights) const override {
        pcoords[1] = pcoords[2] = 0.0;

        double closestWeights[2] = {0.0, 0.0};
        minDist2 = DBL_MAX;
        subId = -1;
        int returnStatus = 0;

        const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
        if (numPts <= 0 || weights == nullptr) { return 0; }

        // 每线程一份 Line，避免并行时共享可变状态（等价 VTK 的每线程 CurrentCell）
        thread_local Line::Pointer s_line = Line::New();
        s_line->Reset();
        for (int i = 0; i < numPts - 1; ++i) {
            const Point& a = this->m_Points->GetPoint(i);
            const Point& b = this->m_Points->GetPoint(i + 1);
            s_line->m_Points->AddPoint(a[0], a[1], a[2]);
            s_line->m_Points->AddPoint(b[0], b[1], b[2]);
            s_line->m_PointIds->AddId(i);
            s_line->m_PointIds->AddId(i + 1);

            double pc[3] = {0.0, 0.0, 0.0};
            double dist2 = 0.0;
            double lineWeights[2] = {0.0, 1.0};
            Point closest;
            int ignoreId = 0;
            const int status = s_line->EvaluatePosition(x, closest, ignoreId, pc, dist2, lineWeights);
            if (status != -1 && ((dist2 < minDist2) || ((dist2 == minDist2) && (returnStatus == 0)))) {
                returnStatus = status;
                closestPoint = closest;
                minDist2 = dist2;
                subId = i;
                pcoords[0] = pc[0];
                closestWeights[0] = lineWeights[0];
                closestWeights[1] = lineWeights[1];
            }
            s_line->Reset();
        }

        for (int i = 0; i < numPts; ++i) { weights[i] = 0.0; }
        if (subId >= 0) {
            weights[subId] = closestWeights[0];
            weights[subId + 1] = closestWeights[1];
        }
        return returnStatus;
    }

protected:
    PolyLine() {}
    ~PolyLine() override = default;
};
IGAME_NAMESPACE_END
#endif