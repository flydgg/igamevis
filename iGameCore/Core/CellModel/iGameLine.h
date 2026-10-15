#ifndef iGameLine_h
#define iGameLine_h

#include "iGameCell.h"

IGAME_NAMESPACE_BEGIN
class Line : public Cell {
public:
	I_OBJECT(Line);
	static Pointer New() { return new Line; }

	IGenum GetCellType() const noexcept override { return IG_LINE; }
    int GetCellSize() const noexcept override { return 2; }
    int GetNumberOfEdges() override { return 0; }
    int GetNumberOfFaces() override { return 0; }
	Cell* GetEdge(const int) override { return nullptr; }
    Cell* GetFace(const int) override { return nullptr; }

	static double ComputePointToLineDis(const Vector3d& linePoint,
                                        const Vector3d& normal,
                                        const Vector3d& point) 
	{
        return std::fabs(CrossProduct(normal, point - linePoint).length() / normal.length());
    }

	/* ---------------- VTK 风格插值接口（对照 vtkLine） ---------------- */

	int GetParametricDimension() const override { return 1; }

	/** vtkLine::GetParametricCenter */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = 0.5;
		pcoords[1] = 0.0;
		pcoords[2] = 0.0;
	}

	/** vtkLine::InterpolationFunctions */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		weights[0] = 1.0 - pcoords[0];
		weights[1] = pcoords[0];
	}

	/** vtkLine::InterpolationDerivs（布局：∂N_i/∂r） */
	void InterpolationDerivs(const double*, double* derivs) const override {
		derivs[0] = -1.0;
		derivs[1] = 1.0;
	}

	/**
	 * vtkLine::EvaluatePosition：用 vtkLine::DistanceToLine 求 t 与最近点；
	 * t 可超出 [0,1]（此时最近点夹在端点上），仅当 t ∈ [0,1] 判为在单元内。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[1] = pcoords[2] = 0.0;

		dist2 = DistancePointToLine(x, this->GetPoint(0), this->GetPoint(1), pcoords[0],
		                            closestPoint);
		weights[0] = 1.0 - pcoords[0];
		weights[1] = pcoords[0];

		return (pcoords[0] >= 0.0 && pcoords[0] <= 1.0) ? 1 : 0;
	}

protected:
	Line()
	{
		this->m_Points->SetNumberOfPoints(2);
		this->m_PointIds->SetNumberOfIds(2);
		for (int i = 0; i < 2; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}
	}
	~Line() override = default;
};
IGAME_NAMESPACE_END
#endif