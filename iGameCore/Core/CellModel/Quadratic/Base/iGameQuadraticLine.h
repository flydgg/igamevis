#ifndef iGameQuadraticLine_h
#define iGameQuadraticLine_h

#include "iGameCell.h"

IGAME_NAMESPACE_BEGIN
class QuadraticLine : public Cell {
public:
	I_OBJECT(QuadraticLine);
	static Pointer New() { return new QuadraticLine; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_EDGE; }
	int GetCellSize() const noexcept override { return 3; }
	int GetNumberOfEdges() override { return 0; }
	int GetNumberOfFaces() override { return 0; }
	Cell* GetEdge(const int) override { return nullptr; }
	Cell* GetFace(const int) override { return nullptr; }

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticEdge） ---------------- */
	int GetParametricDimension() const noexcept override { return 1; }

	// 等价 vtkQuadraticEdge / vtkCell 的参数中心：r = 0.5
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = 0.5;
		pcoords[1] = 0.0;
		pcoords[2] = 0.0;
	}

	// 节点序：0、1 = 两端点，2 = 中点（等价 vtkQuadraticEdge）
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		const double r = pcoords[0];
		weights[0] = 2.0 * (r - 0.5) * (r - 1.0);
		weights[1] = 2.0 * r * (r - 0.5);
		weights[2] = 4.0 * r * (1.0 - r);
	}

	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double r = pcoords[0];
		derivs[0] = 4.0 * r - 3.0;
		derivs[1] = 4.0 * r - 1.0;
		derivs[2] = 4.0 - r * 8.0;
	}

	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
	                     double& dist2, double* weights) const override {
		return this->EvaluatePositionNewton(x, closestPoint, subId, pcoords, dist2, weights);
	}


protected:
	QuadraticLine()
	{
		this->m_Points->SetNumberOfPoints(3);
		this->m_PointIds->SetNumberOfIds(3);
		for (int i = 0; i < 3; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}
	}
	~QuadraticLine() override = default;
};
IGAME_NAMESPACE_END
#endif