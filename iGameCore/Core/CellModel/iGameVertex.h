#ifndef iGameVertex_h
#define iGameVertex_h

#include "iGameCell.h"

IGAME_NAMESPACE_BEGIN
class Vertex : public Cell {
public:
	I_OBJECT(Vertex);
	static Pointer New() { return new Vertex; }

	IGenum GetCellType() const noexcept override { return IG_VERTEX; }
    int GetCellSize() const noexcept override { return 1; }
    int GetNumberOfEdges() override { return 0; }
    int GetNumberOfFaces() override { return 0; }
	Cell* GetEdge(const int) override { return nullptr; }
    Cell* GetFace(const int) override { return nullptr; }

	/* ---------------- VTK 风格插值接口（对照 vtkVertex） ---------------- */

	int GetParametricDimension() const override { return 0; }

	/** vtkVertex::GetParametricCenter */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = 0.0;
		pcoords[1] = 0.0;
		pcoords[2] = 0.0;
	}

	/** vtkVertex::InterpolationFunctions */
	void InterpolationFunctions(const double*, double* weights) const override {
		weights[0] = 1.0;
	}

	/** vtkVertex::InterpolationDerivs */
	void InterpolationDerivs(const double*, double* derivs) const override {
		derivs[0] = 0.0;
		derivs[1] = 0.0;
		derivs[2] = 0.0;
	}

	/** vtkVertex::EvaluatePosition：仅当查询点与顶点重合（dist2 == 0）才判为命中 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[1] = pcoords[2] = 0.0;

		const Point& X = this->GetPoint(0);
		closestPoint = X;
		const double dx = static_cast<double>(X[0]) - static_cast<double>(x[0]);
		const double dy = static_cast<double>(X[1]) - static_cast<double>(x[1]);
		const double dz = static_cast<double>(X[2]) - static_cast<double>(x[2]);
		dist2 = dx * dx + dy * dy + dz * dz;
		weights[0] = 1.0;

		if (dist2 == 0.0) {
			pcoords[0] = 0.0;
			return 1;
		}
		pcoords[0] = -1.0;
		return 0;
	}

protected:
	Vertex()
	{
		this->m_Points->SetNumberOfPoints(1);
		this->m_PointIds->SetNumberOfIds(1);
		for (int i = 0; i < 1; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}
	}
	~Vertex() override = default;
};
IGAME_NAMESPACE_END
#endif