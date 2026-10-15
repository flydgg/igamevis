#ifndef iGameTriangle_h
#define iGameTriangle_h

#include "iGameFace.h"

IGAME_NAMESPACE_BEGIN
class Triangle : public Face {
public:
	I_OBJECT(Triangle);
	static Pointer New() { return new Triangle; }

	IGenum GetCellType() const noexcept override { return IG_TRIANGLE; }
    int GetCellSize() const noexcept override { return 3; }
    int GetNumberOfEdges() override { return 3; }

	Vector3f GetNormal() override
	{
		const Vector3f& v0 = this->m_Points->GetPoint(0);
		const Vector3f& v1 = this->m_Points->GetPoint(1);
		const Vector3f& v2 = this->m_Points->GetPoint(2);
		return this->Self::ComputrNormal(v0, v1, v2);
	}

	static Vector3f ComputrNormal(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2)
	{
		Vector3f d10 = v1 - v0;
		Vector3f d20 = v2 - v0;

		Vector3f normal = CrossProduct(d10, d20);
		normal.normalize();
		return normal;
	}
	static constexpr int edges[3][2] = { { 0, 1 }, { 1, 2 }, { 2, 0 } };

	/* ---------------- VTK 风格插值接口（对照 vtkTriangle） ---------------- */

	int GetParametricDimension() const override { return 2; }

	/** vtkTriangle::GetParametricCenter */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = 1.0 / 3.0;
		pcoords[1] = 1.0 / 3.0;
		pcoords[2] = 0.0;
	}

	/** vtkTriangle::InterpolationFunctions */
	void InterpolationFunctions(const double pcoords[3], double* sf) const override {
		sf[0] = 1.0 - pcoords[0] - pcoords[1];
		sf[1] = pcoords[0];
		sf[2] = pcoords[1];
	}

	/** vtkTriangle::InterpolationDerivs（布局：∂N_i/∂r ×3, ∂N_i/∂s ×3） */
	void InterpolationDerivs(const double*, double* derivs) const override {
		derivs[0] = -1.0;
		derivs[1] = 1.0;
		derivs[2] = 0.0;

		derivs[3] = -1.0;
		derivs[4] = 0.0;
		derivs[5] = 1.0;
	}

	/**
	 * vtkTriangle::EvaluatePosition 的逐行复刻：
	 *   - 用 Gram 矩阵解出参数坐标 pcoords（点在三角形所在平面上的投影）；
	 *   - 按「顶点 → 边 → 内部」的优先级求最近点（VTK 原文顺序）；
	 *   - 只有投影落在三角形内部才返回 1，顶点/边分支返回 0；
	 *   - 退化三角形（面积 2 为 0）返回 -1。
	 * 注意 VTK 语义：返回 1 表示“投影落在参数域内”，与查询点到平面的距离无关。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[2] = 0.0;

		const Point& P0 = this->GetPoint(0);
		const Point& P1 = this->GetPoint(1);
		const Point& P2 = this->GetPoint(2);
		const double pt0[3] = {static_cast<double>(P0[0]), static_cast<double>(P0[1]),
		                       static_cast<double>(P0[2])};
		const double pt1[3] = {static_cast<double>(P1[0]), static_cast<double>(P1[1]),
		                       static_cast<double>(P1[2])};
		const double pt2[3] = {static_cast<double>(P2[0]), static_cast<double>(P2[1]),
		                       static_cast<double>(P2[2])};
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		double c1[3], c2[3], p[3];
		for (int i = 0; i < 3; ++i) {
			c1[i] = pt1[i] - pt0[i];
			c2[i] = pt2[i] - pt0[i];
			p[i] = xd[i] - pt0[i];
		}
		const auto dot3 = [](const double a[3], const double b[3]) {
			return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		};
		const double g11 = dot3(c1, c1);
		const double g12 = dot3(c1, c2);
		const double g22 = dot3(c2, c2);
		const double p1 = dot3(p, c1);
		const double p2 = dot3(p, c2);
		const double area2 = g11 * g22 - g12 * g12;
		const auto setClosest = [&](const double c[3]) {
			closestPoint = Point(static_cast<float>(c[0]), static_cast<float>(c[1]),
			                     static_cast<float>(c[2]));
			const double dx = c[0] - xd[0];
			const double dy = c[1] - xd[1];
			const double dz = c[2] - xd[2];
			dist2 = dx * dx + dy * dy + dz * dz;
		};

		if (area2 == 0.0) {
			// 退化三角形：VTK 返回 -1（此处额外给出中性权重，避免调用方读到未初始化值）
			pcoords[0] = pcoords[1] = 0.0;
			weights[0] = 1.0;
			weights[1] = 0.0;
			weights[2] = 0.0;
			setClosest(pt0);
			return -1;
		}

		pcoords[0] = (g22 * p1 - g12 * p2) / area2;
		pcoords[1] = (g11 * p2 - g12 * p1) / area2;

		weights[0] = 1.0 - (pcoords[0] + pcoords[1]);
		weights[1] = pcoords[0];
		weights[2] = pcoords[1];

		// 顶点 → 边 → 内部（顺序与 VTK 一致，保证最近点选取相同）
		if (p1 < 0 && p2 < 0) { // 顶点 0
			setClosest(pt0);
			return 0;
		}
		const double p1_2 = p2 - p1 - g12 + g11;
		if (p1_2 < 0 && p1 > g11) { // 顶点 1
			setClosest(pt1);
			return 0;
		}
		const double g1_22 = g22 + g11 - 2 * g12;
		if (p1_2 > g1_22 && p2 > g22) { // 顶点 2
			setClosest(pt2);
			return 0;
		}
		if (weights[0] < 0 && p1_2 >= 0 && p1_2 <= g1_22) { // 边 1-2
			const double rho2 = p1_2 / g1_22;
			const double rho1 = 1.0 - rho2;
			const double c[3] = {rho1 * pt1[0] + rho2 * pt2[0], rho1 * pt1[1] + rho2 * pt2[1],
			                     rho1 * pt1[2] + rho2 * pt2[2]};
			setClosest(c);
			return 0;
		}
		if (weights[1] < 0 && p2 >= 0 && p2 <= g22) { // 边 0-1
			const double rho2 = p2 / g22;
			const double c[3] = {pt0[0] + rho2 * c2[0], pt0[1] + rho2 * c2[1],
			                     pt0[2] + rho2 * c2[2]};
			setClosest(c);
			return 0;
		}
		if (weights[2] < 0 && p1 >= 0 && p1 <= g11) { // 边 0-2
			const double rho1 = p1 / g11;
			const double c[3] = {pt0[0] + rho1 * c1[0], pt0[1] + rho1 * c1[1],
			                     pt0[2] + rho1 * c1[2]};
			setClosest(c);
			return 0;
		}
		// 投影落在三角形内部
		const double c[3] = {pt0[0] + pcoords[0] * c1[0] + pcoords[1] * c2[0],
		                     pt0[1] + pcoords[0] * c1[1] + pcoords[1] * c2[1],
		                     pt0[2] + pcoords[0] * c1[2] + pcoords[1] * c2[2]};
		setClosest(c);
		return 1;
	}

protected:
	Triangle()
	{
		this->m_Points->SetNumberOfPoints(3);
		this->m_PointIds->SetNumberOfIds(3);
		for (int i = 0; i < 3; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}
	}
	~Triangle() override = default;
};
IGAME_NAMESPACE_END
#endif