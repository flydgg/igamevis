#ifndef iGameQuad_h
#define iGameQuad_h

#include "iGameFace.h"
#include "iGameTriangle.h"

IGAME_NAMESPACE_BEGIN
class Quad : public Face {
public:
	I_OBJECT(Quad);
	static Pointer New() { return new Quad; }

	IGenum GetCellType() const noexcept override { return IG_QUAD; }
    int GetCellSize() const noexcept override { return 4; }
    int GetNumberOfEdges() override { return 4; }

	Vector3f GetNormal() override
	{
		const Vector3f& v0 = this->m_Points->GetPoint(0);
		const Vector3f& v1 = this->m_Points->GetPoint(1);
		const Vector3f& v2 = this->m_Points->GetPoint(2);
		const Vector3f& v3 = this->m_Points->GetPoint(3);
		return this->Self::ComputrAverageNormal(v0, v1, v2, v3);
	}

	static Vector3f ComputrAverageNormal(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& v3)
	{
		Vector3f center = (v0 + v1 + v2 + v3) / 4;

		Vector3f normal;
		normal.setZero();

		normal += Triangle::ComputrNormal(center, v0, v1);
		normal += Triangle::ComputrNormal(center, v1, v2);
		normal += Triangle::ComputrNormal(center, v2, v3);
		normal += Triangle::ComputrNormal(center, v3, v0);

		normal.normalize();
		return normal;
	}
	static constexpr int edges[4][2] = {
  { 0, 1 },
  { 1, 2 },
  { 3, 2 },
  { 0, 3 },
	};

	/* ---------------- VTK 风格插值接口（对照 vtkQuad） ---------------- */

	int GetParametricDimension() const override { return 2; }

	/** vtkQuad::GetParametricCenter */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = 0.5;
		pcoords[1] = 0.5;
		pcoords[2] = 0.0;
	}

	/** vtkQuad::InterpolationFunctions（双线性形函数） */
	void InterpolationFunctions(const double pcoords[3], double* sf) const override {
		const double rm = 1.0 - pcoords[0];
		const double sm = 1.0 - pcoords[1];

		sf[0] = rm * sm;
		sf[1] = pcoords[0] * sm;
		sf[2] = pcoords[0] * pcoords[1];
		sf[3] = rm * pcoords[1];
	}

	/** vtkQuad::InterpolationDerivs（布局：∂N_i/∂r ×4, ∂N_i/∂s ×4） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double rm = 1.0 - pcoords[0];
		const double sm = 1.0 - pcoords[1];

		derivs[0] = -sm;
		derivs[1] = sm;
		derivs[2] = pcoords[1];
		derivs[3] = -pcoords[1];
		derivs[4] = -rm;
		derivs[5] = -pcoords[0];
		derivs[6] = pcoords[0];
		derivs[7] = rm;
	}

	/**
	 * 双线性曲面 S(u,v) = pt0 + u·B + v·C + uv·D 上离 x 的驻点（Newton 解驻值方程）：
	 *   f0 = r·Su = 0, f1 = r·Sv = 0, r = S(u,v) - x
	 *   雅可比 [[|Su|², Su·Sv + r·D], [Su·Sv + r·D, |Sv|²]]
	 * 等价于 vtkQuad 里 BilinearPatchClosestParameters 求的参数（内部驻点）。
	 * 奇异（退化四边形）时返回 false。
	 */
	static bool BilinearClosestParameters(const double x[3], const double pt0[3],
	                                      const double B[3], const double C[3], const double D[3],
	                                      double& u, double& v) {
		const auto dot3 = [](const double a[3], const double b[3]) {
			return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
		};
		u = 0.5;
		v = 0.5;
		for (int iter = 0; iter < 20; ++iter) {
			const double Su[3] = {B[0] + v * D[0], B[1] + v * D[1], B[2] + v * D[2]};
			const double Sv[3] = {C[0] + u * D[0], C[1] + u * D[1], C[2] + u * D[2]};
			const double r[3] = {pt0[0] + u * B[0] + v * C[0] + u * v * D[0] - x[0],
			                     pt0[1] + u * B[1] + v * C[1] + u * v * D[1] - x[1],
			                     pt0[2] + u * B[2] + v * C[2] + u * v * D[2] - x[2]};
			const double f0 = dot3(r, Su);
			const double f1 = dot3(r, Sv);
			const double rD = dot3(r, D);
			const double a00 = dot3(Su, Su);
			const double a01 = dot3(Su, Sv) + rD;
			const double a11 = dot3(Sv, Sv);

			const double det = a00 * a11 - a01 * a01;
			if (std::fabs(det) <= 1e-30) { return false; }
			const double du = -(a11 * f0 - a01 * f1) / det;
			const double dv = -(-a01 * f0 + a00 * f1) / det;
			u += du;
			v += dv;
			if (du * du + dv * dv <= 1e-24) { break; }
		}
		return true;
	}

	/**
	 * vtkQuad::EvaluatePosition 的语义复刻：
	 *   - inside 只取决于“垂直落点/内部驻点是否落在 [0,1]²”（与离平面距离无关）；
	 *   - closestPoint/pcoords 取「内部驻点（若 inside）与 4 条边精确投影」中的最小值；
	 *   - weights 用 pcoords 处的双线性形函数（允许外插）。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[2] = 0.0;

		const Point& P0 = this->GetPoint(0);
		const Point& P1 = this->GetPoint(1);
		const Point& P2 = this->GetPoint(2);
		const Point& P3 = this->GetPoint(3);
		const double pt0[3] = {static_cast<double>(P0[0]), static_cast<double>(P0[1]),
		                       static_cast<double>(P0[2])};
		const double pt1[3] = {static_cast<double>(P1[0]), static_cast<double>(P1[1]),
		                       static_cast<double>(P1[2])};
		const double pt2[3] = {static_cast<double>(P2[0]), static_cast<double>(P2[1]),
		                       static_cast<double>(P2[2])};
		const double pt3[3] = {static_cast<double>(P3[0]), static_cast<double>(P3[1]),
		                       static_cast<double>(P3[2])};
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		double B[3], C[3], D[3];
		for (int i = 0; i < 3; ++i) {
			B[i] = pt1[i] - pt0[i];
			C[i] = pt3[i] - pt0[i];
			D[i] = pt2[i] - pt1[i] - pt3[i] + pt0[i];
		}

		double bestU = 0.5;
		double bestV = 0.5;
		const bool converged = BilinearClosestParameters(xd, pt0, B, C, D, bestU, bestV);
		const bool inside = converged && bestU >= 0.0 && bestU <= 1.0 && bestV >= 0.0 &&
		                    bestV <= 1.0;

		double bestDist2 = DBL_MAX;
		if (inside) {
			const double p[3] = {pt0[0] + bestU * B[0] + bestV * C[0] + bestU * bestV * D[0],
			                     pt0[1] + bestU * B[1] + bestV * C[1] + bestU * bestV * D[1],
			                     pt0[2] + bestU * B[2] + bestV * C[2] + bestU * bestV * D[2]};
			const double dx = p[0] - xd[0];
			const double dy = p[1] - xd[1];
			const double dz = p[2] - xd[2];
			bestDist2 = dx * dx + dy * dy + dz * dz;
		}

		// 4 条边的精确投影（顺序与 VTK 一致：v=0, u=1, v=1, u=0），取全局最小
		double t = 0.0;
		Point cp;
		double d2 = DistancePointToLine(x, P0, P1, t, cp);
		if (d2 < bestDist2) { bestDist2 = d2; bestU = t; bestV = 0.0; }
		d2 = DistancePointToLine(x, P1, P2, t, cp);
		if (d2 < bestDist2) { bestDist2 = d2; bestU = 1.0; bestV = t; }
		d2 = DistancePointToLine(x, P3, P2, t, cp);
		if (d2 < bestDist2) { bestDist2 = d2; bestU = t; bestV = 1.0; }
		d2 = DistancePointToLine(x, P0, P3, t, cp);
		if (d2 < bestDist2) { bestDist2 = d2; bestU = 0.0; bestV = t; }

		pcoords[0] = bestU;
		pcoords[1] = bestV;
		this->InterpolationFunctions(pcoords, weights);

		closestPoint = Point(static_cast<float>(pt0[0] + bestU * B[0] + bestV * C[0] +
		                                        bestU * bestV * D[0]),
		                     static_cast<float>(pt0[1] + bestU * B[1] + bestV * C[1] +
		                                        bestU * bestV * D[1]),
		                     static_cast<float>(pt0[2] + bestU * B[2] + bestV * C[2] +
		                                        bestU * bestV * D[2]));
		dist2 = bestDist2;

		return inside ? 1 : 0;
	}
protected:
	Quad()
	{
		this->m_Points->SetNumberOfPoints(4);
		this->m_PointIds->SetNumberOfIds(4);
		for (int i = 0; i < 4; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}
	}
	~Quad() override = default;
};
IGAME_NAMESPACE_END
#endif