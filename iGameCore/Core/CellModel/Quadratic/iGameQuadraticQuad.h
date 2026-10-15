#ifndef iGameQuadraticQuad_h
#define iGameQuadraticQuad_h

#include "Base/iGameQuadraticFace.h"
#include "iGameQuad.h"
#include "iGameTriangle.h"

IGAME_NAMESPACE_BEGIN
class QuadraticQuad : public QuadraticFace {
public:
	I_OBJECT(QuadraticQuad);
	static Pointer New() { return new QuadraticQuad; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_QUAD; }
	int GetCellSize() const noexcept override { return 8; }
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

	static int constexpr SubTriangles[6][3] = { {0,4,7},{4,1,3},{5,2,6},{6,3,7},{4,3,7},{5,6,7}};

	/**
	 * 子四边形顶点表：等价 vtkQuadraticQuad.cxx 的 LinearCells[4][4]。
	 * 索引 8 是 VTK 的「假面心点」：vtkQuadraticQuad 在 Subdivide() 里把形函数算出的
	 * 面心写进 Points 的第 9 个点（Points 比 PointIds 多一个），本仓库的 m_Points 只有
	 * 8 个点，故 EvaluatePosition 里按 Subdivide 的形函数现场计算该点的坐标。
	 */
	static int constexpr SubQuads[4][4] = {
	  { 0, 4, 8, 7 },
	  { 4, 1, 5, 8 },
	  { 8, 5, 2, 6 },
	  { 7, 8, 6, 3 },
	};

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticQuad） ---------------- */

	int GetParametricDimension() const override { return 2; }

	/** vtkQuadraticQuad::GetParametricCenter（vtkQuadraticQuad.h 内联实现） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = 0.5;
		pcoords[2] = 0.0;
	}

	/** vtkQuadraticQuad::InterpolationFunctions（8 节点 Serendipity 形函数，含棱中点） */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		const double r = pcoords[0];
		const double s = pcoords[1];

		// midedge weights
		weights[4] = 4 * r * (1.0 - r) * (1.0 - s);
		weights[5] = 4 * r * (1.0 - s) * s;
		weights[6] = 4 * r * (1.0 - r) * s;
		weights[7] = 4 * (1.0 - r) * (1.0 - s) * s;

		// corner
		weights[0] = (1.0 - r) * (1.0 - s) - 0.5 * (weights[4] + weights[7]);
		weights[1] = r * (1.0 - s) - 0.5 * (weights[4] + weights[5]);
		weights[2] = r * s - 0.5 * (weights[5] + weights[6]);
		weights[3] = (1.0 - r) * s - 0.5 * (weights[6] + weights[7]);
	}

	/** vtkQuadraticQuad::InterpolationDerivs（布局：∂N_i/∂r ×8, ∂N_i/∂s ×8） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double r = pcoords[0];
		const double s = pcoords[1];

		// Derivatives in the r-direction
		// midedge
		derivs[4] = 4 * (1.0 - s) * (1.0 - 2 * r);
		derivs[5] = 4 * (1.0 - s) * s;
		derivs[6] = 4 * s * (1.0 - 2 * r);
		derivs[7] = -4 * (1.0 - s) * s;
		derivs[0] = -(1.0 - s) - 0.5 * (derivs[4] + derivs[7]);
		derivs[1] = (1.0 - s) - 0.5 * (derivs[4] + derivs[5]);
		derivs[2] = s - 0.5 * (derivs[5] + derivs[6]);
		derivs[3] = -s - 0.5 * (derivs[6] + derivs[7]);

		// Derivatives in the s-direction
		// midedge
		derivs[12] = -4 * r * (1.0 - r);
		derivs[13] = 4 * r * (1.0 - 2 * s);
		derivs[14] = 4 * r * (1.0 - r);
		derivs[15] = 4 * (1.0 - r) * (1.0 - 2 * s);
		derivs[8] = -(1.0 - r) - 0.5 * (derivs[12] + derivs[15]);
		derivs[9] = -r - 0.5 * (derivs[12] + derivs[13]);
		derivs[10] = r - 0.5 * (derivs[13] + derivs[14]);
		derivs[11] = (1.0 - r) - 0.5 * (derivs[14] + derivs[15]);
	}

	/**
	 * vtkQuadraticQuad::EvaluatePosition 的逐行复刻（4 个线性子四边形）：
	 *   - 先用 Subdivide 的思路算「假面心点」（形函数在 (0.5,0.5) 处的加权点，即 VTK 的
	 *     Points 第 9 点），放在局部 9 点数组里；
	 *   - 按 SubQuads（= vtkQuadraticQuad.cxx 的 LinearCells[4][4]）组装 4 个线性子四边形，
	 *     交给线性 Quad::EvaluatePosition（= vtkQuad::EvaluatePosition）取 dist2 最小者，
	 *     再按 VTK 原文把子四边形的参数坐标仿射映射回父单元（4 个分支）；
	 *   - VTK 的 returnStatus 初值 0 且只在 `status != -1` 时被赋值，恒不等于 -1
	 *     （本仓库的 Quad 端口亦只返回 0/1），故 `if (returnStatus != -1)` 恒真、
	 *     本函数不会返回 -1；「4 个子四边形都无有效结果」时 VTK 的 minDist2 停在
	 *     VTK_DOUBLE_MAX，这里补与中性输出自洽的 |x - P0|²；
	 *   - VTK 末段调用 EvaluateLocation(subId, ...)，而 vtkQuadraticQuad::EvaluateLocation
	 *     忽略 subId；基类 Cell::EvaluateLocation 会把 subId 置 0，故这里内联
	 *     vtkQuadraticQuad::EvaluateLocation 的语义（形函数 + 加权求和）以保住 subId；
	 *   - 节点序与 VTK_QUADRATIC_QUAD 一致（0-3 角点、4-7 棱 (0,1)(1,2)(2,3)(3,0) 中点），
	 *     无需索引置换。
	 *   - 注意（VTK 原样行为）：最优点落在子四边形的棱分支时，vtkQuad 的 pcoords 不做夹紧，
	 *     末段重算的 closestPoint 可能是外插点，与 dist2（子四边形真实最近点的平方距离）
	 *     可以不自洽；这里与 VTK 逐行一致。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[0] = pcoords[1] = pcoords[2] = 0.0;

		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// vtkQuadraticQuad::Subdivide：把假面心点（索引 8）算出来
		double pts[9][3];
		for (int i = 0; i < 8; ++i) {
			const Point& P = this->GetPoint(i);
			pts[i][0] = static_cast<double>(P[0]);
			pts[i][1] = static_cast<double>(P[1]);
			pts[i][2] = static_cast<double>(P[2]);
		}
		{
			const double centerPc[3] = {0.5, 0.5, 0.0};
			double centerWeights[8];
			this->InterpolationFunctions(centerPc, centerWeights);
			for (int j = 0; j < 3; ++j) {
				pts[8][j] = 0.0;
				for (int i = 0; i < 8; ++i) { pts[8][j] += pts[i][j] * centerWeights[i]; }
			}
		}

		// 四个线性子四边形依次定位（顺序与 VTK 一致，等距时的取舍也一致）
		double minDist2 = DBL_MAX; // VTK_DOUBLE_MAX
		int returnStatus = 0;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				const int id = SubQuads[i][j];
				m_SingleQuad->m_Points->SetPoint(j, static_cast<float>(pts[id][0]),
				                                 static_cast<float>(pts[id][1]),
				                                 static_cast<float>(pts[id][2]));
			}
			int ignoreId = 0;
			double pc[3] = {0.0, 0.0, 0.0};
			double dist2Sub = 0.0;
			double tempWeights[4] = {0.0, 0.0, 0.0, 0.0};
			Point closest;
			const int status =
			    m_SingleQuad->EvaluatePosition(x, closest, ignoreId, pc, dist2Sub, tempWeights);
			if (status != -1 && ((dist2Sub < minDist2) ||
			                     ((dist2Sub == minDist2) && (returnStatus == 0)))) {
				returnStatus = status;
				minDist2 = dist2Sub;
				subId = i;
				pcoords[0] = pc[0];
				pcoords[1] = pc[1];
			}
		}

		// adjust parametric coordinates（VTK 原文的 returnStatus != -1 守卫，实际恒真）
		if (returnStatus != -1) {
			if (subId == 0) {
				pcoords[0] /= 2.0;
				pcoords[1] /= 2.0;
			} else if (subId == 1) {
				pcoords[0] = 0.5 + (pcoords[0] / 2.0);
				pcoords[1] /= 2.0;
			} else if (subId == 2) {
				pcoords[0] = 0.5 + (pcoords[0] / 2.0);
				pcoords[1] = 0.5 + (pcoords[1] / 2.0);
			} else {
				pcoords[0] /= 2.0;
				pcoords[1] = 0.5 + (pcoords[1] / 2.0);
			}
			pcoords[2] = 0.0;

			// 等价 vtkQuadraticQuad::EvaluateLocation（忽略 subId）+ 其后的 dist2 赋值
			this->InterpolationFunctions(pcoords, weights);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 8; ++i) {
				cp[0] += weights[i] * pts[i][0];
				cp[1] += weights[i] * pts[i][1];
				cp[2] += weights[i] * pts[i][2];
			}
			closestPoint = Point(static_cast<float>(cp[0]), static_cast<float>(cp[1]),
			                     static_cast<float>(cp[2]));
			if (minDist2 == DBL_MAX) {
				const double dx = cp[0] - xd[0];
				const double dy = cp[1] - xd[1];
				const double dz = cp[2] - xd[2];
				dist2 = dx * dx + dy * dy + dz * dz;
			} else {
				dist2 = minDist2;
			}
		}
		return returnStatus;
	}

protected:
	QuadraticQuad()
	{
		this->m_Points->SetNumberOfPoints(8);
		this->m_PointIds->SetNumberOfIds(8);
		for (int i = 0; i < 8; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		// 供 EvaluatePosition 复用的线性子四边形（对照 vtkQuadraticQuad::Quad）
		m_SingleQuad = Quad::New();
	}
	~QuadraticQuad() override = default;

	Quad::Pointer m_SingleQuad;
};
IGAME_NAMESPACE_END
#endif