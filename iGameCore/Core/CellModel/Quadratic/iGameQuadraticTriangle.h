#ifndef iGameQuadraticTriangle_h
#define iGameQuadraticTriangle_h

#include "Base/iGameQuadraticFace.h"
#include "iGameTriangle.h"

IGAME_NAMESPACE_BEGIN
class QuadraticTriangle : public QuadraticFace {
public:
	I_OBJECT(QuadraticTriangle);
	static Pointer New() { return new QuadraticTriangle; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_TRIANGLE; }
	int GetCellSize() const noexcept override { return 6; }
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
	static int constexpr SubTriangles[4][3] = { 
  { 0, 3, 5 },
  { 3, 1, 4 },
  { 5, 4, 2 },
  { 4, 5, 3 } };

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticTriangle） ---------------- */

	int GetParametricDimension() const override { return 2; }

	/** vtkQuadraticTriangle::GetParametricCenter（vtkQuadraticTriangle.h 内联实现） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = 1.0 / 3.0;
		pcoords[2] = 0.0;
	}

	/** vtkQuadraticTriangle::InterpolationFunctions（重心坐标 t = L0 = 1-r-s, r = L1, s = L2） */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		const double r = pcoords[0];
		const double s = pcoords[1];
		const double t = 1.0 - r - s;

		weights[0] = t * (2.0 * t - 1.0);
		weights[1] = r * (2.0 * r - 1.0);
		weights[2] = s * (2.0 * s - 1.0);
		weights[3] = 4.0 * r * t;
		weights[4] = 4.0 * r * s;
		weights[5] = 4.0 * s * t;
	}

	/** vtkQuadraticTriangle::InterpolationDerivs（布局：∂N_i/∂r ×6, ∂N_i/∂s ×6） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double r = pcoords[0];
		const double s = pcoords[1];

		// r-derivatives
		derivs[0] = 4.0 * r + 4.0 * s - 3.0;
		derivs[1] = 4.0 * r - 1.0;
		derivs[2] = 0.0;
		derivs[3] = 4.0 - 8.0 * r - 4.0 * s;
		derivs[4] = 4.0 * s;
		derivs[5] = -4.0 * s;

		// s-derivatives
		derivs[6] = 4.0 * r + 4.0 * s - 3.0;
		derivs[7] = 0.0;
		derivs[8] = 4.0 * s - 1.0;
		derivs[9] = -4.0 * r;
		derivs[10] = 4.0 * r;
		derivs[11] = 4.0 - 8.0 * s - 4.0 * r;
	}

	/**
	 * vtkQuadraticTriangle::EvaluatePosition 的逐行复刻（4 个线性子三角形）：
	 *   - 子三角形顶点表 SubTriangles 与 vtkQuadraticTriangle.cxx 的 LinearCells[4][3]
	 *     逐项相同，节点序与 VTK_QUADRATIC_TRIANGLE 一致：0-2 为角点、3=(0,1) 中点、
	 *     4=(1,2) 中点、5=(2,0) 中点，因此无需索引置换；
	 *   - 每个子三角形交给线性 Triangle::EvaluatePosition（= vtkTriangle::EvaluatePosition），
	 *     取 dist2 最小者，再按 VTK 原文把子三角形的参数坐标仿射映射回父单元（4 个分支）；
	 *   - VTK 的 returnStatus 初值为 0 且只在 `status != -1` 时被赋值，所以恒不等于 -1，
	 *     `if (returnStatus != -1)` 是恒真的历史守卫，本函数**不会**返回 -1；这里照搬该
	 *     行为（0/1），并对「4 个子三角形全部退化」这一 VTK 未定义输出的分支补自洽值；
	 *   - VTK 末段调用 EvaluateLocation(subId, ...)，而 vtkQuadraticTriangle::EvaluateLocation
	 *     忽略 subId（vtkNotUsed）；基类 Cell::EvaluateLocation 却会把 subId 置 0，故这里按
	 *     vtkQuadraticTriangle::EvaluateLocation 的语义内联（形函数 + 加权求和）以保住 subId；
	 *   - dist2 取子三角形中的最小值；VTK 在无有效子三角形时返回未参与比较的
	 *     VTK_DOUBLE_MAX，这里改为与中性输出自洽的 |x - P0|²（pcoords=(0,0) ⇒ 权重
	 *     (1,0,...,0) ⇒ closestPoint = P0）。
	 *   - 注意（VTK 原样行为）：当最优点落在子三角形的顶点/棱分支时，vtkTriangle 的
	 *     pcoords 不做夹紧，于是末段重算出的 closestPoint 可能是外插点，而 dist2 仍是
	 *     子三角形真实最近点的平方距离，两者可以不自洽；VTK 的 vtkQuadraticTriangle
	 *     与 LinearCells 组合起来正是这个语义，这里逐行照搬。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[0] = pcoords[1] = pcoords[2] = 0.0;

		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// 四个线性子三角形依次定位（顺序与 VTK 一致，等距时的取舍也一致）
		double minDist2 = DBL_MAX; // VTK_DOUBLE_MAX
		int returnStatus = 0;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 3; ++j) {
				const int id = SubTriangles[i][j];
				m_SubTriangle->m_PointIds->SetId(j, this->m_PointIds->GetId(id));
				m_SubTriangle->m_Points->SetPoint(j, this->m_Points->GetPoint(id));
			}
			int ignoreId = 0;
			double pc[3] = {0.0, 0.0, 0.0};
			double dist2Sub = 0.0;
			double tempWeights[3] = {0.0, 0.0, 0.0};
			Point closest;
			const int status =
			    m_SubTriangle->EvaluatePosition(x, closest, ignoreId, pc, dist2Sub, tempWeights);
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
				pcoords[0] /= 2.0;
				pcoords[1] = 0.5 + (pcoords[1] / 2.0);
			} else {
				pcoords[0] = 0.5 - pcoords[0] / 2.0;
				pcoords[1] = 0.5 - pcoords[1] / 2.0;
			}
			pcoords[2] = 0.0;

			// 等价 vtkQuadraticTriangle::EvaluateLocation（忽略 subId）+ 其后的 dist2 赋值
			this->InterpolationFunctions(pcoords, weights);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 6; ++i) {
				const Point& P = this->GetPoint(i);
				cp[0] += weights[i] * static_cast<double>(P[0]);
				cp[1] += weights[i] * static_cast<double>(P[1]);
				cp[2] += weights[i] * static_cast<double>(P[2]);
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
	QuadraticTriangle()
	{
		this->m_Points->SetNumberOfPoints(6);
		this->m_PointIds->SetNumberOfIds(6);
		for (int i = 0; i < 6; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		// 供 EvaluatePosition 复用的线性子三角形（对照 vtkQuadraticTriangle::Face）
		m_SubTriangle = Triangle::New();
	}
	~QuadraticTriangle() override = default;

	Triangle::Pointer m_SubTriangle;
};
IGAME_NAMESPACE_END
#endif