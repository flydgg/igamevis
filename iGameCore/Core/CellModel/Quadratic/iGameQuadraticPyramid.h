#ifndef iGameQuadraticPyramid_h
#define iGameQuadraticPyramid_h

#include "Base/iGameQuadraticVolume.h"
#include "iGameQuadraticTriangle.h"
#include "iGameQuadraticQuad.h"

IGAME_NAMESPACE_BEGIN
class QuadraticPyramid : public QuadraticVolume {
public:
	I_OBJECT(QuadraticPyramid);
	static Pointer New() { return new QuadraticPyramid; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_PYRAMID; }
	int GetCellSize() const noexcept override { return NumberOfPoints; }
	int GetNumberOfEdges() override { return NumberOfEdges; }
	int GetNumberOfFaces() override { return NumberOfFaces; }

	Cell* GetEdge(const int edgeId) override {
		const int* verts = edges[edgeId];
		m_Line->m_PointIds->SetId(0, this->m_PointIds->GetId(verts[0]));
		m_Line->m_PointIds->SetId(1, this->m_PointIds->GetId(verts[1]));
		m_Line->m_PointIds->SetId(2, this->m_PointIds->GetId(verts[2]));
		m_Line->m_Points->SetPoint(0, this->m_Points->GetPoint(verts[0]));
		m_Line->m_Points->SetPoint(1, this->m_Points->GetPoint(verts[1]));
		m_Line->m_Points->SetPoint(2, this->m_Points->GetPoint(verts[2]));
		return m_Line.get();
	}

	Cell* GetFace(const int faceId) override {
		const int* verts = faces[faceId];
		const int* edges = faceEdges[faceId];
		QuadraticFace* face;
		if (faces[faceId][MaxFaceSize] == 6) {
			face = m_Triangle.get();
		}
		else {
			face = m_Quad.get();
		}
		for (int i = 0; i < verts[MaxFaceSize]; ++i) {
			face->m_PointIds->SetId(i,  this->m_PointIds->GetId(verts[i]));
			face->m_Points->SetPoint(i, this->m_Points->GetPoint(verts[i]));
        }
        if (EdgeIds && EdgeIds->GetNumberOfIds() > 0) {
            for (int i = 0; i < edges[MaxFaceSize / 2]; ++i) { face->EdgeIds->SetId(i, EdgeIds->GetId(edges[i])); }
        }
		return face;
	}

	static constexpr int NumberOfPoints = 13;

	static constexpr int NumberOfEdges = 8;

	static constexpr int NumberOfFaces = 5;

	static constexpr int MaxFaceSize = 8;

	static constexpr int MaxValence = 4;

	static constexpr int edges[NumberOfEdges][3] = {
	  { 0, 1, 5 },
	  { 1, 2, 6 },
	  { 2, 3, 7 },
	  { 3, 0, 8 },
	  { 0, 4, 9 },
	  { 1, 4, 10 },
	  { 2, 4, 11 },
	  { 3, 4, 12 },
	};

	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 3, 2, 1, 8, 7, 6, 5, 8 },
	  { 0, 1, 4, 5, 10, 9, -1, -1, 6},
	  { 1, 2, 4, 6, 11, 10, -1, -1, 6},
	  { 2, 3, 4, 7, 12, 11, -1, -1, 6},
	  { 3, 0, 4, 8, 9, 12, -1, -1, 6},
	};

	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 3, 2, 1, 0, 4},
	  { 0, 5, 4, -1, 3},
	  { 1, 6, 5, -1, 3},
	  { 2, 7, 6, -1, 3},
	  { 3, 4, 7, -1, 3},
	};

	static constexpr int edgeToNeighborFaces[NumberOfEdges][2] = {
	  { 0, 1 },
	  { 0, 2 },
	  { 0, 3 },
	  { 0, 4 },
	  { 1, 4 },
	  { 1, 2 },
	  { 2, 3 },
	  { 3, 4 },
	};

	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 4, 3, 2, 1,  4 },
	  { 0, 2, 4, -1, 3 },
	  { 0, 3, 1, -1, 3 },
	  { 0, 4, 2, -1, 3 },
	  { 0, 1, 3, -1, 3 },
	};

	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
	  { 0, 4, 3, -1, 3 },
	  { 0, 1, 5, -1, 3 },
	  { 1, 2, 6, -1, 3 },
	  { 2, 3, 7, -1, 3 },
	  { 4, 5, 6, 7 , 4 },
	};

	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
	  { 1, 4, 0, -1, 3 },
	  { 0, 2, 1, -1, 3 },
	  { 0, 3, 2, -1, 3 },
	  { 0, 4, 3, -1, 3 },
	  { 1, 2, 3, 4 , 4 },
	};

	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
	  { 1, 4, 3, -1, 3 },
	  { 0, 2, 4, -1, 3 },
	  { 1, 3, 4, -1, 3 },
	  { 2, 0, 4, -1, 3 },
	  { 0, 1, 2, 3 , 4 },
	};
	static constexpr int SubPyramids[10][5] = {
		// 6 pyramids
		{ 0, 5, 13, 8, 9 },
		{ 5, 1, 6, 13, 10 },
		{ 8, 13, 7, 3, 12 },
		{ 13, 6, 2, 7, 11 },
		{ 9, 10, 11, 12, 4 },
		{ 9, 12, 11, 10, 13 },
		// 4 tets
		{ 5, 13, 9, 10, 0 },
		{ 6, 13, 10, 11, 0 },
		{ 7, 13, 11, 12, 0 },
		{ 8, 13, 12, 9, 0 },
	};

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticPyramid） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/** vtkQuadraticPyramid::GetParametricCenter（vtkQuadraticPyramid.h 内联实现：6/13, 6/13, 3/13） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = 6.0 / 13.0;
		pcoords[2] = 3.0 / 13.0;
	}

	/** vtkQuadraticPyramid::InterpolationFunctions（13 节点：(0,1)→(-1,1) 换算） */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		// VTK needs parametric coordinates to be between (0,1). Isoparametric
		// shape functions are formulated between (-1,1). Here we do a
		// coordinate system conversion from (0,1) to (-1,1).
		const double r = 2.0 * (pcoords[0] - 0.5);
		const double s = 2.0 * (pcoords[1] - 0.5);
		const double t = 2.0 * (pcoords[2] - 0.5);
		const double r2 = r * r;
		const double s2 = s * s;
		const double t2 = t * t;

		weights[0] = -(1 - r) * (1 - s) * (1 - t) *
		    (4 + 3 * r + 3 * s + 2 * r * s + 2 * t + r * t + s * t + 2 * r * s * t) / 16.0;
		weights[1] = -(1 + r) * (1 - s) * (1 - t) *
		    (4 - 3 * r + 3 * s - 2 * r * s + 2 * t - r * t + s * t - 2 * r * s * t) / 16.0;
		weights[2] = -(1 + r) * (1 + s) * (1 - t) *
		    (4 - 3 * r - 3 * s + 2 * r * s + 2 * t - r * t - s * t + 2 * r * s * t) / 16.0;
		weights[3] = -(1 - r) * (1 + s) * (1 - t) *
		    (4 + 3 * r - 3 * s - 2 * r * s + 2 * t + r * t - s * t - 2 * r * s * t) / 16.0;

		weights[4] = t * (1 + t) / 2.0;

		weights[5] = (1 - r2) * (1 - s) * (1 - t) * (2 + s + s * t) / 8.0;
		weights[6] = (1 + r) * (1 - s2) * (1 - t) * (2 - r - r * t) / 8.0;
		weights[7] = (1 - r2) * (1 + s) * (1 - t) * (2 - s - s * t) / 8.0;
		weights[8] = (1 - r) * (1 - s2) * (1 - t) * (2 + r + r * t) / 8.0;

		weights[9] = (1 - r) * (1 - s) * (1 - t2) / 4.0;
		weights[10] = (1 + r) * (1 - s) * (1 - t2) / 4.0;
		weights[11] = (1 + r) * (1 + s) * (1 - t2) / 4.0;
		weights[12] = (1 - r) * (1 + s) * (1 - t2) / 4.0;
	}

	/**
	 * vtkQuadraticPyramid::InterpolationDerivs（布局：∂N_i/∂r ×13, ∂N_i/∂s ×13, ∂N_i/∂t ×13）
	 *
	 * ⚠ 已知问题（VTK 自带，非移植错误）：VTK master 的这段导数**不是**上面
	 * InterpolationFunctions 的解析导数（例如 corners 的 dN/dr 少了一个因子、midside 的
	 * dN/dr 漏掉了 (2+s+s·t) 之类的多项式因子）。用中心差分对 VTK 原文逐点核对，相对误差
	 * 可达 O(0.1~0.3)；VTK 8.2 / 9.0 / 9.3 / master 的这段代码完全相同，属长期存在的行为。
	 * 按任务要求「逐行移植」，这里**原样照搬**，因此 QuadraticPyramid::EvaluatePosition
	 * 的牛顿反解与 VTK 用的是同一个（不自洽的）雅可比；实测域内往返仍能收敛到 ~2.4e-4
	 * （VTK_CONVERGED=1e-4 量级），与其余三个二次体单元相当。若将来要与本仓库
	 * iGameResampleToLine.cpp 里那套自洽的 13 节点金字塔基函数对齐，需要同时替换
	 * InterpolationFunctions 与 InterpolationDerivs（两者的金字塔基函数本来就不同）。
	 */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		// VTK needs parametric coordinates to be between (0,1). Isoparametric
		// shape functions are formulated between (-1,1). Here we do a
		// coordinate system conversion from (0,1) to (-1,1).
		const double r = 2 * pcoords[0] - 1;
		const double s = 2 * pcoords[1] - 1;
		const double t = 2 * pcoords[2] - 1;

		const double rm = 1.0 - r;
		const double rp = 1.0 + r;
		const double sm = 1.0 - s;
		const double sp = 1.0 + s;
		const double tm = 1.0 - t;
		const double r2 = 1.0 - r * r;
		const double t2 = 1.0 - t * t;

		// r-derivatives
		// corners
		derivs[0] = -0.125 * (sm * tm - 2.0 * r * sm * tm - s * sm * tm - t * sm * tm - 2.0 * sm * tm);
		derivs[1] = 0.125 * (sm * tm + 2.0 * r * sm * tm - s * sm * tm - t * sm * tm - 2.0 * sm * tm);
		derivs[2] = 0.125 * (sp * tm + 2.0 * r * sp * tm + s * sp * tm - t * sp * tm - 2.0 * sp * tm);
		derivs[3] = -0.125 * (sp * tm - 2.0 * r * sp * tm + s * sp * tm - t * sp * tm - 2.0 * sp * tm);
		derivs[4] = 0.0;

		// midsides of rectangles
		derivs[5] = -0.5 * r * sm * tm;
		derivs[6] = 0.25 * (tm - s * s * tm);
		derivs[7] = -0.5 * r * sp * tm;
		derivs[8] = -0.25 * (tm - s * s * tm);

		// midsides of triangles
		derivs[9] = -0.25 * (1 - s) * (1 - t * t);
		derivs[10] = 0.25 * (1 - s) * (1 - t * t);
		derivs[11] = 0.25 * (1 + s) * (1 - t * t);
		derivs[12] = -0.25 * (1 + s) * (1 - t * t);

		// s-derivatives
		// corners
		derivs[13] = -0.125 * (rm * tm - 2.0 * s * rm * tm - r * rm * tm - t * rm * tm - 2.0 * rm * tm);
		derivs[14] = -0.125 * (rp * tm - 2.0 * s * rp * tm + r * rp * tm - t * rp * tm - 2.0 * rp * tm);
		derivs[15] = 0.125 * (rp * tm + 2.0 * s * rp * tm + r * rp * tm - t * rp * tm - 2.0 * rp * tm);
		derivs[16] = 0.125 * (rm * tm + 2.0 * s * rm * tm - r * rm * tm - t * rm * tm - 2.0 * rm * tm);
		derivs[17] = 0.0;

		// midsides of rectangles
		derivs[18] = -0.25 * tm * r2;
		derivs[19] = -0.5 * tm * s * rp;
		derivs[20] = 0.25 * tm * r2;
		derivs[21] = -0.5 * tm * s * rm;

		// midsides of triangles
		derivs[22] = -0.25 * rm * t2;
		derivs[23] = -0.25 * rp * t2;
		derivs[24] = 0.25 * rp * t2;
		derivs[25] = 0.25 * rm * t2;

		// t-derivatives
		// corners
		derivs[26] = -0.125 * (rm * sm - 2.0 * t * rm * sm - r * rm * sm - s * rm * sm - 2.0 * rm * sm);
		derivs[27] = -0.125 * (rp * sm - 2.0 * t * rp * sm + r * rp * sm - s * rp * sm - 2.0 * rp * sm);
		derivs[28] = -0.125 * (rp * sp - 2.0 * t * rp * sp + r * rp * sp + s * rp * sp - 2.0 * rp * sp);
		derivs[29] = -0.125 * (rm * sp - 2.0 * t * rm * sp - r * rm * sp + s * rm * sp - 2.0 * rm * sp);
		derivs[30] = 0.5 + t;

		// midsides of rectangles
		derivs[31] = -0.25 * (sm - r * r * sm);
		derivs[32] = -0.25 * (rp - s * s * rp);
		derivs[33] = -0.25 * (sp - r * r * sp);
		derivs[34] = -0.25 * (rm - s * s * rm);

		// midsides of triangles
		derivs[35] = -0.5 * (1 - r) * (1 - s) * t;
		derivs[36] = -0.5 * (1 + r) * (1 - s) * t;
		derivs[37] = -0.5 * (1 + r) * (1 + s) * t;
		derivs[38] = -0.5 * (1 - r) * (1 + s) * t;

		// we compute derivatives in [-1; 1] but we need them in [ 0; 1]
		for (int i = 0; i < 39; i++)
		{
			derivs[i] *= 2;
		}
	}

	/**
	 * vtkQuadraticPyramid::EvaluatePosition 的逐行复刻（二次形函数 + 牛顿迭代）：
	 *   - 先对塔尖（顶点 4）特判（VTK 注释：Newton 在塔尖附近不可靠）：
	 *     dist2 = |P4 - x|²，length2 = |P4 - 底面中心|²，若 dist2 == 0 或
	 *     dist2/length2 < 1e-6 则命中，返回 pcoords=(0,0,1)、weights 为形函数、
	 *     closestPoint=x、dist2=0、返回值 1；
	 *   - 用 8 条棱（edges[] 与 vtkQuadraticPyramid.cxx 的 Edges 逐项相同）中最长者的立方
	 *     作为体积上界定行列式容差；初值固定 (0.5,0.5,0.5)（**不是** GetParametricCenter
	 *     的 6/13,6/13,3/13）；
	 *   - 迭代为**带 0.5 阻尼**的牛顿步（二次单元比线性单元多这个 1/2 因子），判据
	 *     VTK_MAX_ITERATIONS=20、VTK_CONVERGED=1e-4、VTK_DIVERGED=1e6；这里不用基类
	 *     Cell::EvaluatePositionNewton（JᵀJ 最小二乘、无阻尼，域内输出语义也不同）；
	 *   - 失败（行列式过小 / 参数发散 / 不收敛）时 VTK 直接 return -1 且不写输出参数：
	 *     这里按约定补中性权重 weights[0]=1（其余 0）与自洽的 pcoords=(0,0,0)、
	 *     closestPoint=P0、dist2=|x-P0|²；
	 *   - 收敛后按三个分量 ∈ [-0.001, 1.001] 判定（VTK 原文注释：XY 参数平面随 Z 收缩，
	 *     但 X、Y 始终在 [0,1] 内，故盒式判定成立）；域内返回 1 并把 closestPoint 取为 x、
	 *     dist2 = 0；域外返回 0，把 pcoords 逐分量夹到 [0,1] 后求近似最近点；
	 *   - 节点序与 VTK_QUADRATIC_PYRAMID 完全一致（0-3 底面角点、4 塔尖、5-8 底棱中点、
	 *     9-12 侧棱中点；edges/faces 表逐项相同），无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[13][3];
		for (int i = 0; i < 13; ++i) {
			const Point& P = this->GetPoint(i);
			pts[i][0] = static_cast<double>(P[0]);
			pts[i][1] = static_cast<double>(P[1]);
			pts[i][2] = static_cast<double>(P[2]);
		}
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// 退化/失败分支的统一输出：pcoords=(0,0,0) 处的形函数恰为 (1,0,...,0) 且对应顶点 0，
		// 因此中性权重、pcoords、closestPoint = P0 三者自洽
		const auto setFailedOutput = [&]() {
			pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
			for (int i = 0; i < 13; ++i) { weights[i] = 0.0; }
			weights[0] = 1.0;
			closestPoint = Point(static_cast<float>(pts[0][0]), static_cast<float>(pts[0][1]),
			                     static_cast<float>(pts[0][2]));
			dist2 = Distance2BetweenPoints(pts[0], xd);
			return -1;
		};

		// There are problems searching for the apex point so we check if
		// we are there first before doing the full parametric inversion.
		const double* apexPoint = pts[4];
		dist2 = Distance2BetweenPoints(apexPoint, xd);
		double baseMidpoint[3] = {pts[0][0], pts[0][1], pts[0][2]};
		for (int i = 1; i < 4; ++i) {
			for (int j = 0; j < 3; ++j) { baseMidpoint[j] += pts[i][j]; }
		}
		for (int i = 0; i < 3; ++i) { baseMidpoint[i] /= 4.0; }

		const double length2 = Distance2BetweenPoints(apexPoint, baseMidpoint);
		// we use .001 as the relative tolerance here since that is the same
		// that is used for the interior cell check below but we need to
		// square it here because we're looking at dist2^2.
		if (dist2 == 0. || (length2 != 0. && dist2 / length2 < 1.e-6)) {
			pcoords[0] = pcoords[1] = 0.0;
			pcoords[2] = 1.0;
			this->InterpolationFunctions(pcoords, weights);
			// VTK 原文中 closestPoint/dist2 的赋值位于 `if (closestPoint)` 内，
			// 我们的出参是引用（永不为空），因此等价于无条件执行
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		double derivs[39];

		// compute a bound on the volume to get a scale for an acceptable determinant
		double longestEdge = 0.0;
		for (int i = 0; i < 8; ++i) {
			longestEdge = std::max(longestEdge, Distance2BetweenPoints(pts[edges[i][0]],
			                                                           pts[edges[i][1]]));
		}
		// longestEdge value is already squared
		const double volumeBound = longestEdge * std::sqrt(longestEdge);
		const double determinantTolerance =
		    1e-20 < 0.00001 * volumeBound ? 1e-20 : 0.00001 * volumeBound;

		// set initial position for Newton's method
		double params[3] = {0.5, 0.5, 0.5};
		pcoords[0] = pcoords[1] = pcoords[2] = 0.5;

		constexpr double kConverged = 1e-4; // VTK_CONVERGED
		constexpr double kDiverged = 1e6;   // VTK_DIVERGED
		constexpr int kMaxIterations = 20;  // VTK_MAX_ITERATIONS

		// enter iteration loop
		int converged = 0;
		for (int iteration = 0; !converged && iteration < kMaxIterations; ++iteration) {
			// calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			// calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 13; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 13];
					tcol[j] += coord * derivs[i + 26];
				}
			}
			for (int i = 0; i < 3; ++i) { fcol[i] -= xd[i]; }

			// compute determinants and generate improvements
			const double d = Determinant3x3(rcol, scol, tcol);
			if (std::fabs(d) < determinantTolerance) { return setFailedOutput(); }

			// 二次单元的牛顿步带 0.5 阻尼（与线性 tetra/wedge/hexahedron 不同）
			pcoords[0] = params[0] - 0.5 * Determinant3x3(fcol, scol, tcol) / d;
			pcoords[1] = params[1] - 0.5 * Determinant3x3(rcol, fcol, tcol) / d;
			pcoords[2] = params[2] - 0.5 * Determinant3x3(rcol, scol, fcol) / d;

			// check for convergence
			if (std::fabs(pcoords[0] - params[0]) < kConverged &&
			    std::fabs(pcoords[1] - params[1]) < kConverged &&
			    std::fabs(pcoords[2] - params[2]) < kConverged) {
				converged = 1;
			}
			// Test for bad divergence (S.Hirschberg 11.12.2001)
			else if (std::fabs(pcoords[0]) > kDiverged || std::fabs(pcoords[1]) > kDiverged ||
			         std::fabs(pcoords[2]) > kDiverged) {
				return setFailedOutput();
			}
			// if not converged, repeat
			else {
				params[0] = pcoords[0];
				params[1] = pcoords[1];
				params[2] = pcoords[2];
			}
		}

		// if not converged, set the parametric coordinates to arbitrary values
		// outside of element
		if (!converged) { return setFailedOutput(); }

		this->InterpolationFunctions(pcoords, weights);

		// This is correct in that the XY parametric coordinate plane "shrinks"
		// while Z increases and X and Y always are between 0 and 1.
		// （VTK 原文此处硬编码 -0.001 / 1.001，恰为 kVTKParametricTol）
		if (pcoords[0] >= -kVTKParametricTol && pcoords[0] <= 1.0 + kVTKParametricTol &&
		    pcoords[1] >= -kVTKParametricTol && pcoords[1] <= 1.0 + kVTKParametricTol &&
		    pcoords[2] >= -kVTKParametricTol && pcoords[2] <= 1.0 + kVTKParametricTol) {
			// 单元内：VTK 直接把 closestPoint 取为 x、dist2 = 0（不做距离判定）
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		{
			double pc[3];
			for (int i = 0; i < 3; ++i) { // only approximate, not really true for warped pyramid
				if (pcoords[i] < 0.0) { pc[i] = 0.0; }
				else if (pcoords[i] > 1.0) { pc[i] = 1.0; }
				else { pc[i] = pcoords[i]; }
			}
			// 等价 vtkQuadraticPyramid::EvaluateLocation（内联 double 计算，避免 float 往返损失）
			double w[13];
			this->InterpolationFunctions(pc, w);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 13; ++i) {
				for (int j = 0; j < 3; ++j) { cp[j] += w[i] * pts[i][j]; }
			}
			closestPoint = Point(static_cast<float>(cp[0]), static_cast<float>(cp[1]),
			                     static_cast<float>(cp[2]));
			dist2 = Distance2BetweenPoints(cp, xd);
		}
		return 0;
	}

	int GetEdgePointIds(const int edgeId, const igIndex*& pts) override {
		pts = edges[edgeId];
		return 3;
	}
	int GetFacePointIds(const int faceId, const igIndex*& pts) override {
		pts = faces[faceId];
		return faces[faceId][MaxFaceSize];
	}
	int GetFaceEdgeIds(const int faceId, const igIndex*& edgeIds) override {
		edgeIds = faceEdges[faceId];
		return faceEdges[faceId][MaxFaceSize / 2];
	}
	int GetEdgeToNeighborFaces(const int edgeId, const igIndex*& pts) override {
		pts = edgeToNeighborFaces[edgeId];
		return 2;
	}
	int GetFaceToNeighborFaces(const int faceId, const igIndex*& faceIds) override {
		faceIds = faceToNeighborFaces[faceId];
		return faceToNeighborFaces[faceId][MaxFaceSize / 2];
	}
	int GetPointToNeighborEdges(const int pointId, const igIndex*& edgeIds) override {
		edgeIds = pointToNeighborEdges[pointId];
		return pointToNeighborEdges[pointId][MaxValence];
	}
	int GetPointToNeighborFaces(const int pointId, const igIndex*& faceIds) override {
		faceIds = pointToNeighborFaces[pointId];
		return pointToNeighborFaces[pointId][MaxValence];
	}
	int GetPointToOneRingPoints(const int pointId, const igIndex*& pts) override {
		pts = pointToOneRingPoints[pointId];
		return pointToOneRingPoints[pointId][MaxValence];
	}

private:
	QuadraticPyramid()
	{
		this->m_Points->SetNumberOfPoints(NumberOfPoints);
		this->m_PointIds->SetNumberOfIds(NumberOfPoints);
		for (int i = 0; i < NumberOfPoints; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = QuadraticLine::New();
		m_Triangle = QuadraticTriangle::New();
		m_Quad = QuadraticQuad::New();
	}
	~QuadraticPyramid() override = default;

	QuadraticLine::Pointer m_Line;
	QuadraticTriangle::Pointer m_Triangle;
	QuadraticQuad::Pointer m_Quad;
};


IGAME_NAMESPACE_END
#endif