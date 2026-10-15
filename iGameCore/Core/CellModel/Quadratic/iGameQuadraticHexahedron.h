#ifndef iGameQuadraticHexahedron_h
#define iGameQuadraticHexahedron_h

#include "Base/iGameQuadraticVolume.h"
#include "iGameQuadraticQuad.h"

IGAME_NAMESPACE_BEGIN
class QuadraticHexahedron : public QuadraticVolume {
public:
	I_OBJECT(QuadraticHexahedron);
	static Pointer New() { return new QuadraticHexahedron; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_HEXAHEDRON; }
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
		for (int i = 0; i < 8; ++i) {
			m_Quad->m_PointIds->SetId(i, m_PointIds->GetId(verts[i]));
			m_Quad->m_Points->SetPoint(i, m_Points->GetPoint(verts[i]));
        }
        if (EdgeIds && EdgeIds->GetNumberOfIds() > 0) {
            for (int i = 0; i < 4; ++i) { m_Quad->EdgeIds->SetId(i, EdgeIds->GetId(edges[i])); }
        }
		return m_Quad.get();
	}


	static constexpr int NumberOfPoints = 20;

	static constexpr int NumberOfEdges = 12;

	static constexpr int NumberOfFaces = 6;

	static constexpr int MaxFaceSize = 8;

	static constexpr int MaxValence = 3;

	static constexpr int edges[NumberOfEdges][3] = {
	  { 0, 1 ,8 },
	  { 1, 2 ,9 },
	  { 3, 2 ,10 },
	  { 0, 3 ,11 },
	  { 4, 5 ,12 },
	  { 5, 6 ,13 },
	  { 7, 6 ,14 },
	  { 4, 7 ,15 },
	  { 0, 4 ,16 },
	  { 1, 5 ,17 },
	  { 2, 6 ,18 },
	  { 3, 7 ,19 },
	};

	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 4, 7, 3, 16, 15, 19, 11, 4 },
	  { 1, 2, 6, 5, 9, 18, 13, 17, 4 },
	  { 0, 1, 5, 4, 8, 17, 12, 16, 4 },
	  { 3, 7, 6, 2, 19, 14, 18, 10, 4 },
	  { 0, 3, 2, 1, 11, 10, 9, 8, 4 },
	  { 4, 5, 6, 7, 12, 13, 14, 15, 4 },
	};

	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 8, 7, 11, 3, 4 },
	  { 1, 10, 5, 9, 4 },
	  { 0, 9, 4, 8, 4 },
	  { 11, 6, 10, 2, 4 },
	  { 3, 2, 1, 0, 4 },
	  { 4, 5, 6, 7, 4 },
	};

	static constexpr int edgeToNeighborFaces[NumberOfEdges][2] = {
	  { 2, 4 },
	  { 1, 4 },
	  { 3, 4 },
	  { 0, 4 },
	  { 2, 5 },
	  { 1, 5 },
	  { 3, 5 },
	  { 0, 5 },
	  { 0, 2 },
	  { 1, 2 },
	  { 1, 3 },
	  { 0, 3 },
	};

	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize / 2 + 1] = {
		{ 4, 2, 5, 3, 4 },
		{ 4, 3, 5, 2, 4 },
		{ 4, 1, 5, 0, 4 },
		{ 0, 5, 1, 4, 4 },
		{ 0, 3, 1, 2, 4 },
		{ 2, 1, 0, 3, 4 },
	};

	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
		{ 0, 8,  3,  3 },
		{ 0, 1,  9,  3 },
		{ 1, 2,  10, 3 },
		{ 2, 3,  11, 3 },
		{ 7, 8,  4,  3 },
		{ 4, 9,  5,  3 },
		{ 5, 10, 6,  3 },
		{ 6, 11, 7,  3 },
	};

	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
		{ 2, 0, 4, 3 },
		{ 4, 1, 2, 3 },
		{ 4, 3, 1, 3 },
		{ 4, 0, 3, 3 },
		{ 5, 2, 0, 3 },
		{ 2, 1, 5, 3 },
		{ 1, 3, 5, 3 },
		{ 3, 0, 5, 3 },
	};

	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
		{ 1, 4, 3, 3 },
		{ 0, 2, 5, 3 },
		{ 1, 3, 6, 3 },
		{ 2, 0, 7, 3 },
		{ 5, 7, 0, 3 },
		{ 4, 1, 6, 3 },
		{ 5, 2, 7, 3 },
		{ 6, 3, 4, 3 },
	};

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticHexahedron） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/**
	 * vtkQuadraticHexahedron 未覆写 GetParametricCenter，等价 vtkCell::GetParametricCenter
	 * 的默认实现（参数域中心 0.5, 0.5, 0.5；注意基类 Cell 的默认值是 (0,0,0)，故必须覆写）。
	 */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = pcoords[2] = 0.5;
	}

	/** vtkQuadraticHexahedron::InterpolationFunctions（20 节点 Serendipity，(0,1)→(-1,1) 换算） */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		// VTK needs parametric coordinates to be between (0,1). Isoparametric
		// shape functions are formulated between (-1,1). Here we do a
		// coordinate system conversion from (0,1) to (-1,1).
		const double r = 2.0 * (pcoords[0] - 0.5);
		const double s = 2.0 * (pcoords[1] - 0.5);
		const double t = 2.0 * (pcoords[2] - 0.5);

		const double rm = 1.0 - r;
		const double rp = 1.0 + r;
		const double sm = 1.0 - s;
		const double sp = 1.0 + s;
		const double tm = 1.0 - t;
		const double tp = 1.0 + t;
		const double r2 = 1.0 - r * r;
		const double s2 = 1.0 - s * s;
		const double t2 = 1.0 - t * t;

		// The eight corner points
		weights[0] = 0.125 * rm * sm * tm * (-r - s - t - 2.0);
		weights[1] = 0.125 * rp * sm * tm * (r - s - t - 2.0);
		weights[2] = 0.125 * rp * sp * tm * (r + s - t - 2.0);
		weights[3] = 0.125 * rm * sp * tm * (-r + s - t - 2.0);
		weights[4] = 0.125 * rm * sm * tp * (-r - s + t - 2.0);
		weights[5] = 0.125 * rp * sm * tp * (r - s + t - 2.0);
		weights[6] = 0.125 * rp * sp * tp * (r + s + t - 2.0);
		weights[7] = 0.125 * rm * sp * tp * (-r + s + t - 2.0);

		// The mid-edge nodes
		weights[8] = 0.25 * r2 * sm * tm;
		weights[9] = 0.25 * s2 * rp * tm;
		weights[10] = 0.25 * r2 * sp * tm;
		weights[11] = 0.25 * s2 * rm * tm;
		weights[12] = 0.25 * r2 * sm * tp;
		weights[13] = 0.25 * s2 * rp * tp;
		weights[14] = 0.25 * r2 * sp * tp;
		weights[15] = 0.25 * s2 * rm * tp;
		weights[16] = 0.25 * t2 * rm * sm;
		weights[17] = 0.25 * t2 * rp * sm;
		weights[18] = 0.25 * t2 * rp * sp;
		weights[19] = 0.25 * t2 * rm * sp;
	}

	/** vtkQuadraticHexahedron::InterpolationDerivs（布局：∂N_i/∂r ×20, ∂N_i/∂s ×20, ∂N_i/∂t ×20） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		// VTK needs parametric coordinates to be between (0,1). Isoparametric
		// shape functions are formulated between (-1,1). Here we do a
		// coordinate system conversion from (0,1) to (-1,1).
		const double r = 2.0 * (pcoords[0] - 0.5);
		const double s = 2.0 * (pcoords[1] - 0.5);
		const double t = 2.0 * (pcoords[2] - 0.5);

		const double rm = 1.0 - r;
		const double rp = 1.0 + r;
		const double sm = 1.0 - s;
		const double sp = 1.0 + s;
		const double tm = 1.0 - t;
		const double tp = 1.0 + t;

		// r-derivatives
		derivs[0] = -0.125 * (sm * tm - 2.0 * r * sm * tm - s * sm * tm - t * sm * tm - 2.0 * sm * tm);
		derivs[1] = 0.125 * (sm * tm + 2.0 * r * sm * tm - s * sm * tm - t * sm * tm - 2.0 * sm * tm);
		derivs[2] = 0.125 * (sp * tm + 2.0 * r * sp * tm + s * sp * tm - t * sp * tm - 2.0 * sp * tm);
		derivs[3] = -0.125 * (sp * tm - 2.0 * r * sp * tm + s * sp * tm - t * sp * tm - 2.0 * sp * tm);
		derivs[4] = -0.125 * (sm * tp - 2.0 * r * sm * tp - s * sm * tp + t * sm * tp - 2.0 * sm * tp);
		derivs[5] = 0.125 * (sm * tp + 2.0 * r * sm * tp - s * sm * tp + t * sm * tp - 2.0 * sm * tp);
		derivs[6] = 0.125 * (sp * tp + 2.0 * r * sp * tp + s * sp * tp + t * sp * tp - 2.0 * sp * tp);
		derivs[7] = -0.125 * (sp * tp - 2.0 * r * sp * tp + s * sp * tp + t * sp * tp - 2.0 * sp * tp);
		derivs[8] = -0.5 * r * sm * tm;
		derivs[9] = 0.25 * (tm - s * s * tm);
		derivs[10] = -0.5 * r * sp * tm;
		derivs[11] = -0.25 * (tm - s * s * tm);
		derivs[12] = -0.5 * r * sm * tp;
		derivs[13] = 0.25 * (tp - s * s * tp);
		derivs[14] = -0.5 * r * sp * tp;
		derivs[15] = -0.25 * (tp - s * s * tp);
		derivs[16] = -0.25 * (sm - t * t * sm);
		derivs[17] = 0.25 * (sm - t * t * sm);
		derivs[18] = 0.25 * (sp - t * t * sp);
		derivs[19] = -0.25 * (sp - t * t * sp);

		// s-derivatives
		derivs[20] = -0.125 * (rm * tm - 2.0 * s * rm * tm - r * rm * tm - t * rm * tm - 2.0 * rm * tm);
		derivs[21] = -0.125 * (rp * tm - 2.0 * s * rp * tm + r * rp * tm - t * rp * tm - 2.0 * rp * tm);
		derivs[22] = 0.125 * (rp * tm + 2.0 * s * rp * tm + r * rp * tm - t * rp * tm - 2.0 * rp * tm);
		derivs[23] = 0.125 * (rm * tm + 2.0 * s * rm * tm - r * rm * tm - t * rm * tm - 2.0 * rm * tm);
		derivs[24] = -0.125 * (rm * tp - 2.0 * s * rm * tp - r * rm * tp + t * rm * tp - 2.0 * rm * tp);
		derivs[25] = -0.125 * (rp * tp - 2.0 * s * rp * tp + r * rp * tp + t * rp * tp - 2.0 * rp * tp);
		derivs[26] = 0.125 * (rp * tp + 2.0 * s * rp * tp + r * rp * tp + t * rp * tp - 2.0 * rp * tp);
		derivs[27] = 0.125 * (rm * tp + 2.0 * s * rm * tp - r * rm * tp + t * rm * tp - 2.0 * rm * tp);
		derivs[28] = -0.25 * (tm - r * r * tm);
		derivs[29] = -0.5 * s * rp * tm;
		derivs[30] = 0.25 * (tm - r * r * tm);
		derivs[31] = -0.5 * s * rm * tm;
		derivs[32] = -0.25 * (tp - r * r * tp);
		derivs[33] = -0.5 * s * rp * tp;
		derivs[34] = 0.25 * (tp - r * r * tp);
		derivs[35] = -0.5 * s * rm * tp;
		derivs[36] = -0.25 * (rm - t * t * rm);
		derivs[37] = -0.25 * (rp - t * t * rp);
		derivs[38] = 0.25 * (rp - t * t * rp);
		derivs[39] = 0.25 * (rm - t * t * rm);

		// t-derivatives
		derivs[40] = -0.125 * (rm * sm - 2.0 * t * rm * sm - r * rm * sm - s * rm * sm - 2.0 * rm * sm);
		derivs[41] = -0.125 * (rp * sm - 2.0 * t * rp * sm + r * rp * sm - s * rp * sm - 2.0 * rp * sm);
		derivs[42] = -0.125 * (rp * sp - 2.0 * t * rp * sp + r * rp * sp + s * rp * sp - 2.0 * rp * sp);
		derivs[43] = -0.125 * (rm * sp - 2.0 * t * rm * sp - r * rm * sp + s * rm * sp - 2.0 * rm * sp);
		derivs[44] = 0.125 * (rm * sm + 2.0 * t * rm * sm - r * rm * sm - s * rm * sm - 2.0 * rm * sm);
		derivs[45] = 0.125 * (rp * sm + 2.0 * t * rp * sm + r * rp * sm - s * rp * sm - 2.0 * rp * sm);
		derivs[46] = 0.125 * (rp * sp + 2.0 * t * rp * sp + r * rp * sp + s * rp * sp - 2.0 * rp * sp);
		derivs[47] = 0.125 * (rm * sp + 2.0 * t * rm * sp - r * rm * sp + s * rm * sp - 2.0 * rm * sp);
		derivs[48] = -0.25 * (sm - r * r * sm);
		derivs[49] = -0.25 * (rp - s * s * rp);
		derivs[50] = -0.25 * (sp - r * r * sp);
		derivs[51] = -0.25 * (rm - s * s * rm);
		derivs[52] = 0.25 * (sm - r * r * sm);
		derivs[53] = 0.25 * (rp - s * s * rp);
		derivs[54] = 0.25 * (sp - r * r * sp);
		derivs[55] = 0.25 * (rm - s * s * rm);
		derivs[56] = -0.5 * t * rm * sm;
		derivs[57] = -0.5 * t * rp * sm;
		derivs[58] = -0.5 * t * rp * sp;
		derivs[59] = -0.5 * t * rm * sp;

		// we compute derivatives in [-1; 1] but we need them in [ 0; 1]
		for (int i = 0; i < 60; i++)
		{
			derivs[i] *= 2;
		}
	}

	/**
	 * vtkQuadraticHexahedron::EvaluatePosition 的逐行复刻（二次形函数 + 牛顿迭代）：
	 *   - 用 4 条体对角线 {{0,6},{1,7},{2,4},{3,5}}（与 VTK 原文相同）最长者的立方作为
	 *     体积上界，据此定行列式容差；
	 *   - 初值固定 (0.5,0.5,0.5)（VTK 原文，与 vtkCell::GetParametricCenter 相同）；
	 *   - 迭代是**带 0.5 阻尼**的牛顿步（二次单元比线性单元多这个 1/2 因子）；
	 *     参数与判据 VTK_MAX_ITERATIONS=20、VTK_CONVERGED=1e-4、VTK_DIVERGED=1e6；
	 *     这里不用基类 Cell::EvaluatePositionNewton（它走 JᵀJ 最小二乘、无阻尼，域内
	 *     输出语义也与 VTK 二次单元不同）；
	 *   - 失败（行列式过小 / 参数发散 / 不收敛）时 VTK 直接 return -1 且不写输出参数：
	 *     这里按约定补中性权重 weights[0]=1（其余 0）与自洽的 pcoords=(0,0,0)、
	 *     closestPoint=P0、dist2=|x-P0|²；
	 *   - 收敛后按 [-0.001, 1.001]³ 判定参数域；域内返回 1 并按 VTK 原文把 closestPoint
	 *     取为 x、dist2 = 0（与点到单元的真实距离无关）；域外返回 0，把 pcoords 逐分量
	 *     夹到 [0,1] 后求近似最近点（VTK 注释：warped 时只是近似）；
	 *   - 节点序与 VTK_QUADRATIC_HEXAHEDRON 完全一致（仅棱编号 10/11 与 VTK 互换、棱方向
	 *     记号不同，EvaluatePosition 只用对角线与棱的端点做容差，故无影响），无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[20][3];
		for (int i = 0; i < 20; ++i) {
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
			for (int i = 0; i < 20; ++i) { weights[i] = 0.0; }
			weights[0] = 1.0;
			closestPoint = Point(static_cast<float>(pts[0][0]), static_cast<float>(pts[0][1]),
			                     static_cast<float>(pts[0][2]));
			dist2 = Distance2BetweenPoints(pts[0], xd);
			return -1;
		};

		// compute a bound on the volume to get a scale for an acceptable determinant
		static constexpr int diagonals[4][2] = {{0, 6}, {1, 7}, {2, 4}, {3, 5}};
		double longestDiagonal = 0.0;
		for (int i = 0; i < 4; ++i) {
			longestDiagonal = std::max(longestDiagonal, Distance2BetweenPoints(pts[diagonals[i][0]],
			                                                                   pts[diagonals[i][1]]));
		}
		// longestDiagonal value is already squared
		const double volumeBound = longestDiagonal * std::sqrt(longestDiagonal);
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
			double derivs[60];
			// calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			// calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 20; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 20];
					tcol[j] += coord * derivs[i + 40];
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
			for (int i = 0; i < 3; ++i) { // only approximate, not really true for warped hexa
				if (pcoords[i] < 0.0) { pc[i] = 0.0; }
				else if (pcoords[i] > 1.0) { pc[i] = 1.0; }
				else { pc[i] = pcoords[i]; }
			}
			// 等价 vtkQuadraticHexahedron::EvaluateLocation（内联 double 计算，避免 float 往返损失）
			double w[20];
			this->InterpolationFunctions(pc, w);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 20; ++i) {
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
	QuadraticHexahedron()
	{
		this->m_Points->SetNumberOfPoints(20);
		this->m_PointIds->SetNumberOfIds(20);
		for (int i = 0; i < 20; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = QuadraticLine::New();
		m_Quad = QuadraticQuad::New();
	}
	~QuadraticHexahedron() override = default;

	QuadraticLine::Pointer m_Line;
	QuadraticQuad::Pointer m_Quad;
};


IGAME_NAMESPACE_END
#endif