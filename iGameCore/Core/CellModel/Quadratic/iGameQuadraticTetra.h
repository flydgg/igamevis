#ifndef iGameQuadraticTetra_h
#define iGameQuadraticTetra_h

#include "Base/iGameQuadraticVolume.h"
#include "iGameQuadraticTriangle.h"
IGAME_NAMESPACE_BEGIN
class QuadraticTetra : public QuadraticVolume {
public:
	I_OBJECT(QuadraticTetra);
	static Pointer New() { return new QuadraticTetra; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_TETRA; }
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
		for (int i = 0; i < 6; ++i) {
			m_Triangle->m_PointIds->SetId(i,  m_PointIds->GetId(verts[i]));
			m_Triangle->m_Points->SetPoint(i, m_Points->GetPoint(verts[i]));
		}
        if (EdgeIds && EdgeIds->GetNumberOfIds() > 0) {
            for (int i = 0; i < 3; ++i) { m_Triangle->EdgeIds->SetId(i, EdgeIds->GetId(edges[i])); }
		}
		return m_Triangle.get();
	}

	static constexpr int NumberOfPoints = 10;

	static constexpr int NumberOfEdges = 6;

	static constexpr int NumberOfFaces = 4;

	static constexpr int MaxFaceSize = 6;

	static constexpr int MaxValence = 3;

	static constexpr int edges[NumberOfEdges][3] = {
	  { 0, 1, 4},
	  { 1, 2, 5},
	  { 2, 0, 6},
	  { 0, 3, 7},
	  { 1, 3, 8},
	  { 2, 3, 9},
	};

	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 1, 3, 4, 8, 7, 6 },
	  { 1, 2, 3, 5, 9, 8, 6 },
	  { 2, 0, 3, 6, 7, 9, 6 },
	  { 0, 2, 1, 6, 5, 4, 6 },
	};

	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 0, 4, 3, 3 },
	  { 1, 5, 4, 3 },
	  { 2, 3, 5, 3 },
	  { 2, 1, 0, 3 },
	};

	static constexpr int edgeToNeighborFaces[NumberOfEdges][2] = {
	  { 0, 3 },
	  { 1, 3 },
	  { 2, 3 },
	  { 0, 2 },
	  { 0, 1 },
	  { 1, 2 },
	};

	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 3, 1, 2, 3 },
	  { 3, 2, 0, 3 },
	  { 3, 0, 1, 3 },
	  { 2, 1, 0, 3 },
	};

	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
	  { 0, 3, 2, 3 },
	  { 0, 1, 4, 3 },
	  { 1, 2, 5, 3 },
	  { 3, 4, 5, 3 },
	};

	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
	  { 0, 2, 3, 3 },
	  { 3, 1, 0, 3 },
	  { 3, 2, 1, 3 },
	  { 0, 1, 2, 3 },
	};

	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
	  { 1, 3, 2, 3 },
	  { 0, 2, 3, 3 },
	  { 1, 0, 3, 3 },
	  { 0, 1, 2, 3 },
	};

	static int constexpr SubTetras[3][8][4] = {
	  { { 0, 4, 6, 7 }, { 4, 1, 5, 8 }, { 6, 5, 2, 9 }, { 7, 8, 9, 3 }, { 5, 9, 6, 7 }, { 5, 6, 4, 7 },
	    { 5, 4, 8, 7 }, { 5, 8, 9, 7 } },
	  { { 0, 4, 6, 7 }, { 4, 1, 5, 8 }, { 6, 5, 2, 9 }, { 7, 8, 9, 3 }, { 6, 4, 5, 8 }, { 6, 5, 9, 8 },
		{ 6, 9, 7, 8 }, { 6, 7, 4, 8 } },
	  { { 0, 4, 6, 7 }, { 4, 1, 5, 8 }, { 6, 5, 2, 9 }, { 7, 8, 9, 3 }, { 4, 8, 5, 9 }, { 4, 5, 6, 9 },
		{ 4, 6, 7, 9 }, { 4, 7, 8, 9 } }
	};

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticTetra） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/** vtkQuadraticTetra::GetParametricCenter（vtkQuadraticTetra.cxx：重心参数中心） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = pcoords[2] = 0.25;
	}

	/** vtkQuadraticTetra::InterpolationFunctions（u = L0 = 1-r-s-t，角点 + 6 个棱中点） */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		const double r = pcoords[0];
		const double s = pcoords[1];
		const double t = pcoords[2];
		const double u = 1.0 - r - s - t;

		// corners
		weights[0] = u * (2.0 * u - 1.0);
		weights[1] = r * (2.0 * r - 1.0);
		weights[2] = s * (2.0 * s - 1.0);
		weights[3] = t * (2.0 * t - 1.0);

		// midedge
		weights[4] = 4.0 * u * r;
		weights[5] = 4.0 * r * s;
		weights[6] = 4.0 * s * u;
		weights[7] = 4.0 * u * t;
		weights[8] = 4.0 * r * t;
		weights[9] = 4.0 * s * t;
	}

	/** vtkQuadraticTetra::InterpolationDerivs（布局：∂N_i/∂r ×10, ∂N_i/∂s ×10, ∂N_i/∂t ×10） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double r = pcoords[0];
		const double s = pcoords[1];
		const double t = pcoords[2];

		// r-derivatives: dW0/dr to dW9/dr
		derivs[0] = 4.0 * (r + s + t) - 3.0;
		derivs[1] = 4.0 * r - 1.0;
		derivs[2] = 0.0;
		derivs[3] = 0.0;
		derivs[4] = 4.0 - 8.0 * r - 4.0 * s - 4.0 * t;
		derivs[5] = 4.0 * s;
		derivs[6] = -4.0 * s;
		derivs[7] = -4.0 * t;
		derivs[8] = 4.0 * t;
		derivs[9] = 0.0;

		// s-derivatives: dW0/ds to dW9/ds
		derivs[10] = 4.0 * (r + s + t) - 3.0;
		derivs[11] = 0.0;
		derivs[12] = 4.0 * s - 1.0;
		derivs[13] = 0.0;
		derivs[14] = -4.0 * r;
		derivs[15] = 4.0 * r;
		derivs[16] = 4.0 - 4.0 * r - 8.0 * s - 4.0 * t;
		derivs[17] = -4.0 * t;
		derivs[18] = 0.0;
		derivs[19] = 4.0 * t;

		// t-derivatives: dW0/dt to dW9/dt
		derivs[20] = 4.0 * (r + s + t) - 3.0;
		derivs[21] = 0.0;
		derivs[22] = 0.0;
		derivs[23] = 4.0 * t - 1.0;
		derivs[24] = -4.0 * r;
		derivs[25] = 0.0;
		derivs[26] = -4.0 * s;
		derivs[27] = 4.0 - 4.0 * r - 4.0 * s - 8.0 * t;
		derivs[28] = 4.0 * r;
		derivs[29] = 4.0 * s;
	}

	/**
	 * vtkQuadraticTetra::EvaluatePosition 的逐行复刻（二次形函数 + 牛顿迭代）：
	 *   - 用 6 条棱（edges[] 的 [0][1] 两列与 vtkQuadraticTetra.cxx 的 Edges 逐项相同）
	 *     中最长者的立方作为体积上界，据此定行列式容差；
	 *   - 初值固定 (0.25,0.25,0.25)（VTK 原文，恰与 GetParametricCenter 相同）；
	 *   - 迭代是**带 0.5 阻尼**的牛顿步（二次单元比线性单元多这个 1/2 因子）：
	 *     pcoords = params - 0.5·det(fcol,scol,tcol)/d，收敛判据 VTK_CONVERGED=1e-4、
	 *     发散判据 VTK_DIVERGED=1e6、上限 VTK_MAX_ITERATIONS=20；这里不用基类
	 *     Cell::EvaluatePositionNewton，因为该基类走的是 JᵀJ 最小二乘、无阻尼，
	 *     且域内输出语义（closestPoint/dist2）与 VTK 二次单元不同；
	 *   - 失败（行列式过小 / 参数发散 / 不收敛）时 VTK 直接 return -1 且不写任何输出参数：
	 *     这里按约定补中性权重 weights[0]=1（其余 0）与自洽的 pcoords=(0,0,0)、
	 *     closestPoint=P0、dist2=|x-P0|²（(0,0,0) 处形函数恰为 (1,0,...,0)）；
	 *   - 收敛后按 [-0.001, 1.001]³ 且 r+s+t ≤ 1.001 判定参数域；域内返回 1 并照 VTK
	 *     原文把 closestPoint 取为 x、dist2 置 0（与点到单元的距离无关）；域外返回 0，
	 *     把 pcoords 逐分量夹到 [0,1] 后求近似最近点（VTK 注释：warped 时只是近似）；
	 *   - 节点序与 VTK_QUADRATIC_TETRA 完全一致（棱表逐项相同），无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[10][3];
		for (int i = 0; i < 10; ++i) {
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
			for (int i = 0; i < 10; ++i) { weights[i] = 0.0; }
			weights[0] = 1.0;
			closestPoint = Point(static_cast<float>(pts[0][0]), static_cast<float>(pts[0][1]),
			                     static_cast<float>(pts[0][2]));
			dist2 = Distance2BetweenPoints(pts[0], xd);
			return -1;
		};

		// compute a bound on the volume to get a scale for an acceptable determinant
		// （VTK 用 6 条棱中最长者；本类 edges[] 的前两列与 vtkQuadraticTetra::Edges 相同）
		double longestEdge = 0.0;
		for (int i = 0; i < 6; ++i) {
			longestEdge = std::max(longestEdge, Distance2BetweenPoints(pts[edges[i][0]],
			                                                           pts[edges[i][1]]));
		}
		// longestEdge value is already squared
		const double volumeBound = longestEdge * std::sqrt(longestEdge);
		const double determinantTolerance =
		    1e-20 < 0.00001 * volumeBound ? 1e-20 : 0.00001 * volumeBound;

		// set initial position for Newton's method
		double params[3] = {0.25, 0.25, 0.25};
		pcoords[0] = pcoords[1] = pcoords[2] = 0.25;

		constexpr double kConverged = 1e-4; // VTK_CONVERGED
		constexpr double kDiverged = 1e6;   // VTK_DIVERGED
		constexpr int kMaxIterations = 20;  // VTK_MAX_ITERATIONS

		// enter iteration loop
		int converged = 0;
		for (int iteration = 0; !converged && iteration < kMaxIterations; ++iteration) {
			double derivs[30];
			// calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			// calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 10; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 10];
					tcol[j] += coord * derivs[i + 20];
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
		    pcoords[2] >= -kVTKParametricTol && pcoords[2] <= 1.0 + kVTKParametricTol &&
		    pcoords[0] + pcoords[1] + pcoords[2] <= 1.0 + kVTKParametricTol) {
			// 单元内：VTK 直接把 closestPoint 取为 x、dist2 = 0（不做距离判定）
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		{
			double pc[3];
			for (int i = 0; i < 3; ++i) { // only approximate, not really true for warped tetra
				if (pcoords[i] < 0.0) { pc[i] = 0.0; }
				else if (pcoords[i] > 1.0) { pc[i] = 1.0; }
				else { pc[i] = pcoords[i]; }
			}
			// 等价 vtkQuadraticTetra::EvaluateLocation（内联 double 计算，避免 float 往返损失）
			double w[10];
			this->InterpolationFunctions(pc, w);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 10; ++i) {
				for (int j = 0; j < 3; ++j) { cp[j] += w[i] * pts[i][j]; }
			}
			closestPoint = Point(static_cast<float>(cp[0]), static_cast<float>(cp[1]),
			                     static_cast<float>(cp[2]));
			dist2 = Distance2BetweenPoints(cp, xd);
		}
		return 0;
	}

	int GetEdgePointIds(const int edgeId, const igIndex*& ptIds) override {
		ptIds = edges[edgeId];
		return 3;
	}
	int GetFacePointIds(const int faceId, const igIndex*& ptIds) override {
		ptIds = faces[faceId];
		return faces[faceId][MaxFaceSize];
	}
	int GetFaceEdgeIds(const int faceId, const igIndex*& edgeIds) override {
		edgeIds = faceEdges[faceId];
		return faceEdges[faceId][MaxFaceSize / 2];
	}

	int GetPointToOneRingPoints(const int pointId, const igIndex*& ptIds) override {
		ptIds = pointToOneRingPoints[pointId];
		return pointToOneRingPoints[pointId][MaxValence];
	}
	int GetPointToNeighborEdges(const int pointId, const igIndex*& edgeIds) override {
		edgeIds = pointToNeighborEdges[pointId];
		return pointToNeighborEdges[pointId][MaxValence];
	}
	int GetPointToNeighborFaces(const int pointId, const igIndex*& faceIds) override {
		faceIds = pointToNeighborFaces[pointId];
		return pointToNeighborFaces[pointId][MaxValence];
	}
	int GetEdgeToNeighborFaces(const int edgeId, const igIndex*& ptIds) override {
		ptIds = edgeToNeighborFaces[edgeId];
		return 2;
	}
	int GetFaceToNeighborFaces(const int faceId, const igIndex*& faceIds) override {
		faceIds = faceToNeighborFaces[faceId];
		return faceToNeighborFaces[faceId][MaxFaceSize / 2];
	}

	static int EdgePointIds(const int edgeId, const igIndex*& ptIds) {
		ptIds = edges[edgeId];
		return 3;
	}
	static int FacePointIds(const int faceId, const igIndex*& ptIds) {
		ptIds = faces[faceId];
		return faces[faceId][MaxFaceSize];
	}
	static int FaceEdgeIds(const int faceId, const igIndex*& edgeIds) {
		edgeIds = faceEdges[faceId];
		return faceEdges[faceId][MaxFaceSize / 2];
	}

	static int PointToOneRingPoints(const int pointId, const igIndex*& ptIds) {
		ptIds = pointToOneRingPoints[pointId];
		return pointToOneRingPoints[pointId][MaxValence];
	}
	static int PointToNeighborEdges(const int pointId, const igIndex*& edgeIds) {
		edgeIds = pointToNeighborEdges[pointId];
		return pointToNeighborEdges[pointId][MaxValence];
	}
	static int PointToNeighborFaces(const int pointId, const igIndex*& faceIds) {
		faceIds = pointToNeighborFaces[pointId];
		return pointToNeighborFaces[pointId][MaxValence];
	}
	static int EdgeToNeighborFaces(const int edgeId, const igIndex*& ptIds) {
		ptIds = edgeToNeighborFaces[edgeId];
		return 2;
	}
	static int FaceToNeighborFaces(const int faceId, const igIndex*& faceIds) {
		faceIds = faceToNeighborFaces[faceId];
		return faceToNeighborFaces[faceId][MaxFaceSize / 2];
	}

private:
	QuadraticTetra()
	{
		this->m_Points->SetNumberOfPoints(10);
		this->m_PointIds->SetNumberOfIds(10);
		for (int i = 0; i < 10; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = QuadraticLine::New();
		m_Triangle = QuadraticTriangle::New();
	}
	~QuadraticTetra() override = default;

	QuadraticLine::Pointer m_Line;
	QuadraticTriangle::Pointer m_Triangle;
};


IGAME_NAMESPACE_END
#endif