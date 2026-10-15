#ifndef iGameQuadraticPrism_h
#define iGameQuadraticPrism_h

#include "iGameVolume.h"
#include "iGameQuadraticTriangle.h"
#include "iGameQuadraticQuad.h"
#include "./Base/iGameQuadraticVolume.h"

IGAME_NAMESPACE_BEGIN
class QuadraticPrism : public QuadraticVolume {
public:
	I_OBJECT(QuadraticPrism);
	static Pointer New() { return new QuadraticPrism; }

	IGenum GetCellType() const noexcept override { return IG_QUADRATIC_PRISM; }
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
		if (faces[faceId][MaxFaceSize] == 8) {
			face = m_Triangle.get();
		}
		else {
			face = m_Quad.get();
		}
		for (int i = 0; i < verts[MaxFaceSize]; ++i) {
			face->m_PointIds->SetId(i, this->m_PointIds->GetId(verts[i]));
			face->m_Points->SetPoint(i, this->m_Points->GetPoint(verts[i]));
        }
        if (EdgeIds && EdgeIds->GetNumberOfIds() > 0) {
            for (int i = 0; i < edges[MaxFaceSize / 2]; ++i) { face->EdgeIds->SetId(i, EdgeIds->GetId(edges[i])); }
        }
		return face;
	}

	static constexpr int NumberOfPoints = 15;

	static constexpr int NumberOfEdges = 9;

	static constexpr int NumberOfFaces = 5;

	static constexpr int MaxFaceSize = 8;

	static constexpr int MaxValence = 3;

	static constexpr int edges[NumberOfEdges][3] = {
	  { 0, 1 ,6 },
	  { 1, 2 ,7 },
	  { 2, 0 ,8 },
	  { 3, 4 ,9 },
	  { 4, 5 ,10 },
	  { 5, 3 ,11 },
	  { 0, 3 ,12 },
	  { 1, 4 ,13 },
	  { 2, 5 ,14 },
	};

	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 1, 2, 6, 7, 8, -1, -1, 6 },
	  { 3, 5, 4, 11, 10, 9, -1, -1, 6 },
	  { 0, 3, 4, 1, 12, 9, 13, 6, 8 },
	  { 1, 4, 5, 2, 13, 10, 14, 7, 8 },
	  { 2, 5, 3, 0, 14, 11, 12, 8, 8 },
	};

	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 0, 1, 2, -1, 3},
	  { 5, 4, 3, -1, 3 },
	  { 6, 3, 7, 0, 4 },
	  { 7, 4, 8, 1, 4 },
	  { 8, 5, 6, 2, 4 },
	};

	static constexpr int edgeToNeighborFaces[NumberOfEdges][2] = {
	  { 0, 2 },
	  { 0, 3 },
	  { 0, 4 },
	  { 1, 2 },
	  { 1, 3 },
	  { 1, 4 },
	  { 2, 4 },
	  { 2, 3 },
	  { 3, 4 },
	};

	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize / 2 + 1] = {
	  { 4, 3, 2, -1, 3 },
	  { 2, 3, 4, -1, 3 },
	  { 0, 3, 1, 4 , 4 },
	  { 0, 4, 1, 2 , 4 },
	  { 0, 2, 1, 3 , 4 },
	};

	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
	  { 0, 6, 2, 3 },
	  { 0, 1, 7, 3 },
	  { 1, 2, 8, 3 },
	  { 3, 5, 6, 3 },
	  { 3, 7, 4, 3 },
	  { 4, 8, 5, 3 },
	};

	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
	  { 2, 4, 0, 3 },
	  { 0, 3, 2, 3 },
	  { 0, 4, 3, 3 },
	  { 1, 4, 2, 3 },
	  { 2, 3, 1, 3 },
	  { 3, 4, 1, 3 },
	};

	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
	  { 1, 3, 2, 3 },
	  { 0, 2, 4, 3 },
	  { 1, 0, 5, 3 },
	  { 4, 5, 0, 3 },
	  { 3, 1, 5, 3 },
	  { 4, 2, 3, 3 },
	};

	/* ---------------- VTK 风格插值接口（对照 vtkQuadraticWedge） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/** vtkQuadraticWedge::GetParametricCenter（vtkQuadraticWedge.h 内联实现） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = 1.0 / 3.0;
		pcoords[2] = 0.5;
	}

	/** vtkQuadraticWedge::InterpolationFunctions（15 节点二次楔形：三角形二次 × 轴向二次） */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		// VTK needs parametric coordinates to be between (0,1). Isoparametric
		// shape functions are formulated between (-1,1). Here we do a
		// coordinate system conversion from (0,1) to (-1,1).
		const double r = pcoords[0];
		const double s = pcoords[1];
		const double t = pcoords[2];
		// corners
		weights[0] = 2 * (1 - r - s) * (1 - t) * (.5 - r - s - t);
		weights[1] = 2 * r * (1 - t) * (r - t - 0.5);
		weights[2] = 2 * s * (1 - t) * (s - t - 0.5);
		weights[3] = 2 * (1 - r - s) * t * (t - r - s - 0.5);
		weights[4] = 2 * r * t * (r + t - 1.5);
		weights[5] = 2 * s * t * (s + t - 1.5);

		// midsides of triangles
		weights[6] = 4 * r * (1 - r - s) * (1 - t);
		weights[7] = 4 * r * s * (1 - t);
		weights[8] = 4 * (1 - r - s) * s * (1 - t);
		weights[9] = 4 * r * (1 - r - s) * t;
		weights[10] = 4 * r * s * t;
		weights[11] = 4 * (1 - r - s) * s * t;

		// midsides of rectangles
		weights[12] = 4 * t * (1 - r - s) * (1 - t);
		weights[13] = 4 * t * r * (1 - t);
		weights[14] = 4 * t * s * (1 - t);
	}

	/** vtkQuadraticWedge::InterpolationDerivs（布局：∂N_i/∂r ×15, ∂N_i/∂s ×15, ∂N_i/∂t ×15） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		// VTK needs parametric coordinates to be between (0,1). Isoparametric
		// shape functions are formulated between (-1,1). Here we do a
		// coordinate system conversion from (0,1) to (-1,1).
		const double r = pcoords[0];
		const double s = pcoords[1];
		const double t = pcoords[2];
		// r-derivatives
		// corners
		derivs[0] = 2 * (1 - t) * (-1.5 + 2 * r + 2 * s + t);
		derivs[1] = 2 * (1 - t) * (-0.5 + 2 * r - t);
		derivs[2] = 0;
		derivs[3] = 2 * t * (-0.5 + 2 * r + 2 * s - t);
		derivs[4] = 2 * t * (-1.5 + 2 * r + t);
		derivs[5] = 0;
		// midsides of triangles
		derivs[6] = 4 * (1 - t) * (1 - 2 * r - s);
		derivs[7] = 4 * (1 - t) * s;
		derivs[8] = -derivs[7];
		derivs[9] = 4 * t * (1 - 2 * r - s);
		derivs[10] = 4 * s * t;
		derivs[11] = -derivs[10];
		// midsides of rectangles
		derivs[12] = -4 * t * (1 - t);
		derivs[13] = -derivs[12];
		derivs[14] = 0;

		// s-derivatives
		// corners
		derivs[15] = derivs[0];
		derivs[16] = 0;
		derivs[17] = 2 * (1 - t) * (-0.5 + 2 * s - t);
		derivs[18] = derivs[3];
		derivs[19] = 0;
		derivs[20] = 2 * t * (-1.5 + 2 * s + t);
		// midsides of triangles
		derivs[21] = -4 * (1 - t) * r;
		derivs[22] = -derivs[21];
		derivs[23] = 4 * (1 - t) * (1 - r - 2 * s);
		derivs[24] = -4 * r * t;
		derivs[25] = -derivs[24];
		derivs[26] = 4 * t * (1 - r - 2 * s);
		// midsides of rectangles
		derivs[27] = derivs[12];
		derivs[28] = 0;
		derivs[29] = -derivs[27];

		// t-derivatives
		// corners
		derivs[30] = 2 * (1 - r - s) * (-1.5 + r + s + 2 * t);
		derivs[31] = 2 * r * (-0.5 - r + 2 * t);
		derivs[32] = 2 * s * (-0.5 - s + 2 * t);
		derivs[33] = 2 * (1 - r - s) * (-0.5 - r - s + 2 * t);
		derivs[34] = 2 * r * (-1.5 + r + 2 * t);
		derivs[35] = 2 * s * (-1.5 + s + 2 * t);
		// midsides of triangles
		derivs[36] = -4 * r * (1 - r - s);
		derivs[37] = -4 * r * s;
		derivs[38] = -4 * s * (1 - r - s);
		derivs[39] = -derivs[36];
		derivs[40] = -derivs[37];
		derivs[41] = -derivs[38];
		// midsides of rectangles
		derivs[42] = 4 * (1 - 2 * t) * (1 - r - s);
		derivs[43] = 4 * (1 - 2 * t) * r;
		derivs[44] = 4 * (1 - 2 * t) * s;
	}

	/**
	 * vtkQuadraticWedge::EvaluatePosition 的逐行复刻（二次形函数 + 牛顿迭代）：
	 *   - 用 9 条棱（edges[] 的 [0][1] 两列与 vtkQuadraticWedge.cxx 的 Edges 逐项相同）
	 *     中最长者的立方作为体积上界，据此定行列式容差；
	 *   - 初值固定 (0.5,0.5,0.5)；迭代为**带 0.5 阻尼**的牛顿步（二次单元比线性单元
	 *     多这个 1/2 因子），判据 VTK_MAX_ITERATIONS=20、VTK_CONVERGED=1e-4、
	 *     VTK_DIVERGED=1e6；这里不用基类 Cell::EvaluatePositionNewton（JᵀJ 最小二乘、
	 *     无阻尼，且域内输出语义与 VTK 二次单元不同）；
	 *   - 失败（行列式过小 / 参数发散 / 不收敛）时 VTK 直接 return -1 且不写输出参数：
	 *     这里按约定补中性权重 weights[0]=1（其余 0）与自洽的 pcoords=(0,0,0)、
	 *     closestPoint=P0、dist2=|x-P0|²；
	 *   - 收敛后按 VTK 原文的 [-0.001, 1.001]³ 且 r+s ≤ 1.001 判定参数域；域内返回 1 并把
	 *     closestPoint 取为 x、dist2 = 0；域外返回 0，把 pcoords 逐分量夹到 [0,1] 后求近似
	 *     最近点（VTK 注释：warped 时只是近似，不保证 r+s ≤ 1 的夹紧）；
	 *   - 节点序与 VTK_QUADRATIC_WEDGE 完全一致（棱表逐项相同），无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[15][3];
		for (int i = 0; i < 15; ++i) {
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
			for (int i = 0; i < 15; ++i) { weights[i] = 0.0; }
			weights[0] = 1.0;
			closestPoint = Point(static_cast<float>(pts[0][0]), static_cast<float>(pts[0][1]),
			                     static_cast<float>(pts[0][2]));
			dist2 = Distance2BetweenPoints(pts[0], xd);
			return -1;
		};

		// compute a bound on the volume to get a scale for an acceptable determinant
		// （VTK 用 9 条棱中最长者；本类 edges[] 的前两列与 vtkQuadraticWedge::Edges 相同）
		double longestEdge = 0.0;
		for (int i = 0; i < 9; ++i) {
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
			double derivs[45];
			// calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			// calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 15; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 15];
					tcol[j] += coord * derivs[i + 30];
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
		    pcoords[0] + pcoords[1] <= 1.0 + kVTKParametricTol) {
			// 单元内：VTK 直接把 closestPoint 取为 x、dist2 = 0（不做距离判定）
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		{
			double pc[3];
			for (int i = 0; i < 3; ++i) { // only approximate, not really true for warped prism
				if (pcoords[i] < 0.0) { pc[i] = 0.0; }
				else if (pcoords[i] > 1.0) { pc[i] = 1.0; }
				else { pc[i] = pcoords[i]; }
			}
			// 等价 vtkQuadraticWedge::EvaluateLocation（内联 double 计算，避免 float 往返损失）
			double w[15];
			this->InterpolationFunctions(pc, w);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 15; ++i) {
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
	QuadraticPrism()
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
	~QuadraticPrism() override = default;

	QuadraticLine::Pointer m_Line;
	QuadraticTriangle::Pointer m_Triangle;
	QuadraticQuad::Pointer m_Quad;
};


IGAME_NAMESPACE_END
#endif