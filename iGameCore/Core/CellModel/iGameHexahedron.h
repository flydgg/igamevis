#ifndef iGameHexahedron_h
#define iGameHexahedron_h

#include "iGameVolume.h"
#include<iGameTetra.h>
IGAME_NAMESPACE_BEGIN
class Hexahedron : public Volume {
public:
	I_OBJECT(Hexahedron);
	static Pointer New() { return new Hexahedron; }

	IGenum GetCellType() const noexcept override { return IG_HEXAHEDRON; }
	int GetCellSize() const noexcept override { return 8; }
	int GetNumberOfEdges() override { return 12; }
	int GetNumberOfFaces() override { return 6; }

	Cell* GetEdge(const int edgeId) override {
		const int* verts = edges[edgeId];

		m_Line->m_PointIds->SetId(0, this->m_PointIds->GetId(verts[0]));
		m_Line->m_PointIds->SetId(1, this->m_PointIds->GetId(verts[1]));

		m_Line->m_Points->SetPoint(0, this->m_Points->GetPoint(verts[0]));
		m_Line->m_Points->SetPoint(1, this->m_Points->GetPoint(verts[1]));

		return m_Line.get();
	}
	Cell* GetFace(const int faceId) override {
		const int* verts = faces[faceId];
		const int* edges = faceEdges[faceId];
		for (int i = 0; i < 4; ++i) {
			m_Quad->m_PointIds->SetId(i,  m_PointIds->GetId(verts[i]));
			m_Quad->m_Points->SetPoint(i, m_Points->GetPoint(verts[i]));
		}

		return m_Quad.get();
	}
    std::vector<iGame::Cell::Pointer> clipCelltoTetra() override {
        std::vector<iGame::Cell::Pointer> result;
        for (int i = 0; i < 11; i++) {
			const int* verts = clipedCell[i];
            Tetra::Pointer tetra = Tetra::New();
			for (int j = 0; j < 4; j++) {
				tetra->m_Points->SetPoint(j, this->m_Points->GetPoint(verts[j]));
				tetra->m_PointIds->SetId(j,  this->m_PointIds->GetId(verts[j]));
			}
			result.emplace_back(tetra);
		}
		return result;
	}
	/* ---------------- VTK 风格插值接口（对照 vtkHexahedron） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/**
	 * vtkHexahedron 未覆写 GetParametricCenter，等价 vtkCell::GetParametricCenter
	 * 的默认实现（参数域中心 0.5, 0.5, 0.5）。
	 */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = pcoords[2] = 0.5;
	}

	/** vtkHexahedron::InterpolationFunctions（三线性形函数） */
	void InterpolationFunctions(const double pcoords[3], double* sf) const override {
		const double rm = 1.0 - pcoords[0];
		const double sm = 1.0 - pcoords[1];
		const double tm = 1.0 - pcoords[2];

		const double rmXsm = rm * sm;
		const double p0Xsm = pcoords[0] * sm;
		const double p0Xp1 = pcoords[0] * pcoords[1];
		const double rmXp1 = rm * pcoords[1];

		sf[0] = rmXsm * tm;
		sf[1] = p0Xsm * tm;
		sf[2] = p0Xp1 * tm;
		sf[3] = rmXp1 * tm;
		sf[4] = rmXsm * pcoords[2];
		sf[5] = p0Xsm * pcoords[2];
		sf[6] = p0Xp1 * pcoords[2];
		sf[7] = rmXp1 * pcoords[2];
	}

	/** vtkHexahedron::InterpolationDerivs（布局：∂N_i/∂r ×8, ∂N_i/∂s ×8, ∂N_i/∂t ×8） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double rm = 1.0 - pcoords[0];
		const double sm = 1.0 - pcoords[1];
		const double tm = 1.0 - pcoords[2];

		// r-derivatives
		derivs[0] = -sm * tm;
		derivs[1] = -derivs[0];
		derivs[2] = pcoords[1] * tm;
		derivs[3] = -derivs[2];
		derivs[4] = -sm * pcoords[2];
		derivs[5] = -derivs[4];
		derivs[6] = pcoords[1] * pcoords[2];
		derivs[7] = -derivs[6];

		// s-derivatives
		derivs[8] = -rm * tm;
		derivs[9] = -pcoords[0] * tm;
		derivs[10] = -derivs[9];
		derivs[11] = -derivs[8];
		derivs[12] = -rm * pcoords[2];
		derivs[13] = -pcoords[0] * pcoords[2];
		derivs[14] = -derivs[13];
		derivs[15] = -derivs[12];

		// t-derivatives
		derivs[16] = -rm * sm;
		derivs[17] = -pcoords[0] * sm;
		derivs[18] = -pcoords[0] * pcoords[1];
		derivs[19] = -rm * pcoords[1];
		derivs[20] = -derivs[16];
		derivs[21] = -derivs[17];
		derivs[22] = -derivs[18];
		derivs[23] = -derivs[19];
	}

	/** vtkMath::Determinant3x3 的复刻：det = a · (b × c) */
	static double Determinant3x3(const double a[3], const double b[3], const double c[3]) {
		return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
		       a[2] * (b[0] * c[1] - b[1] * c[0]);
	}

	/**
	 * vtkHexahedron::EvaluatePosition 的逐行复刻（三线性形函数 + 牛顿迭代）：
	 *   - 初值取顶点 0 出发的三条棱（P1-P0, P3-P0, P4-P0）构成的平行六面体的闭式解，
	 *     退化时回退到单元参数中心；
	 *   - 迭代失败（雅可比行列式小于容差 / 参数发散 / 20 次内不收敛）时 VTK 返回 -1
	 *     且不写输出参数：这里按约定补上中性权重 weights[0]=1（其余 0）以及与之自洽的
	 *     pcoords=(0,0,0)、closestPoint=P0、dist2 = |x-P0|²；
	 *   - 收敛后按 VTK_OUTSIDE_CELL_TOLERANCE = 1e-6 判定参数域（注意与三棱柱的 1e-3
	 *     不同）；落在域内返回 1，并把 closestPoint 取为 x、dist2 置 0
	 *     （VTK 语义：与点到单元的距离无关）；
	 *   - 落在域外返回 0，用 6 个（可能是非平面双线性）面各自的 EvaluatePosition 投影
	 *     取最近点，与 VTK 原文一致。
	 * 顶点序与 VTK_HEXAHEDRON 完全一致（仅棱编号 10/11 与 VTK 互换，面表逐项相同），
	 * 无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[8][3];
		for (int i = 0; i < 8; ++i) {
			const Point& P = this->GetPoint(i);
			pts[i][0] = static_cast<double>(P[0]);
			pts[i][1] = static_cast<double>(P[1]);
			pts[i][2] = static_cast<double>(P[2]);
		}
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// 退化/失败分支的统一输出：pcoords=(0,0,0) 处的三线性形函数恰为 (1,0,...,0)，
		// 且对应顶点 0，因此中性权重、pcoords、closestPoint = P0 三者自洽
		const auto setFailedOutput = [&]() {
			pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
			for (int i = 0; i < 8; ++i) { weights[i] = 0.0; }
			weights[0] = 1.0;
			closestPoint = Point(static_cast<float>(pts[0][0]), static_cast<float>(pts[0][1]),
			                     static_cast<float>(pts[0][2]));
			const double dx = pts[0][0] - xd[0];
			const double dy = pts[0][1] - xd[1];
			const double dz = pts[0][2] - xd[2];
			dist2 = dx * dx + dy * dy + dz * dz;
			return -1;
		};

		// compute a bound on the volume to get a scale for an acceptable determinant
		constexpr int diagonals[4][2] = {{0, 6}, {1, 7}, {2, 4}, {3, 5}};
		double longestDiagonal = 0.0;
		for (int i = 0; i < 4; ++i) {
			const double* a = pts[diagonals[i][0]];
			const double* b = pts[diagonals[i][1]];
			const double dx = b[0] - a[0];
			const double dy = b[1] - a[1];
			const double dz = b[2] - a[2];
			longestDiagonal = std::max(longestDiagonal, dx * dx + dy * dy + dz * dz);
		}
		// longestDiagonal value is already squared
		const double volumeBound = longestDiagonal * std::sqrt(longestDiagonal);
		const double determinantTolerance =
		    1e-20 < 0.00001 * volumeBound ? 1e-20 : 0.00001 * volumeBound;

		// set initial position for Newton's method: seed it with the exact solution of
		// the affine (parallelepiped) approximation defined by the edges out of vertex 0.
		// For a degenerate triple of edges, fall back to the cell center.
		{
			double c1[3], c2[3], c3[3], p[3];
			for (int i = 0; i < 3; ++i) {
				c1[i] = pts[1][i] - pts[0][i];
				c2[i] = pts[3][i] - pts[0][i];
				c3[i] = pts[4][i] - pts[0][i];
				p[i] = xd[i] - pts[0][i];
			}
			const double detGuess = Determinant3x3(c1, c2, c3);
			if (detGuess != 0.0) {
				pcoords[0] = Determinant3x3(p, c2, c3) / detGuess;
				pcoords[1] = Determinant3x3(c1, p, c3) / detGuess;
				pcoords[2] = Determinant3x3(c1, c2, p) / detGuess;
			} else {
				pcoords[0] = pcoords[1] = pcoords[2] = 0.5;
			}
		}
		double params[3] = {pcoords[0], pcoords[1], pcoords[2]};

		constexpr double kConverged = 1e-4; // VTK_CONVERGED
		constexpr double kDiverged = 1e6;   // VTK_DIVERGED
		constexpr int kMaxIterations = 20;  // VTK_MAX_ITERATIONS

		// enter iteration loop
		int converged = 0;
		for (int iteration = 0; !converged && iteration < kMaxIterations; ++iteration) {
			double derivs[24];
			// calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			// calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 8; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 8];
					tcol[j] += coord * derivs[i + 16];
				}
			}
			for (int i = 0; i < 3; ++i) { fcol[i] -= xd[i]; }

			// compute determinants and generate improvements
			const double d = Determinant3x3(rcol, scol, tcol);
			if (std::fabs(d) < determinantTolerance) {
				return setFailedOutput();
			}

			pcoords[0] = params[0] - Determinant3x3(fcol, scol, tcol) / d;
			pcoords[1] = params[1] - Determinant3x3(rcol, fcol, tcol) / d;
			pcoords[2] = params[2] - Determinant3x3(rcol, scol, fcol) / d;

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
		if (!converged) {
			return setFailedOutput();
		}

		this->InterpolationFunctions(pcoords, weights);

		constexpr double kOutsideTol = 1e-6; // VTK_OUTSIDE_CELL_TOLERANCE
		const double lowerlimit = 0.0 - kOutsideTol;
		const double upperlimit = 1.0 + kOutsideTol;
		if (pcoords[0] >= lowerlimit && pcoords[0] <= upperlimit && pcoords[1] >= lowerlimit &&
		    pcoords[1] <= upperlimit && pcoords[2] >= lowerlimit && pcoords[2] <= upperlimit) {
			// 单元内：VTK 直接把 closestPoint 取为 x、dist2 = 0（不做距离判定）
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		// The point lies outside the cell. Since a general hexahedron is trilinearly
		// warped, its 6 faces are (possibly non-planar) bilinear patches rather than
		// planes, so project x onto each of the 6 faces via Quad::EvaluatePosition
		// (which handles a non-planar quad correctly) and keep the nearest result.
		// 复用 m_Quad 作为临时面单元（与 VTK 用 this->Quad 暂存的用法一致）；
		// 面顶点序取自本类的 faces[]，与 vtkHexahedron::Faces 逐项相同。
		dist2 = DBL_MAX;
		Quad* face = m_Quad.get();
		for (int f = 0; f < 6; ++f) {
			const int* fv = faces[f];
			for (int i = 0; i < 4; ++i) {
				face->m_PointIds->SetId(i, this->m_PointIds->GetId(fv[i]));
				face->m_Points->SetPoint(i, this->m_Points->GetPoint(fv[i]));
			}
			int quadSubId = 0;
			double quadPcoords[3] = {0.0, 0.0, 0.0};
			double quadWeights[4] = {0.0, 0.0, 0.0, 0.0};
			Point cp;
			double d2 = 0.0;
			face->EvaluatePosition(x, cp, quadSubId, quadPcoords, d2, quadWeights);
			if (d2 < dist2) {
				dist2 = d2;
				closestPoint = cp;
			}
		}
		return 0;
	}

	/**
	 * The number of points of the cell
	 */
	static constexpr int NumberOfPoints = 8;

	/**
	 * The number of edges of the cell.
	 */
	static constexpr int NumberOfEdges = 12;

	/**
	 * The number of faces of the cell.
	 */
	static constexpr int NumberOfFaces = 6;

	/**
	 * The largest number of points of the element face.
	 */
	static constexpr int MaxFaceSize = 4;

	/**
	 * The maximum degree of a cell point.
	 */
	static constexpr int MaxValence = 3;

	/***************** The last digit of the array for all Max + 1 indicates the number *****************/

	// pointIds of a edge
	static constexpr int edges[NumberOfEdges][2] = {
	  { 0, 1 },
	  { 1, 2 },
	  { 3, 2 },
	  { 0, 3 },
	  { 4, 5 },
	  { 5, 6 },
	  { 7, 6 },
	  { 4, 7 },
	  { 0, 4 },
	  { 1, 5 },
	  { 2, 6 },
	  { 3, 7 },
	};

	// pointIds of a face
	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 4, 7, 3, 4 },
	  { 1, 2, 6, 5, 4 },
	  { 0, 1, 5, 4, 4 },
	  { 3, 7, 6, 2, 4 },
	  { 0, 3, 2, 1, 4 },
	  { 4, 5, 6, 7, 4 },
	};

	// edgeIds of a face
	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize + 1] = {
	  { 8, 7, 11, 3, 4 },
	  { 1, 10, 5, 9, 4 },
	  { 0, 9, 4, 8, 4 },
	  { 11, 6, 10, 2, 4 },
	  { 3, 2, 1, 0, 4 },
	  { 4, 5, 6, 7, 4 },
	};

	// The adjacent face number of the edge
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

	// Face adjacent face index number
	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize + 1] = {
		{ 4, 2, 5, 3, 4 },
		{ 4, 3, 5, 2, 4 },
		{ 4, 1, 5, 0, 4 },
		{ 0, 5, 1, 4, 4 },
		{ 0, 3, 1, 2, 4 },
		{ 2, 1, 0, 3, 4 },
	};

	// The sequence number of the adjacent edges of the vertex
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

	// The sequence number of the adjacent face of the vertex
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

	// PointIds of a neighborhood of a point
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
	//PointIds of cliped cell
	static constexpr int clipedCell[11][4] = {
		{ 0, 1, 3, 4 },
		{ 1, 2, 3, 6 },
		{ 3, 4, 6, 7 },
		{ 1, 4, 5, 6 },
		{ 1, 3, 4, 6 },
		{ 0, 4, 7, 3},
		{ 1, 2, 6, 5},
		{ 0, 1, 5, 4},
		{ 3, 7, 6, 2},
		{ 0, 3, 2, 1},
		{ 4, 5, 6, 7},
	};

	int GetEdgePointIds(const int edgeId, const igIndex*& pts) override {
		pts = edges[edgeId];
		return 2;
	}
	int GetFacePointIds(const int faceId, const igIndex*& pts) override {
		pts = faces[faceId];
		return faces[faceId][MaxFaceSize];
	}
	int GetFaceEdgeIds(const int faceId, const igIndex*& edgeIds) override {
		edgeIds = faceEdges[faceId];
		return faceEdges[faceId][MaxFaceSize];
	}
	int GetEdgeToNeighborFaces(const int edgeId, const igIndex*& pts) override {
		pts = edgeToNeighborFaces[edgeId];
		return 2;
	}
	int GetFaceToNeighborFaces(const int faceId, const igIndex*& faceIds) override {
		faceIds = faceToNeighborFaces[faceId];
		return faceToNeighborFaces[faceId][MaxFaceSize];
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
	Hexahedron()
	{
		// 六面体共 8 个点（原实现只分配 4 个：Reset()+AddPoint 的调用方不受影响，
		// 但直接 SetPoint(4..7) 会越界；此处修正为 8）
		this->m_Points->SetNumberOfPoints(8);
		this->m_PointIds->SetNumberOfIds(8);
		for (int i = 0; i < 8; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = Line::New();
		m_Quad = Quad::New();
	}
	~Hexahedron() override = default;

	Line::Pointer m_Line;
	Quad::Pointer m_Quad;
};


IGAME_NAMESPACE_END
#endif