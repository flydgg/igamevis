#ifndef iGamePrism_h
#define iGamePrism_h

#include "iGameVolume.h"
#include <iGameTetra.h>
IGAME_NAMESPACE_BEGIN
class Prism : public Volume {
public:
	I_OBJECT(Prism);
	static Pointer New() { return new Prism; }

	IGenum GetCellType() const noexcept override { return IG_PRISM; }
    int GetCellSize() const noexcept override { return 6; }
    int GetNumberOfEdges() override { return 9; }
    int GetNumberOfFaces() override { return 5; }

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
        Face* face;
        if (faces[faceId][MaxFaceSize] == 3) {
            face = m_Triangle.get();
        } else {
            face = m_Quad.get();
        }
        for (int i = 0; i < faces[faceId][MaxFaceSize]; ++i) {
            face->m_PointIds->SetId(i,  this->m_PointIds->GetId(verts[i]));
            face->m_Points->SetPoint(i, this->m_Points->GetPoint(verts[i]));
        }

        return face;
    }
    std::vector<iGame::Cell::Pointer> clipCelltoTetra() override {
        std::vector<iGame::Cell::Pointer> result;
        for (int i = 0; i < 6; i++) {
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
	/* ---------------- VTK 风格插值接口（对照 vtkWedge，6 节点三棱柱） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/** vtkWedge::GetParametricCenter（vtkWedge.h 内联实现：底面重心参数中心） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = 0.333333;
		pcoords[2] = 0.5;
	}

	/** vtkWedge::InterpolationFunctions（三角形线性 × 轴向线性） */
	void InterpolationFunctions(const double pcoords[3], double* sf) const override {
		sf[0] = (1.0 - pcoords[0] - pcoords[1]) * (1.0 - pcoords[2]);
		sf[1] = pcoords[0] * (1.0 - pcoords[2]);
		sf[2] = pcoords[1] * (1.0 - pcoords[2]);
		sf[3] = (1.0 - pcoords[0] - pcoords[1]) * pcoords[2];
		sf[4] = pcoords[0] * pcoords[2];
		sf[5] = pcoords[1] * pcoords[2];
	}

	/** vtkWedge::InterpolationDerivs（布局：∂N_i/∂r ×6, ∂N_i/∂s ×6, ∂N_i/∂t ×6） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		// r-derivatives
		derivs[0] = -1.0 + pcoords[2];
		derivs[1] = 1.0 - pcoords[2];
		derivs[2] = 0.0;
		derivs[3] = -pcoords[2];
		derivs[4] = pcoords[2];
		derivs[5] = 0.0;

		// s-derivatives
		derivs[6] = -1.0 + pcoords[2];
		derivs[7] = 0.0;
		derivs[8] = 1.0 - pcoords[2];
		derivs[9] = -pcoords[2];
		derivs[10] = 0.0;
		derivs[11] = pcoords[2];

		// t-derivatives
		derivs[12] = -1.0 + pcoords[0] + pcoords[1];
		derivs[13] = -pcoords[0];
		derivs[14] = -pcoords[1];
		derivs[15] = 1.0 - pcoords[0] - pcoords[1];
		derivs[16] = pcoords[0];
		derivs[17] = pcoords[1];
	}

	/** vtkMath::Determinant3x3 的复刻：det = a · (b × c) */
	static double Determinant3x3(const double a[3], const double b[3], const double c[3]) {
		return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
		       a[2] * (b[0] * c[1] - b[1] * c[0]);
	}

	/**
	 * vtkWedge::EvaluatePosition 的逐行复刻（形函数 + 牛顿迭代）：
	 *   - 初值取参数中心 (0.5, 0.5, 0.5)；
	 *   - 迭代失败（行列式小于容差 / 参数发散 / 20 次内不收敛）时 VTK 返回 -1，且不写
	 *     输出参数：这里按约定补上中性权重 weights[0]=1（其余 0）以及与之自洽的
	 *     pcoords=(0,0,0)、closestPoint=P0、dist2 = |x-P0|²；
	 *   - 收敛后按 VTK 原文的 [-0.001, 1.001]（且 r+s ≤ 1.001）判定参数域；域内返回 1
	 *     并把 closestPoint 取为 x、dist2 置 0（与点到单元的距离无关）；
	 *   - 域外返回 0：把 pcoords 各分量夹到 [0,1] 后经 EvaluateLocation 求近似最近点
	 *     （VTK 原文注释即声明这是近似，不保证 r+s ≤ 1 的夹紧）。
	 * 顶点序与 VTK_WEDGE 完全一致（棱表逐项相同），无需索引置换；仅本类 faces[] 的
	 * 面绕向与 vtkWedge::Faces 相反，而 EvaluatePosition 不使用面表，故无影响。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[6][3];
		for (int i = 0; i < 6; ++i) {
			const Point& P = this->GetPoint(i);
			pts[i][0] = static_cast<double>(P[0]);
			pts[i][1] = static_cast<double>(P[1]);
			pts[i][2] = static_cast<double>(P[2]);
		}
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// 退化/失败分支的统一输出：pcoords=(0,0,0) 处的形函数恰为 (1,0,...,0)，
		// 且对应顶点 0，因此中性权重、pcoords、closestPoint = P0 三者自洽
		const auto setFailedOutput = [&]() {
			pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
			for (int i = 0; i < 6; ++i) { weights[i] = 0.0; }
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
		// （VTK 用 9 条棱中最长者；这里的棱表 edges[] 与 vtkWedge 的 Edges 逐项相同）
		double longestEdge = 0.0;
		for (int i = 0; i < 9; ++i) {
			const double* a = pts[edges[i][0]];
			const double* b = pts[edges[i][1]];
			const double dx = b[0] - a[0];
			const double dy = b[1] - a[1];
			const double dz = b[2] - a[2];
			longestEdge = std::max(longestEdge, dx * dx + dy * dy + dz * dz);
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
			double derivs[18];
			// calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			// calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 6; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 6];
					tcol[j] += coord * derivs[i + 12];
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

		if (pcoords[0] >= -0.001 && pcoords[0] <= 1.001 && pcoords[1] >= -0.001 &&
		    pcoords[1] <= 1.001 && pcoords[2] >= -0.001 && pcoords[2] <= 1.001 &&
		    pcoords[0] + pcoords[1] <= 1.001) {
			// 单元内：VTK 直接把 closestPoint 取为 x、dist2 = 0
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		// 单元外：VTK 原文注释即声明这是近似（不保证 r+s ≤ 1 的夹紧），照搬其行为
		double pc[3];
		for (int i = 0; i < 3; ++i) { // only approximate, not really true for warped prism
			if (pcoords[i] < 0.0) {
				pc[i] = 0.0;
			} else if (pcoords[i] > 1.0) {
				pc[i] = 1.0;
			} else {
				pc[i] = pcoords[i];
			}
		}
		double w[6];
		this->EvaluateLocation(subId, pc, closestPoint, w);
		const double dx = static_cast<double>(closestPoint[0]) - xd[0];
		const double dy = static_cast<double>(closestPoint[1]) - xd[1];
		const double dz = static_cast<double>(closestPoint[2]) - xd[2];
		dist2 = dx * dx + dy * dy + dz * dz;
		return 0;
	}

	/**
	 * The number of points of the cell
	 */
	static constexpr int NumberOfPoints = 6;

	/**
	 * The number of edges of the cell.
	 */
	static constexpr int NumberOfEdges = 9;

	/**
	 * The number of faces of the cell.
	 */
	static constexpr int NumberOfFaces = 5;

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
	  { 2, 0 },
	  { 3, 4 },
	  { 4, 5 },
	  { 5, 3 },
	  { 0, 3 },
	  { 1, 4 },
	  { 2, 5 },
	};

	// pointIds of a face
	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 1, 2, -1, 3 },
	  { 3, 5, 4, -1, 3 },
	  { 0, 3, 4, 1,  4 },
	  { 1, 4, 5, 2,  4 },
	  { 2, 5, 3, 0,  4 },
	};

	// edgeIds of a face
	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 1, 2, -1, 3},
	  { 5, 4, 3, -1, 3 },
	  { 6, 3, 7, 0, 4 },
	  { 7, 4, 8, 1, 4 },
	  { 8, 5, 6, 2, 4 },
	};

	// The adjacent face number of the edge
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

	// Face adjacent face index number
	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 4, 3, 2, -1, 3 },
	  { 2, 3, 4, -1, 3 },
	  { 0, 3, 1, 4 , 4 },
	  { 0, 4, 1, 2 , 4 },
	  { 0, 2, 1, 3 , 4 },
	};

	// The sequence number of the adjacent edges of the vertex
	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
	  { 0, 6, 2, 3 },
	  { 0, 1, 7, 3 },
	  { 1, 2, 8, 3 },
	  { 3, 5, 6, 3 },
	  { 3, 7, 4, 3 },
	  { 4, 8, 5, 3 },
	};

	// The sequence number of the adjacent face of the vertex
	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
	  { 2, 4, 0, 3 },
	  { 0, 3, 2, 3 },
	  { 0, 4, 3, 3 },
	  { 1, 4, 2, 3 },
	  { 2, 3, 1, 3 },
	  { 3, 4, 1, 3 },
	};

	// PointIds of a neighborhood of a point
	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
	  { 1, 3, 2, 3 },
	  { 0, 2, 4, 3 },
	  { 1, 0, 5, 3 },
	  { 4, 5, 0, 3 },
	  { 3, 1, 5, 3 },
	  { 4, 2, 3, 3 },
	};	
	static constexpr int clipedCell[6][4] = {
	  { 0, 1, 2, 5 },
	  { 0, 3, 4, 5 },
	  { 0, 1, 4, 5 },
	  { 0, 3, 4, 1},
	  { 1, 4, 5, 2},
	  { 2, 5, 3, 0},
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
	Prism()
	{
		this->m_Points->SetNumberOfPoints(6);
		this->m_PointIds->SetNumberOfIds(6);
		for (int i = 0; i < 6; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = Line::New();
		m_Triangle = Triangle::New();
		m_Quad = Quad::New();
	}
	~Prism() override = default;

	Line::Pointer m_Line;
    Triangle::Pointer m_Triangle;
    Quad::Pointer m_Quad;
};


IGAME_NAMESPACE_END
#endif