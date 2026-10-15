#ifndef iGameTetra_h
#define iGameTetra_h

#include "iGameVolume.h"

IGAME_NAMESPACE_BEGIN
class Tetra : public Volume {
public:
	I_OBJECT(Tetra);
    static Pointer New() { return new Tetra; }
	IGenum GetCellType() const noexcept override { return IG_TETRA; }
    int GetCellSize() const noexcept override { return 4; }
    int GetNumberOfEdges() override { return 6; }
    int GetNumberOfFaces() override { return 4; }

    Cell* GetEdge(const int edgeId) override {
        const int* verts = edges[edgeId];

        m_Line->m_PointIds->SetId(0, m_PointIds->GetId(verts[0]));
        m_Line->m_PointIds->SetId(1, m_PointIds->GetId(verts[1]));

        m_Line->m_Points->SetPoint(0, m_Points->GetPoint(verts[0]));
        m_Line->m_Points->SetPoint(1, m_Points->GetPoint(verts[1]));

        return m_Line.get();
    }
    Cell* GetFace(const int faceId) override {
        const int* verts = faces[faceId];
        const int* edges = faceEdges[faceId];
        for (int i = 0; i < 3; ++i) {
            m_Triangle->m_PointIds->SetId(i, m_PointIds->GetId(verts[i]));
            m_Triangle->m_Points->SetPoint(i, m_Points->GetPoint(verts[i]));
        }
        return m_Triangle.get();
    }
    std::vector<iGame::Cell::Pointer> clipCelltoTetra() override {
        std::vector<iGame::Cell::Pointer> result;
        result.emplace_back(this);
        return result;
	}

	/* ---------------- VTK 风格插值接口（对照 vtkTetra） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/** vtkTetra::GetParametricCenter（vtkTetra.h 内联实现：重心参数中心） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = pcoords[2] = 0.25;
	}

	/** vtkTetra::InterpolationFunctions（体积坐标即重心坐标） */
	void InterpolationFunctions(const double pcoords[3], double* sf) const override {
		sf[0] = 1.0 - pcoords[0] - pcoords[1] - pcoords[2];
		sf[1] = pcoords[0];
		sf[2] = pcoords[1];
		sf[3] = pcoords[2];
	}

	/** vtkTetra::InterpolationDerivs（布局：∂N_i/∂r ×4, ∂N_i/∂s ×4, ∂N_i/∂t ×4） */
	void InterpolationDerivs(const double*, double* derivs) const override {
		// r-derivatives
		derivs[0] = -1.0;
		derivs[1] = 1.0;
		derivs[2] = 0.0;
		derivs[3] = 0.0;

		// s-derivatives
		derivs[4] = -1.0;
		derivs[5] = 0.0;
		derivs[6] = 1.0;
		derivs[7] = 0.0;

		// t-derivatives
		derivs[8] = -1.0;
		derivs[9] = 0.0;
		derivs[10] = 0.0;
		derivs[11] = 1.0;
	}

	/** vtkMath::Determinant3x3 的复刻：det = a · (b × c) */
	static double Determinant3x3(const double a[3], const double b[3], const double c[3]) {
		return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
		       a[2] * (b[0] * c[1] - b[1] * c[0]);
	}

	/** vtkMath::Determinant2x2 的复刻：a0·b1 - a1·b0（面投影用的伴随度量） */
	static double Determinant2x2(double a0, double a1, double b0, double b1) {
		return a0 * b1 - b0 * a1;
	}

	/**
	 * vtkTetra::EvaluatePosition 的逐行复刻（闭式重心坐标 + 顶点/边/面最近点分支）：
	 *   - c1 = P1-P0、c2 = P2-P0、c3 = P3-P0，det 为 6 倍有符号体积；
	 *   - det == 0（退化）时 VTK 直接返回 -1 且不写任何输出参数：这里按约定补上中性
	 *     权重 weights[0]=1（其余 0）以及与之自洽的 pcoords=(0,0,0)、closestPoint=P0、
	 *     dist2 = |x-P0|²（pcoords=(0,0,0) 处的形函数恰为 (1,0,0,0)）；
	 *   - 否则 pcoords 即重心坐标（允许超出 [0,1]），weights 由形函数给出；
	 *   - inside 判定沿用 VTK 的 eps = 100·DBL_EPSILON（**不是** 1e-3 参数域容差）；
	 *   - 返回 1 时 closestPoint = x、dist2 = 0（VTK 语义），返回 0 时用度量张量在
	 *     顶点 / 6 条边 / 4 个面上取精确最近点。
	 * 顶点序与 VTK_TETRA 完全一致（edges/faces 表逐项相同），无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;
		pcoords[0] = pcoords[1] = pcoords[2] = 0.0;

		double pt[4][3];
		for (int i = 0; i < 4; ++i) {
			const Point& P = this->GetPoint(i);
			pt[i][0] = static_cast<double>(P[0]);
			pt[i][1] = static_cast<double>(P[1]);
			pt[i][2] = static_cast<double>(P[2]);
		}
		const double* pt0 = pt[0];
		const double* pt1 = pt[1];
		const double* pt2 = pt[2];
		const double* pt3 = pt[3];
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		const auto toPoint = [](const double c[3]) {
			return Point(static_cast<float>(c[0]), static_cast<float>(c[1]),
			             static_cast<float>(c[2]));
		};
		const auto squaredDistance = [&xd](const double c[3]) {
			const double dx = c[0] - xd[0];
			const double dy = c[1] - xd[1];
			const double dz = c[2] - xd[2];
			return dx * dx + dy * dy + dz * dz;
		};

		double c1[3], c2[3], c3[3], p[3];
		for (int i = 0; i < 3; ++i) {
			c1[i] = pt1[i] - pt0[i];
			c2[i] = pt2[i] - pt0[i];
			c3[i] = pt3[i] - pt0[i];
			p[i] = xd[i] - pt0[i];
		}

		const double det = Determinant3x3(c1, c2, c3);
		if (det == 0.0) {
			// 退化四面体：VTK 返回 -1 且不填输出；此处给出中性权重与自洽的距离信息
			weights[0] = 1.0;
			weights[1] = weights[2] = weights[3] = 0.0;
			closestPoint = toPoint(pt0);
			dist2 = squaredDistance(pt0);
			return -1;
		}

		pcoords[0] = Determinant3x3(p, c2, c3) / det;
		pcoords[1] = Determinant3x3(c1, p, c3) / det;
		pcoords[2] = Determinant3x3(c1, c2, p) / det;

		weights[0] = 1.0 - pcoords[0] - pcoords[1] - pcoords[2];
		weights[1] = pcoords[0];
		weights[2] = pcoords[1];
		weights[3] = pcoords[2];

		static constexpr double eps = 100.0 * DBL_EPSILON;

		const bool inside =
		    (weights[0] > -eps && weights[1] > -eps && weights[2] > -eps && weights[3] > -eps);

		if (inside) {
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		// 点不在单元内：以下为 VTK 原文的顶点 → 边 → 面分支链（顺序不可改动，
		// 否则最近点的选取会与 VTK 不一致）
		// Metric of the edge vectors with origin at pt0
		const double g11 = c1[0] * c1[0] + c1[1] * c1[1] + c1[2] * c1[2];
		const double g12 = c1[0] * c2[0] + c1[1] * c2[1] + c1[2] * c2[2];
		const double g13 = c1[0] * c3[0] + c1[1] * c3[1] + c1[2] * c3[2];
		const double g22 = c2[0] * c2[0] + c2[1] * c2[1] + c2[2] * c2[2];
		const double g23 = c2[0] * c3[0] + c2[1] * c3[1] + c2[2] * c3[2];
		const double g33 = c3[0] * c3[0] + c3[1] * c3[1] + c3[2] * c3[2];
		// Metric of the edge vectors with origin at different vertices
		// g_{i,jk} = (c_j - c_i) · (c_k - c_i)
		const double g1_22 = g22 + g11 - 2 * g12;
		const double g1_33 = g33 + g11 - 2 * g13;
		const double g1_23 = g23 + g11 - g12 - g13;
		const double g2_33 = g33 + g22 - 2 * g23;
		const double g2_13 = g13 + g22 - g12 - g23;
		const double g1_02 = g11 - g12;
		const double g1_03 = g11 - g13;
		const double g2_03 = g22 - g23;
		// Project point to each of the edge vectors with origin at vertex pt0
		const double p1 = p[0] * c1[0] + p[1] * c1[1] + p[2] * c1[2];
		const double p2 = p[0] * c2[0] + p[1] * c2[1] + p[2] * c2[2];
		const double p3 = p[0] * c3[0] + p[1] * c3[1] + p[2] * c3[2];
		// Project point to each of the edge vectors with origin at different vertices
		// p_{i,j} = (p - c_i) · (c_j - c_i)
		const double p1_0 = g11 - p1;
		const double p2_0 = g22 - p2;
		const double p3_0 = g33 - p3;
		const double p1_2 = p2 - p1 + g1_02;
		const double p1_3 = p3 - p1 + g1_03;
		const double p2_3 = p3 - p2 + g2_03;
		const double p2_1 = g1_22 - p1_2;
		const double p3_1 = g1_33 - p1_3;
		const double p3_2 = g2_33 - p2_3;
		// Multiple of the signed distance of the point to the plane
		// perpendicular to each of the faces and containing each of their edges:
		// q_{ij,k} = (((c_k - c_i) ∧ (c_j - c_i)) · (c_j - c_i)) · (p - c_i)
		//          = g_{i,jj} p_{i,k} - g_{i,jk} p_{i,j}
		// Although not evident in the expression, it satisfies the simmetry
		// q_{ij,k} = q_{ji,k}
		const double q01_2 = g11 * p2 - g12 * p1;
		const double q01_3 = g11 * p3 - g13 * p1;
		const double q02_1 = g22 * p1 - g12 * p2;
		const double q02_3 = g22 * p3 - g23 * p2;
		const double q03_1 = g33 * p1 - g13 * p3;
		const double q03_2 = g33 * p2 - g23 * p3;
		const double q12_3 = g1_22 * p1_3 - g1_23 * p1_2;
		const double q13_2 = g1_33 * p1_2 - g1_23 * p1_3;
		const double q23_1 = g2_33 * p2_1 - g2_13 * p2_3;
		const double q12_0 = g1_22 * p1_0 - g1_02 * p1_2;
		const double q13_0 = g1_33 * p1_0 - g1_03 * p1_3;
		const double q23_0 = g2_33 * p2_0 - g2_03 * p2_3;

		double cp[3];
		for (int i = 0; i < 3; ++i) { cp[i] = 0.0; }
		if (p1 <= 0 && p2 <= 0 && p3 <= 0) { // Vertex 0
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i]; }
		} else if (p1_2 <= 0 && p1_3 <= 0 && p1_0 <= 0) { // Vertex 1
			for (int i = 0; i < 3; ++i) { cp[i] = pt1[i]; }
		} else if (p2_3 <= 0 && p2_0 <= 0 && p2_1 <= 0) { // Vertex 2
			for (int i = 0; i < 3; ++i) { cp[i] = pt2[i]; }
		} else if (p3_0 <= 0 && p3_1 <= 0 && p3_2 <= 0) { // Vertex 3
			for (int i = 0; i < 3; ++i) { cp[i] = pt3[i]; }
		} else if (q01_2 <= 0 && q01_3 <= 0 && p1 >= 0 && p1_0 >= 0) { // Edge 0-1
			const double rho1 = p1 / g11;
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i] + rho1 * c1[i]; }
		} else if (q02_1 <= 0 && q02_3 <= 0 && p2 >= 0 && p2_0 >= 0) { // Edge 0-2
			const double rho2 = p2 / g22;
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i] + rho2 * c2[i]; }
		} else if (q03_1 <= 0 && q03_2 <= 0 && p3 >= 0 && p3_0 >= 0) { // Edge 0-3
			const double rho3 = p3 / g33;
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i] + rho3 * c3[i]; }
		} else if (q12_0 <= 0 && q12_3 <= 0 && p1_2 >= 0 && p2_1 >= 0) { // Edge 1-2
			const double rho2 = p1_2 / g1_22;
			const double rho1 = 1 - rho2;
			for (int i = 0; i < 3; ++i) { cp[i] = rho1 * pt1[i] + rho2 * pt2[i]; }
		} else if (q13_0 <= 0 && q13_2 <= 0 && p1_3 >= 0 && p3_1 >= 0) { // Edge 1-3
			const double rho3 = p1_3 / g1_33;
			const double rho1 = 1 - rho3;
			for (int i = 0; i < 3; ++i) { cp[i] = rho1 * pt1[i] + rho3 * pt3[i]; }
		} else if (q23_0 <= 0 && q23_1 <= 0 && p2_3 >= 0 && p3_2 >= 0) { // Edge 2-3
			const double rho3 = p2_3 / g2_33;
			const double rho2 = 1 - rho3;
			for (int i = 0; i < 3; ++i) { cp[i] = rho2 * pt2[i] + rho3 * pt3[i]; }
		} else if (weights[1] <= 0 && q02_3 >= 0 && q23_0 >= 0 && q03_2 >= 0) { // Face 0-2-3
			const double adjg11 = Determinant2x2(g22, g23, g23, g33);
			const double rho2 = q03_2 / adjg11;
			const double rho3 = q02_3 / adjg11;
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i] + rho2 * c2[i] + rho3 * c3[i]; }
		} else if (weights[2] <= 0 && q01_3 >= 0 && q13_0 >= 0) { // Face 0-1-3
			const double adjg22 = Determinant2x2(g11, g13, g13, g33);
			const double rho1 = q03_1 / adjg22;
			const double rho3 = q01_3 / adjg22;
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i] + rho1 * c1[i] + rho3 * c3[i]; }
		} else if (weights[3] <= 0 && q12_0 >= 0) { // Face 0-1-2
			const double adjg33 = Determinant2x2(g11, g12, g12, g22);
			const double rho1 = q02_1 / adjg33;
			const double rho2 = q01_2 / adjg33;
			for (int i = 0; i < 3; ++i) { cp[i] = pt0[i] + rho1 * c1[i] + rho2 * c2[i]; }
		} else { // Face 1-2-3
			const double adjg1_00 = Determinant2x2(g1_22, g1_23, g1_23, g1_33);
			const double rho2 = q13_2 / adjg1_00;
			const double rho3 = q12_3 / adjg1_00;
			const double rho1 = 1 - rho2 - rho3;
			for (int i = 0; i < 3; ++i) { cp[i] = rho1 * pt1[i] + rho2 * pt2[i] + rho3 * pt3[i]; }
		}
		closestPoint = toPoint(cp);
		dist2 = squaredDistance(cp);
		return 0;
	}

	/**
	 * The number of points of the cell
	 */
	static constexpr int NumberOfPoints = 4;

	/**
	 * The number of edges of the cell.
	 */
	static constexpr int NumberOfEdges = 6;

	/**
	 * The number of faces of the cell.
	 */
	static constexpr int NumberOfFaces = 4;

	/**
	 * The largest number of points of the element face.
	 */
	static constexpr int MaxFaceSize = 3;

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
	  { 0, 3 },
	  { 1, 3 },
	  { 2, 3 },
	};

	// pointIds of a face
	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 1, 3, 3 },
	  { 1, 2, 3, 3 },
	  { 2, 0, 3, 3 },
	  { 0, 2, 1, 3 },
	}; 

	// edgeIds of a face
	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 4, 3, 3 },
	  { 1, 5, 4, 3 },
	  { 2, 3, 5, 3 },
	  { 2, 1, 0, 3 },
	};

	// The adjacent face number of the edge
	static constexpr int edgeToNeighborFaces[NumberOfEdges][2] = {
	  { 0, 3 },
	  { 1, 3 },
	  { 2, 3 },
	  { 0, 2 },
	  { 0, 1 },
	  { 1, 2 },
	};

	// Face adjacent face index number
	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 3, 1, 2, 3 },
	  { 3, 2, 0, 3 },
	  { 3, 0, 1, 3 },
	  { 2, 1, 0, 3 },
	};

	// The sequence number of the adjacent edges of the vertex
	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
	  { 0, 3, 2, 3 },
	  { 0, 1, 4, 3 },
	  { 1, 2, 5, 3 },
	  { 3, 4, 5, 3 },
	};

	// The sequence number of the adjacent face of the vertex
	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
	  { 0, 2, 3, 3 },
	  { 3, 1, 0, 3 },
	  { 3, 2, 1, 3 },
	  { 0, 1, 2, 3 },
	};

	// PointIds of a neighborhood of a point
	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
	  { 1, 3, 2, 3 },
	  { 0, 2, 3, 3 },
	  { 1, 0, 3, 3 },
	  { 0, 1, 2, 3 },
	};

	int GetEdgePointIds(const int edgeId, const igIndex*& ptIds) override {
        ptIds = edges[edgeId];
        return 2;
	}
    int GetFacePointIds(const int faceId, const igIndex*& ptIds) override {
        ptIds = faces[faceId];
		return faces[faceId][MaxFaceSize];
	}
	int GetFaceEdgeIds(const int faceId, const igIndex*& edgeIds) override {
		edgeIds = faceEdges[faceId];
		return faceEdges[faceId][MaxFaceSize];
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
		return faceToNeighborFaces[faceId][MaxFaceSize];
	}

	static int EdgePointIds(const int edgeId, const igIndex*& ptIds) {
		ptIds = edges[edgeId];
		return 2;
	}
	static int FacePointIds(const int faceId, const igIndex*& ptIds) {
		ptIds = faces[faceId];
		return faces[faceId][MaxFaceSize];
	}
	static int FaceEdgeIds(const int faceId, const igIndex*& edgeIds) {
		edgeIds = faceEdges[faceId];
		return faceEdges[faceId][MaxFaceSize];
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
		return faceToNeighborFaces[faceId][MaxFaceSize];
	}

private:
	Tetra()
	{
		this->m_Points->SetNumberOfPoints(4);
		this->m_PointIds->SetNumberOfIds(4);
		for (int i = 0; i < 4; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = Line::New();
        m_Triangle = Triangle::New();
	}
	~Tetra() override = default;

	Line::Pointer m_Line;
    Triangle::Pointer m_Triangle;
};


IGAME_NAMESPACE_END
#endif