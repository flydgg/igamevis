#ifndef iGamePyramid_h
#define iGamePyramid_h

#include "iGameVolume.h"
#include <iGameTetra.h>
IGAME_NAMESPACE_BEGIN
class Pyramid : public Volume {
public:
	I_OBJECT(Pyramid);
	static Pointer New() { return new Pyramid; }

	IGenum GetCellType() const noexcept override { return IG_PYRAMID; }
    int GetCellSize() const noexcept override { return 5; }
    int GetNumberOfEdges() override { return 8; }
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
        for (int i = 0; i < 3; i++) {
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
	/**
	 * The number of points of the cell
	 */
	static constexpr int NumberOfPoints = 5;

	/**
	 * The number of edges of the cell.
	 */
	static constexpr int NumberOfEdges = 8;

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
	static constexpr int MaxValence = 4;

	/***************** The last digit of the array for all Max + 1 indicates the number *****************/

	// pointIds of a edge
	static constexpr int edges[NumberOfEdges][2] = {
	  { 0, 1 },
	  { 1, 2 },
	  { 2, 3 },
	  { 3, 0 },
	  { 0, 4 },
	  { 1, 4 },
	  { 2, 4 },
	  { 3, 4 },
	};

	// pointIds of a face
	static constexpr int faces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 3, 2,  1, 4 },
	  { 0, 1, 4, -1, 3 },
	  { 1, 2, 4, -1, 3 },
	  { 2, 3, 4, -1, 3 },
	  { 3, 0, 4, -1, 3 },
	};

	// edgeIds of a face
	// TODO: wait to modify
	static constexpr int faceEdges[NumberOfFaces][MaxFaceSize + 1] = {
	  { 0, 4, 7, 3, 4 },
	  { 1, 2, 6, 5, 4 },
	  { 0, 1, 5, 4, 4 },
	  { 3, 7, 6, 2, 4 },
	  { 0, 3, 2, 1, 4 },
	};

	// The adjacent face number of the edge
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

	// Face adjacent face index number
	static constexpr int faceToNeighborFaces[NumberOfFaces][MaxFaceSize + 1] = {
	  { 4, 3, 2, 1,  4 },
	  { 0, 2, 4, -1, 3 },
	  { 0, 3, 1, -1, 3 },
	  { 0, 4, 2, -1, 3 },
	  { 0, 1, 3, -1, 3 },
	};

	// The sequence number of the adjacent edges of the vertex
	static constexpr int pointToNeighborEdges[NumberOfPoints][MaxValence + 1] = {
	  { 0, 4, 3, -1, 3 },
	  { 0, 1, 5, -1, 3 },
	  { 1, 2, 6, -1, 3 },
	  { 2, 3, 7, -1, 3 },
	  { 4, 5, 6, 7 , 4 },
	};

	// The sequence number of the adjacent face of the vertex
	static constexpr int pointToNeighborFaces[NumberOfPoints][MaxValence + 1] = {
	  { 1, 4, 0, -1, 3 },
	  { 0, 2, 1, -1, 3 },
	  { 0, 3, 2, -1, 3 },
	  { 0, 4, 3, -1, 3 },
	  { 1, 2, 3, 4 , 4 },
	};

	// PointIds of a neighborhood of a point
	static constexpr int pointToOneRingPoints[NumberOfPoints][MaxValence + 1] = {
	  { 1, 4, 3, -1, 3 },
	  { 0, 2, 4, -1, 3 },
	  { 1, 3, 4, -1, 3 },
	  { 2, 0, 4, -1, 3 },
	  { 0, 1, 2, 3 , 4 },
	};
	static constexpr int clipedCell[3][4] = {
	  { 0, 1, 3,4 },
	  { 1, 2, 3,4 },
	  { 0, 3, 2, 1 },

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

	/* ---------------- VTK 风格插值接口（对照 vtkPyramid） ---------------- */

	int GetParametricDimension() const override { return 3; }

	/** vtkPyramid::GetParametricCenter（vtkPyramid.h 内联实现，注意不是立方体中心） */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = 0.4;
		pcoords[2] = 0.2;
	}

	/** vtkPyramid::InterpolationFunctions（底面 0-3 的 r-s 双线性 × (1-t)，塔尖为 t） */
	void InterpolationFunctions(const double pcoords[3], double* sf) const override {
		const double rm = 1.0 - pcoords[0];
		const double sm = 1.0 - pcoords[1];
		const double tm = 1.0 - pcoords[2];

		sf[0] = rm * sm * tm;
		sf[1] = pcoords[0] * sm * tm;
		sf[2] = pcoords[0] * pcoords[1] * tm;
		sf[3] = rm * pcoords[1] * tm;
		sf[4] = pcoords[2];
	}

	/** vtkPyramid::InterpolationDerivs（布局：∂N_i/∂r ×5, ∂N_i/∂s ×5, ∂N_i/∂t ×5） */
	void InterpolationDerivs(const double pcoords[3], double* derivs) const override {
		const double rm = 1.0 - pcoords[0];
		const double sm = 1.0 - pcoords[1];
		const double tm = 1.0 - pcoords[2];

		// r-derivatives
		derivs[0] = -sm * tm;
		derivs[1] = sm * tm;
		derivs[2] = pcoords[1] * tm;
		derivs[3] = -pcoords[1] * tm;
		derivs[4] = 0.0;

		// s-derivatives
		derivs[5] = -rm * tm;
		derivs[6] = -pcoords[0] * tm;
		derivs[7] = pcoords[0] * tm;
		derivs[8] = rm * tm;
		derivs[9] = 0.0;

		// t-derivatives
		derivs[10] = -rm * sm;
		derivs[11] = -pcoords[0] * sm;
		derivs[12] = -pcoords[0] * pcoords[1];
		derivs[13] = -rm * pcoords[1];
		derivs[14] = 1.0;
	}

	/** vtkMath::Determinant3x3 的复刻：det = a · (b × c) */
	static double Determinant3x3(const double a[3], const double b[3], const double c[3]) {
		return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
		       a[2] * (b[0] * c[1] - b[1] * c[0]);
	}

	/** vtkMath::Distance2BetweenPoints 的复刻：返回**平方**距离 */
	static double Distance2BetweenPoints(const double a[3], const double b[3]) {
		const double dx = a[0] - b[0];
		const double dy = a[1] - b[1];
		const double dz = a[2] - b[2];
		return dx * dx + dy * dy + dz * dz;
	}

	/**
	 * vtkPyramid::EvaluatePosition 的逐行复刻：
	 *   - 先对塔尖（顶点 4）做特判（VTK 注释：Newton 在塔尖附近不可靠），命中则
	 *     pcoords=(0,0,1)、weights 为形函数、closestPoint=x、dist2=0，返回 1；
	 *   - 用 8 条棱中最长的棱估行列式容差 determinantTolerance；
	 *   - 初值固定为 (0.3333333, 0.3333333, 0.3333333)（**不是** GetParametricCenter 的
	 *     0.4/0.2）；迭代参数 VTK_MAX_ITERATIONS=20、VTK_CONVERGED=1e-4、VTK_DIVERGED=1e6；
	 *   - 失败（行列式过小 / 参数发散 / 20 次内不收敛）时 VTK 返回 -1 且不写输出参数：
	 *     这里按约定补上中性权重 weights[0]=1（其余 0）以及与之自洽的 pcoords=(0,0,0)、
	 *     closestPoint=P0、dist2 = |x-P0|²；
	 *   - 收敛后按三个分量 ∈ [-0.001, 1+0.001] 判定参数域（VTK 原文注释：XY 参数平面随 Z
	 *     收缩，但 X、Y 始终在 [0,1] 内，故盒式判定成立）；域内返回 1，并按 VTK 原文把
	 *     closestPoint 直接取为 x、dist2 = 0（与点到单元的真实距离无关）；
	 *   - 域外返回 0：把 pcoords 逐分量夹到 [0,1] 后由形函数求近似最近点（VTK 原文注释：
	 *     warped 情况下只是近似）。
	 * 顶点序与 VTK_PYRAMID 完全一致（底面 0-3 逆时针、塔尖 4；edges/faces 表逐项相同，
	 * 仅表尾长度标记的写法不同），无需索引置换。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                     double pcoords[3], double& dist2, double* weights) const override {
		subId = 0;

		double pts[5][3];
		for (int i = 0; i < 5; ++i) {
			const Point& P = this->GetPoint(i);
			pts[i][0] = static_cast<double>(P[0]);
			pts[i][1] = static_cast<double>(P[1]);
			pts[i][2] = static_cast<double>(P[2]);
		}
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// 失败分支的统一输出：pcoords=(0,0,0) 处的形函数恰好是 (1,0,0,0,0) 且对应顶点 0，
		// 因此中性权重、pcoords、closestPoint=P0、dist2=|x-P0|² 四者自洽
		const auto setFailedOutput = [&]() {
			pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
			for (int i = 0; i < 5; ++i) { weights[i] = 0.0; }
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
		double baseMidpoint[3];
		baseMidpoint[0] = pts[0][0];
		baseMidpoint[1] = pts[0][1];
		baseMidpoint[2] = pts[0][2];
		for (int i = 1; i < 4; ++i) {
			for (int j = 0; j < 3; ++j) { baseMidpoint[j] += pts[i][j]; }
		}
		for (int i = 0; i < 3; ++i) { baseMidpoint[i] /= 4.0; }

		const double length2 = Distance2BetweenPoints(apexPoint, baseMidpoint);
		// we use .001 as the relative tolerance here since that is the same
		// that is used for the interior cell check below but we need to
		// square it here because we're looking at dist2^2.
		if (dist2 == 0.0 || (length2 != 0.0 && dist2 / length2 < 1.0e-6)) {
			pcoords[0] = pcoords[1] = 0.0;
			pcoords[2] = 1.0;
			this->InterpolationFunctions(pcoords, weights);
			// VTK 原文中 closestPoint/dist2 的赋值位于 `if (closestPoint)` 内，
			// 我们的出参是引用（永不为空），因此等价于无条件执行
			closestPoint = x;
			dist2 = 0.0;
			return 1;
		}

		double derivs[15];

		// compute a bound on the volume to get a scale for an acceptable determinant
		double longestEdge = 0.0;
		for (int i = 0; i < 8; ++i) {
			const double* pt0 = pts[edges[i][0]];
			const double* pt1 = pts[edges[i][1]];
			longestEdge = std::max(longestEdge, Distance2BetweenPoints(pt0, pt1));
		}
		// longestEdge value is already squared
		const double volumeBound = longestEdge * std::sqrt(longestEdge);
		const double determinantTolerance =
		    1e-20 < 0.00001 * volumeBound ? 1e-20 : 0.00001 * volumeBound;

		//  set initial position for Newton's method
		double params[3] = {0.3333333, 0.3333333, 0.3333333};
		pcoords[0] = pcoords[1] = pcoords[2] = params[0];

		constexpr double kConverged = 1e-4; // VTK_CONVERGED
		constexpr double kDiverged = 1e6;   // VTK_DIVERGED
		constexpr int kMaxIterations = 20;  // VTK_MAX_ITERATIONS

		//  enter iteration loop
		int converged = 0;
		for (int iteration = 0; !converged && iteration < kMaxIterations; ++iteration) {
			//  calculate element interpolation functions and derivatives
			this->InterpolationFunctions(pcoords, weights);
			this->InterpolationDerivs(pcoords, derivs);

			//  calculate newton functions
			double fcol[3] = {0, 0, 0}, rcol[3] = {0, 0, 0}, scol[3] = {0, 0, 0},
			       tcol[3] = {0, 0, 0};
			for (int i = 0; i < 5; ++i) {
				for (int j = 0; j < 3; ++j) {
					const double coord = pts[i][j];
					fcol[j] += coord * weights[i];
					rcol[j] += coord * derivs[i];
					scol[j] += coord * derivs[i + 5];
					tcol[j] += coord * derivs[i + 10];
				}
			}
			for (int i = 0; i < 3; ++i) { fcol[i] -= xd[i]; }

			//  compute determinants and generate improvements
			const double d = Determinant3x3(rcol, scol, tcol);
			if (std::fabs(d) < determinantTolerance) { return setFailedOutput(); }

			pcoords[0] = params[0] - Determinant3x3(fcol, scol, tcol) / d;
			pcoords[1] = params[1] - Determinant3x3(rcol, fcol, tcol) / d;
			pcoords[2] = params[2] - Determinant3x3(rcol, scol, fcol) / d;

			//  check for convergence
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
			//  if not converged, repeat
			else {
				params[0] = pcoords[0];
				params[1] = pcoords[1];
				params[2] = pcoords[2];
			}
		}

		//  if not converged, set the parametric coordinates to arbitrary values
		//  outside of element
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
			double pc[3], w[5];
			// only approximate, not really true for warped hexa
			for (int i = 0; i < 3; ++i) {
				if (pcoords[i] < 0.0) { pc[i] = 0.0; }
				else if (pcoords[i] > 1.0) { pc[i] = 1.0; }
				else { pc[i] = pcoords[i]; }
			}
			// 等价 vtkPyramid::EvaluateLocation（内联其 double 计算，避免 float 往返损失）
			this->InterpolationFunctions(pc, w);
			double cp[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < 5; ++i) {
				for (int j = 0; j < 3; ++j) { cp[j] += w[i] * pts[i][j]; }
			}
			closestPoint = Point(static_cast<float>(cp[0]), static_cast<float>(cp[1]),
			                     static_cast<float>(cp[2]));
			dist2 = Distance2BetweenPoints(cp, xd);
		}
		return 0;
	}

private:
	Pyramid()
	{
		this->m_Points->SetNumberOfPoints(5);
		this->m_PointIds->SetNumberOfIds(5);
		for (int i = 0; i < 5; i++)
		{
			this->m_Points->SetPoint(i, 0.0, 0.0, 0.0);
			this->m_PointIds->SetId(i, 0);
		}

		m_Line = Line::New();
		m_Triangle = Triangle::New();
		m_Quad = Quad::New();
	}
	~Pyramid() override = default;

	Line::Pointer m_Line;
    Triangle::Pointer m_Triangle;
    Quad::Pointer m_Quad;
};


IGAME_NAMESPACE_END
#endif