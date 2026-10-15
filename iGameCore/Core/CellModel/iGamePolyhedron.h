#ifndef iGamePolyhedron_h
#define iGamePolyhedron_h

#include "iGameVolume.h"
#include "iGamePolygon.h"

#include <limits>

IGAME_NAMESPACE_BEGIN
class Polyhedron : public Volume {
public:
	I_OBJECT(Polyhedron);
	static Pointer New() { return new Polyhedron; }

	IGenum GetCellType() const noexcept override { return IG_POLYHEDRON; }
	int GetCellSize() const noexcept override { return m_Points->GetNumberOfPoints(); }
	int GetNumberOfEdges() override { return 0; }
	int GetNumberOfFaces() override { return m_FaceOffset->GetNumberOfIds() - 1; }

	int GetEdgePointIds(const int edgeId, const igIndex*& ptIds)override { return 0; };
	int GetFacePointIds(const int faceId, const igIndex*& ptIds) override { return 0; };
	int GetFaceEdgeIds(const int faceId, const igIndex*& edgeIds) override { return 0; };
	int GetPointToOneRingPoints(const int ptId, const igIndex*& ptIds) override { return 0; };
	int GetPointToNeighborEdges(const int ptId, const igIndex*& edgeIds)override { return 0; };
	int GetPointToNeighborFaces(const int ptId, const igIndex*& faceIds)override { return 0; };
	int GetEdgeToNeighborFaces(const int edgeId, const igIndex*& faceIds)override { return 0; };
	int GetFaceToNeighborFaces(const int faceId, const igIndex*& faceIds)override { return 0; };
	std::vector<iGame::Cell::Pointer> clipCelltoTetra() override { return std::vector<iGame::Cell::Pointer>(); };

	Cell* GetEdge(const int edgeId) override {
		return nullptr;
	}
	Cell* GetFace(const int faceId) override {
		if (faceId >= GetNumberOfFaces())return nullptr;
		m_Polygon->Reset();
		igIndex st = m_FaceOffset->GetId(faceId);
		igIndex ed = m_FaceOffset->GetId(faceId + 1);
		//std::cout << st << ' ' << ed << '\n';
		for (int i = st; i < ed; ++i) {
			m_Polygon->m_PointIds->AddId(m_PointIds->GetId(i));
			m_Polygon->m_Points->AddPoint(m_Points->GetPoint(i));
		}
		return m_Polygon.get();
	}


	virtual void Reset() override {
		this->Cell::Reset();
		this->m_FaceOffset->Reset();
	}

	IdArray::Pointer m_FaceOffset;

	/* ==================================================================
	 * VTK 风格插值接口（对照 vtkPolyhedron / vtkMeanValueCoordinatesInterpolator）
	 *
	 * 拓扑存储差异（**必须先读**，这里的每个 id 语义都与 VTK 不同）：
	 *   - VTK vtkPolyhedron：Points 是单元的**唯一**顶点（局部 id 0..n-1），Faces 是
	 *     vtkCellArray，其元素是 Points 的局部 id；GlobalFaces 存全局点 id，内部用
	 *     PointIdMap 做 全局 id → 局部 id 的映射。
	 *   - 本类：m_PointIds / m_Points 是**按面展开**的扁平列表（同一顶点属于多个面时会
	 *     重复占用多个槽位），m_FaceOffset 给出每个面在扁平列表中的区间
	 *     [m_FaceOffset[f], m_FaceOffset[f+1])。于是“面 f 的第 k 个点”= 扁平下标
	 *     st + k（st = m_FaceOffset->GetId(f)），它同时是 m_Points 与 m_PointIds 的下标。
	 * 因此 VTK 中所有“用局部/全局 id 索引 Points 并回写 weights[id]”的地方，这里直接换成
	 * 扁平槽位下标：插值权重长度取 GetCellSize()（= 槽位数），同一顶点的多个槽位各自只累加
	 * “所属面”的贡献（VTK 中该顶点在所有相邻面上的贡献之和恰好等于这些槽位之和），
	 * 因此权重总和与 VTK 按唯一顶点计的结果一致，Σw_i·value_i 亦一致
	 * （value 由 m_PointIds->GetId(slot) 映射到点数据）。
	 * 无需索引置换；面的顶点顺序按网格给定的环绕顺序使用（与 VTK 的面顶点顺序一致）。
	 * ================================================================== */

	int GetParametricDimension() const override { return 3; }

	/* ---------------------- 小工具（vtkMath 复刻） ---------------------- */

	static double Dot3(const double a[3], const double b[3]) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	static void Cross3(const double a[3], const double b[3], double c[3]) {
		c[0] = a[1] * b[2] - a[2] * b[1];
		c[1] = a[2] * b[0] - a[0] * b[2];
		c[2] = a[0] * b[1] - a[1] * b[0];
	}

	static void Subtract3(const double a[3], const double b[3], double c[3]) {
		c[0] = a[0] - b[0];
		c[1] = a[1] - b[1];
		c[2] = a[2] - b[2];
	}

	static double Norm3(const double a[3]) {
		return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	}

	/** vtkMath::Distance2BetweenPoints 的复刻：返回**平方**距离 */
	static double Distance2BetweenPoints(const double a[3], const double b[3]) {
		const double dx = a[0] - b[0];
		const double dy = a[1] - b[1];
		const double dz = a[2] - b[2];
		return dx * dx + dy * dy + dz * dz;
	}

	/** vtkMath::Normalize 的复刻：返回原向量长度（长度为 0 时向量不变） */
	static double NormalizeInPlace(double v[3]) {
		const double den = Norm3(v);
		if (den != 0.0) {
			const double s = 1.0 / den;
			v[0] *= s;
			v[1] *= s;
			v[2] *= s;
		}
		return den;
	}

	/** vtkMath::Determinant3x3 的复刻：det = a · (b × c) */
	static double Determinant3x3(const double a[3], const double b[3], const double c[3]) {
		return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
		       a[2] * (b[0] * c[1] - b[1] * c[0]);
	}

	/**
	 * 2·asin(l/2) 的复刻：l 是两个单位向量的弦长（数学上 ≤ 2）。
	 * 与 VTK 的唯一差异：把 l/2 夹到 ≤1，避免舍入误差使其略大于 1 而产生 NaN
	 * （仓库既有的 MeanValueFaceWeights 亦如此处理）。
	 */
	static double ChordAngle(double l) {
		const double half = 0.5 * l;
		return 2.0 * std::asin(half <= 1.0 ? half : 1.0);
	}

	/**
	 * vtkLine::DistanceToLine 的 double 数组版复刻：
	 *   t 返回参数位置且**不夹紧**（可 <0 或 >1）；closest 返回线段上的最近点
	 *   （已夹到两端点，可为 nullptr）；返回 x 到最近点的**平方**距离。
	 * 与基类 Cell::DistancePointToLine（Point 版）语义相同，此处避免 float 往返。
	 */
	static double DistanceToLine(const double x[3], const double p1[3], const double p2[3],
	                             double& t, double* closest) {
		const double p21[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
		double closestPt[3] = {p1[0], p1[1], p1[2]};
		const double num = p21[0] * (x[0] - p1[0]) + p21[1] * (x[1] - p1[1]) +
		                   p21[2] * (x[2] - p1[2]);
		if (num == 0.0) {
			t = 0.0;
		} else {
			const double denom = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];
			double tolerance = 1.0e-8 * num; // VTK_TOL
			if (tolerance < 0.0) { tolerance = -tolerance; }
			if (denom < tolerance) { // 端点退化：用极值 t 表达“落在端点上”
				if (num > 0.0) {
					closestPt[0] = p1[0] + p21[0];
					closestPt[1] = p1[1] + p21[1];
					closestPt[2] = p1[2] + p21[2];
					t = 1.0e300;
				} else {
					t = -1.0e300;
				}
			} else if ((t = num / denom) < 0.0) {
				// 最近点是端点 p1，t 保留负值（VTK 语义）
			} else if (t > 1.0) {
				closestPt[0] = p1[0] + p21[0];
				closestPt[1] = p1[1] + p21[1];
				closestPt[2] = p1[2] + p21[2];
			} else {
				closestPt[0] = p1[0] + t * p21[0];
				closestPt[1] = p1[1] + t * p21[1];
				closestPt[2] = p1[2] + t * p21[2];
			}
		}
		if (closest != nullptr) {
			closest[0] = closestPt[0];
			closest[1] = closestPt[1];
			closest[2] = closestPt[2];
		}
		return Distance2BetweenPoints(closestPt, x);
	}

	/**
	 * vtkTriangle::DistanceToTriangle 的复刻：返回 x 到三角形的**平方**距离，
	 * 可选返回最近点（对应 VTK 传 nullptr 的最近点出参，这里扩展出最近点以便填写
	 * EvaluatePosition 的 closestPoint）。分支与 VTK 逐行一致：
	 *   - 退化三角形（法向长度为 0）→ 三条边的最小距离；
	 *   - 投影落在三角形内 → 平面距离的平方（最近点即投影点 xp）；
	 *   - 否则 → 三条边的最小距离（最近点为该边上的最近点）。
	 */
	static double DistanceToTriangle(const double x[3], const double p1[3], const double p2[3],
	                                 const double p3[3], double* closest = nullptr) {
		double unusedT;
		const double e1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
		const double e2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};
		double n[3];
		Cross3(e1, e2, n);

		const double n2 = Dot3(n, n);
		if (n2 <= 0.0) {
			// Degenerate triangle -- fall back to edge distances.
			double c1[3], c2[3], c3[3];
			const double d1 = DistanceToLine(x, p1, p2, unusedT, c1);
			const double d2 = DistanceToLine(x, p2, p3, unusedT, c2);
			const double d3 = DistanceToLine(x, p3, p1, unusedT, c3);
			double best = d1;
			const double* bc = c1;
			if (d2 < best) { best = d2; bc = c2; }
			if (d3 < best) { best = d3; bc = c3; }
			if (closest != nullptr) {
				closest[0] = bc[0];
				closest[1] = bc[1];
				closest[2] = bc[2];
			}
			return best;
		}

		const double invLenN = 1.0 / std::sqrt(n2);

		// Signed distance to the triangle plane.
		double p1x[3];
		Subtract3(x, p1, p1x);
		const double distPlane = Dot3(p1x, n) * invLenN;

		// Project x onto the plane.
		const double xp[3] = {x[0] - distPlane * n[0] * invLenN,
		                      x[1] - distPlane * n[1] * invLenN,
		                      x[2] - distPlane * n[2] * invLenN};

		// Barycentric test using dot products.
		double v0[3], v1[3], v2[3];
		Subtract3(p3, p1, v0);
		Subtract3(p2, p1, v1);
		Subtract3(xp, p1, v2);

		const double dot00 = Dot3(v0, v0);
		const double dot01 = Dot3(v0, v1);
		const double dot02 = Dot3(v0, v2);
		const double dot11 = Dot3(v1, v1);
		const double dot12 = Dot3(v1, v2);

		const double denom = dot00 * dot11 - dot01 * dot01;
		if (denom != 0.0) {
			const double invDenom = 1.0 / denom;
			const double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
			const double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

			if (u >= 0.0 && v >= 0.0 && (u + v) <= 1.0) {
				if (closest != nullptr) {
					closest[0] = xp[0];
					closest[1] = xp[1];
					closest[2] = xp[2];
				}
				return distPlane * distPlane;
			}
		}

		// Outside the triangle -- closest point lies on an edge.
		double c1[3], c2[3], c3[3];
		const double d1 = DistanceToLine(x, p1, p2, unusedT, c1);
		const double d2 = DistanceToLine(x, p2, p3, unusedT, c2);
		const double d3 = DistanceToLine(x, p3, p1, unusedT, c3);
		double best = d1;
		const double* bc = c1;
		if (d2 < best) { best = d2; bc = c2; }
		if (d3 < best) { best = d3; bc = c3; }
		if (closest != nullptr) {
			closest[0] = bc[0];
			closest[1] = bc[1];
			closest[2] = bc[2];
		}
		return best;
	}

	/** vtkTriangle::SolidAngle 的复刻：Van Oosterom-Strackee 有符号立体角公式 */
	static double SolidAngle(const double x[3], const double p1[3], const double p2[3],
	                         const double p3[3]) {
		double a[3], b[3], c[3];
		Subtract3(p1, x, a);
		Subtract3(p2, x, b);
		Subtract3(p3, x, c);

		const double la = Norm3(a);
		const double lb = Norm3(b);
		const double lc = Norm3(c);

		// det = a . (b x c)
		double bxc[3];
		Cross3(b, c, bxc);
		const double det = Dot3(a, bxc);

		const double denom = la * lb * lc + Dot3(a, b) * lc + Dot3(a, c) * lb + Dot3(b, c) * la;

		// atan2 handles denom ~ 0 robustly.
		return 2.0 * std::atan2(det, denom);
	}

	/* ---------------- 存储访问与包围盒 / 参数坐标（vtkPolyhedron） ---------------- */

	/**
	 * 面数（等价 GetNumberOfFaces()）。基类的 GetNumberOfFaces() 不是 const，
	 * 而 VTK 风格接口全部是 const，因此这里提供 const 版本；空 m_FaceOffset 返回 -1，
	 * 与 GetNumberOfFaces() 的行为一致。
	 */
	int FaceCount() const { return static_cast<int>(this->m_FaceOffset->GetNumberOfIds()) - 1; }

	/** 面 faceId 在扁平点表中的槽位区间 [st, ed) */
	void GetFaceSlotRange(int faceId, int& st, int& ed) const {
		const int nfaces = this->FaceCount();
		if (faceId < 0 || faceId >= nfaces) {
			st = 0;
			ed = 0;
			return;
		}
		st = this->m_FaceOffset->GetId(faceId);
		ed = this->m_FaceOffset->GetId(faceId + 1);
	}

	/** 取扁平槽位 slot 的坐标（double） */
	void GetSlotPoint(int slot, double p[3]) const {
		const Point& P = this->m_Points->GetPoint(slot);
		p[0] = static_cast<double>(P[0]);
		p[1] = static_cast<double>(P[1]);
		p[2] = static_cast<double>(P[2]);
	}

	/** vtkCell::ComputeBounds 的复刻：bounds = (xmin,xmax,ymin,ymax,zmin,zmax) */
	void ComputeBounds(double bounds[6]) const {
		bounds[0] = bounds[2] = bounds[4] = DBL_MAX;
		bounds[1] = bounds[3] = bounds[5] = -DBL_MAX;
		const int npts = this->GetCellSize();
		for (int i = 0; i < npts; ++i) {
			const Point& p = this->m_Points->GetPoint(i);
			for (int j = 0; j < 3; ++j) {
				const double v = static_cast<double>(p[j]);
				if (v < bounds[2 * j]) { bounds[2 * j] = v; }
				if (v > bounds[2 * j + 1]) { bounds[2 * j + 1] = v; }
			}
		}
	}

	/**
	 * vtkPolyhedron::ComputeParametricCoordinate 的复刻：pcoords 为 x 在单元包围盒内的
	 * 归一化坐标（允许超出 [0,1]）。VTK 不做零跨度保护，这里保持一致（退化的包围盒会产生
	 * inf/nan，与 VTK 行为相同）。
	 */
	void ComputeParametricCoordinate(const double x[3], double pc[3]) const {
		double bounds[6];
		this->ComputeBounds(bounds);

		pc[0] = (x[0] - bounds[0]) / (bounds[1] - bounds[0]);
		pc[1] = (x[1] - bounds[2]) / (bounds[3] - bounds[2]);
		pc[2] = (x[2] - bounds[4]) / (bounds[5] - bounds[4]);
	}

	/** vtkPolyhedron::ComputePositionFromParametricCoordinate 的复刻（包围盒线性映射） */
	void ComputePositionFromParametricCoordinate(const double pc[3], double x[3]) const {
		double bounds[6];
		this->ComputeBounds(bounds);

		x[0] = (1 - pc[0]) * bounds[0] + pc[0] * bounds[1];
		x[1] = (1 - pc[1]) * bounds[2] + pc[1] * bounds[3];
		x[2] = (1 - pc[2]) * bounds[4] + pc[2] * bounds[5];
	}

	/**
	 * vtkPolyhedron::GetCentroid 的复刻（散度定理）：每个面以其首个顶点扇形三角化，
	 * 三角形 (v0,v1,v2) 与原点围成有符号四面体，其 6 倍体积 = v0·(v1×v2)；质心为各
	 * 四面体质心 (v0+v1+v2)/4 按有符号体积加权。|6V| <= 1e-12 时退化为**槽位**坐标平均
	 * （VTK 为唯一顶点平均；本类点表含重复顶点，故重复顶点会按出现次数加权，这是与 VTK
	 * 的唯一差别，且仅在退化体积时生效）。返回是否使用了体积加权。
	 */
	bool ComputeCentroid(double centroid[3]) const {
		double totalVolume6 = 0.0;
		centroid[0] = centroid[1] = centroid[2] = 0.0;

		const int nfaces = this->FaceCount();
		for (int f = 0; f < nfaces; ++f) {
			int st = 0, ed = 0;
			this->GetFaceSlotRange(f, st, ed);
			const int numFacePts = ed - st;
			if (numFacePts < 3) { continue; }

			double v0[3], v1[3], v2[3], cross[3];
			this->GetSlotPoint(st, v0);
			for (int j = 1; j + 1 < numFacePts; ++j) {
				this->GetSlotPoint(st + j, v1);
				this->GetSlotPoint(st + j + 1, v2);

				// Signed tet volume x 6 = v0 . (v1 x v2)
				Cross3(v1, v2, cross);
				const double det = Dot3(v0, cross);

				totalVolume6 += det;
				// Combined: det * (v0 + v1 + v2) / 24. Defer the 1/24 division.
				for (int k = 0; k < 3; ++k) {
					centroid[k] += det * (v0[k] + v1[k] + v2[k]);
				}
			}
		}

		if (std::fabs(totalVolume6) > 1e-12) {
			// centroid = accumulated / (24 * (totalVolume6 / 6)) = accumulated / (4 * totalVolume6)
			const double invW = 1.0 / (4.0 * totalVolume6);
			centroid[0] *= invW;
			centroid[1] *= invW;
			centroid[2] *= invW;
			return true;
		}

		// Degenerate volume -- fall back to point average
		const int numPts = this->GetCellSize();
		centroid[0] = centroid[1] = centroid[2] = 0.0;
		if (numPts <= 0) { return false; }
		for (int i = 0; i < numPts; ++i) {
			double x[3];
			this->GetSlotPoint(i, x);
			centroid[0] += x[0];
			centroid[1] += x[1];
			centroid[2] += x[2];
		}
		const double inv = 1.0 / static_cast<double>(numPts);
		centroid[0] *= inv;
		centroid[1] *= inv;
		centroid[2] *= inv;
		return false;
	}

	/**
	 * vtkPolyhedron::GetParametricCenter 的复刻：
	 * GenerateFaces() + GetCentroid() + ComputeParametricCoordinate()，
	 * 即“质心 → 包围盒归一化坐标”。
	 */
	void GetParametricCenter(double pcoords[3]) const override {
		double centroid[3];
		this->ComputeCentroid(centroid);
		this->ComputeParametricCoordinate(centroid, pcoords);
	}

	/* ---------------------- 包含判定 / 边界最近点 ---------------------- */

	/**
	 * 点到多面体边界面片的最小**平方**距离与最近点。
	 * master 的 vtkPolyhedron::EvaluatePosition 用 vtkCellLocator::FindClosestPoint 在
	 * 边界面 polydata 上取最近点；这里把每个面扇形三角化后直接取精确最小值，二者语义相同
	 * （都是“边界上的最近点”），差别仅在于没有 locator 的加速，且对非凸面用扇形三角化时
	 * 理论上可能漏掉极少数最近点（凸面/平面面不会有此问题）。
	 */
	double DistanceToBoundary(const double x[3], double closest[3]) const {
		double best = DBL_MAX;
		closest[0] = x[0];
		closest[1] = x[1];
		closest[2] = x[2];

		const int nfaces = this->FaceCount();
		for (int f = 0; f < nfaces; ++f) {
			int st = 0, ed = 0;
			this->GetFaceSlotRange(f, st, ed);
			const int numFacePts = ed - st;
			if (numFacePts < 3) { continue; }

			double v0[3], v1[3], v2[3], c[3];
			this->GetSlotPoint(st, v0);
			for (int k = 1; k + 1 < numFacePts; ++k) {
				this->GetSlotPoint(st + k, v1);
				this->GetSlotPoint(st + k + 1, v2);
				const double d2 = DistanceToTriangle(x, v0, v1, v2, c);
				if (d2 < best) {
					best = d2;
					closest[0] = c[0];
					closest[1] = c[1];
					closest[2] = c[2];
				}
			}
		}
		return best;
	}

	/**
	 * vtkPolyhedron::IsInside 的复刻（master 的“立体角/绕数”路线）：
	 *   - 先做包围盒快速排除（盒外一定不在体内）；
	 *   - 把每个面扇形三角化，累加 x 对每个三角形的**有符号**立体角：
	 *     闭合且外法向一致时，内部 |Σ| = 4π、外部 ≈ 0；
	 *   - 贴面早退：tol = tolerance · length（length = 包围盒对角线长，等价
	 *     sqrt(vtkCell::GetLength2())），某三角形距离 ≤ tol² 时立即判为内部；
	 *   - |Σ| 与 {0, 4π} 的接近程度不足以判定时，取更接近的一类。
	 * 与 VTK master 的差异（显式说明）：
	 *   1) master 的 EvaluatePosition 传 tolerance = +inf，会使 `dist ≤ tol2` 恒真——
	 *      只要点落在包围盒内就判为内部（疑似上游回归）。这里把**非有限** tolerance
	 *      解释为“关闭贴面早退、只用立体角判定”，与 master 注释表达的意图一致；
	 *      有限容差时行为与 master 逐行一致。
	 *   2) master 对 >4 边面用 vtkPolygon::EarClipPolygon3D 三角化，这里用扇形三角化：
	 *      对平面多边形，有符号立体角之和与三角化方式无关（扇形落在多边形外的三角形
	 *      贡献会相互抵消），结果一致。
	 */
	int IsInside(const double x[3], double tolerance) const {
		double bounds[6];
		this->ComputeBounds(bounds);
		if (x[0] < bounds[0] || x[0] > bounds[1] || x[1] < bounds[2] || x[1] > bounds[3] ||
		    x[2] < bounds[4] || x[2] > bounds[5]) {
			return 0;
		}

		// Match legacy behavior: scale tolerance by cell length
		double length2 = 0.0;
		for (int i = 0; i < 3; ++i) {
			const double diff = bounds[2 * i + 1] - bounds[2 * i];
			length2 += diff * diff;
		}
		const double length = std::sqrt(length2);
		const double tol = tolerance * length;
		const double tol2 = tol * tol;
		// 见上面差异说明 1)：非有限容差表示关闭贴面早退
		const bool useNearSurfaceEarlyOut = std::isfinite(tol);

		constexpr double kPi = 3.14159265358979323846;
		constexpr double FourPi = 4.0 * kPi;
		constexpr double angleTol = 1e-3;

		double total = 0.0;
		const int nfaces = this->FaceCount();
		for (int f = 0; f < nfaces; ++f) {
			int st = 0, ed = 0;
			this->GetFaceSlotRange(f, st, ed);
			const int numFacePts = ed - st;
			if (numFacePts < 3) { continue; }

			double v0[3], v1[3], v2[3];
			this->GetSlotPoint(st, v0);
			for (int k = 1; k + 1 < numFacePts; ++k) {
				this->GetSlotPoint(st + k, v1);
				this->GetSlotPoint(st + k + 1, v2);

				if (useNearSurfaceEarlyOut && tol > 0.0 &&
				    DistanceToTriangle(x, v0, v1, v2, nullptr) <= tol2) {
					return 1; // 贴面：判为内部
				}
				total += SolidAngle(x, v0, v1, v2);
			}
		}

		const double absTotal = std::fabs(total);

		if (std::fabs(absTotal - FourPi) <= angleTol) {
			return 1; // clearly inside
		}
		if (absTotal <= angleTol) {
			return 0; // clearly outside
		}

		// Ambiguous: choose closer class
		return (std::fabs(absTotal - FourPi) < absTotal) ? 1 : 0;
	}

	/* ---------------- 均值坐标插值权重 ---------------- */

	/** 是否所有面都是三角形（对应 VTK 的 cells->IsHomogeneous() == 3 快速路径判据） */
	bool IsTriangleMesh() const {
		const int nfaces = this->FaceCount();
		if (nfaces <= 0) { return false; }
		for (int f = 0; f < nfaces; ++f) {
			int st = 0, ed = 0;
			this->GetFaceSlotRange(f, st, ed);
			if (ed - st != 3) { return false; }
		}
		return true;
	}

	/**
	 * vtkMeanValueCoordinatesInterpolator 的**多边形网格**分支复刻
	 * （ComputeWeightsForPolygonMesh）：把每个面的顶点方向投影到单位球面，
	 * 用球面多边形的角度权重逐面累加，最后整体归一化。
	 * 权重长度 = GetCellSize()（扁平槽位数）；累加下标用扁平槽位（VTK 用 Points 局部 id），
	 * 语义换算见类内 VTK 风格接口开头的大段注释。
	 * 与 VTK 的差异仅在于 asin 自变量的夹紧（见 ChordAngle）。
	 */
	void ComputeInterpolationWeightsPolygonMesh(const double x[3], double* weights) const {
		static constexpr double eps = 0.00000001;
		const int npts = this->GetCellSize();
		for (int i = 0; i < npts; ++i) { weights[i] = 0.0; }
		if (npts <= 0) { return; }

		// create local array for storing point-to-vertex vectors and distances
		std::vector<double> dist(static_cast<size_t>(npts));
		std::vector<double> uVec(static_cast<size_t>(3 * npts));
		for (int pid = 0; pid < npts; ++pid) {
			double pt[3];
			this->GetSlotPoint(pid, pt);
			uVec[3 * pid] = pt[0] - x[0];
			uVec[3 * pid + 1] = pt[1] - x[1];
			uVec[3 * pid + 2] = pt[2] - x[2];

			// distance
			dist[pid] = std::sqrt(uVec[3 * pid] * uVec[3 * pid] +
			                      uVec[3 * pid + 1] * uVec[3 * pid + 1] +
			                      uVec[3 * pid + 2] * uVec[3 * pid + 2]);

			// handle special case when the point is really close to a vertex
			if (dist[pid] < eps) {
				weights[pid] = 1.0;
				return;
			}

			// project onto unit sphere
			uVec[3 * pid] /= dist[pid];
			uVec[3 * pid + 1] /= dist[pid];
			uVec[3 * pid + 2] /= dist[pid];
		}

		const int nfaces = this->FaceCount();
		std::vector<const double*> u;
		std::vector<double> alpha;
		std::vector<double> theta;

		for (int f = 0; f < nfaces; ++f) {
			int st = 0, ed = 0;
			this->GetFaceSlotRange(f, st, ed);
			const int nPolyPts = ed - st;
			if (nPolyPts == 0) { continue; }

			u.resize(static_cast<size_t>(nPolyPts));
			for (int j = 0; j < nPolyPts; ++j) { u[j] = &uVec[3 * (st + j)]; }

			// unit vector v.
			double v[3] = {0.0, 0.0, 0.0};
			double temp[3];
			for (int j = 0; j + 1 < nPolyPts; ++j) {
				Cross3(u[j], u[j + 1], temp);
				NormalizeInPlace(temp);

				const double l = std::sqrt(Distance2BetweenPoints(u[j], u[j + 1]));
				const double angle = ChordAngle(l);

				v[0] += 0.5 * angle * temp[0];
				v[1] += 0.5 * angle * temp[1];
				v[2] += 0.5 * angle * temp[2];
			}
			{
				const double l = std::sqrt(Distance2BetweenPoints(u[nPolyPts - 1], u[0]));
				const double angle = ChordAngle(l);
				Cross3(u[nPolyPts - 1], u[0], temp);
				NormalizeInPlace(temp);
				v[0] += 0.5 * angle * temp[0];
				v[1] += 0.5 * angle * temp[1];
				v[2] += 0.5 * angle * temp[2];
			}

			const double vNorm = NormalizeInPlace(v);

			// The direction of v depends on the orientation (clockwise or
			// contour-clockwise) of the polygon. We want to make sure that v
			// starts from x and point towards the polygon.
			if (Dot3(v, u[0]) < 0) {
				v[0] = -v[0];
				v[1] = -v[1];
				v[2] = -v[2];
			}

			alpha.assign(static_cast<size_t>(nPolyPts), 0.0);
			theta.assign(static_cast<size_t>(nPolyPts), 0.0);

			// angles between edges
			double n0[3], n1[3];
			for (int j = 0; j + 1 < nPolyPts; ++j) {
				// alpha
				Cross3(u[j], v, n0);
				NormalizeInPlace(n0);
				Cross3(u[j + 1], v, n1);
				NormalizeInPlace(n1);

				// alpha[j] = acos(vtkMath::Dot(n0, n1));
				alpha[j] = ChordAngle(std::sqrt(Distance2BetweenPoints(n0, n1)));
				Cross3(n0, n1, temp);
				if (Dot3(temp, v) < 0) { alpha[j] = -alpha[j]; }

				// theta_j
				// theta[j] = acos(vtkMath::Dot(u[j], v));
				theta[j] = ChordAngle(std::sqrt(Distance2BetweenPoints(u[j], v)));
			}

			Cross3(u[nPolyPts - 1], v, n0);
			NormalizeInPlace(n0);
			Cross3(u[0], v, n1);
			NormalizeInPlace(n1);
			// alpha[nPolyPts-1] = acos(vtkMath::Dot(n0, n1));
			alpha[nPolyPts - 1] = ChordAngle(std::sqrt(Distance2BetweenPoints(n0, n1)));
			Cross3(n0, n1, temp);
			if (Dot3(temp, v) < 0) { alpha[nPolyPts - 1] = -alpha[nPolyPts - 1]; }

			// theta[nPolyPts-1] = acos(vtkMath::Dot(u[nPolyPts-1], v));
			theta[nPolyPts - 1] = ChordAngle(std::sqrt(Distance2BetweenPoints(u[nPolyPts - 1], v)));

			bool outlierFlag = false;
			for (int j = 0; j < nPolyPts; ++j) {
				if (std::fabs(theta[j]) < eps) {
					outlierFlag = true;
					weights[st + j] += vNorm / dist[st + j];
					break;
				}
			}
			if (outlierFlag) { continue; }

			double sum = 0.0;
			sum += 1.0 / std::tan(theta[0]) *
			       (std::tan(alpha[0] / 2.0) + std::tan(alpha[nPolyPts - 1] / 2.0));
			for (int j = 1; j < nPolyPts; ++j) {
				sum += 1.0 / std::tan(theta[j]) * (std::tan(alpha[j] / 2.0) +
				                                   std::tan(alpha[j - 1] / 2.0));
			}

			// the special case when x lies on the polygon, handle it using 2D mvc.
			// in the 2D case, alpha = theta
			if (std::fabs(sum) < eps) {
				for (int i = 0; i < npts; ++i) { weights[i] = 0.0; }

				// recompute theta, the theta computed previously are not robust
				for (int j = 0; j + 1 < nPolyPts; ++j) {
					theta[j] = ChordAngle(std::sqrt(Distance2BetweenPoints(u[j], u[j + 1])));
				}
				theta[nPolyPts - 1] =
				    ChordAngle(std::sqrt(Distance2BetweenPoints(u[nPolyPts - 1], u[0])));

				weights[st] = 1.0 / dist[st] * (std::tan(theta[nPolyPts - 1] / 2.0) +
				                                std::tan(theta[0] / 2.0));
				double sumWeight = weights[st];
				for (int j = 1; j < nPolyPts; ++j) {
					weights[st + j] = 1.0 / dist[st + j] *
					                  (std::tan(theta[j - 1] / 2.0) + std::tan(theta[j] / 2.0));
					sumWeight = sumWeight + weights[st + j];
				}

				// 注意：VTK 此处没有取绝对值（与上面的 |sum| 判定不同），保持原样
				if (sumWeight < eps) { return; }

				for (int j = 0; j < nPolyPts; ++j) { weights[st + j] /= sumWeight; }
				return;
			}

			// weight
			weights[st] += vNorm / sum / dist[st] / std::sin(theta[0]) *
			               (std::tan(alpha[0] / 2.0) + std::tan(alpha[nPolyPts - 1] / 2.0));
			for (int j = 1; j < nPolyPts; ++j) {
				weights[st + j] += vNorm / sum / dist[st + j] / std::sin(theta[j]) *
				                   (std::tan(alpha[j] / 2.0) + std::tan(alpha[j - 1] / 2.0));
			}
		}

		// normalize weight
		double sumWeight = 0.0;
		for (int i = 0; i < npts; ++i) { sumWeight += weights[i]; }

		if (std::fabs(sumWeight) < eps) { return; }

		for (int i = 0; i < npts; ++i) { weights[i] /= sumWeight; }
	}

	/**
	 * vtkMeanValueCoordinatesInterpolator 的**三角形网格**分支复刻
	 * （ComputeWeightsForTriangleMesh）：只在“所有面都是三角形”时使用
	 * （与 VTK 的 canFastPath 判据一致，注意该分支的 eps 是 1e-9，与多边形分支的 1e-8 不同）。
	 * 权重长度 = GetCellSize()；三角形列表即按面展开的扁平点表（每 3 个槽位一个三角形）。
	 */
	void ComputeInterpolationWeightsTriangleMesh(const double x[3], double* weights) const {
		constexpr double kPi = 3.14159265358979323846;
		static constexpr double eps = 0.000000001;
		const int npts = this->GetCellSize();
		for (int i = 0; i < npts; ++i) { weights[i] = 0.0; }
		if (npts <= 0) { return; }

		// create local array for storing point-to-vertex vectors and distances
		std::vector<double> dist(static_cast<size_t>(npts));
		std::vector<double> uVec(static_cast<size_t>(3 * npts));
		for (int pid = 0; pid < npts; ++pid) {
			double pt[3];
			this->GetSlotPoint(pid, pt);
			uVec[3 * pid] = pt[0] - x[0];
			uVec[3 * pid + 1] = pt[1] - x[1];
			uVec[3 * pid + 2] = pt[2] - x[2];

			// distance
			dist[pid] = std::sqrt(uVec[3 * pid] * uVec[3 * pid] +
			                      uVec[3 * pid + 1] * uVec[3 * pid + 1] +
			                      uVec[3 * pid + 2] * uVec[3 * pid + 2]);

			// handle special case when the point is really close to a vertex
			if (dist[pid] < eps) {
				weights[pid] = 1.0;
				return;
			}

			// project onto unit sphere
			uVec[3 * pid] /= dist[pid];
			uVec[3 * pid + 1] /= dist[pid];
			uVec[3 * pid + 2] /= dist[pid];
		}

		// Now loop over all triangle to compute weights
		const int numTris = npts / 3;
		for (int tri = 0; tri < numTris; ++tri) {
			// vertex id（这里即扁平槽位号）
			const int pid0 = 3 * tri;
			const int pid1 = 3 * tri + 1;
			const int pid2 = 3 * tri + 2;

			// unit vector
			const double* u0 = &uVec[3 * pid0];
			const double* u1 = &uVec[3 * pid1];
			const double* u2 = &uVec[3 * pid2];

			// edge length
			const double l0 = std::sqrt(Distance2BetweenPoints(u1, u2));
			const double l1 = std::sqrt(Distance2BetweenPoints(u2, u0));
			const double l2 = std::sqrt(Distance2BetweenPoints(u0, u1));

			// angle
			const double theta0 = ChordAngle(l0);
			const double theta1 = ChordAngle(l1);
			const double theta2 = ChordAngle(l2);
			const double halfSum = (theta0 + theta1 + theta2) / 2.0;

			// special case when the point lies on the triangle
			if (kPi - halfSum < eps) {
				for (int i = 0; i < npts; ++i) { weights[i] = 0.0; }

				weights[pid0] = std::sin(theta0) * dist[pid1] * dist[pid2];
				weights[pid1] = std::sin(theta1) * dist[pid2] * dist[pid0];
				weights[pid2] = std::sin(theta2) * dist[pid0] * dist[pid1];

				double sumWeight = weights[pid0] + weights[pid1] + weights[pid2];

				weights[pid0] /= sumWeight;
				weights[pid1] /= sumWeight;
				weights[pid2] /= sumWeight;
				return;
			}

			// coefficient
			const double sinHalfSum = std::sin(halfSum);
			const double sinHalfSumSubTheta0 = std::sin(halfSum - theta0);
			const double sinHalfSumSubTheta1 = std::sin(halfSum - theta1);
			const double sinHalfSumSubTheta2 = std::sin(halfSum - theta2);
			const double sinTheta0 = std::sin(theta0);
			const double sinTheta1 = std::sin(theta1);
			const double sinTheta2 = std::sin(theta2);

			double c0 = 2 * sinHalfSum * sinHalfSumSubTheta0 / sinTheta1 / sinTheta2 - 1;
			double c1 = 2 * sinHalfSum * sinHalfSumSubTheta1 / sinTheta2 / sinTheta0 - 1;
			double c2 = 2 * sinHalfSum * sinHalfSumSubTheta2 / sinTheta0 / sinTheta1 - 1;

			if (std::fabs(c0) > 1) { c0 = c0 > 0 ? 1 : -1; }
			if (std::fabs(c1) > 1) { c1 = c1 > 0 ? 1 : -1; }
			if (std::fabs(c2) > 1) { c2 = c2 > 0 ? 1 : -1; }

			// sign
			const double det = Determinant3x3(u0, u1, u2);

			if (std::fabs(det) < eps) { continue; }

			const double detSign = det > 0 ? 1 : -1;
			const double sign0 = detSign * std::sqrt(1 - c0 * c0);
			const double sign1 = detSign * std::sqrt(1 - c1 * c1);
			const double sign2 = detSign * std::sqrt(1 - c2 * c2);

			// if x lies on the plane of current triangle but outside it, ignore
			// the current triangle.
			if (std::fabs(sign0) < eps || std::fabs(sign1) < eps || std::fabs(sign2) < eps) {
				continue;
			}

			// weight
			weights[pid0] +=
			    (theta0 - c1 * theta2 - c2 * theta1) / (dist[pid0] * sinTheta1 * sign2);
			weights[pid1] +=
			    (theta1 - c2 * theta0 - c0 * theta2) / (dist[pid1] * sinTheta2 * sign0);
			weights[pid2] +=
			    (theta2 - c0 * theta1 - c1 * theta0) / (dist[pid2] * sinTheta0 * sign1);
		}

		// normalize weight
		double sumWeight = 0.0;
		for (int i = 0; i < npts; ++i) { sumWeight += weights[i]; }

		if (std::fabs(sumWeight) < eps) { return; }

		for (int i = 0; i < npts; ++i) { weights[i] /= sumWeight; }
	}

	/**
	 * vtkPolyhedron::InterpolateFunctions 的复刻：用均值坐标求插值权重（入参是**物理坐标 x**）。
	 * 权重长度 = GetCellSize()；分派判据与 VTK 一致（所有面都是三角形 → 三角形网格分支）。
	 * 归一化后权重和 ≈ 1（若上游分支因退化提前返回，权重和可能不是 1，与 VTK 相同）。
	 */
	void ComputeInterpolationWeights(const double x[3], double* weights) const {
		if (this->IsTriangleMesh()) {
			this->ComputeInterpolationWeightsTriangleMesh(x, weights);
		} else {
			this->ComputeInterpolationWeightsPolygonMesh(x, weights);
		}
	}

	/* ---------------------- 基类接口实现 ---------------------- */

	/**
	 * 基类签名以 pcoords 为入参，而 vtkPolyhedron 的权重函数（InterpolateFunctions）以
	 * **物理坐标**为入参。这里按 EvaluateLocation 的流程先把 pcoords 映射回物理点
	 * （包围盒线性映射），再调用均值坐标权重函数。
	 */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		double x[3];
		this->ComputePositionFromParametricCoordinate(pcoords, x);
		this->ComputeInterpolationWeights(x, weights);
	}

	/**
	 * vtkPolyhedron::InterpolateDerivs 是空实现（原文只有 `(void)x; (void)derivs;`）：
	 * 多面体的形函数导数没有闭式表达，VTK 的 vtkPolyhedron::Derivatives 是以
	 * EvaluateLocation 的有限差分求导的。这里按基类约定清零，避免调用方读到未初始化值。
	 */
	void InterpolationDerivs(const double*, double* derivs) const override {
		const int n = this->GetCellSize();
		for (int i = 0; i < 3 * n; ++i) { derivs[i] = 0.0; }
	}

	/**
	 * vtkPolyhedron::EvaluateLocation 的逐行复刻：
	 *   x = 由 pcoords 反算的物理坐标（包围盒线性映射），再取该物理点的均值坐标作为权重
	 *   （VTK 原文：ComputePositionFromParametricCoordinate(pcoords, x) 之后
	 *   InterpolateFunctions(x, weights)）。
	 */
	void EvaluateLocation(int& subId, const double pcoords[3], Point& x,
	                      double* weights) const override {
		subId = 0;
		double xd[3];
		this->ComputePositionFromParametricCoordinate(pcoords, xd);
		x = Point(static_cast<float>(xd[0]), static_cast<float>(xd[1]), static_cast<float>(xd[2]));

		this->ComputeInterpolationWeights(xd, weights);
	}

	/**
	 * vtkPolyhedron::EvaluatePosition 的逐行复刻：
	 *   - pcoords 直接由包围盒归一化给出（ComputeParametricCoordinate，**不是** Newton 反解，
	 *     因此 pcoords 与“点是否在体内”无关；允许超出 [0,1]）；
	 *   - weights 始终用查询点的均值坐标（VTK 注释：调用方总会用到）；
	 *   - 包含判定用 IsInside(x, +inf)（见 IsInside 的说明：非有限容差表示关闭贴面早退）；
	 *   - 内部：closestPoint = x、dist2 = 0，返回 1；
	 *   - 外部：closestPoint / dist2 取边界面片上的最近点与**平方**距离，返回 0
	 *     （master 用 vtkCellLocator 求该最近点，这里用精确的逐面三角化最小距离，语义相同）。
	 * 无论返回 0 还是 1，pcoords / weights / closestPoint / dist2 都会被填写。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
	                     double& dist2, double* weights) const override {
		subId = 0;
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// compute parametric coordinates
		this->ComputeParametricCoordinate(xd, pcoords);

		// The interpolation weights are always needed by callers
		this->ComputeInterpolationWeights(xd, weights);

		// Inside/outside test first.
		const int isInside = this->IsInside(xd, std::numeric_limits<double>::infinity());
		if (isInside) {
			dist2 = 0.0;
			closestPoint = x;
			return 1;
		}

		// Point is outside: find the closest point on the boundary and its squared distance.
		double cp[3];
		dist2 = this->DistanceToBoundary(xd, cp);
		closestPoint = Point(static_cast<float>(cp[0]), static_cast<float>(cp[1]),
		                     static_cast<float>(cp[2]));
		return 0;
	}

protected:
	Polyhedron() {
		m_Polygon = Polygon::New();
		m_FaceOffset = IdArray::New();
	}
	~Polyhedron() override = default;
	Polygon::Pointer m_Polygon;

};
IGAME_NAMESPACE_END
#endif