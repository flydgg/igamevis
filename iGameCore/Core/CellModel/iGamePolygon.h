#ifndef iGamePolygon_h
#define iGamePolygon_h

#include "iGameFace.h"
#include "iGameTriangle.h"

IGAME_NAMESPACE_BEGIN
class Polygon : public Face {
public:
	I_OBJECT(Polygon);
	static Pointer New() { return new Polygon; }

	IGenum GetCellType() const noexcept override { return IG_POLYGON; }
    int GetNumberOfEdges() override { return this->GetCellSize(); }

	Vector3f GetNormal() override
	{
		return this->Self::ComputrAverageNormal(this->m_Points);
	}

	static Vector3f ComputrAverageNormal(Points::Pointer points)
	{
		int npts = points->GetNumberOfPoints();
		Vector3f center(0);
		for (int i = 0; i < npts; i++) {
			center += points->GetPoint(i);
		}
		center /= npts;

		Vector3f normal;
		normal.setZero();

		for (int i = 0; i < npts - 1; i++) {
			const Point& v0 = points->GetPoint(i);
			const Point& v1 = points->GetPoint(i + 1);
			normal += Triangle::ComputrNormal(center, v0, v1);
		}
		const Point& v0 = points->GetPoint(npts - 1);
		const Point& v1 = points->GetPoint(0);
		normal += Triangle::ComputrNormal(center, v0, v1);

		normal.normalize();
		return normal;
	}

	/* ---------------- VTK 风格插值接口（对照 vtkPolygon） ----------------
	 * 拓扑约定：顶点按环序存放（0→1→…→n-1→0），边为 (i, i+1)、(n-1, 0)，
	 * 与 vtkPolygon 的顶点环序完全一致，无需索引置换。
	 * ==================================================================== */

	/** vtkPolygon.cxx 文件级常量 */
	static constexpr double PolygonTol = 1.e-08; // VTK_POLYGON_TOL
	static constexpr int PolygonFailure = -1;    // VTK_POLYGON_FAILURE（退化：法向为零）
	static constexpr int PolygonOutside = 0;     // VTK_POLYGON_OUTSIDE
	static constexpr int PolygonInside = 1;      // VTK_POLYGON_INSIDE

	int GetParametricDimension() const override { return 2; }

	/**
	 * vtkPolygon 未覆写 GetParametricCenter，因此沿用 vtkCell::GetParametricCenter
	 * （三个分量都是 0.5；注意基类 Cell 的默认值是 (0,0,0)，故此处必须显式覆写）。
	 */
	void GetParametricCenter(double pcoords[3]) const override {
		pcoords[0] = pcoords[1] = pcoords[2] = 0.5;
	}

	/** 是否使用均值坐标（MVC）插值：对应 vtkPolygon::UseMVCInterpolation，VTK 默认关闭 */
	void SetUseMVCInterpolation(bool flag) { m_UseMVCInterpolation = flag; }
	bool GetUseMVCInterpolation() const { return m_UseMVCInterpolation; }

	/** vtkMath::Normalize 的复刻：返回原向量长度（长度为 0 时向量不变） */
	static double NormalizeInPlace(double v[3]) {
		const double den = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		if (den != 0.0) {
			const double s = 1.0 / den;
			v[0] *= s;
			v[1] *= s;
			v[2] *= s;
		}
		return den;
	}

	/**
	 * vtkPolygon::ComputeNormal 的复刻（文件内 NormalWorker）：
	 * 以第一个非重合边的起点 P_k 为公共点，累加 (P_i - P_k) × (P_{i+1} - P_k)，
	 * 因此对非凸多边形同样成立。返回是否得到非零法向（false 对应
	 * vtkCellStatus::DegenerateFaces）。
	 */
	static bool ComputeNormal(Points::Pointer points, int numPts, double n[3]) {
		n[0] = n[1] = n[2] = 0.0;
		if (numPts < 3) { return false; }

		double v1[3] = {0.0, 0.0, 0.0};
		int commonPointId = -1;
		int pointId = 0;
		for (; pointId < numPts - 2; ++pointId) {
			const Point& pa = points->GetPoint(pointId);
			const Point& pb = points->GetPoint(pointId + 1);
			v1[0] = static_cast<double>(pb[0]) - static_cast<double>(pa[0]);
			v1[1] = static_cast<double>(pb[1]) - static_cast<double>(pa[1]);
			v1[2] = static_cast<double>(pb[2]) - static_cast<double>(pa[2]);
			if (v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] > 0.0) {
				commonPointId = pointId;
				pointId += 2; // consume the two points we just used to obtain a non-zero v1
				break;
			}
		}
		if (pointId >= numPts || commonPointId < 0) {
			// 所有点重合，或已把所有点用于求 v1 而没有剩余点求 v2
			return false;
		}

		const Point& pc = points->GetPoint(commonPointId);
		const double p0[3] = {static_cast<double>(pc[0]), static_cast<double>(pc[1]),
		                      static_cast<double>(pc[2])};
		double v2[3];
		for (; pointId < numPts; ++pointId) {
			const Point& pd = points->GetPoint(pointId);
			v2[0] = static_cast<double>(pd[0]) - p0[0];
			v2[1] = static_cast<double>(pd[1]) - p0[1];
			v2[2] = static_cast<double>(pd[2]) - p0[2];
			// v1 = v1 × v2
			const double v1x = v1[1] * v2[2] - v1[2] * v2[1];
			const double v1y = v1[2] * v2[0] - v1[0] * v2[2];
			const double v1z = v1[0] * v2[1] - v1[1] * v2[0];
			v1[0] = v1x;
			v1[1] = v1y;
			v1[2] = v1z;
			n[0] += v1[0];
			n[1] += v1[1];
			n[2] += v1[2];
			v1[0] = v2[0];
			v1[1] = v2[1];
			v1[2] = v2[2];
		}
		return NormalizeInPlace(n) != 0.0;
	}

	/**
	 * vtkPolygon::ParameterizePolygon 的逐行复刻：建立局部 s-t 坐标系
	 * （p0 为原点、p10 为 s 轴、p20 为 t 轴、n 为多边形法向），并平移缩放坐标轴使所有
	 * 顶点的 (s,t) 落在 [0,1]² 内。出参 l10/l20 为 s/t 轴的**长度**。
	 * 退化情形（点数 < 3、法向为零、首边长度为零、第二轴长度为零）返回 false。
	 */
	bool ParameterizePolygon(double* p0, double* p10, double& l10, double* p20, double& l20,
	                         double* n) const {
		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
		double p[3], p1[3], p2[3], sbounds[2], tbounds[2];
		double x1[3], x2[3];

		if (numPts < 3) { return false; }

		//  This is a two pass process: first create a p' coordinate system
		//  that is then adjusted to ensure that the polygon points are all in
		//  the range 0<=s,t<=1.  The p' system is defined by the polygon normal,
		//  first vertex and the first edge.
		ComputeNormal(this->m_Points, numPts, n);
		const Point& P0 = this->m_Points->GetPoint(0);
		const Point& P1 = this->m_Points->GetPoint(1);
		for (int i = 0; i < 3; ++i) {
			x1[i] = static_cast<double>(P0[i]);
			x2[i] = static_cast<double>(P1[i]);
			p0[i] = x1[i];
			p10[i] = x2[i] - x1[i];
		}
		// vtkMath::Cross(n, p10, p20)
		p20[0] = n[1] * p10[2] - n[2] * p10[1];
		p20[1] = n[2] * p10[0] - n[0] * p10[2];
		p20[2] = n[0] * p10[1] - n[1] * p10[0];

		// Determine lengths of edges
		l10 = p10[0] * p10[0] + p10[1] * p10[1] + p10[2] * p10[2];
		l20 = p20[0] * p20[0] + p20[1] * p20[1] + p20[2] * p20[2];
		if (l10 == 0.0 || l20 == 0.0) { return false; }

		//  Now evaluate all polygon points to determine min/max parametric
		//  coordinate values.  first vertex has (s,t) = (0,0)
		sbounds[0] = sbounds[1] = 0.0;
		tbounds[0] = tbounds[1] = 0.0;

		for (int i = 1; i < numPts; ++i) {
			const Point& Pi = this->m_Points->GetPoint(i);
			for (int j = 0; j < 3; ++j) { x1[j] = static_cast<double>(Pi[j]); }
			for (int j = 0; j < 3; ++j) { p[j] = x1[j] - p0[j]; }
			const double s = (p[0] * p10[0] + p[1] * p10[1] + p[2] * p10[2]) / l10;
			const double t = (p[0] * p20[0] + p[1] * p20[1] + p[2] * p20[2]) / l20;
			sbounds[0] = (s < sbounds[0] ? s : sbounds[0]);
			sbounds[1] = (s > sbounds[1] ? s : sbounds[1]);
			tbounds[0] = (t < tbounds[0] ? t : tbounds[0]);
			tbounds[1] = (t > tbounds[1] ? t : tbounds[1]);
		}

		//  Re-evaluate coordinate system
		for (int i = 0; i < 3; ++i) {
			p1[i] = p0[i] + sbounds[1] * p10[i] + tbounds[0] * p20[i];
			p2[i] = p0[i] + sbounds[0] * p10[i] + tbounds[1] * p20[i];
			p0[i] = p0[i] + sbounds[0] * p10[i] + tbounds[0] * p20[i];
			p10[i] = p1[i] - p0[i];
			p20[i] = p2[i] - p0[i];
		}
		l10 = std::sqrt(p10[0] * p10[0] + p10[1] * p10[1] + p10[2] * p10[2]);
		l20 = std::sqrt(p20[0] * p20[0] + p20[1] * p20[1] + p20[2] * p20[2]);

		return true;
	}

	/**
	 * vtkLine::DistanceToLine 的 double 版复刻（closestPoint 传 nullptr 的情形）：
	 * t 不夹紧、closest 夹在线段两端，返回**平方**距离。
	 * 与基类 Cell::DistancePointToLine 语义相同，此处用 double 数组避免 float 往返。
	 */
	static double DistanceToLine(const double x[3], const double p1[3], const double p2[3],
	                             double& t) {
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
		const double dx = closestPt[0] - x[0];
		const double dy = closestPt[1] - x[1];
		const double dz = closestPt[2] - x[2];
		return dx * dx + dy * dy + dz * dz;
	}

	/** vtkPolygon.cxx 匿名命名空间里的 PointLocation：点在有向边的左侧为正 */
	static double PointLocation(int axis0, int axis1, const double* p0, const double* p1,
	                            const double* x) {
		return (((p1[axis0] - p0[axis0]) * (x[axis1] - p0[axis1])) -
		        ((x[axis0] - p0[axis0]) * (p1[axis1] - p0[axis1])));
	}

	/**
	 * vtkPolygon::PointInPolygon 的逐行复刻（Dan Sunday 的绕数法，适用于任意 3D 朝向的
	 * 平面多边形）：返回 PolygonInside(1) / PolygonOutside(0) / PolygonFailure(-1)。
	 * bounds 为（已按 PolygonTol 处理过的）包围盒，pts 为 3*numPts 的扁平坐标数组。
	 * 注意 VTK 在此处对 tol2 做了**两次**平方（`tol2 *= tol2`），其量级约 1e-16·尺度⁴，
	 * 使得“与顶点/边重合即判为内部”的提前返回几乎不生效——为保持语义这里原样保留。
	 */
	static int PointInPolygon(const double x[3], int numPts, const double* pts,
	                          const double bounds[6], const double* n) {
		// Do a quick bounds check to throw out trivial cases.
		if (x[0] < bounds[0] || x[0] > bounds[1] || x[1] < bounds[2] || x[1] > bounds[3] ||
		    x[2] < bounds[4] || x[2] > bounds[5]) {
			return PolygonOutside;
		}

		//  Check that the normal is non-zero.
		if (std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) <= FLT_EPSILON) {
			return PolygonFailure;
		}

		// Assess whether the point lies on the boundary of the polygon. Points on
		// the boundary are considered inside the polygon. Need to define a small
		// tolerance relative to the bounding box diagonal length of the polygon.
		double tol2 = PolygonTol * ((bounds[1] - bounds[0]) * (bounds[1] - bounds[0]) +
		                            (bounds[3] - bounds[2]) * (bounds[3] - bounds[2]) +
		                            (bounds[5] - bounds[4]) * (bounds[5] - bounds[4]));
		tol2 *= tol2;
		tol2 = (tol2 == 0.0 ? FLT_EPSILON : tol2);

		for (int i = 0; i < numPts; ++i) {
			// Check coincidence to polygon vertices
			const double* p0 = pts + 3 * i;
			const double dx = x[0] - p0[0];
			const double dy = x[1] - p0[1];
			const double dz = x[2] - p0[2];
			if (dx * dx + dy * dy + dz * dz <= tol2) { return PolygonInside; }

			// Check coincidence to polygon edges
			const double* p1 = pts + 3 * ((i + 1) % numPts);
			double t;
			const double d2 = DistanceToLine(x, p0, p1, t);
			if (d2 <= tol2 && 0.0 < t && t < 1.0) { return PolygonInside; }
		}

		// If here, begin computation of the winding number. This method works for
		// points/polygons arbitrarily oriented in 3D space.  Hence a projection
		// onto one of the x-y-z coordinate planes using the maximum normal
		// component. The computation will be performed in the (axis0,axis1) plane.
		int axis0, axis1;
		if (std::fabs(n[0]) > std::fabs(n[1])) {
			if (std::fabs(n[0]) > std::fabs(n[2])) {
				axis0 = 1;
				axis1 = 2;
			} else {
				axis0 = 0;
				axis1 = 1;
			}
		} else {
			if (std::fabs(n[1]) > std::fabs(n[2])) {
				axis0 = 0;
				axis1 = 2;
			} else {
				axis0 = 0;
				axis1 = 1;
			}
		}

		// Compute the winding number wn. If after processing all polygon edges
		// wn==0, then the point is outside.  Otherwise, the point is inside the
		// polygon. Process all polygon edges determining if there are ascending or
		// descending crossings of the line axis1=constant.
		int wn = 0;
		for (int i = 0; i < numPts; ++i) {
			const double* p0 = pts + 3 * i;
			const double* p1 = pts + 3 * ((i + 1) % numPts);

			if (p0[axis1] <= x[axis1]) {
				if (p1[axis1] > x[axis1]) { // if an upward crossing
					if (PointLocation(axis0, axis1, p0, p1, x) > 0) { ++wn; }
				}
			} else {
				if (p1[axis1] <= x[axis1]) { // if a downward crossing
					if (PointLocation(axis0, axis1, p0, p1, x) < 0) { --wn; }
				}
			}
		}

		// A winding number==0 is outside the polygon
		return ((wn == 0 ? PolygonOutside : PolygonInside));
	}

	/**
	 * vtkPolygon::InterpolateFunctions 的复刻（UseMVCInterpolation == false 的默认路线）。
	 * 注意 VTK 的入参是**物理坐标 x**（不是参数坐标 pcoords）：权重取
	 * 1/|x-P_i|² 再归一化；当 x 恰好落在某个顶点上时该顶点权重为 1、其余为 0
	 * （VTK 的 “exact hit” 分支）。
	 */
	void ComputeInterpolationWeights(const double x[3], double* weights) const {
		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
		double sum = 0.0;
		for (int i = 0; i < numPts; ++i) {
			const Point& P = this->m_Points->GetPoint(i);
			const double dx = static_cast<double>(P[0]) - x[0];
			const double dy = static_cast<double>(P[1]) - x[1];
			const double dz = static_cast<double>(P[2]) - x[2];
			weights[i] = dx * dx + dy * dy + dz * dz;
			if (weights[i] == 0.0) { // exact hit
				for (int j = 0; j < numPts; ++j) { weights[j] = 0.0; }
				weights[i] = 1.0;
				return;
			}
			weights[i] = 1.0 / weights[i];
			sum += weights[i];
		}
		for (int i = 0; i < numPts; ++i) { weights[i] /= sum; }
	}

	/**
	 * vtkPolygon::InterpolateFunctionsUsingMVC 的复刻（UseMVCInterpolation == true 的路线），
	 * 入参同样是物理坐标 x；逐行保留 VTK 的三个特判：
	 *   1) x 与某顶点几乎重合（dist < 1e-8）→ 该顶点权重 1、其余 0；
	 *   2) x 落在某条边上（π-θ < 0.001）→ 该边两端点按距离反比分配；
	 *   3) 权重和近似为 0（|sum| < 1e-8）→ 直接返回（保留已写入的值）。
	 * 与 VTK 的唯一差异：asin 的自变量按 clamp 到 ≤1，避免舍入误差使其略大于 1 而产生 NaN
	 * （数学上 l/2 ≤ 1）；仓库既有的 MeanValueFaceWeights 亦如此处理。
	 */
	void ComputeInterpolationWeightsUsingMVC(const double x[3], double* weights) const {
		constexpr double kPi = 3.14159265358979323846;
		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());

		// Begin by initializing weights.
		for (int i = 0; i < numPts; ++i) { weights[i] = 0.0; }

		// create local array for storing point-to-vertex vectors and distances
		std::vector<double> dist(static_cast<size_t>(numPts));
		std::vector<double> uVec(static_cast<size_t>(3 * numPts));
		static constexpr double eps = 0.00000001;
		for (int i = 0; i < numPts; ++i) {
			const Point& P = this->m_Points->GetPoint(i);
			uVec[3 * i] = static_cast<double>(P[0]) - x[0];
			uVec[3 * i + 1] = static_cast<double>(P[1]) - x[1];
			uVec[3 * i + 2] = static_cast<double>(P[2]) - x[2];
			dist[i] = std::sqrt(uVec[3 * i] * uVec[3 * i] + uVec[3 * i + 1] * uVec[3 * i + 1] +
			                    uVec[3 * i + 2] * uVec[3 * i + 2]);
			// handle special case when the point is really close to a vertex
			if (dist[i] < eps) {
				weights[i] = 1.0;
				return;
			}
			uVec[3 * i] /= dist[i];
			uVec[3 * i + 1] /= dist[i];
			uVec[3 * i + 2] /= dist[i];
		}

		// Now loop over all vertices to compute weight
		// w_i = ( tan(theta_i/2) + tan(theta_(i+1)/2) ) / dist_i
		std::vector<double> tanHalfTheta(static_cast<size_t>(numPts));
		for (int i = 0; i < numPts; ++i) {
			int i1 = i + 1;
			if (i1 == numPts) { i1 = 0; }

			const double* u0 = &uVec[3 * i];
			const double* u1 = &uVec[3 * i1];

			const double l = std::sqrt((u0[0] - u1[0]) * (u0[0] - u1[0]) +
			                           (u0[1] - u1[1]) * (u0[1] - u1[1]) +
			                           (u0[2] - u1[2]) * (u0[2] - u1[2]));
			const double theta = 2.0 * std::asin(l * 0.5 <= 1.0 ? l * 0.5 : 1.0);

			// special case where x lies on an edge
			if (kPi - theta < 0.001) {
				weights[i] = dist[i1] / (dist[i] + dist[i1]);
				weights[i1] = 1 - weights[i];
				return;
			}

			tanHalfTheta[i] = std::tan(theta / 2.0);
		}

		// Normal case
		for (int i = 0; i < numPts; ++i) {
			int i1 = i - 1;
			if (i1 == -1) { i1 = numPts - 1; }
			weights[i] = (tanHalfTheta[i] + tanHalfTheta[i1]) / dist[i];
		}

		// normalize weight
		double sum = 0.0;
		for (int i = 0; i < numPts; ++i) { sum += weights[i]; }
		if (std::fabs(sum) < eps) { return; }
		for (int i = 0; i < numPts; ++i) { weights[i] /= sum; }
	}

	/**
	 * 基类签名要求由 pcoords 给出权重，但 vtkPolygon 的权重函数
	 * （InterpolateFunctions / InterpolateFunctionsUsingMVC）以**物理坐标**为入参。
	 * 这里按 EvaluateLocation 的方式先把 pcoords 映射回物理点，再调用对应权重函数；
	 * EvaluatePosition / EvaluateLocation 内部则直接调用物理坐标版本，与 VTK 完全一致。
	 * 参数化失败（退化多边形）时给出中性权重 (1,0,…,0)。
	 */
	void InterpolationFunctions(const double pcoords[3], double* weights) const override {
		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
		double p0[3], p10[3], l10, p20[3], l20, n[3];
		if (numPts < 3 || !this->ParameterizePolygon(p0, p10, l10, p20, l20, n)) {
			for (int i = 0; i < numPts; ++i) { weights[i] = 0.0; }
			if (numPts > 0) { weights[0] = 1.0; }
			return;
		}
		const double x[3] = {p0[0] + pcoords[0] * p10[0] + pcoords[1] * p20[0],
		                     p0[1] + pcoords[0] * p10[1] + pcoords[1] * p20[1],
		                     p0[2] + pcoords[0] * p10[2] + pcoords[1] * p20[2]};
		if (m_UseMVCInterpolation) {
			this->ComputeInterpolationWeightsUsingMVC(x, weights);
		} else {
			this->ComputeInterpolationWeights(x, weights);
		}
	}

	/**
	 * vtkPolygon 没有 InterpolationDerivs（其导数由 vtkPolygon::Derivatives 借助面法向与
	 * 局部 s-t 轴解析给出，不经过形函数导数）。这里按基类约定清零，避免调用方读到未初始化值。
	 */
	void InterpolationDerivs(const double*, double* derivs) const override {
		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
		for (int i = 0; i < 3 * numPts; ++i) { derivs[i] = 0.0; }
	}

	/**
	 * vtkPolygon::EvaluateLocation 的逐行复刻：
	 *   x = p0 + pcoords[0]·p10 + pcoords[1]·p20（线性外插，pcoords 可超出 [0,1]），
	 *   随后在该物理点上求插值权重（VTK 原文顺序）。
	 * 参数化失败（退化多边形）时给出中性输出：x = P0、weights = (1,0,…,0)。
	 */
	void EvaluateLocation(int& subId, const double pcoords[3], Point& x,
	                      double* weights) const override {
		subId = 0;
		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
		double p0[3], p10[3], l10, p20[3], l20, n[3];
		if (numPts < 3 || !this->ParameterizePolygon(p0, p10, l10, p20, l20, n)) {
			for (int i = 0; i < numPts; ++i) { weights[i] = 0.0; }
			if (numPts > 0) {
				weights[0] = 1.0;
				x = this->m_Points->GetPoint(0);
			} else {
				x = Point(0.0f, 0.0f, 0.0f);
			}
			return;
		}
		double xd[3];
		for (int i = 0; i < 3; ++i) { xd[i] = p0[i] + pcoords[0] * p10[i] + pcoords[1] * p20[i]; }
		x = Point(static_cast<float>(xd[0]), static_cast<float>(xd[1]), static_cast<float>(xd[2]));

		if (m_UseMVCInterpolation) {
			this->ComputeInterpolationWeightsUsingMVC(xd, weights);
		} else {
			this->ComputeInterpolationWeights(xd, weights);
		}
	}

	/**
	 * vtkPolygon::EvaluatePosition 的逐行复刻（“先把点投影到多边形平面 → 参数坐标 +
	 * 绕数包含判定 → 权重”的路线）：
	 *   - 先用**查询点物理坐标**求权重（VTK 原文 InterpolateFunctions(x, weights)），
	 *     weights 为 1/r² 归一化（或 MVC，取决于 UseMVCInterpolation）；
	 *   - ParameterizePolygon 建立局部 s-t 系，vtkPlane::ProjectPoint 把 x 投到多边形平面，
	 *     再投影得到 pcoords（pcoords[2] 恒为 0，允许超出 [0,1]）；
	 *   - 包围盒按 vtkBoundingBox::InflateSlice(PolygonTol) 处理后用于 PointInPolygon；
	 *   - 只有 “pcoords 两分量 ∈ [0,1] 且 PointInPolygon == PolygonInside” 才返回 1，
	 *     此时 closestPoint = 平面投影点 cp、dist2 = |x-cp|²（**不是** 0）；
	 *   - 否则返回 0，并在 numPts 条边上取最近点（DistanceToLine，顺序与 VTK 一致，
	 *     minDist2 初值为 DBL_MAX，即“边界最近点”语义）；
	 *   - 退化多边形（ParameterizePolygon 失败）在 VTK 中会使用未初始化的 p0/l10（未定义
	 *     行为）：这里显式给出中性输出（pcoords=(0,0,0)、weights 已在前面算好、
	 *     closestPoint=P0、dist2=|x-P0|²）并返回 -1。
	 * 无论返回 0 还是 1，pcoords / weights / closestPoint / dist2 都会被填写。
	 */
	int EvaluatePosition(const Point& x, Point& closestPoint, int& subId, double pcoords[3],
	                     double& minDist2, double* weights) const override {
		subId = 0;
		pcoords[2] = 0.0;

		const int numPts = static_cast<int>(this->m_Points->GetNumberOfPoints());
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};

		// 扁平 double 点表（对应 vtkPolygon 里 Points->GetData() 的 double 视图）
		std::vector<double> ptsd(static_cast<size_t>(3 * numPts));
		for (int i = 0; i < numPts; ++i) {
			const Point& P = this->m_Points->GetPoint(i);
			ptsd[3 * i] = static_cast<double>(P[0]);
			ptsd[3 * i + 1] = static_cast<double>(P[1]);
			ptsd[3 * i + 2] = static_cast<double>(P[2]);
		}

		// VTK 原文顺序：先算权重（用查询点，不是投影点）
		if (m_UseMVCInterpolation) {
			this->ComputeInterpolationWeightsUsingMVC(xd, weights);
		} else {
			this->ComputeInterpolationWeights(xd, weights);
		}

		double p0[3], p10[3], l10, p20[3], l20, n[3];
		if (numPts < 3 || !this->ParameterizePolygon(p0, p10, l10, p20, l20, n)) {
			pcoords[0] = pcoords[1] = 0.0;
			if (numPts > 0) {
				closestPoint = this->m_Points->GetPoint(0);
				const double dx = ptsd[0] - xd[0];
				const double dy = ptsd[1] - xd[1];
				const double dz = ptsd[2] - xd[2];
				minDist2 = dx * dx + dy * dy + dz * dz;
			} else {
				closestPoint = Point(0.0f, 0.0f, 0.0f);
				minDist2 = 0.0;
			}
			return -1;
		}

		// vtkPlane::ProjectPoint(x, p0, n, cp)：t = n·(x-p0)，cp = x - t·n
		const double t0 = n[0] * (xd[0] - p0[0]) + n[1] * (xd[1] - p0[1]) + n[2] * (xd[2] - p0[2]);
		const double cp[3] = {xd[0] - t0 * n[0], xd[1] - t0 * n[1], xd[2] - t0 * n[2]};

		const double ray[3] = {cp[0] - p0[0], cp[1] - p0[1], cp[2] - p0[2]};
		pcoords[0] = (ray[0] * p10[0] + ray[1] * p10[1] + ray[2] * p10[2]) / (l10 * l10);
		pcoords[1] = (ray[0] * p20[0] + ray[1] * p20[1] + ray[2] * p20[2]) / (l20 * l20);

		// Make sure that the bounding box has non-zero volume, so that all bounding
		// box sides have non-zero thickness. This prevents tolerancing issues when
		// the polygon lies exactly on a coordinate plane.
		double bounds[6] = {DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX};
		for (int i = 0; i < numPts; ++i) {
			for (int j = 0; j < 3; ++j) {
				const double v = ptsd[3 * i + j];
				if (v < bounds[2 * j]) { bounds[2 * j] = v; }
				if (v > bounds[2 * j + 1]) { bounds[2 * j + 1] = v; }
			}
		}
		// vtkBoundingBox::InflateSlice(delta)：某轴跨度 < 2·delta 时把该轴两侧各扩 delta
		for (int j = 0; j < 3; ++j) {
			if (bounds[2 * j + 1] - bounds[2 * j] < 2.0 * PolygonTol) {
				bounds[2 * j] -= PolygonTol;
				bounds[2 * j + 1] += PolygonTol;
			}
		}

		if (pcoords[0] >= 0.0 && pcoords[0] <= 1.0 && pcoords[1] >= 0.0 && pcoords[1] <= 1.0 &&
		    PointInPolygon(cp, numPts, ptsd.data(), bounds, n) == PolygonInside) {
			closestPoint = Point(static_cast<float>(cp[0]), static_cast<float>(cp[1]),
			                     static_cast<float>(cp[2]));
			const double dx = cp[0] - xd[0];
			const double dy = cp[1] - xd[1];
			const double dz = cp[2] - xd[2];
			minDist2 = dx * dx + dy * dy + dz * dz;
			return 1;
		}

		// If here, point is outside of polygon, so need to find distance to boundary
		minDist2 = DBL_MAX;
		double t = 0.0;
		Point closest;
		for (int i = 0; i < numPts; ++i) {
			const Point& p1 = this->m_Points->GetPoint(i);
			const Point& p2 = this->m_Points->GetPoint((i + 1) % numPts);
			const double d2 = Cell::DistancePointToLine(x, p1, p2, t, closest);
			if (d2 < minDist2) {
				closestPoint = closest;
				minDist2 = d2;
			}
		}
		return 0;
	}

protected:
	Polygon() = default;
	~Polygon() override = default;

	/** 对应 vtkPolygon::UseMVCInterpolation，VTK 默认 false（即 1/r² 反距离权重） */
	bool m_UseMVCInterpolation = false;
};
IGAME_NAMESPACE_END
#endif