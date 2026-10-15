#ifndef iGameCell_h
#define iGameCell_h

#include "iGameCellType.h"
#include "iGameIdArray.h"
#include "iGameObject.h"
#include "iGamePoints.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <utility>
#define IGAME_CELL_MAX_SIZE 256

IGAME_NAMESPACE_BEGIN

/**
 * VTK 参数域容差：等价 vtkCell 各高阶子类 EvaluatePosition 末尾的
 * “pcoords ∈ [-0.001, 1+0.001] 即判为单元内”约定。
 */
inline constexpr double kVTKParametricTol = 1e-3;

/** 通用 Newton 反解支持的形函数个数上限（覆盖 VTK 标准单元，最大 27 节点三二次六面体） */
inline constexpr int kVTKMaxShapePoints = 64;

class Cell : public Object {
public:
	I_OBJECT(Cell);

	virtual IGenum GetCellType() const noexcept = 0;
	virtual int GetCellSize() const noexcept {
		return this->m_PointIds->GetNumberOfIds();
	}

    virtual int GetNumberOfPoints() { return this->m_PointIds->GetNumberOfIds(); }
    virtual int GetNumberOfEdges() = 0;
    virtual int GetNumberOfFaces() = 0;

	// return   position and id of the i th point in cell
    igIndex GetPointId(const int id) { return this->m_PointIds->GetId(id); }
    Point& GetPoint(const int id) { return this->m_Points->GetPoint(id); }
    const Point& GetPoint(const int id) const {
        return this->m_Points->GetPoint(id);
    }

	virtual Cell* GetEdge(const int edgeId) = 0;
	virtual Cell* GetFace(const int faceId) = 0;

	virtual void Reset() {
		this->m_PointIds->Reset();
		this->m_Points->Reset();
	}
	static igIndex GetCellDimension(igIndex celltype) {
		return GetCellDimension(IGCellType(celltype));
	}
	static igIndex GetCellDimension(IGCellType celltype) {
		switch (celltype) {
		case IG_EMPTY_CELL:
			return 0;
		case IG_VERTEX:
		case IG_LINE:
			return 1;
		case IG_FACE:
		case IG_TRIANGLE:
		case IG_QUAD:
		case IG_POLYGON:
			return 2;
		case IG_VOLUME:
		case IG_POLYHEDRON:
		case IG_TETRA:
		case IG_HEXAHEDRON:
		case IG_PYRAMID:
		case IG_PRISM:
			return 3;
		case IG_QUADRATIC_EDGE:
		case IG_QUADRATIC_TRIANGLE:
		case IG_QUADRATIC_QUAD:
		case IG_QUADRATIC_POLYGON:
		case IG_QUADRATIC_TETRA:
		case IG_QUADRATIC_HEXAHEDRON:
		case IG_QUADRATIC_PRISM:
		case IG_QUADRATIC_PYRAMID:
		case IG_BIQUADRATIC_QUAD:
		case IG_TRIQUADRATIC_HEXAHEDRON:
		case IG_TRIQUADRATIC_PYRAMID:
		case IG_QUADRATIC_LINEAR_QUAD:
		case IG_QUADRATIC_LINEAR_WEDGE:
		case IG_BIQUADRATIC_QUADRATIC_WEDGE:
		case IG_BIQUADRATIC_QUADRATIC_HEXAHEDRON:
		case IG_BIQUADRATIC_TRIANGLE:
			return 4;
		default:
			return 0;
			break;
		}
	}

	Points::Pointer m_Points{};
	IdArray::Pointer m_PointIds{};

	/* ==================================================================
	 * VTK 风格单元插值 / 参数坐标接口（对照 vtkCell 及各 vtkXxx 子类复刻）
	 *
	 * 约定（与 VTK 一致）：
	 *   - pcoords 为参数坐标，维数由 GetParametricDimension() 给出（0/1/2/3），
	 *     参数域为 [0,1]^dim；
	 *   - InterpolationFunctions(pcoords, weights) 等价 vtkXxx::InterpolationFunctions：
	 *     由参数坐标求形函数值，允许 pcoords 落在 [0,1] 之外（外插）；
	 *   - InterpolationDerivs(pcoords, derivs) 等价 vtkXxx::InterpolationDerivs，
	 *     布局为 [∂N_i/∂r (i=0..n-1), ∂N_i/∂s (i=0..n-1), ∂N_i/∂t (i=0..n-1)]；
	 *   - EvaluateLocation 等价 vtkXxx::EvaluateLocation：x = Σ w_i · P_i；
	 *   - EvaluatePosition 等价 vtkXxx::EvaluatePosition：
	 *       返回 1 = 点在单元内（VTK 的“垂直落点/参数坐标落在参数域内”判定），
	 *       返回 0 = 点不在单元内，-1 = 退化单元（VTK 亦有此返回值）；
	 *       **两种结果都会**填写 closestPoint（单元上离 x 最近的点）、pcoords（可超出
	 *       [0,1]）、dist2（x 到 closestPoint 的平方距离）与 weights（pcoords 处形函数）。
	 *       注意：VTK 的“在单元内”**不含**距离容差——离平面很远的点也算落在参数域内；
	 *       距离容差由调用方（FindCell/探针内核）用 dist2 ≤ tol2 判定。
	 * ================================================================== */

	/** 参数域维数（等价 vtkCell::GetCellDimension 的语义，顶点为 0） */
	virtual int GetParametricDimension() const {
		return static_cast<int>(GetCellDimension(this->GetCellType()));
	}

	/** 参数中心（等价 vtkXxx::GetParametricCenter），默认取原点 */
	virtual void GetParametricCenter(double pcoords[3]) const {
		pcoords[0] = 0.0;
		pcoords[1] = 0.0;
		pcoords[2] = 0.0;
	}

	/** 形函数：等价 vtkXxx::InterpolationFunctions（未实现的类型清零） */
	virtual void InterpolationFunctions(const double pcoords[3], double* weights) const {
		(void)pcoords;
		const int n = this->GetCellSize();
		for (int i = 0; i < n; ++i) { weights[i] = 0.0; }
	}

	/** 形函数导数：等价 vtkXxx::InterpolationDerivs（未实现的类型清零） */
	virtual void InterpolationDerivs(const double pcoords[3], double* derivs) const {
		(void)pcoords;
		const int n = this->GetCellSize();
		for (int i = 0; i < 3 * n; ++i) { derivs[i] = 0.0; }
	}

	/**
	 * 参数域判定（等价 vtkXxx::IsInside 的默认实现）：pcoords 各分量落在
	 * [-tol, 1+tol] 内即视为在单元内。线性单元可用更严格/更宽松的覆盖。
	 */
	virtual int IsInsideParametricDomain(const double pcoords[3], double tol = kVTKParametricTol) const {
		const int dim = this->GetParametricDimension();
		for (int i = 0; i < dim; ++i) {
			if (pcoords[i] < -tol || pcoords[i] > 1.0 + tol) { return 0; }
		}
		return 1;
	}

	/** 由参数坐标求物理坐标与权重（等价 vtkXxx::EvaluateLocation，x = Σ w_i P_i） */
	virtual void EvaluateLocation(int& subId, const double pcoords[3], Point& x, double* weights) const {
		subId = 0;
		const int n = this->GetCellSize();
		this->InterpolationFunctions(pcoords, weights);
		double s[3] = {0.0, 0.0, 0.0};
		for (int i = 0; i < n; ++i) {
			const Point& p = this->GetPoint(i);
			const double w = weights[i];
			s[0] += w * static_cast<double>(p[0]);
			s[1] += w * static_cast<double>(p[1]);
			s[2] += w * static_cast<double>(p[2]);
		}
		x = Point(static_cast<float>(s[0]), static_cast<float>(s[1]), static_cast<float>(s[2]));
	}

	/**
	 * 参数坐标反解（等价 vtkXxx::EvaluatePosition）。基类默认实现：不支持的类型
	 * 返回 0 并把 pcoords/weights/closestPoint/dist2 置零；各单元类型覆写。
	 */
	virtual int EvaluatePosition(const Point& x, Point& closestPoint, int& subId,
	                             double pcoords[3], double& dist2, double* weights) const {
		(void)x;
		subId = 0;
		pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
		dist2 = 0.0;
		const int n = this->GetCellSize();
		for (int i = 0; i < n; ++i) { weights[i] = 0.0; }
		return 0;
	}

	/**
	 * 通用 Newton 反解：供已实现 InterpolationFunctions/InterpolationDerivs 的单元
	 * 复用（等价 VTK 高阶单元用形函数 + 雅可比迭代求 pcoords 的实现）。
	 * 收敛后用 [-kVTKParametricTol, 1+kVTKParametricTol] 判定参数域。
	 */
	/* ---------------- VTK 风格 Lagrange 插值辅助 ----------------
	 * 逐行对照 vtkLagrangeInterpolation / vtkHigherOrderInterpolation：
	 * 参考节点等距，且 1D 基的索引顺序是「两端点在前、内部点在后」（等价 Tensor1ShapeFunctions）。
	 * 注意：VTK 的 Tensor1ShapeDerivatives 未做同样的重排（与自身形函数不自洽），这里为保证
	 * Newton 反解的一致性对导数做**与形函数相同**的重排（数学上正确）。
	 */
	static void LagrangeShape1D(int order, double t, double* shape) {
		const double v = order * t;
		for (int j = 0; j <= order; ++j) {
			shape[j] = 1.0;
			for (int k = 0; k <= order; ++k) {
				if (j != k) { shape[j] *= (v - k) / (j - k); }
			}
		}
	}

	static void LagrangeShapeAndGradient1D(int order, double t, double* shape, double* derivs) {
		const double v = order * t;
		for (int j = 0; j <= order; ++j) {
			shape[j] = 1.0;
			derivs[j] = 0.0;
			for (int k = 0; k <= order; ++k) {
				if (j == k) { continue; }
				shape[j] *= (v - k) / (j - k);
				double dtmp = 1.0;
				for (int q = 0; q <= order; ++q) {
					if (q == j) { continue; }
					dtmp *= (q == k ? 1.0 : (v - q)) / (j - q);
				}
				derivs[j] += order * dtmp;
			}
		}
	}

	/** 1D（曲线）：等价 Tensor1ShapeFunctions，形函数与导数一起重排 */
	static int LagrangeTensor1(int order, double t, double* shape, double* derivs) {
		double ll[64], dd[64];
		if (order < 1 || order > 63) { return 0; }
		LagrangeShapeAndGradient1D(order, t, ll, dd);
		int sn = 0;
		shape[sn] = ll[0];
		derivs[sn] = dd[0];
		++sn;
		shape[sn] = ll[order];
		derivs[sn] = dd[order];
		++sn;
		for (int i = 1; i < order; ++i) {
			shape[sn] = ll[i];
			derivs[sn] = dd[i];
			++sn;
		}
		return sn;
	}

	/**
	 * 2D 四边形：等价 Tensor2ShapeFunctions / Tensor2ShapeDerivatives，节点序为
	 * 「4 角点 → 各棱内部点（0-1、1-2、2-3、3-0）→ 面内部点」。
	 * 导数输出为本仓库布局：前 n 项 ∂N/∂r，后 n 项 ∂N/∂s。
	 */
	static int LagrangeTensor2(int orderR, int orderS, double r, double s, double* shape, double* derivs) {
		if (orderR < 1 || orderS < 1 || orderR > 15 || orderS > 15) { return 0; }
		double ll0[16], ll1[16], dd0[16], dd1[16];
		LagrangeShapeAndGradient1D(orderR, r, ll0, dd0);
		LagrangeShapeAndGradient1D(orderS, s, ll1, dd1);

		const bool wantDerivs = (derivs != nullptr);
		double dv[512];
		const auto put = [&](int pos, int i0, int i1) {
			shape[pos] = ll0[i0] * ll1[i1];
			if (!wantDerivs) { return; }
			dv[2 * pos] = dd0[i0] * ll1[i1];
			dv[2 * pos + 1] = ll0[i0] * dd1[i1];
		};

		put(0, 0, 0);
		put(1, orderR, 0);
		put(2, orderR, orderS);
		put(3, 0, orderS);

		int sn = 4;
		int sn1 = 4 + orderR + orderS - 2;
		for (int i = 1; i < orderR; ++i) {
			put(sn++, i, 0);		// 棱 0-1
			put(sn1++, i, orderS);  // 棱 2-3
		}
		for (int i = 1; i < orderS; ++i) {
			put(sn++, orderR, i);   // 棱 1-2
			put(sn1++, 0, i);	   // 棱 3-0
		}
		sn = sn1;
		for (int i = 1; i < orderS; ++i) {
			for (int j = 1; j < orderR; ++j) { put(sn++, j, i); }
		}
		const int n = sn;
		if (wantDerivs) {
			for (int i = 0; i < n; ++i) {
				derivs[i] = dv[2 * i];
				derivs[n + i] = dv[2 * i + 1];
			}
		}
		return n;
	}

	/**
	 * 3D 六面体：等价 Tensor3ShapeFunctions / Tensor3ShapeDerivatives（节点序：
	 * 8 角点 → 12 棱内部点 → 6 面内部点 → 体内部点）。
	 * 导数输出为本仓库布局：前 n 项 ∂N/∂r，接着 n 项 ∂N/∂s，最后 n 项 ∂N/∂t。
	 */
	static int LagrangeTensor3(int orderR, int orderS, int orderT, double r, double s, double t,
	                          double* shape, double* derivs) {
		if (orderR < 1 || orderS < 1 || orderT < 1 || orderR > 9 || orderS > 9 || orderT > 9) { return 0; }
		double ll0[10], ll1[10], ll2[10], dd0[10], dd1[10], dd2[10];
		LagrangeShapeAndGradient1D(orderR, r, ll0, dd0);
		LagrangeShapeAndGradient1D(orderS, s, ll1, dd1);
		LagrangeShapeAndGradient1D(orderT, t, ll2, dd2);

		const bool wantDerivs = (derivs != nullptr);
		double dv[3072];
		const auto put = [&](int pos, int i0, int i1, int i2) {
			shape[pos] = ll0[i0] * ll1[i1] * ll2[i2];
			if (!wantDerivs) { return; }
			dv[3 * pos] = dd0[i0] * ll1[i1] * ll2[i2];
			dv[3 * pos + 1] = ll0[i0] * dd1[i1] * ll2[i2];
			dv[3 * pos + 2] = ll0[i0] * ll1[i1] * dd2[i2];
		};

		int sn = 0;
		put(sn++, 0, 0, 0);
		put(sn++, orderR, 0, 0);
		put(sn++, orderR, orderS, 0);
		put(sn++, 0, orderS, 0);
		put(sn++, 0, 0, orderT);
		put(sn++, orderR, 0, orderT);
		put(sn++, orderR, orderS, orderT);
		put(sn++, 0, orderS, orderT);

		int sn1 = sn + (orderR + orderS - 2);
		int sn2 = sn1 + (orderR + orderS - 2);
		int sn3 = sn2 + (orderR + orderS - 2);
		for (int i = 1; i < orderR; ++i) {
			put(sn++, i, 0, 0);          // 棱 0-1
			put(sn1++, i, orderS, 0);    // 棱 2-3
			put(sn2++, i, 0, orderT);    // 棱 4-5
			put(sn3++, i, orderS, orderT);// 棱 6-7
		}
		for (int i = 1; i < orderS; ++i) {
			put(sn++, orderR, i, 0);          // 棱 1-2
			put(sn1++, 0, i, 0);              // 棱 3-0
			put(sn2++, orderR, i, orderT);    // 棱 5-6
			put(sn3++, 0, i, orderT);         // 棱 7-4
		}
		sn = sn3;
		sn1 = sn + (orderT - 1);
		sn2 = sn1 + (orderT - 1);
		sn3 = sn2 + (orderT - 1);
		for (int i = 1; i < orderT; ++i) {
			put(sn++, 0, 0, i);            // 棱 0-4
			put(sn1++, orderR, 0, i);      // 棱 1-5
			put(sn2++, orderR, orderS, i); // 棱 2-6
			put(sn3++, 0, orderS, i);      // 棱 3-7
		}
		sn = sn3;
		const int facePerT = (orderS - 1) * (orderT - 1);
		sn1 = sn + facePerT;
		sn2 = sn1 + facePerT;
		sn3 = sn2 + facePerT;
		for (int i = 1; i < orderT; ++i) {
			for (int j = 1; j < orderS; ++j) {
				put(sn++, 0, j, i);          // 面 0-4-7-3
				put(sn1++, orderR, j, i);    // 面 1-2-6-5
			}
			for (int j = 1; j < orderR; ++j) {
				put(sn2++, j, 0, i);         // 面 0-1-5-4
				put(sn3++, j, orderS, i);    // 面 2-3-7-6
			}
		}
		sn = sn3;
		sn1 = sn + (orderR - 1) * (orderS - 1);
		for (int i = 1; i < orderS; ++i) {
			for (int j = 1; j < orderR; ++j) {
				put(sn++, j, i, 0);          // 面 0-1-2-3
				put(sn1++, j, i, orderT);    // 面 4-7-6-5
			}
		}
		sn = sn1;
		for (int k = 1; k < orderT; ++k) {
			for (int j = 1; j < orderS; ++j) {
				for (int i = 1; i < orderR; ++i) { put(sn++, i, j, k); }
			}
		}
		const int n = sn;
		if (wantDerivs) {
			for (int i = 0; i < n; ++i) {
				derivs[i] = dv[3 * i];
				derivs[n + i] = dv[3 * i + 1];
				derivs[2 * n + i] = dv[3 * i + 2];
			}
		}
		return n;
	}

	/** 单纯形（三角形 / 四面体）基函数：等价 vtkHigherOrderTriangle::Eta */
	static double SimplexEta(int n, int chi, double sigma) {
		double result = 1.0;
		for (int i = 1; i <= chi; ++i) { result *= (n * sigma - i + 1.0) / i; }
		return result;
	}

	/** 单纯形基函数导数：等价 vtkHigherOrderTriangle::Deta */
	static double SimplexDeta(int n, int chi, double sigma) {
		if (chi == 0) { return 0.0; }
		const double chiD = static_cast<double>(chi);
		return (static_cast<double>(n) / chiD) * SimplexEta(n, chi - 1, sigma) +
		       ((n * sigma - chiD + 1.0) / chiD) * SimplexDeta(n, chi - 1, sigma);
	}

	/** 三角形重心索引：等价 vtkHigherOrderTriangle::BarycentricIndex（index → 三重指数） */
	static void SimplexBarycentricIndex(int index, int* bindex, int order) {
		int maxV = order;
		int minV = 0;
		int ord = order;
		while (index != 0 && index >= 3 * ord) {
			index -= 3 * ord;
			maxV -= 2;
			++minV;
			ord -= 3;
		}
		if (index < 3) {
			bindex[index] = minV;
			bindex[(index + 1) % 3] = minV;
			bindex[(index + 2) % 3] = maxV;
		} else {
			index -= 3;
			const int dim = index / (ord - 1);
			const int offset = index - dim * (ord - 1);
			bindex[(dim + 1) % 3] = minV;
			bindex[(dim + 2) % 3] = (maxV - 1) - offset;
			bindex[dim] = (minV + 1) + offset;
		}
	}

	/** 三角形：由点数反推阶数（n = (p+1)(p+2)/2） */
	static int TriangleOrderFromPointCount(int n) {
		for (int p = 1; p <= 12; ++p) {
			if ((p + 1) * (p + 2) / 2 == n) { return p; }
		}
		return -1;
	}

	/** 六面体：由点数反推阶数（n = (p+1)³） */
	static int HexOrderFromPointCount(int n) {
		for (int p = 1; p <= 9; ++p) {
			if ((p + 1) * (p + 1) * (p + 1) == n) { return p; }
		}
		return -1;
	}

	/** 四边形：由点数反推阶数（n = (p+1)²，等价 VTK 的 GetNumberOfPoints 反推） */
	static int QuadOrderFromPointCount(int n) {
		for (int p = 1; p <= 15; ++p) {
			if ((p + 1) * (p + 1) == n) { return p; }
		}
		return -1;
	}

	int EvaluatePositionNewton(const Point& x, Point& closestPoint, int& subId,
	                          double pcoords[3], double& dist2, double* weights) const {
		subId = 0;
		const int dim = this->GetParametricDimension();
		const int n = this->GetCellSize();
		if (dim <= 0 || dim > 3 || n <= 0 || n > kVTKMaxShapePoints || weights == nullptr) {
			pcoords[0] = pcoords[1] = pcoords[2] = 0.0;
			dist2 = 0.0;
			for (int i = 0; i < n; ++i) { weights[i] = 0.0; }
			return 0;
		}

		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};
		double w[kVTKMaxShapePoints];
		double derivs[3 * kVTKMaxShapePoints];
		double pc[3] = {0.0, 0.0, 0.0};
		this->GetParametricCenter(pc);

		constexpr int kMaxIter = 20;
		for (int iter = 0; iter < kMaxIter; ++iter) {
			this->InterpolationFunctions(pc, w);
			this->InterpolationDerivs(pc, derivs);

			double xc[3] = {0.0, 0.0, 0.0};
			for (int i = 0; i < n; ++i) {
				const Point& p = this->GetPoint(i);
				xc[0] += w[i] * static_cast<double>(p[0]);
				xc[1] += w[i] * static_cast<double>(p[1]);
				xc[2] += w[i] * static_cast<double>(p[2]);
			}
			const double f[3] = {xd[0] - xc[0], xd[1] - xc[1], xd[2] - xc[2]};

			// 雅可比 J[k][j] = Σ_i P_i[k] · ∂N_i/∂ξ_j
			double J[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
			for (int j = 0; j < dim; ++j) {
				for (int i = 0; i < n; ++i) {
					const double d = derivs[j * n + i];
					if (d == 0.0) { continue; }
					const Point& p = this->GetPoint(i);
					J[0][j] += d * static_cast<double>(p[0]);
					J[1][j] += d * static_cast<double>(p[1]);
					J[2][j] += d * static_cast<double>(p[2]);
				}
			}

			// 最小二乘正规方程 (JᵀJ)·δ = Jᵀ·f
			double A[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
			double b[3] = {0.0, 0.0, 0.0};
			for (int a = 0; a < dim; ++a) {
				for (int c = 0; c < dim; ++c) {
					double s = 0.0;
					for (int k = 0; k < 3; ++k) { s += J[k][a] * J[k][c]; }
					A[a][c] = s;
				}
				double s = 0.0;
				for (int k = 0; k < 3; ++k) { s += J[k][a] * f[k]; }
				b[a] = s;
			}
			if (!SolveSmallSystem(A, b, dim)) { break; }

			double step2 = 0.0;
			for (int a = 0; a < dim; ++a) {
				pc[a] += b[a];
				step2 += b[a] * b[a];
			}
			if (step2 <= 1e-24) { break; }
		}

		// 收敛后统一填写输出（与 VTK 一致：即使判定为外也填 pcoords/weights/closestPoint/dist2）
		this->InterpolationFunctions(pc, w);
		for (int i = 0; i < n; ++i) { weights[i] = w[i]; }
		double xc[3] = {0.0, 0.0, 0.0};
		for (int i = 0; i < n; ++i) {
			const Point& p = this->GetPoint(i);
			xc[0] += w[i] * static_cast<double>(p[0]);
			xc[1] += w[i] * static_cast<double>(p[1]);
			xc[2] += w[i] * static_cast<double>(p[2]);
		}
		closestPoint = Point(static_cast<float>(xc[0]), static_cast<float>(xc[1]),
		                     static_cast<float>(xc[2]));
		const double dx = xd[0] - xc[0];
		const double dy = xd[1] - xc[1];
		const double dz = xd[2] - xc[2];
		dist2 = dx * dx + dy * dy + dz * dz;
		pcoords[0] = pc[0];
		pcoords[1] = pc[1];
		pcoords[2] = pc[2];
		return this->IsInsideParametricDomain(pc, kVTKParametricTol);
	}

	/** dim×dim 小线性方程组求解（列主元高斯消元），b 原地变为解 */
	static bool SolveSmallSystem(double A[3][3], double b[3], int dim) {
		for (int col = 0; col < dim; ++col) {
			int pivot = col;
			double best = std::fabs(A[col][col]);
			for (int r = col + 1; r < dim; ++r) {
				const double v = std::fabs(A[r][col]);
				if (v > best) { best = v; pivot = r; }
			}
			if (best <= 1e-30) { return false; }
			if (pivot != col) {
				for (int c = 0; c < dim; ++c) { std::swap(A[col][c], A[pivot][c]); }
				std::swap(b[col], b[pivot]);
			}
			const double inv = 1.0 / A[col][col];
			for (int r = col + 1; r < dim; ++r) {
				const double factor = A[r][col] * inv;
				if (factor == 0.0) { continue; }
				for (int c = col; c < dim; ++c) { A[r][c] -= factor * A[col][c]; }
				b[r] -= factor * b[col];
			}
		}
		for (int r = dim - 1; r >= 0; --r) {
			double s = b[r];
			for (int c = r + 1; c < dim; ++c) { s -= A[r][c] * b[c]; }
			b[r] = s / A[r][r];
		}
		return true;
	}

	/**
	 * 点到线段/直线的最近点（逐行复刻 vtkLine::DistanceToLine）：
	 *   t 返回参数位置，**不做夹紧**（VTK 语义：t 可 <0 或 >1）；
	 *   closest 返回线段上的最近点（已夹到两端点）；
	 *   返回 x 到 closest 的平方距离。
	 */
	static double DistancePointToLine(const Point& x, const Point& p1, const Point& p2,
	                                  double& t, Point& closest) {
		const double p21[3] = {static_cast<double>(p2[0]) - static_cast<double>(p1[0]),
		                       static_cast<double>(p2[1]) - static_cast<double>(p1[1]),
		                       static_cast<double>(p2[2]) - static_cast<double>(p1[2])};
		const double xd[3] = {static_cast<double>(x[0]), static_cast<double>(x[1]),
		                      static_cast<double>(x[2])};
		const double p1d[3] = {static_cast<double>(p1[0]), static_cast<double>(p1[1]),
		                       static_cast<double>(p1[2])};

		double closestPt[3] = {p1d[0], p1d[1], p1d[2]};
		const double num = p21[0] * (xd[0] - p1d[0]) + p21[1] * (xd[1] - p1d[1]) +
		                   p21[2] * (xd[2] - p1d[2]);
		if (num == 0.0) {
			t = 0.0;
		} else {
			const double denom = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];
			double tolerance = 1.0e-8 * num; // VTK_TOL
			if (tolerance < 0.0) { tolerance = -tolerance; }
			if (denom < tolerance) { // 端点退化：VTK 用极值 t 表达“落在端点上”
				if (num > 0.0) {
					closestPt[0] = p1d[0] + p21[0];
					closestPt[1] = p1d[1] + p21[1];
					closestPt[2] = p1d[2] + p21[2];
					t = 1.0e300;
				} else {
					t = -1.0e300;
				}
			} else if ((t = num / denom) < 0.0) {
				// 最近点是端点 p1，t 保留负值（VTK 语义）
			} else if (t > 1.0) {
				closestPt[0] = p1d[0] + p21[0];
				closestPt[1] = p1d[1] + p21[1];
				closestPt[2] = p1d[2] + p21[2];
			} else {
				closestPt[0] = p1d[0] + t * p21[0];
				closestPt[1] = p1d[1] + t * p21[1];
				closestPt[2] = p1d[2] + t * p21[2];
			}
		}
		closest = Point(static_cast<float>(closestPt[0]), static_cast<float>(closestPt[1]),
		                static_cast<float>(closestPt[2]));
		const double dx = closestPt[0] - xd[0];
		const double dy = closestPt[1] - xd[1];
		const double dz = closestPt[2] - xd[2];
		return dx * dx + dy * dy + dz * dz;
	}

protected:
	Cell() {
		this->m_Points = Points::New();
		this->m_PointIds = IdArray::New();
	};
	~Cell() override = default;
};



IGAME_NAMESPACE_END
#endif