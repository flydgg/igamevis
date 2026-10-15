#ifndef iGameQuadraticVolume_h
#define iGameQuadraticVolume_h

#include "iGameCell.h"
#include "iGameQuadraticLine.h"

IGAME_NAMESPACE_BEGIN
class QuadraticVolume : public Cell {
public:
	I_OBJECT(QuadraticVolume);

	virtual int GetEdgePointIds(const int edgeId, const igIndex*& ptIds) = 0;
	virtual int GetFacePointIds(const int faceId, const igIndex*& ptIds) = 0;
	virtual int GetFaceEdgeIds(const int faceId, const igIndex*& edgeIds) = 0;
	virtual int GetPointToOneRingPoints(const int ptId, const igIndex*& ptIds) = 0;
	virtual int GetPointToNeighborEdges(const int ptId, const igIndex*& edgeIds) = 0;
	virtual int GetPointToNeighborFaces(const int ptId, const igIndex*& faceIds) = 0;
	virtual int GetEdgeToNeighborFaces(const int edgeId, const igIndex*& faceIds) = 0;
	virtual int GetFaceToNeighborFaces(const int faceId, const igIndex*& faceIds) = 0;

		igIndex GetEdgeId(const int id) { return this->EdgeIds->GetId(id); }
	igIndex GetFaceId(const int id) { return this->FaceIds->GetId(id); }
	IdArray::Pointer EdgeIds{};
	IdArray::Pointer FaceIds{};

	/* ---------------- 二次体单元 Newton 反解共用的 VTK 数学小工具 ----------------
	 * 对照线性单元的写法（iGameHexahedron / iGamePrism / iGamePyramid 各自局部定义），
	 * 这里把 4 个二次体单元（Tetra / Hexahedron / Prism / Pyramid）EvaluatePosition
	 * 的牛顿迭代都要用的两个 vtkMath 函数提到公共基类里，避免逐类重复。
	 */

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

protected:
	QuadraticVolume()
	{
		this->EdgeIds = IdArray::New();
		this->FaceIds = IdArray::New();
	}
	~QuadraticVolume() override = default;

};
IGAME_NAMESPACE_END
#endif