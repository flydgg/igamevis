#ifndef IGAME_MESH_SIMPLIFIER_H
#define IGAME_MESH_SIMPLIFIER_H

#include <algorithm>

#include "iGameFilter.h"
#include "iGameModel.h"
#include "iGameSurfaceMesh.h"
#include "iGameMeshTriangulationFilter.h"

IGAME_NAMESPACE_BEGIN
class MeshSimplificationFilterPro : public Filter {
public:
    I_OBJECT(MeshSimplificationFilterPro);
    static Pointer New() { return new MeshSimplificationFilterPro; }

    bool Execute() override;

    // Reduce to the target number. 
    // target: [0,1]
    void SetTargetReduction(float target = 0.5f) { this->TargetReduction = std::min(1.0f, std::max(0.0f, target)); }
    // If TargetFaceCount is not 0, TargetFaceCount is used first
    void SetTargetFaceCount(int target = 0) { this->TargetFaceCount = target; }

    void SetPreserveBoundary(bool flag = true) { this->PreserveBoundary = flag; }

    // true: 使用原来的顶点数据
    void SetFreeze(bool flag = false) { this->Freeze = flag; }

    // true: 使用原来的顶点数据
    void SetTransformToCellData(bool flag = false) { this->Transform = flag; }

    // ---- 交互期 LOD 用的快速聚类抽稀（ParaView 的 LOD 思路）----
    // 把包围盒划分成 resolution^3 的规则网格，每个格子的顶点合并成一个代表点
    // （点属性取平均），退化三角形丢弃。复杂度 O(n)，与边收缩二次误差简化相比
    // 便宜几个数量级，适合“大网格拖拽时用的低模”。
    void SetUseClustering(bool flag = true) { this->UseClustering = flag; }
    // resolution <= 0：按 TargetReduction 自动推导（约等于让输出点数接近
    // 输入点数 * TargetReduction）；否则使用指定分辨率。
    void SetClusterResolution(int resolution = 0) {
        this->ClusteringResolution = resolution <= 0 ? 0 : std::min(resolution, 256);
    }

protected:
    MeshSimplificationFilterPro();
    ~MeshSimplificationFilterPro() override = default;

    /** 聚类抽稀：与边收缩简化互斥（UseClustering 为真时走聚类路径） */
    bool ExecuteClustering();

    float TargetReduction = 0.5;
    float TargetFaceCount = 0;
    bool PreserveBoundary = true;
    bool Freeze = false;
    bool Transform = false;
    bool UseClustering = false;
    int ClusteringResolution = 0;
};
IGAME_NAMESPACE_END
#endif