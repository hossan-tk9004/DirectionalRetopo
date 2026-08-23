#pragma once

#include <maya/MDagPath.h>
#include <maya/MMatrix.h>
#include <maya/MObjectHandle.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MVector.h>

#include <array>
#include <cstdint>
#include <vector>

namespace directional_retopo {

struct MeshEdgeTopology final
{
    std::array<int, 2> vertexIds = {-1, -1};
    std::vector<int> faceIds;
    double worldLength = 0.0;
    bool originalMeshBoundary = false;
};

struct MeshFaceTopology final
{
    std::vector<int> vertexIds;
    // Maya's own polygon triangulation, cached at the same lifecycle as the
    // topology. Each entry contains source Maya vertex IDs.
    std::vector<std::array<int, 3>> triangleVertexIds;
    std::vector<int> edgeIds;
    std::vector<int> adjacentFaceIds;
    MPoint worldCenter;
    MVector worldNormal;
    bool worldGeometryValid = false;
};

class MeshTopologyCache final
{
public:
    MStatus setTarget(const MDagPath& meshPath);
    void clear() noexcept;

    [[nodiscard]] bool hasTarget() const noexcept;
    [[nodiscard]] MStatus ensureCurrent();
    void invalidateTopology() noexcept;
    void invalidateGeometry() noexcept;

    [[nodiscard]] const MDagPath& meshPath() const noexcept;
    [[nodiscard]] const std::vector<MPoint>& worldVertexPositions() const noexcept;
    [[nodiscard]] const std::vector<MeshEdgeTopology>& edges() const noexcept;
    [[nodiscard]] const std::vector<MeshFaceTopology>& faces() const noexcept;
    [[nodiscard]] const std::vector<std::vector<int>>& vertexEdgeIds() const noexcept;

    [[nodiscard]] std::uint64_t topologyBuildCount() const noexcept;
    [[nodiscard]] std::uint64_t geometryBuildCount() const noexcept;

private:
    MStatus rebuildTopology();
    MStatus updateWorldGeometry();
    MStatus validateTopologyCounts();

    MDagPath meshPath_;
    MObjectHandle meshHandle_;
    std::vector<MPoint> worldVertexPositions_;
    std::vector<MeshEdgeTopology> edges_;
    std::vector<MeshFaceTopology> faces_;
    std::vector<std::vector<int>> vertexEdgeIds_;
    MMatrix cachedInclusiveMatrix_;
    int cachedVertexCount_ = -1;
    int cachedEdgeCount_ = -1;
    int cachedFaceCount_ = -1;
    bool topologyDirty_ = true;
    bool geometryDirty_ = true;
    std::uint64_t topologyBuildCount_ = 0;
    std::uint64_t geometryBuildCount_ = 0;
};

}  // namespace directional_retopo
