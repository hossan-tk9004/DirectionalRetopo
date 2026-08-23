#pragma once

#include <maya/MPoint.h>

#include <array>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace directional_retopo {

struct PatchVertex final
{
    MPoint position;
    int sourceVertexId = -1;
    bool sourceBoundary = false;
};

struct PatchTriangle final
{
    std::array<std::size_t, 3> vertexIndices = {0U, 0U, 0U};
    int sourceFaceId = -1;
};

struct PatchBoundaryEdge final
{
    std::array<std::size_t, 2> vertexIndices = {0U, 0U};
    int sourceEdgeId = -1;
    bool originalMeshBoundary = false;
};

struct PatchBoundaryLoop final
{
    std::vector<std::size_t> vertexIndices;
    std::vector<int> sourceVertexIds;
    std::vector<int> sourceEdgeIds;
    bool closed = false;
    bool touchesOriginalMeshBoundary = false;
};

struct TriangulatedPatch final
{
    std::size_t componentId = 0;
    enum class Purpose
    {
        CompleteRegion,
        InnerRemeshCore,
    };
    Purpose purpose = Purpose::CompleteRegion;
    std::vector<PatchVertex> vertices;
    std::vector<PatchTriangle> triangles;
    std::vector<PatchBoundaryEdge> boundaryEdges;
    std::vector<PatchBoundaryLoop> boundaryLoops;
    std::vector<std::vector<std::size_t>> sourceFaceToTriangleIndices;
    bool touchesOriginalMeshBoundary = false;
    std::string diagnosticMessage;

    [[nodiscard]] bool empty() const noexcept
    {
        return vertices.empty() || triangles.empty();
    }
};

struct PatchBuildFailure final
{
    std::size_t componentId = 0;
    std::string message;
};

struct PatchBuildResult final
{
    std::vector<TriangulatedPatch> patches;
    std::vector<PatchBuildFailure> failures;
};

}  // namespace directional_retopo
