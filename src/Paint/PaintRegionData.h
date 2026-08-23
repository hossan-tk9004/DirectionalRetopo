#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace directional_retopo {

struct BoundaryEdge final
{
    int edgeId = -1;
    std::array<int, 2> vertexIds = {-1, -1};
    int insideFaceId = -1;
    int outsideFaceId = -1;
    bool isOriginalMeshBoundary = false;
    bool ambiguous = false;
};

struct BoundaryLoop final
{
    std::vector<int> vertexIds;
    std::vector<int> edgeIds;
    bool closed = false;
    bool touchesOriginalMeshBoundary = false;
    bool ambiguous = false;
};

struct PaintRegionComponent final
{
    std::vector<int> coreFaceIds;
    std::vector<int> transitionFaceIds;
    std::vector<int> allFaceIds;
    std::vector<int> boundaryVertexIds;
    std::vector<BoundaryEdge> boundaryEdges;
    std::vector<BoundaryLoop> boundaryLoops;
    bool hasAmbiguousBoundary = false;
};

struct PaintRegionData final
{
    std::vector<PaintRegionComponent> components;
    std::vector<float> vertexInfluence;
    std::vector<float> faceInfluence;

    void clear() noexcept
    {
        components.clear();
        vertexInfluence.clear();
        faceInfluence.clear();
    }

    [[nodiscard]] std::size_t coreFaceCount() const noexcept
    {
        std::size_t count = 0;
        for (const PaintRegionComponent& component : components) {
            count += component.coreFaceIds.size();
        }
        return count;
    }

    [[nodiscard]] std::size_t transitionFaceCount() const noexcept
    {
        std::size_t count = 0;
        for (const PaintRegionComponent& component : components) {
            count += component.transitionFaceIds.size();
        }
        return count;
    }

    [[nodiscard]] std::size_t totalFaceCount() const noexcept
    {
        std::size_t count = 0;
        for (const PaintRegionComponent& component : components) {
            count += component.allFaceIds.size();
        }
        return count;
    }

    [[nodiscard]] std::size_t boundaryEdgeCount() const noexcept
    {
        std::size_t count = 0;
        for (const PaintRegionComponent& component : components) {
            count += component.boundaryEdges.size();
        }
        return count;
    }

    [[nodiscard]] std::size_t boundaryLoopCount() const noexcept
    {
        std::size_t count = 0;
        for (const PaintRegionComponent& component : components) {
            count += component.boundaryLoops.size();
        }
        return count;
    }

    [[nodiscard]] bool hasAmbiguousBoundary() const noexcept
    {
        for (const PaintRegionComponent& component : components) {
            if (component.hasAmbiguousBoundary) {
                return true;
            }
        }
        return false;
    }
};

}  // namespace directional_retopo
