#include "Remesh/PatchTriangulator.h"
#include "Mesh/BoundaryExtractor.h"


#include <algorithm>
#include <array>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace directional_retopo {

const PatchTriangulatorSettings& PatchTriangulator::settings() const noexcept
{
    return settings_;
}

void PatchTriangulator::setSettings(
    const PatchTriangulatorSettings& settings) noexcept
{
    settings_ = settings;
    settings_.minimumVertexCount = std::max<std::size_t>(settings_.minimumVertexCount, 3U);
    settings_.minimumTriangleCount = std::max<std::size_t>(settings_.minimumTriangleCount, 1U);
}

PatchBuildResult PatchTriangulator::build(
    const PaintRegionData& region,
    const MeshTopologyCache& topology) const
{
    PatchBuildResult result;
    result.patches.reserve(region.components.size());
    for (std::size_t componentId = 0;
         componentId < region.components.size();
         ++componentId) {
        TriangulatedPatch patch;
        std::string failure;
        if (buildComponent(
                componentId,
                region.components[componentId],
                topology,
                patch,
                failure)) {
            result.patches.push_back(std::move(patch));
        } else {
            result.failures.push_back({componentId, std::move(failure)});
        }
    }
    return result;
}
PatchBuildResult PatchTriangulator::buildInnerCores(
    const PaintRegionData& region,
    const MeshTopologyCache& topology) const
{
    PatchBuildResult result;
    result.patches.reserve(region.components.size());
    BoundaryExtractor extractor;
    for (std::size_t componentId = 0U;
         componentId < region.components.size();
         ++componentId) {
        const PaintRegionComponent& complete = region.components[componentId];
        if (complete.coreFaceIds.empty()) {
            result.failures.push_back({
                componentId,
                "Paint Core contains no faces for the inner remesh solve."});
            continue;
        }
        PaintRegionComponent core;
        core.coreFaceIds = complete.coreFaceIds;
        core.allFaceIds = complete.coreFaceIds;
        extractor.extract(topology, core);
        TriangulatedPatch patch;
        std::string failure;
        if (buildComponent(componentId, core, topology, patch, failure)) {
            patch.purpose = TriangulatedPatch::Purpose::InnerRemeshCore;
            patch.diagnosticMessage =
                "Inner Remesh Core triangulated from coreFaceIds.";
            result.patches.push_back(std::move(patch));
        } else {
            result.failures.push_back({componentId, std::move(failure)});
        }
    }
    return result;
}


bool PatchTriangulator::buildComponent(
    std::size_t componentId,
    const PaintRegionComponent& component,
    const MeshTopologyCache& topology,
    TriangulatedPatch& patch,
    std::string& failure) const
{
    patch = TriangulatedPatch();
    patch.componentId = componentId;
    const auto& faces = topology.faces();
    const auto& edges = topology.edges();
    const auto& positions = topology.worldVertexPositions();
    if (component.allFaceIds.empty()) {
        failure = "Complete Region contains no faces.";
        return false;
    }
    if (component.hasAmbiguousBoundary) {
        failure = "Region boundary is branching, ambiguous, or non-manifold.";
        return false;
    }

    for (const int faceId : component.allFaceIds) {
        if (faceId < 0 || static_cast<std::size_t>(faceId) >= faces.size()) {
            failure = "Region contains an out-of-range source face ID.";
            return false;
        }
        for (const int edgeId : faces[static_cast<std::size_t>(faceId)].edgeIds) {
            if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
                failure = "Region face contains an out-of-range source edge ID.";
                return false;
            }
            if (edges[static_cast<std::size_t>(edgeId)].faceIds.size() > 2U) {
                failure = "Region contains non-manifold source topology.";
                return false;
            }
        }
    }

    std::unordered_set<int> sourceBoundaryVertices;
    for (const BoundaryEdge& sourceEdge : component.boundaryEdges) {
        if (sourceEdge.ambiguous || sourceEdge.edgeId < 0 ||
            static_cast<std::size_t>(sourceEdge.edgeId) >= edges.size()) {
            failure = "Region contains an invalid or ambiguous boundary edge.";
            return false;
        }
        sourceBoundaryVertices.insert(sourceEdge.vertexIds[0]);
        sourceBoundaryVertices.insert(sourceEdge.vertexIds[1]);
        patch.touchesOriginalMeshBoundary =
            patch.touchesOriginalMeshBoundary || sourceEdge.isOriginalMeshBoundary;
    }

    std::unordered_map<int, std::size_t> sourceToLocalVertex;
    sourceToLocalVertex.reserve(component.allFaceIds.size() * 4U);
    const auto localVertex = [&](int sourceVertexId, std::size_t& localIndex) {
        if (sourceVertexId < 0 ||
            static_cast<std::size_t>(sourceVertexId) >= positions.size()) {
            return false;
        }
        const auto found = sourceToLocalVertex.find(sourceVertexId);
        if (found != sourceToLocalVertex.end()) {
            localIndex = found->second;
            return true;
        }
        localIndex = patch.vertices.size();
        sourceToLocalVertex.emplace(sourceVertexId, localIndex);
        patch.vertices.push_back({
            positions[static_cast<std::size_t>(sourceVertexId)],
            sourceVertexId,
            sourceBoundaryVertices.find(sourceVertexId) != sourceBoundaryVertices.end()});
        return true;
    };

    patch.sourceFaceToTriangleIndices.resize(faces.size());
    for (const int sourceFaceId : component.allFaceIds) {
        const MeshFaceTopology& sourceFace = faces[static_cast<std::size_t>(sourceFaceId)];
        if (sourceFace.triangleVertexIds.empty()) {
            failure = "Maya returned no triangulation for a Region face.";
            return false;
        }
        for (const std::array<int, 3>& sourceTriangle : sourceFace.triangleVertexIds) {
            PatchTriangle triangle;
            triangle.sourceFaceId = sourceFaceId;
            for (std::size_t corner = 0; corner < 3U; ++corner) {
                if (!localVertex(sourceTriangle[corner], triangle.vertexIndices[corner])) {
                    failure = "Maya triangulation contains an invalid source vertex ID.";
                    return false;
                }
            }
            if (triangle.vertexIndices[0] == triangle.vertexIndices[1] ||
                triangle.vertexIndices[1] == triangle.vertexIndices[2] ||
                triangle.vertexIndices[2] == triangle.vertexIndices[0]) {
                failure = "Maya triangulation contains a degenerate triangle.";
                return false;
            }
            const std::size_t triangleIndex = patch.triangles.size();
            patch.triangles.push_back(triangle);
            patch.sourceFaceToTriangleIndices[static_cast<std::size_t>(sourceFaceId)]
                .push_back(triangleIndex);
        }
    }

    for (const BoundaryEdge& sourceEdge : component.boundaryEdges) {
        const auto first = sourceToLocalVertex.find(sourceEdge.vertexIds[0]);
        const auto second = sourceToLocalVertex.find(sourceEdge.vertexIds[1]);
        if (first == sourceToLocalVertex.end() || second == sourceToLocalVertex.end()) {
            failure = "Source boundary edge could not be mapped to patch vertices.";
            return false;
        }
        patch.boundaryEdges.push_back({
            {first->second, second->second},
            sourceEdge.edgeId,
            sourceEdge.isOriginalMeshBoundary});
    }

    for (const BoundaryLoop& sourceLoop : component.boundaryLoops) {
        if (sourceLoop.ambiguous) {
            failure = "Source boundary loop is ambiguous.";
            return false;
        }
        PatchBoundaryLoop loop;
        loop.sourceVertexIds = sourceLoop.vertexIds;
        loop.sourceEdgeIds = sourceLoop.edgeIds;
        loop.closed = sourceLoop.closed;
        if (loop.closed && loop.sourceVertexIds.size() > 1U &&
            loop.sourceVertexIds.front() == loop.sourceVertexIds.back()) {
            // Accept legacy boundary traversal data with an explicit closing endpoint.
            loop.sourceVertexIds.pop_back();
        }
        loop.touchesOriginalMeshBoundary = sourceLoop.touchesOriginalMeshBoundary;
        loop.vertexIndices.reserve(loop.sourceVertexIds.size());
        for (const int sourceVertexId : loop.sourceVertexIds) {
            const auto found = sourceToLocalVertex.find(sourceVertexId);
            if (found == sourceToLocalVertex.end()) {
                failure = "Source boundary loop could not be mapped to patch vertices.";
                return false;
            }
            loop.vertexIndices.push_back(found->second);
        }
        patch.boundaryLoops.push_back(std::move(loop));
    }

    if (patch.vertices.size() < settings_.minimumVertexCount ||
        patch.triangles.size() < settings_.minimumTriangleCount) {
        std::ostringstream message;
        message << "Region too small for quad solve ("
                << patch.vertices.size() << " vertices, "
                << patch.triangles.size() << " triangles).";
        failure = message.str();
        return false;
    }
    patch.diagnosticMessage = patch.touchesOriginalMeshBoundary
        ? "Patch touches the original mesh boundary; no automatic repair was applied."
        : "Patch triangulated with Maya's cached polygon triangulation.";
    return true;
}

}  // namespace directional_retopo
