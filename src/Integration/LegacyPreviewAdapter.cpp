#include "Integration/LegacyPreviewAdapter.h"

#include <maya/MPoint.h>

#include <algorithm>

namespace directional_retopo {
namespace {

MPoint toMPoint(const solver::Vec3& value) noexcept
{
    return MPoint(value.x, value.y, value.z);
}

TriangleReason toLegacy(solver::TriangleReason reason) noexcept
{
    switch (reason) {
    case solver::TriangleReason::BoundaryCountMismatch: return TriangleReason::BoundaryCountMismatch;
    case solver::TriangleReason::BoundaryParity: return TriangleReason::BoundaryParity;
    case solver::TriangleReason::DensityTransition: return TriangleReason::DensityTransition;
    case solver::TriangleReason::FlowTermination: return TriangleReason::FlowTermination;
    case solver::TriangleReason::SmallHoleRepair: return TriangleReason::SmallHoleRepair;
    case solver::TriangleReason::SolverFallback: return TriangleReason::SolverFallback;
    }
    return TriangleReason::SolverFallback;
}

const solver::RegionComponent* findComponent(
    const solver::RemeshInput& input,
    std::size_t componentId) noexcept
{
    const auto found = std::find_if(
        input.components.begin(), input.components.end(),
        [componentId](const solver::RegionComponent& component) {
            return component.componentId == componentId;
        });
    return found == input.components.end() ? nullptr : &*found;
}

}  // namespace

QuadPatchResult LegacyPreviewAdapter::convert(
    const solver::ComponentResult& source,
    const solver::RemeshInput& input)
{
    QuadPatchResult result;
    result.componentId = source.componentId;
    result.success = source.status == solver::SolveStatus::Success && !source.debugOnly;
    result.debugPreviewAvailable = source.debugOnly;
    result.debugInnerResultOnly = source.debugOnly;
    result.boundaryLocked = result.success;
    result.diagnosticMessage = source.diagnosticMessage;
    result.topologyPolicy = input.settings.topologyPolicy == solver::TopologyPolicy::StrictAllQuads
        ? TopologyPolicy::StrictAllQuads
        : TopologyPolicy::QuadDominant;
    result.trianglePolicy = input.settings.trianglePolicy == solver::TrianglePolicy::Disallow
        ? TrianglePolicy::Disallow
        : TrianglePolicy::MinimalNecessary;

    result.rawVertices.reserve(source.rawVertices.size());
    for (const solver::Vec3& vertex : source.rawVertices) {
        result.rawVertices.push_back(toMPoint(vertex));
    }
    result.conformedVertices.reserve(source.vertices.size());
    for (const solver::Vec3& vertex : source.vertices) {
        result.conformedVertices.push_back(toMPoint(vertex));
    }
    for (std::size_t polygonIndex = 0U; polygonIndex < source.polygons.size(); ++polygonIndex) {
        const solver::ResultPolygon& polygon = source.polygons[polygonIndex];
        result.polygons.push_back(polygon.vertexIndices);
        result.polygonRegions.push_back(polygon.region == solver::PolygonRegion::Transition
            ? ResultPolygonRegion::TransitionCollar
            : ResultPolygonRegion::Core);
        if (polygon.type == solver::PolygonType::Triangle) {
            result.triangleDiagnostics.push_back({
                polygonIndex,
                toLegacy(polygon.triangleReason)});
        }
    }
    for (const solver::ResultBoundaryLoop& loop : source.boundaryLoops) {
        directional_retopo::ResultBoundaryLoop destination;
        destination.vertexIndices = loop.vertexIndices;
        destination.closed = loop.closed;
        result.boundaryLoops.push_back(std::move(destination));
    }
    result.quadCount = source.quality.quadCount;
    result.triangleCount = source.quality.triangleCount;
    result.nGonCount = source.quality.nGonCount;
    result.nonQuadCount = result.triangleCount + result.nGonCount;
    result.targetEdgeLength = source.quality.actualCoreEdgeLength;
    result.maximumSurfaceDistance = source.quality.maximumSurfaceDistance;
    result.fidelity.meanConformedSurfaceDistance = source.quality.meanSurfaceDistance;
    result.fidelity.maximumConformedSurfaceDistance = source.quality.maximumSurfaceDistance;
    result.boundaryLockedDiagnostic.success = result.success;
    result.boundaryLockedDiagnostic.topologyBlendWidth = input.settings.topologyBlendWidth;
    result.boundaryLockedDiagnostic.maximumSourceBoundaryDisplacement =
        source.quality.maximumBoundaryDisplacement;
    result.boundaryLockedDiagnostic.boundaryCrossingCount =
        source.quality.boundaryCrossingCount;
    result.boundaryLockedDiagnostic.requestedCoreTargetEdgeLength =
        source.quality.requestedCoreEdgeLength;

    for (const solver::FixedBoundaryMapping& mapping : source.fixedBoundaryMappings) {
        if (mapping.resultVertexIndex < result.conformedVertices.size()) {
            result.fixedBoundaryVertexIndices.push_back(mapping.resultVertexIndex);
        }
    }

    const solver::RegionComponent* component = findComponent(input, source.componentId);
    if (component != nullptr) {
        result.boundaryLockedDiagnostic.fixedBoundaryVertexCount =
            source.fixedBoundaryMappings.size();
        for (std::size_t loopIndex = 0U;
             loopIndex < component->fixedBoundaryLoops.size();
             ++loopIndex) {
            const solver::OrderedBoundaryLoop& sourceLoop =
                component->fixedBoundaryLoops[loopIndex];
            BoundaryLoopCorrespondence correspondence;
            correspondence.sourceLoopIndex = loopIndex;
            correspondence.resultLoopIndex = loopIndex;
            correspondence.sourceClosed = sourceLoop.closed;
            correspondence.resultClosed = loopIndex < source.boundaryLoops.size()
                ? source.boundaryLoops[loopIndex].closed
                : false;
            correspondence.closedStateMatches =
                correspondence.sourceClosed == correspondence.resultClosed;
            correspondence.sourceVertexCount = sourceLoop.vertexIndices.size();
            correspondence.sourceEdgeCount = sourceLoop.edgeIndices.size();
            if (loopIndex < source.boundaryLoops.size()) {
                correspondence.resultVertexCount =
                    source.boundaryLoops[loopIndex].vertexIndices.size();
                correspondence.resultEdgeCount = correspondence.resultVertexCount;
            }
            correspondence.vertexCountDifference =
                static_cast<long long>(correspondence.resultVertexCount) -
                static_cast<long long>(correspondence.sourceVertexCount);
            correspondence.orderedMappingValid = result.success;
            correspondence.windingAlignedAfterConformation = result.success;
            correspondence.winding = BoundaryWinding::Aligned;
            for (const std::size_t vertexIndex : sourceLoop.vertexIndices) {
                if (vertexIndex < input.sourceMesh.vertices.size()) {
                    correspondence.sourcePolylinePositions.push_back(
                        toMPoint(input.sourceMesh.vertices[vertexIndex].position));
                }
            }
            for (const solver::FixedBoundaryMapping& mapping : source.fixedBoundaryMappings) {
                if (std::find(
                        sourceLoop.sourceVertexIds.begin(),
                        sourceLoop.sourceVertexIds.end(),
                        mapping.sourceVertexId) == sourceLoop.sourceVertexIds.end()) {
                    continue;
                }
                BoundaryVertexCorrespondence vertex;
                vertex.resultVertexIndex = mapping.resultVertexIndex;
                vertex.sourceVertex0 = static_cast<int>(mapping.sourceVertexId);
                vertex.sourceVertex1 = static_cast<int>(mapping.sourceVertexId);
                vertex.sourcePosition = toMPoint(mapping.sourcePosition);
                correspondence.vertices.push_back(vertex);
            }
            result.boundaryCorrespondences.push_back(std::move(correspondence));
        }
    }
    return result;
}

}  // namespace directional_retopo
