#include "Remesh/BoundaryLockedPatchBuilder.h"

#include <algorithm>
#include <limits>
#include <sstream>

namespace directional_retopo {
namespace {

double loopLength(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& loop,
    bool closed)
{
    double result = 0.0;
    for (std::size_t i = 1U; i < loop.size(); ++i) {
        result += (vertices[loop[i]] - vertices[loop[i - 1U]]).length();
    }
    if (closed && loop.size() > 1U) {
        result += (vertices[loop.front()] - vertices[loop.back()]).length();
    }
    return result;
}

MPoint centroid(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& loop)
{
    MPoint result(0.0, 0.0, 0.0);
    for (const std::size_t index : loop) {
        result += MVector(vertices[index].x, vertices[index].y, vertices[index].z);
    }
    if (!loop.empty()) {
        result.x /= static_cast<double>(loop.size());
        result.y /= static_cast<double>(loop.size());
        result.z /= static_cast<double>(loop.size());
    }
    return result;
}

BoundaryLoopCorrespondence identityCorrespondence(
    const TriangulatedPatch& patch,
    std::size_t sourceLoopIndex,
    std::size_t resultLoopIndex,
    const ResultBoundaryLoop& resultLoop,
    const std::vector<MPoint>& vertices)
{
    const PatchBoundaryLoop& sourceLoop = patch.boundaryLoops[sourceLoopIndex];
    std::size_t sourceVertexCount = sourceLoop.vertexIndices.size();
    if (sourceLoop.closed && sourceVertexCount > 1U &&
        sourceLoop.vertexIndices.front() == sourceLoop.vertexIndices.back()) {
        // An explicit repeated endpoint is traversal syntax, not a topology vertex.
        --sourceVertexCount;
    }
    const std::size_t sourceIdCycleSize =
        std::min(sourceVertexCount, sourceLoop.sourceVertexIds.size());
    BoundaryLoopCorrespondence result;
    result.sourceLoopIndex = sourceLoopIndex;
    result.resultLoopIndex = resultLoopIndex;
    result.sourceClosed = sourceLoop.closed;
    result.resultClosed = resultLoop.closed;
    result.closedStateMatches = sourceLoop.closed == resultLoop.closed;
    result.winding = BoundaryWinding::Aligned;
    result.windingAlignedAfterConformation = true;
    result.orderedMappingValid = true;
    result.sourceVertexCount = sourceVertexCount;
    result.resultVertexCount = resultLoop.vertexIndices.size();
    result.sourceEdgeCount = sourceLoop.closed ? sourceVertexCount :
        (sourceVertexCount > 0U ? sourceVertexCount - 1U : 0U);
    result.resultEdgeCount = resultLoop.closed
        ? resultLoop.vertexIndices.size()
        : resultLoop.vertexIndices.size() - 1U;
    result.sourceTotalArcLength =
        loopLength(vertices, resultLoop.vertexIndices, resultLoop.closed);
    result.resultTotalArcLengthBefore = result.sourceTotalArcLength;
    result.resultTotalArcLengthAfter = result.sourceTotalArcLength;
    for (const std::size_t vertexIndex : resultLoop.vertexIndices) {
        result.sourcePolylinePositions.push_back(vertices[vertexIndex]);
    }

    double cumulative = 0.0;
    for (std::size_t i = 0U; i < resultLoop.vertexIndices.size(); ++i) {
        if (i > 0U) {
            cumulative += (
                vertices[resultLoop.vertexIndices[i]] -
                vertices[resultLoop.vertexIndices[i - 1U]]).length();
        }
        const double normalized = result.sourceTotalArcLength > 0.0
            ? cumulative / result.sourceTotalArcLength
            : 0.0;
        BoundaryVertexCorrespondence vertex;
        vertex.resultVertexIndex = resultLoop.vertexIndices[i];
        vertex.resultOrderIndex = i;
        vertex.sourceVertex0 = i < sourceLoop.sourceVertexIds.size()
            ? sourceLoop.sourceVertexIds[i] : -1;
        const std::size_t next =
            sourceIdCycleSize == 0U ? 0U : (i + 1U) % sourceIdCycleSize;
        vertex.sourceVertex1 =
            sourceIdCycleSize > 0U && next < sourceLoop.sourceVertexIds.size()
            ? sourceLoop.sourceVertexIds[next] : -1;
        vertex.sourceEdgeId = i < sourceLoop.sourceEdgeIds.size()
            ? sourceLoop.sourceEdgeIds[i] : -1;
        vertex.resultNormalizedParameter = normalized;
        vertex.sourceNormalizedParameter = normalized;
        vertex.sourceUnwrappedParameter = normalized;
        vertex.resultPositionBeforeConformation = vertices[vertex.resultVertexIndex];
        vertex.sourcePosition = vertices[vertex.resultVertexIndex];
        result.vertices.push_back(vertex);
    }
    return result;
}

}  // namespace

const BoundaryLockedPatchBuilderSettings&
BoundaryLockedPatchBuilder::settings() const noexcept
{
    return settings_;
}

void BoundaryLockedPatchBuilder::setSettings(
    const BoundaryLockedPatchBuilderSettings& settings) noexcept
{
    settings_ = settings;
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 1.0e-15);
    TransitionCollarSettings collar = collarBuilder_.settings();
    collar.topologyPolicy = settings_.topologyPolicy;
    collar.trianglePolicy = settings_.trianglePolicy;
    collar.topologyBlendWidth = settings_.topologyBlendWidth;
    collar.geometryEpsilon = settings_.geometryEpsilon;
    collarBuilder_.setSettings(collar);
}

bool BoundaryLockedPatchBuilder::build(
    const TriangulatedPatch& completeSourcePatch,
    const QuadPatchResult& innerRemeshResult,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    result.clear();
    result.componentId = completeSourcePatch.componentId;
    result.boundaryLocked = true;
    result.topologyPolicy = settings_.topologyPolicy;
    result.trianglePolicy = settings_.trianglePolicy;
    result.targetEdgeLength = innerRemeshResult.targetEdgeLength;
    if (!innerRemeshResult.success ||
        innerRemeshResult.conformedVertices.empty() ||
        innerRemeshResult.polygons.empty()) {
        diagnostic = "Boundary-Locked builder received no valid Inner Remesh result.";
        return false;
    }
    if (completeSourcePatch.boundaryLoops.empty() ||
        innerRemeshResult.boundaryLoops.size() !=
            completeSourcePatch.boundaryLoops.size()) {
        diagnostic =
            "Fixed Source and Inner Remesh Boundary loop counts do not match.";
        return false;
    }

    result.rawVertices = innerRemeshResult.conformedVertices;
    result.polygons = innerRemeshResult.polygons;
    result.polygonRegions.assign(
        result.polygons.size(),
        ResultPolygonRegion::Core);
    for (std::size_t i = 0U; i < result.polygons.size(); ++i) {
        if (result.polygons[i].size() == 4U) {
            ++result.boundaryLockedDiagnostic.coreQuadCount;
        } else if (result.polygons[i].size() == 3U) {
            ++result.boundaryLockedDiagnostic.coreTriangleCount;
            result.triangleDiagnostics.push_back({
                i,
                TriangleReason::SolverFallback});
        }
    }

    std::vector<bool> innerUsed(innerRemeshResult.boundaryLoops.size(), false);
    for (std::size_t sourceLoopIndex = 0U;
         sourceLoopIndex < completeSourcePatch.boundaryLoops.size();
         ++sourceLoopIndex) {
        const PatchBoundaryLoop& sourceLoop =
            completeSourcePatch.boundaryLoops[sourceLoopIndex];
        if (!sourceLoop.closed) {
            diagnostic =
                "Open Fixed Source Boundary is unsupported in Phase 5A Preview.";
            return false;
        }

        std::vector<std::size_t> outer;
        std::size_t outerSourceVertexCount = sourceLoop.vertexIndices.size();
        if (sourceLoop.closed && outerSourceVertexCount > 1U &&
            sourceLoop.vertexIndices.front() == sourceLoop.vertexIndices.back()) {
            --outerSourceVertexCount;
        }
        if (outerSourceVertexCount < 3U) {
            diagnostic =
                "Fixed Source Boundary has fewer than three unique cycle vertices.";
            return false;
        }
        outer.reserve(outerSourceVertexCount);
        for (std::size_t sourceOrder = 0U;
             sourceOrder < outerSourceVertexCount;
             ++sourceOrder) {
            const std::size_t patchVertexIndex =
                sourceLoop.vertexIndices[sourceOrder];
            if (patchVertexIndex >= completeSourcePatch.vertices.size()) {
                diagnostic = "Fixed Source Boundary vertex index is invalid.";
                return false;
            }
            const std::size_t resultIndex = result.rawVertices.size();
            result.rawVertices.push_back(
                completeSourcePatch.vertices[patchVertexIndex].position);
            outer.push_back(resultIndex);
            result.fixedBoundaryVertexIndices.push_back(resultIndex);
        }

        const MPoint outerCenter = centroid(result.rawVertices, outer);
        std::size_t innerIndex = std::numeric_limits<std::size_t>::max();
        double nearest = std::numeric_limits<double>::infinity();
        for (std::size_t candidate = 0U;
             candidate < innerRemeshResult.boundaryLoops.size();
             ++candidate) {
            if (innerUsed[candidate]) {
                continue;
            }
            const MVector centerDelta =
                centroid(result.rawVertices,
                    innerRemeshResult.boundaryLoops[candidate].vertexIndices) - outerCenter;
            const double distance = centerDelta * centerDelta;
            if (distance < nearest) {
                nearest = distance;
                innerIndex = candidate;
            }
        }
        if (innerIndex == std::numeric_limits<std::size_t>::max()) {
            diagnostic = "Inner Remesh Boundary matching failed.";
            return false;
        }
        innerUsed[innerIndex] = true;
        const ResultBoundaryLoop& innerLoop =
            innerRemeshResult.boundaryLoops[innerIndex];

        TransitionCollarBuildResult collar;
        if (!collarBuilder_.build(
                result.rawVertices,
                outer,
                innerLoop.vertexIndices,
                true,
                completeSourcePatch,
                directionField,
                densityField,
                collar)) {
            result.boundaryLockedDiagnostic.boundaryCrossingCount +=
                collar.crossingCount;
            diagnostic = collar.diagnosticMessage;
            return false;
        }

        const std::size_t polygonOffset = result.polygons.size();
        result.polygons.insert(
            result.polygons.end(),
            collar.polygons.begin(),
            collar.polygons.end());
        result.polygonRegions.insert(
            result.polygonRegions.end(),
            collar.polygons.size(),
            ResultPolygonRegion::TransitionCollar);
        for (TriangleDiagnostic triangle : collar.triangleDiagnostics) {
            triangle.polygonIndex += polygonOffset;
            result.triangleDiagnostics.push_back(triangle);
        }
        auto& locked = result.boundaryLockedDiagnostic;
        if (sourceLoopIndex == 0U) {
            locked.outerBoundaryTopologySimple =
                collar.outerValidation.topologySimple;
            locked.innerBoundaryTopologySimple =
                collar.innerValidation.topologySimple;
        } else {
            locked.outerBoundaryTopologySimple =
                locked.outerBoundaryTopologySimple &&
                collar.outerValidation.topologySimple;
            locked.innerBoundaryTopologySimple =
                locked.innerBoundaryTopologySimple &&
                collar.innerValidation.topologySimple;
        }
        locked.outerBoundaryTrueIntersectionCount +=
            collar.outerValidation.trueIntersectionCount;
        locked.innerBoundaryTrueIntersectionCount +=
            collar.innerValidation.trueIntersectionCount;
        locked.selectedSeamOffset = collar.seamOffset;
        locked.innerOrderReversed = collar.innerOrderReversed;
        locked.boundaryAlignmentCost += collar.alignmentCost;
        locked.collarQuadCount += collar.quadCount;
        locked.collarTriangleCount += collar.triangleCount;
        locked.boundaryCrossingCount += collar.crossingCount;
        locked.innerBoundaryVertexCount += innerLoop.vertexIndices.size();
        locked.innerBoundaryEdgeCount += innerLoop.vertexIndices.size();

        ResultBoundaryLoop finalLoop;
        finalLoop.vertexIndices = std::move(outer);
        finalLoop.closed = true;
        finalLoop.totalLength =
            loopLength(result.rawVertices, finalLoop.vertexIndices, true);
        const std::size_t resultLoopIndex = result.boundaryLoops.size();
        result.boundaryLoops.push_back(finalLoop);
        result.boundaryCorrespondences.push_back(identityCorrespondence(
            completeSourcePatch,
            sourceLoopIndex,
            resultLoopIndex,
            result.boundaryLoops.back(),
            result.rawVertices));
        locked.fixedBoundaryVertexCount +=
            result.boundaryLoops.back().vertexIndices.size();
        locked.fixedBoundaryEdgeCount +=
            result.boundaryLoops.back().vertexIndices.size();
    }

    result.conformedVertices = result.rawVertices;
    result.boundaryLockedDiagnostic.topologyBlendWidth =
        settings_.topologyBlendWidth;
    result.boundaryLockedDiagnostic.maximumSourceBoundaryDisplacement = 0.0;
    result.boundaryLockedDiagnostic.success = true;
    result.success = true;
    std::ostringstream message;
    message << "Boundary-Locked Patch: fixed "
            << result.boundaryLockedDiagnostic.fixedBoundaryVertexCount
            << " vertices; inner "
            << result.boundaryLockedDiagnostic.innerBoundaryVertexCount
            << "; collar "
            << result.boundaryLockedDiagnostic.collarQuadCount << " quads/"
            << result.boundaryLockedDiagnostic.collarTriangleCount
            << " triangles; maximum fixed displacement 0.";
    diagnostic = message.str();
    result.diagnosticMessage = diagnostic;
    return true;
}

}  // namespace directional_retopo
