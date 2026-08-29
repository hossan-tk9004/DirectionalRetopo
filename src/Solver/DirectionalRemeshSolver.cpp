#include "Solver/DirectionalRemeshSolver.h"

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/AutoRemesherAdapter.h"
#include "Remesh/BoundaryCompatibilityDensity.h"
#include "Remesh/BoundaryLockedPatchBuilder.h"
#include "Remesh/LocalPatch.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <numeric>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace directional_retopo {
namespace {

MPoint toMPoint(const solver::Vec3& value) noexcept
{
    return MPoint(value.x, value.y, value.z);
}

MVector toMVector(const solver::Vec3& value) noexcept
{
    return MVector(value.x, value.y, value.z);
}

solver::Vec3 toPortable(const MPoint& value) noexcept
{
    return {value.x, value.y, value.z};
}

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if (values.size() % 2U == 0U) {
        result = (*std::max_element(values.begin(), values.begin() + middle) + result) * 0.5;
    }
    return result;
}

double compatibilityTarget(
    double requested,
    double boundaryMedian,
    double innerArcLength,
    std::size_t boundaryVertexCount,
    unsigned int blendWidth) noexcept
{
    if (!(requested > 0.0) || !std::isfinite(requested)) {
        return 0.0;
    }
    const double blend = static_cast<double>(std::max(blendWidth, 1U));
    const std::size_t desiredSegments = std::max<std::size_t>(
        3U,
        static_cast<std::size_t>(std::ceil(
            static_cast<double>(std::max<std::size_t>(boundaryVertexCount, 3U)) /
            blend)));
    const double samplingTarget = innerArcLength > 0.0
        ? innerArcLength / static_cast<double>(desiredSegments)
        : boundaryMedian;
    const double compatible = std::max(boundaryMedian, samplingTarget);
    return compatible > 0.0 ? std::min(requested, compatible) : requested;
}

DirectionFieldData makeDirectionField(const solver::RemeshInput& input)
{
    DirectionFieldData result;
    result.perFace.resize(input.directionField.size());
    for (std::size_t index = 0; index < input.directionField.size(); ++index) {
        const solver::FaceDirection& source = input.directionField[index];
        FaceDirectionField& destination = result.perFace[index];
        destination.normal = toMVector(source.normal);
        destination.uDirection = toMVector(source.uDirection);
        destination.vDirection = toMVector(source.vDirection);
        destination.constraintWeight = source.paintConstraintWeight;
        destination.topologyGuidanceWeight = source.topologyGuidanceWeight;
        destination.hasPaintConstraint = source.paintConstraintWeight > 0.0;
        destination.valid = source.valid;
    }
    return result;
}

DensityFieldData makeDensityField(const solver::RemeshInput& input)
{
    DensityFieldData result;
    result.perFace.resize(input.densityField.size());
    for (std::size_t index = 0; index < input.densityField.size(); ++index) {
        const solver::FaceDensity& source = input.densityField[index];
        directional_retopo::FaceDensity& destination = result.perFace[index];
        destination.targetEdgeLength = source.effectiveTargetEdgeLength;
        destination.baseTargetEdgeLength = source.requestedTargetEdgeLength;
        destination.scaleU = source.scaleU;
        destination.scaleV = source.scaleV;
        destination.curvatureLimited = source.curvatureConstrained;
        destination.valid = source.valid;
    }
    return result;
}

bool appendOrderedBoundaryLoops(
    const solver::RemeshInput& input,
    const std::unordered_set<std::size_t>& faceSet,
    std::vector<solver::OrderedBoundaryLoop>& loops,
    std::string& diagnostic)
{
    std::vector<std::size_t> boundaryEdges;
    std::unordered_map<std::size_t, std::vector<std::size_t>> vertexEdges;
    for (std::size_t edgeIndex = 0; edgeIndex < input.sourceMesh.edges.size(); ++edgeIndex) {
        const solver::SourceEdge& edge = input.sourceMesh.edges[edgeIndex];
        std::size_t insideCount = 0U;
        for (const std::size_t faceIndex : edge.faceIndices) {
            insideCount += faceSet.count(faceIndex);
        }
        if (insideCount != 1U) {
            continue;
        }
        boundaryEdges.push_back(edgeIndex);
        vertexEdges[edge.vertexIndices[0]].push_back(edgeIndex);
        vertexEdges[edge.vertexIndices[1]].push_back(edgeIndex);
    }
    if (boundaryEdges.empty()) {
        diagnostic = "Inner solve domain has no boundary edges.";
        return false;
    }
    for (const auto& entry : vertexEdges) {
        if (entry.second.size() != 2U) {
            diagnostic = "Inner solve boundary is open, branched, or non-manifold.";
            return false;
        }
    }

    std::unordered_set<std::size_t> unused(boundaryEdges.begin(), boundaryEdges.end());
    while (!unused.empty()) {
        const std::size_t firstEdgeIndex = *unused.begin();
        const solver::SourceEdge& firstEdge = input.sourceMesh.edges[firstEdgeIndex];
        const std::size_t start = firstEdge.vertexIndices[0];
        std::size_t current = firstEdge.vertexIndices[1];
        std::size_t previousEdge = firstEdgeIndex;
        solver::OrderedBoundaryLoop loop;
        loop.closed = true;
        loop.vertexIndices.push_back(start);
        loop.edgeIndices.push_back(firstEdgeIndex);
        unused.erase(firstEdgeIndex);

        std::size_t guard = 0U;
        while (current != start && guard++ <= boundaryEdges.size()) {
            loop.vertexIndices.push_back(current);
            const std::vector<std::size_t>& incident = vertexEdges[current];
            const std::size_t nextEdge = incident[0] == previousEdge ? incident[1] : incident[0];
            if (unused.erase(nextEdge) == 0U) {
                diagnostic = "Inner solve boundary traversal repeated an edge.";
                return false;
            }
            loop.edgeIndices.push_back(nextEdge);
            const solver::SourceEdge& edge = input.sourceMesh.edges[nextEdge];
            current = edge.vertexIndices[0] == current
                ? edge.vertexIndices[1]
                : edge.vertexIndices[0];
            previousEdge = nextEdge;
        }
        if (current != start || loop.vertexIndices.size() < 3U) {
            diagnostic = "Inner solve boundary traversal did not form a closed loop.";
            return false;
        }
        for (const std::size_t vertexIndex : loop.vertexIndices) {
            loop.sourceVertexIds.push_back(input.sourceMesh.vertices[vertexIndex].sourceVertexId);
        }
        for (const std::size_t edgeIndex : loop.edgeIndices) {
            loop.sourceEdgeIds.push_back(input.sourceMesh.edges[edgeIndex].sourceEdgeId);
            loop.touchesOriginalMeshBoundary = loop.touchesOriginalMeshBoundary ||
                input.sourceMesh.edges[edgeIndex].originalMeshBoundary;
        }
        loops.push_back(std::move(loop));
    }
    return true;
}

bool buildPatch(
    const solver::RemeshInput& input,
    std::size_t componentId,
    const std::vector<std::size_t>& faceIndices,
    const std::vector<solver::OrderedBoundaryLoop>* fixedLoops,
    TriangulatedPatch::Purpose purpose,
    TriangulatedPatch& patch,
    std::string& diagnostic)
{
    patch = TriangulatedPatch();
    patch.componentId = componentId;
    patch.purpose = purpose;
    if (faceIndices.empty()) {
        diagnostic = "Patch contains no faces.";
        return false;
    }
    std::unordered_set<std::size_t> faceSet(faceIndices.begin(), faceIndices.end());
    std::vector<solver::OrderedBoundaryLoop> computedLoops;
    const std::vector<solver::OrderedBoundaryLoop>* loops = fixedLoops;
    if (loops == nullptr) {
        if (!appendOrderedBoundaryLoops(input, faceSet, computedLoops, diagnostic)) {
            return false;
        }
        loops = &computedLoops;
    }

    std::unordered_set<std::size_t> boundaryVertices;
    for (const solver::OrderedBoundaryLoop& loop : *loops) {
        boundaryVertices.insert(loop.vertexIndices.begin(), loop.vertexIndices.end());
        patch.touchesOriginalMeshBoundary =
            patch.touchesOriginalMeshBoundary || loop.touchesOriginalMeshBoundary;
    }

    std::unordered_map<std::size_t, std::size_t> sourceToLocal;
    const auto localVertex = [&](std::size_t sourceIndex, std::size_t& localIndex) {
        if (sourceIndex >= input.sourceMesh.vertices.size()) {
            return false;
        }
        const auto found = sourceToLocal.find(sourceIndex);
        if (found != sourceToLocal.end()) {
            localIndex = found->second;
            return true;
        }
        localIndex = patch.vertices.size();
        sourceToLocal.emplace(sourceIndex, localIndex);
        const solver::SourceVertex& source = input.sourceMesh.vertices[sourceIndex];
        patch.vertices.push_back({
            toMPoint(source.position),
            static_cast<int>(source.sourceVertexId),
            boundaryVertices.count(sourceIndex) != 0U});
        return true;
    };

    patch.sourceFaceToTriangleIndices.resize(input.sourceMesh.faces.size());
    for (const std::size_t faceIndex : faceIndices) {
        if (faceIndex >= input.sourceMesh.faces.size()) {
            diagnostic = "Patch contains an out-of-range face index.";
            return false;
        }
        const solver::SourceFace& face = input.sourceMesh.faces[faceIndex];
        if (face.triangleIndices.empty()) {
            diagnostic = "Patch face contains no cached triangulation.";
            return false;
        }
        for (const std::size_t triangleIndex : face.triangleIndices) {
            if (triangleIndex >= input.sourceMesh.triangles.size()) {
                diagnostic = "Patch face contains an invalid triangle index.";
                return false;
            }
            const solver::SourceTriangle& source = input.sourceMesh.triangles[triangleIndex];
            PatchTriangle triangle;
            triangle.sourceFaceId = static_cast<int>(face.sourceFaceId);
            for (std::size_t corner = 0U; corner < 3U; ++corner) {
                if (!localVertex(source.vertexIndices[corner], triangle.vertexIndices[corner])) {
                    diagnostic = "Patch triangulation contains an invalid vertex index.";
                    return false;
                }
            }
            const std::size_t localTriangleIndex = patch.triangles.size();
            patch.triangles.push_back(triangle);
            patch.sourceFaceToTriangleIndices[faceIndex].push_back(localTriangleIndex);
        }
    }

    for (const solver::OrderedBoundaryLoop& sourceLoop : *loops) {
        PatchBoundaryLoop loop;
        loop.closed = sourceLoop.closed;
        loop.touchesOriginalMeshBoundary = sourceLoop.touchesOriginalMeshBoundary;
        for (const std::size_t sourceVertexIndex : sourceLoop.vertexIndices) {
            std::size_t localIndex = 0U;
            if (!localVertex(sourceVertexIndex, localIndex)) {
                diagnostic = "Boundary loop contains an invalid source vertex.";
                return false;
            }
            loop.vertexIndices.push_back(localIndex);
            loop.sourceVertexIds.push_back(
                static_cast<int>(input.sourceMesh.vertices[sourceVertexIndex].sourceVertexId));
        }
        for (const std::size_t sourceEdgeIndex : sourceLoop.edgeIndices) {
            if (sourceEdgeIndex >= input.sourceMesh.edges.size()) {
                diagnostic = "Boundary loop contains an invalid source edge.";
                return false;
            }
            const solver::SourceEdge& sourceEdge = input.sourceMesh.edges[sourceEdgeIndex];
            std::size_t local0 = 0U;
            std::size_t local1 = 0U;
            if (!localVertex(sourceEdge.vertexIndices[0], local0) ||
                !localVertex(sourceEdge.vertexIndices[1], local1)) {
                diagnostic = "Boundary edge could not be mapped to local vertices.";
                return false;
            }
            loop.sourceEdgeIds.push_back(static_cast<int>(sourceEdge.sourceEdgeId));
            patch.boundaryEdges.push_back({
                {local0, local1},
                static_cast<int>(sourceEdge.sourceEdgeId),
                sourceEdge.originalMeshBoundary});
        }
        patch.boundaryLoops.push_back(std::move(loop));
    }

    if (patch.vertices.size() < 4U || patch.triangles.size() < 2U) {
        diagnostic = "Region too small for quad solve.";
        return false;
    }
    patch.diagnosticMessage = fixedLoops == nullptr
        ? "Inner Remesh Core triangulated from the portable source snapshot."
        : "Complete Region triangulated from the portable source snapshot.";
    return true;
}

std::vector<std::size_t> buildInnerFaces(
    const solver::RemeshInput& input,
    const solver::RegionComponent& component,
    unsigned int expansionRings)
{
    std::unordered_set<std::size_t> complete(
        component.allFaceIndices.begin(), component.allFaceIndices.end());
    std::unordered_set<std::size_t> fixedBoundaryFaces;
    for (const solver::OrderedBoundaryLoop& loop : component.fixedBoundaryLoops) {
        for (const std::size_t edgeIndex : loop.edgeIndices) {
            if (edgeIndex >= input.sourceMesh.edges.size()) {
                continue;
            }
            for (const std::size_t faceIndex : input.sourceMesh.edges[edgeIndex].faceIndices) {
                if (complete.count(faceIndex) != 0U) {
                    fixedBoundaryFaces.insert(faceIndex);
                }
            }
        }
    }
    std::unordered_set<std::size_t> result(
        component.coreFaceIndices.begin(), component.coreFaceIndices.end());
    std::vector<std::size_t> frontier(component.coreFaceIndices.begin(), component.coreFaceIndices.end());
    for (unsigned int ring = 0U; ring < expansionRings && !frontier.empty(); ++ring) {
        std::vector<std::size_t> next;
        for (const std::size_t faceIndex : frontier) {
            if (faceIndex >= input.sourceMesh.faces.size()) {
                continue;
            }
            for (const std::size_t adjacent : input.sourceMesh.faces[faceIndex].adjacentFaceIndices) {
                if (complete.count(adjacent) == 0U || fixedBoundaryFaces.count(adjacent) != 0U) {
                    continue;
                }
                if (result.insert(adjacent).second) {
                    next.push_back(adjacent);
                }
            }
        }
        frontier = std::move(next);
    }
    std::vector<std::size_t> ordered(result.begin(), result.end());
    std::sort(ordered.begin(), ordered.end());
    return ordered;
}

double boundaryLength(const TriangulatedPatch& patch)
{
    double result = 0.0;
    for (const PatchBoundaryLoop& loop : patch.boundaryLoops) {
        for (std::size_t index = 0U; index < loop.vertexIndices.size(); ++index) {
            if (!loop.closed && index + 1U == loop.vertexIndices.size()) {
                break;
            }
            const std::size_t next = (index + 1U) % loop.vertexIndices.size();
            result += (patch.vertices[loop.vertexIndices[index]].position -
                       patch.vertices[loop.vertexIndices[next]].position).length();
        }
    }
    return result;
}

BoundaryCompatibilityDensityResult buildCompatibilityDensity(
    const solver::RemeshInput& input,
    const TriangulatedPatch& completePatch,
    const TriangulatedPatch& innerPatch,
    const DensityFieldData& requested,
    unsigned int blendWidth)
{
    BoundaryCompatibilityDensityResult result;
    result.densityField = requested;
    result.interfaceBandRings = std::max(blendWidth, 1U);
    std::unordered_set<int> innerFaces;
    std::vector<double> targets;
    for (const PatchTriangle& triangle : innerPatch.triangles) {
        if (!innerFaces.insert(triangle.sourceFaceId).second) {
            continue;
        }
        const directional_retopo::FaceDensity* density = requested.face(triangle.sourceFaceId);
        if (density != nullptr && density->valid && density->targetEdgeLength > 0.0 &&
            std::isfinite(density->targetEdgeLength)) {
            targets.push_back(density->targetEdgeLength);
        }
    }
    result.requestedCoreTargetEdgeLength = median(std::move(targets));
    if (!(result.requestedCoreTargetEdgeLength > 0.0)) {
        result.diagnosticMessage = "Compatibility Density found no valid Core target.";
        return result;
    }
    std::vector<double> sourceLengths;
    for (const PatchBoundaryEdge& edge : completePatch.boundaryEdges) {
        const double length = (completePatch.vertices[edge.vertexIndices[1]].position -
                               completePatch.vertices[edge.vertexIndices[0]].position).length();
        if (length > 0.0 && std::isfinite(length)) {
            sourceLengths.push_back(length);
        }
    }
    result.sourceBoundaryMedianEdgeLength = median(std::move(sourceLengths));
    std::size_t sourceBoundaryVertexCount = 0U;
    for (const PatchBoundaryLoop& loop : completePatch.boundaryLoops) {
        sourceBoundaryVertexCount += loop.vertexIndices.size();
    }
    result.effectiveInterfaceTargetEdgeLength =
        compatibilityTarget(
            result.requestedCoreTargetEdgeLength,
            result.sourceBoundaryMedianEdgeLength,
            boundaryLength(innerPatch),
            sourceBoundaryVertexCount,
            blendWidth);

    std::unordered_set<int> bandFaces;
    std::vector<int> frontier;
    for (const PatchBoundaryEdge& edge : innerPatch.boundaryEdges) {
        if (edge.sourceEdgeId < 0 ||
            static_cast<std::size_t>(edge.sourceEdgeId) >= input.sourceMesh.edges.size()) {
            continue;
        }
        for (const std::size_t faceIndex :
             input.sourceMesh.edges[static_cast<std::size_t>(edge.sourceEdgeId)].faceIndices) {
            const int faceId = static_cast<int>(faceIndex);
            if (innerFaces.count(faceId) != 0U && bandFaces.insert(faceId).second) {
                frontier.push_back(faceId);
            }
        }
    }
    for (unsigned int ring = 1U; ring < result.interfaceBandRings && !frontier.empty(); ++ring) {
        std::vector<int> next;
        for (const int faceId : frontier) {
            for (const std::size_t adjacent : input.sourceMesh.faces[static_cast<std::size_t>(faceId)].adjacentFaceIndices) {
                const int adjacentId = static_cast<int>(adjacent);
                if (innerFaces.count(adjacentId) != 0U && bandFaces.insert(adjacentId).second) {
                    next.push_back(adjacentId);
                }
            }
        }
        frontier = std::move(next);
    }
    for (const int faceId : bandFaces) {
        directional_retopo::FaceDensity& density = result.densityField.perFace[static_cast<std::size_t>(faceId)];
        if (density.valid) {
            density.targetEdgeLength = std::min(
                density.targetEdgeLength,
                result.effectiveInterfaceTargetEdgeLength);
        }
    }
    result.interfaceBandFaceCount = bandFaces.size();
    result.success = true;
    return result;
}

solver::TriangleReason toPortable(TriangleReason reason) noexcept
{
    switch (reason) {
    case TriangleReason::BoundaryCountMismatch: return solver::TriangleReason::BoundaryCountMismatch;
    case TriangleReason::BoundaryParity: return solver::TriangleReason::BoundaryParity;
    case TriangleReason::DensityTransition: return solver::TriangleReason::DensityTransition;
    case TriangleReason::FlowTermination: return solver::TriangleReason::FlowTermination;
    case TriangleReason::SmallHoleRepair: return solver::TriangleReason::SmallHoleRepair;
    case TriangleReason::SolverFallback: return solver::TriangleReason::SolverFallback;
    }
    return solver::TriangleReason::SolverFallback;
}

double p95(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t index = std::min(
        values.size() - 1U,
        static_cast<std::size_t>(std::ceil(0.95 * static_cast<double>(values.size()))) - 1U);
    return values[index];
}

solver::ComponentResult makePortableResult(
    const QuadPatchResult& source,
    const QuadComponentSolveReport& report,
    bool debugOnly)
{
    solver::ComponentResult result;
    result.componentId = source.componentId;
    result.status = source.success ? solver::SolveStatus::Success : solver::SolveStatus::Failed;
    result.failureCode = source.success ? solver::FailureCode::Success : solver::FailureCode::TransitionBuildFailed;
    result.failedStage = source.success ? std::string() : "BoundaryLockedFinalization";
    result.diagnosticMessage = report.diagnosticMessage;
    result.debugOnly = debugOnly;
    result.retryCount = static_cast<unsigned int>(report.retryAttemptCount);
    result.retryReason = report.retryReason;
    result.rawVertices.reserve(source.rawVertices.size());
    for (const MPoint& vertex : source.rawVertices) {
        result.rawVertices.push_back(toPortable(vertex));
    }
    result.vertices.reserve(source.conformedVertices.size());
    for (const MPoint& vertex : source.conformedVertices) {
        result.vertices.push_back(toPortable(vertex));
    }
    std::unordered_map<std::size_t, TriangleReason> triangleReasons;
    for (const TriangleDiagnostic& diagnostic : source.triangleDiagnostics) {
        triangleReasons[diagnostic.polygonIndex] = diagnostic.reason;
    }
    for (std::size_t polygonIndex = 0U; polygonIndex < source.polygons.size(); ++polygonIndex) {
        solver::ResultPolygon polygon;
        polygon.vertexIndices = source.polygons[polygonIndex];
        polygon.type = polygon.vertexIndices.size() == 3U
            ? solver::PolygonType::Triangle
            : (polygon.vertexIndices.size() == 4U
                ? solver::PolygonType::Quad
                : solver::PolygonType::NGon);
        polygon.region = polygonIndex < source.polygonRegions.size() &&
                source.polygonRegions[polygonIndex] == ResultPolygonRegion::TransitionCollar
            ? solver::PolygonRegion::Transition
            : solver::PolygonRegion::Core;
        const auto reason = triangleReasons.find(polygonIndex);
        if (reason != triangleReasons.end()) {
            polygon.triangleReason = toPortable(reason->second);
        }
        result.polygons.push_back(std::move(polygon));
    }
    for (const directional_retopo::ResultBoundaryLoop& sourceLoop : source.boundaryLoops) {
        result.boundaryLoops.push_back({sourceLoop.vertexIndices, sourceLoop.closed});
    }
    std::vector<double> surfaceDistances;
    for (const ResultVertexSourceMapping& sourceMapping : source.sourceMappings) {
        solver::ResultSourceMapping mapping;
        mapping.sourceTriangleIndex = sourceMapping.patchTriangleIndex;
        mapping.sourceFaceId = sourceMapping.sourceFaceId;
        mapping.surfaceDistance = sourceMapping.surfaceDistance;
        result.sourceMappings.push_back(mapping);
        if (std::isfinite(mapping.surfaceDistance)) {
            surfaceDistances.push_back(mapping.surfaceDistance);
        }
    }
    for (const std::size_t vertexIndex : source.fixedBoundaryVertexIndices) {
        if (vertexIndex >= source.conformedVertices.size()) {
            continue;
        }
        solver::FixedBoundaryMapping mapping;
        mapping.resultVertexIndex = vertexIndex;
        mapping.sourcePosition = toPortable(source.conformedVertices[vertexIndex]);
        for (const BoundaryLoopCorrespondence& correspondence : source.boundaryCorrespondences) {
            const auto found = std::find_if(
                correspondence.vertices.begin(), correspondence.vertices.end(),
                [vertexIndex](const BoundaryVertexCorrespondence& candidate) {
                    return candidate.resultVertexIndex == vertexIndex;
                });
            if (found != correspondence.vertices.end()) {
                mapping.sourcePosition = toPortable(found->sourcePosition);
                mapping.sourceVertexId = found->sourceEdgeParameter <= 0.5
                    ? found->sourceVertex0
                    : found->sourceVertex1;
                break;
            }
        }
        result.fixedBoundaryMappings.push_back(mapping);
    }
    result.quality.quadCount = source.quadCount;
    result.quality.triangleCount = source.triangleCount;
    result.quality.nGonCount = source.nGonCount;
    result.quality.boundaryCrossingCount = source.boundaryLockedDiagnostic.boundaryCrossingCount;
    result.quality.maximumBoundaryDisplacement =
        source.boundaryLockedDiagnostic.maximumSourceBoundaryDisplacement;
    result.quality.meanSurfaceDistance = source.fidelity.meanConformedSurfaceDistance;
    result.quality.p95SurfaceDistance = p95(std::move(surfaceDistances));
    result.quality.maximumSurfaceDistance = source.fidelity.maximumConformedSurfaceDistance;
    result.quality.maximumCoreDirectionDeviationDegrees = report.maximumGuidanceDeviationDegrees;
    result.quality.requestedCoreEdgeLength = report.requestedCoreTargetEdgeLength;
    result.quality.actualCoreEdgeLength = source.targetEdgeLength;
    result.timings.patchBuildMilliseconds = report.timings.patchBuildMilliseconds;
    result.timings.parameterizationMilliseconds = report.timings.parameterizationMilliseconds;
    result.timings.extractionMilliseconds = report.timings.extractionMilliseconds;
    result.timings.conformationMilliseconds = report.timings.conformationMilliseconds;
    result.timings.transitionMilliseconds = report.timings.collarBuildMilliseconds;
    result.timings.validationMilliseconds = report.timings.validationMilliseconds;
    result.timings.totalMilliseconds = report.timings.totalMilliseconds;
    return result;
}

solver::FailureCode reportFailureCode(const QuadComponentSolveReport& report) noexcept
{
    if (!report.parameterizationSuccess) {
        return solver::FailureCode::ParameterizationFailed;
    }
    if (!report.extractionSuccess) {
        return solver::FailureCode::QuadExtractionFailed;
    }
    if (!report.conformationSuccess) {
        return solver::FailureCode::SurfaceConformationFailed;
    }
    if (report.boundaryLockedCollarAttempted && !report.boundaryLockedCollarSuccess) {
        return solver::FailureCode::TransitionBuildFailed;
    }
    if (report.finalConformationAttempted && !report.finalConformationSuccess) {
        return solver::FailureCode::SurfaceConformationFailed;
    }
    return solver::FailureCode::FinalValidationFailed;
}

}  // namespace

solver::RemeshResult DirectionalRemeshSolver::solve(
    const solver::RemeshInput& input) const noexcept
{
    solver::RemeshResult output;
    const auto totalStart = std::chrono::steady_clock::now();
    try {
        std::string validationDiagnostic;
        if (!input.valid(&validationDiagnostic)) {
            output.failureCode = solver::FailureCode::InvalidInput;
            output.warnings.push_back(validationDiagnostic);
            return output;
        }

        const DirectionFieldData directionField = makeDirectionField(input);
        const DensityFieldData requestedDensity = makeDensityField(input);
        AutoRemesherAdapter autoRemesher;
        BoundaryLockedPatchBuilder lockedBuilder;
        BoundaryLockedPatchBuilderSettings lockedSettings = lockedBuilder.settings();
        lockedSettings.topologyBlendWidth = input.settings.topologyBlendWidth;
        lockedSettings.topologyPolicy = input.settings.topologyPolicy == solver::TopologyPolicy::StrictAllQuads
            ? TopologyPolicy::StrictAllQuads
            : TopologyPolicy::QuadDominant;
        lockedSettings.trianglePolicy = input.settings.trianglePolicy == solver::TrianglePolicy::Disallow
            ? TrianglePolicy::Disallow
            : TrianglePolicy::MinimalNecessary;
        lockedBuilder.setSettings(lockedSettings);

        bool anySuccess = false;
        for (const solver::RegionComponent& component : input.components) {
            const auto patchStart = std::chrono::steady_clock::now();
            TriangulatedPatch completePatch;
            std::string buildDiagnostic;
            if (!buildPatch(
                    input,
                    component.componentId,
                    component.allFaceIndices,
                    &component.fixedBoundaryLoops,
                    TriangulatedPatch::Purpose::CompleteRegion,
                    completePatch,
                    buildDiagnostic)) {
                solver::ComponentResult failure;
                failure.componentId = component.componentId;
                failure.failureCode = buildDiagnostic.find("too small") != std::string::npos
                    ? solver::FailureCode::RegionTooSmall
                    : solver::FailureCode::PatchBuildFailed;
                failure.failedStage = "PatchBuild";
                failure.diagnosticMessage = buildDiagnostic;
                output.components.push_back(std::move(failure));
                continue;
            }

            const std::vector<std::size_t> standardFaces = buildInnerFaces(input, component, 0U);
            const std::vector<std::size_t> adaptiveFaces = input.settings.topologyBlendWidth > 1U
                ? buildInnerFaces(input, component, input.settings.topologyBlendWidth - 1U)
                : std::vector<std::size_t>();
            TriangulatedPatch standardPatch;
            TriangulatedPatch adaptivePatch;
            std::string standardDiagnostic;
            std::string adaptiveDiagnostic;
            const bool hasStandard = buildPatch(
                input,
                component.componentId,
                standardFaces,
                nullptr,
                TriangulatedPatch::Purpose::InnerRemeshCore,
                standardPatch,
                standardDiagnostic);
            const bool hasAdaptive = !adaptiveFaces.empty() && buildPatch(
                input,
                component.componentId,
                adaptiveFaces,
                nullptr,
                TriangulatedPatch::Purpose::InnerRemeshCore,
                adaptivePatch,
                adaptiveDiagnostic);
            if (!hasStandard && !hasAdaptive) {
                solver::ComponentResult failure;
                failure.componentId = component.componentId;
                failure.failureCode = solver::FailureCode::RegionTooSmall;
                failure.failedStage = "InnerPatchBuild";
                failure.diagnosticMessage = standardDiagnostic + " " + adaptiveDiagnostic;
                output.components.push_back(std::move(failure));
                continue;
            }

            BoundaryCompatibilityDensityResult standardCompatibility;
            if (hasStandard) {
                standardCompatibility = buildCompatibilityDensity(
                    input, completePatch, standardPatch, requestedDensity,
                    input.settings.topologyBlendWidth);
            }
            BoundaryCompatibilityDensityResult adaptiveCompatibility;
            if (hasAdaptive) {
                adaptiveCompatibility = buildCompatibilityDensity(
                    input, completePatch, adaptivePatch, requestedDensity,
                    input.settings.topologyBlendWidth);
            }
            struct Attempt final
            {
                const TriangulatedPatch* patch = nullptr;
                const DensityFieldData* density = nullptr;
                const BoundaryCompatibilityDensityResult* compatibility = nullptr;
                const char* reason = nullptr;
            };
            std::vector<Attempt> attempts;
            if (hasStandard) {
                attempts.push_back({&standardPatch, &requestedDensity, nullptr, "Requested Core Density"});
                if (standardCompatibility.success) {
                    attempts.push_back({&standardPatch, &standardCompatibility.densityField,
                                        &standardCompatibility, "Boundary Compatibility Density"});
                }
            }
            if (hasAdaptive && (!hasStandard || adaptivePatch.triangles.size() > standardPatch.triangles.size()) &&
                adaptiveCompatibility.success) {
                attempts.push_back({&adaptivePatch, &adaptiveCompatibility.densityField,
                                    &adaptiveCompatibility,
                                    "Adaptive Inner Solve Region + Compatibility Density"});
            }
            const std::size_t attemptLimit = std::min<std::size_t>(
                attempts.size(), input.settings.maximumRetryAttempts);
            QuadComponentSolveReport report;
            QuadPatchResult lastInnerDebug;
            bool finalValid = false;
            std::ostringstream retryHistory;
            for (std::size_t attemptIndex = 0U; attemptIndex < attemptLimit; ++attemptIndex) {
                const Attempt& attempt = attempts[attemptIndex];
                report = autoRemesher.solve(*attempt.patch, directionField, *attempt.density);
                report.retryAttemptCount = attemptIndex + 1U;
                report.retryReason = attempt.reason;
                report.requestedCoreTargetEdgeLength = attempt.compatibility != nullptr
                    ? attempt.compatibility->requestedCoreTargetEdgeLength
                    : report.meanEffectiveTargetEdgeLength;
                report.effectiveInterfaceTargetEdgeLength = attempt.compatibility != nullptr
                    ? attempt.compatibility->effectiveInterfaceTargetEdgeLength
                    : report.meanEffectiveTargetEdgeLength;
                std::unordered_set<int> innerFaces;
                for (const PatchTriangle& triangle : attempt.patch->triangles) {
                    innerFaces.insert(triangle.sourceFaceId);
                }
                report.innerSolveFaceCount = innerFaces.size();
                if (!report.result.success) {
                    retryHistory << "Attempt " << (attemptIndex + 1U) << " ("
                                 << attempt.reason << "): " << report.diagnosticMessage << ' ';
                    continue;
                }
                lastInnerDebug = report.result;
                const auto transitionStart = std::chrono::steady_clock::now();
                QuadPatchResult boundaryLocked;
                std::string boundaryDiagnostic;
                const bool collarBuilt = lockedBuilder.build(
                    completePatch, report.result, directionField, *attempt.density,
                    boundaryLocked, boundaryDiagnostic);
                report.boundaryLockedCollarAttempted = true;
                report.boundaryLockedCollarSuccess = collarBuilt;
                bool conformed = false;
                bool validated = false;
                if (collarBuilt) {
                    boundaryLocked.boundaryLockedDiagnostic.requestedCoreTargetEdgeLength =
                        report.requestedCoreTargetEdgeLength;
                    boundaryLocked.boundaryLockedDiagnostic.effectiveInterfaceTargetEdgeLength =
                        report.effectiveInterfaceTargetEdgeLength;
                    boundaryLocked.boundaryLockedDiagnostic.innerSolveFaceCount = report.innerSolveFaceCount;
                    report.finalConformationAttempted = true;
                    conformed = autoRemesher.conformToSurface(
                        completePatch, boundaryLocked, boundaryDiagnostic);
                    report.finalConformationSuccess = conformed;
                    if (conformed) {
                        report.finalValidationAttempted = true;
                        validated = autoRemesher.validateResult(
                            completePatch, boundaryLocked, boundaryDiagnostic);
                        report.finalValidationSuccess = validated;
                    }
                }
                report.timings.collarBuildMilliseconds =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - transitionStart).count();
                report.timings.totalMilliseconds += report.timings.collarBuildMilliseconds;
                finalValid = collarBuilt && conformed && validated;
                if (finalValid) {
                    report.result = std::move(boundaryLocked);
                    report.diagnosticMessage += " | " + boundaryDiagnostic;
                    break;
                }
                retryHistory << "Attempt " << (attemptIndex + 1U) << " ("
                             << attempt.reason << "): " << boundaryDiagnostic << ' ';
            }
            report.timings.patchBuildMilliseconds =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - patchStart).count();
            report.timings.totalMilliseconds += report.timings.patchBuildMilliseconds;
            if (finalValid) {
                solver::ComponentResult success = makePortableResult(report.result, report, false);
                success.status = solver::SolveStatus::Success;
                success.failureCode = solver::FailureCode::Success;
                output.components.push_back(std::move(success));
                anySuccess = true;
            } else {
                solver::ComponentResult failure;
                failure.componentId = component.componentId;
                failure.status = solver::SolveStatus::Failed;
                failure.failureCode = reportFailureCode(report);
                failure.failedStage = solver::failureCodeName(failure.failureCode);
                failure.retryCount = static_cast<unsigned int>(report.retryAttemptCount);
                failure.retryReason = report.retryReason;
                failure.diagnosticMessage = "Boundary-Locked retry matrix exhausted: " + retryHistory.str();
                failure.timings.patchBuildMilliseconds = report.timings.patchBuildMilliseconds;
                failure.timings.totalMilliseconds = report.timings.totalMilliseconds;
                output.components.push_back(std::move(failure));
                if (input.settings.retainDebugResults && !lastInnerDebug.rawVertices.empty()) {
                    lastInnerDebug.success = false;
                    lastInnerDebug.debugPreviewAvailable = true;
                    lastInnerDebug.debugInnerResultOnly = true;
                    solver::ComponentResult debug = makePortableResult(lastInnerDebug, report, true);
                    debug.debugOnly = true;
                    output.debugComponents.push_back(std::move(debug));
                }
            }
        }
        output.status = anySuccess ? solver::SolveStatus::Success : solver::SolveStatus::Failed;
        output.failureCode = anySuccess ? solver::FailureCode::Success :
            (output.components.empty() ? solver::FailureCode::UnknownFailure
                                       : output.components.front().failureCode);
    } catch (const std::exception& exception) {
        output.status = solver::SolveStatus::Failed;
        output.failureCode = solver::FailureCode::UnknownFailure;
        output.warnings.push_back(std::string("Unhandled legacy solver exception: ") + exception.what());
    } catch (...) {
        output.status = solver::SolveStatus::Failed;
        output.failureCode = solver::FailureCode::UnknownFailure;
        output.warnings.push_back("Unhandled non-standard legacy solver exception.");
    }
    output.timings.totalMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - totalStart).count();
    return output;
}

}  // namespace directional_retopo
