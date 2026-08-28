#include "Remesh/BoundaryLockedPatchBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>

namespace directional_retopo {
namespace {
class VertexDisjointSet final
{
public:
    explicit VertexDisjointSet(std::size_t size)
        : parent_(size), rank_(size, 0U)
    {
        std::iota(parent_.begin(), parent_.end(), 0U);
    }

    std::size_t find(std::size_t value)
    {
        while (parent_[value] != value) {
            parent_[value] = parent_[parent_[value]];
            value = parent_[value];
        }
        return value;
    }

    void unite(std::size_t first, std::size_t second)
    {
        first = find(first);
        second = find(second);
        if (first == second) {
            return;
        }
        if (rank_[first] < rank_[second]) {
            std::swap(first, second);
        }
        parent_[second] = first;
        if (rank_[first] == rank_[second]) {
            ++rank_[first];
        }
    }

private:
    std::vector<std::size_t> parent_;
    std::vector<unsigned char> rank_;
};

std::vector<std::size_t> polygonVertexComponents(
    const QuadPatchResult& result)
{
    const std::size_t invalid = std::numeric_limits<std::size_t>::max();
    VertexDisjointSet components(result.rawVertices.size());
    std::vector<bool> active(result.rawVertices.size(), false);
    for (const std::vector<std::size_t>& polygon : result.polygons) {
        if (polygon.empty() || polygon.front() >= result.rawVertices.size()) {
            continue;
        }
        active[polygon.front()] = true;
        for (std::size_t index = 1U; index < polygon.size(); ++index) {
            if (polygon[index] >= result.rawVertices.size()) {
                continue;
            }
            active[polygon[index]] = true;
            components.unite(polygon.front(), polygon[index]);
        }
    }
    std::vector<std::size_t> resultComponents(
        result.rawVertices.size(), invalid);
    for (std::size_t index = 0U; index < resultComponents.size(); ++index) {
        if (active[index]) {
            resultComponents[index] = components.find(index);
        }
    }
    return resultComponents;
}

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

double approximateLoopArea(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& loop)
{
    if (loop.size() < 3U) {
        return 0.0;
    }
    MVector areaNormal = MVector::zero;
    const MPoint& origin = vertices[loop.front()];
    for (std::size_t index = 1U; index + 1U < loop.size(); ++index) {
        areaNormal += (vertices[loop[index]] - origin) ^
            (vertices[loop[index + 1U]] - origin);
    }
    return areaNormal.length() * 0.5;
}

double loopMatchCost(
    const std::vector<MPoint>& sourceVertices,
    const std::vector<std::size_t>& sourceLoop,
    const std::vector<MPoint>& innerVertices,
    const ResultBoundaryLoop& innerLoop,
    double epsilon)
{
    const double sourceLength =
        loopLength(sourceVertices, sourceLoop, true);
    const double innerLength =
        loopLength(innerVertices, innerLoop.vertexIndices, innerLoop.closed);
    const double scale = std::max(sourceLength, epsilon);
    const MVector centerDelta =
        centroid(innerVertices, innerLoop.vertexIndices) -
        centroid(sourceVertices, sourceLoop);
    const double centerCost = (centerDelta * centerDelta) / (scale * scale);
    const double perimeterCost = std::abs(std::log(
        std::max(innerLength, epsilon) / scale));
    const double sourceArea = approximateLoopArea(sourceVertices, sourceLoop);
    const double innerArea = approximateLoopArea(
        innerVertices,
        innerLoop.vertexIndices);
    const double areaCost = sourceArea > epsilon && innerArea > epsilon
        ? std::abs(std::log(innerArea / sourceArea))
        : 0.0;
    return centerCost + 0.35 * perimeterCost + 0.15 * areaCost;
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
    settings_.topologyBlendWidth = std::max(settings_.topologyBlendWidth, 1U);
    settings_.maximumRepairHoleVertexCount =
        std::max<std::size_t>(settings_.maximumRepairHoleVertexCount, 3U);
    settings_.maximumRepairHolePerimeterRatio = std::clamp(
        settings_.maximumRepairHolePerimeterRatio, 0.0, 1.0);
    settings_.maximumRepairHoleAreaRatio = std::clamp(
        settings_.maximumRepairHoleAreaRatio, 0.0, 1.0);
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
    if (completeSourcePatch.boundaryLoops.empty()) {
        diagnostic = "Fixed Source Patch contains no ordered Boundary loop.";
        return false;
    }
    if (innerRemeshResult.boundaryLoops.empty()) {
        diagnostic = "Inner Remesh Result contains no ordered Boundary loop.";
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

    auto& locked = result.boundaryLockedDiagnostic;
    locked.rawInnerBoundaryLoopCount =
        innerRemeshResult.boundaryLoops.size();
    locked.requestedCoreTargetEdgeLength =
        innerRemeshResult.targetEdgeLength;
    locked.effectiveInterfaceTargetEdgeLength =
        innerRemeshResult.targetEdgeLength;

    std::vector<MPoint> sourcePositions;
    sourcePositions.reserve(completeSourcePatch.vertices.size());
    for (const PatchVertex& vertex : completeSourcePatch.vertices) {
        sourcePositions.push_back(vertex.position);
    }

    const std::vector<std::size_t> innerVertexComponents =
        polygonVertexComponents(result);
    std::unordered_set<std::size_t> primaryComponents;
    std::size_t primarySourceLoopIndex = 0U;
    double largestSourceLoopArea = -1.0;
    for (std::size_t loopIndex = 0U;
         loopIndex < completeSourcePatch.boundaryLoops.size();
         ++loopIndex) {
        std::vector<std::size_t> cycle =
            completeSourcePatch.boundaryLoops[loopIndex].vertexIndices;
        if (cycle.size() > 1U && cycle.front() == cycle.back()) {
            cycle.pop_back();
        }
        const double area = approximateLoopArea(sourcePositions, cycle);
        if (area > largestSourceLoopArea) {
            largestSourceLoopArea = area;
            primarySourceLoopIndex = loopIndex;
        }
    }
    std::vector<bool> innerUsed(innerRemeshResult.boundaryLoops.size(), false);
    std::vector<std::size_t> matchedInner(
        completeSourcePatch.boundaryLoops.size(),
        std::numeric_limits<std::size_t>::max());
    for (std::size_t sourceLoopIndex = 0U;
         sourceLoopIndex < completeSourcePatch.boundaryLoops.size();
         ++sourceLoopIndex) {
        const PatchBoundaryLoop& sourceLoop =
            completeSourcePatch.boundaryLoops[sourceLoopIndex];
        std::vector<std::size_t> sourceCycle = sourceLoop.vertexIndices;
        if (sourceLoop.closed && sourceCycle.size() > 1U &&
            sourceCycle.front() == sourceCycle.back()) {
            sourceCycle.pop_back();
        }

        double bestCost = std::numeric_limits<double>::infinity();
        for (std::size_t candidate = 0U;
             candidate < innerRemeshResult.boundaryLoops.size();
             ++candidate) {
            const ResultBoundaryLoop& loop =
                innerRemeshResult.boundaryLoops[candidate];
            if (innerUsed[candidate] || !loop.closed ||
                loop.vertexIndices.size() < 3U) {
                continue;
            }
            const double cost = loopMatchCost(
                sourcePositions,
                sourceCycle,
                result.rawVertices,
                loop,
                settings_.geometryEpsilon);
            if (cost < bestCost) {
                bestCost = cost;
                matchedInner[sourceLoopIndex] = candidate;
            }
        }
        if (matchedInner[sourceLoopIndex] ==
            std::numeric_limits<std::size_t>::max()) {
            diagnostic =
                "No valid Primary Inner Boundary loop matches a Fixed Source loop.";
            return false;
        }
        innerUsed[matchedInner[sourceLoopIndex]] = true;
        const ResultBoundaryLoop& primary =
            innerRemeshResult.boundaryLoops[matchedInner[sourceLoopIndex]];
        std::unordered_set<std::size_t> loopComponents;
        for (const std::size_t vertex : primary.vertexIndices) {
            if (vertex >= innerVertexComponents.size() ||
                innerVertexComponents[vertex] ==
                    std::numeric_limits<std::size_t>::max()) {
                diagnostic =
                    "Primary Inner Boundary is not attached to result polygons.";
                return false;
            }
            loopComponents.insert(innerVertexComponents[vertex]);
        }
        if (loopComponents.size() != 1U) {
            diagnostic =
                "Primary Inner Boundary spans branched/disconnected components.";
            return false;
        }
        primaryComponents.insert(*loopComponents.begin());

        locked.innerLoopDiagnostics.push_back({
            matchedInner[sourceLoopIndex],
            sourceLoopIndex == primarySourceLoopIndex
                ? InnerBoundaryLoopClassification::PrimaryOuter
                : InnerBoundaryLoopClassification::SecondaryHole,
            primary.vertexIndices.size(),
            loopLength(result.rawVertices, primary.vertexIndices, primary.closed),
            approximateLoopArea(result.rawVertices, primary.vertexIndices),
            false});
        if (sourceLoopIndex == primarySourceLoopIndex) {
            ++locked.primaryInnerLoopCount;
        } else {
            ++locked.secondaryHoleLoopCount;
        }
    }

    const PatchBoundaryLoop& referenceSourceLoop =
        completeSourcePatch.boundaryLoops.front();
    std::vector<std::size_t> referenceSourceCycle =
        referenceSourceLoop.vertexIndices;
    if (referenceSourceLoop.closed && referenceSourceCycle.size() > 1U &&
        referenceSourceCycle.front() == referenceSourceCycle.back()) {
        referenceSourceCycle.pop_back();
    }
    const double referencePerimeter = loopLength(
        sourcePositions,
        referenceSourceCycle,
        true);
    const double referenceArea = approximateLoopArea(
        sourcePositions,
        referenceSourceCycle);

    for (std::size_t loopIndex = 0U;
         loopIndex < innerRemeshResult.boundaryLoops.size();
         ++loopIndex) {
        if (innerUsed[loopIndex]) {
            continue;
        }
        const ResultBoundaryLoop& loop =
            innerRemeshResult.boundaryLoops[loopIndex];
        const double perimeter = loopLength(
            result.rawVertices,
            loop.vertexIndices,
            loop.closed);
        const double area = approximateLoopArea(
            result.rawVertices,
            loop.vertexIndices);
        InnerBoundaryLoopDiagnostic loopDiagnostic;
        loopDiagnostic.loopIndex = loopIndex;
        loopDiagnostic.vertexCount = loop.vertexIndices.size();
        loopDiagnostic.perimeter = perimeter;
        loopDiagnostic.approximateArea = area;

        std::unordered_set<std::size_t> loopComponents;
        bool attachedToPolygons = true;
        for (const std::size_t vertex : loop.vertexIndices) {
            if (vertex >= innerVertexComponents.size() ||
                innerVertexComponents[vertex] ==
                    std::numeric_limits<std::size_t>::max()) {
                attachedToPolygons = false;
                break;
            }
            loopComponents.insert(innerVertexComponents[vertex]);
        }
        if (!attachedToPolygons ||
            loopComponents.size() != 1U ||
            primaryComponents.find(*loopComponents.begin()) ==
                primaryComponents.end()) {
            loopDiagnostic.classification =
                InnerBoundaryLoopClassification::InvalidOrBranched;
            locked.innerLoopDiagnostics.push_back(loopDiagnostic);
            std::ostringstream message;
            message << "Inner Remesh Result contains a disconnected or "
                    << "unattached result component at Boundary loop "
                    << loopIndex
                    << "; controlled compatibility retry is required.";
            diagnostic = message.str();
            return false;
        }

        const bool repairable =
            loop.closed &&
            loop.vertexIndices.size() >= 3U &&
            loop.vertexIndices.size() <=
                settings_.maximumRepairHoleVertexCount &&
            perimeter <= referencePerimeter *
                settings_.maximumRepairHolePerimeterRatio &&
            (referenceArea <= settings_.geometryEpsilon ||
             area <= referenceArea * settings_.maximumRepairHoleAreaRatio);
        if (!repairable) {
            loopDiagnostic.classification =
                loop.closed
                    ? InnerBoundaryLoopClassification::SecondaryHole
                    : InnerBoundaryLoopClassification::InvalidOrBranched;
            locked.innerLoopDiagnostics.push_back(loopDiagnostic);
            ++locked.secondaryHoleLoopCount;
            std::ostringstream message;
            message << "Inner Remesh Result contains a significant secondary "
                    << "Boundary loop (loop " << loopIndex << ", "
                    << loop.vertexIndices.size() << " vertices, perimeter "
                    << perimeter << "); controlled compatibility retry is required.";
            diagnostic = message.str();
            return false;
        }

        loopDiagnostic.classification =
            InnerBoundaryLoopClassification::TinyArtifactLoop;
        loopDiagnostic.repaired = true;
        ++locked.tinyArtifactLoopCount;
        ++locked.holeRepairCount;

        MPoint center(0.0, 0.0, 0.0);
        for (const std::size_t vertex : loop.vertexIndices) {
            center += MVector(
                result.rawVertices[vertex].x,
                result.rawVertices[vertex].y,
                result.rawVertices[vertex].z);
        }
        const double inverse =
            1.0 / static_cast<double>(loop.vertexIndices.size());
        center.x *= inverse;
        center.y *= inverse;
        center.z *= inverse;
        const std::size_t centerIndex = result.rawVertices.size();
        result.rawVertices.push_back(center);
        for (std::size_t edge = 0U;
             edge < loop.vertexIndices.size();
             ++edge) {
            const std::size_t polygonIndex = result.polygons.size();
            result.polygons.push_back({
                loop.vertexIndices[edge],
                loop.vertexIndices[(edge + 1U) % loop.vertexIndices.size()],
                centerIndex});
            result.polygonRegions.push_back(ResultPolygonRegion::Core);
            result.triangleDiagnostics.push_back({
                polygonIndex,
                TriangleReason::SmallHoleRepair});
            ++locked.coreTriangleCount;
        }
        locked.innerLoopDiagnostics.push_back(loopDiagnostic);
    }

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

        const std::size_t innerIndex = matchedInner[sourceLoopIndex];
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
        locked.seamCandidatesTested += collar.seamCandidatesTested;
        locked.dpFeasibleCandidateCount +=
            collar.dpFeasibleCandidateCount;
        locked.geometryValidCandidateCount +=
            collar.geometryValidCandidateCount;
        locked.rejectedZeroAreaCandidateCount +=
            collar.rejectedZeroAreaCandidateCount;
        locked.rejectedSliverCandidateCount +=
            collar.rejectedSliverCandidateCount;
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
