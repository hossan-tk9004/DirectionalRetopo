#include "Remesh/QuadResultValidator.h"

#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace directional_retopo {
namespace {

using Edge = std::pair<std::size_t, std::size_t>;

Edge orderedEdge(std::size_t first, std::size_t second)
{
    return first < second ? Edge(first, second) : Edge(second, first);
}

bool finitePoint(const MPoint& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
}

double polygonArea(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& polygon)
{
    if (polygon.size() < 3U) {
        return 0.0;
    }
    MVector areaNormal = MVector::zero;
    const MPoint& origin = vertices[polygon.front()];
    for (std::size_t index = 1; index + 1U < polygon.size(); ++index) {
        areaNormal += (vertices[polygon[index]] - origin) ^
            (vertices[polygon[index + 1U]] - origin);
    }
    return areaNormal.length() * 0.5;
}

double loopLength(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& indices,
    bool closed)
{
    if (indices.size() < 2U) {
        return 0.0;
    }
    double length = 0.0;
    for (std::size_t index = 1; index < indices.size(); ++index) {
        length += (vertices[indices[index]] - vertices[indices[index - 1U]]).length();
    }
    if (closed && indices.front() != indices.back()) {
        length += (vertices[indices.front()] - vertices[indices.back()]).length();
    }
    return length;
}

struct BoundaryExtractionResult final
{
    bool success = false;
    std::vector<ResultBoundaryLoop> loops;
    std::string diagnostic;
};

BoundaryExtractionResult extractBoundaryLoopsValidated(
    const std::vector<MPoint>& vertices,
    const std::map<Edge, std::size_t>& edgeUseCount)
{
    BoundaryExtractionResult result;
    std::unordered_map<std::size_t, std::vector<std::size_t>> adjacency;
    std::set<Edge> remaining;
    for (const auto& [edge, count] : edgeUseCount) {
        if (count != 1U) {
            continue;
        }
        adjacency[edge.first].push_back(edge.second);
        adjacency[edge.second].push_back(edge.first);
        remaining.insert(edge);
    }

    for (auto& [vertex, neighbors] : adjacency) {
        (void)vertex;
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(
            std::unique(neighbors.begin(), neighbors.end()),
            neighbors.end());
        if (neighbors.size() > 2U) {
            result.diagnostic =
                "Inner result boundary is branched/non-manifold: "
                "a boundary vertex has degree greater than two.";
            return result;
        }
    }

    while (!remaining.empty()) {
        const Edge seed = *remaining.begin();
        std::unordered_set<std::size_t> componentVertices;
        std::vector<std::size_t> stack = {seed.first};
        while (!stack.empty()) {
            const std::size_t vertex = stack.back();
            stack.pop_back();
            if (!componentVertices.insert(vertex).second) {
                continue;
            }
            const auto found = adjacency.find(vertex);
            if (found == adjacency.end()) {
                continue;
            }
            for (const std::size_t neighbor : found->second) {
                stack.push_back(neighbor);
            }
        }

        std::vector<std::size_t> endpoints;
        for (const std::size_t vertex : componentVertices) {
            const std::size_t degree = adjacency[vertex].size();
            if (degree == 1U) {
                endpoints.push_back(vertex);
            } else if (degree != 2U) {
                result.diagnostic =
                    "Inner result boundary contains an isolated or "
                    "non-manifold boundary vertex.";
                return result;
            }
        }
        if (!endpoints.empty() && endpoints.size() != 2U) {
            result.diagnostic =
                "Inner result boundary is branched/non-manifold: "
                "an open chain must have exactly two endpoints.";
            return result;
        }

        const std::size_t start =
            endpoints.empty() ? seed.first : endpoints.front();
        ResultBoundaryLoop loop;
        loop.vertexIndices.push_back(start);
        std::unordered_set<std::size_t> traversedVertices = {start};
        std::size_t previous = std::numeric_limits<std::size_t>::max();
        std::size_t current = start;
        while (true) {
            std::vector<std::size_t> available;
            const auto found = adjacency.find(current);
            if (found != adjacency.end()) {
                for (const std::size_t candidate : found->second) {
                    if (remaining.find(orderedEdge(current, candidate)) !=
                        remaining.end()) {
                        available.push_back(candidate);
                    }
                }
            }
            if (available.empty()) {
                break;
            }
            if (available.size() > 1U &&
                previous != std::numeric_limits<std::size_t>::max()) {
                result.diagnostic =
                    "Inner result boundary traversal became ambiguous.";
                return result;
            }
            const std::size_t next = available.front();
            remaining.erase(orderedEdge(current, next));
            previous = current;
            current = next;
            if (current == start) {
                loop.closed = true;
                break;
            }
            if (!traversedVertices.insert(current).second) {
                result.diagnostic =
                    "Inner result boundary contains a repeated traversal vertex.";
                return result;
            }
            loop.vertexIndices.push_back(current);
        }

        if (endpoints.empty() && !loop.closed) {
            result.diagnostic =
                "Inner result closed boundary traversal did not return to its start.";
            return result;
        }
        if (!endpoints.empty() && loop.closed) {
            result.diagnostic =
                "Inner result open boundary unexpectedly formed a closed traversal.";
            return result;
        }
        for (const Edge& edge : remaining) {
            if (componentVertices.find(edge.first) != componentVertices.end()) {
                result.diagnostic =
                    "Inner result boundary requires multiple traversal paths.";
                return result;
            }
        }
        if (loop.closed) {
            for (const std::size_t vertex : loop.vertexIndices) {
                if (adjacency[vertex].size() != 2U) {
                    result.diagnostic =
                        "Inner result closed boundary vertex degree is not two.";
                    return result;
                }
            }
        }
        loop.totalLength = loopLength(vertices, loop.vertexIndices, loop.closed);
        result.loops.push_back(std::move(loop));
    }

    result.success = true;
    result.diagnostic =
        "Inner result boundary topology is ordered, unbranched, and manifold.";
    return result;
}

}  // namespace

const QuadResultValidatorSettings& QuadResultValidator::settings() const noexcept
{
    return settings_;
}

void QuadResultValidator::setSettings(
    const QuadResultValidatorSettings& settings) noexcept
{
    settings_ = settings;
    settings_.areaEpsilon = std::max(settings_.areaEpsilon, 0.0);
    settings_.surfaceProximityTargetLengthMultiplier =
        std::max(settings_.surfaceProximityTargetLengthMultiplier, 0.0);
    settings_.minimumSurfaceTolerance =
        std::max(settings_.minimumSurfaceTolerance, 0.0);
}

bool QuadResultValidator::validate(
    const TriangulatedPatch& patch,
    QuadPatchResult& result) const
{
    result.success = false;
    result.quadCount = 0;
    result.nonQuadCount = 0;
    result.triangleCount = 0;
    result.nGonCount = 0;
    result.maximumSurfaceDistance = 0.0;
    result.boundaryDiagnostic = BoundaryComparisonDiagnostic();
    if (result.rawVertices.empty() || result.conformedVertices.empty() ||
        result.rawVertices.size() != result.conformedVertices.size() ||
        result.polygons.empty()) {
        result.diagnosticMessage = "Quad result contains no vertices or polygons.";
        return false;
    }
    for (const MPoint& vertex : result.conformedVertices) {
        if (!finitePoint(vertex)) {
            result.diagnosticMessage = "Quad result contains a non-finite vertex.";
            return false;
        }
    }

    std::map<Edge, std::size_t> edgeUseCount;
    for (const std::vector<std::size_t>& polygon : result.polygons) {
        if (polygon.size() < 3U) {
            result.diagnosticMessage = "Quad result contains a polygon with fewer than three vertices.";
            return false;
        }
        std::unordered_set<std::size_t> uniqueIndices;
        for (const std::size_t vertexIndex : polygon) {
            if (vertexIndex >= result.conformedVertices.size() ||
                !uniqueIndices.insert(vertexIndex).second) {
                result.diagnosticMessage =
                    "Quad result contains an invalid or repeated polygon vertex index.";
                return false;
            }
        }
        if (polygonArea(result.conformedVertices, polygon) <= settings_.areaEpsilon) {
            result.diagnosticMessage = "Quad result contains an obvious zero-area polygon.";
            return false;
        }
        for (std::size_t index = 0; index < polygon.size(); ++index) {
            const Edge edge = orderedEdge(
                polygon[index],
                polygon[(index + 1U) % polygon.size()]);
            if (++edgeUseCount[edge] > 2U) {
                result.diagnosticMessage = "Quad result contains non-manifold edge connectivity.";
                return false;
            }
        }
        if (polygon.size() == 4U) {
            ++result.quadCount;
        } else if (polygon.size() == 3U) {
            ++result.triangleCount;
        } else {
            ++result.nGonCount;
            if (result.boundaryLocked) {
                result.diagnosticMessage =
                    "Boundary-Locked Quad-Dominant result contains an unintended N-gon.";
                return false;
            }
        }
    result.nonQuadCount = result.triangleCount + result.nGonCount;
    }
    BoundaryExtractionResult boundaryExtraction =
        extractBoundaryLoopsValidated(
        result.conformedVertices,
        edgeUseCount);
    if (!boundaryExtraction.success) {
        result.diagnosticMessage = boundaryExtraction.diagnostic;
        return false;
    }
    std::vector<ResultBoundaryLoop> extractedBoundaryLoops =
        std::move(boundaryExtraction.loops);

    if (result.boundaryCorrespondences.empty() || result.boundaryLoops.empty()) {
        result.boundaryLoops = std::move(extractedBoundaryLoops);
    } else if (result.boundaryLoops.size() != extractedBoundaryLoops.size()) {
        result.diagnosticMessage =
            "Ordered Result Boundary loop count changed during validation.";
        return false;
    } else {
        for (ResultBoundaryLoop& loop : result.boundaryLoops) {
            loop.totalLength = loopLength(
                result.conformedVertices,
                loop.vertexIndices,
                loop.closed);
        }
    }

    if (result.sourceMappings.size() != result.conformedVertices.size()) {
        result.diagnosticMessage =
            "Surface Conformer source mappings are missing or incomplete.";
        return false;
    }
    for (const ResultVertexSourceMapping& sourceMapping : result.sourceMappings) {
        if (sourceMapping.patchTriangleIndex >= patch.triangles.size() ||
            sourceMapping.sourceFaceId < 0 ||
            !std::isfinite(sourceMapping.surfaceDistance)) {
            result.diagnosticMessage =
                "Surface Conformer returned an invalid source mapping.";
            return false;
        }
        result.maximumSurfaceDistance = std::max(
            result.maximumSurfaceDistance,
            sourceMapping.surfaceDistance);
    }
    const double surfaceTolerance = std::max(
        result.targetEdgeLength * settings_.surfaceProximityTargetLengthMultiplier,
        settings_.minimumSurfaceTolerance);
    if (result.maximumSurfaceDistance > surfaceTolerance) {
        std::ostringstream message;
        message << "Quad result is too far from the source surface (max "
                << result.maximumSurfaceDistance << ", tolerance "
                << surfaceTolerance << ").";
        result.diagnosticMessage = message.str();
        return false;
    }

    BoundaryComparisonDiagnostic& diagnostic = result.boundaryDiagnostic;
    diagnostic.sourceLoopCount = patch.boundaryLoops.size();
    diagnostic.resultLoopCount = result.boundaryLoops.size();
    std::vector<MPoint> patchPositions;
    patchPositions.reserve(patch.vertices.size());
    for (const PatchVertex& vertex : patch.vertices) {
        patchPositions.push_back(vertex.position);
    }
    for (const PatchBoundaryLoop& loop : patch.boundaryLoops) {
        diagnostic.sourceVertexCount += loop.vertexIndices.size();
        diagnostic.sourceEdgeCount += loop.vertexIndices.size() < 2U
            ? 0U
            : (loop.closed
                ? loop.vertexIndices.size()
                : loop.vertexIndices.size() - 1U);
        diagnostic.sourceTotalLength += loopLength(
            patchPositions,
            loop.vertexIndices,
            loop.closed);
    }
    for (const ResultBoundaryLoop& loop : result.boundaryLoops) {
        diagnostic.resultVertexCount += loop.vertexIndices.size();
        diagnostic.resultEdgeCount += loop.vertexIndices.size() < 2U
            ? 0U
            : (loop.closed
                ? loop.vertexIndices.size()
                : loop.vertexIndices.size() - 1U);
        diagnostic.resultTotalLength += loop.totalLength;
    }
    diagnostic.vertexCountDifference =
        static_cast<long long>(diagnostic.resultVertexCount) -
        static_cast<long long>(diagnostic.sourceVertexCount);

    if (!patch.boundaryLoops.empty() && !result.boundaryLoops.empty() &&
        result.boundaryCorrespondences.empty()) {
        result.diagnosticMessage =
            "Result Boundary exists but no ordered Source Boundary correspondence was produced.";
        return false;
    }

    double boundaryDistanceSum = 0.0;
    std::size_t boundarySampleCount = 0;
    for (BoundaryLoopCorrespondence& correspondence :
         result.boundaryCorrespondences) {
        if (correspondence.sourceLoopIndex >= patch.boundaryLoops.size() ||
            correspondence.resultLoopIndex >= result.boundaryLoops.size()) {
            result.diagnosticMessage =
                "Boundary correspondence contains an invalid loop index.";
            return false;
        }
        const ResultBoundaryLoop& loop =
            result.boundaryLoops[correspondence.resultLoopIndex];
        if (!correspondence.closedStateMatches ||
            !correspondence.windingAlignedAfterConformation ||
            !correspondence.orderedMappingValid ||
            correspondence.vertices.size() != loop.vertexIndices.size()) {
            result.diagnosticMessage =
                "Boundary correspondence is not a complete ordered/winding-aligned mapping.";
            return false;
        }
        std::unordered_set<std::size_t> loopVertices(
            loop.vertexIndices.begin(),
            loop.vertexIndices.end());
        double correspondenceDistanceSum = 0.0;
        correspondence.maximumDistanceAfter = 0.0;
        double previousUnwrappedParameter = -std::numeric_limits<double>::infinity();
        for (BoundaryVertexCorrespondence& vertex : correspondence.vertices) {
            if (vertex.resultVertexIndex >= result.conformedVertices.size() ||
                loopVertices.find(vertex.resultVertexIndex) == loopVertices.end() ||
                !finitePoint(vertex.sourcePosition) ||
                !std::isfinite(vertex.resultNormalizedParameter) ||
                !std::isfinite(vertex.sourceNormalizedParameter) ||
                !std::isfinite(vertex.sourceUnwrappedParameter) ||
                !std::isfinite(vertex.sourceEdgeParameter) ||
                vertex.resultNormalizedParameter < -settings_.areaEpsilon ||
                vertex.resultNormalizedParameter > 1.0 + settings_.areaEpsilon ||
                vertex.sourceNormalizedParameter < -settings_.areaEpsilon ||
                vertex.sourceNormalizedParameter > 1.0 + settings_.areaEpsilon ||
                vertex.sourceUnwrappedParameter < -settings_.areaEpsilon ||
                vertex.sourceUnwrappedParameter > 1.0 + settings_.areaEpsilon ||
                vertex.sourceEdgeParameter < -settings_.areaEpsilon ||
                vertex.sourceEdgeParameter > 1.0 + settings_.areaEpsilon ||
                vertex.sourceVertex0 < 0 || vertex.sourceVertex1 < 0) {
                result.diagnosticMessage =
                    "Boundary correspondence contains invalid normalized data.";
                return false;
            }
            if (vertex.sourceUnwrappedParameter <=
                previousUnwrappedParameter + settings_.areaEpsilon) {
                result.diagnosticMessage =
                    "Boundary correspondence source parameters are not strictly monotonic.";
                return false;
            }
            previousUnwrappedParameter = vertex.sourceUnwrappedParameter;
            vertex.distanceAfterConformation =
                (result.conformedVertices[vertex.resultVertexIndex] -
                 vertex.sourcePosition).length();
            if (!std::isfinite(vertex.distanceAfterConformation)) {
                result.diagnosticMessage =
                    "Boundary correspondence contains a non-finite distance.";
                return false;
            }
            correspondenceDistanceSum += vertex.distanceAfterConformation;
            correspondence.maximumDistanceAfter = std::max(
                correspondence.maximumDistanceAfter,
                vertex.distanceAfterConformation);
            boundaryDistanceSum += vertex.distanceAfterConformation;
            diagnostic.maximumNearestDistance = std::max(
                diagnostic.maximumNearestDistance,
                vertex.distanceAfterConformation);
            ++boundarySampleCount;
        }
        correspondence.meanDistanceAfter = correspondence.vertices.empty()
            ? 0.0
            : correspondenceDistanceSum /
                static_cast<double>(correspondence.vertices.size());
        correspondence.resultTotalArcLengthAfter = loop.totalLength;
        diagnostic.monotonicViolationCount +=
            correspondence.monotonicViolationCount;
        diagnostic.crossingCount +=
            correspondence.selfIntersectionCount +
            correspondence.sourceCrossingCount;
        diagnostic.requiredBoundaryAnchorCount +=
            correspondence.requiredBoundaryAnchors.size();
        for (const RequiredBoundaryAnchor& anchor :
             correspondence.requiredBoundaryAnchors) {
            if (anchor.requiresResultSplit) {
                ++diagnostic.requiredResultSplitCount;
            }
        }
        diagnostic.closedStateMatches = diagnostic.closedStateMatches &&
            correspondence.closedStateMatches;
        if (correspondence.winding == BoundaryWinding::Aligned) {
            ++diagnostic.alignedLoopCount;
        } else if (correspondence.winding == BoundaryWinding::Reversed) {
            ++diagnostic.reversedLoopCount;
        }
    }
    diagnostic.meanNearestDistance = boundarySampleCount > 0U
        ? boundaryDistanceSum / static_cast<double>(boundarySampleCount)
        : 0.0;
    diagnostic.orientationAligned = std::all_of(
        result.boundaryCorrespondences.begin(),
        result.boundaryCorrespondences.end(),
        [](const BoundaryLoopCorrespondence& correspondence) {
            return correspondence.windingAlignedAfterConformation;
        });
    diagnostic.correspondenceComplete =
        result.boundaryCorrespondences.size() == patch.boundaryLoops.size() &&
        result.boundaryCorrespondences.size() == result.boundaryLoops.size();
    const bool hasResultBoundary = !result.boundaryLoops.empty();
    diagnostic.orderedMappingValid = !hasResultBoundary ||
        (diagnostic.correspondenceComplete &&
         diagnostic.orientationAligned &&
         diagnostic.closedStateMatches &&
         diagnostic.monotonicViolationCount == 0U &&
         diagnostic.crossingCount == 0U);
    if (!diagnostic.orderedMappingValid) {
        std::ostringstream message;
        message << "Boundary correspondence failed ordered/crossing validation"
                << " (complete=" << diagnostic.correspondenceComplete
                << ", loops=" << result.boundaryCorrespondences.size()
                << '/' << patch.boundaryLoops.size()
                << '/' << result.boundaryLoops.size()
                << ", winding=" << diagnostic.orientationAligned
                << ", closed=" << diagnostic.closedStateMatches
                << ", monotonic=" << diagnostic.monotonicViolationCount
                << ", crossing=" << diagnostic.crossingCount << ").";
        result.diagnosticMessage = message.str();
        return false;
    }

    result.success = true;
    std::ostringstream successMessage;
    successMessage << "Validated " << result.conformedVertices.size()
                   << " Conformed vertices and "
                   << result.polygons.size() << " polygons; max surface distance "
                   << result.maximumSurfaceDistance << ".";
    result.diagnosticMessage = successMessage.str();
    return true;
}

}  // namespace directional_retopo
