#include "Remesh/BoundaryConformer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace directional_retopo {
namespace {

using Edge = std::pair<std::size_t, std::size_t>;

struct CurveSummary final
{
    MPoint center;
    double length = 0.0;
    bool closed = false;
    bool valid = false;
};

Edge orderedEdge(std::size_t first, std::size_t second)
{
    return first < second ? Edge(first, second) : Edge(second, first);
}

double polylineLength(
    const std::vector<MPoint>& positions,
    const ResultBoundaryLoop& loop)
{
    if (loop.vertexIndices.size() < 2U) {
        return 0.0;
    }
    double length = 0.0;
    const std::size_t edgeCount = loop.closed
        ? loop.vertexIndices.size()
        : loop.vertexIndices.size() - 1U;
    for (std::size_t edgeIndex = 0U; edgeIndex < edgeCount; ++edgeIndex) {
        const std::size_t next =
            (edgeIndex + 1U) % loop.vertexIndices.size();
        length += (positions[loop.vertexIndices[next]] -
            positions[loop.vertexIndices[edgeIndex]]).length();
    }
    return length;
}

std::vector<ResultBoundaryLoop> extractResultBoundaryLoops(
    const std::vector<MPoint>& positions,
    const std::vector<std::vector<std::size_t>>& polygons)
{
    std::map<Edge, std::size_t> edgeUseCount;
    for (const std::vector<std::size_t>& polygon : polygons) {
        for (std::size_t index = 0U; index < polygon.size(); ++index) {
            const std::size_t first = polygon[index];
            const std::size_t second = polygon[(index + 1U) % polygon.size()];
            if (first < positions.size() && second < positions.size() &&
                first != second) {
                ++edgeUseCount[orderedEdge(first, second)];
            }
        }
    }

    std::unordered_map<std::size_t, std::vector<std::size_t>> adjacency;
    std::set<Edge> remaining;
    for (const auto& [edge, useCount] : edgeUseCount) {
        if (useCount != 1U) {
            continue;
        }
        adjacency[edge.first].push_back(edge.second);
        adjacency[edge.second].push_back(edge.first);
        remaining.insert(edge);
    }
    for (auto& [vertex, neighbors] : adjacency) {
        (void)vertex;
        std::sort(neighbors.begin(), neighbors.end());
    }

    std::vector<ResultBoundaryLoop> loops;
    while (!remaining.empty()) {
        const Edge seed = *remaining.begin();
        std::size_t start = seed.first;
        if (adjacency[seed.first].size() != 1U &&
            adjacency[seed.second].size() == 1U) {
            start = seed.second;
        }
        ResultBoundaryLoop loop;
        loop.vertexIndices.push_back(start);
        std::size_t previous = std::numeric_limits<std::size_t>::max();
        std::size_t current = start;
        while (true) {
            std::size_t next = std::numeric_limits<std::size_t>::max();
            for (const std::size_t candidate : adjacency[current]) {
                if (candidate != previous &&
                    remaining.find(orderedEdge(current, candidate)) !=
                        remaining.end()) {
                    next = candidate;
                    break;
                }
            }
            if (next == std::numeric_limits<std::size_t>::max()) {
                break;
            }
            remaining.erase(orderedEdge(current, next));
            previous = current;
            current = next;
            if (current == start) {
                loop.closed = true;
                break;
            }
            loop.vertexIndices.push_back(current);
        }
        loop.totalLength = polylineLength(positions, loop);
        loops.push_back(std::move(loop));
    }
    return loops;
}

CurveSummary sourceSummary(
    const TriangulatedPatch& patch,
    const PatchBoundaryLoop& loop)
{
    CurveSummary summary;
    summary.closed = loop.closed;
    std::vector<MPoint> points;
    points.reserve(loop.vertexIndices.size());
    for (const std::size_t index : loop.vertexIndices) {
        if (index >= patch.vertices.size()) {
            return summary;
        }
        points.push_back(patch.vertices[index].position);
    }
    if (summary.closed && points.size() > 1U &&
        (points.front() - points.back()).length() <= 1.0e-12) {
        points.pop_back();
    }
    if (points.size() < 2U) {
        return summary;
    }
    for (const MPoint& point : points) {
        summary.center.x += point.x;
        summary.center.y += point.y;
        summary.center.z += point.z;
    }
    const double inverseCount = 1.0 / static_cast<double>(points.size());
    summary.center.x *= inverseCount;
    summary.center.y *= inverseCount;
    summary.center.z *= inverseCount;
    const std::size_t edgeCount = summary.closed
        ? points.size()
        : points.size() - 1U;
    for (std::size_t edgeIndex = 0U; edgeIndex < edgeCount; ++edgeIndex) {
        summary.length +=
            (points[(edgeIndex + 1U) % points.size()] - points[edgeIndex]).length();
    }
    summary.valid = std::isfinite(summary.length) && summary.length > 0.0;
    return summary;
}

CurveSummary resultSummary(
    const std::vector<MPoint>& positions,
    const ResultBoundaryLoop& loop)
{
    CurveSummary summary;
    summary.closed = loop.closed;
    if (loop.vertexIndices.size() < 2U) {
        return summary;
    }
    for (const std::size_t index : loop.vertexIndices) {
        if (index >= positions.size()) {
            return summary;
        }
        summary.center.x += positions[index].x;
        summary.center.y += positions[index].y;
        summary.center.z += positions[index].z;
    }
    const double inverseCount =
        1.0 / static_cast<double>(loop.vertexIndices.size());
    summary.center.x *= inverseCount;
    summary.center.y *= inverseCount;
    summary.center.z *= inverseCount;
    summary.length = polylineLength(positions, loop);
    summary.valid = std::isfinite(summary.length) && summary.length > 0.0;
    return summary;
}

}  // namespace

const BoundaryConformerSettings& BoundaryConformer::settings() const noexcept
{
    return settings_;
}

void BoundaryConformer::setSettings(
    const BoundaryConformerSettings& settings) noexcept
{
    settings_ = settings;
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 0.0);
    OrderedBoundaryCorrespondenceSettings orderedSettings =
        orderedCorrespondence_.settings();
    orderedSettings.geometryEpsilon = settings_.geometryEpsilon;
    orderedCorrespondence_.setSettings(orderedSettings);
}

bool BoundaryConformer::conform(
    const TriangulatedPatch& patch,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    result.boundaryCorrespondences.clear();
    result.boundaryLoops = extractResultBoundaryLoops(
        result.conformedVertices,
        result.polygons);
    if (patch.boundaryLoops.empty() || result.boundaryLoops.empty()) {
        diagnostic = "Source or Result Boundary contains no usable loops/chains.";
        return false;
    }
    if (patch.boundaryLoops.size() != result.boundaryLoops.size()) {
        std::ostringstream message;
        message << "Source/Result Boundary loop count mismatch ("
                << patch.boundaryLoops.size() << " vs "
                << result.boundaryLoops.size() << ").";
        diagnostic = message.str();
        return false;
    }

    std::vector<CurveSummary> sourceSummaries;
    sourceSummaries.reserve(patch.boundaryLoops.size());
    for (const PatchBoundaryLoop& loop : patch.boundaryLoops) {
        sourceSummaries.push_back(sourceSummary(patch, loop));
    }
    std::vector<bool> sourceUsed(sourceSummaries.size(), false);

    for (std::size_t resultLoopIndex = 0U;
         resultLoopIndex < result.boundaryLoops.size();
         ++resultLoopIndex) {
        const CurveSummary currentResult = resultSummary(
            result.conformedVertices,
            result.boundaryLoops[resultLoopIndex]);
        if (!currentResult.valid) {
            diagnostic = "Result Boundary summary is invalid.";
            return false;
        }

        std::size_t bestSource = std::numeric_limits<std::size_t>::max();
        double bestCost = std::numeric_limits<double>::infinity();
        for (std::size_t sourceIndex = 0U;
             sourceIndex < sourceSummaries.size();
             ++sourceIndex) {
            const CurveSummary& source = sourceSummaries[sourceIndex];
            if (sourceUsed[sourceIndex] || !source.valid ||
                source.closed != currentResult.closed) {
                continue;
            }
            const double lengthScale = std::max(
                source.length,
                settings_.geometryEpsilon);
            const double centerCost =
                (source.center - currentResult.center).length() / lengthScale;
            const double lengthCost =
                std::abs(source.length - currentResult.length) / lengthScale;
            const double cost = centerCost + lengthCost;
            if (cost < bestCost) {
                bestCost = cost;
                bestSource = sourceIndex;
            }
        }
        if (bestSource == std::numeric_limits<std::size_t>::max()) {
            diagnostic = "No closed/open-compatible Source Boundary was available.";
            return false;
        }

        BoundaryLoopCorrespondence correspondence;
        std::string orderedDiagnostic;
        if (!orderedCorrespondence_.solve(
                patch,
                bestSource,
                result.boundaryLoops[resultLoopIndex],
                result.conformedVertices,
                correspondence,
                orderedDiagnostic)) {
            std::ostringstream message;
            message << "Ordered Boundary correspondence failed for Result loop "
                    << resultLoopIndex << ": " << orderedDiagnostic;
            diagnostic = message.str();
            return false;
        }
        sourceUsed[bestSource] = true;
        correspondence.resultLoopIndex = resultLoopIndex;
        result.boundaryCorrespondences.push_back(std::move(correspondence));
    }

    const bool complete = result.boundaryCorrespondences.size() ==
        patch.boundaryLoops.size();
    std::ostringstream message;
    message << "Ordered Boundary conformation matched "
            << result.boundaryCorrespondences.size() << '/'
            << result.boundaryLoops.size()
            << " loops/chains without changing topology.";
    diagnostic = message.str();
    return complete;
}

}  // namespace directional_retopo
