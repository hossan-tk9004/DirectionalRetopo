#include "Remesh/SurfaceConformer.h"

#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

namespace directional_retopo {
namespace {

using Edge = std::pair<std::size_t, std::size_t>;

struct SourceTriangle final
{
    MPoint a;
    MPoint b;
    MPoint c;
    MVector normal;
    int sourceFaceId = -1;
    std::vector<std::size_t> adjacentTriangleIndices;
};

struct ProjectionHit final
{
    MPoint point;
    MVector normal;
    std::size_t triangleIndex = std::numeric_limits<std::size_t>::max();
    int sourceFaceId = -1;
    double distance = std::numeric_limits<double>::infinity();
    bool valid = false;
};

Edge orderedEdge(std::size_t first, std::size_t second)
{
    return first < second ? Edge(first, second) : Edge(second, first);
}

bool finitePoint(const MPoint& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
}

MPoint closestPointOnTriangle(
    const MPoint& point,
    const MPoint& a,
    const MPoint& b,
    const MPoint& c)
{
    const MVector ab = b - a;
    const MVector ac = c - a;
    const MVector ap = point - a;
    const double d1 = ab * ap;
    const double d2 = ac * ap;
    if (d1 <= 0.0 && d2 <= 0.0) {
        return a;
    }

    const MVector bp = point - b;
    const double d3 = ab * bp;
    const double d4 = ac * bp;
    if (d3 >= 0.0 && d4 <= d3) {
        return b;
    }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        return a + ab * (d1 / (d1 - d3));
    }

    const MVector cp = point - c;
    const double d5 = ab * cp;
    const double d6 = ac * cp;
    if (d6 >= 0.0 && d5 <= d6) {
        return c;
    }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        return a + ac * (d2 / (d2 - d6));
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const MVector bc = c - b;
        return b + bc * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    }

    const double denominator = 1.0 / (va + vb + vc);
    return a + ab * (vb * denominator) + ac * (vc * denominator);
}

FidelityBounds bounds(const std::vector<MPoint>& points)
{
    FidelityBounds result;
    for (const MPoint& point : points) {
        if (!finitePoint(point)) {
            continue;
        }
        if (!result.valid) {
            result.minimum = point;
            result.maximum = point;
            result.valid = true;
            continue;
        }
        result.minimum.x = std::min(result.minimum.x, point.x);
        result.minimum.y = std::min(result.minimum.y, point.y);
        result.minimum.z = std::min(result.minimum.z, point.z);
        result.maximum.x = std::max(result.maximum.x, point.x);
        result.maximum.y = std::max(result.maximum.y, point.y);
        result.maximum.z = std::max(result.maximum.z, point.z);
    }
    return result;
}

double polygonArea(
    const std::vector<MPoint>& vertices,
    const std::vector<std::vector<std::size_t>>& polygons)
{
    double area = 0.0;
    for (const std::vector<std::size_t>& polygon : polygons) {
        if (polygon.size() < 3U || polygon.front() >= vertices.size()) {
            continue;
        }
        const MPoint& origin = vertices[polygon.front()];
        for (std::size_t index = 1U; index + 1U < polygon.size(); ++index) {
            if (polygon[index] >= vertices.size() ||
                polygon[index + 1U] >= vertices.size()) {
                continue;
            }
            area += ((vertices[polygon[index]] - origin) ^
                (vertices[polygon[index + 1U]] - origin)).length() * 0.5;
        }
    }
    return area;
}

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
        ? (values[middle - 1U] + values[middle]) * 0.5
        : values[middle];
}

ProjectionHit closestOnTriangles(
    const MPoint& point,
    const std::vector<SourceTriangle>& triangles,
    const std::vector<std::size_t>* candidateIndices)
{
    ProjectionHit result;
    const auto inspect = [&](std::size_t triangleIndex) {
        if (triangleIndex >= triangles.size()) {
            return;
        }
        const SourceTriangle& triangle = triangles[triangleIndex];
        const MPoint candidate = closestPointOnTriangle(
            point,
            triangle.a,
            triangle.b,
            triangle.c);
        const double distance = (candidate - point).length();
        if (!std::isfinite(distance) || distance >= result.distance) {
            return;
        }
        result.point = candidate;
        result.normal = triangle.normal;
        result.triangleIndex = triangleIndex;
        result.sourceFaceId = triangle.sourceFaceId;
        result.distance = distance;
        result.valid = true;
    };

    if (candidateIndices != nullptr && !candidateIndices->empty()) {
        for (const std::size_t triangleIndex : *candidateIndices) {
            inspect(triangleIndex);
        }
    } else {
        for (std::size_t triangleIndex = 0U;
             triangleIndex < triangles.size();
             ++triangleIndex) {
            inspect(triangleIndex);
        }
    }
    return result;
}

std::vector<std::size_t> localTriangleNeighborhood(
    std::size_t seed,
    unsigned int rings,
    const std::vector<SourceTriangle>& triangles)
{
    if (seed >= triangles.size()) {
        return {};
    }
    std::vector<std::size_t> result;
    std::vector<int> depths(triangles.size(), -1);
    std::deque<std::size_t> queue;
    depths[seed] = 0;
    queue.push_back(seed);
    while (!queue.empty()) {
        const std::size_t current = queue.front();
        queue.pop_front();
        result.push_back(current);
        if (static_cast<unsigned int>(depths[current]) >= rings) {
            continue;
        }
        for (const std::size_t adjacent : triangles[current].adjacentTriangleIndices) {
            if (adjacent < depths.size() && depths[adjacent] < 0) {
                depths[adjacent] = depths[current] + 1;
                queue.push_back(adjacent);
            }
        }
    }
    return result;
}

ResultVertexSourceMapping mapping(
    const MPoint& rawPosition,
    const MPoint& projectedPosition,
    const ProjectionHit& hit,
    double surfaceDistance)
{
    ResultVertexSourceMapping result;
    result.patchTriangleIndex = hit.triangleIndex;
    result.sourceFaceId = hit.sourceFaceId;
    result.rawPosition = rawPosition;
    result.projectedPosition = projectedPosition;
    result.sourceNormal = hit.normal;
    result.projectionDistance = (projectedPosition - rawPosition).length();
    result.surfaceDistance = surfaceDistance;
    return result;
}

}  // namespace

SurfaceConformer::SurfaceConformer()
{
    setSettings(settings_);
}

const SurfaceConformerSettings& SurfaceConformer::settings() const noexcept
{
    return settings_;
}

void SurfaceConformer::setSettings(
    const SurfaceConformerSettings& settings) noexcept
{
    settings_ = settings;
    settings_.tangentialRelaxStrength =
        std::clamp(settings_.tangentialRelaxStrength, 0.0, 1.0);
    settings_.meanDistanceWarningTargetLengthRatio =
        std::max(settings_.meanDistanceWarningTargetLengthRatio, 0.0);
    settings_.maximumDistanceWarningTargetLengthRatio =
        std::max(settings_.maximumDistanceWarningTargetLengthRatio, 0.0);
    settings_.maximumRelaxAreaLossRatio =
        std::clamp(settings_.maximumRelaxAreaLossRatio, 0.0, 1.0);
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 0.0);
    BoundaryConformerSettings boundarySettings = boundaryConformer_.settings();
    boundarySettings.geometryEpsilon = settings_.geometryEpsilon;
    boundaryConformer_.setSettings(boundarySettings);
}

bool SurfaceConformer::conform(
    const TriangulatedPatch& patch,
    QuadPatchResult& result,
    std::string& diagnostic) const
{
    result.conformedVertices.clear();
    result.rawSourceMappings.clear();
    result.sourceMappings.clear();
    result.fidelity = SurfaceFidelityMetrics();
    if (patch.empty() || result.rawVertices.empty() || result.polygons.empty()) {
        diagnostic = "Surface Conformer received an empty patch or Raw result.";
        return false;
    }

    std::vector<SourceTriangle> sourceTriangles;
    sourceTriangles.reserve(patch.triangles.size());
    std::vector<std::vector<std::size_t>> vertexTriangles(patch.vertices.size());
    std::vector<MPoint> sourcePoints;
    sourcePoints.reserve(patch.vertices.size());
    for (const PatchVertex& vertex : patch.vertices) {
        sourcePoints.push_back(vertex.position);
    }
    std::set<Edge> sourceEdges;
    for (std::size_t triangleIndex = 0U;
         triangleIndex < patch.triangles.size();
         ++triangleIndex) {
        const PatchTriangle& triangle = patch.triangles[triangleIndex];
        if (triangle.vertexIndices[0] >= patch.vertices.size() ||
            triangle.vertexIndices[1] >= patch.vertices.size() ||
            triangle.vertexIndices[2] >= patch.vertices.size()) {
            diagnostic = "Surface Conformer found an invalid source triangle.";
            return false;
        }
        SourceTriangle source;
        source.a = patch.vertices[triangle.vertexIndices[0]].position;
        source.b = patch.vertices[triangle.vertexIndices[1]].position;
        source.c = patch.vertices[triangle.vertexIndices[2]].position;
        source.normal = (source.b - source.a) ^ (source.c - source.a);
        if (source.normal.length() <= settings_.geometryEpsilon) {
            diagnostic = "Surface Conformer found a zero-area source triangle.";
            return false;
        }
        source.normal.normalize();
        source.sourceFaceId = triangle.sourceFaceId;
        sourceTriangles.push_back(source);
        for (std::size_t corner = 0U; corner < 3U; ++corner) {
            const std::size_t first = triangle.vertexIndices[corner];
            const std::size_t second = triangle.vertexIndices[(corner + 1U) % 3U];
            sourceEdges.insert(orderedEdge(first, second));
            vertexTriangles[first].push_back(triangleIndex);
        }
    }
    for (const std::vector<std::size_t>& incident : vertexTriangles) {
        for (const std::size_t first : incident) {
            for (const std::size_t second : incident) {
                if (first != second) {
                    sourceTriangles[first].adjacentTriangleIndices.push_back(second);
                }
            }
        }
    }
    for (SourceTriangle& triangle : sourceTriangles) {
        std::sort(
            triangle.adjacentTriangleIndices.begin(),
            triangle.adjacentTriangleIndices.end());
        triangle.adjacentTriangleIndices.erase(
            std::unique(
                triangle.adjacentTriangleIndices.begin(),
                triangle.adjacentTriangleIndices.end()),
            triangle.adjacentTriangleIndices.end());
    }

    for (const std::vector<std::size_t>& polygon : result.polygons) {
        for (const std::size_t index : polygon) {
            if (index >= result.rawVertices.size()) {
                diagnostic = "Surface Conformer found an invalid Raw polygon index.";
                return false;
            }
        }
    }

    result.conformedVertices.resize(result.rawVertices.size());
    result.rawSourceMappings.resize(result.rawVertices.size());
    result.sourceMappings.resize(result.rawVertices.size());
    std::vector<ProjectionHit> currentHits(result.rawVertices.size());
    std::vector<bool> fixedBoundary(result.rawVertices.size(), false);
    for (const std::size_t fixedIndex : result.fixedBoundaryVertexIndices) {
        if (fixedIndex >= fixedBoundary.size()) {
            diagnostic = "Fixed Source Boundary contains an invalid result index.";
            return false;
        }
        fixedBoundary[fixedIndex] = true;
    }
    double rawDistanceSum = 0.0;
    for (std::size_t index = 0U; index < result.rawVertices.size(); ++index) {
        if (!finitePoint(result.rawVertices[index])) {
            diagnostic = "Surface Conformer received a non-finite Raw vertex.";
            return false;
        }
        ProjectionHit hit = closestOnTriangles(
            result.rawVertices[index],
            sourceTriangles,
            nullptr);
        if (!hit.valid || !finitePoint(hit.point)) {
            diagnostic = "Raw vertex could not be projected to its source Patch Component.";
            return false;
        }
        if (fixedBoundary[index]) {
            hit.point = result.rawVertices[index];
            hit.distance = 0.0;
        }
        currentHits[index] = hit;
        result.conformedVertices[index] = hit.point;
        result.rawSourceMappings[index] = mapping(
            result.rawVertices[index],
            hit.point,
            hit,
            hit.distance);
        rawDistanceSum += hit.distance;
        result.fidelity.maximumRawSurfaceDistance = std::max(
            result.fidelity.maximumRawSurfaceDistance,
            hit.distance);
    }
    result.fidelity.meanRawSurfaceDistance = result.rawVertices.empty()
        ? 0.0
        : rawDistanceSum / static_cast<double>(result.rawVertices.size());

    std::vector<std::unordered_set<std::size_t>> resultNeighbors(
        result.rawVertices.size());
    std::map<Edge, std::size_t> resultEdgeUseCount;
    for (const std::vector<std::size_t>& polygon : result.polygons) {
        for (std::size_t index = 0U; index < polygon.size(); ++index) {
            const std::size_t first = polygon[index];
            const std::size_t second = polygon[(index + 1U) % polygon.size()];
            if (first == second) {
                continue;
            }
            resultNeighbors[first].insert(second);
            resultNeighbors[second].insert(first);
            ++resultEdgeUseCount[orderedEdge(first, second)];
        }
    }
    std::vector<bool> resultBoundary(result.rawVertices.size(), false);
    for (const auto& [edge, useCount] : resultEdgeUseCount) {
        if (useCount == 1U) {
            resultBoundary[edge.first] = true;
            resultBoundary[edge.second] = true;
        }
    }

    std::string boundaryConformationDiagnostic;
    if (!result.boundaryLocked && settings_.projectResultBoundaryToSourceBoundary &&
        !patch.boundaryLoops.empty()) {
        if (!boundaryConformer_.conform(
                patch,
                result,
                boundaryConformationDiagnostic)) {
            if (!result.boundaryLoops.empty()) {
                diagnostic =
                    "Ordered Boundary conformation failed: " +
                    boundaryConformationDiagnostic;
                return false;
            }
            // Some upstream extraction results are closed and expose no
            // topological Result boundary. There is no Result sequence on
            // which ordered invariants can operate; retain the explicit
            // no-boundary diagnostic for Phase 5 instead of fabricating one.
            result.boundaryCorrespondences.clear();
        } else {
            for (std::size_t index = 0U; index < resultBoundary.size(); ++index) {
                if (resultBoundary[index]) {
                    ProjectionHit triangleHit = closestOnTriangles(
                        result.conformedVertices[index],
                        sourceTriangles,
                        nullptr);
                    if (!triangleHit.valid) {
                        diagnostic =
                            "Boundary vertex could not be remapped to a source triangle.";
                        return false;
                    }
                    triangleHit.point = result.conformedVertices[index];
                    currentHits[index] = triangleHit;
                }
            }
        }
    }

    const std::vector<MPoint> initiallyProjectedVertices =
        result.conformedVertices;
    const std::vector<ProjectionHit> initiallyProjectedHits = currentHits;
    const double initiallyProjectedArea = polygonArea(
        initiallyProjectedVertices,
        result.polygons);

    for (unsigned int iteration = 0U;
         iteration < settings_.tangentialRelaxIterations;
         ++iteration) {
        const std::vector<MPoint> source = result.conformedVertices;
        std::vector<MPoint> destination = source;
        std::vector<ProjectionHit> nextHits = currentHits;
        for (std::size_t index = 0U; index < source.size(); ++index) {
            if (fixedBoundary[index] || resultNeighbors[index].empty() ||
                (settings_.lockResultBoundaryDuringRelax &&
                 resultBoundary[index])) {
                continue;
            }
            MVector neighborSum(0.0, 0.0, 0.0);
            for (const std::size_t neighbor : resultNeighbors[index]) {
                neighborSum += MVector(
                    source[neighbor].x,
                    source[neighbor].y,
                    source[neighbor].z);
            }
            const double neighborCount =
                static_cast<double>(resultNeighbors[index].size());
            const MPoint neighborCenter(
                neighborSum.x / neighborCount,
                neighborSum.y / neighborCount,
                neighborSum.z / neighborCount);
            MVector displacement = neighborCenter - source[index];
            MVector normal = currentHits[index].normal;
            if (normal.length() <= settings_.geometryEpsilon) {
                continue;
            }
            normal.normalize();
            displacement -= normal * (displacement * normal);
            const MPoint candidate = source[index] +
                displacement * settings_.tangentialRelaxStrength;
            const std::vector<std::size_t> localTriangles = localTriangleNeighborhood(
                currentHits[index].triangleIndex,
                settings_.localProjectionTriangleRings,
                sourceTriangles);
            ProjectionHit hit = closestOnTriangles(
                candidate,
                sourceTriangles,
                &localTriangles);
            if (!hit.valid) {
                hit = closestOnTriangles(candidate, sourceTriangles, nullptr);
            }
            if (hit.valid && finitePoint(hit.point)) {
                destination[index] = hit.point;
                nextHits[index] = hit;
            }
        }
        result.conformedVertices = std::move(destination);
        currentHits = std::move(nextHits);
    }

    const double relaxedArea = polygonArea(
        result.conformedVertices,
        result.polygons);
    if (initiallyProjectedArea > settings_.geometryEpsilon &&
        relaxedArea < initiallyProjectedArea *
            (1.0 - settings_.maximumRelaxAreaLossRatio)) {
        result.conformedVertices = initiallyProjectedVertices;
        currentHits = initiallyProjectedHits;
    }
    result.boundaryLockedDiagnostic.maximumSourceBoundaryDisplacement = 0.0;
    for (const std::size_t fixedIndex : result.fixedBoundaryVertexIndices) {
        result.conformedVertices[fixedIndex] = result.rawVertices[fixedIndex];
    }

    double projectionDistanceSum = 0.0;
    double conformedDistanceSum = 0.0;
    for (std::size_t index = 0U;
         index < result.conformedVertices.size();
         ++index) {
        const std::vector<std::size_t> localTriangles = localTriangleNeighborhood(
            currentHits[index].triangleIndex,
            settings_.localProjectionTriangleRings,
            sourceTriangles);
        ProjectionHit verification = closestOnTriangles(
            result.conformedVertices[index],
            sourceTriangles,
            &localTriangles);
        if (!verification.valid) {
            verification = closestOnTriangles(
                result.conformedVertices[index],
                sourceTriangles,
                nullptr);
        }
        if (!verification.valid || !std::isfinite(verification.distance)) {
            diagnostic = "Conformed vertex could not be verified on the source Patch.";
            return false;
        }
        const double projectionDistance =
            (result.conformedVertices[index] - result.rawVertices[index]).length();
        projectionDistanceSum += projectionDistance;
        conformedDistanceSum += verification.distance;
        result.fidelity.maximumProjectionDistance = std::max(
            result.fidelity.maximumProjectionDistance,
            projectionDistance);
        result.fidelity.maximumConformedSurfaceDistance = std::max(
            result.fidelity.maximumConformedSurfaceDistance,
            verification.distance);
        result.sourceMappings[index] = mapping(
            result.rawVertices[index],
            result.conformedVertices[index],
            verification,
            verification.distance);
    }
    const double resultVertexCount =
        static_cast<double>(result.conformedVertices.size());
    result.fidelity.meanProjectionDistance = resultVertexCount > 0.0
        ? projectionDistanceSum / resultVertexCount
        : 0.0;
    result.fidelity.meanConformedSurfaceDistance = resultVertexCount > 0.0
        ? conformedDistanceSum / resultVertexCount
        : 0.0;

    double sourceArea = 0.0;
    for (const SourceTriangle& triangle : sourceTriangles) {
        sourceArea += ((triangle.b - triangle.a) ^
            (triangle.c - triangle.a)).length() * 0.5;
    }
    std::vector<double> sourceEdgeLengths;
    sourceEdgeLengths.reserve(sourceEdges.size());
    double sourceEdgeLengthSum = 0.0;
    for (const Edge& edge : sourceEdges) {
        const double length =
            (patch.vertices[edge.second].position -
             patch.vertices[edge.first].position).length();
        if (std::isfinite(length) && length > settings_.geometryEpsilon) {
            sourceEdgeLengths.push_back(length);
            sourceEdgeLengthSum += length;
        }
    }

    SurfaceFidelityMetrics& fidelity = result.fidelity;
    fidelity.sourceArea = sourceArea;
    fidelity.rawQuadArea = polygonArea(result.rawVertices, result.polygons);
    fidelity.conformedArea = polygonArea(result.conformedVertices, result.polygons);
    fidelity.rawAreaRatio = sourceArea > settings_.geometryEpsilon
        ? fidelity.rawQuadArea / sourceArea
        : 0.0;
    fidelity.conformedAreaRatio = sourceArea > settings_.geometryEpsilon
        ? fidelity.conformedArea / sourceArea
        : 0.0;
    fidelity.sourceAverageEdgeLength = sourceEdgeLengths.empty()
        ? 0.0
        : sourceEdgeLengthSum / static_cast<double>(sourceEdgeLengths.size());
    fidelity.sourceMedianEdgeLength = median(sourceEdgeLengths);
    fidelity.sourceBounds = bounds(sourcePoints);
    fidelity.rawBounds = bounds(result.rawVertices);
    fidelity.conformedBounds = bounds(result.conformedVertices);
    const double meanWarning = result.targetEdgeLength *
        settings_.meanDistanceWarningTargetLengthRatio;
    const double maximumWarning = result.targetEdgeLength *
        settings_.maximumDistanceWarningTargetLengthRatio;
    fidelity.distanceQualityWarning =
        fidelity.meanConformedSurfaceDistance > meanWarning ||
        fidelity.maximumConformedSurfaceDistance > maximumWarning;
    result.maximumSurfaceDistance = fidelity.maximumConformedSurfaceDistance;

    std::ostringstream message;
    message << "Surface conformation completed on Patch Component "
            << patch.componentId << "; Raw max distance "
            << fidelity.maximumRawSurfaceDistance
            << ", Conformed max distance "
            << fidelity.maximumConformedSurfaceDistance << '.';
    if (!boundaryConformationDiagnostic.empty()) {
        message << " Boundary: " << boundaryConformationDiagnostic;
    }
    diagnostic = message.str();
    return true;
}

}  // namespace directional_retopo
