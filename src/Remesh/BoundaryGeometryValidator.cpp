#include "Remesh/BoundaryGeometryValidator.h"

#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace directional_retopo {
namespace {

using Edge = std::pair<std::size_t, std::size_t>;

struct IndexedSegment final
{
    std::size_t first = 0U;
    std::size_t second = 0U;
};

struct SegmentClosestResult final
{
    double firstParameter = 0.0;
    double secondParameter = 0.0;
    double distanceSquared = std::numeric_limits<double>::infinity();
};

bool finitePoint(const MPoint& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
}

Edge orderedEdge(std::size_t first, std::size_t second)
{
    return first < second ? Edge(first, second) : Edge(second, first);
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
        const auto lower = std::max_element(
            values.begin(),
            values.begin() + middle);
        result = (*lower + result) * 0.5;
    }
    return result;
}

double edgeScale(
    const std::vector<MPoint>& vertices,
    const std::vector<IndexedSegment>& segments)
{
    std::vector<double> lengths;
    lengths.reserve(segments.size());
    for (const IndexedSegment& segment : segments) {
        if (segment.first >= vertices.size() ||
            segment.second >= vertices.size()) {
            continue;
        }
        const double length =
            (vertices[segment.second] - vertices[segment.first]).length();
        if (std::isfinite(length) && length > 0.0) {
            lengths.push_back(length);
        }
    }
    return median(std::move(lengths));
}

double validationScale(double geometryScale, double targetEdgeLength)
{
    if (geometryScale > 0.0 && targetEdgeLength > 0.0 &&
        std::isfinite(targetEdgeLength)) {
        return std::min(geometryScale, targetEdgeLength);
    }
    if (geometryScale > 0.0) {
        return geometryScale;
    }
    return std::isfinite(targetEdgeLength)
        ? std::max(targetEdgeLength, 0.0)
        : 0.0;
}

double intersectionTolerance(
    double geometryScale,
    double targetEdgeLength,
    const BoundaryGeometryValidationSettings& settings)
{
    return settings.absoluteTolerance +
        settings.relativeIntersectionTolerance *
            validationScale(geometryScale, targetEdgeLength);
}

SegmentClosestResult closestSegmentPoints(
    const MPoint& firstStart,
    const MPoint& firstEnd,
    const MPoint& secondStart,
    const MPoint& secondEnd,
    double epsilon)
{
    const MVector firstDirection = firstEnd - firstStart;
    const MVector secondDirection = secondEnd - secondStart;
    const MVector offset = firstStart - secondStart;
    const double firstLengthSquared = firstDirection * firstDirection;
    const double secondLengthSquared = secondDirection * secondDirection;
    const double secondOffset = secondDirection * offset;

    SegmentClosestResult result;
    if (firstLengthSquared <= epsilon && secondLengthSquared <= epsilon) {
        result.distanceSquared = offset * offset;
        return result;
    }
    if (firstLengthSquared <= epsilon) {
        result.secondParameter = std::clamp(
            secondOffset / secondLengthSquared,
            0.0,
            1.0);
    } else {
        const double firstOffset = firstDirection * offset;
        if (secondLengthSquared <= epsilon) {
            result.firstParameter = std::clamp(
                -firstOffset / firstLengthSquared,
                0.0,
                1.0);
        } else {
            const double directionsDot = firstDirection * secondDirection;
            const double denominator =
                firstLengthSquared * secondLengthSquared -
                directionsDot * directionsDot;
            if (std::abs(denominator) > epsilon) {
                result.firstParameter = std::clamp(
                    (directionsDot * secondOffset -
                     firstOffset * secondLengthSquared) / denominator,
                    0.0,
                    1.0);
            }
            result.secondParameter =
                (directionsDot * result.firstParameter + secondOffset) /
                secondLengthSquared;
            if (result.secondParameter < 0.0) {
                result.secondParameter = 0.0;
                result.firstParameter = std::clamp(
                    -firstOffset / firstLengthSquared,
                    0.0,
                    1.0);
            } else if (result.secondParameter > 1.0) {
                result.secondParameter = 1.0;
                result.firstParameter = std::clamp(
                    (directionsDot - firstOffset) / firstLengthSquared,
                    0.0,
                    1.0);
            }
        }
    }

    const MPoint firstPoint =
        firstStart + firstDirection * result.firstParameter;
    const MPoint secondPoint =
        secondStart + secondDirection * result.secondParameter;
    const MVector separation = firstPoint - secondPoint;
    result.distanceSquared = separation * separation;
    return result;
}

bool trueSegmentIntersection(
    const MPoint& firstStart,
    const MPoint& firstEnd,
    const MPoint& secondStart,
    const MPoint& secondEnd,
    double tolerance,
    double parameterTolerance)
{
    const double squaredTolerance = tolerance * tolerance;
    const SegmentClosestResult closest = closestSegmentPoints(
        firstStart,
        firstEnd,
        secondStart,
        secondEnd,
        squaredTolerance);
    if (closest.distanceSquared <= squaredTolerance &&
        closest.firstParameter > parameterTolerance &&
        closest.firstParameter < 1.0 - parameterTolerance &&
        closest.secondParameter > parameterTolerance &&
        closest.secondParameter < 1.0 - parameterTolerance) {
        return true;
    }

    const MVector firstDirection = firstEnd - firstStart;
    const MVector secondDirection = secondEnd - secondStart;
    const double firstLengthSquared = firstDirection * firstDirection;
    const double secondLengthSquared = secondDirection * secondDirection;
    if (firstLengthSquared <= squaredTolerance ||
        secondLengthSquared <= squaredTolerance) {
        return false;
    }
    const MVector cross = firstDirection ^ secondDirection;
    if ((cross * cross) >
        squaredTolerance * firstLengthSquared * secondLengthSquared) {
        return false;
    }

    const double secondStartOnFirst =
        ((secondStart - firstStart) * firstDirection) / firstLengthSquared;
    const double secondEndOnFirst =
        ((secondEnd - firstStart) * firstDirection) / firstLengthSquared;
    const double overlapStart = std::max(
        parameterTolerance,
        std::min(secondStartOnFirst, secondEndOnFirst));
    const double overlapEnd = std::min(
        1.0 - parameterTolerance,
        std::max(secondStartOnFirst, secondEndOnFirst));
    if (overlapEnd <= overlapStart) {
        return false;
    }
    const double firstParameter = (overlapStart + overlapEnd) * 0.5;
    const MPoint overlapPoint = firstStart + firstDirection * firstParameter;
    const double secondParameter =
        ((overlapPoint - secondStart) * secondDirection) /
        secondLengthSquared;
    const MPoint secondPoint =
        secondStart + secondDirection * secondParameter;
    return secondParameter > parameterTolerance &&
        secondParameter < 1.0 - parameterTolerance &&
        (overlapPoint - secondPoint).length() <= tolerance;
}

MVector polygonNormal(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& polygon)
{
    MVector normal = MVector::zero;
    if (polygon.size() < 3U) {
        return normal;
    }
    const MPoint& origin = vertices[polygon.front()];
    for (std::size_t index = 1U; index + 1U < polygon.size(); ++index) {
        normal += (vertices[polygon[index]] - origin) ^
            (vertices[polygon[index + 1U]] - origin);
    }
    return normal;
}

MPoint polygonCenter(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& polygon)
{
    MPoint center(0.0, 0.0, 0.0);
    for (const std::size_t index : polygon) {
        center.x += vertices[index].x;
        center.y += vertices[index].y;
        center.z += vertices[index].z;
    }
    const double inverse = 1.0 / static_cast<double>(polygon.size());
    center.x *= inverse;
    center.y *= inverse;
    center.z *= inverse;
    return center;
}

MVector nearestSourceNormal(
    const MPoint& position,
    const TriangulatedPatch& patch,
    double epsilon)
{
    MVector bestNormal = MVector::zero;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const PatchTriangle& triangle : patch.triangles) {
        if (triangle.vertexIndices[0] >= patch.vertices.size() ||
            triangle.vertexIndices[1] >= patch.vertices.size() ||
            triangle.vertexIndices[2] >= patch.vertices.size()) {
            continue;
        }
        const MPoint& first =
            patch.vertices[triangle.vertexIndices[0]].position;
        const MPoint& second =
            patch.vertices[triangle.vertexIndices[1]].position;
        const MPoint& third =
            patch.vertices[triangle.vertexIndices[2]].position;
        MVector normal = (second - first) ^ (third - first);
        if (normal.length() <= epsilon) {
            continue;
        }
        const MPoint center(
            (first.x + second.x + third.x) / 3.0,
            (first.y + second.y + third.y) / 3.0,
            (first.z + second.z + third.z) / 3.0);
        const MVector delta = center - position;
        const double distance = delta * delta;
        if (distance < bestDistance) {
            bestDistance = distance;
            normal.normalize();
            bestNormal = normal;
        }
    }
    return bestNormal;
}

}  // namespace

BoundaryLoopValidationDiagnostic
BoundaryGeometryValidator::validateClosedLoop(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& orderedLoop,
    double localTargetEdgeLength,
    const BoundaryGeometryValidationSettings& settings)
{
    BoundaryLoopValidationDiagnostic result;
    result.vertexCount = orderedLoop.size();
    if (orderedLoop.size() < 3U) {
        result.message = "Closed loop has fewer than three vertices.";
        return result;
    }

    std::set<std::size_t> uniqueVertices;
    std::vector<IndexedSegment> segments;
    segments.reserve(orderedLoop.size());
    for (std::size_t index = 0U; index < orderedLoop.size(); ++index) {
        const std::size_t vertex = orderedLoop[index];
        const std::size_t next = orderedLoop[(index + 1U) % orderedLoop.size()];
        if (vertex >= vertices.size() || next >= vertices.size()) {
            result.message = "Boundary loop contains an invalid vertex index.";
            return result;
        }
        if (!finitePoint(vertices[vertex]) || !finitePoint(vertices[next])) {
            result.message = "Boundary loop contains a non-finite vertex.";
            return result;
        }
        if (!uniqueVertices.insert(vertex).second) {
            ++result.duplicateVertexCount;
        }
        segments.push_back({vertex, next});
    }
    result.topologySimple = result.duplicateVertexCount == 0U;
    if (!result.topologySimple) {
        result.message =
            "Boundary loop repeats a vertex and is not a simple ordered cycle.";
        return result;
    }

    result.localEdgeScale = edgeScale(vertices, segments);
    result.intersectionTolerance = intersectionTolerance(
        result.localEdgeScale,
        localTargetEdgeLength,
        settings);
    for (const IndexedSegment& segment : segments) {
        if ((vertices[segment.second] - vertices[segment.first]).length() <=
            result.intersectionTolerance) {
            ++result.zeroLengthEdgeCount;
        }
    }
    if (result.zeroLengthEdgeCount > 0U) {
        result.message = "Boundary loop contains a zero-length edge.";
        return result;
    }

    for (std::size_t first = 0U; first < segments.size(); ++first) {
        for (std::size_t second = first + 1U;
             second < segments.size();
             ++second) {
            const IndexedSegment& a = segments[first];
            const IndexedSegment& b = segments[second];
            if (a.first == b.first || a.first == b.second ||
                a.second == b.first || a.second == b.second) {
                continue;
            }
            if (trueSegmentIntersection(
                    vertices[a.first],
                    vertices[a.second],
                    vertices[b.first],
                    vertices[b.second],
                    result.intersectionTolerance,
                    settings.interiorParameterTolerance)) {
                ++result.trueIntersectionCount;
            }
        }
    }
    if (result.trueIntersectionCount > 0U) {
        result.message =
            "Boundary loop has a true 3D geometric self-intersection. "
            "Triangle fallback is not applicable to invalid boundary geometry.";
        return result;
    }

    result.valid = true;
    result.message =
        "Ordered topology and true 3D intersection validation passed.";
    return result;
}

CollarPolygonValidationDiagnostic
BoundaryGeometryValidator::validateCollarPolygons(
    const std::vector<MPoint>& vertices,
    std::vector<std::vector<std::size_t>>& polygons,
    const TriangulatedPatch& sourcePatch,
    double localTargetEdgeLength,
    const BoundaryGeometryValidationSettings& settings)
{
    CollarPolygonValidationDiagnostic result;
    std::vector<IndexedSegment> segments;
    std::map<Edge, std::size_t> edgeUseCount;
    for (std::vector<std::size_t>& polygon : polygons) {
        if (polygon.size() != 3U && polygon.size() != 4U) {
            result.message =
                "Transition Collar contains a polygon that is not a triangle or quad.";
            return result;
        }
        std::set<std::size_t> uniqueVertices;
        bool indicesValid = true;
        for (const std::size_t vertex : polygon) {
            if (vertex >= vertices.size() || !finitePoint(vertices[vertex])) {
                ++result.invalidIndexCount;
                indicesValid = false;
            } else if (!uniqueVertices.insert(vertex).second) {
                ++result.repeatedVertexCount;
            }
        }
        if (!indicesValid || result.repeatedVertexCount > 0U) {
            result.message =
                "Transition Collar contains invalid or repeated polygon vertices.";
            return result;
        }

        MVector normal = polygonNormal(vertices, polygon);
        if (normal.length() <= settings.absoluteTolerance) {
            ++result.zeroAreaPolygonCount;
            result.message =
                "Transition Collar contains a zero-area 3D polygon.";
            return result;
        }
        normal.normalize();
        const MVector sourceNormal = nearestSourceNormal(
            polygonCenter(vertices, polygon),
            sourcePatch,
            settings.absoluteTolerance);
        if (sourceNormal.length() > settings.absoluteTolerance &&
            normal * sourceNormal < 0.0) {
            std::reverse(polygon.begin(), polygon.end());
            ++result.reversedPolygonCount;
        }

        for (std::size_t edge = 0U; edge < polygon.size(); ++edge) {
            const std::size_t first = polygon[edge];
            const std::size_t second = polygon[(edge + 1U) % polygon.size()];
            segments.push_back({first, second});
            if (++edgeUseCount[orderedEdge(first, second)] > 2U) {
                ++result.nonManifoldEdgeCount;
            }
        }
    }
    if (result.nonManifoldEdgeCount > 0U) {
        result.message =
            "Transition Collar contains an unintended non-manifold edge.";
        return result;
    }

    result.localEdgeScale = edgeScale(vertices, segments);
    result.intersectionTolerance = intersectionTolerance(
        result.localEdgeScale,
        localTargetEdgeLength,
        settings);
    for (const IndexedSegment& segment : segments) {
        if ((vertices[segment.second] - vertices[segment.first]).length() <=
            result.intersectionTolerance) {
            ++result.zeroLengthEdgeCount;
        }
    }
    if (result.zeroLengthEdgeCount > 0U) {
        result.message = "Transition Collar contains a zero-length edge.";
        return result;
    }

    for (std::size_t first = 0U; first < segments.size(); ++first) {
        for (std::size_t second = first + 1U;
             second < segments.size();
             ++second) {
            const IndexedSegment& a = segments[first];
            const IndexedSegment& b = segments[second];
            if (a.first == b.first || a.first == b.second ||
                a.second == b.first || a.second == b.second) {
                continue;
            }
            if (trueSegmentIntersection(
                    vertices[a.first],
                    vertices[a.second],
                    vertices[b.first],
                    vertices[b.second],
                    result.intersectionTolerance,
                    settings.interiorParameterTolerance)) {
                ++result.trueIntersectionCount;
            }
        }
    }
    if (result.trueIntersectionCount > 0U) {
        result.message =
            "Transition Collar has a true 3D geometric edge intersection. "
            "Triangle fallback is not applicable to invalid collar geometry.";
        return result;
    }

    result.valid = true;
    std::ostringstream message;
    message << "3D polygon validation passed; "
            << result.reversedPolygonCount
            << " polygon winding(s) aligned to the local source surface.";
    result.message = message.str();
    return result;
}

}  // namespace directional_retopo
