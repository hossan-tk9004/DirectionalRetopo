#include "Remesh/OrderedBoundaryCorrespondence.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace directional_retopo {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

using Edge = std::pair<std::size_t, std::size_t>;

struct ArcCurve final
{
    std::vector<MPoint> points;
    std::vector<int> sourceVertexIds;
    std::vector<int> sourceEdgeIds;
    std::vector<std::size_t> originalVertexIndices;
    std::vector<double> cumulativeLengths;
    std::vector<double> vertexParameters;
    double totalLength = 0.0;
    bool closed = false;

    [[nodiscard]] std::size_t edgeCount() const noexcept
    {
        return points.size() < 2U
            ? 0U
            : (closed ? points.size() : points.size() - 1U);
    }
};

struct CurveSample final
{
    MPoint position;
    MVector tangent;
    std::size_t edgeIndex = 0U;
    double edgeParameter = 0.0;
    double normalizedParameter = 0.0;
};

struct Alignment final
{
    double cost = std::numeric_limits<double>::infinity();
    std::size_t resultOffset = 0U;
    std::size_t sourceOffset = 0U;
    bool reverseResult = false;
};

struct SegmentDistanceResult final
{
    double distance = std::numeric_limits<double>::infinity();
    double firstParameter = 0.0;
    double secondParameter = 0.0;
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

double normalizedClosedParameter(double parameter)
{
    parameter = std::fmod(parameter, 1.0);
    return parameter < 0.0 ? parameter + 1.0 : parameter;
}

MPoint closestPointOnSegment(
    const MPoint& point,
    const MPoint& first,
    const MPoint& second,
    double& segmentParameter)
{
    const MVector segment = second - first;
    const double lengthSquared = segment * segment;
    if (lengthSquared <= 1.0e-20) {
        segmentParameter = 0.0;
        return first;
    }
    segmentParameter = std::clamp(
        (segment * (point - first)) / lengthSquared,
        0.0,
        1.0);
    return first + segment * segmentParameter;
}

double pointSegmentDistance(
    const MPoint& point,
    const MPoint& first,
    const MPoint& second)
{
    double parameter = 0.0;
    return (point - closestPointOnSegment(
        point,
        first,
        second,
        parameter)).length();
}

SegmentDistanceResult segmentDistance(
    const MPoint& firstStart,
    const MPoint& firstEnd,
    const MPoint& secondStart,
    const MPoint& secondEnd)
{
    const MVector u = firstEnd - firstStart;
    const MVector v = secondEnd - secondStart;
    const MVector w = firstStart - secondStart;
    const double a = u * u;
    const double b = u * v;
    const double c = v * v;
    const double d = u * w;
    const double e = v * w;
    const double denominator = a * c - b * b;
    double numeratorFirst = 0.0;
    double denominatorFirst = denominator;
    double numeratorSecond = 0.0;
    double denominatorSecond = denominator;

    if (denominator <= 1.0e-20) {
        numeratorFirst = 0.0;
        denominatorFirst = 1.0;
        numeratorSecond = e;
        denominatorSecond = c;
    } else {
        numeratorFirst = b * e - c * d;
        numeratorSecond = a * e - b * d;
        if (numeratorFirst < 0.0) {
            numeratorFirst = 0.0;
            numeratorSecond = e;
            denominatorSecond = c;
        } else if (numeratorFirst > denominatorFirst) {
            numeratorFirst = denominatorFirst;
            numeratorSecond = e + b;
            denominatorSecond = c;
        }
    }

    if (numeratorSecond < 0.0) {
        numeratorSecond = 0.0;
        if (-d < 0.0) {
            numeratorFirst = 0.0;
        } else if (-d > a) {
            numeratorFirst = denominatorFirst;
        } else {
            numeratorFirst = -d;
            denominatorFirst = a;
        }
    } else if (numeratorSecond > denominatorSecond) {
        numeratorSecond = denominatorSecond;
        if (-d + b < 0.0) {
            numeratorFirst = 0.0;
        } else if (-d + b > a) {
            numeratorFirst = denominatorFirst;
        } else {
            numeratorFirst = -d + b;
            denominatorFirst = a;
        }
    }

    SegmentDistanceResult result;
    result.firstParameter = std::abs(numeratorFirst) <= 1.0e-20
        ? 0.0
        : numeratorFirst / denominatorFirst;
    result.secondParameter = std::abs(numeratorSecond) <= 1.0e-20
        ? 0.0
        : numeratorSecond / denominatorSecond;
    const MVector separation = w +
        u * result.firstParameter - v * result.secondParameter;
    result.distance = separation.length();
    return result;
}

bool buildLengths(ArcCurve& curve, double epsilon)
{
    const std::size_t edgeCount = curve.edgeCount();
    if (edgeCount == 0U) {
        return false;
    }
    curve.cumulativeLengths.assign(edgeCount + 1U, 0.0);
    for (std::size_t edgeIndex = 0U; edgeIndex < edgeCount; ++edgeIndex) {
        const std::size_t next = curve.closed
            ? (edgeIndex + 1U) % curve.points.size()
            : edgeIndex + 1U;
        const double length =
            (curve.points[next] - curve.points[edgeIndex]).length();
        curve.cumulativeLengths[edgeIndex + 1U] =
            curve.cumulativeLengths[edgeIndex] + length;
    }
    curve.totalLength = curve.cumulativeLengths.back();
    if (!std::isfinite(curve.totalLength) || curve.totalLength <= epsilon) {
        return false;
    }
    curve.vertexParameters.resize(curve.points.size(), 0.0);
    for (std::size_t index = 0U; index < curve.points.size(); ++index) {
        curve.vertexParameters[index] =
            curve.cumulativeLengths[index] / curve.totalLength;
    }
    return true;
}

int sourceEdgeIdForPair(
    const TriangulatedPatch& patch,
    std::size_t first,
    std::size_t second)
{
    const Edge requested = orderedEdge(first, second);
    for (const PatchBoundaryEdge& edge : patch.boundaryEdges) {
        if (orderedEdge(edge.vertexIndices[0], edge.vertexIndices[1]) == requested) {
            return edge.sourceEdgeId;
        }
    }
    return -1;
}

bool makeSourceCurve(
    const TriangulatedPatch& patch,
    const PatchBoundaryLoop& loop,
    double epsilon,
    ArcCurve& curve,
    std::string& diagnostic)
{
    curve = ArcCurve();
    curve.closed = loop.closed;
    for (std::size_t index = 0U; index < loop.vertexIndices.size(); ++index) {
        const std::size_t vertexIndex = loop.vertexIndices[index];
        if (vertexIndex >= patch.vertices.size() ||
            !finitePoint(patch.vertices[vertexIndex].position)) {
            diagnostic = "Source Boundary contains an invalid vertex.";
            return false;
        }
        curve.points.push_back(patch.vertices[vertexIndex].position);
        curve.sourceVertexIds.push_back(
            index < loop.sourceVertexIds.size()
                ? loop.sourceVertexIds[index]
                : patch.vertices[vertexIndex].sourceVertexId);
        curve.originalVertexIndices.push_back(index);
    }
    if (curve.closed && curve.points.size() > 1U &&
        (curve.points.front() - curve.points.back()).length() <= epsilon) {
        curve.points.pop_back();
        curve.sourceVertexIds.pop_back();
        curve.originalVertexIndices.pop_back();
    }
    if (!buildLengths(curve, epsilon)) {
        diagnostic = "Source Boundary arc-length parameterization failed.";
        return false;
    }

    curve.sourceEdgeIds.reserve(curve.edgeCount());
    for (std::size_t edgeIndex = 0U; edgeIndex < curve.edgeCount(); ++edgeIndex) {
        int sourceEdgeId = edgeIndex < loop.sourceEdgeIds.size()
            ? loop.sourceEdgeIds[edgeIndex]
            : -1;
        if (sourceEdgeId < 0) {
            const std::size_t next = curve.closed
                ? (edgeIndex + 1U) % curve.points.size()
                : edgeIndex + 1U;
            sourceEdgeId = sourceEdgeIdForPair(
                patch,
                loop.vertexIndices[curve.originalVertexIndices[edgeIndex]],
                loop.vertexIndices[curve.originalVertexIndices[next]]);
        }
        curve.sourceEdgeIds.push_back(sourceEdgeId);
    }
    return true;
}

ArcCurve makeResultCurve(
    const ResultBoundaryLoop& loop,
    const std::vector<MPoint>& vertices,
    std::size_t offset,
    bool reverse,
    double epsilon)
{
    ArcCurve curve;
    curve.closed = loop.closed;
    const std::size_t count = loop.vertexIndices.size();
    curve.points.reserve(count);
    for (std::size_t order = 0U; order < count; ++order) {
        std::size_t index = order;
        if (loop.closed) {
            index = reverse
                ? (offset + count - order) % count
                : (offset + order) % count;
        } else if (reverse) {
            index = count - 1U - order;
        }
        const std::size_t vertexIndex = loop.vertexIndices[index];
        if (vertexIndex >= vertices.size() || !finitePoint(vertices[vertexIndex])) {
            curve.points.clear();
            return curve;
        }
        curve.points.push_back(vertices[vertexIndex]);
    }
    if (!buildLengths(curve, epsilon)) {
        curve.points.clear();
    }
    return curve;
}

void reorderResultLoop(
    ResultBoundaryLoop& loop,
    std::size_t offset,
    bool reverse)
{
    std::vector<std::size_t> ordered;
    ordered.reserve(loop.vertexIndices.size());
    const std::size_t count = loop.vertexIndices.size();
    for (std::size_t order = 0U; order < count; ++order) {
        std::size_t index = order;
        if (loop.closed) {
            index = reverse
                ? (offset + count - order) % count
                : (offset + order) % count;
        } else if (reverse) {
            index = count - 1U - order;
        }
        ordered.push_back(loop.vertexIndices[index]);
    }
    loop.vertexIndices = std::move(ordered);
}

ArcCurve rotateSourceCurve(const ArcCurve& source, std::size_t offset)
{
    if (!source.closed || offset == 0U) {
        return source;
    }
    ArcCurve rotated;
    rotated.closed = true;
    const std::size_t count = source.points.size();
    rotated.points.reserve(count);
    rotated.sourceVertexIds.reserve(count);
    rotated.originalVertexIndices.reserve(count);
    rotated.sourceEdgeIds.reserve(source.edgeCount());
    for (std::size_t index = 0U; index < count; ++index) {
        const std::size_t original = (offset + index) % count;
        rotated.points.push_back(source.points[original]);
        rotated.sourceVertexIds.push_back(source.sourceVertexIds[original]);
        rotated.originalVertexIndices.push_back(
            source.originalVertexIndices[original]);
        rotated.sourceEdgeIds.push_back(source.sourceEdgeIds[original]);
    }
    (void)buildLengths(rotated, 0.0);
    return rotated;
}

MPoint evaluateCurve(
    const ArcCurve& curve,
    double normalizedParameter,
    std::size_t* edgeIndexOut = nullptr,
    double* edgeParameterOut = nullptr)
{
    normalizedParameter = curve.closed
        ? normalizedClosedParameter(normalizedParameter)
        : std::clamp(normalizedParameter, 0.0, 1.0);
    const double distance = normalizedParameter * curve.totalLength;
    const auto upper = std::upper_bound(
        curve.cumulativeLengths.begin(),
        curve.cumulativeLengths.end(),
        distance);
    std::size_t edgeIndex = upper == curve.cumulativeLengths.begin()
        ? 0U
        : static_cast<std::size_t>(
            std::distance(curve.cumulativeLengths.begin(), upper) - 1);
    edgeIndex = std::min(edgeIndex, curve.edgeCount() - 1U);
    const double start = curve.cumulativeLengths[edgeIndex];
    const double end = curve.cumulativeLengths[edgeIndex + 1U];
    const double edgeParameter = end > start
        ? (distance - start) / (end - start)
        : 0.0;
    const std::size_t next = curve.closed
        ? (edgeIndex + 1U) % curve.points.size()
        : edgeIndex + 1U;
    if (edgeIndexOut != nullptr) {
        *edgeIndexOut = edgeIndex;
    }
    if (edgeParameterOut != nullptr) {
        *edgeParameterOut = edgeParameter;
    }
    return curve.points[edgeIndex] +
        (curve.points[next] - curve.points[edgeIndex]) * edgeParameter;
}

MVector curveTangent(const ArcCurve& curve, double parameter)
{
    std::size_t edgeIndex = 0U;
    double edgeParameter = 0.0;
    (void)evaluateCurve(curve, parameter, &edgeIndex, &edgeParameter);
    const std::size_t next = curve.closed
        ? (edgeIndex + 1U) % curve.points.size()
        : edgeIndex + 1U;
    MVector tangent = curve.points[next] - curve.points[edgeIndex];
    if (tangent.length() > 1.0e-20) {
        tangent.normalize();
    }
    return tangent;
}

MVector resultTangent(const ArcCurve& curve, std::size_t index)
{
    if (curve.points.size() < 2U) {
        return MVector::zero;
    }
    MVector tangent;
    if (curve.closed) {
        const std::size_t previous =
            (index + curve.points.size() - 1U) % curve.points.size();
        const std::size_t next = (index + 1U) % curve.points.size();
        tangent = curve.points[next] - curve.points[previous];
    } else if (index == 0U) {
        tangent = curve.points[1U] - curve.points[0U];
    } else if (index + 1U == curve.points.size()) {
        tangent = curve.points[index] - curve.points[index - 1U];
    } else {
        tangent = curve.points[index + 1U] - curve.points[index - 1U];
    }
    if (tangent.length() > 1.0e-20) {
        tangent.normalize();
    }
    return tangent;
}

double alignmentCost(
    const ArcCurve& result,
    const ArcCurve& source,
    double sourceSeam,
    const OrderedBoundaryCorrespondenceSettings& settings)
{
    if (result.points.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    const std::size_t stride = std::max<std::size_t>(
        1U,
        result.points.size() / 128U);
    const double scale = std::max(
        source.totalLength / static_cast<double>(source.edgeCount()),
        settings.geometryEpsilon);
    double cost = 0.0;
    std::size_t samples = 0U;
    for (std::size_t index = 0U; index < result.points.size(); index += stride) {
        const double sourceParameter = source.closed
            ? normalizedClosedParameter(
                sourceSeam + result.vertexParameters[index])
            : result.vertexParameters[index];
        const MVector delta =
            evaluateCurve(source, sourceParameter) - result.points[index];
        const MVector sourceDirection = curveTangent(source, sourceParameter);
        const MVector resultDirection = resultTangent(result, index);
        const double tangentDot = std::clamp(
            sourceDirection * resultDirection,
            -1.0,
            1.0);
        cost += (delta * delta) / (scale * scale) +
            settings.tangentCostWeight * (1.0 - tangentDot) * (1.0 - tangentDot);
        ++samples;
    }
    const double relativeLengthDifference =
        (result.totalLength - source.totalLength) /
        std::max(source.totalLength, settings.geometryEpsilon);
    cost = samples == 0U ? cost : cost / static_cast<double>(samples);
    return cost + settings.arcLengthCostWeight *
        relativeLengthDifference * relativeLengthDifference;
}

Alignment findAlignment(
    const ArcCurve& source,
    const ResultBoundaryLoop& resultLoop,
    const std::vector<MPoint>& resultVertices,
    const OrderedBoundaryCorrespondenceSettings& settings)
{
    Alignment best;
    if (!source.closed || !resultLoop.closed) {
        for (const bool reverse : {false, true}) {
            const ArcCurve result = makeResultCurve(
                resultLoop,
                resultVertices,
                0U,
                reverse,
                settings.geometryEpsilon);
            if (result.points.empty()) {
                continue;
            }
            const double cost = alignmentCost(result, source, 0.0, settings);
            if (cost < best.cost) {
                best.cost = cost;
                best.reverseResult = reverse;
            }
        }
        return best;
    }

    for (const bool reverse : {false, true}) {
        for (std::size_t resultOffset = 0U;
             resultOffset < resultLoop.vertexIndices.size();
             ++resultOffset) {
            const ArcCurve result = makeResultCurve(
                resultLoop,
                resultVertices,
                resultOffset,
                reverse,
                settings.geometryEpsilon);
            if (result.points.empty()) {
                continue;
            }
            // Exhaustive cyclic seam search. The cost samples at most 128
            // Result vertices per candidate so large boundaries remain usable.
            for (std::size_t sourceOffset = 0U;
                 sourceOffset < source.points.size();
                 ++sourceOffset) {
                const double cost = alignmentCost(
                    result,
                    source,
                    source.vertexParameters[sourceOffset],
                    settings);
                if (cost < best.cost) {
                    best.cost = cost;
                    best.resultOffset = resultOffset;
                    best.sourceOffset = sourceOffset;
                    best.reverseResult = reverse;
                }
            }
        }
    }
    return best;
}

std::vector<CurveSample> sampleSourceCurve(
    const ArcCurve& source,
    std::size_t resultVertexCount,
    const OrderedBoundaryCorrespondenceSettings& settings)
{
    std::vector<CurveSample> samples;
    const std::size_t desiredCount = std::max<std::size_t>(
        source.edgeCount() * 2U,
        resultVertexCount *
            std::max<std::size_t>(settings.samplesPerResultVertex, 2U));
    const std::size_t sampleBudget = std::max<std::size_t>(
        resultVertexCount + 1U,
        std::min(desiredCount, settings.maximumCandidateSamples));

    for (std::size_t edgeIndex = 0U;
         edgeIndex < source.edgeCount();
         ++edgeIndex) {
        const double edgeLength =
            source.cumulativeLengths[edgeIndex + 1U] -
            source.cumulativeLengths[edgeIndex];
        std::size_t subdivisions = std::max<std::size_t>(
            1U,
            static_cast<std::size_t>(std::ceil(
                static_cast<double>(sampleBudget) *
                edgeLength / source.totalLength)));
        for (std::size_t step = 0U; step < subdivisions; ++step) {
            const double edgeParameter =
                static_cast<double>(step) / static_cast<double>(subdivisions);
            const double distance = source.cumulativeLengths[edgeIndex] +
                edgeLength * edgeParameter;
            CurveSample sample;
            sample.edgeIndex = edgeIndex;
            sample.edgeParameter = edgeParameter;
            sample.normalizedParameter = distance / source.totalLength;
            const std::size_t next = source.closed
                ? (edgeIndex + 1U) % source.points.size()
                : edgeIndex + 1U;
            sample.position = source.points[edgeIndex] +
                (source.points[next] - source.points[edgeIndex]) * edgeParameter;
            sample.tangent = source.points[next] - source.points[edgeIndex];
            if (sample.tangent.length() > settings.geometryEpsilon) {
                sample.tangent.normalize();
            }
            samples.push_back(sample);
        }
    }
    if (!source.closed) {
        CurveSample end;
        end.edgeIndex = source.edgeCount() - 1U;
        end.edgeParameter = 1.0;
        end.normalizedParameter = 1.0;
        end.position = source.points.back();
        end.tangent = source.points.back() -
            source.points[source.points.size() - 2U];
        if (end.tangent.length() > settings.geometryEpsilon) {
            end.tangent.normalize();
        }
        samples.push_back(end);
    }
    return samples;
}

double correspondenceCost(
    const MPoint& resultPosition,
    const MVector& resultDirection,
    double desiredParameter,
    const CurveSample& sourceSample,
    double positionScale,
    const OrderedBoundaryCorrespondenceSettings& settings)
{
    const MVector delta = sourceSample.position - resultPosition;
    const double tangentDot = std::clamp(
        resultDirection * sourceSample.tangent,
        -1.0,
        1.0);
    const double arcDifference =
        sourceSample.normalizedParameter - desiredParameter;
    return (delta * delta) / (positionScale * positionScale) +
        settings.tangentCostWeight * (1.0 - tangentDot) * (1.0 - tangentDot) +
        settings.arcLengthCostWeight * arcDifference * arcDifference;
}

double sourceCornerAngle(const ArcCurve& source, std::size_t vertexIndex);

std::vector<std::size_t> forcedCornerAssignments(
    const ArcCurve& source,
    const ArcCurve& result,
    const std::vector<CurveSample>& candidates,
    double positionScale,
    const OrderedBoundaryCorrespondenceSettings& settings)
{
    std::vector<std::size_t> forced(result.points.size(), kInvalidIndex);
    std::vector<std::size_t> cornerSamples;
    cornerSamples.push_back(0U);
    for (std::size_t sourceVertex = 1U;
         sourceVertex < source.points.size();
         ++sourceVertex) {
        const bool openEnd = !source.closed &&
            sourceVertex + 1U == source.points.size();
        if (!openEnd && sourceCornerAngle(source, sourceVertex) <= 1.0e-8) {
            continue;
        }
        const double parameter = source.vertexParameters[sourceVertex];
        const auto found = std::lower_bound(
            candidates.begin(),
            candidates.end(),
            parameter,
            [](const CurveSample& sample, double value) {
                return sample.normalizedParameter < value;
            });
        if (found != candidates.end()) {
            cornerSamples.push_back(static_cast<std::size_t>(
                std::distance(candidates.begin(), found)));
        }
    }
    if (cornerSamples.size() > result.points.size()) {
        // There are not enough Result vertices to represent every Source
        // corner. The later RequiredBoundaryAnchor pass records exactly which
        // corners need Phase 5 split propagation.
        forced[0U] = 0U;
        return forced;
    }

    const std::size_t cornerCount = cornerSamples.size();
    const std::size_t resultCount = result.points.size();
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> costs(
        cornerCount,
        std::vector<double>(resultCount, infinity));
    std::vector<std::vector<std::size_t>> predecessor(
        cornerCount,
        std::vector<std::size_t>(resultCount, kInvalidIndex));
    costs[0U][0U] = correspondenceCost(
        result.points[0U],
        resultTangent(result, 0U),
        0.0,
        candidates[0U],
        positionScale,
        settings);

    for (std::size_t corner = 1U; corner < cornerCount; ++corner) {
        const bool forceOpenEnd = !source.closed &&
            corner + 1U == cornerCount;
        double prefixBest = infinity;
        std::size_t prefixIndex = kInvalidIndex;
        for (std::size_t resultIndex = 1U;
             resultIndex < resultCount;
             ++resultIndex) {
            if (costs[corner - 1U][resultIndex - 1U] < prefixBest) {
                prefixBest = costs[corner - 1U][resultIndex - 1U];
                prefixIndex = resultIndex - 1U;
            }
            if (!std::isfinite(prefixBest) ||
                (forceOpenEnd && resultIndex + 1U != resultCount)) {
                continue;
            }
            const CurveSample& sourceSample = candidates[cornerSamples[corner]];
            costs[corner][resultIndex] = prefixBest + correspondenceCost(
                result.points[resultIndex],
                resultTangent(result, resultIndex),
                sourceSample.normalizedParameter,
                sourceSample,
                positionScale,
                settings);
            predecessor[corner][resultIndex] = prefixIndex;
        }
    }

    std::size_t resultIndex = kInvalidIndex;
    if (!source.closed) {
        resultIndex = resultCount - 1U;
    } else {
        resultIndex = static_cast<std::size_t>(std::distance(
            costs.back().begin(),
            std::min_element(costs.back().begin(), costs.back().end())));
    }
    if (resultIndex >= resultCount ||
        !std::isfinite(costs.back()[resultIndex])) {
        forced[0U] = 0U;
        return forced;
    }
    for (std::size_t corner = cornerCount; corner-- > 0U;) {
        forced[resultIndex] = cornerSamples[corner];
        if (corner > 0U) {
            resultIndex = predecessor[corner][resultIndex];
            if (resultIndex == kInvalidIndex) {
                forced.assign(resultCount, kInvalidIndex);
                forced[0U] = 0U;
                return forced;
            }
        }
    }
    return forced;
}

bool solveMonotonicMapping(
    const ArcCurve& source,
    const ArcCurve& result,
    const OrderedBoundaryCorrespondenceSettings& settings,
    std::vector<CurveSample>& selected,
    std::string& diagnostic)
{
    const std::vector<CurveSample> candidates = sampleSourceCurve(
        source,
        result.points.size(),
        settings);
    const std::size_t resultCount = result.points.size();
    const std::size_t candidateCount = candidates.size();
    if (candidateCount < resultCount || resultCount < 2U) {
        diagnostic = "Ordered Boundary candidate sampling is insufficient.";
        return false;
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> previous(candidateCount, infinity);
    std::vector<double> current(candidateCount, infinity);
    std::vector<std::vector<std::size_t>> predecessor(
        resultCount,
        std::vector<std::size_t>(candidateCount, kInvalidIndex));
    const double positionScale = std::max(
        source.totalLength / static_cast<double>(source.edgeCount()),
        settings.geometryEpsilon);
    const std::vector<std::size_t> forcedCandidates = forcedCornerAssignments(
        source,
        result,
        candidates,
        positionScale,
        settings);

    // Fix the first pair at the seam/end point selected by the exhaustive
    // alignment. All subsequent states must advance strictly in arc order.
    previous[0U] = correspondenceCost(
        result.points[0U],
        resultTangent(result, 0U),
        0.0,
        candidates[0U],
        positionScale,
        settings);

    for (std::size_t resultIndex = 1U;
         resultIndex < resultCount;
         ++resultIndex) {
        std::fill(current.begin(), current.end(), infinity);
        double prefixBest = infinity;
        std::size_t prefixIndex = kInvalidIndex;
        for (std::size_t candidateIndex = 1U;
             candidateIndex < candidateCount;
             ++candidateIndex) {
            const std::size_t previousCandidate = candidateIndex - 1U;
            if (previous[previousCandidate] < prefixBest) {
                prefixBest = previous[previousCandidate];
                prefixIndex = previousCandidate;
            }
            if (!std::isfinite(prefixBest)) {
                continue;
            }
            if (forcedCandidates[resultIndex] != kInvalidIndex &&
                candidateIndex != forcedCandidates[resultIndex]) {
                continue;
            }
            current[candidateIndex] = prefixBest + correspondenceCost(
                result.points[resultIndex],
                resultTangent(result, resultIndex),
                result.vertexParameters[resultIndex],
                candidates[candidateIndex],
                positionScale,
                settings);
            predecessor[resultIndex][candidateIndex] = prefixIndex;
        }
        previous.swap(current);
    }

    std::size_t lastCandidate = kInvalidIndex;
    if (!source.closed) {
        lastCandidate = candidateCount - 1U;
    } else {
        lastCandidate = static_cast<std::size_t>(
            std::distance(
                previous.begin(),
                std::min_element(previous.begin(), previous.end())));
    }
    if (lastCandidate == kInvalidIndex ||
        lastCandidate >= candidateCount ||
        !std::isfinite(previous[lastCandidate])) {
        diagnostic = "Ordered Boundary monotonic optimization has no valid path.";
        return false;
    }

    std::vector<std::size_t> path(resultCount, kInvalidIndex);
    path.back() = lastCandidate;
    for (std::size_t resultIndex = resultCount - 1U;
         resultIndex > 0U;
         --resultIndex) {
        path[resultIndex - 1U] = predecessor[resultIndex][path[resultIndex]];
        if (path[resultIndex - 1U] == kInvalidIndex) {
            diagnostic = "Ordered Boundary monotonic path reconstruction failed.";
            return false;
        }
    }
    selected.reserve(resultCount);
    for (const std::size_t candidateIndex : path) {
        selected.push_back(candidates[candidateIndex]);
    }
    return true;
}

double sourceCornerAngle(const ArcCurve& source, std::size_t vertexIndex)
{
    if (source.points.size() < 3U ||
        (!source.closed &&
         (vertexIndex == 0U || vertexIndex + 1U == source.points.size()))) {
        return 0.0;
    }
    const std::size_t previous = vertexIndex == 0U
        ? source.points.size() - 1U
        : vertexIndex - 1U;
    const std::size_t next = (vertexIndex + 1U) % source.points.size();
    MVector incoming = source.points[vertexIndex] - source.points[previous];
    MVector outgoing = source.points[next] - source.points[vertexIndex];
    if (incoming.length() <= 1.0e-20 || outgoing.length() <= 1.0e-20) {
        return kPi;
    }
    incoming.normalize();
    outgoing.normalize();
    return std::acos(std::clamp(incoming * outgoing, -1.0, 1.0));
}

void findRequiredAnchors(
    const ArcCurve& source,
    const ResultBoundaryLoop& resultLoop,
    const std::vector<CurveSample>& mapping,
    double sourceSeamParameter,
    const OrderedBoundaryCorrespondenceSettings& settings,
    std::vector<RequiredBoundaryAnchor>& anchors)
{
    const double tolerance = std::max(
        settings.geometryEpsilon * 10.0,
        source.totalLength * 1.0e-9);
    const std::size_t resultEdgeCount = resultLoop.closed
        ? resultLoop.vertexIndices.size()
        : resultLoop.vertexIndices.size() - 1U;
    for (std::size_t resultEdgeIndex = 0U;
         resultEdgeIndex < resultEdgeCount;
         ++resultEdgeIndex) {
        const std::size_t nextResult =
            (resultEdgeIndex + 1U) % resultLoop.vertexIndices.size();
        const double lower = mapping[resultEdgeIndex].normalizedParameter;
        const double upper = resultLoop.closed && nextResult == 0U
            ? 1.0
            : mapping[nextResult].normalizedParameter;
        for (std::size_t sourceVertexIndex = 1U;
             sourceVertexIndex < source.points.size();
             ++sourceVertexIndex) {
            const double parameter = source.vertexParameters[sourceVertexIndex];
            if (parameter <= lower + tolerance ||
                parameter >= upper - tolerance) {
                continue;
            }
            const double angle = sourceCornerAngle(source, sourceVertexIndex);
            const double shortcutDistance = pointSegmentDistance(
                source.points[sourceVertexIndex],
                mapping[resultEdgeIndex].position,
                mapping[nextResult].position);
            if (angle < settings.boundaryCornerAngleRadians &&
                shortcutDistance <= tolerance) {
                continue;
            }
            RequiredBoundaryAnchor anchor;
            anchor.sourceVertexId = source.sourceVertexIds[sourceVertexIndex];
            anchor.sourceLoopVertexIndex =
                source.originalVertexIndices[sourceVertexIndex];
            anchor.normalizedArcLength = source.closed
                ? normalizedClosedParameter(
                    sourceSeamParameter + parameter)
                : parameter;
            anchor.resultEdgeIndex = resultEdgeIndex;
            anchor.resultVertex0 = resultLoop.vertexIndices[resultEdgeIndex];
            anchor.resultVertex1 = resultLoop.vertexIndices[nextResult];
            anchor.boundaryCornerAngleRadians = angle;
            anchor.shortcutDistance = shortcutDistance;
            anchor.sourcePosition = source.points[sourceVertexIndex];
            anchors.push_back(anchor);
        }
    }
}

std::size_t countSelfIntersections(
    const ResultBoundaryLoop& loop,
    const std::vector<MPoint>& vertices,
    double tolerance)
{
    const std::size_t edgeCount = loop.closed
        ? loop.vertexIndices.size()
        : loop.vertexIndices.size() - 1U;
    std::size_t intersections = 0U;
    for (std::size_t firstEdge = 0U; firstEdge < edgeCount; ++firstEdge) {
        const std::size_t firstNext =
            (firstEdge + 1U) % loop.vertexIndices.size();
        for (std::size_t secondEdge = firstEdge + 1U;
             secondEdge < edgeCount;
             ++secondEdge) {
            const std::size_t secondNext =
                (secondEdge + 1U) % loop.vertexIndices.size();
            if (firstEdge == secondEdge || firstNext == secondEdge ||
                secondNext == firstEdge ||
                (loop.closed && firstEdge == 0U &&
                 secondNext == 0U)) {
                continue;
            }
            const SegmentDistanceResult distance = segmentDistance(
                vertices[loop.vertexIndices[firstEdge]],
                vertices[loop.vertexIndices[firstNext]],
                vertices[loop.vertexIndices[secondEdge]],
                vertices[loop.vertexIndices[secondNext]]);
            if (distance.distance <= tolerance &&
                distance.firstParameter > tolerance &&
                distance.firstParameter < 1.0 - tolerance &&
                distance.secondParameter > tolerance &&
                distance.secondParameter < 1.0 - tolerance) {
                ++intersections;
            }
        }
    }
    return intersections;
}

std::size_t countSourceCrossings(
    const ArcCurve& source,
    const ResultBoundaryLoop& resultLoop,
    const std::vector<MPoint>& vertices,
    const std::vector<CurveSample>& mapping,
    double tolerance)
{
    const std::size_t resultEdgeCount = resultLoop.closed
        ? resultLoop.vertexIndices.size()
        : resultLoop.vertexIndices.size() - 1U;
    std::size_t crossings = 0U;
    for (std::size_t resultEdge = 0U;
         resultEdge < resultEdgeCount;
         ++resultEdge) {
        const std::size_t resultNext =
            (resultEdge + 1U) % resultLoop.vertexIndices.size();
        const double lower = mapping[resultEdge].normalizedParameter;
        const double upper = resultLoop.closed && resultNext == 0U
            ? 1.0
            : mapping[resultNext].normalizedParameter;
        for (std::size_t sourceEdge = 0U;
             sourceEdge < source.edgeCount();
             ++sourceEdge) {
            const double sourceLower = source.cumulativeLengths[sourceEdge] /
                source.totalLength;
            const double sourceUpper =
                source.cumulativeLengths[sourceEdge + 1U] / source.totalLength;
            const bool belongsToMappedInterval =
                sourceUpper >= lower - tolerance &&
                sourceLower <= upper + tolerance;
            if (belongsToMappedInterval) {
                continue;
            }
            const std::size_t sourceNext = source.closed
                ? (sourceEdge + 1U) % source.points.size()
                : sourceEdge + 1U;
            const SegmentDistanceResult distance = segmentDistance(
                vertices[resultLoop.vertexIndices[resultEdge]],
                vertices[resultLoop.vertexIndices[resultNext]],
                source.points[sourceEdge],
                source.points[sourceNext]);
            if (distance.distance <= tolerance &&
                distance.firstParameter > tolerance &&
                distance.firstParameter < 1.0 - tolerance) {
                ++crossings;
            }
        }
    }
    return crossings;
}

}  // namespace

const OrderedBoundaryCorrespondenceSettings&
OrderedBoundaryCorrespondence::settings() const noexcept
{
    return settings_;
}

void OrderedBoundaryCorrespondence::setSettings(
    const OrderedBoundaryCorrespondenceSettings& settings) noexcept
{
    settings_ = settings;
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 0.0);
    settings_.tangentCostWeight = std::max(settings_.tangentCostWeight, 0.0);
    settings_.arcLengthCostWeight = std::max(settings_.arcLengthCostWeight, 0.0);
    settings_.boundaryCornerAngleRadians = std::clamp(
        settings_.boundaryCornerAngleRadians,
        0.0,
        kPi);
    settings_.samplesPerResultVertex = std::max(
        settings_.samplesPerResultVertex,
        2U);
    settings_.maximumCandidateSamples = std::max<std::size_t>(
        settings_.maximumCandidateSamples,
        64U);
    settings_.seamCandidateCount = std::max<std::size_t>(
        settings_.seamCandidateCount,
        1U);
}

bool OrderedBoundaryCorrespondence::solve(
    const TriangulatedPatch& patch,
    std::size_t sourceLoopIndex,
    ResultBoundaryLoop& resultLoop,
    std::vector<MPoint>& resultVertices,
    BoundaryLoopCorrespondence& correspondence,
    std::string& diagnostic) const
{
    correspondence = BoundaryLoopCorrespondence();
    if (sourceLoopIndex >= patch.boundaryLoops.size() ||
        resultLoop.vertexIndices.size() < 2U) {
        diagnostic = "Ordered Boundary solver received an invalid loop.";
        return false;
    }

    ArcCurve source;
    if (!makeSourceCurve(
            patch,
            patch.boundaryLoops[sourceLoopIndex],
            settings_.geometryEpsilon,
            source,
            diagnostic)) {
        return false;
    }
    if (source.closed != resultLoop.closed) {
        diagnostic = "Source/Result Boundary closed state does not match.";
        return false;
    }

    const Alignment alignment = findAlignment(
        source,
        resultLoop,
        resultVertices,
        settings_);
    if (!std::isfinite(alignment.cost)) {
        diagnostic = "Boundary seam/winding alignment failed.";
        return false;
    }
    const double sourceSeamParameter = source.closed
        ? source.vertexParameters[alignment.sourceOffset]
        : 0.0;
    source = rotateSourceCurve(source, alignment.sourceOffset);
    reorderResultLoop(
        resultLoop,
        alignment.resultOffset,
        alignment.reverseResult);
    const ArcCurve orderedResult = makeResultCurve(
        resultLoop,
        resultVertices,
        0U,
        false,
        settings_.geometryEpsilon);
    if (orderedResult.points.empty()) {
        diagnostic = "Aligned Result Boundary parameterization failed.";
        return false;
    }

    std::vector<CurveSample> mapping;
    if (!solveMonotonicMapping(
            source,
            orderedResult,
            settings_,
            mapping,
            diagnostic)) {
        return false;
    }

    correspondence.sourceLoopIndex = sourceLoopIndex;
    correspondence.sourceClosed = source.closed;
    correspondence.resultClosed = resultLoop.closed;
    correspondence.closedStateMatches = true;
    correspondence.winding = alignment.reverseResult
        ? BoundaryWinding::Reversed
        : BoundaryWinding::Aligned;
    correspondence.sourceSeamParameter = sourceSeamParameter;
    correspondence.resultSeamOffset = alignment.resultOffset;
    correspondence.resultOrderReversed = alignment.reverseResult;
    correspondence.windingAlignedAfterConformation = true;
    correspondence.sourceVertexCount = source.points.size();
    correspondence.sourceEdgeCount = source.edgeCount();
    correspondence.resultVertexCount = orderedResult.points.size();
    correspondence.resultEdgeCount = resultLoop.closed
        ? resultLoop.vertexIndices.size()
        : resultLoop.vertexIndices.size() - 1U;
    correspondence.vertexCountDifference =
        static_cast<long long>(correspondence.resultVertexCount) -
        static_cast<long long>(correspondence.sourceVertexCount);
    correspondence.sourceTotalArcLength = source.totalLength;
    correspondence.resultTotalArcLengthBefore = orderedResult.totalLength;
    correspondence.sourcePolylinePositions = source.points;
    correspondence.vertices.reserve(mapping.size());

    double distanceSum = 0.0;
    for (std::size_t orderIndex = 0U;
         orderIndex < mapping.size();
         ++orderIndex) {
        const CurveSample& sample = mapping[orderIndex];
        const std::size_t resultVertexIndex = resultLoop.vertexIndices[orderIndex];
        const MPoint before = resultVertices[resultVertexIndex];
        const double distance = (before - sample.position).length();
        resultVertices[resultVertexIndex] = sample.position;

        const std::size_t sourceNext = source.closed
            ? (sample.edgeIndex + 1U) % source.points.size()
            : sample.edgeIndex + 1U;
        BoundaryVertexCorrespondence vertex;
        vertex.resultVertexIndex = resultVertexIndex;
        vertex.resultOrderIndex = orderIndex;
        vertex.sourceEdgeId = sample.edgeIndex < source.sourceEdgeIds.size()
            ? source.sourceEdgeIds[sample.edgeIndex]
            : -1;
        vertex.sourceVertex0 = source.sourceVertexIds[sample.edgeIndex];
        vertex.sourceVertex1 = source.sourceVertexIds[sourceNext];
        vertex.sourceEdgeParameter = sample.edgeParameter;
        vertex.resultNormalizedParameter =
            orderedResult.vertexParameters[orderIndex];
        vertex.sourceUnwrappedParameter = sample.normalizedParameter;
        vertex.sourceNormalizedParameter = source.closed
            ? normalizedClosedParameter(
                sourceSeamParameter + sample.normalizedParameter)
            : sample.normalizedParameter;
        vertex.resultPositionBeforeConformation = before;
        vertex.sourcePosition = sample.position;
        vertex.distanceBeforeConformation = distance;
        vertex.distanceAfterConformation = 0.0;
        correspondence.vertices.push_back(vertex);
        distanceSum += distance;
        correspondence.maximumDistanceBefore = std::max(
            correspondence.maximumDistanceBefore,
            distance);
    }
    correspondence.meanDistanceBefore = correspondence.vertices.empty()
        ? 0.0
        : distanceSum / static_cast<double>(correspondence.vertices.size());

    const double parameterTolerance = std::max(
        settings_.geometryEpsilon,
        1.0e-12);
    for (std::size_t index = 1U; index < mapping.size(); ++index) {
        const double delta = mapping[index].normalizedParameter -
            mapping[index - 1U].normalizedParameter;
        if (delta < -parameterTolerance) {
            ++correspondence.monotonicViolationCount;
        }
        if (delta <= parameterTolerance) {
            ++correspondence.duplicatedParameterCount;
        }
    }
    const std::size_t resultEdgeCount = correspondence.resultEdgeCount;
    for (std::size_t edgeIndex = 0U; edgeIndex < resultEdgeCount; ++edgeIndex) {
        const std::size_t next =
            (edgeIndex + 1U) % resultLoop.vertexIndices.size();
        if ((resultVertices[resultLoop.vertexIndices[next]] -
             resultVertices[resultLoop.vertexIndices[edgeIndex]]).length() <=
            settings_.geometryEpsilon) {
            ++correspondence.zeroLengthBoundaryEdgeCount;
        }
    }

    findRequiredAnchors(
        source,
        resultLoop,
        mapping,
        sourceSeamParameter,
        settings_,
        correspondence.requiredBoundaryAnchors);
    const double intersectionTolerance = std::max(
        settings_.geometryEpsilon * 10.0,
        source.totalLength * 1.0e-8);
    correspondence.selfIntersectionCount = countSelfIntersections(
        resultLoop,
        resultVertices,
        intersectionTolerance);
    correspondence.sourceCrossingCount = countSourceCrossings(
        source,
        resultLoop,
        resultVertices,
        mapping,
        intersectionTolerance);
    correspondence.orderedMappingValid =
        correspondence.monotonicViolationCount == 0U &&
        correspondence.duplicatedParameterCount == 0U &&
        correspondence.zeroLengthBoundaryEdgeCount == 0U &&
        correspondence.selfIntersectionCount == 0U &&
        correspondence.sourceCrossingCount == 0U &&
        correspondence.windingAlignedAfterConformation;

    double conformedLength = 0.0;
    for (std::size_t edgeIndex = 0U; edgeIndex < resultEdgeCount; ++edgeIndex) {
        const std::size_t next =
            (edgeIndex + 1U) % resultLoop.vertexIndices.size();
        conformedLength +=
            (resultVertices[resultLoop.vertexIndices[next]] -
             resultVertices[resultLoop.vertexIndices[edgeIndex]]).length();
    }
    resultLoop.totalLength = conformedLength;
    correspondence.resultTotalArcLengthAfter = conformedLength;

    std::ostringstream message;
    message << "Ordered Boundary mapped " << correspondence.resultVertexCount
            << " Result vertices monotonically to "
            << correspondence.sourceEdgeCount << " Source edges; seam "
            << correspondence.resultSeamOffset << ", reversed "
            << (correspondence.resultOrderReversed ? "yes" : "no")
            << ", required anchors "
            << correspondence.requiredBoundaryAnchors.size() << ".";
    diagnostic = message.str();
    return correspondence.orderedMappingValid;
}

}  // namespace directional_retopo
