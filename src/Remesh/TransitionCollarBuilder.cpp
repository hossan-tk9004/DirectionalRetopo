#include "Remesh/TransitionCollarBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>

namespace directional_retopo {
namespace {

enum class Move : unsigned char { None, Quad, OuterTriangle, InnerTriangle };

struct Cell final
{
    double cost = std::numeric_limits<double>::infinity();
    Move move = Move::None;
};

double distanceSquared(const MPoint& first, const MPoint& second)
{
    const MVector delta = first - second;
    return delta * delta;
}
double representativeTargetEdgeLength(const DensityFieldData& densityField)
{
    std::vector<double> values;
    values.reserve(densityField.perFace.size());
    for (const FaceDensity& density : densityField.perFace) {
        if (density.valid && std::isfinite(density.targetEdgeLength) &&
            density.targetEdgeLength > 0.0) {
            values.push_back(density.targetEdgeLength);
        }
    }
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if (values.size() % 2U == 0U) {
        result = (*std::max_element(values.begin(), values.begin() + middle) +
                  result) * 0.5;
    }
    return result;
}

BoundaryGeometryValidationSettings geometryValidationSettings(
    const TransitionCollarSettings& settings)
{
    BoundaryGeometryValidationSettings result;
    result.absoluteTolerance = settings.geometryEpsilon;
    result.relativeIntersectionTolerance =
        settings.relativeIntersectionTolerance;
    result.interiorParameterTolerance =
        settings.interiorParameterTolerance;
    result.relativeAreaTolerance = settings.relativeAreaTolerance;
    result.minimumNormalizedArea = settings.minimumNormalizedArea;
    return result;
}



double loopLength(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& loop)
{
    double result = 0.0;
    for (std::size_t i = 0U; i < loop.size(); ++i) {
        result += (vertices[loop[(i + 1U) % loop.size()]] -
            vertices[loop[i]]).length();
    }
    return result;
}

std::vector<double> parameters(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& loop)
{
    std::vector<double> result(loop.size(), 0.0);
    const double total = loopLength(vertices, loop);
    double cumulative = 0.0;
    for (std::size_t i = 1U; i < loop.size() && total > 0.0; ++i) {
        cumulative += (vertices[loop[i]] - vertices[loop[i - 1U]]).length();
        result[i] = cumulative / total;
    }
    return result;
}

std::vector<std::size_t> orderedCandidate(
    const std::vector<std::size_t>& loop,
    std::size_t seam,
    bool reverse)
{
    std::vector<std::size_t> result;
    result.reserve(loop.size());
    for (std::size_t i = 0U; i < loop.size(); ++i) {
        const std::size_t index = reverse
            ? (seam + loop.size() - i) % loop.size()
            : (seam + i) % loop.size();
        result.push_back(loop[index]);
    }
    return result;
}

double alignmentCost(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& outer,
    const std::vector<std::size_t>& inner)
{
    const std::vector<double> outerParameters = parameters(vertices, outer);
    const std::vector<double> innerParameters = parameters(vertices, inner);
    double result = 0.0;
    for (std::size_t i = 0U; i < inner.size(); ++i) {
        std::size_t nearest = 0U;
        double nearestParameter = std::numeric_limits<double>::infinity();
        for (std::size_t j = 0U; j < outer.size(); ++j) {
            const double direct = std::abs(
                innerParameters[i] - outerParameters[j]);
            const double wrapped = std::min(direct, 1.0 - direct);
            if (wrapped < nearestParameter) {
                nearestParameter = wrapped;
                nearest = j;
            }
        }
        result += distanceSquared(vertices[inner[i]], vertices[outer[nearest]]) +
            nearestParameter * nearestParameter;
    }
    return result;
}

int nearestSourceFace(const MPoint& point, const TriangulatedPatch& patch)
{
    int result = -1;
    double nearest = std::numeric_limits<double>::infinity();
    for (const PatchTriangle& triangle : patch.triangles) {
        MPoint center(0.0, 0.0, 0.0);
        for (const std::size_t index : triangle.vertexIndices) {
            center += MVector(
                patch.vertices[index].position.x,
                patch.vertices[index].position.y,
                patch.vertices[index].position.z);
        }
        center.x /= 3.0;
        center.y /= 3.0;
        center.z /= 3.0;
        const double distance = distanceSquared(center, point);
        if (distance < nearest) {
            nearest = distance;
            result = triangle.sourceFaceId;
        }
    }
    return result;
}

double polygonCost(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& polygon,
    const TriangulatedPatch& sourcePatch,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField,
    const TransitionCollarSettings& settings,
    bool triangle,
    std::size_t& rejectedZeroArea,
    std::size_t& rejectedSliver)
{
    std::set<std::size_t> uniqueVertices;
    for (const std::size_t index : polygon) {
        if (index >= vertices.size() ||
            !std::isfinite(vertices[index].x) ||
            !std::isfinite(vertices[index].y) ||
            !std::isfinite(vertices[index].z) ||
            !uniqueVertices.insert(index).second) {
            ++rejectedZeroArea;
            return std::numeric_limits<double>::infinity();
        }
    }
    MPoint center(0.0, 0.0, 0.0);
    std::vector<MVector> edges;
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = 0.0;
    for (const std::size_t index : polygon) {
        center += MVector(vertices[index].x, vertices[index].y, vertices[index].z);
    }
    center.x /= static_cast<double>(polygon.size());
    center.y /= static_cast<double>(polygon.size());
    center.z /= static_cast<double>(polygon.size());
    for (std::size_t i = 0U; i < polygon.size(); ++i) {
        const MVector edge =
            vertices[polygon[(i + 1U) % polygon.size()]] -
            vertices[polygon[i]];
        edges.push_back(edge);
        minimum = std::min(minimum, edge.length());
        maximum = std::max(maximum, edge.length());
    }
    const double relativeEdgeTolerance =
        settings.geometryEpsilon + maximum * 1.0e-9;
    if (!(minimum > relativeEdgeTolerance)) {
        ++rejectedZeroArea;
        return std::numeric_limits<double>::infinity();
    }

    const MPoint& origin = vertices[polygon.front()];
    MVector areaNormal = MVector::zero;
    for (std::size_t index = 1U; index + 1U < polygon.size(); ++index) {
        areaNormal += (vertices[polygon[index]] - origin) ^
            (vertices[polygon[index + 1U]] - origin);
    }
    const double area = areaNormal.length() * 0.5;
    const double areaScale = std::max(maximum, settings.geometryEpsilon);
    const double areaTolerance = std::max(
        settings.geometryEpsilon * settings.geometryEpsilon,
        areaScale * areaScale * settings.relativeAreaTolerance);
    if (!(area > areaTolerance) || !std::isfinite(area)) {
        ++rejectedZeroArea;
        return std::numeric_limits<double>::infinity();
    }
    if (area / (areaScale * areaScale) <
        settings.minimumNormalizedArea) {
        ++rejectedSliver;
        return std::numeric_limits<double>::infinity();
    }
    if (polygon.size() == 4U) {
        const MVector first = (vertices[polygon[1U]] - origin) ^
            (vertices[polygon[2U]] - origin);
        const MVector second = (vertices[polygon[2U]] - origin) ^
            (vertices[polygon[3U]] - origin);
        if (first.length() <= areaTolerance ||
            second.length() <= areaTolerance ||
            first * second < -0.05 * first.length() * second.length()) {
            ++rejectedSliver;
            return std::numeric_limits<double>::infinity();
        }
    }

    const int sourceFaceId = nearestSourceFace(center, sourcePatch);
    const FaceDensity* density = densityField.face(sourceFaceId);
    const double target =
        density != nullptr && density->valid &&
        density->targetEdgeLength > settings.geometryEpsilon
        ? density->targetEdgeLength
        : maximum;
    double densityCost = 0.0;
    for (const MVector& edge : edges) {
        densityCost += std::abs(std::log(
            edge.length() / std::max(target, settings.geometryEpsilon)));
    }

    double directionCost = 0.0;
    const FaceDirectionField* field = directionField.face(sourceFaceId);
    if (field != nullptr && field->valid) {
        for (MVector edge : edges) {
            edge -= field->normal * (edge * field->normal);
            if (edge.length() <= settings.geometryEpsilon) {
                continue;
            }
            edge.normalize();
            directionCost += 1.0 - std::max(
                std::abs(edge * field->uDirection),
                std::abs(edge * field->vDirection));
        }
        directionCost *= std::clamp(
            field->constraintWeight + field->topologyGuidanceWeight,
            0.15,
            1.0);
    }

    return (triangle ? settings.trianglePenalty : 0.0) +
        settings.aspectRatioWeight * (maximum / minimum - 1.0) +
        settings.edgeLengthWeight * densityCost +
        settings.directionWeight * directionCost;
}

}  // namespace

const TransitionCollarSettings&
TransitionCollarBuilder::settings() const noexcept
{
    return settings_;
}

void TransitionCollarBuilder::setSettings(
    const TransitionCollarSettings& settings) noexcept
{
    settings_ = settings;
    settings_.topologyBlendWidth = std::max(settings_.topologyBlendWidth, 1U);
    settings_.trianglePenalty = std::max(settings_.trianglePenalty, 0.0);
    settings_.aspectRatioWeight = std::max(settings_.aspectRatioWeight, 0.0);
    settings_.edgeLengthWeight = std::max(settings_.edgeLengthWeight, 0.0);
    settings_.directionWeight = std::max(settings_.directionWeight, 0.0);
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 1.0e-15);
    settings_.relativeIntersectionTolerance =
        std::max(settings_.relativeIntersectionTolerance, 0.0);
    settings_.interiorParameterTolerance = std::clamp(
        settings_.interiorParameterTolerance, 0.0, 0.49);
    settings_.relativeAreaTolerance =
        std::max(settings_.relativeAreaTolerance, 0.0);
    settings_.minimumNormalizedArea =
        std::max(settings_.minimumNormalizedArea, 0.0);
}

bool TransitionCollarBuilder::build(
    const std::vector<MPoint>& vertices,
    const std::vector<std::size_t>& orderedOuter,
    const std::vector<std::size_t>& orderedInner,
    bool closed,
    const TriangulatedPatch& sourcePatch,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField,
    TransitionCollarBuildResult& result) const
{
    result = TransitionCollarBuildResult();
    if (!closed) {
        result.diagnosticMessage =
            "Open Boundary Collar is unsupported in Phase 5A Preview.";
        return false;
    }
    if (orderedOuter.size() < 3U || orderedInner.size() < 3U) {
        result.diagnosticMessage =
            "Transition Collar requires two closed loops with at least three vertices.";
        return false;
    }
    for (const std::size_t index : orderedOuter) {
        if (index >= vertices.size()) {
            result.diagnosticMessage = "Fixed Boundary index is invalid.";
            return false;
        }
    }
    for (const std::size_t index : orderedInner) {
        if (index >= vertices.size()) {
            result.diagnosticMessage = "Inner Boundary index is invalid.";
            return false;
        }
    }
    const double targetEdgeLength =
        representativeTargetEdgeLength(densityField);
    const BoundaryGeometryValidationSettings validationSettings =
        geometryValidationSettings(settings_);
    result.outerValidation = BoundaryGeometryValidator::validateClosedLoop(
        vertices,
        orderedOuter,
        targetEdgeLength,
        validationSettings);
    result.innerValidation = BoundaryGeometryValidator::validateClosedLoop(
        vertices,
        orderedInner,
        targetEdgeLength,
        validationSettings);
    if (!result.outerValidation.valid || !result.innerValidation.valid) {
        result.crossingCount =
            result.outerValidation.trueIntersectionCount +
            result.innerValidation.trueIntersectionCount;
        std::ostringstream message;
        message << "Transition Collar boundary validation failed. Outer: vertices="
                << result.outerValidation.vertexCount << ", topologySimple="
                << (result.outerValidation.topologySimple ? "true" : "false")
                << ", 3D intersections="
                << result.outerValidation.trueIntersectionCount << ", status="
                << (result.outerValidation.valid ? "pass" : "fail")
                << "; Inner: vertices=" << result.innerValidation.vertexCount
                << ", topologySimple="
                << (result.innerValidation.topologySimple ? "true" : "false")
                << ", 3D intersections="
                << result.innerValidation.trueIntersectionCount << ", status="
                << (result.innerValidation.valid ? "pass" : "fail") << ". ";
        if (!result.outerValidation.valid) {
            message << "Outer Source Boundary failed: "
                    << result.outerValidation.message << ' ';
        }
        if (!result.innerValidation.valid) {
            message << "Inner Remesh Boundary failed: "
                    << result.innerValidation.message;
        }
        result.diagnosticMessage = message.str();
        return false;
    }

    const std::size_t outerCount = orderedOuter.size();
    const std::size_t innerCount = orderedInner.size();
    const bool allowTriangles =
        settings_.topologyPolicy == TopologyPolicy::QuadDominant &&
        settings_.trianglePolicy == TrianglePolicy::MinimalNecessary;
    const auto at = [innerCount](std::size_t i, std::size_t j) {
        return i * (innerCount + 1U) + j;
    };

    double bestTotalCost = std::numeric_limits<double>::infinity();
    std::vector<std::size_t> bestInner;
    std::vector<std::vector<std::size_t>> bestPolygons;
    CollarPolygonValidationDiagnostic bestValidation;
    // Seam/winding candidates reuse the same local polygons. Cache their
    // geometry and field cost so exhaustive validation remains practical.
    std::map<std::vector<std::size_t>, double> polygonCostCache;

    for (const bool reverse : {false, true}) {
        for (std::size_t seam = 0U; seam < orderedInner.size(); ++seam) {
            ++result.seamCandidatesTested;
            std::vector<std::size_t> aligned =
                orderedCandidate(orderedInner, seam, reverse);
            const double candidateAlignment =
                alignmentCost(vertices, orderedOuter, aligned);

            std::vector<Cell> dp((outerCount + 1U) * (innerCount + 1U));
            dp[at(0U, 0U)].cost = 0.0;
            const auto update = [&](std::size_t i,
                                    std::size_t j,
                                    std::size_t nextI,
                                    std::size_t nextJ,
                                    Move move,
                                    const std::vector<std::size_t>& polygon,
                                    bool triangle) {
                if (!std::isfinite(dp[at(i, j)].cost)) {
                    return;
                }
                std::vector<std::size_t> cacheKey = polygon;
                const auto cached = polygonCostCache.find(cacheKey);
                const double local = cached != polygonCostCache.end()
                    ? cached->second : polygonCost(
                    vertices,
                    polygon,
                    sourcePatch,
                    directionField,
                    densityField,
                    settings_,
                    triangle,
                    result.rejectedZeroAreaCandidateCount,
                    result.rejectedSliverCandidateCount);
                if (cached == polygonCostCache.end()) {
                    polygonCostCache.emplace(std::move(cacheKey), local);
                }
                if (!std::isfinite(local)) {
                    return;
                }
                const double candidateCost = dp[at(i, j)].cost + local;
                if (candidateCost + settings_.geometryEpsilon <
                    dp[at(nextI, nextJ)].cost) {
                    dp[at(nextI, nextJ)] = {candidateCost, move};
                }
            };

            for (std::size_t i = 0U; i <= outerCount; ++i) {
                for (std::size_t j = 0U; j <= innerCount; ++j) {
                    if (i < outerCount && j < innerCount) {
                        update(i, j, i + 1U, j + 1U, Move::Quad, {
                            orderedOuter[i % outerCount],
                            orderedOuter[(i + 1U) % outerCount],
                            aligned[(j + 1U) % innerCount],
                            aligned[j % innerCount]}, false);
                    }
                    if (allowTriangles && i < outerCount) {
                        update(i, j, i + 1U, j, Move::OuterTriangle, {
                            orderedOuter[i % outerCount],
                            orderedOuter[(i + 1U) % outerCount],
                            aligned[j % innerCount]}, true);
                    }
                    if (allowTriangles && j < innerCount) {
                        update(i, j, i, j + 1U, Move::InnerTriangle, {
                            orderedOuter[i % outerCount],
                            aligned[(j + 1U) % innerCount],
                            aligned[j % innerCount]}, true);
                    }
                }
            }

            const double dpCost = dp[at(outerCount, innerCount)].cost;
            if (!std::isfinite(dpCost)) {
                continue;
            }
            ++result.dpFeasibleCandidateCount;

            std::vector<std::vector<std::size_t>> candidatePolygons;
            std::size_t i = outerCount;
            std::size_t j = innerCount;
            bool backtrackValid = true;
            while (i > 0U || j > 0U) {
                const Move move = dp[at(i, j)].move;
                if (move == Move::Quad) {
                    candidatePolygons.push_back({
                        orderedOuter[(i - 1U) % outerCount],
                        orderedOuter[i % outerCount],
                        aligned[j % innerCount],
                        aligned[(j - 1U) % innerCount]});
                    --i;
                    --j;
                } else if (move == Move::OuterTriangle) {
                    candidatePolygons.push_back({
                        orderedOuter[(i - 1U) % outerCount],
                        orderedOuter[i % outerCount],
                        aligned[j % innerCount]});
                    --i;
                } else if (move == Move::InnerTriangle) {
                    candidatePolygons.push_back({
                        orderedOuter[i % outerCount],
                        aligned[j % innerCount],
                        aligned[(j - 1U) % innerCount]});
                    --j;
                } else {
                    backtrackValid = false;
                    break;
                }
            }
            if (!backtrackValid) {
                continue;
            }
            std::reverse(candidatePolygons.begin(), candidatePolygons.end());
            CollarPolygonValidationDiagnostic validation =
                BoundaryGeometryValidator::validateCollarPolygons(
                    vertices,
                    candidatePolygons,
                    sourcePatch,
                    targetEdgeLength,
                    validationSettings);
            if (!validation.valid) {
                continue;
            }
            ++result.geometryValidCandidateCount;

            const double totalCost = candidateAlignment + dpCost;
            if (totalCost < bestTotalCost) {
                bestTotalCost = totalCost;
                bestInner = std::move(aligned);
                bestPolygons = std::move(candidatePolygons);
                bestValidation = std::move(validation);
                result.seamOffset = seam;
                result.innerOrderReversed = reverse;
                result.alignmentCost = candidateAlignment;
            }
        }
    }

    if (bestPolygons.empty()) {
        std::ostringstream message;
        message << "No geometry-valid monotonic Transition Collar was found "
                << "after " << result.seamCandidatesTested
                << " seam/winding candidates (DP feasible "
                << result.dpFeasibleCandidateCount << ", final-valid "
                << result.geometryValidCandidateCount
                << ", rejected zero-area/sliver moves "
                << result.rejectedZeroAreaCandidateCount << '/'
                << result.rejectedSliverCandidateCount << ").";
        result.diagnosticMessage = message.str();
        return false;
    }
    result.alignedInnerVertexIndices = std::move(bestInner);
    result.polygons = std::move(bestPolygons);
    result.collarValidation = std::move(bestValidation);
    result.crossingCount = result.collarValidation.trueIntersectionCount;

    for (std::size_t polygonIndex = 0U;
         polygonIndex < result.polygons.size();
         ++polygonIndex) {
        if (result.polygons[polygonIndex].size() == 4U) {
            ++result.quadCount;
        } else {
            ++result.triangleCount;
            result.triangleDiagnostics.push_back({
                polygonIndex,
                outerCount != innerCount
                    ? TriangleReason::BoundaryCountMismatch
                    : TriangleReason::FlowTermination});
        }
    }

    result.success = true;
    std::ostringstream message;
    message << "Ordered DP Collar: " << result.quadCount << " quads, "
            << result.triangleCount << " triangles, seam "
            << result.seamOffset << ", winding "
            << (result.innerOrderReversed ? "reversed" : "aligned")
            << ", alignment cost " << result.alignmentCost
            << ", blend width " << settings_.topologyBlendWidth
            << ". Outer(vertices=" << result.outerValidation.vertexCount
            << ", topologySimple="
            << (result.outerValidation.topologySimple ? "true" : "false")
            << ", 3D intersections="
            << result.outerValidation.trueIntersectionCount
            << "); Inner(vertices=" << result.innerValidation.vertexCount
            << ", topologySimple="
            << (result.innerValidation.topologySimple ? "true" : "false")
            << ", 3D intersections="
            << result.innerValidation.trueIntersectionCount << ").";
    result.diagnosticMessage = message.str();
    return true;
}

}  // namespace directional_retopo
