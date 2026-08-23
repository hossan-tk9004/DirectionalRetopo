#include "Remesh/TransitionCollarBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
    bool triangle)
{
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
    if (!(minimum > settings.geometryEpsilon)) {
        return std::numeric_limits<double>::infinity();
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
    settings_.trianglePenalty = std::max(settings_.trianglePenalty, 0.0);
    settings_.aspectRatioWeight = std::max(settings_.aspectRatioWeight, 0.0);
    settings_.edgeLengthWeight = std::max(settings_.edgeLengthWeight, 0.0);
    settings_.directionWeight = std::max(settings_.directionWeight, 0.0);
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 1.0e-15);
    settings_.relativeIntersectionTolerance =
        std::max(settings_.relativeIntersectionTolerance, 0.0);
    settings_.interiorParameterTolerance = std::clamp(
        settings_.interiorParameterTolerance, 0.0, 0.49);
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

    double bestCost = std::numeric_limits<double>::infinity();
    for (const bool reverse : {false, true}) {
        for (std::size_t seam = 0U; seam < orderedInner.size(); ++seam) {
            std::vector<std::size_t> candidate =
                orderedCandidate(orderedInner, seam, reverse);
            const double cost = alignmentCost(vertices, orderedOuter, candidate);
            if (cost < bestCost) {
                bestCost = cost;
                result.alignedInnerVertexIndices = std::move(candidate);
                result.seamOffset = seam;
                result.innerOrderReversed = reverse;
            }
        }
    }
    result.alignmentCost = bestCost;

    const std::size_t outerCount = orderedOuter.size();
    const std::size_t innerCount = result.alignedInnerVertexIndices.size();
    std::vector<Cell> dp((outerCount + 1U) * (innerCount + 1U));
    const auto at = [innerCount](std::size_t i, std::size_t j) {
        return i * (innerCount + 1U) + j;
    };
    dp[at(0U, 0U)].cost = 0.0;
    const auto update = [&](std::size_t i,
                            std::size_t j,
                            std::size_t nextI,
                            std::size_t nextJ,
                            Move move,
                            std::vector<std::size_t> polygon,
                            bool triangle) {
        if (!std::isfinite(dp[at(i, j)].cost)) {
            return;
        }
        const double local = polygonCost(
            vertices,
            polygon,
            sourcePatch,
            directionField,
            densityField,
            settings_,
            triangle);
        const double candidate = dp[at(i, j)].cost + local;
        if (std::isfinite(local) &&
            candidate + settings_.geometryEpsilon < dp[at(nextI, nextJ)].cost) {
            dp[at(nextI, nextJ)] = {candidate, move};
        }
    };

    const bool allowTriangles =
        settings_.topologyPolicy == TopologyPolicy::QuadDominant &&
        settings_.trianglePolicy == TrianglePolicy::MinimalNecessary;
    for (std::size_t i = 0U; i <= outerCount; ++i) {
        for (std::size_t j = 0U; j <= innerCount; ++j) {
            if (i < outerCount && j < innerCount) {
                update(i, j, i + 1U, j + 1U, Move::Quad, {
                    orderedOuter[i % outerCount],
                    orderedOuter[(i + 1U) % outerCount],
                    result.alignedInnerVertexIndices[(j + 1U) % innerCount],
                    result.alignedInnerVertexIndices[j % innerCount]}, false);
            }
            if (allowTriangles &&
                i < outerCount) {
                update(i, j, i + 1U, j, Move::OuterTriangle, {
                    orderedOuter[i % outerCount],
                    orderedOuter[(i + 1U) % outerCount],
                    result.alignedInnerVertexIndices[j % innerCount]}, true);
            }
            if (allowTriangles &&
                j < innerCount) {
                update(i, j, i, j + 1U, Move::InnerTriangle, {
                    orderedOuter[i % outerCount],
                    result.alignedInnerVertexIndices[(j + 1U) % innerCount],
                    result.alignedInnerVertexIndices[j % innerCount]}, true);
            }
        }
    }
    if (!std::isfinite(dp[at(outerCount, innerCount)].cost)) {
        result.diagnosticMessage = settings_.topologyBlendWidth == 0U
            ? "Topology Blend Width is too small for the requested density."
            : "No valid monotonic Transition Collar was found.";
        return false;
    }

    std::size_t i = outerCount;
    std::size_t j = innerCount;
    while (i > 0U || j > 0U) {
        const Move move = dp[at(i, j)].move;
        if (move == Move::Quad) {
            result.polygons.push_back({
                orderedOuter[(i - 1U) % outerCount],
                orderedOuter[i % outerCount],
                result.alignedInnerVertexIndices[j % innerCount],
                result.alignedInnerVertexIndices[(j - 1U) % innerCount]});
            --i;
            --j;
        } else if (move == Move::OuterTriangle) {
            result.polygons.push_back({
                orderedOuter[(i - 1U) % outerCount],
                orderedOuter[i % outerCount],
                result.alignedInnerVertexIndices[j % innerCount]});
            --i;
        } else if (move == Move::InnerTriangle) {
            result.polygons.push_back({
                orderedOuter[i % outerCount],
                result.alignedInnerVertexIndices[j % innerCount],
                result.alignedInnerVertexIndices[(j - 1U) % innerCount]});
            --j;
        } else {
            result.diagnosticMessage = "Transition Collar DP backtracking failed.";
            return false;
        }
    }
    std::reverse(result.polygons.begin(), result.polygons.end());
    result.collarValidation =
        BoundaryGeometryValidator::validateCollarPolygons(
            vertices,
            result.polygons,
            sourcePatch,
            targetEdgeLength,
            validationSettings);
    if (!result.collarValidation.valid) {
        result.crossingCount =
            result.collarValidation.trueIntersectionCount;
        result.diagnosticMessage =
            "Transition Collar polygon validation failed: " +
            result.collarValidation.message;
        result.polygons.clear();
        return false;
    }
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
