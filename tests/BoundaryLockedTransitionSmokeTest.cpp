#include "Remesh/TransitionCollarBuilder.h"
#include "Remesh/BoundaryGeometryValidator.h"
#include "Remesh/QuadResultValidator.h"
#include "Remesh/BoundaryLockedPatchBuilder.h"

#include <maya/MPoint.h>

#include <cmath>
#include <iostream>
#include <vector>
#include <string>

namespace {

using namespace directional_retopo;

std::vector<std::size_t> appendLoop(
    std::vector<MPoint>& vertices,
    std::size_t count,
    double radius,
    bool bowTie = false)
{
    std::vector<std::size_t> loop;
    static constexpr double kTau = 6.28318530717958647692;
    static constexpr std::size_t kBowTieOrder[4] = {0U, 2U, 1U, 3U};
    for (std::size_t index = 0U; index < count; ++index) {
        const std::size_t ordered = bowTie && count == 4U
            ? kBowTieOrder[index] : index;
        const double angle = kTau * static_cast<double>(ordered) /
            static_cast<double>(count);
        loop.push_back(vertices.size());
        vertices.emplace_back(
            radius * std::cos(angle),
            radius * std::sin(angle),
            0.0);
    }
    return loop;
}

TriangulatedPatch sourcePatch()
{
    TriangulatedPatch patch;
    patch.vertices = {
        {{-2.0, -2.0, 0.0}, 0, true},
        {{ 2.0, -2.0, 0.0}, 1, true},
        {{ 2.0,  2.0, 0.0}, 2, true},
        {{-2.0,  2.0, 0.0}, 3, true},
    };
    PatchTriangle first;
    first.vertexIndices = {0U, 1U, 2U};
    first.sourceFaceId = 0;
    PatchTriangle second;
    second.vertexIndices = {0U, 2U, 3U};
    second.sourceFaceId = 0;
    patch.triangles = {first, second};
    return patch;
}
bool runNonPlanarProjectionFalsePositiveCase()
{
    const std::vector<MPoint> corners = {
        {-1.0, -1.0,  0.0},
        { 1.0,  1.0,  1.0},
        {-1.0,  1.0,  0.0},
        { 1.0, -1.0, -1.0},
    };
    std::vector<MPoint> vertices;
    for (std::size_t index = 0U; index < corners.size(); ++index) {
        const MPoint& first = corners[index];
        const MPoint& second = corners[(index + 1U) % corners.size()];
        vertices.push_back(first);
        vertices.push_back(first + (second - first) * 0.5);
    }
    std::vector<std::size_t> loop;
    for (std::size_t index = 0U; index < vertices.size(); ++index) {
        loop.push_back(index);
    }

    const BoundaryLoopValidationDiagnostic diagnostic =
        BoundaryGeometryValidator::validateClosedLoop(
            vertices,
            loop,
            1.0,
            BoundaryGeometryValidationSettings());
    if (!diagnostic.valid || !diagnostic.topologySimple ||
        diagnostic.trueIntersectionCount != 0U) {
        std::cerr << "Non-planar projection false-positive case failed: "
                  << diagnostic.message << std::endl;
        return false;
    }
    return true;
}

MPoint curvedPoint(double radius, double angle)
{
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    const double z =
        0.18 * x * x - 0.12 * y * y + 0.08 * std::sin(3.0 * angle);
    return MPoint(x, y, z);
}

std::vector<std::size_t> appendCurvedLoop(
    std::vector<MPoint>& vertices,
    std::size_t count,
    double radius)
{
    static constexpr double kTau = 6.28318530717958647692;
    std::vector<std::size_t> loop;
    loop.reserve(count);
    for (std::size_t index = 0U; index < count; ++index) {
        const double angle =
            kTau * static_cast<double>(index) / static_cast<double>(count);
        loop.push_back(vertices.size());
        vertices.push_back(curvedPoint(radius, angle));
    }
    return loop;
}

TriangulatedPatch curvedSourcePatch(std::size_t outerCount)
{
    static constexpr double kTau = 6.28318530717958647692;
    TriangulatedPatch patch;
    for (std::size_t index = 0U; index < outerCount; ++index) {
        const double angle =
            kTau * static_cast<double>(index) / static_cast<double>(outerCount);
        patch.vertices.push_back({
            curvedPoint(2.0, angle),
            static_cast<int>(index),
            true});
    }
    const std::size_t center = patch.vertices.size();
    patch.vertices.push_back({curvedPoint(0.0, 0.0), -1, false});
    for (std::size_t index = 0U; index < outerCount; ++index) {
        PatchTriangle triangle;
        triangle.vertexIndices = {
            center,
            index,
            (index + 1U) % outerCount};
        triangle.sourceFaceId = 0;
        patch.triangles.push_back(triangle);
    }
    return patch;
}

bool runCurvedCollarCase(
    std::size_t outerCount,
    std::size_t innerCount,
    std::size_t expectedTriangles)
{
    std::vector<MPoint> vertices;
    const std::vector<std::size_t> outer =
        appendCurvedLoop(vertices, outerCount, 2.0);
    const std::vector<std::size_t> inner =
        appendCurvedLoop(vertices, innerCount, 1.0);

    DirectionFieldData directions;
    directions.perFace.resize(1U);
    directions.perFace[0].normal = MVector(0.0, 0.0, 1.0);
    directions.perFace[0].uDirection = MVector(1.0, 0.0, 0.0);
    directions.perFace[0].vDirection = MVector(0.0, 1.0, 0.0);
    directions.perFace[0].valid = true;
    DensityFieldData density;
    density.perFace.resize(1U);
    density.perFace[0].targetEdgeLength = 1.0;
    density.perFace[0].valid = true;

    TransitionCollarBuilder builder;
    TransitionCollarBuildResult result;
    const bool success = builder.build(
        vertices,
        outer,
        inner,
        true,
        curvedSourcePatch(outerCount),
        directions,
        density,
        result);
    if (!success ||
        result.triangleCount != expectedTriangles ||
        result.outerValidation.trueIntersectionCount != 0U ||
        result.innerValidation.trueIntersectionCount != 0U ||
        result.collarValidation.trueIntersectionCount != 0U) {
        std::cerr << "Curved collar case failed: "
                  << result.diagnosticMessage << std::endl;
        return false;
    }
    return true;
}

bool runTrue3DCrossingCase()
{
    const std::vector<MPoint> vertices = {
        {-1.0, -1.0, 0.0},
        { 1.0,  1.0, 0.0},
        {-1.0,  1.0, 0.0},
        { 1.0, -1.0, 0.0},
    };
    const std::vector<std::size_t> loop = {0U, 1U, 2U, 3U};
    const BoundaryLoopValidationDiagnostic diagnostic =
        BoundaryGeometryValidator::validateClosedLoop(
            vertices,
            loop,
            1.0,
            BoundaryGeometryValidationSettings());
    if (diagnostic.valid || diagnostic.trueIntersectionCount == 0U) {
        std::cerr << "True 3D crossing was not rejected." << std::endl;
        return false;
    }
    return true;
}

bool runBranchedInnerBoundaryCase()
{
    QuadPatchResult result;
    result.rawVertices = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0},
    };
    result.conformedVertices = result.rawVertices;
    result.polygons = {
        {0U, 1U, 2U},
        {0U, 3U, 4U},
    };
    result.targetEdgeLength = 1.0;

    QuadResultValidator validator;
    TriangulatedPatch emptyPatch;
    if (validator.validate(emptyPatch, result) ||
        result.diagnosticMessage.find("branched/non-manifold") ==
            std::string::npos) {
        std::cerr << "Branched Inner Boundary did not fail explicitly: "
                  << result.diagnosticMessage << std::endl;
        return false;
    }
    return true;
}


bool runCase(
    std::size_t outerCount,
    std::size_t innerCount,
    std::size_t expectedTriangles,
    bool bowTie = false,
    bool strict = false,
    unsigned int topologyBlendWidth = 2U)
{
    std::vector<MPoint> vertices;
    const std::vector<std::size_t> outer =
        appendLoop(vertices, outerCount, 2.0, bowTie);
    const std::vector<MPoint> originalOuter = vertices;
    const std::vector<std::size_t> inner =
        appendLoop(vertices, innerCount, 1.0);

    DirectionFieldData directions;
    directions.perFace.resize(1U);
    directions.perFace[0].normal = MVector(0.0, 0.0, 1.0);
    directions.perFace[0].uDirection = MVector(1.0, 0.0, 0.0);
    directions.perFace[0].vDirection = MVector(0.0, 1.0, 0.0);
    directions.perFace[0].valid = true;
    DensityFieldData density;
    density.perFace.resize(1U);
    density.perFace[0].targetEdgeLength = 1.0;
    density.perFace[0].valid = true;

    TransitionCollarBuilder builder;
    TransitionCollarSettings settings = builder.settings();
    settings.topologyBlendWidth = topologyBlendWidth;
    if (strict) {
        settings.topologyPolicy = TopologyPolicy::StrictAllQuads;
    }
    builder.setSettings(settings);
    TransitionCollarBuildResult result;
    const bool success = builder.build(
        vertices,
        outer,
        inner,
        true,
        sourcePatch(),
        directions,
        density,
        result);
    if (strict) {
        return !success;
    }
    if (bowTie) {
        return !success &&
            result.outerValidation.trueIntersectionCount > 0U;
    }
    if (!success || result.triangleCount != expectedTriangles ||
        result.quadCount != std::min(outerCount, innerCount)) {
        std::cerr << result.diagnosticMessage << std::endl;
        return false;
    }
    for (std::size_t index = 0U; index < outerCount; ++index) {
        if ((vertices[index] - originalOuter[index]).length() != 0.0) {
            return false;
        }
    }
    return true;
}

bool runBoundaryLockedBuilderCase()
{
    static constexpr std::size_t kOuterCount = 8U;
    static constexpr std::size_t kInnerCount = 4U;
    static constexpr double kTau = 6.28318530717958647692;
    TriangulatedPatch patch;
    patch.componentId = 0U;
    PatchBoundaryLoop sourceLoop;
    sourceLoop.closed = true;
    for (std::size_t index = 0U; index < kOuterCount; ++index) {
        const double angle = kTau * static_cast<double>(index) /
            static_cast<double>(kOuterCount);
        patch.vertices.push_back({
            MPoint(2.0 * std::cos(angle), 2.0 * std::sin(angle), 0.0),
            static_cast<int>(index),
            true});
        sourceLoop.vertexIndices.push_back(index);
        sourceLoop.sourceVertexIds.push_back(static_cast<int>(index));
        sourceLoop.sourceEdgeIds.push_back(static_cast<int>(index));
    }
    // Reproduce Maya's legacy closed traversal: N edges, N+1 entries, last == first.
    sourceLoop.vertexIndices.push_back(sourceLoop.vertexIndices.front());
    sourceLoop.sourceVertexIds.push_back(sourceLoop.sourceVertexIds.front());
    const std::size_t centerIndex = patch.vertices.size();
    patch.vertices.push_back({MPoint(0.0, 0.0, 0.0), 8, false});
    for (std::size_t index = 0U; index < kOuterCount; ++index) {
        PatchTriangle triangle;
        triangle.vertexIndices = {
            centerIndex,
            index,
            (index + 1U) % kOuterCount};
        triangle.sourceFaceId = 0;
        patch.triangles.push_back(triangle);
    }
    patch.boundaryLoops.push_back(sourceLoop);

    QuadPatchResult inner;
    inner.success = true;
    inner.targetEdgeLength = 1.0;
    ResultBoundaryLoop innerLoop;
    innerLoop.closed = true;
    for (std::size_t index = 0U; index < kInnerCount; ++index) {
        const double angle = kTau * static_cast<double>(index) /
            static_cast<double>(kInnerCount);
        inner.conformedVertices.emplace_back(std::cos(angle), std::sin(angle), 0.0);
        innerLoop.vertexIndices.push_back(index);
    }
    inner.polygons.push_back({0U, 1U, 2U, 3U});
    inner.boundaryLoops.push_back(innerLoop);

    DirectionFieldData directions;
    directions.perFace.resize(1U);
    directions.perFace[0].normal = MVector(0.0, 0.0, 1.0);
    directions.perFace[0].uDirection = MVector(1.0, 0.0, 0.0);
    directions.perFace[0].vDirection = MVector(0.0, 1.0, 0.0);
    directions.perFace[0].valid = true;
    DensityFieldData density;
    density.perFace.resize(1U);
    density.perFace[0].targetEdgeLength = 1.0;
    density.perFace[0].valid = true;

    BoundaryLockedPatchBuilder builder;
    BoundaryLockedPatchBuilderSettings settings = builder.settings();
    settings.topologyBlendWidth = 5U;
    builder.setSettings(settings);
    QuadPatchResult result;
    std::string diagnostic;
    if (!builder.build(patch, inner, directions, density, result, diagnostic)) {
        std::cerr << diagnostic << std::endl;
        return false;
    }
    if (!result.boundaryLocked || result.boundaryLoops.size() != 1U ||
        result.boundaryLoops[0].vertexIndices.size() != kOuterCount ||
        result.fixedBoundaryVertexIndices.size() != kOuterCount ||
        result.boundaryCorrespondences.size() != 1U ||
        result.boundaryCorrespondences[0].sourceVertexCount != kOuterCount ||
        result.boundaryCorrespondences[0].sourceEdgeCount != kOuterCount ||
        result.boundaryLockedDiagnostic.maximumSourceBoundaryDisplacement != 0.0 ||
        result.boundaryLockedDiagnostic.boundaryCrossingCount != 0U ||
        result.boundaryLockedDiagnostic.outerBoundaryTrueIntersectionCount != 0U ||
        result.boundaryLockedDiagnostic.innerBoundaryTrueIntersectionCount != 0U ||
        !result.boundaryLockedDiagnostic.outerBoundaryTopologySimple ||
        !result.boundaryLockedDiagnostic.innerBoundaryTopologySimple ||
        result.boundaryLockedDiagnostic.collarTriangleCount != 4U ||
        result.boundaryLockedDiagnostic.coreTriangleCount != 0U ||
        result.boundaryLockedDiagnostic.topologyBlendWidth != 5U) {
        std::cerr << "Boundary-Locked result diagnostic mismatch." << std::endl;
        return false;
    }
    for (std::size_t index = 0U; index < kOuterCount; ++index) {
        const std::size_t resultIndex = result.boundaryLoops[0].vertexIndices[index];
        if ((result.rawVertices[resultIndex] - patch.vertices[index].position).length() != 0.0) {
            std::cerr << "Fixed Source Boundary moved." << std::endl;
            return false;
        }
        std::size_t edgeUseCount = 0U;
        const std::size_t nextResultIndex =
            result.boundaryLoops[0].vertexIndices[(index + 1U) % kOuterCount];
        for (std::size_t polygonIndex = 1U;
             polygonIndex < result.polygons.size();
             ++polygonIndex) {
            const auto& polygon = result.polygons[polygonIndex];
            for (std::size_t edge = 0U; edge < polygon.size(); ++edge) {
                const std::size_t first = polygon[edge];
                const std::size_t second = polygon[(edge + 1U) % polygon.size()];
                if ((first == resultIndex && second == nextResultIndex) ||
                    (first == nextResultIndex && second == resultIndex)) {
                    ++edgeUseCount;
                }
            }
        }
        if (edgeUseCount != 1U) {
            std::cerr << "Fixed Source Boundary connectivity changed." << std::endl;
            return false;
        }
    }
    return true;
}

}  // namespace

int main()
{
    if (!runCase(8U, 4U, 4U) ||
        !runCase(8U, 4U, 4U, false, false, 0U) ||
        !runCase(8U, 4U, 4U, false, false, 5U) ||
        !runCase(7U, 4U, 3U) ||
        !runCase(4U, 4U, 0U) ||
        !runCase(12U, 3U, 9U) ||
        !runCase(4U, 3U, 1U) ||
        !runCase(8U, 4U, 0U, false, true) ||
        !runBoundaryLockedBuilderCase() ||
        !runCase(4U, 3U, 0U, true) ||
        !runNonPlanarProjectionFalsePositiveCase() ||
        !runCurvedCollarCase(8U, 4U, 4U) ||
        !runCurvedCollarCase(12U, 5U, 7U) ||
        !runTrue3DCrossingCase() ||
        !runBranchedInnerBoundaryCase()) {
        return 1;
    }
    std::cout
        << "BoundaryLockedTransitionSmokeTest passed: fixed boundary, "
        << "quad-preferred DP, minimal triangles, crossing hard failure."
        << std::endl;
    return 0;
}
