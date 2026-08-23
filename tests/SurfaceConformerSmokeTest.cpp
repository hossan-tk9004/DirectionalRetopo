#include "Remesh/SurfaceConformer.h"

#include <maya/MPoint.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kColumns = 7U;
constexpr std::size_t kRows = 7U;
constexpr double kPi = 3.14159265358979323846;

std::size_t vertexIndex(std::size_t column, std::size_t row)
{
    return row * kColumns + column;
}

template <typename SurfaceFunction>
directional_retopo::TriangulatedPatch makePatch(SurfaceFunction surface)
{
    using namespace directional_retopo;
    TriangulatedPatch patch;
    for (std::size_t row = 0U; row < kRows; ++row) {
        for (std::size_t column = 0U; column < kColumns; ++column) {
            const MPoint point = surface(column, row);
            const bool boundary = column == 0U || column + 1U == kColumns ||
                row == 0U || row + 1U == kRows;
            patch.vertices.push_back({
                point,
                static_cast<int>(patch.vertices.size()),
                boundary});
        }
    }
    int sourceFaceId = 0;
    for (std::size_t row = 0U; row + 1U < kRows; ++row) {
        for (std::size_t column = 0U; column + 1U < kColumns; ++column) {
            const std::size_t a = vertexIndex(column, row);
            const std::size_t b = vertexIndex(column + 1U, row);
            const std::size_t c = vertexIndex(column + 1U, row + 1U);
            const std::size_t d = vertexIndex(column, row + 1U);
            patch.triangles.push_back({{a, b, c}, sourceFaceId});
            patch.triangles.push_back({{a, c, d}, sourceFaceId});
            ++sourceFaceId;
        }
    }
    PatchBoundaryLoop boundary;
    boundary.closed = true;
    for (std::size_t column = 0U; column < kColumns; ++column) {
        boundary.vertexIndices.push_back(vertexIndex(column, 0U));
    }
    for (std::size_t row = 1U; row < kRows; ++row) {
        boundary.vertexIndices.push_back(vertexIndex(kColumns - 1U, row));
    }
    for (std::size_t column = kColumns - 1U; column-- > 0U;) {
        boundary.vertexIndices.push_back(vertexIndex(column, kRows - 1U));
    }
    for (std::size_t row = kRows - 1U; row-- > 1U;) {
        boundary.vertexIndices.push_back(vertexIndex(0U, row));
    }
    patch.boundaryLoops.push_back(std::move(boundary));
    return patch;
}

directional_retopo::QuadPatchResult makeRawResult(
    const directional_retopo::TriangulatedPatch& patch,
    const std::vector<MPoint>& rawVertices)
{
    using namespace directional_retopo;
    QuadPatchResult result;
    result.componentId = patch.componentId;
    result.targetEdgeLength = 1.0;
    result.rawVertices = rawVertices;
    for (std::size_t row = 0U; row + 1U < kRows; ++row) {
        for (std::size_t column = 0U; column + 1U < kColumns; ++column) {
            result.polygons.push_back({
                vertexIndex(column, row),
                vertexIndex(column + 1U, row),
                vertexIndex(column + 1U, row + 1U),
                vertexIndex(column, row + 1U)});
        }
    }
    return result;
}

bool verify(
    const char* name,
    const directional_retopo::TriangulatedPatch& patch,
    directional_retopo::QuadPatchResult& result,
    double minimumExpectedRawDistance,
    double minimumExpectedConformedZ = -1.0e20,
    double minimumConformedAreaRatio = 0.90)
{
    using namespace directional_retopo;
    SurfaceConformer conformer;
    std::string diagnostic;
    if (!conformer.conform(patch, result, diagnostic)) {
        std::cerr << name << " failed: " << diagnostic << '\n';
        return false;
    }
    if (result.conformedVertices.size() != result.rawVertices.size() ||
        result.sourceMappings.size() != result.rawVertices.size() ||
        result.fidelity.meanRawSurfaceDistance < minimumExpectedRawDistance ||
        result.fidelity.maximumConformedSurfaceDistance > 1.0e-8 ||
        result.fidelity.conformedAreaRatio < minimumConformedAreaRatio) {
        std::cerr << name << " produced unexpected fidelity metrics: "
                  << diagnostic << '\n';
        return false;
    }
    for (const MPoint& point : result.conformedVertices) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z) || point.z < minimumExpectedConformedZ) {
            std::cerr << name << " produced an invalid Conformed vertex\n";
            return false;
        }
    }
    std::cout << name
              << ": raw max distance="
              << result.fidelity.maximumRawSurfaceDistance
              << ", conformed max distance="
              << result.fidelity.maximumConformedSurfaceDistance
              << ", raw mean distance="
              << result.fidelity.meanRawSurfaceDistance
              << ", mean/max projection="
              << result.fidelity.meanProjectionDistance << '/'
              << result.fidelity.maximumProjectionDistance
              << ", conformed mean distance="
              << result.fidelity.meanConformedSurfaceDistance
              << ", raw/conformed area ratio="
              << result.fidelity.rawAreaRatio << '/'
              << result.fidelity.conformedAreaRatio << '\n';
    return true;
}

}  // namespace

int main()
{
    using namespace directional_retopo;

    const TriangulatedPatch plane = makePatch(
        [](std::size_t column, std::size_t row) {
            return MPoint(
                static_cast<double>(column),
                static_cast<double>(row),
                0.0);
        });
    std::vector<MPoint> planeRaw;
    for (const PatchVertex& vertex : plane.vertices) {
        planeRaw.emplace_back(vertex.position.x, vertex.position.y, -0.35);
    }
    QuadPatchResult planeResult = makeRawResult(plane, planeRaw);
    if (!verify("plane", plane, planeResult, 0.34)) {
        return EXIT_FAILURE;
    }

    const TriangulatedPatch sphere = makePatch(
        [](std::size_t column, std::size_t row) {
            const double u = (static_cast<double>(column) / (kColumns - 1U) - 0.5) * 1.2;
            const double v = (static_cast<double>(row) / (kRows - 1U) - 0.5) * 1.2;
            const double z = std::sqrt(std::max(4.0 - u * u - v * v, 0.0));
            return MPoint(u, v, z);
        });
    std::vector<MPoint> sphereRaw;
    for (const PatchVertex& vertex : sphere.vertices) {
        sphereRaw.emplace_back(
            vertex.position.x * 0.82,
            vertex.position.y * 0.82,
            vertex.position.z * 0.82);
    }
    QuadPatchResult sphereResult = makeRawResult(sphere, sphereRaw);
    if (!verify("high-curvature sphere", sphere, sphereResult, 0.20)) {
        return EXIT_FAILURE;
    }

    const TriangulatedPatch chestShoulder = makePatch(
        [](std::size_t column, std::size_t row) {
            const double x = static_cast<double>(column) * 0.4;
            const double y = static_cast<double>(row) * 0.4;
            const double chest = 0.90 * std::exp(
                -((x - 1.00) * (x - 1.00) / 0.42 +
                  (y - 1.15) * (y - 1.15) / 0.62));
            const double shoulder = 0.58 * std::exp(
                -((x - 2.15) * (x - 2.15) / 0.30 +
                  (y - 1.45) * (y - 1.45) / 0.80));
            return MPoint(x, y, 0.35 + chest + shoulder);
        });
    std::vector<MPoint> chestShoulderRaw;
    for (const PatchVertex& vertex : chestShoulder.vertices) {
        chestShoulderRaw.emplace_back(
            vertex.position.x,
            vertex.position.y,
            0.35 + (vertex.position.z - 0.35) * 0.55);
    }
    QuadPatchResult chestShoulderResult = makeRawResult(
        chestShoulder,
        chestShoulderRaw);
    if (!verify(
            "analytic chest/shoulder curvature",
            chestShoulder,
            chestShoulderResult,
            0.08)) {
        return EXIT_FAILURE;
    }

    const TriangulatedPatch cloth = makePatch(
        [](std::size_t column, std::size_t row) {
            const double x = static_cast<double>(column) * 0.5;
            const double y = static_cast<double>(row) * 0.5;
            const double z = 2.0 + 0.28 * std::sin(x * kPi * 0.75) *
                std::cos(y * kPi * 0.65);
            return MPoint(x, y, z);
        });
    // Simulate Raw output collapsing toward a nearby, unrelated body layer.
    // Only the selected cloth component is made available to SurfaceConformer.
    std::vector<MPoint> clothRaw;
    for (const PatchVertex& vertex : cloth.vertices) {
        clothRaw.emplace_back(vertex.position.x, vertex.position.y, 0.25);
    }
    QuadPatchResult clothResult = makeRawResult(cloth, clothRaw);
    if (!verify("near-surface cloth isolation", cloth, clothResult, 1.4, 1.65)) {
        return EXIT_FAILURE;
    }

    std::cout << "SurfaceConformer smoke tests: success\n";
    return EXIT_SUCCESS;
}
