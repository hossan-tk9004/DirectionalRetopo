#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/AutoRemesherAdapter.h"
#include "Remesh/LocalPatch.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

namespace {

constexpr std::size_t kColumns = 7U;
constexpr std::size_t kRows = 7U;
constexpr double kBaseTargetEdgeLength = 0.55;

std::size_t vertexIndex(std::size_t column, std::size_t row)
{
    return row * kColumns + column;
}

directional_retopo::TriangulatedPatch makeCurvedPatch()
{
    using namespace directional_retopo;
    TriangulatedPatch patch;
    patch.componentId = 0U;
    for (std::size_t row = 0U; row < kRows; ++row) {
        for (std::size_t column = 0U; column < kColumns; ++column) {
            const double x =
                (static_cast<double>(column) / (kColumns - 1U) - 0.5) * 2.8;
            const double y =
                (static_cast<double>(row) / (kRows - 1U) - 0.5) * 2.8;
            const double z = std::sqrt(std::max(4.0 - x * x - y * y, 0.0));
            const bool boundary = column == 0U || column + 1U == kColumns ||
                row == 0U || row + 1U == kRows;
            patch.vertices.push_back({
                MPoint(x, y, z),
                static_cast<int>(patch.vertices.size()),
                boundary});
        }
    }

    const std::size_t faceCount = (kColumns - 1U) * (kRows - 1U);
    patch.sourceFaceToTriangleIndices.resize(faceCount);
    std::size_t faceId = 0U;
    for (std::size_t row = 0U; row + 1U < kRows; ++row) {
        for (std::size_t column = 0U; column + 1U < kColumns; ++column) {
            const std::size_t a = vertexIndex(column, row);
            const std::size_t b = vertexIndex(column + 1U, row);
            const std::size_t c = vertexIndex(column + 1U, row + 1U);
            const std::size_t d = vertexIndex(column, row + 1U);
            patch.sourceFaceToTriangleIndices[faceId] = {
                patch.triangles.size(),
                patch.triangles.size() + 1U};
            patch.triangles.push_back({{a, b, c}, static_cast<int>(faceId)});
            patch.triangles.push_back({{a, c, d}, static_cast<int>(faceId)});
            ++faceId;
        }
    }

    PatchBoundaryLoop loop;
    loop.closed = true;
    for (std::size_t column = 0U; column < kColumns; ++column) {
        loop.vertexIndices.push_back(vertexIndex(column, 0U));
    }
    for (std::size_t row = 1U; row < kRows; ++row) {
        loop.vertexIndices.push_back(vertexIndex(kColumns - 1U, row));
    }
    for (std::size_t column = kColumns - 1U; column-- > 0U;) {
        loop.vertexIndices.push_back(vertexIndex(column, kRows - 1U));
    }
    for (std::size_t row = kRows - 1U; row-- > 1U;) {
        loop.vertexIndices.push_back(vertexIndex(0U, row));
    }
    for (const std::size_t index : loop.vertexIndices) {
        loop.sourceVertexIds.push_back(static_cast<int>(index));
    }
    patch.boundaryLoops.push_back(std::move(loop));
    return patch;
}

double meanResultEdgeLength(const directional_retopo::QuadPatchResult& result)
{
    std::set<std::pair<std::size_t, std::size_t>> edges;
    for (const std::vector<std::size_t>& polygon : result.polygons) {
        for (std::size_t index = 0U; index < polygon.size(); ++index) {
            const std::size_t first = polygon[index];
            const std::size_t second = polygon[(index + 1U) % polygon.size()];
            edges.insert(first < second
                ? std::make_pair(first, second)
                : std::make_pair(second, first));
        }
    }
    double sum = 0.0;
    std::size_t count = 0U;
    for (const auto& edge : edges) {
        if (edge.second < result.conformedVertices.size()) {
            const double length =
                (result.conformedVertices[edge.second] -
                 result.conformedVertices[edge.first]).length();
            if (std::isfinite(length) && length > 1.0e-10) {
                sum += length;
                ++count;
            }
        }
    }
    return count > 0U ? sum / static_cast<double>(count) : 0.0;
}

struct ScaleMeasurement final
{
    double scale = 1.0;
    double effectiveTarget = 0.0;
    std::size_t polygonCount = 0U;
    std::size_t quadCount = 0U;
    double meanEdgeLength = 0.0;
    directional_retopo::SurfaceFidelityMetrics fidelity;
};

bool measure(
    const directional_retopo::TriangulatedPatch& patch,
    double scale,
    ScaleMeasurement& measurement)
{
    using namespace directional_retopo;
    const std::size_t faceCount = (kColumns - 1U) * (kRows - 1U);
    DirectionFieldData directionField;
    DensityFieldData densityField;
    directionField.perFace.resize(faceCount);
    densityField.perFace.resize(faceCount);
    for (std::size_t faceId = 0U; faceId < faceCount; ++faceId) {
        const PatchTriangle& triangle =
            patch.triangles[patch.sourceFaceToTriangleIndices[faceId].front()];
        const MPoint& a = patch.vertices[triangle.vertexIndices[0]].position;
        const MPoint& b = patch.vertices[triangle.vertexIndices[1]].position;
        const MPoint& c = patch.vertices[triangle.vertexIndices[2]].position;
        MVector normal = (b - a) ^ (c - a);
        normal.normalize();
        MVector u = MVector(1.0, 0.0, 0.0) -
            normal * (normal * MVector(1.0, 0.0, 0.0));
        if (u.length() <= 1.0e-10) {
            u = MVector(0.0, 1.0, 0.0) -
                normal * (normal * MVector(0.0, 1.0, 0.0));
        }
        u.normalize();
        MVector v = normal ^ u;
        v.normalize();
        directionField.perFace[faceId] = {
            normal,
            u,
            v,
            1.0,
            0.0,
            true,
            true};

        FaceDensity& density = densityField.perFace[faceId];
        density.targetEdgeLength = kBaseTargetEdgeLength * scale;
        density.baseTargetEdgeLength = density.targetEdgeLength;
        density.referenceEdgeLength = kBaseTargetEdgeLength;
        density.curvatureTargetEdgeLength = density.targetEdgeLength;
        density.curvatureRefinementFactor = 1.0 / scale;
        density.scaleU = scale;
        density.scaleV = scale;
        density.valid = true;
    }

    AutoRemesherAdapter adapter;
    AutoRemesherInput input;
    std::string diagnostic;
    if (!adapter.buildInput(patch, directionField, densityField, input, diagnostic)) {
        std::cerr << "Scale " << scale << " buildInput failed: "
                  << diagnostic << '\n';
        return false;
    }
    measurement.scale = scale;
    measurement.effectiveTarget =
        input.globalScaling * input.patchAverageEdgeLength;
    if (std::abs(measurement.effectiveTarget -
                 kBaseTargetEdgeLength * scale) > 1.0e-9) {
        std::cerr << "Scale " << scale
                  << " produced an incorrect effective target\n";
        return false;
    }

    const QuadComponentSolveReport report = adapter.solve(
        patch,
        directionField,
        densityField);
    if (!report.result.success) {
        std::cerr << "Scale " << scale << " solve failed: "
                  << report.diagnosticMessage << '\n';
        return false;
    }
    measurement.polygonCount = report.result.polygons.size();
    measurement.quadCount = report.result.quadCount;
    measurement.meanEdgeLength = meanResultEdgeLength(report.result);
    measurement.fidelity = report.result.fidelity;
    return true;
}

}  // namespace

int main()
{
    const directional_retopo::TriangulatedPatch patch = makeCurvedPatch();
    std::vector<ScaleMeasurement> measurements(3U);
    const std::vector<double> scales = {1.0, 0.5, 0.2};
    for (std::size_t index = 0U; index < scales.size(); ++index) {
        if (!measure(patch, scales[index], measurements[index])) {
            return EXIT_FAILURE;
        }
    }
    for (const ScaleMeasurement& measurement : measurements) {
        std::cout << "Scale=" << measurement.scale
                  << " effectiveTarget=" << measurement.effectiveTarget
                  << " polygons=" << measurement.polygonCount
                  << " quads=" << measurement.quadCount
                  << " sourceArea=" << measurement.fidelity.sourceArea
                  << " rawArea=" << measurement.fidelity.rawQuadArea
                  << " rawAreaRatio=" << measurement.fidelity.rawAreaRatio
                  << " conformedArea=" << measurement.fidelity.conformedArea
                  << " conformedAreaRatio="
                  << measurement.fidelity.conformedAreaRatio
                  << " meanRawDistance="
                  << measurement.fidelity.meanRawSurfaceDistance
                  << " maxRawDistance="
                  << measurement.fidelity.maximumRawSurfaceDistance
                  << " meanConformedDistance="
                  << measurement.fidelity.meanConformedSurfaceDistance
                  << " maxConformedDistance="
                  << measurement.fidelity.maximumConformedSurfaceDistance
                  << " meanHighCurvatureEdgeLength="
                  << measurement.meanEdgeLength << '\n';
    }
    if (!(measurements[0].polygonCount < measurements[1].polygonCount &&
          measurements[1].polygonCount < measurements[2].polygonCount) ||
        !(measurements[0].fidelity.rawAreaRatio <
              measurements[1].fidelity.rawAreaRatio &&
          measurements[1].fidelity.rawAreaRatio <
              measurements[2].fidelity.rawAreaRatio)) {
        std::cerr << "Smaller Scale did not improve curved-patch sampling\n";
        return EXIT_FAILURE;
    }
    std::cout << "Scale fidelity diagnostic: success\n";
    return EXIT_SUCCESS;
}
