#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/AutoRemesherAdapter.h"
#include "Remesh/LocalPatch.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

constexpr std::size_t kColumns = 5U;
constexpr std::size_t kRows = 4U;

std::size_t vertexIndex(std::size_t column, std::size_t row)
{
    return row * kColumns + column;
}

directional_retopo::TriangulatedPatch makePatch()
{
    using namespace directional_retopo;

    TriangulatedPatch patch;
    patch.componentId = 0U;
    for (std::size_t row = 0; row < kRows; ++row) {
        for (std::size_t column = 0; column < kColumns; ++column) {
            const std::size_t index = vertexIndex(column, row);
            const bool boundary = column == 0U || column + 1U == kColumns ||
                row == 0U || row + 1U == kRows;
            patch.vertices.push_back({
                MPoint(static_cast<double>(column), static_cast<double>(row), 0.0),
                static_cast<int>(index),
                boundary});
        }
    }

    const std::size_t faceCount = (kColumns - 1U) * (kRows - 1U);
    patch.sourceFaceToTriangleIndices.resize(faceCount);
    std::size_t faceId = 0U;
    for (std::size_t row = 0; row + 1U < kRows; ++row) {
        for (std::size_t column = 0; column + 1U < kColumns; ++column) {
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
    for (std::size_t column = 0; column < kColumns; ++column) {
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

}  // namespace

int main()
{
    using namespace directional_retopo;

    const TriangulatedPatch patch = makePatch();
    const std::size_t faceCount = (kColumns - 1U) * (kRows - 1U);
    DirectionFieldData directionField;
    DensityFieldData densityField;
    directionField.perFace.resize(faceCount);
    densityField.perFace.resize(faceCount);
    for (std::size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex) {
        FaceDirectionField& direction = directionField.perFace[faceIndex];
        direction.normal = MVector(0.0, 0.0, 1.0);
        direction.uDirection = MVector(1.0, 0.0, 0.0);
        direction.vDirection = MVector(0.0, 1.0, 0.0);
        direction.constraintWeight = 1.0;
        direction.hasPaintConstraint = true;
        direction.valid = true;

        FaceDensity& density = densityField.perFace[faceIndex];
        density.targetEdgeLength = 1.0;
        density.referenceEdgeLength = 1.0;
        density.scaleU = 1.0;
        density.scaleV = 1.0;
        density.valid = true;
    }

    AutoRemesherAdapter adapter;
    AutoRemesherInput input;
    std::string diagnostic;
    if (!adapter.buildInput(patch, directionField, densityField, input, diagnostic)) {
        std::cerr << "buildInput failed: " << diagnostic << '\n';
        return EXIT_FAILURE;
    }
    const double effectiveBaseSpacing = input.globalScaling *
        input.patchAverageEdgeLength;
    if (!std::isfinite(effectiveBaseSpacing) ||
        std::abs(effectiveBaseSpacing - 1.0) > 1.0e-9) {
        std::cerr << "density adapter produced an unexpected base spacing: "
                  << effectiveBaseSpacing << '\n';
        return EXIT_FAILURE;
    }

    QuadComponentSolveReport report = adapter.solve(
        patch,
        directionField,
        densityField);
    if (!report.parameterizationSuccess || !report.extractionSuccess ||
        !report.conformationSuccess || !report.result.success ||
        report.result.rawVertices.empty() ||
        report.result.conformedVertices.empty() ||
        report.result.polygons.empty() ||
        report.result.rawVertices.size() != report.result.conformedVertices.size() ||
        report.result.sourceMappings.size() !=
            report.result.conformedVertices.size()) {
        std::cerr << "adapter solve failed: " << report.diagnosticMessage << '\n';
        return EXIT_FAILURE;
    }
    for (const MPoint& point : report.result.conformedVertices) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            std::cerr << "adapter returned a non-finite point\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "adapter parameterization success\n"
              << "adapter quad extraction success\n"
              << "adapter surface conformation success\n"
              << "result vertices: "
              << report.result.conformedVertices.size() << '\n'
              << "result polygons: " << report.result.polygons.size() << '\n'
              << "quads: " << report.result.quadCount << '\n'
              << "non-quads: " << report.result.nonQuadCount << '\n'
              << "max surface distance: "
              << report.result.maximumSurfaceDistance << '\n'
              << "raw area ratio: "
              << report.result.fidelity.rawAreaRatio << '\n'
              << "conformed area ratio: "
              << report.result.fidelity.conformedAreaRatio << '\n'
              << "raw max surface distance: "
              << report.result.fidelity.maximumRawSurfaceDistance << '\n'
              << "conformed max surface distance: "
              << report.result.fidelity.maximumConformedSurfaceDistance << '\n'
              << "parameterization ms: "
              << report.timings.parameterizationMilliseconds << '\n'
              << "extraction ms: "
              << report.timings.extractionMilliseconds << '\n'
              << "conformation ms: "
              << report.timings.conformationMilliseconds << '\n'
              << "validation ms: "
              << report.timings.validationMilliseconds << '\n'
              << "total ms: " << report.timings.totalMilliseconds << '\n';
    return EXIT_SUCCESS;
}
