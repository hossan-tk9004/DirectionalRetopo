#include "Remesh/BoundaryConformer.h"
#include "Remesh/QuadResultValidator.h"

#include <maya/MPoint.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

int main()
{
    using namespace directional_retopo;

    TriangulatedPatch patch;
    patch.vertices = {
        {MPoint(0.0, 0.0, 0.0), 0, true},
        {MPoint(2.0, 0.0, 0.0), 1, true},
        {MPoint(2.0, 2.0, 0.0), 2, true},
        {MPoint(0.0, 2.0, 0.0), 3, true}};
    PatchBoundaryLoop sourceLoop;
    sourceLoop.vertexIndices = {0U, 1U, 2U, 3U};
    sourceLoop.sourceVertexIds = {0, 1, 2, 3};
    sourceLoop.sourceEdgeIds = {10, 11, 12, 13};
    sourceLoop.closed = true;
    patch.boundaryLoops.push_back(sourceLoop);
    patch.triangles.push_back({{0U, 1U, 2U}, 0});
    patch.triangles.push_back({{0U, 2U, 3U}, 0});

    QuadPatchResult result;
    // Deliberately use eight Result vertices, a rotated seam, reversed winding,
    // and an offset from the Source boundary. Topology must remain unchanged.
    result.conformedVertices = {
        MPoint(2.2, 2.2, 0.0),
        MPoint(2.2, 1.0, 0.0),
        MPoint(2.2, -0.2, 0.0),
        MPoint(1.0, -0.2, 0.0),
        MPoint(-0.2, -0.2, 0.0),
        MPoint(-0.2, 1.0, 0.0),
        MPoint(-0.2, 2.2, 0.0),
        MPoint(1.0, 2.2, 0.0)};
    result.polygons.push_back({0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U});
    result.rawVertices = result.conformedVertices;
    result.targetEdgeLength = 1.0;
    const std::size_t originalVertexCount = result.conformedVertices.size();
    const std::vector<std::vector<std::size_t>> originalPolygons = result.polygons;

    BoundaryConformer conformer;
    std::string diagnostic;
    if (!conformer.conform(patch, result, diagnostic)) {
        std::cerr << "Boundary Conformer failed: " << diagnostic << '\n';
        return EXIT_FAILURE;
    }
    if (result.conformedVertices.size() != originalVertexCount ||
        result.polygons != originalPolygons ||
        result.boundaryCorrespondences.size() != 1U) {
        std::cerr << "Boundary Conformer changed topology or lost correspondence\n";
        return EXIT_FAILURE;
    }

    const BoundaryLoopCorrespondence& correspondence =
        result.boundaryCorrespondences.front();
    if (correspondence.sourceVertexCount != 4U ||
        correspondence.resultVertexCount != 8U ||
        correspondence.vertexCountDifference != 4 ||
        correspondence.winding != BoundaryWinding::Reversed ||
        !correspondence.closedStateMatches ||
        !correspondence.windingAlignedAfterConformation ||
        !correspondence.orderedMappingValid ||
        !correspondence.requiredBoundaryAnchors.empty() ||
        correspondence.vertices.size() != 8U ||
        correspondence.meanDistanceBefore <= 0.0) {
        std::cerr << "Boundary correspondence diagnostics are incorrect\n";
        return EXIT_FAILURE;
    }
    double previousUnwrapped = -1.0;
    for (std::size_t orderIndex = 0U;
         orderIndex < correspondence.vertices.size();
         ++orderIndex) {
        const BoundaryVertexCorrespondence& vertex =
            correspondence.vertices[orderIndex];
        if (vertex.resultVertexIndex >= result.conformedVertices.size() ||
            vertex.resultOrderIndex != orderIndex ||
            vertex.sourceEdgeId < 10 || vertex.sourceEdgeId > 13 ||
            vertex.sourceVertex0 < 0 || vertex.sourceVertex1 < 0 ||
            vertex.sourceUnwrappedParameter <= previousUnwrapped ||
            !std::isfinite(vertex.resultNormalizedParameter) ||
            !std::isfinite(vertex.sourceNormalizedParameter) ||
            vertex.resultNormalizedParameter < 0.0 ||
            vertex.resultNormalizedParameter > 1.0 ||
            vertex.sourceNormalizedParameter < 0.0 ||
            vertex.sourceNormalizedParameter > 1.0 ||
            (result.conformedVertices[vertex.resultVertexIndex] -
             vertex.sourcePosition).length() > 1.0e-10) {
            std::cerr << "Boundary vertex is not on its arc-length correspondence\n";
            return EXIT_FAILURE;
        }
        previousUnwrapped = vertex.sourceUnwrappedParameter;
    }

    result.sourceMappings.resize(result.conformedVertices.size());
    for (std::size_t index = 0U; index < result.sourceMappings.size(); ++index) {
        ResultVertexSourceMapping& mapping = result.sourceMappings[index];
        mapping.patchTriangleIndex = 0U;
        mapping.sourceFaceId = 0;
        mapping.rawPosition = result.rawVertices[index];
        mapping.projectedPosition = result.conformedVertices[index];
        mapping.sourceNormal = MVector(0.0, 0.0, 1.0);
        mapping.projectionDistance =
            (mapping.projectedPosition - mapping.rawPosition).length();
        mapping.surfaceDistance = 0.0;
    }
    QuadResultValidator validator;
    if (!validator.validate(patch, result) ||
        result.boundaryDiagnostic.sourceVertexCount != 4U ||
        result.boundaryDiagnostic.resultVertexCount != 8U ||
        result.boundaryDiagnostic.vertexCountDifference != 4 ||
        result.boundaryDiagnostic.reversedLoopCount != 1U ||
        !result.boundaryDiagnostic.correspondenceComplete ||
        result.boundaryDiagnostic.maximumNearestDistance > 1.0e-10) {
        std::cerr << "Boundary correspondence validation failed: "
                  << result.diagnosticMessage << '\n';
        return EXIT_FAILURE;
    }

    // A triangle cannot represent all four square corners without a Result
    // topology split. The ordered mapping remains valid, while the skipped
    // Source corner must be retained as a Phase 5 RequiredBoundaryAnchor.
    QuadPatchResult coarseResult;
    coarseResult.conformedVertices = {
        MPoint(-0.1, -0.1, 0.0),
        MPoint(2.1, -0.1, 0.0),
        MPoint(1.0, 2.1, 0.0)};
    coarseResult.rawVertices = coarseResult.conformedVertices;
    coarseResult.polygons.push_back({0U, 1U, 2U});
    const std::vector<std::vector<std::size_t>> coarseTopology =
        coarseResult.polygons;
    if (!conformer.conform(patch, coarseResult, diagnostic) ||
        coarseResult.polygons != coarseTopology ||
        coarseResult.boundaryCorrespondences.size() != 1U ||
        !coarseResult.boundaryCorrespondences.front().orderedMappingValid ||
        coarseResult.boundaryCorrespondences.front().requiredBoundaryAnchors.empty()) {
        std::cerr << "Required Source corner anchor was not retained: "
                  << diagnostic << '\n';
        return EXIT_FAILURE;
    }
    for (const RequiredBoundaryAnchor& anchor :
         coarseResult.boundaryCorrespondences.front().requiredBoundaryAnchors) {
        if (!anchor.requiresResultSplit || anchor.sourceVertexId < 0 ||
            !std::isfinite(anchor.normalizedArcLength) ||
            anchor.normalizedArcLength < 0.0 ||
            anchor.normalizedArcLength >= 1.0 ||
            anchor.resultVertex0 >= coarseResult.conformedVertices.size() ||
            anchor.resultVertex1 >= coarseResult.conformedVertices.size()) {
            std::cerr << "Required Boundary Anchor metadata is invalid\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << "Boundary Conformer smoke test: success\n"
              << "Source vertices/edges/arc: "
              << correspondence.sourceVertexCount << '/'
              << correspondence.sourceEdgeCount << '/'
              << correspondence.sourceTotalArcLength << '\n'
              << "Result vertices/edges/arc before/after: "
              << correspondence.resultVertexCount << '/'
              << correspondence.resultEdgeCount << '/'
              << correspondence.resultTotalArcLengthBefore << '/'
              << correspondence.resultTotalArcLengthAfter << '\n'
              << "Count difference: "
              << correspondence.vertexCountDifference << '\n'
              << "Winding: reversed\n";
    std::cout << "Required split anchors: "
              << coarseResult.boundaryCorrespondences.front()
                     .requiredBoundaryAnchors.size()
              << '\n';
    return EXIT_SUCCESS;
}
