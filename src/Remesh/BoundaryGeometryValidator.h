#pragma once

#include "Remesh/LocalPatch.h"

#include <maya/MPoint.h>

#include <cstddef>
#include <string>
#include <vector>

namespace directional_retopo {

struct BoundaryGeometryValidationSettings final
{
    double absoluteTolerance = 1.0e-10;
    double relativeIntersectionTolerance = 1.0e-7;
    double interiorParameterTolerance = 1.0e-8;
};

struct BoundaryLoopValidationDiagnostic final
{
    bool valid = false;
    bool topologySimple = false;
    std::size_t vertexCount = 0U;
    std::size_t duplicateVertexCount = 0U;
    std::size_t zeroLengthEdgeCount = 0U;
    std::size_t trueIntersectionCount = 0U;
    double localEdgeScale = 0.0;
    double intersectionTolerance = 0.0;
    std::string message;
};

struct CollarPolygonValidationDiagnostic final
{
    bool valid = false;
    std::size_t invalidIndexCount = 0U;
    std::size_t repeatedVertexCount = 0U;
    std::size_t zeroLengthEdgeCount = 0U;
    std::size_t zeroAreaPolygonCount = 0U;
    std::size_t nonManifoldEdgeCount = 0U;
    std::size_t trueIntersectionCount = 0U;
    std::size_t reversedPolygonCount = 0U;
    double localEdgeScale = 0.0;
    double intersectionTolerance = 0.0;
    std::string message;
};

class BoundaryGeometryValidator final
{
public:
    [[nodiscard]] static BoundaryLoopValidationDiagnostic validateClosedLoop(
        const std::vector<MPoint>& vertices,
        const std::vector<std::size_t>& orderedLoop,
        double localTargetEdgeLength,
        const BoundaryGeometryValidationSettings& settings);

    [[nodiscard]] static CollarPolygonValidationDiagnostic validateCollarPolygons(
        const std::vector<MPoint>& vertices,
        std::vector<std::vector<std::size_t>>& polygons,
        const TriangulatedPatch& sourcePatch,
        double localTargetEdgeLength,
        const BoundaryGeometryValidationSettings& settings);
};

}  // namespace directional_retopo
