#pragma once

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/BoundaryGeometryValidator.h"
#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"

#include <cstddef>
#include <string>
#include <vector>

namespace directional_retopo {

struct TransitionCollarSettings final
{
    TopologyPolicy topologyPolicy = TopologyPolicy::QuadDominant;
    TrianglePolicy trianglePolicy = TrianglePolicy::MinimalNecessary;
    unsigned int topologyBlendWidth = 2U;
    double trianglePenalty = 8.0;
    double aspectRatioWeight = 0.75;
    double edgeLengthWeight = 0.35;
    double directionWeight = 0.50;
    double geometryEpsilon = 1.0e-10;
    double relativeIntersectionTolerance = 1.0e-7;
    double interiorParameterTolerance = 1.0e-8;
};

struct TransitionCollarBuildResult final
{
    bool success = false;
    std::vector<std::vector<std::size_t>> polygons;
    std::vector<TriangleDiagnostic> triangleDiagnostics;
    std::vector<std::size_t> alignedInnerVertexIndices;
    std::size_t quadCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t crossingCount = 0U;
    std::size_t seamOffset = 0U;
    bool innerOrderReversed = false;
    std::string diagnosticMessage;
    double alignmentCost = 0.0;
    BoundaryLoopValidationDiagnostic outerValidation;
    BoundaryLoopValidationDiagnostic innerValidation;
    CollarPolygonValidationDiagnostic collarValidation;
};

class TransitionCollarBuilder final
{
public:
    [[nodiscard]] const TransitionCollarSettings& settings() const noexcept;
    void setSettings(const TransitionCollarSettings& settings) noexcept;

    bool build(
        const std::vector<MPoint>& vertices,
        const std::vector<std::size_t>& orderedOuter,
        const std::vector<std::size_t>& orderedInner,
        bool closed,
        const TriangulatedPatch& sourcePatch,
        const DirectionFieldData& directionField,
        const DensityFieldData& densityField,
        TransitionCollarBuildResult& result) const;

private:
    TransitionCollarSettings settings_;
};

}  // namespace directional_retopo
