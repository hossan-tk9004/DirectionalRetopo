#pragma once

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"
#include "Remesh/TransitionCollarBuilder.h"

#include <string>

namespace directional_retopo {

struct BoundaryLockedPatchBuilderSettings final
{
    TopologyPolicy topologyPolicy = TopologyPolicy::QuadDominant;
    TrianglePolicy trianglePolicy = TrianglePolicy::MinimalNecessary;
    unsigned int topologyBlendWidth = 2U;
    double geometryEpsilon = 1.0e-10;
    std::size_t maximumRepairHoleVertexCount = 8U;
    double maximumRepairHolePerimeterRatio = 0.20;
    double maximumRepairHoleAreaRatio = 0.04;
};

class BoundaryLockedPatchBuilder final
{
public:
    [[nodiscard]] const BoundaryLockedPatchBuilderSettings& settings() const noexcept;
    void setSettings(const BoundaryLockedPatchBuilderSettings& settings) noexcept;

    bool build(
        const TriangulatedPatch& completeSourcePatch,
        const QuadPatchResult& innerRemeshResult,
        const DirectionFieldData& directionField,
        const DensityFieldData& densityField,
        QuadPatchResult& result,
        std::string& diagnostic) const;

private:
    BoundaryLockedPatchBuilderSettings settings_;
    TransitionCollarBuilder collarBuilder_;
};

}  // namespace directional_retopo
