#pragma once

#include "Field/DensityFieldData.h"
#include "Mesh/MeshTopologyCache.h"
#include "Remesh/LocalPatch.h"

#include <cstddef>
#include <string>

namespace directional_retopo {

struct BoundaryCompatibilityDensityResult final
{
    bool success = false;
    DensityFieldData densityField;
    double requestedCoreTargetEdgeLength = 0.0;
    double effectiveInterfaceTargetEdgeLength = 0.0;
    double sourceBoundaryMedianEdgeLength = 0.0;
    std::size_t interfaceBandFaceCount = 0U;
    unsigned int interfaceBandRings = 1U;
    std::string diagnosticMessage;
};

class BoundaryCompatibilityDensity final
{
public:
    [[nodiscard]] static double computeCompatibilityTarget(
        double requestedCoreTargetEdgeLength,
        double sourceBoundaryMedianEdgeLength,
        double innerBoundaryArcLength,
        std::size_t sourceBoundaryVertexCount,
        unsigned int topologyBlendWidth) noexcept;

    [[nodiscard]] static BoundaryCompatibilityDensityResult build(
        const TriangulatedPatch& completeSourcePatch,
        const TriangulatedPatch& innerPatch,
        const MeshTopologyCache& topology,
        const DensityFieldData& requestedDensity,
        unsigned int topologyBlendWidth);
};

}  // namespace directional_retopo
