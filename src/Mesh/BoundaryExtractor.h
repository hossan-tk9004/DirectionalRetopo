#pragma once

#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"

#include <vector>

namespace directional_retopo {

class BoundaryExtractor final
{
public:
    // Populates boundary fields without modifying the component's face sets.
    // Ambiguous/non-manifold branches are emitted as finite chains and flagged.
    void extract(
        const MeshTopologyCache& topology,
        PaintRegionComponent& component) const;
};

}  // namespace directional_retopo
