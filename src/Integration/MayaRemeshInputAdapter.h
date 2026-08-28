#pragma once

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"
#include "Solver/RemeshContract.h"

#include <string>

namespace directional_retopo {

class MayaRemeshInputAdapter final
{
public:
    [[nodiscard]] static bool build(
        const MeshTopologyCache& topology,
        const PaintRegionData& region,
        const DirectionFieldData& directionField,
        const DensityFieldData& densityField,
        const solver::RemeshSettings& settings,
        solver::RemeshInput& output,
        std::string& diagnostic);
};

}  // namespace directional_retopo
