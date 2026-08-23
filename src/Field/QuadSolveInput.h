#pragma once

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Paint/PaintRegionData.h"

namespace directional_retopo {

struct QuadSolveInput final
{
    const PaintRegionData* region = nullptr;
    const DirectionFieldData* directionField = nullptr;
    const DensityFieldData* densityField = nullptr;

    [[nodiscard]] bool valid() const noexcept
    {
        return region != nullptr && directionField != nullptr && densityField != nullptr &&
            !region->components.empty() && !directionField->perFace.empty() &&
            !densityField->perFace.empty();
    }
};

}  // namespace directional_retopo
