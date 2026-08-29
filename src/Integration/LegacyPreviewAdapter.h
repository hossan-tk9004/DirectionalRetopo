#pragma once

#include "Remesh/QuadPatchResult.h"
#include "Solver/RemeshContract.h"

namespace directional_retopo {

class LegacyPreviewAdapter final
{
public:
    [[nodiscard]] static QuadPatchResult convert(
        const solver::ComponentResult& source,
        const solver::RemeshInput& input);
};

}  // namespace directional_retopo
