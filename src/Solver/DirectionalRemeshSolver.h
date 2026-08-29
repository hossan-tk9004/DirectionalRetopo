#pragma once

#include "Solver/RemeshContract.h"

namespace directional_retopo {

// The only orchestration entry point used by Maya integration and the
// deterministic harness. R2 intentionally preserves the current legacy
// algorithm behind this portable contract.
class DirectionalRemeshSolver final
{
public:
    [[nodiscard]] solver::RemeshResult solve(
        const solver::RemeshInput& input) const noexcept;
};

}  // namespace directional_retopo
