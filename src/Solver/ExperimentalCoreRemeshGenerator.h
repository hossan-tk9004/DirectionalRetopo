#pragma once

#include "Solver/CoreRemeshResult.h"

namespace directional_retopo::solver {

struct ExperimentalCoreRemeshSettings final
{
    unsigned int insetSourceFaceRings = 1U;
    std::size_t minimumInsetFaceCount = 4U;
    bool projectBoundaryToSourcePolyline = false;
    double geometryEpsilon = 1.0e-12;
};

// Maya/AutoRemesher-backed experimental adapter.  Its public contract remains
// portable and the target is deliberately not linked into the plug-in.
class ExperimentalCoreRemeshGenerator final
{
public:
    [[nodiscard]] CoreRemeshResult generate(
        const RemeshInput& input,
        const RegionComponent& component,
        const ExperimentalCoreRemeshSettings& settings = {}) const noexcept;
};

}  // namespace directional_retopo::solver
