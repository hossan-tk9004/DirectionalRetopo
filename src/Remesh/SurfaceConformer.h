#pragma once

#include "Remesh/BoundaryConformer.h"
#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"

#include <cstddef>
#include <string>

namespace directional_retopo {

struct SurfaceConformerSettings final
{
    unsigned int tangentialRelaxIterations = 3U;
    double tangentialRelaxStrength = 0.30;
    unsigned int localProjectionTriangleRings = 2U;
    double meanDistanceWarningTargetLengthRatio = 0.05;
    double maximumDistanceWarningTargetLengthRatio = 0.20;
    double maximumRelaxAreaLossRatio = 0.05;
    double geometryEpsilon = 1.0e-12;
    bool lockResultBoundaryDuringRelax = true;
    bool projectResultBoundaryToSourceBoundary = true;
};

class SurfaceConformer final
{
public:
    SurfaceConformer();
    [[nodiscard]] const SurfaceConformerSettings& settings() const noexcept;
    void setSettings(const SurfaceConformerSettings& settings) noexcept;

    bool conform(
        const TriangulatedPatch& patch,
        QuadPatchResult& result,
        std::string& diagnostic) const;

private:
    SurfaceConformerSettings settings_;
    BoundaryConformer boundaryConformer_;
};

}  // namespace directional_retopo
