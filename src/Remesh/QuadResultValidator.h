#pragma once

#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"

namespace directional_retopo {

struct QuadResultValidatorSettings final
{
    double areaEpsilon = 1.0e-12;
    double surfaceProximityTargetLengthMultiplier = 2.0;
    double minimumSurfaceTolerance = 1.0e-6;
};

class QuadResultValidator final
{
public:
    [[nodiscard]] const QuadResultValidatorSettings& settings() const noexcept;
    void setSettings(const QuadResultValidatorSettings& settings) noexcept;

    bool validate(
        const TriangulatedPatch& patch,
        QuadPatchResult& result) const;

private:
    QuadResultValidatorSettings settings_;
};

}  // namespace directional_retopo

