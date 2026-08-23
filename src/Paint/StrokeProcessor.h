#pragma once

#include "Paint/StrokeData.h"

#include <cstddef>

namespace directional_retopo {

struct StrokeProcessingSettings final
{
    double endTrimSpacingMultiplier = 2.0;
    double endTrimRadiusMultiplier = 0.15;
    double maximumEndTrimLengthRatio = 0.2;
    std::size_t minimumSamplesAfterTrim = 3;
    std::size_t directionWindowRadius = 2;
    unsigned int directionSmoothingPasses = 2;
    double directionSmoothingBlend = 0.35;
};

class StrokeProcessor final
{
public:
    [[nodiscard]] const StrokeProcessingSettings& settings() const noexcept;
    void setSettings(const StrokeProcessingSettings& settings) noexcept;

    [[nodiscard]] StrokeData process(
        const StrokeData& rawStroke,
        double sampleSpacing,
        double brushRadius,
        bool trimEndpoint) const;

private:
    StrokeProcessingSettings settings_;
};

}  // namespace directional_retopo
