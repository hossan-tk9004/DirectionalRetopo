#pragma once

#include <algorithm>

namespace directional_retopo {

class BrushSettings final
{
public:
    static constexpr double kDefaultRadius = 1.0;
    static constexpr double kDefaultSampleSpacingRatio = 0.15;
    static constexpr double kMinimumRadius = 1.0e-5;

    [[nodiscard]] double radius() const noexcept
    {
        return radius_;
    }

    void setRadius(double radius) noexcept
    {
        radius_ = std::max(radius, kMinimumRadius);
    }

    [[nodiscard]] double sampleSpacingRatio() const noexcept
    {
        return sampleSpacingRatio_;
    }

    void setSampleSpacingRatio(double ratio) noexcept
    {
        sampleSpacingRatio_ = std::max(ratio, 0.0);
    }

    [[nodiscard]] double sampleSpacing() const noexcept
    {
        return std::max(radius_ * sampleSpacingRatio_, kMinimumRadius);
    }

private:
    double radius_ = kDefaultRadius;
    double sampleSpacingRatio_ = kDefaultSampleSpacingRatio;
};

}  // namespace directional_retopo
