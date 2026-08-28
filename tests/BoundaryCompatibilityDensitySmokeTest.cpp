#include "Remesh/BoundaryCompatibilityDensity.h"

#include <cmath>
#include <iostream>

namespace {

bool nearlyEqual(double first, double second)
{
    return std::abs(first - second) <= 1.0e-12;
}

}  // namespace

int main()
{
    using directional_retopo::BoundaryCompatibilityDensity;

    const double scales[] = {0.1, 0.5, 1.0, 2.0};
    const double expected[] = {0.1, 0.5, 1.0, 1.0};
    for (std::size_t index = 0U; index < 4U; ++index) {
        const double target =
            BoundaryCompatibilityDensity::computeCompatibilityTarget(
                scales[index],
                1.0,
                16.0,
                16U,
                1U);
        if (!nearlyEqual(target, expected[index])) {
            std::cerr << "Scale matrix failed at " << scales[index]
                      << ": " << target << std::endl;
            return 1;
        }
    }

    if (!nearlyEqual(
            BoundaryCompatibilityDensity::computeCompatibilityTarget(
                2.0, 1.0, 16.0, 16U, 0U),
            1.0) ||
        !nearlyEqual(
            BoundaryCompatibilityDensity::computeCompatibilityTarget(
                2.0, 1.0, 16.0, 16U, 2U),
            2.0) ||
        !nearlyEqual(
            BoundaryCompatibilityDensity::computeCompatibilityTarget(
                2.0, 1.0, 16.0, 16U, 5U),
            2.0)) {
        std::cerr << "Blend 0/1 clamp or Blend 2/5 policy failed."
                  << std::endl;
        return 1;
    }

    struct RegionCase final {
        double arcLength;
        std::size_t boundaryVertices;
    };
    const RegionCase regions[] = {
        {3.0, 4U},
        {14.0, 16U},
        {60.0, 64U},
    };
    for (const RegionCase& region : regions) {
        const double target =
            BoundaryCompatibilityDensity::computeCompatibilityTarget(
                2.0,
                1.0,
                region.arcLength,
                region.boundaryVertices,
                1U);
        if (!(target > 0.0 && target <= 1.0)) {
            std::cerr << "Small/Medium/Large Region policy failed: "
                      << target << std::endl;
            return 1;
        }
    }

    std::cout
        << "BoundaryCompatibilityDensitySmokeTest passed: Scale "
        << "0.1/0.5/1/2, Blend 1/2/5, Small/Medium/Large."
        << std::endl;
    return 0;
}
