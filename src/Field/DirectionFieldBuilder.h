#pragma once

#include "Field/CrossFieldMath.h"
#include "Field/DirectionFieldData.h"
#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"
#include "Paint/StrokeData.h"

namespace directional_retopo {

struct DirectionFieldBuilderSettings final
{
    unsigned int directionSmoothingIterations = 30;
    double directionSmoothStrength = 0.85;
    double directHitConstraintWeight = 1.0;
    double minimumCoreInfluenceForConstraint = 0.15;
    double coreTopologyGuidanceWeight = 0.02;
    double transitionTopologyGuidanceWeight = 0.12;
    double boundaryTopologyGuidanceWeight = 0.35;
    double singularityEpsilon = 1.0e-7;
    double geometryEpsilon = 1.0e-10;
    double validationTolerance = 1.0e-5;
};

class DirectionFieldBuilder final
{
public:
    [[nodiscard]] const DirectionFieldBuilderSettings& settings() const noexcept;
    void setSettings(const DirectionFieldBuilderSettings& settings) noexcept;

    [[nodiscard]] bool build(
        const StrokeData& processedStroke,
        const PaintRegionData& region,
        const MeshTopologyCache& topology,
        DirectionFieldData& output,
        DirectionFieldBuildMetrics* metrics = nullptr) const;

private:
    DirectionFieldBuilderSettings settings_;
};

}  // namespace directional_retopo
