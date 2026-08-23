#pragma once

#include "Field/CurvatureEstimator.h"
#include "Field/DensityFieldData.h"
#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"

namespace directional_retopo {

struct DensityFieldBuilderSettings final
{
    DensityMode mode = DensityMode::Auto;
    double manualTargetEdgeLength = 1.0;
    double edgeLengthScale = 1.0;
    int outsideReferenceFaceRings = 2;
    std::size_t minimumReferenceEdgeCount = 3;
    double outlierMadMultiplier = 3.5;
    double transitionBoundaryBlend = 1.0;
    double boundaryBlend = 0.85;
    double minimumTargetEdgeLength = 1.0e-4;
    double maximumCurvatureRefinementFactor = 5.0;
    double minimumUsableEdgeLength = 1.0e-8;
    CurvatureEstimatorSettings curvature;
};

class DensityFieldBuilder final
{
public:
    [[nodiscard]] const DensityFieldBuilderSettings& settings() const noexcept;
    void setSettings(const DensityFieldBuilderSettings& settings) noexcept;

    [[nodiscard]] bool build(
        const PaintRegionData& region,
        const MeshTopologyCache& topology,
        DensityFieldData& output,
        DensityFieldBuildMetrics* metrics = nullptr) const;

    // Geometry-only entry point used by deterministic solver tests and future
    // non-DG patch adapters. The Maya-backed cache remains the normal caller.
    [[nodiscard]] bool build(
        const PaintRegionData& region,
        const std::vector<MeshFaceTopology>& faces,
        const std::vector<MeshEdgeTopology>& edges,
        DensityFieldData& output,
        DensityFieldBuildMetrics* metrics = nullptr) const;

private:
    DensityFieldBuilderSettings settings_;
    CurvatureEstimator curvatureEstimator_;
};

}  // namespace directional_retopo
