#pragma once

#include "Mesh/MeshTopologyCache.h"

#include <cstddef>
#include <vector>

namespace directional_retopo {

struct CurvatureEstimatorSettings final
{
    // A target edge should normally turn the sampled surface normal by no more
    // than this amount. Four degrees deliberately favors silhouette fidelity;
    // maximumCurvatureRefinementFactor caps the resulting polygon count.
    double desiredNormalVariationDegrees = 4.0;
    double minimumSignificantDihedralDegrees = 0.5;
    double rmsWeight = 0.70;
    double peakWeight = 0.30;
    int neighborSpreadIterations = 1;
    double neighborSpreadStrength = 0.35;
    double geometryEpsilon = 1.0e-8;
};

struct FaceCurvature final
{
    // Radians of normal change per world-space length. This is a robust
    // curvature proxy rather than an exact principal-curvature estimate.
    double indicator = 0.0;
    double meanDihedralRadians = 0.0;
    double maximumDihedralRadians = 0.0;
    double localEdgeLength = 0.0;
    bool valid = false;
};

struct CurvatureEstimateMetrics final
{
    std::size_t validFaceCount = 0U;
    std::size_t significantFaceCount = 0U;
    double minimumIndicator = 0.0;
    double maximumIndicator = 0.0;
    double meanIndicator = 0.0;
};

class CurvatureEstimator final
{
public:
    [[nodiscard]] const CurvatureEstimatorSettings& settings() const noexcept;
    void setSettings(const CurvatureEstimatorSettings& settings) noexcept;

    bool estimate(
        const std::vector<MeshFaceTopology>& faces,
        const std::vector<MeshEdgeTopology>& edges,
        std::vector<FaceCurvature>& output,
        CurvatureEstimateMetrics* metrics = nullptr) const;

    bool estimate(
        const MeshTopologyCache& topology,
        std::vector<FaceCurvature>& output,
        CurvatureEstimateMetrics* metrics = nullptr) const;

private:
    CurvatureEstimatorSettings settings_;
};

}  // namespace directional_retopo
