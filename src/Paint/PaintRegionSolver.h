#pragma once

#include "Mesh/BoundaryExtractor.h"
#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"
#include "Paint/StrokeData.h"

#include <cstddef>

namespace directional_retopo {

enum class PaintFalloff
{
    Linear,
};

struct PaintRegionSolverSettings final
{
    static constexpr int kMinimumTransitionRings = 0;
    static constexpr int kMaximumTransitionRings = 10;
    static constexpr int kDefaultTransitionRings = 2;

    int transitionRings = kDefaultTransitionRings;
    float coreInfluenceThreshold = 0.0F;
    PaintFalloff falloff = PaintFalloff::Linear;
    double minimumUsableRadius = 1.0e-8;
};

struct PaintRegionSolveMetrics final
{
    std::size_t validSampleCount = 0;
    std::size_t visitedVertexCount = 0;
    std::size_t expandedEdgeCount = 0;
};

class PaintRegionSolver final
{
public:
    [[nodiscard]] const PaintRegionSolverSettings& settings() const noexcept;
    void setSettings(const PaintRegionSolverSettings& settings) noexcept;

    [[nodiscard]] bool solve(
        const StrokeData& processedStroke,
        const MeshTopologyCache& topology,
        PaintRegionData& output,
        PaintRegionSolveMetrics* metrics = nullptr) const;

private:
    PaintRegionSolverSettings settings_;
    BoundaryExtractor boundaryExtractor_;
};

}  // namespace directional_retopo
