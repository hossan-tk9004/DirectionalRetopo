#pragma once

#include <cstddef>
#include <vector>

namespace directional_retopo {

enum class DensityMode
{
    Manual,
    Auto,
};

enum class DensityFallback
{
    None,
    BoundaryEdges,
    LocalRegionEdges,
    ManualDefault,
};

struct FaceDensity final
{
    // Target edge length is the source of truth. scaleU/scaleV are normalized
    // against the surrounding reference length for future anisotropic solvers.
    double targetEdgeLength = 0.0;
    double baseTargetEdgeLength = 0.0;
    double referenceEdgeLength = 0.0;
    double curvatureTargetEdgeLength = 0.0;
    double curvatureIndicator = 0.0;
    double localSurfaceEdgeLength = 0.0;
    double curvatureRefinementFactor = 1.0;
    double scaleU = 1.0;
    double scaleV = 1.0;
    bool curvatureLimited = false;
    bool autoDerived = false;
    bool valid = false;
};

struct DensityFieldData final
{
    // Indexed by original polygon face ID; child triangles may inherit it.
    std::vector<FaceDensity> perFace;
    DensityMode mode = DensityMode::Auto;

    void clear() noexcept
    {
        perFace.clear();
        mode = DensityMode::Auto;
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return perFace.empty();
    }

    [[nodiscard]] const FaceDensity* face(int faceId) const noexcept
    {
        return faceId >= 0 && static_cast<std::size_t>(faceId) < perFace.size()
            ? &perFace[static_cast<std::size_t>(faceId)]
            : nullptr;
    }
};

struct DensityFieldBuildMetrics final
{
    DensityMode mode = DensityMode::Auto;
    DensityFallback fallback = DensityFallback::None;
    std::size_t referenceEdgeCount = 0;
    double medianReferenceEdgeLength = 0.0;
    double edgeLengthScale = 1.0;
    double minimumTargetEdgeLength = 0.0;
    double meanTargetEdgeLength = 0.0;
    double maximumTargetEdgeLength = 0.0;
    std::size_t curvatureConstrainedFaceCount = 0U;
    double minimumCurvatureTargetEdgeLength = 0.0;
    double maximumCurvatureIndicator = 0.0;
    double meanCurvatureIndicator = 0.0;
    double meanCurvatureRegionSourceEdgeLength = 0.0;
    double maximumAppliedCurvatureRefinementFactor = 1.0;
};

}  // namespace directional_retopo
