#pragma once

#include "Remesh/QuadPatchResult.h"
#include "Viewport/VisualizationSettings.h"

#include <maya/MPointArray.h>

#include <cstdint>
#include <mutex>
#include <vector>

namespace directional_retopo {

struct QuadPreviewSnapshot final
{
    bool visible = false;
    MPointArray rawWorldLinePoints;
    MPointArray conformedWorldLinePoints;
    MPointArray transitionCollarWorldLinePoints;
    MPointArray triangleWorldLinePoints;
    MPointArray sourceBoundaryWorldLinePoints;
    MPointArray resultBoundaryWorldLinePoints;
    MPointArray boundaryCorrespondenceWorldLinePoints;
    MPointArray requiredBoundaryAnchorWorldPoints;
    QuadPreviewVisualizationSettings style;
};

class QuadPreviewModel final
{
public:
    void clear() noexcept;
    void setResults(
        const std::vector<QuadPatchResult>& results,
        const QuadPreviewVisualizationSettings& settings);
    void setSettings(const QuadPreviewVisualizationSettings& settings) noexcept;
    bool snapshot(QuadPreviewSnapshot& snapshot) const;

    [[nodiscard]] std::uint64_t generation() const noexcept;

private:
    mutable std::mutex mutex_;
    MPointArray rawWorldLinePoints_;
    MPointArray conformedWorldLinePoints_;
    MPointArray transitionCollarWorldLinePoints_;
    MPointArray triangleWorldLinePoints_;
    MPointArray sourceBoundaryWorldLinePoints_;
    MPointArray resultBoundaryWorldLinePoints_;
    MPointArray boundaryCorrespondenceWorldLinePoints_;
    MPointArray requiredBoundaryAnchorWorldPoints_;
    QuadPreviewVisualizationSettings settings_;
    std::uint64_t generation_ = 0;
};

}  // namespace directional_retopo
