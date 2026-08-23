#pragma once

#include "Paint/StrokeData.h"

#include <maya/MDagPath.h>
#include <maya/MObjectHandle.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace directional_retopo {

struct RegionPreviewCalculationSettings final
{
    double brushRadiusPaddingRatio = 0.05;
};

class RegionPreviewCalculator final
{
public:
    MStatus setTarget(const MDagPath& meshPath);
    void clear() noexcept;

    [[nodiscard]] bool hasTarget() const noexcept;
    [[nodiscard]] const RegionPreviewCalculationSettings& settings() const noexcept;
    void setSettings(const RegionPreviewCalculationSettings& settings) noexcept;

    // Adds a provisional, locally connected set of faces around one stroke
    // sample. Returns true only when the output face-id set changed.
    bool addFacesForSample(
        const StrokeSample& sample,
        std::unordered_set<int>& faceIds);

private:
    struct FaceBounds final
    {
        MPoint centerObject;
        double radiusObject = 0.0;
        std::vector<int> connectedFaces;
    };

    RegionPreviewCalculationSettings settings_;
    MDagPath meshPath_;
    MObjectHandle meshHandle_;
    std::vector<FaceBounds> faces_;
    std::vector<std::uint32_t> visitGenerations_;
    std::uint32_t currentVisitGeneration_ = 0;
};

}  // namespace directional_retopo
