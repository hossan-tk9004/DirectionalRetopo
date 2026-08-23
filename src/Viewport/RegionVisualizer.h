#pragma once

#include "Paint/PaintRegionData.h"
#include "Viewport/VisualizationSettings.h"

#include <maya/MDagPath.h>
#include <maya/MMatrix.h>
#include <maya/MObjectHandle.h>
#include <maya/MPointArray.h>
#include <maya/MStatus.h>
#include <maya/MUIDrawManager.h>

#include <unordered_set>
#include <vector>

namespace directional_retopo {

class RegionVisualizer final
{
public:
    MStatus setTarget(const MDagPath& meshPath);
    void clear() noexcept;
    void clearFaceIds() noexcept;

    [[nodiscard]] bool hasTarget() const noexcept;
    [[nodiscard]] const RegionVisualizationSettings& settings() const noexcept;
    void setSettings(const RegionVisualizationSettings& settings) noexcept;

    // Provisional faces are used only while dragging.
    void setFaceIds(const std::unordered_set<int>& faceIds);
    // Final Core/Transition/Boundary replaces provisional geometry on release.
    void setFinalRegion(const PaintRegionData& region);
    void invalidateGeometry() noexcept;
    MStatus draw(MHWRender::MUIDrawManager& drawManager);

private:
    MStatus rebuildObjectSpaceGeometry();
    MStatus updateWorldSpaceGeometry();
    MStatus appendFaceTriangles(
        const std::vector<int>& faceIds,
        double normalOffset,
        MPointArray& trianglePoints);

    RegionVisualizationSettings settings_;
    MDagPath meshPath_;
    MObjectHandle meshHandle_;
    std::vector<int> provisionalFaceIds_;
    std::vector<int> coreFaceIds_;
    std::vector<int> transitionFaceIds_;
    std::vector<int> boundaryEdgeIds_;
    MPointArray objectProvisionalTrianglePoints_;
    MPointArray objectCoreTrianglePoints_;
    MPointArray objectTransitionTrianglePoints_;
    MPointArray objectBoundaryLinePoints_;
    MPointArray worldProvisionalTrianglePoints_;
    MPointArray worldCoreTrianglePoints_;
    MPointArray worldTransitionTrianglePoints_;
    MPointArray worldBoundaryLinePoints_;
    MMatrix cachedInclusiveMatrix_;
    bool finalRegionActive_ = false;
    bool geometryDirty_ = true;
    bool worldGeometryDirty_ = true;
};

}  // namespace directional_retopo
