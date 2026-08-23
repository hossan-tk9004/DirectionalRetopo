#pragma once

#include "Viewport/VisualizationSettings.h"

#include <maya/MCallbackIdArray.h>
#include <maya/MDagMessage.h>
#include <maya/MDagPath.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MIntArray.h>
#include <maya/MMatrix.h>
#include <maya/MObjectHandle.h>
#include <maya/MPointArray.h>
#include <maya/MPlug.h>
#include <maya/MStatus.h>

#include <cstdint>

namespace directional_retopo {

struct TargetDisplaySnapshot final
{
    bool visible = false;
    MPointArray worldLinePoints;
    TargetVisualizationSettings style;
};

class TargetDisplayModel final
{
public:
    TargetDisplayModel() = default;
    ~TargetDisplayModel();

    TargetDisplayModel(const TargetDisplayModel&) = delete;
    TargetDisplayModel& operator=(const TargetDisplayModel&) = delete;

    void setDisplayShapeObject(const MObject& shapeObject) noexcept;
    MStatus setTarget(
        const MDagPath& meshPath,
        const TargetVisualizationSettings& settings);
    void clear() noexcept;

    [[nodiscard]] bool hasTarget() const noexcept;
    void setSettings(const TargetVisualizationSettings& settings) noexcept;

    // Explicit invalidation entry points for a target change, a future
    // remesher, or a visualization-settings editor.
    void invalidateTopologyCache() noexcept;
    void invalidateGeometryCache() noexcept;
    void invalidateWorldCache() noexcept;

    bool snapshot(TargetDisplaySnapshot& snapshot);

    [[nodiscard]] std::uint64_t topologyBuildCount() const noexcept;
    [[nodiscard]] std::uint64_t geometryBuildCount() const noexcept;
    [[nodiscard]] std::uint64_t displayDirtyCount() const noexcept;

private:
    static void nodeDirtyPlugCallback(MObject& node, MPlug& plug, void* clientData);
    static void worldMatrixChangedCallback(
        MObject& transformNode,
        MDagMessage::MatrixModifiedFlags& modified,
        void* clientData);
    static void nodePreRemovalCallback(MObject& node, void* clientData);

    void registerCallbacks();
    void removeCallbacks() noexcept;
    void notifyDisplayDirty() noexcept;
    MStatus rebuildTopologyCache();
    MStatus updateVertexPositionCache();
    MStatus rebuildObjectLinePoints();
    MStatus updateWorldSpaceCache();
    bool validateTopologyFallback();
    bool ensureCacheCurrent();

    TargetVisualizationSettings settings_;
    MDagPath meshPath_;
    MObjectHandle meshHandle_;
    MObjectHandle displayShapeHandle_;
    MCallbackIdArray callbackIds_;
    MPointArray objectVertices_;
    MFloatVectorArray objectVertexNormals_;
    MIntArray edgeVertexIndices_;
    MPointArray objectLinePoints_;
    MPointArray worldLinePoints_;
    MMatrix cachedInclusiveMatrix_;
    int cachedVertexCount_ = -1;
    int cachedEdgeCount_ = -1;
    double normalOffsetDistance_ = 0.0;
    bool topologyCacheDirty_ = true;
    bool vertexPositionCacheDirty_ = true;
    bool worldCacheDirty_ = true;
    bool topologyCallbackRegistered_ = false;
    bool matrixCallbackRegistered_ = false;
    bool targetBeingRemoved_ = false;
    std::uint64_t topologyBuildCount_ = 0;
    std::uint64_t geometryBuildCount_ = 0;
    std::uint64_t displayDirtyCount_ = 0;
};

}  // namespace directional_retopo
