#include "Viewport/TargetDisplayModel.h"

#include <maya/MFnAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MGlobal.h>
#include <maya/MMessage.h>
#include <maya/MNodeMessage.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace directional_retopo {
namespace {

void displayModelFailure(const char* operation, const MStatus& status)
{
    MString message("[DirectionalRetopo] Target display ");
    message += operation;
    message += " failed";
    const MString error = status.errorString();
    if (error.length() > 0) {
        message += ": ";
        message += error;
    }
    MGlobal::displayWarning(message);
}

void displayCallbackFallback(const char* callbackName, const MStatus& status)
{
    MString message("[DirectionalRetopo] Target display callback '");
    message += callbackName;
    message += "' is unavailable";
    const MString error = status.errorString();
    if (error.length() > 0) {
        message += ": ";
        message += error;
    }
    message += ". Explicit cache invalidation remains available.";
    MGlobal::displayWarning(message);
}

MString rootAttributeName(const MPlug& dirtyPlug)
{
    MStatus status;
    MPlug rootPlug = dirtyPlug;
    while (rootPlug.isChild(&status) && status) {
        rootPlug = rootPlug.parent(&status);
        if (!status) {
            return MString();
        }
    }
    if (rootPlug.isElement(&status) && status) {
        rootPlug = rootPlug.array(&status);
        if (!status) {
            return MString();
        }
    }

    MFnAttribute attribute(rootPlug.attribute(&status), &status);
    return status ? attribute.name() : MString();
}

enum class MeshDirtyKind
{
    None,
    VertexPositions,
    Topology,
};

MeshDirtyKind meshDirtyKind(const MPlug& plug)
{
    // Display and selection plugs can become dirty during viewport input.
    // Only plugs that represent mesh geometry may invalidate this cache.
    const MString attributeName = rootAttributeName(plug);
    if (attributeName == "pnts") {
        return MeshDirtyKind::VertexPositions;
    }
    if (attributeName == "inMesh" || attributeName == "outMesh") {
        return MeshDirtyKind::Topology;
    }
    return MeshDirtyKind::None;
}

}  // namespace

TargetDisplayModel::~TargetDisplayModel()
{
    clear();
}

void TargetDisplayModel::setDisplayShapeObject(
    const MObject& shapeObject) noexcept
{
    displayShapeHandle_ = MObjectHandle(shapeObject);
}

MStatus TargetDisplayModel::setTarget(
    const MDagPath& meshPath,
    const TargetVisualizationSettings& settings)
{
    removeCallbacks();
    meshPath_ = MDagPath();
    meshHandle_ = MObjectHandle();
    objectVertices_.clear();
    objectVertexNormals_.clear();
    edgeVertexIndices_.clear();
    objectLinePoints_.clear();
    worldLinePoints_.clear();

    MStatus status;
    MFnMesh mesh(meshPath, &status);
    if (!status) {
        displayModelFailure("mesh initialization", status);
        notifyDisplayDirty();
        return status;
    }

    settings_ = settings;
    settings_.targetWireOpacity =
        std::clamp(settings_.targetWireOpacity, 0.0F, 1.0F);
    settings_.targetWireLineWidth =
        std::max(settings_.targetWireLineWidth, 0.1F);
    settings_.targetWireNormalOffsetRatio =
        std::max(settings_.targetWireNormalOffsetRatio, 0.0);
    meshPath_ = meshPath;
    meshHandle_ = MObjectHandle(meshPath.node());
    targetBeingRemoved_ = false;
    topologyCacheDirty_ = true;
    vertexPositionCacheDirty_ = true;
    worldCacheDirty_ = true;
    topologyCallbackRegistered_ = false;
    matrixCallbackRegistered_ = false;

    status = rebuildTopologyCache();
    if (!status) {
        displayModelFailure("topology and vertex cache rebuild", status);
        clear();
        return status;
    }
    status = updateWorldSpaceCache();
    if (!status) {
        displayModelFailure("world-space cache update", status);
        clear();
        return status;
    }

    registerCallbacks();
    notifyDisplayDirty();
    return MS::kSuccess;
}

void TargetDisplayModel::clear() noexcept
{
    removeCallbacks();
    meshPath_ = MDagPath();
    meshHandle_ = MObjectHandle();
    objectVertices_.clear();
    objectVertexNormals_.clear();
    edgeVertexIndices_.clear();
    objectLinePoints_.clear();
    worldLinePoints_.clear();
    cachedVertexCount_ = -1;
    cachedEdgeCount_ = -1;
    normalOffsetDistance_ = 0.0;
    topologyCacheDirty_ = true;
    vertexPositionCacheDirty_ = true;
    worldCacheDirty_ = true;
    topologyCallbackRegistered_ = false;
    matrixCallbackRegistered_ = false;
    targetBeingRemoved_ = false;
    notifyDisplayDirty();
}

bool TargetDisplayModel::hasTarget() const noexcept
{
    return !targetBeingRemoved_ && meshHandle_.isValid() && meshHandle_.isAlive() &&
        meshPath_.isValid();
}

void TargetDisplayModel::setSettings(
    const TargetVisualizationSettings& settings) noexcept
{
    const bool offsetChanged =
        settings_.targetWireNormalOffsetRatio != settings.targetWireNormalOffsetRatio;
    settings_ = settings;
    settings_.targetWireOpacity =
        std::clamp(settings_.targetWireOpacity, 0.0F, 1.0F);
    settings_.targetWireLineWidth =
        std::max(settings_.targetWireLineWidth, 0.1F);
    settings_.targetWireNormalOffsetRatio =
        std::max(settings_.targetWireNormalOffsetRatio, 0.0);
    if (offsetChanged && hasTarget()) {
        vertexPositionCacheDirty_ = true;
        worldCacheDirty_ = true;
    }
    notifyDisplayDirty();
}

void TargetDisplayModel::invalidateTopologyCache() noexcept
{
    topologyCacheDirty_ = true;
    vertexPositionCacheDirty_ = true;
    worldCacheDirty_ = true;
    notifyDisplayDirty();
}

void TargetDisplayModel::invalidateGeometryCache() noexcept
{
    vertexPositionCacheDirty_ = true;
    worldCacheDirty_ = true;
    notifyDisplayDirty();
}

void TargetDisplayModel::invalidateWorldCache() noexcept
{
    worldCacheDirty_ = true;
    notifyDisplayDirty();
}

bool TargetDisplayModel::snapshot(TargetDisplaySnapshot& snapshot)
{
    snapshot.visible = false;
    snapshot.worldLinePoints.clear();
    snapshot.style = settings_;
    if (!settings_.showTargetWireframe || !ensureCacheCurrent() ||
        worldLinePoints_.length() == 0) {
        return true;
    }

    snapshot.visible = true;
    snapshot.worldLinePoints = worldLinePoints_;
    return true;
}

std::uint64_t TargetDisplayModel::topologyBuildCount() const noexcept
{
    return topologyBuildCount_;
}

std::uint64_t TargetDisplayModel::geometryBuildCount() const noexcept
{
    return geometryBuildCount_;
}

std::uint64_t TargetDisplayModel::displayDirtyCount() const noexcept
{
    return displayDirtyCount_;
}

void TargetDisplayModel::nodeDirtyPlugCallback(
    MObject& /*node*/,
    MPlug& plug,
    void* clientData)
{
    TargetDisplayModel* model = static_cast<TargetDisplayModel*>(clientData);
    switch (meshDirtyKind(plug)) {
    case MeshDirtyKind::VertexPositions:
        model->invalidateGeometryCache();
        break;
    case MeshDirtyKind::Topology:
        model->invalidateTopologyCache();
        break;
    case MeshDirtyKind::None:
        break;
    }
}

void TargetDisplayModel::worldMatrixChangedCallback(
    MObject& /*transformNode*/,
    MDagMessage::MatrixModifiedFlags& /*modified*/,
    void* clientData)
{
    static_cast<TargetDisplayModel*>(clientData)->invalidateWorldCache();
}

void TargetDisplayModel::nodePreRemovalCallback(
    MObject& /*node*/,
    void* clientData)
{
    TargetDisplayModel* model = static_cast<TargetDisplayModel*>(clientData);
    model->targetBeingRemoved_ = true;
    model->topologyCacheDirty_ = true;
    model->vertexPositionCacheDirty_ = true;
    model->worldCacheDirty_ = true;
    model->notifyDisplayDirty();
}

void TargetDisplayModel::registerCallbacks()
{
    MObject meshNode = meshPath_.node();
    MStatus status;
    MCallbackId callbackId = MNodeMessage::addNodeDirtyPlugCallback(
        meshNode,
        &TargetDisplayModel::nodeDirtyPlugCallback,
        this,
        &status);
    if (status && callbackId != 0) {
        callbackIds_.append(callbackId);
        topologyCallbackRegistered_ = true;
    } else {
        // A broad node-dirty fallback is intentionally not registered: Maya
        // can emit it for selection and viewport state, which would reconnect
        // Target Display invalidation to mouse Hover.
        displayCallbackFallback("node dirty plug", status);
    }

    callbackId = MDagMessage::addWorldMatrixModifiedCallback(
        meshPath_,
        &TargetDisplayModel::worldMatrixChangedCallback,
        this,
        &status);
    if (status && callbackId != 0) {
        callbackIds_.append(callbackId);
        matrixCallbackRegistered_ = true;
    } else {
        displayCallbackFallback("world matrix changed", status);
    }

    callbackId = MNodeMessage::addNodePreRemovalCallback(
        meshNode,
        &TargetDisplayModel::nodePreRemovalCallback,
        this,
        &status);
    if (status && callbackId != 0) {
        callbackIds_.append(callbackId);
    } else {
        displayCallbackFallback("node pre-removal", status);
    }
}

void TargetDisplayModel::removeCallbacks() noexcept
{
    for (unsigned int index = 0; index < callbackIds_.length(); ++index) {
        (void)MMessage::removeCallback(callbackIds_[index]);
    }
    callbackIds_.clear();
}

void TargetDisplayModel::notifyDisplayDirty() noexcept
{
    ++displayDirtyCount_;
    if (displayShapeHandle_.isValid() && displayShapeHandle_.isAlive()) {
        MHWRender::MRenderer::setGeometryDrawDirty(
            displayShapeHandle_.object(),
            true);
    }
}

MStatus TargetDisplayModel::rebuildTopologyCache()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    status = mesh.getPoints(objectVertices_, MSpace::kObject);
    if (!status) {
        return status;
    }

    const MStatus normalStatus =
        mesh.getVertexNormals(false, objectVertexNormals_, MSpace::kObject);
    if (!normalStatus || objectVertexNormals_.length() != objectVertices_.length()) {
        objectVertexNormals_.clear();
    }

    const int edgeCount = mesh.numEdges(&status);
    if (!status || edgeCount < 0) {
        return status ? MS::kFailure : status;
    }
    const int vertexCount = mesh.numVertices(&status);
    if (!status || vertexCount < 0) {
        return status ? MS::kFailure : status;
    }

    status = edgeVertexIndices_.setLength(static_cast<unsigned int>(edgeCount) * 2U);
    if (!status) {
        return status;
    }
    for (int edgeId = 0; edgeId < edgeCount; ++edgeId) {
        int vertexIds[2] = {-1, -1};
        status = mesh.getEdgeVertices(edgeId, vertexIds);
        if (!status || vertexIds[0] < 0 || vertexIds[1] < 0 ||
            static_cast<unsigned int>(vertexIds[0]) >= objectVertices_.length() ||
            static_cast<unsigned int>(vertexIds[1]) >= objectVertices_.length()) {
            edgeVertexIndices_.clear();
            return status ? MS::kFailure : status;
        }
        const unsigned int lineIndex = static_cast<unsigned int>(edgeId) * 2U;
        edgeVertexIndices_[lineIndex] = vertexIds[0];
        edgeVertexIndices_[lineIndex + 1U] = vertexIds[1];
    }

    cachedVertexCount_ = vertexCount;
    cachedEdgeCount_ = edgeCount;
    topologyCacheDirty_ = false;
    vertexPositionCacheDirty_ = false;
    ++topologyBuildCount_;
    return rebuildObjectLinePoints();
}

MStatus TargetDisplayModel::updateVertexPositionCache()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    MPointArray updatedVertices;
    status = mesh.getPoints(updatedVertices, MSpace::kObject);
    if (!status) {
        return status;
    }
    if (updatedVertices.length() != objectVertices_.length()) {
        topologyCacheDirty_ = true;
        vertexPositionCacheDirty_ = true;
        worldCacheDirty_ = true;
        return rebuildTopologyCache();
    }

    objectVertices_ = updatedVertices;
    const MStatus normalStatus =
        mesh.getVertexNormals(false, objectVertexNormals_, MSpace::kObject);
    if (!normalStatus || objectVertexNormals_.length() != objectVertices_.length()) {
        objectVertexNormals_.clear();
    }
    vertexPositionCacheDirty_ = false;
    return rebuildObjectLinePoints();
}

MStatus TargetDisplayModel::rebuildObjectLinePoints()
{
    if ((edgeVertexIndices_.length() % 2U) != 0U) {
        return MS::kFailure;
    }
    MStatus status = objectLinePoints_.setLength(edgeVertexIndices_.length());
    if (!status) {
        return status;
    }

    normalOffsetDistance_ = 0.0;
    if (settings_.targetWireNormalOffsetRatio > 0.0 && objectVertices_.length() > 0 &&
        objectVertexNormals_.length() == objectVertices_.length()) {
        MPoint minimum = objectVertices_[0];
        MPoint maximum = objectVertices_[0];
        for (unsigned int index = 1; index < objectVertices_.length(); ++index) {
            const MPoint& point = objectVertices_[index];
            minimum.x = std::min(minimum.x, point.x);
            minimum.y = std::min(minimum.y, point.y);
            minimum.z = std::min(minimum.z, point.z);
            maximum.x = std::max(maximum.x, point.x);
            maximum.y = std::max(maximum.y, point.y);
            maximum.z = std::max(maximum.z, point.z);
        }
        const double diagonalLength = (maximum - minimum).length();
        if (std::isfinite(diagonalLength)) {
            normalOffsetDistance_ =
                diagonalLength * settings_.targetWireNormalOffsetRatio;
        }
    }

    for (unsigned int index = 0; index < edgeVertexIndices_.length(); ++index) {
        const int vertexId = edgeVertexIndices_[index];
        if (vertexId < 0 ||
            static_cast<unsigned int>(vertexId) >= objectVertices_.length()) {
            objectLinePoints_.clear();
            return MS::kFailure;
        }
        MPoint linePoint = objectVertices_[static_cast<unsigned int>(vertexId)];
        if (normalOffsetDistance_ > std::numeric_limits<double>::epsilon()) {
            const MFloatVector& cachedNormal =
                objectVertexNormals_[static_cast<unsigned int>(vertexId)];
            MVector normal(
                static_cast<double>(cachedNormal.x),
                static_cast<double>(cachedNormal.y),
                static_cast<double>(cachedNormal.z));
            if (normal.length() > std::numeric_limits<double>::epsilon()) {
                normal.normalize();
                linePoint += normal * normalOffsetDistance_;
            }
        }
        objectLinePoints_[index] = linePoint;
    }

    worldCacheDirty_ = true;
    ++geometryBuildCount_;
    return MS::kSuccess;
}

MStatus TargetDisplayModel::updateWorldSpaceCache()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }
    MStatus status;
    const MMatrix inclusiveMatrix = meshPath_.inclusiveMatrix(&status);
    if (!status) {
        return status;
    }
    status = worldLinePoints_.setLength(objectLinePoints_.length());
    if (!status) {
        return status;
    }
    for (unsigned int index = 0; index < objectLinePoints_.length(); ++index) {
        worldLinePoints_[index] = objectLinePoints_[index] * inclusiveMatrix;
    }
    cachedInclusiveMatrix_ = inclusiveMatrix;
    worldCacheDirty_ = false;
    return MS::kSuccess;
}

bool TargetDisplayModel::validateTopologyFallback()
{
    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return false;
    }
    const int currentVertexCount = mesh.numVertices(&status);
    if (!status) {
        return false;
    }
    const int currentEdgeCount = mesh.numEdges(&status);
    if (!status) {
        return false;
    }
    if (currentVertexCount != cachedVertexCount_ || currentEdgeCount != cachedEdgeCount_) {
        topologyCacheDirty_ = true;
        vertexPositionCacheDirty_ = true;
        worldCacheDirty_ = true;
    }
    return true;
}

bool TargetDisplayModel::ensureCacheCurrent()
{
    if (!hasTarget()) {
        return false;
    }
    if (!topologyCallbackRegistered_ && !validateTopologyFallback()) {
        return false;
    }
    if (topologyCacheDirty_ && !rebuildTopologyCache()) {
        return false;
    }
    if (vertexPositionCacheDirty_ && !updateVertexPositionCache()) {
        return false;
    }
    if (!matrixCallbackRegistered_) {
        MStatus status;
        const MMatrix currentMatrix = meshPath_.inclusiveMatrix(&status);
        if (!status) {
            return false;
        }
        if (!cachedInclusiveMatrix_.isEquivalent(currentMatrix)) {
            worldCacheDirty_ = true;
        }
    }
    if (worldCacheDirty_) {
        return updateWorldSpaceCache() == MS::kSuccess;
    }
    return true;
}

}  // namespace directional_retopo
