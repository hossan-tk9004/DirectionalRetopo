#include "Viewport/RegionVisualizer.h"

#include <maya/MBoundingBox.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MFnMesh.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MPointArray.h>
#include <maya/MVector.h>

#include <algorithm>
#include <cmath>

namespace directional_retopo {
namespace {

void transformPoints(
    const MPointArray& source,
    const MMatrix& matrix,
    MPointArray& destination)
{
    (void)destination.setLength(source.length());
    for (unsigned int index = 0; index < source.length(); ++index) {
        destination[index] = source[index] * matrix;
    }
}

void setFillStyle(
    MHWRender::MUIDrawManager& drawManager,
    const MColor& color,
    float opacity,
    unsigned int depthPriority)
{
    drawManager.setColor(MColor(color.r, color.g, color.b, opacity));
    drawManager.setDepthPriority(depthPriority);
    drawManager.setPaintStyle(MHWRender::MUIDrawManager::kFlat);
}

}  // namespace

MStatus RegionVisualizer::setTarget(const MDagPath& meshPath)
{
    clear();

    MStatus status;
    MFnMesh mesh(meshPath, &status);
    if (!status) {
        return status;
    }

    meshPath_ = meshPath;
    meshHandle_ = MObjectHandle(meshPath.node());
    geometryDirty_ = true;
    worldGeometryDirty_ = true;
    return MS::kSuccess;
}

void RegionVisualizer::clear() noexcept
{
    meshPath_ = MDagPath();
    meshHandle_ = MObjectHandle();
    clearFaceIds();
}

void RegionVisualizer::clearFaceIds() noexcept
{
    provisionalFaceIds_.clear();
    coreFaceIds_.clear();
    transitionFaceIds_.clear();
    boundaryEdgeIds_.clear();
    objectProvisionalTrianglePoints_.clear();
    objectCoreTrianglePoints_.clear();
    objectTransitionTrianglePoints_.clear();
    objectBoundaryLinePoints_.clear();
    worldProvisionalTrianglePoints_.clear();
    worldCoreTrianglePoints_.clear();
    worldTransitionTrianglePoints_.clear();
    worldBoundaryLinePoints_.clear();
    finalRegionActive_ = false;
    geometryDirty_ = true;
    worldGeometryDirty_ = true;
}

bool RegionVisualizer::hasTarget() const noexcept
{
    return meshHandle_.isValid() && meshHandle_.isAlive() && meshPath_.isValid();
}

const RegionVisualizationSettings& RegionVisualizer::settings() const noexcept
{
    return settings_;
}

void RegionVisualizer::setSettings(
    const RegionVisualizationSettings& settings) noexcept
{
    const bool offsetChanged =
        settings_.regionNormalOffsetRatio != settings.regionNormalOffsetRatio ||
        settings_.boundaryNormalOffsetRatio != settings.boundaryNormalOffsetRatio;
    settings_ = settings;
    settings_.provisionalOpacity =
        std::clamp(settings_.provisionalOpacity, 0.0F, 1.0F);
    settings_.coreOpacity = std::clamp(settings_.coreOpacity, 0.0F, 1.0F);
    settings_.transitionOpacity =
        std::clamp(settings_.transitionOpacity, 0.0F, 1.0F);
    settings_.boundaryOpacity =
        std::clamp(settings_.boundaryOpacity, 0.0F, 1.0F);
    settings_.boundaryLineWidth = std::max(settings_.boundaryLineWidth, 0.1F);
    settings_.regionNormalOffsetRatio =
        std::max(settings_.regionNormalOffsetRatio, 0.0);
    settings_.boundaryNormalOffsetRatio =
        std::max(settings_.boundaryNormalOffsetRatio, 0.0);
    if (offsetChanged) {
        invalidateGeometry();
    }
}

void RegionVisualizer::setFaceIds(const std::unordered_set<int>& faceIds)
{
    std::vector<int> sortedFaceIds(faceIds.begin(), faceIds.end());
    std::sort(sortedFaceIds.begin(), sortedFaceIds.end());
    if (!finalRegionActive_ && sortedFaceIds == provisionalFaceIds_) {
        return;
    }

    provisionalFaceIds_ = std::move(sortedFaceIds);
    coreFaceIds_.clear();
    transitionFaceIds_.clear();
    boundaryEdgeIds_.clear();
    finalRegionActive_ = false;
    invalidateGeometry();
}

void RegionVisualizer::setFinalRegion(const PaintRegionData& region)
{
    provisionalFaceIds_.clear();
    coreFaceIds_.clear();
    transitionFaceIds_.clear();
    boundaryEdgeIds_.clear();

    for (const PaintRegionComponent& component : region.components) {
        coreFaceIds_.insert(
            coreFaceIds_.end(),
            component.coreFaceIds.begin(),
            component.coreFaceIds.end());
        transitionFaceIds_.insert(
            transitionFaceIds_.end(),
            component.transitionFaceIds.begin(),
            component.transitionFaceIds.end());
        for (const BoundaryEdge& edge : component.boundaryEdges) {
            boundaryEdgeIds_.push_back(edge.edgeId);
        }
    }
    const auto sortAndUnique = [](std::vector<int>& values) {
        std::sort(values.begin(), values.end());
        values.erase(std::unique(values.begin(), values.end()), values.end());
    };
    sortAndUnique(coreFaceIds_);
    sortAndUnique(transitionFaceIds_);
    sortAndUnique(boundaryEdgeIds_);
    finalRegionActive_ = true;
    invalidateGeometry();
}

void RegionVisualizer::invalidateGeometry() noexcept
{
    geometryDirty_ = true;
    worldGeometryDirty_ = true;
}

MStatus RegionVisualizer::draw(MHWRender::MUIDrawManager& drawManager)
{
    const bool hasProvisional = !finalRegionActive_ &&
        settings_.showRegionPreview && !provisionalFaceIds_.empty();
    const bool hasFinal = finalRegionActive_ && settings_.showFinalRegion &&
        (!coreFaceIds_.empty() || !transitionFaceIds_.empty() ||
         !boundaryEdgeIds_.empty());
    if (!hasTarget() || (!hasProvisional && !hasFinal)) {
        return MS::kSuccess;
    }

    MStatus status;
    if (geometryDirty_) {
        status = rebuildObjectSpaceGeometry();
        if (!status) {
            return status;
        }
    }

    const MMatrix currentMatrix = meshPath_.inclusiveMatrix(&status);
    if (!status) {
        return status;
    }
    if (!cachedInclusiveMatrix_.isEquivalent(currentMatrix)) {
        worldGeometryDirty_ = true;
    }
    if (worldGeometryDirty_) {
        status = updateWorldSpaceGeometry();
        if (!status) {
            return status;
        }
    }

    if (hasProvisional && worldProvisionalTrianglePoints_.length() > 0) {
        setFillStyle(
            drawManager,
            settings_.provisionalColor,
            settings_.provisionalOpacity,
            settings_.fillDepthPriority);
        drawManager.mesh(
            MHWRender::MUIDrawManager::kTriangles,
            worldProvisionalTrianglePoints_);
        return MS::kSuccess;
    }

    if (worldTransitionTrianglePoints_.length() > 0) {
        setFillStyle(
            drawManager,
            settings_.transitionColor,
            settings_.transitionOpacity,
            settings_.fillDepthPriority);
        drawManager.mesh(
            MHWRender::MUIDrawManager::kTriangles,
            worldTransitionTrianglePoints_);
    }
    if (worldCoreTrianglePoints_.length() > 0) {
        setFillStyle(
            drawManager,
            settings_.coreColor,
            settings_.coreOpacity,
            settings_.fillDepthPriority);
        drawManager.mesh(
            MHWRender::MUIDrawManager::kTriangles,
            worldCoreTrianglePoints_);
    }
    if (worldBoundaryLinePoints_.length() >= 2) {
        drawManager.setColor(MColor(
            settings_.boundaryColor.r,
            settings_.boundaryColor.g,
            settings_.boundaryColor.b,
            settings_.boundaryOpacity));
        drawManager.setDepthPriority(settings_.boundaryDepthPriority);
        drawManager.setLineWidth(settings_.boundaryLineWidth);
        (void)drawManager.lineList(worldBoundaryLinePoints_, false);
    }
    return MS::kSuccess;
}

MStatus RegionVisualizer::appendFaceTriangles(
    const std::vector<int>& faceIds,
    double normalOffset,
    MPointArray& trianglePoints)
{
    trianglePoints.clear();
    if (faceIds.empty()) {
        return MS::kSuccess;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    const int faceCount = mesh.numPolygons(&status);
    if (!status || faceCount < 0) {
        return status ? MS::kFailure : status;
    }
    MObject component = MObject::kNullObj;
    MItMeshPolygon iterator(meshPath_, component, &status);
    if (!status) {
        return status;
    }

    for (const int faceId : faceIds) {
        if (faceId < 0 || faceId >= faceCount) {
            continue;
        }
        int previousFaceId = -1;
        status = iterator.setIndex(faceId, previousFaceId);
        if (!status) {
            trianglePoints.clear();
            return status;
        }

        MPointArray faceTriangles;
        MIntArray triangleVertexIds;
        status = iterator.getTriangles(
            faceTriangles,
            triangleVertexIds,
            MSpace::kObject);
        if (!status || (faceTriangles.length() % 3U) != 0U) {
            trianglePoints.clear();
            return status ? MS::kFailure : status;
        }
        MVector faceNormal;
        status = iterator.getNormal(faceNormal, MSpace::kObject);
        if (!status) {
            trianglePoints.clear();
            return status;
        }
        if (faceNormal.length() > 1.0e-10) {
            faceNormal.normalize();
        } else {
            faceNormal = MVector::zero;
        }
        for (unsigned int pointIndex = 0; pointIndex < faceTriangles.length(); ++pointIndex) {
            trianglePoints.append(faceTriangles[pointIndex] + faceNormal * normalOffset);
        }
    }
    return MS::kSuccess;
}

MStatus RegionVisualizer::rebuildObjectSpaceGeometry()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    const MBoundingBox bounds = mesh.boundingBox(&status);
    if (!status) {
        return status;
    }
    const double meshDiagonal = (bounds.max() - bounds.min()).length();
    const double regionOffset = std::isfinite(meshDiagonal)
        ? meshDiagonal * settings_.regionNormalOffsetRatio
        : 0.0;
    const double boundaryOffset = std::isfinite(meshDiagonal)
        ? meshDiagonal * settings_.boundaryNormalOffsetRatio
        : 0.0;

    status = appendFaceTriangles(
        provisionalFaceIds_,
        regionOffset,
        objectProvisionalTrianglePoints_);
    if (!status) {
        return status;
    }
    status = appendFaceTriangles(coreFaceIds_, regionOffset, objectCoreTrianglePoints_);
    if (!status) {
        return status;
    }
    status = appendFaceTriangles(
        transitionFaceIds_,
        regionOffset,
        objectTransitionTrianglePoints_);
    if (!status) {
        return status;
    }

    objectBoundaryLinePoints_.clear();
    if (!boundaryEdgeIds_.empty()) {
        MPointArray objectVertices;
        status = mesh.getPoints(objectVertices, MSpace::kObject);
        if (!status) {
            return status;
        }
        MFloatVectorArray objectNormals;
        const MStatus normalStatus =
            mesh.getVertexNormals(false, objectNormals, MSpace::kObject);
        const bool haveNormals = normalStatus &&
            objectNormals.length() == objectVertices.length();
        const int edgeCount = mesh.numEdges(&status);
        if (!status || edgeCount < 0) {
            return status ? MS::kFailure : status;
        }

        for (const int edgeId : boundaryEdgeIds_) {
            if (edgeId < 0 || edgeId >= edgeCount) {
                continue;
            }
            int vertexIds[2] = {-1, -1};
            status = mesh.getEdgeVertices(edgeId, vertexIds);
            if (!status) {
                return status;
            }
            for (const int vertexId : vertexIds) {
                if (vertexId < 0 ||
                    static_cast<unsigned int>(vertexId) >= objectVertices.length()) {
                    return MS::kFailure;
                }
                MPoint point = objectVertices[static_cast<unsigned int>(vertexId)];
                if (haveNormals && boundaryOffset > 0.0) {
                    const MFloatVector& floatNormal =
                        objectNormals[static_cast<unsigned int>(vertexId)];
                    MVector normal(floatNormal.x, floatNormal.y, floatNormal.z);
                    if (normal.length() > 1.0e-10) {
                        normal.normalize();
                        point += normal * boundaryOffset;
                    }
                }
                objectBoundaryLinePoints_.append(point);
            }
        }
    }

    geometryDirty_ = false;
    worldGeometryDirty_ = true;
    return MS::kSuccess;
}

MStatus RegionVisualizer::updateWorldSpaceGeometry()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status;
    const MMatrix inclusiveMatrix = meshPath_.inclusiveMatrix(&status);
    if (!status) {
        return status;
    }
    transformPoints(
        objectProvisionalTrianglePoints_,
        inclusiveMatrix,
        worldProvisionalTrianglePoints_);
    transformPoints(objectCoreTrianglePoints_, inclusiveMatrix, worldCoreTrianglePoints_);
    transformPoints(
        objectTransitionTrianglePoints_,
        inclusiveMatrix,
        worldTransitionTrianglePoints_);
    transformPoints(objectBoundaryLinePoints_, inclusiveMatrix, worldBoundaryLinePoints_);
    cachedInclusiveMatrix_ = inclusiveMatrix;
    worldGeometryDirty_ = false;
    return MS::kSuccess;
}

}  // namespace directional_retopo
