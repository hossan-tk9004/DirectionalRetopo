#include "Paint/RegionPreviewCalculator.h"

#include <maya/MFnMesh.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MMatrix.h>
#include <maya/MPointArray.h>

#include <algorithm>
#include <cmath>
#include <deque>

namespace directional_retopo {

MStatus RegionPreviewCalculator::setTarget(const MDagPath& meshPath)
{
    clear();

    MStatus status;
    MFnMesh mesh(meshPath, &status);
    if (!status) {
        return status;
    }

    const int faceCount = mesh.numPolygons(&status);
    if (!status || faceCount < 0) {
        return status ? MS::kFailure : status;
    }

    meshPath_ = meshPath;
    meshHandle_ = MObjectHandle(meshPath.node());
    faces_.resize(static_cast<std::size_t>(faceCount));
    visitGenerations_.assign(static_cast<std::size_t>(faceCount), 0U);

    MObject component = MObject::kNullObj;
    MItMeshPolygon iterator(meshPath_, component, &status);
    if (!status) {
        clear();
        return status;
    }

    for (; !iterator.isDone(&status) && status; iterator.next()) {
        const unsigned int faceId = iterator.index(&status);
        if (!status || faceId >= faces_.size()) {
            clear();
            return status ? MS::kFailure : status;
        }

        FaceBounds& bounds = faces_[faceId];
        bounds.centerObject = iterator.center(MSpace::kObject, &status);
        if (!status) {
            clear();
            return status;
        }

        MPointArray facePoints;
        iterator.getPoints(facePoints, MSpace::kObject, &status);
        if (!status) {
            clear();
            return status;
        }
        bounds.radiusObject = 0.0;
        for (unsigned int pointIndex = 0; pointIndex < facePoints.length(); ++pointIndex) {
            bounds.radiusObject = std::max(
                bounds.radiusObject,
                (facePoints[pointIndex] - bounds.centerObject).length());
        }

        MIntArray connectedFaces;
        status = iterator.getConnectedFaces(connectedFaces);
        if (!status) {
            clear();
            return status;
        }
        bounds.connectedFaces.reserve(connectedFaces.length());
        for (unsigned int index = 0; index < connectedFaces.length(); ++index) {
            const int connectedFace = connectedFaces[index];
            if (connectedFace >= 0 && connectedFace < faceCount) {
                bounds.connectedFaces.push_back(connectedFace);
            }
        }
    }

    if (!status) {
        clear();
        return status;
    }
    return MS::kSuccess;
}

void RegionPreviewCalculator::clear() noexcept
{
    meshPath_ = MDagPath();
    meshHandle_ = MObjectHandle();
    faces_.clear();
    visitGenerations_.clear();
    currentVisitGeneration_ = 0;
}

bool RegionPreviewCalculator::hasTarget() const noexcept
{
    return meshHandle_.isValid() && meshHandle_.isAlive() && meshPath_.isValid() &&
        !faces_.empty();
}

const RegionPreviewCalculationSettings& RegionPreviewCalculator::settings() const noexcept
{
    return settings_;
}

void RegionPreviewCalculator::setSettings(
    const RegionPreviewCalculationSettings& settings) noexcept
{
    settings_ = settings;
    settings_.brushRadiusPaddingRatio =
        std::max(settings_.brushRadiusPaddingRatio, 0.0);
}

bool RegionPreviewCalculator::addFacesForSample(
    const StrokeSample& sample,
    std::unordered_set<int>& faceIds)
{
    if (!hasTarget() || sample.faceId < 0 ||
        static_cast<std::size_t>(sample.faceId) >= faces_.size() || sample.radius <= 0.0) {
        return false;
    }

    MStatus status;
    const MMatrix inclusiveMatrix = meshPath_.inclusiveMatrix(&status);
    if (!status) {
        return false;
    }

    const double maximumScale = std::max({
        (MVector::xAxis * inclusiveMatrix).length(),
        (MVector::yAxis * inclusiveMatrix).length(),
        (MVector::zAxis * inclusiveMatrix).length()});
    if (!std::isfinite(maximumScale) || maximumScale <= 0.0) {
        return false;
    }

    ++currentVisitGeneration_;
    if (currentVisitGeneration_ == 0U) {
        std::fill(visitGenerations_.begin(), visitGenerations_.end(), 0U);
        currentVisitGeneration_ = 1U;
    }

    std::deque<int> pendingFaces;
    pendingFaces.push_back(sample.faceId);
    visitGenerations_[static_cast<std::size_t>(sample.faceId)] =
        currentVisitGeneration_;

    bool changed = false;
    const double paddedBrushRadius =
        sample.radius * (1.0 + settings_.brushRadiusPaddingRatio);

    while (!pendingFaces.empty()) {
        const int faceId = pendingFaces.front();
        pendingFaces.pop_front();

        const FaceBounds& bounds = faces_[static_cast<std::size_t>(faceId)];
        const MPoint centerWorld = bounds.centerObject * inclusiveMatrix;
        const double faceRadiusWorld = bounds.radiusObject * maximumScale;
        const double centerDistance = (centerWorld - sample.position).length();
        const bool intersectsBrush = faceId == sample.faceId ||
            centerDistance <= paddedBrushRadius + faceRadiusWorld;
        if (!intersectsBrush) {
            continue;
        }

        changed = faceIds.insert(faceId).second || changed;
        for (const int connectedFace : bounds.connectedFaces) {
            const std::size_t connectedIndex = static_cast<std::size_t>(connectedFace);
            if (visitGenerations_[connectedIndex] == currentVisitGeneration_) {
                continue;
            }
            visitGenerations_[connectedIndex] = currentVisitGeneration_;
            pendingFaces.push_back(connectedFace);
        }
    }

    return changed;
}

}  // namespace directional_retopo
