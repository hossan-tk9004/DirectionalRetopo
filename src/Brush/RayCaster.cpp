#include "Brush/RayCaster.h"

#include <maya/M3dView.h>
#include <maya/MFloatPoint.h>
#include <maya/MFloatVector.h>

namespace directional_retopo {

MStatus RayCaster::setTarget(const MDagPath& meshPath)
{
    MStatus status;
    MFnMesh mesh(meshPath, &status);
    if (!status) {
        clearTarget();
        return status;
    }

    meshPath_ = meshPath;
    meshHandle_ = MObjectHandle(meshPath.node());
    accelerator_ = MFnMesh::autoUniformGridParams();
    hasTarget_ = true;
    return MS::kSuccess;
}

void RayCaster::clearTarget() noexcept
{
    meshPath_ = MDagPath();
    meshHandle_ = MObjectHandle();
    hasTarget_ = false;
}

bool RayCaster::hasTarget() const noexcept
{
    return hasTarget_ && meshHandle_.isValid() && meshHandle_.isAlive() &&
        meshPath_.isValid();
}

const MDagPath& RayCaster::targetPath() const noexcept
{
    return meshPath_;
}

bool RayCaster::castFromViewport(short x, short y, SurfaceHit& hit)
{
    MStatus status;
    const M3dView view = M3dView::active3dView(&status);
    if (!status) {
        return false;
    }

    MPoint rayOrigin;
    MVector rayDirection;
    status = view.viewToWorld(x, y, rayOrigin, rayDirection);
    if (!status) {
        return false;
    }

    return castWorldRay(rayOrigin, rayDirection, hit);
}

bool RayCaster::castWorldRay(
    const MPoint& rayOrigin,
    const MVector& rayDirection,
    SurfaceHit& hit)
{
    if (!hasTarget() || rayDirection.length() <= kDirectionEpsilon) {
        return false;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return false;
    }

    MVector normalizedDirection = rayDirection;
    normalizedDirection.normalize();

    const MFloatPoint floatOrigin(
        static_cast<float>(rayOrigin.x),
        static_cast<float>(rayOrigin.y),
        static_cast<float>(rayOrigin.z));
    const MFloatVector floatDirection(
        static_cast<float>(normalizedDirection.x),
        static_cast<float>(normalizedDirection.y),
        static_cast<float>(normalizedDirection.z));

    MFloatPoint hitPoint;
    float hitRayParameter = 0.0F;
    int hitFace = -1;
    int hitTriangle = -1;
    float hitBarycentric1 = 0.0F;
    float hitBarycentric2 = 0.0F;

    const bool didHit = mesh.closestIntersection(
        floatOrigin,
        floatDirection,
        nullptr,
        nullptr,
        false,
        MSpace::kWorld,
        kMaximumRayParameter,
        false,
        &accelerator_,
        hitPoint,
        &hitRayParameter,
        &hitFace,
        &hitTriangle,
        &hitBarycentric1,
        &hitBarycentric2,
        1.0e-6F,
        &status);

    if (!status || !didHit) {
        return false;
    }

    const MPoint worldHit(
        static_cast<double>(hitPoint.x),
        static_cast<double>(hitPoint.y),
        static_cast<double>(hitPoint.z));

    MVector worldNormal;
    int normalFace = hitFace;
    status = mesh.getClosestNormal(
        worldHit,
        worldNormal,
        MSpace::kWorld,
        &normalFace,
        &accelerator_);
    if (!status || worldNormal.length() <= kDirectionEpsilon) {
        return false;
    }
    worldNormal.normalize();

    hit.position = worldHit;
    hit.normal = worldNormal;
    hit.rayParameter = hitRayParameter;
    hit.faceId = hitFace >= 0 ? hitFace : normalFace;
    hit.triangleId = hitTriangle;
    hit.barycentric1 = hitBarycentric1;
    hit.barycentric2 = hitBarycentric2;
    return true;
}

}  // namespace directional_retopo
