#pragma once

#include <maya/MDagPath.h>
#include <maya/MFnMesh.h>
#include <maya/MObjectHandle.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MVector.h>

namespace directional_retopo {

struct SurfaceHit
{
    MPoint position;
    MVector normal;
    float rayParameter = 0.0F;
    int faceId = -1;
    int triangleId = -1;
    float barycentric1 = 0.0F;
    float barycentric2 = 0.0F;
};

class RayCaster final
{
public:
    MStatus setTarget(const MDagPath& meshPath);
    void clearTarget() noexcept;

    [[nodiscard]] bool hasTarget() const noexcept;
    [[nodiscard]] const MDagPath& targetPath() const noexcept;

    bool castFromViewport(short x, short y, SurfaceHit& hit);
    bool castWorldRay(
        const MPoint& rayOrigin,
        const MVector& rayDirection,
        SurfaceHit& hit);

private:
    static constexpr float kMaximumRayParameter = 1.0e7F;
    static constexpr double kDirectionEpsilon = 1.0e-12;

    MDagPath meshPath_;
    MObjectHandle meshHandle_;
    MMeshIsectAccelParams accelerator_;
    bool hasTarget_ = false;
};

}  // namespace directional_retopo
