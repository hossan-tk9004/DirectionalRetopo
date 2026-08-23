#pragma once

#include "Mesh/MeshTopologyCache.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <vector>

namespace directional_retopo {

struct FaceTangentBasis final
{
    MPoint center;
    MVector normal;
    MVector basisX;
    MVector basisY;
    double characteristicLength = 0.0;
    bool valid = false;
};

struct CrossFieldValue final
{
    double x = 0.0;
    double y = 0.0;
    bool valid = false;
};

class CrossFieldMath final
{
public:
    static std::vector<FaceTangentBasis> buildFaceBases(
        const MeshTopologyCache& topology,
        double epsilon = 1.0e-10);

    static MVector projectToTangent(
        const MVector& direction,
        const MVector& normal,
        double epsilon = 1.0e-10);

    static CrossFieldValue encode(
        const MVector& tangentDirection,
        const FaceTangentBasis& basis,
        double epsilon = 1.0e-10);

    static MVector decode(
        const CrossFieldValue& cross,
        const FaceTangentBasis& basis,
        double epsilon = 1.0e-10);

    static CrossFieldValue normalized(
        double x,
        double y,
        double epsilon = 1.0e-10);

    static bool transportDirection(
        const MeshTopologyCache& topology,
        const std::vector<FaceTangentBasis>& bases,
        int sourceFaceId,
        int targetFaceId,
        const MVector& sourceDirection,
        MVector& transportedDirection,
        double epsilon = 1.0e-10);

    static CrossFieldValue transportCross(
        const MeshTopologyCache& topology,
        const std::vector<FaceTangentBasis>& bases,
        int sourceFaceId,
        int targetFaceId,
        const CrossFieldValue& sourceCross,
        double epsilon = 1.0e-10);

    static CrossFieldValue existingTopologyOrientation(
        const MeshTopologyCache& topology,
        const FaceTangentBasis& basis,
        int faceId,
        double& confidence,
        double epsilon = 1.0e-10);
};

}  // namespace directional_retopo
