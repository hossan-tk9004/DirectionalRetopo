#include "Field/CrossFieldMath.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace directional_retopo {
namespace {

constexpr double kQuarterTurnMultiplier = 4.0;

MVector rotateAroundAxis(const MVector& vector, const MVector& axis, double angle)
{
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    return vector * cosine + (axis ^ vector) * sine +
        axis * ((axis * vector) * (1.0 - cosine));
}

int sharedEdgeId(
    const MeshTopologyCache& topology,
    int sourceFaceId,
    int targetFaceId)
{
    const std::vector<MeshFaceTopology>& faces = topology.faces();
    const std::vector<MeshEdgeTopology>& edges = topology.edges();
    if (sourceFaceId < 0 || targetFaceId < 0 ||
        static_cast<std::size_t>(sourceFaceId) >= faces.size() ||
        static_cast<std::size_t>(targetFaceId) >= faces.size()) {
        return -1;
    }
    for (const int edgeId : faces[static_cast<std::size_t>(sourceFaceId)].edgeIds) {
        if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
            continue;
        }
        const auto& edgeFaces = edges[static_cast<std::size_t>(edgeId)].faceIds;
        if (std::find(edgeFaces.begin(), edgeFaces.end(), targetFaceId) !=
            edgeFaces.end()) {
            return edgeId;
        }
    }
    return -1;
}

}  // namespace

std::vector<FaceTangentBasis> CrossFieldMath::buildFaceBases(
    const MeshTopologyCache& topology,
    double epsilon)
{
    const std::vector<MeshFaceTopology>& faces = topology.faces();
    const std::vector<MeshEdgeTopology>& edges = topology.edges();
    const std::vector<MPoint>& positions = topology.worldVertexPositions();
    std::vector<FaceTangentBasis> bases(faces.size());

    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const MeshFaceTopology& face = faces[faceIndex];
        FaceTangentBasis& basis = bases[faceIndex];
        basis.center = face.worldCenter;
        basis.normal = face.worldNormal;
        if (!face.worldGeometryValid || basis.normal.length() <= epsilon) {
            continue;
        }
        basis.normal.normalize();

        MVector longestTangent = MVector::zero;
        double longestLength = 0.0;
        double edgeLengthSum = 0.0;
        std::size_t validEdgeCount = 0;
        for (const int edgeId : face.edgeIds) {
            if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
                continue;
            }
            const MeshEdgeTopology& edge = edges[static_cast<std::size_t>(edgeId)];
            const int first = edge.vertexIds[0];
            const int second = edge.vertexIds[1];
            if (first < 0 || second < 0 ||
                static_cast<std::size_t>(first) >= positions.size() ||
                static_cast<std::size_t>(second) >= positions.size()) {
                continue;
            }
            MVector tangent = positions[static_cast<std::size_t>(second)] -
                positions[static_cast<std::size_t>(first)];
            tangent = projectToTangent(tangent, basis.normal, epsilon);
            const double length = tangent.length();
            if (length <= epsilon || !std::isfinite(length)) {
                continue;
            }
            edgeLengthSum += length;
            ++validEdgeCount;
            if (length > longestLength) {
                longestLength = length;
                longestTangent = tangent;
            }
        }

        if (longestLength <= epsilon) {
            const MVector reference = std::abs(basis.normal.x) < 0.8
                ? MVector::xAxis
                : MVector::yAxis;
            longestTangent = projectToTangent(reference, basis.normal, epsilon);
        }
        if (longestTangent.length() <= epsilon) {
            continue;
        }
        longestTangent.normalize();
        basis.basisX = longestTangent;
        basis.basisY = basis.normal ^ basis.basisX;
        if (basis.basisY.length() <= epsilon) {
            continue;
        }
        basis.basisY.normalize();
        basis.basisX = basis.basisY ^ basis.normal;
        basis.basisX.normalize();
        basis.characteristicLength = validEdgeCount > 0
            ? edgeLengthSum / static_cast<double>(validEdgeCount)
            : longestLength;
        basis.valid = std::isfinite(basis.characteristicLength) &&
            basis.characteristicLength > epsilon;
    }
    return bases;
}

MVector CrossFieldMath::projectToTangent(
    const MVector& direction,
    const MVector& normal,
    double epsilon)
{
    if (normal.length() <= epsilon) {
        return MVector::zero;
    }
    MVector unitNormal = normal;
    unitNormal.normalize();
    MVector tangent = direction - unitNormal * (direction * unitNormal);
    if (tangent.length() <= epsilon) {
        return MVector::zero;
    }
    tangent.normalize();
    return tangent;
}

CrossFieldValue CrossFieldMath::encode(
    const MVector& tangentDirection,
    const FaceTangentBasis& basis,
    double epsilon)
{
    if (!basis.valid) {
        return {};
    }
    const MVector tangent = projectToTangent(
        tangentDirection,
        basis.normal,
        epsilon);
    if (tangent.length() <= epsilon) {
        return {};
    }
    const double theta = std::atan2(tangent * basis.basisY, tangent * basis.basisX);
    return normalized(
        std::cos(kQuarterTurnMultiplier * theta),
        std::sin(kQuarterTurnMultiplier * theta),
        epsilon);
}

MVector CrossFieldMath::decode(
    const CrossFieldValue& cross,
    const FaceTangentBasis& basis,
    double epsilon)
{
    if (!cross.valid || !basis.valid) {
        return MVector::zero;
    }
    const double theta =
        std::atan2(cross.y, cross.x) / kQuarterTurnMultiplier;
    MVector direction = basis.basisX * std::cos(theta) +
        basis.basisY * std::sin(theta);
    direction = projectToTangent(direction, basis.normal, epsilon);
    return direction;
}

CrossFieldValue CrossFieldMath::normalized(double x, double y, double epsilon)
{
    const double length = std::sqrt(x * x + y * y);
    if (!std::isfinite(length) || length <= epsilon) {
        return {};
    }
    CrossFieldValue result;
    result.x = x / length;
    result.y = y / length;
    result.valid = true;
    return result;
}

bool CrossFieldMath::transportDirection(
    const MeshTopologyCache& topology,
    const std::vector<FaceTangentBasis>& bases,
    int sourceFaceId,
    int targetFaceId,
    const MVector& sourceDirection,
    MVector& transportedDirection,
    double epsilon)
{
    transportedDirection = MVector::zero;
    if (sourceFaceId < 0 || targetFaceId < 0 ||
        static_cast<std::size_t>(sourceFaceId) >= bases.size() ||
        static_cast<std::size_t>(targetFaceId) >= bases.size() ||
        !bases[static_cast<std::size_t>(sourceFaceId)].valid ||
        !bases[static_cast<std::size_t>(targetFaceId)].valid) {
        return false;
    }

    const FaceTangentBasis& sourceBasis =
        bases[static_cast<std::size_t>(sourceFaceId)];
    const FaceTangentBasis& targetBasis =
        bases[static_cast<std::size_t>(targetFaceId)];
    MVector tangent = projectToTangent(sourceDirection, sourceBasis.normal, epsilon);
    if (tangent.length() <= epsilon) {
        return false;
    }

    MVector rotationAxis = MVector::zero;
    const int edgeId = sharedEdgeId(topology, sourceFaceId, targetFaceId);
    if (edgeId >= 0 && static_cast<std::size_t>(edgeId) < topology.edges().size()) {
        const MeshEdgeTopology& edge = topology.edges()[static_cast<std::size_t>(edgeId)];
        const auto& positions = topology.worldVertexPositions();
        if (edge.vertexIds[0] >= 0 && edge.vertexIds[1] >= 0 &&
            static_cast<std::size_t>(edge.vertexIds[0]) < positions.size() &&
            static_cast<std::size_t>(edge.vertexIds[1]) < positions.size()) {
            rotationAxis =
                positions[static_cast<std::size_t>(edge.vertexIds[1])] -
                positions[static_cast<std::size_t>(edge.vertexIds[0])];
        }
    }

    double angle = 0.0;
    if (rotationAxis.length() > epsilon) {
        rotationAxis.normalize();
        const double sine = rotationAxis *
            (sourceBasis.normal ^ targetBasis.normal);
        const double cosine = std::clamp(
            sourceBasis.normal * targetBasis.normal,
            -1.0,
            1.0);
        angle = std::atan2(sine, cosine);
    } else {
        rotationAxis = sourceBasis.normal ^ targetBasis.normal;
        if (rotationAxis.length() > epsilon) {
            rotationAxis.normalize();
            angle = std::acos(std::clamp(
                sourceBasis.normal * targetBasis.normal,
                -1.0,
                1.0));
        }
    }

    if (rotationAxis.length() > epsilon && std::abs(angle) > epsilon) {
        tangent = rotateAroundAxis(tangent, rotationAxis, angle);
    }
    transportedDirection = projectToTangent(tangent, targetBasis.normal, epsilon);
    return transportedDirection.length() > epsilon;
}

CrossFieldValue CrossFieldMath::transportCross(
    const MeshTopologyCache& topology,
    const std::vector<FaceTangentBasis>& bases,
    int sourceFaceId,
    int targetFaceId,
    const CrossFieldValue& sourceCross,
    double epsilon)
{
    if (sourceFaceId < 0 || static_cast<std::size_t>(sourceFaceId) >= bases.size()) {
        return {};
    }
    const MVector sourceDirection = decode(
        sourceCross,
        bases[static_cast<std::size_t>(sourceFaceId)],
        epsilon);
    MVector transportedDirection;
    if (!transportDirection(
            topology,
            bases,
            sourceFaceId,
            targetFaceId,
            sourceDirection,
            transportedDirection,
            epsilon)) {
        return {};
    }
    return encode(
        transportedDirection,
        bases[static_cast<std::size_t>(targetFaceId)],
        epsilon);
}

CrossFieldValue CrossFieldMath::existingTopologyOrientation(
    const MeshTopologyCache& topology,
    const FaceTangentBasis& basis,
    int faceId,
    double& confidence,
    double epsilon)
{
    confidence = 0.0;
    const auto& faces = topology.faces();
    const auto& edges = topology.edges();
    const auto& positions = topology.worldVertexPositions();
    if (!basis.valid || faceId < 0 ||
        static_cast<std::size_t>(faceId) >= faces.size()) {
        return {};
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double totalWeight = 0.0;
    double longestLength = 0.0;
    CrossFieldValue longestCross;
    for (const int edgeId : faces[static_cast<std::size_t>(faceId)].edgeIds) {
        if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
            continue;
        }
        const MeshEdgeTopology& edge = edges[static_cast<std::size_t>(edgeId)];
        if (edge.vertexIds[0] < 0 || edge.vertexIds[1] < 0 ||
            static_cast<std::size_t>(edge.vertexIds[0]) >= positions.size() ||
            static_cast<std::size_t>(edge.vertexIds[1]) >= positions.size()) {
            continue;
        }
        const MVector edgeDirection =
            positions[static_cast<std::size_t>(edge.vertexIds[1])] -
            positions[static_cast<std::size_t>(edge.vertexIds[0])];
        const MVector tangent = projectToTangent(edgeDirection, basis.normal, epsilon);
        const double length = edgeDirection.length();
        const CrossFieldValue cross = encode(tangent, basis, epsilon);
        if (!cross.valid || !std::isfinite(length) || length <= epsilon) {
            continue;
        }
        sumX += cross.x * length;
        sumY += cross.y * length;
        totalWeight += length;
        if (length > longestLength) {
            longestLength = length;
            longestCross = cross;
        }
    }

    if (totalWeight <= epsilon) {
        return {};
    }
    const double magnitude = std::sqrt(sumX * sumX + sumY * sumY);
    confidence = std::clamp(magnitude / totalWeight, 0.0, 1.0);
    CrossFieldValue result = normalized(sumX, sumY, epsilon);
    if (!result.valid) {
        result = longestCross;
        confidence = result.valid ? 0.1 : 0.0;
    }
    return result;
}

}  // namespace directional_retopo
