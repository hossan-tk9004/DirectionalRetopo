#include "Solver/ScaffoldAdaptationSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace directional_retopo::solver {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kMaximumCrossDeviation = 0.25 * kPi;

double clamp(double value, double minimum, double maximum) noexcept
{
    return std::max(minimum, std::min(value, maximum));
}

double median(std::vector<double> values)
{
    if (values.empty()) { return 0.0; }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
        ? 0.5 * (values[middle - 1U] + values[middle])
        : values[middle];
}

Vec3 polygonAreaVector(
    const MutableFace& face,
    const LocalMutablePatchMesh& mesh)
{
    Vec3 area;
    for (std::size_t index = 0U; index < face.vertexIds.size(); ++index) {
        const Vec3& current = mesh.vertices()[face.vertexIds[index]].position;
        const Vec3& next = mesh.vertices()[
            face.vertexIds[(index + 1U) % face.vertexIds.size()]].position;
        area += current.cross(next);
    }
    return area * 0.5;
}

double pointTriangleDistance(
    const Vec3& point,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c)
{
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = point - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) { return ap.length(); }

    const Vec3 bp = point - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) { return bp.length(); }

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return (point - (a + ab * v)).length();
    }

    const Vec3 cp = point - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) { return cp.length(); }

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return (point - (a + ac * w)).length();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) /
            ((d4 - d3) + (d5 - d6));
        return (point - (b + (c - b) * w)).length();
    }

    const Vec3 normal = ab.cross(ac).normalized();
    return normal.squaredLength() > 0.0
        ? std::abs((point - a).dot(normal))
        : std::numeric_limits<double>::infinity();
}

struct EvaluationContext final
{
    const SourceMeshSnapshot& source;
    const RegionComponent& component;
    const std::vector<FaceDirection>& directions;
    const std::vector<FaceDensity>& densities;
    unsigned int requestedBlendWidth = 1U;
    unsigned int activeRingCount = 1U;
    double epsilon = 1.0e-12;
    std::map<SourceId, std::size_t> sourceFaceIndexById;

    EvaluationContext(
        const SourceMeshSnapshot& sourceMesh,
        const RegionComponent& region,
        const std::vector<FaceDirection>& directionField,
        const std::vector<FaceDensity>& densityField,
        const RemeshSettings& settings,
        const LocalMutablePatchMesh& mesh)
        : source(sourceMesh),
          component(region),
          directions(directionField),
          densities(densityField),
          requestedBlendWidth(std::max(1U, settings.topologyBlendWidth)),
          activeRingCount(std::max(
              1U,
              std::min(
                  std::max(1U, settings.topologyBlendWidth),
                  std::max(1U, mesh.diagnostics().actualMaximumRingDepth)))),
          epsilon(std::max(1.0e-12, settings.geometryEpsilon))
    {
        for (std::size_t index = 0U; index < source.faces.size(); ++index) {
            sourceFaceIndexById.emplace(source.faces[index].sourceFaceId, index);
        }
    }

    [[nodiscard]] std::vector<std::size_t> sourceFaces(
        const MutableFace& face) const
    {
        std::vector<std::size_t> result;
        for (const SourceId id : face.sourceFaceIds) {
            const auto found = sourceFaceIndexById.find(id);
            if (found != sourceFaceIndexById.end()) {
                result.push_back(found->second);
            }
        }
        std::sort(result.begin(), result.end());
        result.erase(std::unique(result.begin(), result.end()), result.end());
        return result;
    }

    [[nodiscard]] double coreInfluence(std::size_t sourceFaceIndex) const
    {
        if (sourceFaceIndex >= component.transitionRingDepthByFace.size()) {
            return 0.0;
        }
        const int depth = component.transitionRingDepthByFace[sourceFaceIndex];
        if (depth <= 0 || static_cast<unsigned int>(depth) > activeRingCount) {
            return depth == 0 ? 1.0 : 0.0;
        }
        if (activeRingCount == 1U) { return 1.0; }
        return clamp(
            1.0 - static_cast<double>(depth - 1) /
                static_cast<double>(activeRingCount - 1U),
            0.0,
            1.0);
    }

    [[nodiscard]] double sourceTarget(std::size_t sourceFaceIndex) const
    {
        if (sourceFaceIndex >= source.faces.size()) { return 0.0; }
        std::vector<double> lengths;
        for (const std::size_t edgeIndex : source.faces[sourceFaceIndex].edgeIndices) {
            if (edgeIndex < source.edges.size()) {
                const double length = source.edges[edgeIndex].length;
                if (std::isfinite(length) && length > epsilon) {
                    lengths.push_back(length);
                }
            }
        }
        return median(std::move(lengths));
    }

    [[nodiscard]] double requestedTarget(std::size_t sourceFaceIndex) const
    {
        if (sourceFaceIndex >= densities.size() ||
            !densities[sourceFaceIndex].valid) {
            return sourceTarget(sourceFaceIndex);
        }
        const FaceDensity& density = densities[sourceFaceIndex];
        if (std::isfinite(density.effectiveTargetEdgeLength) &&
            density.effectiveTargetEdgeLength > epsilon) {
            return density.effectiveTargetEdgeLength;
        }
        return std::isfinite(density.requestedTargetEdgeLength) &&
                density.requestedTargetEdgeLength > epsilon
            ? density.requestedTargetEdgeLength
            : sourceTarget(sourceFaceIndex);
    }

    [[nodiscard]] double targetForSourceFace(
        std::size_t sourceFaceIndex) const
    {
        const double sourceLength = sourceTarget(sourceFaceIndex);
        const double requestedLength = requestedTarget(sourceFaceIndex);
        if (sourceLength <= epsilon) { return requestedLength; }
        if (requestedLength <= epsilon) { return sourceLength; }
        const double weight = coreInfluence(sourceFaceIndex);
        return std::exp(
            (1.0 - weight) * std::log(sourceLength) +
            weight * std::log(requestedLength));
    }

    [[nodiscard]] double targetForFace(const MutableFace& face) const
    {
        const std::vector<std::size_t> provenance = sourceFaces(face);
        double logSum = 0.0;
        std::size_t count = 0U;
        for (const std::size_t sourceFaceIndex : provenance) {
            const double target = targetForSourceFace(sourceFaceIndex);
            if (std::isfinite(target) && target > epsilon) {
                logSum += std::log(target);
                ++count;
            }
        }
        return count == 0U ? 0.0 : std::exp(logSum / static_cast<double>(count));
    }

    [[nodiscard]] double targetForEdge(
        const MutableEdge& edge,
        const LocalMutablePatchMesh& mesh) const
    {
        double logSum = 0.0;
        std::size_t count = 0U;
        for (const MutableFaceId faceId : edge.faceIds) {
            if (faceId >= mesh.faces().size() || mesh.faces()[faceId].deleted) {
                continue;
            }
            const double target = targetForFace(mesh.faces()[faceId]);
            if (std::isfinite(target) && target > epsilon) {
                logSum += std::log(target);
                ++count;
            }
        }
        return count == 0U ? 0.0 : std::exp(logSum / static_cast<double>(count));
    }
};

double faceCoreInfluence(
    const MutableFace& face,
    const EvaluationContext& context)
{
    const std::vector<std::size_t> provenance = context.sourceFaces(face);
    if (provenance.empty()) { return 0.0; }
    double sum = 0.0;
    for (const std::size_t faceIndex : provenance) {
        sum += context.coreInfluence(faceIndex);
    }
    return sum / static_cast<double>(provenance.size());
}

double directionDeviation(
    const Vec3& edgeDirection,
    std::size_t sourceFaceIndex,
    const EvaluationContext& context)
{
    if (sourceFaceIndex >= context.directions.size()) {
        return 0.0;
    }
    const FaceDirection& field = context.directions[sourceFaceIndex];
    if (!field.valid) { return 0.0; }
    const Vec3 normal = field.normal.normalized(context.epsilon);
    const Vec3 tangent =
        (edgeDirection - normal * edgeDirection.dot(normal))
            .normalized(context.epsilon);
    const Vec3 u = field.uDirection.normalized(context.epsilon);
    const Vec3 v = field.vDirection.normalized(context.epsilon);
    if (tangent.squaredLength() == 0.0 ||
        u.squaredLength() == 0.0 ||
        v.squaredLength() == 0.0) {
        return 0.0;
    }
    const double alignment = clamp(
        std::max(std::abs(tangent.dot(u)), std::abs(tangent.dot(v))),
        0.0,
        1.0);
    return std::min(kMaximumCrossDeviation, std::acos(alignment));
}

double localSurfaceDistance(
    const Vec3& point,
    const std::vector<std::size_t>& sourceFaceIndices,
    const EvaluationContext& context)
{
    double distance = std::numeric_limits<double>::infinity();
    std::set<std::size_t> triangles;
    for (const std::size_t faceIndex : sourceFaceIndices) {
        if (faceIndex >= context.source.faces.size()) { continue; }
        for (const std::size_t triangleIndex :
             context.source.faces[faceIndex].triangleIndices) {
            triangles.insert(triangleIndex);
        }
        for (const std::size_t adjacent :
             context.source.faces[faceIndex].adjacentFaceIndices) {
            if (adjacent >= context.source.faces.size()) { continue; }
            for (const std::size_t triangleIndex :
                 context.source.faces[adjacent].triangleIndices) {
                triangles.insert(triangleIndex);
            }
        }
    }
    for (const std::size_t triangleIndex : triangles) {
        if (triangleIndex >= context.source.triangles.size()) { continue; }
        const SourceTriangle& triangle = context.source.triangles[triangleIndex];
        if (triangle.vertexIndices[0] >= context.source.vertices.size() ||
            triangle.vertexIndices[1] >= context.source.vertices.size() ||
            triangle.vertexIndices[2] >= context.source.vertices.size()) {
            continue;
        }
        distance = std::min(
            distance,
            pointTriangleDistance(
                point,
                context.source.vertices[triangle.vertexIndices[0]].position,
                context.source.vertices[triangle.vertexIndices[1]].position,
                context.source.vertices[triangle.vertexIndices[2]].position));
    }
    return distance;
}

std::vector<std::size_t> edgeSourceFaces(
    const MutableEdge& edge,
    const LocalMutablePatchMesh& mesh,
    const EvaluationContext& context)
{
    std::vector<std::size_t> result;
    for (const MutableFaceId faceId : edge.faceIds) {
        if (faceId >= mesh.faces().size() || mesh.faces()[faceId].deleted) {
            continue;
        }
        const std::vector<std::size_t> provenance =
            context.sourceFaces(mesh.faces()[faceId]);
        result.insert(result.end(), provenance.begin(), provenance.end());
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

ScaffoldAdaptationMetrics evaluateMesh(
    const LocalMutablePatchMesh& mesh,
    const EvaluationContext& context)
{
    ScaffoldAdaptationMetrics result;
    double densitySum = 0.0;
    double directionSum = 0.0;
    double directionWeightSum = 0.0;
    double surfaceSum = 0.0;
    double normalizedSurfaceSum = 0.0;
    std::size_t densitySamples = 0U;
    std::size_t surfaceSamples = 0U;
    double interfacePerimeter = 0.0;
    double interfaceLogTarget = 0.0;
    std::size_t interfaceTargetSamples = 0U;

    for (const MutableVertex& vertex : mesh.vertices()) {
        if (!vertex.deleted) { ++result.vertexCount; }
    }
    for (const MutableFace& face : mesh.faces()) {
        if (face.deleted) { continue; }
        ++result.faceCount;
        if (face.vertexIds.size() == 3U) {
            ++result.triangleCount;
        } else if (face.vertexIds.size() == 4U) {
            ++result.quadCount;
        } else {
            ++result.nGonCount;
        }
        if (face.origin == MutableElementOrigin::Derived &&
            face.vertexIds.size() > 4U) {
            const double excess =
                static_cast<double>(face.vertexIds.size() - 4U);
            const double outerWeight = 1.0 - faceCoreInfluence(face, context);
            result.cost.valencePenalty += excess * excess *
                (0.35 + 0.65 * outerWeight);
        }

        Vec3 center;
        for (const MutableVertexId vertexId : face.vertexIds) {
            center += mesh.vertices()[vertexId].position;
        }
        center = center / static_cast<double>(face.vertexIds.size());
        const std::vector<std::size_t> provenance = context.sourceFaces(face);
        const double distance =
            localSurfaceDistance(center, provenance, context);
        const double target = context.targetForFace(face);
        if (std::isfinite(distance)) {
            surfaceSum += distance;
            result.maximumSurfaceError =
                std::max(result.maximumSurfaceError, distance);
            normalizedSurfaceSum += target > context.epsilon
                ? distance / target : distance;
            ++surfaceSamples;
        }
    }

    for (const MutableEdge& edge : mesh.edges()) {
        if (edge.deleted) { continue; }
        ++result.edgeCount;
        const Vec3 edgeVector =
            mesh.vertices()[edge.vertex1].position -
            mesh.vertices()[edge.vertex0].position;
        const double length = edgeVector.length();
        const double target = context.targetForEdge(edge, mesh);
        if (!edge.fixedOuterBoundary && length > context.epsilon &&
            target > context.epsilon) {
            const double error = std::abs(std::log(length / target));
            densitySum += error;
            result.maximumDensityError =
                std::max(result.maximumDensityError, error);
            ++densitySamples;
        }

        double edgeDirectionCost = 0.0;
        double edgeDirectionWeight = 0.0;
        for (const MutableFaceId faceId : edge.faceIds) {
            if (faceId >= mesh.faces().size() || mesh.faces()[faceId].deleted) {
                continue;
            }
            const MutableFace& face = mesh.faces()[faceId];
            for (const std::size_t sourceFaceIndex :
                 context.sourceFaces(face)) {
                const double weight =
                    0.20 + 0.80 * context.coreInfluence(sourceFaceIndex);
                edgeDirectionCost += weight * directionDeviation(
                    edgeVector, sourceFaceIndex, context);
                edgeDirectionWeight += weight;
            }
        }
        if (!edge.fixedOuterBoundary && edgeDirectionWeight > 0.0) {
            const double deviation =
                edgeDirectionCost / edgeDirectionWeight;
            directionSum += deviation;
            directionWeightSum += 1.0;
            result.maximumDirectionDeviationDegrees = std::max(
                result.maximumDirectionDeviationDegrees,
                deviation * kRadiansToDegrees);
        }

        const std::vector<std::size_t> provenance =
            edgeSourceFaces(edge, mesh, context);
        const double midpointDistance = localSurfaceDistance(
            (mesh.vertices()[edge.vertex0].position +
             mesh.vertices()[edge.vertex1].position) * 0.5,
            provenance,
            context);
        if (std::isfinite(midpointDistance)) {
            surfaceSum += midpointDistance;
            result.maximumSurfaceError =
                std::max(result.maximumSurfaceError, midpointDistance);
            normalizedSurfaceSum += target > context.epsilon
                ? midpointDistance / target : midpointDistance;
            ++surfaceSamples;
        }

        if (edge.innerInterface) {
            interfacePerimeter += length;
            if (target > context.epsilon) {
                interfaceLogTarget += std::log(target);
                ++interfaceTargetSamples;
            }
        }
        if (edge.origin == MutableElementOrigin::Derived) {
            double coreWeight = 0.0;
            for (const MutableFaceId faceId : edge.faceIds) {
                if (faceId < mesh.faces().size() &&
                    !mesh.faces()[faceId].deleted) {
                    coreWeight = std::max(
                        coreWeight,
                        faceCoreInfluence(mesh.faces()[faceId], context));
                }
            }
            result.cost.sourcePreservation += 1.0 - coreWeight;
        }
    }

    result.innerInterfaceVertexCount =
        mesh.orderedInnerInterfaceVertices().size();
    if (interfaceTargetSamples > 0U) {
        const double target = std::exp(
            interfaceLogTarget /
            static_cast<double>(interfaceTargetSamples));
        result.approximateDesiredInterfaceCount = std::max<std::size_t>(
            3U,
            static_cast<std::size_t>(std::llround(interfacePerimeter / target)));
    } else {
        result.approximateDesiredInterfaceCount =
            result.innerInterfaceVertexCount;
    }
    result.meanDensityError = densitySamples == 0U
        ? 0.0 : densitySum / static_cast<double>(densitySamples);
    result.meanDirectionDeviationDegrees = directionWeightSum == 0.0
        ? 0.0
        : directionSum / directionWeightSum * kRadiansToDegrees;
    result.meanSurfaceError = surfaceSamples == 0U
        ? 0.0 : surfaceSum / static_cast<double>(surfaceSamples);
    result.maximumFixedBoundaryDisplacement =
        mesh.diagnostics().maximumFixedBoundaryDisplacement;

    const double faceDenominator =
        static_cast<double>(std::max<std::size_t>(1U, result.faceCount));
    const double edgeDenominator =
        static_cast<double>(std::max<std::size_t>(1U, result.edgeCount));
    result.cost.density = result.meanDensityError;
    result.cost.direction = directionWeightSum == 0.0
        ? 0.0
        : (directionSum / directionWeightSum) / kMaximumCrossDeviation;
    result.cost.surface = surfaceSamples == 0U
        ? 0.0 : normalizedSurfaceSum / static_cast<double>(surfaceSamples);
    result.cost.topologyQuality =
        (0.08 * static_cast<double>(result.triangleCount) +
         0.12 * static_cast<double>(result.nGonCount)) / faceDenominator;
    result.cost.valencePenalty /= faceDenominator;
    result.cost.sourcePreservation /= edgeDenominator;
    result.cost.total =
        4.0 * result.cost.density +
        2.5 * result.cost.direction +
        30.0 * result.cost.surface +
        0.30 * result.cost.topologyQuality +
        0.20 * result.cost.valencePenalty +
        0.10 * result.cost.sourcePreservation;
    return result;
}

struct Candidate final
{
    ScaffoldCandidateType type =
        ScaffoldCandidateType::CollapseEdgeToEndpoint;
    MutableEdgeId edgeId = kInvalidIndex;
    MutableVertexId endpointToKeep = kInvalidIndex;
    double splitParameter = 0.5;
    double priority = 0.0;
};

bool collapseLinkCondition(
    const LocalMutablePatchMesh& mesh,
    const Candidate& candidate,
    const ScaffoldAdaptationSettings& settings)
{
    if (candidate.edgeId >= mesh.edges().size()) { return false; }
    const MutableEdge& edge = mesh.edges()[candidate.edgeId];
    if (edge.deleted || edge.fixedOuterBoundary ||
        candidate.endpointToKeep != edge.vertex0 &&
        candidate.endpointToKeep != edge.vertex1) {
        return false;
    }
    const MutableVertexId removed =
        candidate.endpointToKeep == edge.vertex0 ? edge.vertex1 : edge.vertex0;
    if (mesh.vertices()[candidate.endpointToKeep].fixedOuterBoundary ||
        mesh.vertices()[removed].fixedOuterBoundary) {
        return false;
    }
    if (edge.innerInterface &&
        mesh.orderedInnerInterfaceVertices().size() <=
            settings.minimumInterfaceVertexCount) {
        return false;
    }

    const auto neighbors = [&mesh](MutableVertexId vertexId) {
        std::set<MutableVertexId> result;
        for (const MutableEdgeId edgeId : mesh.vertices()[vertexId].edgeIds) {
            if (edgeId >= mesh.edges().size() || mesh.edges()[edgeId].deleted) {
                continue;
            }
            const MutableEdge& incident = mesh.edges()[edgeId];
            result.insert(
                incident.vertex0 == vertexId
                    ? incident.vertex1 : incident.vertex0);
        }
        return result;
    };
    const std::set<MutableVertexId> keptNeighbors =
        neighbors(candidate.endpointToKeep);
    const std::set<MutableVertexId> removedNeighbors = neighbors(removed);
    std::set<MutableVertexId> permittedCommon;
    for (const MutableFaceId faceId : edge.faceIds) {
        if (faceId >= mesh.faces().size() || mesh.faces()[faceId].deleted) {
            return false;
        }
        for (const MutableVertexId vertexId :
             mesh.faces()[faceId].vertexIds) {
            if (vertexId != candidate.endpointToKeep && vertexId != removed) {
                permittedCommon.insert(vertexId);
            }
        }
    }
    for (const MutableVertexId neighbor : keptNeighbors) {
        if (removedNeighbors.count(neighbor) != 0U &&
            permittedCommon.count(neighbor) == 0U) {
            return false;
        }
    }
    return true;
}

bool affectedFaceOrientationValid(
    const LocalMutablePatchMesh& mesh,
    const MutableOperationRecord& operation,
    const EvaluationContext& context)
{
    std::vector<MutableFaceId> affected = operation.modifiedFaceIds;
    affected.insert(
        affected.end(),
        operation.createdFaceIds.begin(),
        operation.createdFaceIds.end());
    std::sort(affected.begin(), affected.end());
    affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
    for (const MutableFaceId faceId : affected) {
        if (faceId >= mesh.faces().size() || mesh.faces()[faceId].deleted) {
            continue;
        }
        const MutableFace& face = mesh.faces()[faceId];
        const Vec3 polygonNormal =
            polygonAreaVector(face, mesh).normalized(context.epsilon);
        Vec3 sourceNormal;
        for (const std::size_t sourceFaceIndex :
             context.sourceFaces(face)) {
            if (sourceFaceIndex < context.source.faces.size()) {
                sourceNormal += context.source.faces[sourceFaceIndex].normal;
            }
        }
        sourceNormal = sourceNormal.normalized(context.epsilon);
        if (polygonNormal.squaredLength() == 0.0 ||
            sourceNormal.squaredLength() == 0.0 ||
            polygonNormal.dot(sourceNormal) <= 0.0) {
            return false;
        }
    }
    return true;
}

bool affectedVertexLinksValid(
    const LocalMutablePatchMesh& mesh,
    const MutableOperationRecord& operation)
{
    std::set<MutableVertexId> affected(
        operation.modifiedVertexIds.begin(),
        operation.modifiedVertexIds.end());
    affected.insert(
        operation.createdVertexIds.begin(),
        operation.createdVertexIds.end());
    const auto addFaceVertices = [&mesh, &affected](MutableFaceId faceId) {
        if (faceId >= mesh.faces().size() || mesh.faces()[faceId].deleted) {
            return;
        }
        affected.insert(
            mesh.faces()[faceId].vertexIds.begin(),
            mesh.faces()[faceId].vertexIds.end());
    };
    for (const MutableFaceId faceId : operation.modifiedFaceIds) {
        addFaceVertices(faceId);
    }
    for (const MutableFaceId faceId : operation.createdFaceIds) {
        addFaceVertices(faceId);
    }

    for (const MutableVertexId vertexId : affected) {
        if (vertexId >= mesh.vertices().size() ||
            mesh.vertices()[vertexId].deleted) {
            continue;
        }
        const MutableVertex& vertex = mesh.vertices()[vertexId];
        if (vertex.faceIds.empty()) { return false; }
        std::map<MutableFaceId, std::vector<MutableFaceId>> adjacency;
        for (const MutableFaceId faceId : vertex.faceIds) {
            adjacency[faceId];
        }
        std::size_t boundarySpokes = 0U;
        for (const MutableEdgeId edgeId : vertex.edgeIds) {
            if (edgeId >= mesh.edges().size() || mesh.edges()[edgeId].deleted) {
                return false;
            }
            const MutableEdge& edge = mesh.edges()[edgeId];
            if (edge.faceIds.size() == 1U) {
                ++boundarySpokes;
            } else if (edge.faceIds.size() == 2U) {
                const MutableFaceId first = edge.faceIds[0];
                const MutableFaceId second = edge.faceIds[1];
                if (adjacency.count(first) != 0U &&
                    adjacency.count(second) != 0U) {
                    adjacency[first].push_back(second);
                    adjacency[second].push_back(first);
                }
            }
        }
        if (boundarySpokes != 0U && boundarySpokes != 2U) {
            return false;
        }
        std::set<MutableFaceId> visited;
        std::vector<MutableFaceId> pending = {vertex.faceIds.front()};
        while (!pending.empty()) {
            const MutableFaceId faceId = pending.back();
            pending.pop_back();
            if (!visited.insert(faceId).second) { continue; }
            for (const MutableFaceId adjacent : adjacency[faceId]) {
                if (visited.count(adjacent) == 0U) {
                    pending.push_back(adjacent);
                }
            }
        }
        if (visited.size() != adjacency.size()) { return false; }
    }
    return true;
}

double representativeTarget(
    const LocalMutablePatchMesh& mesh,
    const EvaluationContext& context)
{
    double logSum = 0.0;
    std::size_t count = 0U;
    for (const MutableEdge& edge : mesh.edges()) {
        if (edge.deleted || edge.fixedOuterBoundary) { continue; }
        const double target = context.targetForEdge(edge, mesh);
        if (std::isfinite(target) && target > context.epsilon) {
            logSum += std::log(target);
            ++count;
        }
    }
    return count == 0U ? 1.0 :
        std::exp(logSum / static_cast<double>(count));
}

bool surfaceConstraintSatisfied(
    const ScaffoldAdaptationMetrics& before,
    const ScaffoldAdaptationMetrics& after,
    const LocalMutablePatchMesh& mesh,
    const EvaluationContext& context,
    const ScaffoldAdaptationSettings& settings)
{
    const double scale = representativeTarget(mesh, context);
    const double maximumAllowed = std::max(
        settings.maximumSurfaceDistanceRatio * scale,
        before.maximumSurfaceError +
            settings.maximumSurfaceErrorIncreaseRatio * scale);
    const double meanAllowed =
        before.meanSurfaceError +
        0.5 * settings.maximumSurfaceErrorIncreaseRatio * scale;
    return after.maximumSurfaceError <= maximumAllowed &&
        after.meanSurfaceError <= meanAllowed;
}

std::vector<Candidate> generateCandidates(
    const LocalMutablePatchMesh& mesh,
    const EvaluationContext& context,
    const ScaffoldAdaptationSettings& settings)
{
    std::vector<Candidate> result;
    for (const MutableEdge& edge : mesh.edges()) {
        if (edge.deleted || edge.fixedOuterBoundary) { continue; }
        const double target = context.targetForEdge(edge, mesh);
        const double length =
            (mesh.vertices()[edge.vertex1].position -
             mesh.vertices()[edge.vertex0].position).length();
        if (!(target > context.epsilon) || !(length > context.epsilon)) {
            continue;
        }
        if (settings.enableCollapse &&
            length < target * settings.collapseLengthRatioThreshold) {
            const MutableVertexId first =
                std::min(edge.vertex0, edge.vertex1);
            const MutableVertexId second =
                std::max(edge.vertex0, edge.vertex1);
            result.push_back({
                ScaffoldCandidateType::CollapseEdgeToEndpoint,
                edge.id,
                first,
                0.5,
                std::log(target / length)});
            result.push_back({
                ScaffoldCandidateType::CollapseEdgeToEndpoint,
                edge.id,
                second,
                0.5,
                std::log(target / length)});
        }
        const bool sourceLineage =
            edge.sourceEdgeId != kInvalidSourceId ||
            edge.parentSourceEdgeId != kInvalidSourceId;
        if (settings.enableSourceLineageSplit && sourceLineage &&
            length > target * settings.splitLengthRatioThreshold) {
            result.push_back({
                ScaffoldCandidateType::SplitSourceLineageEdge,
                edge.id,
                kInvalidIndex,
                0.5,
                std::log(length / target)});
        }
        if (settings.enableTriangleFlip && !edge.innerInterface &&
            edge.faceIds.size() == 2U &&
            mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
            mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U) {
            result.push_back({
                ScaffoldCandidateType::FlipTriangleEdge,
                edge.id,
                kInvalidIndex,
                0.5,
                0.0});
        }
    }
    std::stable_sort(
        result.begin(),
        result.end(),
        [](const Candidate& left, const Candidate& right) {
            if (left.priority != right.priority) {
                return left.priority > right.priority;
            }
            if (left.type != right.type) {
                return left.type < right.type;
            }
            if (left.edgeId != right.edgeId) {
                return left.edgeId < right.edgeId;
            }
            return left.endpointToKeep < right.endpointToKeep;
        });
    if (result.size() > settings.maximumCandidatesPerPass) {
        result.resize(settings.maximumCandidatesPerPass);
    }
    return result;
}

MutableOperationResult executeCandidate(
    LocalMutablePatchMesh& mesh,
    const Candidate& candidate)
{
    switch (candidate.type) {
    case ScaffoldCandidateType::CollapseEdgeToEndpoint:
        return mesh.collapseEdgeToEndpoint(
            candidate.edgeId, candidate.endpointToKeep);
    case ScaffoldCandidateType::SplitSourceLineageEdge:
        return mesh.splitEdge(candidate.edgeId, candidate.splitParameter);
    case ScaffoldCandidateType::FlipTriangleEdge:
        return mesh.flipTriangleEdge(candidate.edgeId);
    }
    return {};
}

double operationPenalty(ScaffoldCandidateType type) noexcept
{
    switch (type) {
    case ScaffoldCandidateType::CollapseEdgeToEndpoint: return 0.002;
    case ScaffoldCandidateType::SplitSourceLineageEdge: return 0.003;
    case ScaffoldCandidateType::FlipTriangleEdge: return 0.001;
    }
    return 0.0;
}

std::string operationDescription(
    const Candidate& candidate,
    const MutableOperationResult& operation)
{
    std::ostringstream stream;
    stream << scaffoldCandidateTypeName(candidate.type)
           << " edge " << candidate.edgeId;
    if (candidate.type ==
        ScaffoldCandidateType::CollapseEdgeToEndpoint) {
        stream << " -> keep vertex " << candidate.endpointToKeep;
    } else if (candidate.type ==
               ScaffoldCandidateType::SplitSourceLineageEdge) {
        stream << " at " << candidate.splitParameter;
    }
    stream << " (operation " << operation.changes.id << ")";
    return stream.str();
}

}  // namespace

const char* scaffoldAdaptationStatusName(
    ScaffoldAdaptationStatus status) noexcept
{
    switch (status) {
    case ScaffoldAdaptationStatus::Success: return "Success";
    case ScaffoldAdaptationStatus::Partial: return "Partial";
    case ScaffoldAdaptationStatus::Failure: return "Failure";
    }
    return "Unknown";
}

const char* scaffoldAdaptationStopReasonName(
    ScaffoldAdaptationStopReason reason) noexcept
{
    switch (reason) {
    case ScaffoldAdaptationStopReason::Converged: return "Converged";
    case ScaffoldAdaptationStopReason::NoImprovingCandidate:
        return "NoImprovingCandidate";
    case ScaffoldAdaptationStopReason::OperationBudgetReached:
        return "OperationBudgetReached";
    case ScaffoldAdaptationStopReason::SurfaceConstraintReached:
        return "SurfaceConstraintReached";
    case ScaffoldAdaptationStopReason::InterfaceMinimumReached:
        return "InterfaceMinimumReached";
    case ScaffoldAdaptationStopReason::UnsupportedSplit:
        return "UnsupportedSplit";
    case ScaffoldAdaptationStopReason::InvalidInput: return "InvalidInput";
    case ScaffoldAdaptationStopReason::ValidationFailure:
        return "ValidationFailure";
    }
    return "Unknown";
}

const char* scaffoldCandidateTypeName(ScaffoldCandidateType type) noexcept
{
    switch (type) {
    case ScaffoldCandidateType::CollapseEdgeToEndpoint:
        return "CollapseEdgeToEndpoint";
    case ScaffoldCandidateType::SplitSourceLineageEdge:
        return "SplitSourceLineageEdge";
    case ScaffoldCandidateType::FlipTriangleEdge:
        return "FlipTriangleEdge";
    }
    return "Unknown";
}

AdaptedScaffoldResult ScaffoldAdaptationSolver::adapt(
    const SourceTransitionScaffold& scaffold,
    const LocalMutablePatchMesh& mutablePatch,
    const SourceMeshSnapshot& sourceMesh,
    const RegionComponent& component,
    const std::vector<FaceDirection>& directionField,
    const std::vector<FaceDensity>& densityField,
    const RemeshSettings& remeshSettings,
    const ScaffoldAdaptationSettings& settings) const
{
    const auto start = std::chrono::steady_clock::now();
    AdaptedScaffoldResult result;
    result.adaptedMesh = mutablePatch;
    result.innerInterfaceBefore =
        mutablePatch.orderedInnerInterfaceVertices();
    const std::uint64_t immutableScaffoldSignature =
        sourceTransitionScaffoldSignature(scaffold);

    const auto finish = [&result, start]() {
        result.innerInterfaceAfter =
            result.adaptedMesh.orderedInnerInterfaceVertices();
        result.finalSignature = result.adaptedMesh.signature();
        result.adaptationMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
    };
    const auto fail = [&result, &finish](
                          ScaffoldAdaptationStopReason reason,
                          const std::string& message) {
        result.status = ScaffoldAdaptationStatus::Failure;
        result.stopReason = reason;
        result.diagnosticMessage = message;
        finish();
        return result;
    };

    std::string validation;
    if (!scaffold.success() ||
        scaffold.componentId != component.componentId ||
        !sourceMesh.valid(&validation) ||
        !mutablePatch.valid(&validation) ||
        directionField.size() < sourceMesh.faces.size() ||
        densityField.size() < sourceMesh.faces.size() ||
        settings.maxOperations == 0U ||
        settings.maxPasses == 0U ||
        settings.maximumCandidatesPerPass == 0U ||
        settings.minimumInterfaceVertexCount < 3U ||
        !std::isfinite(settings.minimumCostImprovement) ||
        settings.minimumCostImprovement < 0.0) {
        return fail(
            ScaffoldAdaptationStopReason::InvalidInput,
            validation.empty()
                ? "R6 adaptation input or settings are invalid."
                : validation);
    }

    EvaluationContext context(
        sourceMesh,
        component,
        directionField,
        densityField,
        remeshSettings,
        mutablePatch);
    result.before = evaluateMesh(mutablePatch, context);
    result.after = result.before;

    bool surfaceRejected = false;
    bool interfaceMinimumRejected = false;
    bool unsupportedSplitSeen = false;
    ScaffoldAdaptationStopReason stopReason =
        ScaffoldAdaptationStopReason::NoImprovingCandidate;

    for (unsigned int pass = 0U;
         pass < settings.maxPasses &&
         result.operations.size() < settings.maxOperations;
         ++pass) {
        ++result.passes;
        const std::vector<Candidate> candidates =
            generateCandidates(result.adaptedMesh, context, settings);
        result.candidates.generated += candidates.size();

        for (const MutableEdge& edge : result.adaptedMesh.edges()) {
            if (edge.deleted || edge.fixedOuterBoundary) { continue; }
            const double target =
                context.targetForEdge(edge, result.adaptedMesh);
            const double length =
                (result.adaptedMesh.vertices()[edge.vertex1].position -
                 result.adaptedMesh.vertices()[edge.vertex0].position).length();
            if (settings.enableSourceLineageSplit &&
                target > context.epsilon &&
                length > target * settings.splitLengthRatioThreshold &&
                edge.sourceEdgeId == kInvalidSourceId &&
                edge.parentSourceEdgeId == kInvalidSourceId) {
                unsupportedSplitSeen = true;
            }
        }

        bool foundBest = false;
        Candidate bestCandidate;
        MutableOperationResult bestOperation;
        LocalMutablePatchMesh bestMesh;
        ScaffoldAdaptationMetrics bestMetrics;
        double bestObjective = std::numeric_limits<double>::infinity();

        for (const Candidate& candidate : candidates) {
            if (candidate.type ==
                    ScaffoldCandidateType::CollapseEdgeToEndpoint &&
                !collapseLinkCondition(
                    result.adaptedMesh, candidate, settings)) {
                ++result.candidates.rejectedByLinkCondition;
                if (candidate.edgeId < result.adaptedMesh.edges().size() &&
                    result.adaptedMesh.edges()[candidate.edgeId].innerInterface &&
                    result.adaptedMesh.orderedInnerInterfaceVertices().size() <=
                        settings.minimumInterfaceVertexCount) {
                    interfaceMinimumRejected = true;
                }
                continue;
            }

            LocalMutablePatchMesh candidateMesh = result.adaptedMesh;
            ++result.candidates.simulated;
            MutableOperationResult operation =
                executeCandidate(candidateMesh, candidate);
            if (!operation.success() ||
                !candidateMesh.valid(&validation) ||
                !affectedVertexLinksValid(
                    candidateMesh, operation.changes) ||
                !affectedFaceOrientationValid(
                    candidateMesh, operation.changes, context) ||
                candidateMesh.diagnostics().maximumFixedBoundaryDisplacement !=
                    0.0) {
                ++result.candidates.rejectedByTopology;
                continue;
            }
            ++result.candidates.valid;
            const ScaffoldAdaptationMetrics metrics =
                evaluateMesh(candidateMesh, context);
            if (!surfaceConstraintSatisfied(
                    result.after,
                    metrics,
                    candidateMesh,
                    context,
                    settings)) {
                ++result.candidates.rejectedBySurface;
                surfaceRejected = true;
                continue;
            }
            const double objective =
                metrics.cost.total + operationPenalty(candidate.type);
            const double improvement =
                result.after.cost.total - objective;
            if (!(improvement > settings.minimumCostImprovement)) {
                ++result.candidates.rejectedWithoutImprovement;
                continue;
            }
            if (!foundBest ||
                objective < bestObjective -
                    std::numeric_limits<double>::epsilon()) {
                foundBest = true;
                bestCandidate = candidate;
                bestOperation = operation;
                bestMesh = std::move(candidateMesh);
                bestMetrics = metrics;
                bestObjective = objective;
            }
        }

        if (!foundBest) {
            if (surfaceRejected) {
                stopReason =
                    ScaffoldAdaptationStopReason::SurfaceConstraintReached;
            } else if (interfaceMinimumRejected) {
                stopReason =
                    ScaffoldAdaptationStopReason::InterfaceMinimumReached;
            } else if (unsupportedSplitSeen) {
                stopReason =
                    ScaffoldAdaptationStopReason::UnsupportedSplit;
            } else {
                stopReason =
                    ScaffoldAdaptationStopReason::NoImprovingCandidate;
            }
            break;
        }

        ScaffoldAdaptationOperation selected;
        selected.type = bestCandidate.type;
        selected.edgeId = bestCandidate.edgeId;
        selected.endpointToKeep = bestCandidate.endpointToKeep;
        selected.splitParameter = bestCandidate.splitParameter;
        selected.costBefore = result.after.cost.total;
        selected.costAfter = bestMetrics.cost.total;
        selected.lineage = bestOperation.changes;
        selected.description =
            operationDescription(bestCandidate, bestOperation);
        result.operations.push_back(std::move(selected));
        result.adaptedMesh = std::move(bestMesh);
        result.after = bestMetrics;
    }

    if (result.operations.size() >= settings.maxOperations ||
        result.passes >= settings.maxPasses) {
        stopReason =
            ScaffoldAdaptationStopReason::OperationBudgetReached;
    } else if (result.after.meanDensityError <= 0.25 &&
               result.after.meanDirectionDeviationDegrees <=
                   result.before.meanDirectionDeviationDegrees + 1.0e-9) {
        stopReason = ScaffoldAdaptationStopReason::Converged;
    }

    if (!result.adaptedMesh.valid(&validation) ||
        result.adaptedMesh.diagnostics().maximumFixedBoundaryDisplacement !=
            0.0 ||
        sourceTransitionScaffoldSignature(scaffold) !=
            immutableScaffoldSignature) {
        return fail(
            ScaffoldAdaptationStopReason::ValidationFailure,
            validation.empty()
                ? "R6 final hard-invariant validation failed."
                : validation);
    }

    result.stopReason = stopReason;
    const bool improved =
        result.after.cost.total + settings.minimumCostImprovement <
            result.before.cost.total;
    const bool targetAcceptable = result.after.meanDensityError <= 0.25;
    result.status =
        stopReason == ScaffoldAdaptationStopReason::OperationBudgetReached ||
        (!improved && !targetAcceptable) ||
        stopReason == ScaffoldAdaptationStopReason::UnsupportedSplit
            ? ScaffoldAdaptationStatus::Partial
            : ScaffoldAdaptationStatus::Success;

    std::ostringstream diagnostic;
    diagnostic
        << "R6 experimental scaffold adaptation "
        << scaffoldAdaptationStatusName(result.status)
        << "; stop=" << scaffoldAdaptationStopReasonName(result.stopReason)
        << "; operations=" << result.operations.size()
        << "; interface=" << result.before.innerInterfaceVertexCount
        << "->" << result.after.innerInterfaceVertexCount
        << " (desired~"
        << result.after.approximateDesiredInterfaceCount << ")"
        << "; density=" << result.before.meanDensityError
        << "->" << result.after.meanDensityError
        << "; directionDeg="
        << result.before.meanDirectionDeviationDegrees
        << "->" << result.after.meanDirectionDeviationDegrees
        << "; fixedBoundaryDisplacement="
        << result.after.maximumFixedBoundaryDisplacement
        << ". Production solver path was not used.";
    result.diagnosticMessage = diagnostic.str();
    finish();
    return result;
}

}  // namespace directional_retopo::solver
