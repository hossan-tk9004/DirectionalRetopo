#include "Solver/CoreInterfaceJoinSolver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace directional_retopo::solver {
namespace {

using Clock = std::chrono::steady_clock;
using EdgeKey = std::pair<std::size_t, std::size_t>;
constexpr double kPi = 3.14159265358979323846;

double elapsedMilliseconds(Clock::time_point start) noexcept
{
    return std::chrono::duration<double, std::milli>(
        Clock::now() - start).count();
}

EdgeKey edgeKey(std::size_t first, std::size_t second) noexcept
{
    return first < second ? EdgeKey(first, second) : EdgeKey(second, first);
}

std::size_t absoluteDifference(
    std::size_t first,
    std::size_t second) noexcept
{
    return first > second ? first - second : second - first;
}

double countRatio(std::size_t first, std::size_t second) noexcept
{
    if (first == 0U || second == 0U) {
        return std::numeric_limits<double>::infinity();
    }
    return static_cast<double>(std::max(first, second)) /
        static_cast<double>(std::min(first, second));
}

Vec3 polygonNormal(
    const std::vector<Vec3>& points,
    double epsilon) noexcept
{
    Vec3 normal;
    for (std::size_t index = 0U; index < points.size(); ++index) {
        const Vec3& current = points[index];
        const Vec3& next = points[(index + 1U) % points.size()];
        normal.x += (current.y - next.y) * (current.z + next.z);
        normal.y += (current.z - next.z) * (current.x + next.x);
        normal.z += (current.x - next.x) * (current.y + next.y);
    }
    return normal.normalized(epsilon);
}

double polygonArea(
    const std::vector<Vec3>& points,
    double epsilon) noexcept
{
    if (points.size() < 3U) {
        return 0.0;
    }
    const Vec3 normal = polygonNormal(points, epsilon);
    if (normal.squaredLength() <= epsilon * epsilon) {
        return 0.0;
    }
    double twiceArea = 0.0;
    for (std::size_t index = 1U; index + 1U < points.size(); ++index) {
        twiceArea += std::abs(
            (points[index] - points[0])
                .cross(points[index + 1U] - points[0])
                .dot(normal));
    }
    return 0.5 * twiceArea;
}

Vec3 closestPointBarycentric(
    const Vec3& point,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    Vec3& barycentric) noexcept
{
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = point - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        barycentric = {1.0, 0.0, 0.0};
        return a;
    }
    const Vec3 bp = point - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) {
        barycentric = {0.0, 1.0, 0.0};
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        barycentric = {1.0 - v, v, 0.0};
        return a + ab * v;
    }
    const Vec3 cp = point - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) {
        barycentric = {0.0, 0.0, 1.0};
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        barycentric = {1.0 - w, 0.0, w};
        return a + ac * w;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 &&
        (d5 - d6) >= 0.0) {
        const double w =
            (d4 - d3) / ((d4 - d3) + (d5 - d6));
        barycentric = {0.0, 1.0 - w, w};
        return b + (c - b) * w;
    }
    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    barycentric = {1.0 - v - w, v, w};
    return a + ab * v + ac * w;
}

std::size_t sourceFaceIndex(
    const SourceMeshSnapshot& source,
    SourceId sourceFaceId) noexcept
{
    for (std::size_t index = 0U; index < source.faces.size(); ++index) {
        if (source.faces[index].sourceFaceId == sourceFaceId) {
            return index;
        }
    }
    return kInvalidIndex;
}

SurfacePointMapping mapToSource(
    const RemeshInput& input,
    const Vec3& position,
    const std::vector<SourceId>& sourceFaceIds,
    double epsilon)
{
    std::vector<std::size_t> triangles;
    for (const SourceId sourceFaceId : sourceFaceIds) {
        const std::size_t faceIndex =
            sourceFaceIndex(input.sourceMesh, sourceFaceId);
        if (faceIndex == kInvalidIndex) {
            continue;
        }
        const SourceFace& face = input.sourceMesh.faces[faceIndex];
        triangles.insert(
            triangles.end(),
            face.triangleIndices.begin(),
            face.triangleIndices.end());
        for (const std::size_t adjacent : face.adjacentFaceIndices) {
            const auto& adjacentTriangles =
                input.sourceMesh.faces[adjacent].triangleIndices;
            triangles.insert(
                triangles.end(),
                adjacentTriangles.begin(),
                adjacentTriangles.end());
        }
    }
    std::sort(triangles.begin(), triangles.end());
    triangles.erase(
        std::unique(triangles.begin(), triangles.end()), triangles.end());
    SurfacePointMapping best;
    for (const std::size_t triangleIndex : triangles) {
        if (triangleIndex >= input.sourceMesh.triangles.size()) {
            continue;
        }
        const SourceTriangle& triangle =
            input.sourceMesh.triangles[triangleIndex];
        const Vec3 a =
            input.sourceMesh.vertices[triangle.vertexIndices[0]].position;
        const Vec3 b =
            input.sourceMesh.vertices[triangle.vertexIndices[1]].position;
        const Vec3 c =
            input.sourceMesh.vertices[triangle.vertexIndices[2]].position;
        Vec3 barycentric;
        const Vec3 closest =
            closestPointBarycentric(position, a, b, c, barycentric);
        const double distance = (closest - position).length();
        const Vec3 normal = (b - a).cross(c - a).normalized(epsilon);
        if (normal.squaredLength() <= epsilon * epsilon) {
            continue;
        }
        if (!best.valid || distance < best.surfaceDistance) {
            best.sourceTriangleIndex = triangleIndex;
            best.sourceFaceId =
                input.sourceMesh.faces[triangle.faceIndex].sourceFaceId;
            best.barycentric = barycentric;
            best.sourcePosition = closest;
            best.sourceNormal = normal;
            best.surfaceDistance = distance;
            best.valid = true;
        }
    }
    return best;
}

std::vector<SourceId> vertexSourceFaces(
    const LocalMutablePatchMesh& mesh,
    const MutableVertex& vertex)
{
    std::vector<SourceId> result;
    for (const MutableFaceId faceId : vertex.faceIds) {
        if (faceId >= mesh.faces().size() ||
            mesh.faces()[faceId].deleted) {
            continue;
        }
        result.insert(
            result.end(),
            mesh.faces()[faceId].sourceFaceIds.begin(),
            mesh.faces()[faceId].sourceFaceIds.end());
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

AdaptedInterfaceDescriptor describeInterface(
    const RemeshInput& input,
    const LocalMutablePatchMesh& mesh,
    double epsilon)
{
    AdaptedInterfaceDescriptor result;
    result.orderedVertexIds = mesh.orderedInnerInterfaceVertices();
    result.orderedEdgeIds = mesh.orderedInnerInterfaceEdges();
    result.closed = true;
    if (result.orderedVertexIds.size() < 3U ||
        result.orderedEdgeIds.size() != result.orderedVertexIds.size()) {
        result.status = InterfaceDescriptorStatus::Missing;
        result.diagnosticMessage =
            "Adapted Scaffold has no valid closed Inner Interface.";
        return result;
    }
    std::vector<double> cumulative(result.orderedVertexIds.size(), 0.0);
    for (std::size_t index = 0U;
         index < result.orderedVertexIds.size();
         ++index) {
        const MutableVertexId vertexId = result.orderedVertexIds[index];
        const MutableVertexId nextId =
            result.orderedVertexIds[
                (index + 1U) % result.orderedVertexIds.size()];
        const MutableEdgeId edgeId = result.orderedEdgeIds[index];
        if (vertexId >= mesh.vertices().size() ||
            nextId >= mesh.vertices().size() ||
            edgeId >= mesh.edges().size()) {
            result.status = InterfaceDescriptorStatus::InvalidGeometry;
            result.diagnosticMessage =
                "Adapted Interface contains an invalid element ID.";
            return result;
        }
        const MutableVertex& vertex = mesh.vertices()[vertexId];
        const MutableEdge& edge = mesh.edges()[edgeId];
        if (vertex.deleted || !vertex.innerInterface ||
            vertex.fixedOuterBoundary || edge.deleted ||
            !edge.innerInterface || edge.fixedOuterBoundary) {
            result.status = InterfaceDescriptorStatus::InvalidGeometry;
            result.diagnosticMessage =
                "Adapted Interface classification is inconsistent.";
            return result;
        }
        const double length =
            (mesh.vertices()[nextId].position - vertex.position).length();
        if (!std::isfinite(length) || length <= epsilon) {
            result.status = InterfaceDescriptorStatus::InvalidGeometry;
            result.diagnosticMessage =
                "Adapted Interface contains a zero-length edge.";
            return result;
        }
        result.totalArcLength += length;
        const std::size_t next =
            (index + 1U) % result.orderedVertexIds.size();
        if (next != 0U) {
            cumulative[next] = result.totalArcLength;
        }
    }
    if (!(result.totalArcLength > epsilon)) {
        result.status = InterfaceDescriptorStatus::InvalidGeometry;
        result.diagnosticMessage =
            "Adapted Interface has zero arc length.";
        return result;
    }
    for (std::size_t index = 0U;
         index < result.orderedVertexIds.size();
         ++index) {
        const std::size_t previous =
            (index + result.orderedVertexIds.size() - 1U) %
            result.orderedVertexIds.size();
        const std::size_t next =
            (index + 1U) % result.orderedVertexIds.size();
        const MutableVertex& vertex =
            mesh.vertices()[result.orderedVertexIds[index]];
        AdaptedInterfaceVertexDescriptor descriptor;
        descriptor.mutableVertexId = vertex.id;
        descriptor.orderedBoundaryIndex = index;
        descriptor.position = vertex.position;
        descriptor.normal = vertex.normal.normalized(epsilon);
        descriptor.tangent =
            (mesh.vertices()[result.orderedVertexIds[next]].position -
             mesh.vertices()[result.orderedVertexIds[previous]].position)
                .normalized(epsilon);
        descriptor.normalizedArcLength =
            cumulative[index] / result.totalArcLength;
        descriptor.fixedOuterBoundary = vertex.fixedOuterBoundary;
        descriptor.minimumRingDepth = std::numeric_limits<int>::max();
        const std::vector<SourceId> provenance =
            vertexSourceFaces(mesh, vertex);
        for (const MutableFaceId faceId : vertex.faceIds) {
            if (faceId < mesh.faces().size() &&
                !mesh.faces()[faceId].deleted &&
                mesh.faces()[faceId].minimumRingDepth >= 0) {
                descriptor.minimumRingDepth = std::min(
                    descriptor.minimumRingDepth,
                    mesh.faces()[faceId].minimumRingDepth);
            }
        }
        if (descriptor.minimumRingDepth ==
            std::numeric_limits<int>::max()) {
            descriptor.minimumRingDepth = -1;
        }
        descriptor.surface = mapToSource(
            input, descriptor.position, provenance, epsilon);
        if (!descriptor.surface.valid ||
            descriptor.tangent.squaredLength() <= epsilon * epsilon ||
            descriptor.normal.squaredLength() <= epsilon * epsilon) {
            result.status =
                InterfaceDescriptorStatus::SurfaceMappingFailed;
            result.diagnosticMessage =
                "Adapted Interface local Source mapping failed.";
            return result;
        }
        result.vertices.push_back(descriptor);
    }
    result.status = InterfaceDescriptorStatus::Success;
    result.diagnosticMessage =
        "Ordered Adapted Inner Interface descriptor built.";
    return result;
}

InterfaceReconciliationResult reconcileInterface(
    const RemeshInput& input,
    const AdaptedScaffoldResult& adaptation,
    const CoreRemeshResult& core,
    const CoreInterfaceJoinSettings& settings)
{
    InterfaceReconciliationResult result;
    result.mesh = adaptation.adaptedMesh;
    result.interface = describeInterface(
        input, result.mesh, settings.geometryEpsilon);
    if (!result.interface.success() || !core.boundary.success()) {
        result.status = ReconciliationStatus::InvalidInput;
        result.diagnosticMessage =
            "Interface reconciliation received an invalid loop.";
        return result;
    }
    result.scaffoldCountBefore = result.interface.vertices.size();
    result.coreCount = core.boundary.vertices.size();
    result.absoluteDifferenceBefore = absoluteDifference(
        result.scaffoldCountBefore, result.coreCount);
    result.countRatioBefore =
        countRatio(result.scaffoldCountBefore, result.coreCount);
    result.parityMatchedBefore =
        result.scaffoldCountBefore % 2U == result.coreCount % 2U;

    for (std::size_t operation = 0U;
         operation < settings.maximumReconciliationOperations;
         ++operation) {
        const std::size_t scaffoldCount =
            result.mesh.orderedInnerInterfaceVertices().size();
        if (absoluteDifference(scaffoldCount, result.coreCount) <=
            settings.residualTriangleBudget) {
            break;
        }
        bool applied = false;
        if (scaffoldCount > result.coreCount) {
            struct CollapseCandidate final
            {
                double length = 0.0;
                MutableEdgeId edgeId = kInvalidIndex;
                MutableVertexId endpoint = kInvalidIndex;
            };
            std::vector<CollapseCandidate> candidates;
            for (const MutableEdgeId edgeId :
                 result.mesh.orderedInnerInterfaceEdges()) {
                if (edgeId >= result.mesh.edges().size()) {
                    continue;
                }
                const MutableEdge& edge = result.mesh.edges()[edgeId];
                if (edge.deleted || edge.fixedOuterBoundary) {
                    continue;
                }
                const double length =
                    (result.mesh.vertices()[edge.vertex1].position -
                     result.mesh.vertices()[edge.vertex0].position).length();
                candidates.push_back({length, edgeId, edge.vertex0});
                candidates.push_back({length, edgeId, edge.vertex1});
            }
            std::sort(
                candidates.begin(), candidates.end(),
                [](const CollapseCandidate& left,
                   const CollapseCandidate& right) {
                    return std::tie(
                               left.length, left.edgeId, left.endpoint) <
                        std::tie(
                               right.length, right.edgeId, right.endpoint);
                });
            for (const CollapseCandidate& candidate : candidates) {
                LocalMutablePatchMesh trial = result.mesh;
                MutableOperationResult operationResult =
                    trial.collapseEdgeToEndpoint(
                        candidate.edgeId, candidate.endpoint);
                if (!operationResult.success()) {
                    continue;
                }
                const AdaptedInterfaceDescriptor descriptor =
                    describeInterface(
                        input, trial, settings.geometryEpsilon);
                if (!descriptor.success()) {
                    continue;
                }
                result.mesh = std::move(trial);
                result.interface = descriptor;
                result.operations.push_back(operationResult.changes);
                ++result.collapseCount;
                applied = true;
                break;
            }
        } else {
            struct SplitCandidate final
            {
                double length = 0.0;
                MutableEdgeId edgeId = kInvalidIndex;
            };
            std::vector<SplitCandidate> candidates;
            for (const MutableEdgeId edgeId :
                 result.mesh.orderedInnerInterfaceEdges()) {
                if (edgeId >= result.mesh.edges().size()) {
                    continue;
                }
                const MutableEdge& edge = result.mesh.edges()[edgeId];
                if (edge.deleted || edge.fixedOuterBoundary) {
                    continue;
                }
                const double length =
                    (result.mesh.vertices()[edge.vertex1].position -
                     result.mesh.vertices()[edge.vertex0].position).length();
                candidates.push_back({length, edgeId});
            }
            std::sort(
                candidates.begin(), candidates.end(),
                [](const SplitCandidate& left,
                   const SplitCandidate& right) {
                    if (left.length != right.length) {
                        return left.length > right.length;
                    }
                    return left.edgeId < right.edgeId;
                });
            for (const SplitCandidate& candidate : candidates) {
                LocalMutablePatchMesh trial = result.mesh;
                MutableOperationResult operationResult =
                    trial.splitEdge(candidate.edgeId, 0.5);
                if (!operationResult.success()) {
                    continue;
                }
                const AdaptedInterfaceDescriptor descriptor =
                    describeInterface(
                        input, trial, settings.geometryEpsilon);
                if (!descriptor.success()) {
                    continue;
                }
                result.mesh = std::move(trial);
                result.interface = descriptor;
                result.operations.push_back(operationResult.changes);
                ++result.splitCount;
                applied = true;
                break;
            }
        }
        if (!applied) {
            break;
        }
    }
    result.interface = describeInterface(
        input, result.mesh, settings.geometryEpsilon);
    result.scaffoldCountAfter = result.interface.vertices.size();
    result.absoluteDifferenceAfter = absoluteDifference(
        result.scaffoldCountAfter, result.coreCount);
    const double residualFraction =
        static_cast<double>(result.absoluteDifferenceAfter) /
        static_cast<double>(std::max(
            result.scaffoldCountAfter, result.coreCount));
    result.countRatioAfter =
        countRatio(result.scaffoldCountAfter, result.coreCount);
    result.parityMatchedAfter =
        result.scaffoldCountAfter % 2U == result.coreCount % 2U;
    if (!result.interface.success()) {
        result.status = ReconciliationStatus::OperationFailed;
        result.diagnosticMessage =
            "Reconciled Interface is invalid.";
    } else if (result.countRatioAfter >
               settings.maximumJoinCountRatio) {
        result.status = ReconciliationStatus::ExcessiveMismatch;
        result.diagnosticMessage =
            "Interface count mismatch remains excessive after local reconciliation.";
    } else if (result.absoluteDifferenceAfter >
                   settings.maximumResidualTriangleCount ||
               residualFraction >
                   settings.maximumResidualTriangleFraction) {
        result.status = ReconciliationStatus::ExcessiveMismatch;
        result.diagnosticMessage =
            "Residual mismatch is too large for one-row Triangle termination.";
    } else if (result.absoluteDifferenceAfter <=
               settings.residualTriangleBudget) {
        result.status = ReconciliationStatus::Success;
        result.diagnosticMessage =
            "Interface count reconciled to the Triangle termination budget.";
    } else {
        result.status = ReconciliationStatus::Partial;
        result.diagnosticMessage =
            "Interface reconciliation stopped safely before exact target.";
    }
    return result;
}

std::vector<std::size_t> seamCandidates(
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core,
    std::size_t maximum)
{
    std::set<std::size_t> candidates;
    const std::size_t count = core.vertices.size();
    if (count <= maximum) {
        for (std::size_t index = 0U; index < count; ++index) {
            candidates.insert(index);
        }
    } else {
        for (std::size_t item = 0U; item < maximum; ++item) {
            candidates.insert(item * count / maximum);
        }
    }
    if (!scaffold.vertices.empty()) {
        std::vector<std::pair<double, std::size_t>> nearest;
        for (std::size_t index = 0U; index < core.vertices.size(); ++index) {
            nearest.push_back({
                (core.vertices[index].position -
                 scaffold.vertices.front().position).squaredLength(),
                index});
        }
        std::sort(nearest.begin(), nearest.end());
        for (std::size_t index = 0U;
             index < std::min<std::size_t>(4U, nearest.size());
             ++index) {
            candidates.insert(nearest[index].second);
        }
    }
    std::vector<std::size_t> ranked(candidates.begin(), candidates.end());
    if (!scaffold.vertices.empty()) {
        const Vec3 anchor = scaffold.vertices.front().position;
        std::sort(
            ranked.begin(), ranked.end(),
            [&core, &anchor](std::size_t first, std::size_t second) {
                const double firstDistance =
                    (core.vertices[first].position - anchor).squaredLength();
                const double secondDistance =
                    (core.vertices[second].position - anchor).squaredLength();
                return std::tie(firstDistance, first) <
                    std::tie(secondDistance, second);
            });
    }
    ranked.resize(std::min(maximum, ranked.size()));
    return ranked;
}

std::size_t coreOrderIndex(
    std::size_t logicalIndex,
    std::size_t count,
    std::size_t seam,
    bool reversed) noexcept
{
    return reversed
        ? (seam + count - logicalIndex % count) % count
        : (seam + logicalIndex) % count;
}

InterfaceCorrespondence buildCorrespondence(
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core,
    std::size_t seam,
    bool reversed,
    double scale)
{
    InterfaceCorrespondence result;
    result.coreOrderReversed = reversed;
    result.coreSeamOffset = seam;
    result.totalCost = 0.0;
    const std::size_t scaffoldCount = scaffold.vertices.size();
    const std::size_t coreCount = core.vertices.size();
    for (std::size_t index = 0U; index < scaffoldCount; ++index) {
        const double normalized =
            static_cast<double>(index) /
            static_cast<double>(scaffoldCount);
        const std::size_t logicalCore = std::min(
            coreCount - 1U,
            static_cast<std::size_t>(std::llround(
                normalized * static_cast<double>(coreCount))) %
                coreCount);
        const std::size_t actualCore = coreOrderIndex(
            logicalCore, coreCount, seam, reversed);
        const AdaptedInterfaceVertexDescriptor& left =
            scaffold.vertices[index];
        const CoreBoundaryVertexDescriptor& right =
            core.vertices[actualCore];
        InterfaceCorrespondenceEntry entry;
        entry.scaffoldOrderIndex = index;
        entry.coreOrderIndex = actualCore;
        entry.scaffoldNormalizedArcLength =
            left.normalizedArcLength;
        entry.coreNormalizedArcLength =
            static_cast<double>(logicalCore) /
            static_cast<double>(coreCount);
        entry.positionDistance =
            (left.surface.sourcePosition -
             right.surface.sourcePosition).length();
        entry.tangentDeviation =
            1.0 - std::abs(std::clamp(
                left.tangent.dot(right.tangent), -1.0, 1.0));
        entry.normalDeviation =
            1.0 - std::clamp(
                left.surface.sourceNormal.dot(
                    right.surface.sourceNormal),
                -1.0, 1.0);
        const double sourcePenalty =
            left.surface.sourceFaceId == right.surface.sourceFaceId
            ? 0.0 : 0.15;
        const double arcPenalty = std::abs(
            normalized - entry.coreNormalizedArcLength);
        result.totalCost +=
            entry.positionDistance / std::max(scale, 1.0e-12) +
            0.5 * entry.tangentDeviation +
            0.5 * entry.normalDeviation +
            0.25 * arcPenalty + sourcePenalty;
        result.entries.push_back(entry);
    }
    result.success = true;
    result.diagnosticMessage =
        "Monotonic cyclic Interface correspondence initialized.";
    return result;
}

double targetLength(
    const RemeshInput& input,
    const std::vector<SurfacePointMapping>& mappings)
{
    std::vector<double> values;
    for (const SurfacePointMapping& mapping : mappings) {
        const std::size_t face =
            sourceFaceIndex(input.sourceMesh, mapping.sourceFaceId);
        if (face < input.densityField.size() &&
            input.densityField[face].valid) {
            const double value =
                input.densityField[face].effectiveTargetEdgeLength;
            if (std::isfinite(value) && value > 0.0) {
                values.push_back(value);
            }
        }
    }
    if (values.empty()) {
        return 1.0;
    }
    return std::accumulate(values.begin(), values.end(), 0.0) /
        static_cast<double>(values.size());
}

struct SegmentDistance final
{
    double distance = std::numeric_limits<double>::infinity();
    double firstParameter = 0.0;
    double secondParameter = 0.0;
};

SegmentDistance segmentDistance(
    const Vec3& p0,
    const Vec3& p1,
    const Vec3& q0,
    const Vec3& q1,
    double epsilon) noexcept
{
    const Vec3 u = p1 - p0;
    const Vec3 v = q1 - q0;
    const Vec3 w = p0 - q0;
    const double a = u.dot(u);
    const double b = u.dot(v);
    const double c = v.dot(v);
    const double d = u.dot(w);
    const double e = v.dot(w);
    const double denominator = a * c - b * b;
    double s = denominator > epsilon
        ? std::clamp((b * e - c * d) / denominator, 0.0, 1.0)
        : 0.0;
    double t = c > epsilon
        ? std::clamp((b * s + e) / c, 0.0, 1.0)
        : 0.0;
    if (a > epsilon) {
        s = std::clamp((b * t - d) / a, 0.0, 1.0);
    }
    if (c > epsilon) {
        t = std::clamp((b * s + e) / c, 0.0, 1.0);
    }
    return {
        (p0 + u * s - q0 - v * t).length(),
        s,
        t};
}

const Vec3& joinPosition(
    const JoinVertexReference& reference,
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core)
{
    if (reference.domain == JoinVertexDomain::Scaffold) {
        const auto found = std::find_if(
            scaffold.vertices.begin(), scaffold.vertices.end(),
            [&reference](const AdaptedInterfaceVertexDescriptor& vertex) {
                return vertex.mutableVertexId == reference.vertexId;
            });
        return found->position;
    }
    const auto found = std::find_if(
        core.vertices.begin(), core.vertices.end(),
        [&reference](const CoreBoundaryVertexDescriptor& vertex) {
            return vertex.coreVertexId == reference.vertexId;
        });
    return found->position;
}

SurfacePointMapping joinMapping(
    const JoinVertexReference& reference,
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core)
{
    if (reference.domain == JoinVertexDomain::Scaffold) {
        const auto found = std::find_if(
            scaffold.vertices.begin(), scaffold.vertices.end(),
            [&reference](const AdaptedInterfaceVertexDescriptor& vertex) {
                return vertex.mutableVertexId == reference.vertexId;
            });
        return found->surface;
    }
    const auto found = std::find_if(
        core.vertices.begin(), core.vertices.end(),
        [&reference](const CoreBoundaryVertexDescriptor& vertex) {
            return vertex.coreVertexId == reference.vertexId;
        });
    return found->surface;
}

double distanceToLocalSurface(
    const RemeshInput& input,
    const Vec3& point,
    const std::vector<SurfacePointMapping>& mappings)
{
    std::set<std::size_t> triangles;
    for (const SurfacePointMapping& mapping : mappings) {
        if (mapping.sourceTriangleIndex == kInvalidIndex ||
            mapping.sourceTriangleIndex >= input.sourceMesh.triangles.size()) {
            continue;
        }
        triangles.insert(mapping.sourceTriangleIndex);
        const std::size_t face =
            input.sourceMesh.triangles[mapping.sourceTriangleIndex].faceIndex;
        if (face >= input.sourceMesh.faces.size()) {
            continue;
        }
        for (const std::size_t triangle :
             input.sourceMesh.faces[face].triangleIndices) {
            triangles.insert(triangle);
        }
        for (const std::size_t adjacent :
             input.sourceMesh.faces[face].adjacentFaceIndices) {
            for (const std::size_t triangle :
                 input.sourceMesh.faces[adjacent].triangleIndices) {
                triangles.insert(triangle);
            }
        }
    }
    double best = std::numeric_limits<double>::infinity();
    for (const std::size_t triangleId : triangles) {
        const SourceTriangle& triangle =
            input.sourceMesh.triangles[triangleId];
        Vec3 barycentric;
        const Vec3 closest = closestPointBarycentric(
            point,
            input.sourceMesh.vertices[triangle.vertexIndices[0]].position,
            input.sourceMesh.vertices[triangle.vertexIndices[1]].position,
            input.sourceMesh.vertices[triangle.vertexIndices[2]].position,
            barycentric);
        best = std::min(best, (closest - point).length());
    }
    return best;
}

double directionDeviationDegrees(
    const RemeshInput& input,
    const Vec3& edge,
    const std::vector<SurfacePointMapping>& mappings,
    double epsilon)
{
    const Vec3 direction = edge.normalized(epsilon);
    if (direction.squaredLength() <= epsilon * epsilon) {
        return 90.0;
    }
    double sum = 0.0;
    std::size_t count = 0U;
    for (const SurfacePointMapping& mapping : mappings) {
        const std::size_t face =
            sourceFaceIndex(input.sourceMesh, mapping.sourceFaceId);
        if (face >= input.directionField.size() ||
            !input.directionField[face].valid) {
            continue;
        }
        const FaceDirection& field = input.directionField[face];
        const double alignment = std::max(
            std::abs(direction.dot(field.uDirection.normalized(epsilon))),
            std::abs(direction.dot(field.vDirection.normalized(epsilon))));
        sum += std::acos(std::clamp(alignment, 0.0, 1.0)) *
            180.0 / kPi;
        ++count;
    }
    return count == 0U ? 0.0 : sum / static_cast<double>(count);
}

struct FaceEvaluation final
{
    bool valid = false;
    InterfaceJoinFace face;
    double surfaceError = 0.0;
    double directionErrorDegrees = 0.0;
};

FaceEvaluation evaluateJoinFace(
    const RemeshInput& input,
    std::vector<JoinVertexReference> references,
    JoinTriangleReason triangleReason,
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core,
    const CoreInterfaceJoinSettings& settings)
{
    FaceEvaluation result;
    if (references.size() < 3U || references.size() > 4U) {
        return result;
    }
    std::set<std::pair<int, std::size_t>> unique;
    std::vector<Vec3> points;
    std::vector<SurfacePointMapping> mappings;
    for (const JoinVertexReference& reference : references) {
        const auto key = std::make_pair(
            static_cast<int>(reference.domain), reference.vertexId);
        if (!unique.insert(key).second) {
            return result;
        }
        const Vec3& point =
            joinPosition(reference, scaffold, core);
        const SurfacePointMapping mapping =
            joinMapping(reference, scaffold, core);
        if (!point.finite() || !mapping.valid) {
            return result;
        }
        points.push_back(point);
        mappings.push_back(mapping);
    }
    const double target = targetLength(input, mappings);
    double minimumEdge = std::numeric_limits<double>::infinity();
    double maximumEdge = 0.0;
    for (std::size_t index = 0U; index < points.size(); ++index) {
        const double length =
            (points[(index + 1U) % points.size()] - points[index]).length();
        if (!std::isfinite(length) ||
            length <= settings.geometryEpsilon) {
            return result;
        }
        minimumEdge = std::min(minimumEdge, length);
        maximumEdge = std::max(maximumEdge, length);
    }
    const double area = polygonArea(points, settings.geometryEpsilon);
    const double areaTolerance = std::max(
        settings.geometryEpsilon,
        settings.relativeAreaEpsilon * target * target);
    if (!std::isfinite(area) || area <= areaTolerance) {
        return result;
    }
    if (references.size() == 4U) {
        const SegmentDistance crossing = segmentDistance(
            points[0], points[1], points[2], points[3],
            settings.geometryEpsilon);
        const SegmentDistance crossingOther = segmentDistance(
            points[1], points[2], points[3], points[0],
            settings.geometryEpsilon);
        const double crossingTolerance =
            settings.geometryEpsilon + target * 1.0e-8;
        const auto interior = [](const SegmentDistance& value) {
            return value.firstParameter > 1.0e-6 &&
                value.firstParameter < 1.0 - 1.0e-6 &&
                value.secondParameter > 1.0e-6 &&
                value.secondParameter < 1.0 - 1.0e-6;
        };
        if ((crossing.distance <= crossingTolerance &&
             interior(crossing)) ||
            (crossingOther.distance <= crossingTolerance &&
             interior(crossingOther))) {
            return result;
        }
    }
    Vec3 sourceNormal;
    for (const SurfacePointMapping& mapping : mappings) {
        sourceNormal += mapping.sourceNormal;
    }
    sourceNormal = sourceNormal.normalized(settings.geometryEpsilon);
    Vec3 normal = polygonNormal(points, settings.geometryEpsilon);
    if (normal.dot(sourceNormal) < 0.0) {
        std::reverse(references.begin(), references.end());
        std::reverse(points.begin(), points.end());
        std::reverse(mappings.begin(), mappings.end());
        normal = polygonNormal(points, settings.geometryEpsilon);
    }
    if (normal.dot(sourceNormal) <= 1.0e-6) {
        return result;
    }

    Vec3 center;
    for (const Vec3& point : points) {
        center += point;
    }
    center = center / static_cast<double>(points.size());
    double surfaceMaximum =
        distanceToLocalSurface(input, center, mappings);
    double directionSum = 0.0;
    for (std::size_t index = 0U; index < points.size(); ++index) {
        const Vec3 midpoint =
            (points[index] +
             points[(index + 1U) % points.size()]) * 0.5;
        surfaceMaximum = std::max(
            surfaceMaximum,
            distanceToLocalSurface(input, midpoint, mappings));
        directionSum += directionDeviationDegrees(
            input,
            points[(index + 1U) % points.size()] - points[index],
            mappings,
            settings.geometryEpsilon);
    }
    if (!std::isfinite(surfaceMaximum) ||
        surfaceMaximum >
            target * settings.maximumJoinSurfaceDistanceRatio +
                settings.geometryEpsilon) {
        return result;
    }
    const double direction =
        directionSum / static_cast<double>(points.size());
    const double aspect =
        maximumEdge / std::max(minimumEdge, settings.geometryEpsilon);
    const double density = std::abs(
        std::log(std::max(maximumEdge, settings.geometryEpsilon) /
                 std::max(target, settings.geometryEpsilon)));
    const double normalizedArea =
        area / std::max(maximumEdge * maximumEdge,
                        settings.geometryEpsilon);
    const double quality =
        std::max(0.0, aspect - 1.0) +
        1.0 / std::max(normalizedArea, 1.0e-6);
    result.face.vertices = std::move(references);
    result.face.triangleReason = triangleReason;
    result.face.cost =
        settings.densityCostWeight * density +
        settings.directionCostWeight * direction / 45.0 +
        settings.surfaceCostWeight *
            surfaceMaximum / std::max(target, settings.geometryEpsilon) +
        settings.qualityCostWeight * quality +
        (points.size() == 3U ? settings.trianglePenalty : 0.0);
    result.surfaceError = surfaceMaximum;
    result.directionErrorDegrees = direction;
    result.valid = std::isfinite(result.face.cost);
    return result;
}

struct JoinCandidateResult final
{
    bool valid = false;
    InterfaceCorrespondence correspondence;
    std::vector<InterfaceJoinFace> faces;
    double cost = std::numeric_limits<double>::infinity();
    double surfaceSum = 0.0;
    double surfaceMaximum = 0.0;
    double directionSum = 0.0;
    std::size_t triangleCount = 0U;
    std::size_t quadCount = 0U;
    std::size_t rejected = 0U;
};

JoinTriangleReason triangleReasonForCounts(
    std::size_t scaffoldCount,
    std::size_t coreCount) noexcept
{
    if (scaffoldCount != coreCount) {
        return JoinTriangleReason::InterfaceCountMismatch;
    }
    if ((scaffoldCount + coreCount) % 2U != 0U) {
        return JoinTriangleReason::ParityTermination;
    }
    return JoinTriangleReason::FlowTermination;
}

JoinCandidateResult buildJoinCandidate(
    const RemeshInput& input,
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core,
    const InterfaceCorrespondence& correspondence,
    const CoreInterfaceJoinSettings& settings)
{
    JoinCandidateResult result;
    result.correspondence = correspondence;
    const std::size_t scaffoldCount = scaffold.vertices.size();
    const std::size_t coreCount = core.vertices.size();
    const auto scaffoldRef = [&scaffold](std::size_t index) {
        return JoinVertexReference{
            JoinVertexDomain::Scaffold,
            scaffold.vertices[index % scaffold.vertices.size()]
                .mutableVertexId};
    };
    const auto coreRef = [&core, &correspondence](
                             std::size_t logicalIndex) {
        const std::size_t descriptorIndex = coreOrderIndex(
            logicalIndex,
            core.vertices.size(),
            correspondence.coreSeamOffset,
            correspondence.coreOrderReversed);
        return JoinVertexReference{
            JoinVertexDomain::Core,
            core.vertices[descriptorIndex].coreVertexId};
    };

    struct Cell final
    {
        double cost = std::numeric_limits<double>::infinity();
        std::size_t previousI = kInvalidIndex;
        std::size_t previousJ = kInvalidIndex;
        char move = 0;
        FaceEvaluation evaluation;
    };
    const std::size_t width = coreCount + 1U;
    std::vector<Cell> cells((scaffoldCount + 1U) * width);
    const auto cell = [&](std::size_t i, std::size_t j) -> Cell& {
        return cells[i * width + j];
    };
    cell(0U, 0U).cost = 0.0;
    const JoinTriangleReason triangleReason =
        triangleReasonForCounts(scaffoldCount, coreCount);
    const auto consider = [&](std::size_t fromI,
                              std::size_t fromJ,
                              std::size_t toI,
                              std::size_t toJ,
                              char move,
                              std::vector<JoinVertexReference> vertices) {
        FaceEvaluation evaluation = evaluateJoinFace(
            input,
            std::move(vertices),
            triangleReason,
            scaffold,
            core,
            settings);
        if (!evaluation.valid) {
            ++result.rejected;
            return;
        }
        const double candidate =
            cell(fromI, fromJ).cost + evaluation.face.cost;
        Cell& destination = cell(toI, toJ);
        if (candidate < destination.cost - 1.0e-12 ||
            (std::abs(candidate - destination.cost) <= 1.0e-12 &&
             move < destination.move)) {
            destination.cost = candidate;
            destination.previousI = fromI;
            destination.previousJ = fromJ;
            destination.move = move;
            destination.evaluation = std::move(evaluation);
        }
    };
    for (std::size_t i = 0U; i <= scaffoldCount; ++i) {
        for (std::size_t j = 0U; j <= coreCount; ++j) {
            if (!std::isfinite(cell(i, j).cost)) {
                continue;
            }
            if (i < scaffoldCount && j < coreCount) {
                consider(
                    i, j, i + 1U, j + 1U, 'Q',
                    {scaffoldRef(i), scaffoldRef(i + 1U),
                     coreRef(j + 1U), coreRef(j)});
            }
            if (i < scaffoldCount && j < coreCount) {
                consider(
                    i, j, i + 1U, j, 'S',
                    {scaffoldRef(i), scaffoldRef(i + 1U),
                     coreRef(j)});
                consider(
                    i, j, i, j + 1U, 'C',
                    {scaffoldRef(i), coreRef(j + 1U), coreRef(j)});
            }
        }
    }
    if (!std::isfinite(cell(scaffoldCount, coreCount).cost)) {
        return result;
    }
    std::size_t i = scaffoldCount;
    std::size_t j = coreCount;
    while (i != 0U || j != 0U) {
        const Cell& current = cell(i, j);
        if (current.previousI == kInvalidIndex ||
            current.previousJ == kInvalidIndex) {
            return JoinCandidateResult();
        }
        result.faces.push_back(current.evaluation.face);
        result.surfaceSum += current.evaluation.surfaceError;
        result.surfaceMaximum = std::max(
            result.surfaceMaximum,
            current.evaluation.surfaceError);
        result.directionSum +=
            current.evaluation.directionErrorDegrees;
        if (current.evaluation.face.vertices.size() == 3U) {
            ++result.triangleCount;
        } else {
            ++result.quadCount;
        }
        i = current.previousI;
        j = current.previousJ;
    }
    std::reverse(result.faces.begin(), result.faces.end());
    const std::size_t minimumTriangles =
        absoluteDifference(scaffoldCount, coreCount);
    if (result.triangleCount >
        minimumTriangles + settings.residualTriangleBudget) {
        return JoinCandidateResult();
    }
    result.cost = cell(scaffoldCount, coreCount).cost +
        correspondence.totalCost;
    result.valid = true;
    return result;
}

InterfaceJoinResult buildJoin(
    const RemeshInput& input,
    const AdaptedInterfaceDescriptor& scaffold,
    const CoreBoundaryDescriptor& core,
    const CoreInterfaceJoinSettings& settings)
{
    InterfaceJoinResult result;
    if (!scaffold.success() || !core.success()) {
        result.status = InterfaceJoinStatus::InvalidInput;
        result.diagnosticMessage =
            "Join received an invalid ordered loop.";
        return result;
    }
    if (countRatio(scaffold.vertices.size(), core.vertices.size()) >
        settings.maximumJoinCountRatio) {
        result.status = InterfaceJoinStatus::ExcessiveMismatch;
        result.diagnosticMessage =
            "Join count mismatch exceeds the configured safe ratio.";
        return result;
    }
    const std::size_t scaffoldStates = scaffold.vertices.size() + 1U;
    const std::size_t coreStates = core.vertices.size() + 1U;
    if (coreStates != 0U &&
        scaffoldStates >
            settings.maximumDpStatesPerCandidate / coreStates) {
        result.status = InterfaceJoinStatus::ExcessiveMismatch;
        result.diagnosticMessage =
            "Join DP state count exceeds the configured safe per-candidate limit.";
        return result;
    }
    const std::size_t statesPerCandidate = scaffoldStates * coreStates;

    std::vector<SurfacePointMapping> scaleMappings;
    for (const auto& vertex : scaffold.vertices) {
        scaleMappings.push_back(vertex.surface);
    }
    for (const auto& vertex : core.vertices) {
        scaleMappings.push_back(vertex.surface);
    }
    const double scale = targetLength(input, scaleMappings);
    JoinCandidateResult best;
    const std::size_t maximumCandidatesByWork = std::max<std::size_t>(
        1U, settings.maximumTotalDpStates / statesPerCandidate);
    const std::size_t maximumSeams = std::max<std::size_t>(
        1U, std::min(
            settings.maximumSeamCandidates,
            maximumCandidatesByWork / 2U));
    const std::vector<std::size_t> seams = seamCandidates(
        scaffold, core, maximumSeams);
    for (const bool reversed : {false, true}) {
        for (const std::size_t seam : seams) {
            ++result.seamCandidatesTested;
            const Clock::time_point correspondenceStart = Clock::now();
            const InterfaceCorrespondence correspondence =
                buildCorrespondence(
                    scaffold, core, seam, reversed, scale);
            result.correspondenceMilliseconds +=
                elapsedMilliseconds(correspondenceStart);
            JoinCandidateResult candidate = buildJoinCandidate(
                input, scaffold, core, correspondence, settings);
            result.rejectedGeometryCandidateCount += candidate.rejected;
            if (!candidate.valid) {
                continue;
            }
            ++result.feasibleSeamCount;
            if (!best.valid ||
                std::tie(
                    candidate.cost,
                    candidate.triangleCount,
                    candidate.correspondence.coreOrderReversed,
                    candidate.correspondence.coreSeamOffset) <
                std::tie(
                    best.cost,
                    best.triangleCount,
                    best.correspondence.coreOrderReversed,
                    best.correspondence.coreSeamOffset)) {
                best = std::move(candidate);
            }
        }
    }
    if (!best.valid) {
        result.status = InterfaceJoinStatus::NoFeasibleJoin;
        result.diagnosticMessage =
            "No seam/winding candidate produced valid Triangle/Quad Join geometry.";
        return result;
    }
    result.status = InterfaceJoinStatus::Success;
    result.correspondence = std::move(best.correspondence);
    result.faces = std::move(best.faces);
    result.triangleCount = best.triangleCount;
    result.quadCount = best.quadCount;
    result.nGonCount = 0U;
    result.totalCost = best.cost;
    result.meanSurfaceError = result.faces.empty()
        ? 0.0
        : best.surfaceSum / static_cast<double>(result.faces.size());
    result.maximumSurfaceError = best.surfaceMaximum;
    result.meanDirectionDeviationDegrees = result.faces.empty()
        ? 0.0
        : best.directionSum / static_cast<double>(result.faces.size());
    result.diagnosticMessage =
        "Monotonic DP selected the minimum-cost valid seam/winding Join.";
    return result;
}

std::uint64_t bits(double value) noexcept
{
    std::uint64_t result = 0U;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

std::uint64_t combinedSignature(
    const CombinedRemeshResult& result) noexcept
{
    std::uint64_t hash = 1469598103934665603ULL;
    const auto add = [&hash](std::uint64_t value) {
        hash ^= value;
        hash *= 1099511628211ULL;
    };
    add(result.success ? 1U : 0U);
    for (const CombinedVertex& vertex : result.vertices) {
        add(bits(vertex.position.x));
        add(bits(vertex.position.y));
        add(bits(vertex.position.z));
        add(static_cast<std::uint64_t>(vertex.origin));
        add(vertex.sourceLocalId);
    }
    for (const CombinedPolygon& polygon : result.polygons) {
        add(static_cast<std::uint64_t>(polygon.region));
        add(polygon.vertexIndices.size());
        for (const std::size_t vertex : polygon.vertexIndices) {
            add(vertex);
        }
    }
    return hash;
}

CombinedRemeshResult combineAndValidate(
    const LocalMutablePatchMesh& mesh,
    const CoreRemeshResult& core,
    const InterfaceJoinResult& join,
    double epsilon)
{
    CombinedRemeshResult result;
    std::vector<std::size_t> scaffoldMap(
        mesh.vertices().size(), kInvalidIndex);
    for (const MutableVertex& vertex : mesh.vertices()) {
        if (vertex.deleted) {
            continue;
        }
        scaffoldMap[vertex.id] = result.vertices.size();
        CombinedVertex combined;
        combined.position = vertex.position;
        combined.origin = CombinedVertexOrigin::Scaffold;
        combined.sourceLocalId = vertex.id;
        combined.fixedOuterBoundary = vertex.fixedOuterBoundary;
        result.vertices.push_back(combined);
    }
    std::vector<std::size_t> coreMap(core.vertices.size(), kInvalidIndex);
    for (std::size_t vertexId = 0U;
         vertexId < core.vertices.size();
         ++vertexId) {
        coreMap[vertexId] = result.vertices.size();
        CombinedVertex combined;
        combined.position = core.vertices[vertexId];
        combined.origin = CombinedVertexOrigin::Core;
        combined.sourceLocalId = vertexId;
        combined.surface = core.sourceMappings[vertexId];
        result.vertices.push_back(combined);
    }
    for (const MutableFace& face : mesh.faces()) {
        if (face.deleted) {
            continue;
        }
        CombinedPolygon polygon;
        polygon.region = CombinedPolygonRegion::TransitionScaffold;
        for (const MutableVertexId vertex : face.vertexIds) {
            if (vertex >= scaffoldMap.size() ||
                scaffoldMap[vertex] == kInvalidIndex) {
                result.diagnosticMessage =
                    "Scaffold face mapping failed.";
                return result;
            }
            polygon.vertexIndices.push_back(scaffoldMap[vertex]);
        }
        polygon.type = polygon.vertexIndices.size() == 3U
            ? PolygonType::Triangle
            : (polygon.vertexIndices.size() == 4U
                ? PolygonType::Quad : PolygonType::NGon);
        result.polygons.push_back(std::move(polygon));
    }
    for (const ResultPolygon& source : core.polygons) {
        CombinedPolygon polygon;
        polygon.region = CombinedPolygonRegion::Core;
        polygon.type = source.type;
        for (const std::size_t vertex : source.vertexIndices) {
            if (vertex >= coreMap.size()) {
                result.diagnosticMessage =
                    "Core face mapping failed.";
                return result;
            }
            polygon.vertexIndices.push_back(coreMap[vertex]);
        }
        result.polygons.push_back(std::move(polygon));
    }
    for (const InterfaceJoinFace& source : join.faces) {
        CombinedPolygon polygon;
        polygon.region = CombinedPolygonRegion::InterfaceJoin;
        polygon.triangleReason = source.triangleReason;
        for (const JoinVertexReference& vertex : source.vertices) {
            if (vertex.domain == JoinVertexDomain::Scaffold) {
                if (vertex.vertexId >= scaffoldMap.size() ||
                    scaffoldMap[vertex.vertexId] == kInvalidIndex) {
                    result.diagnosticMessage =
                        "Join Scaffold vertex mapping failed.";
                    return result;
                }
                polygon.vertexIndices.push_back(
                    scaffoldMap[vertex.vertexId]);
            } else {
                if (vertex.vertexId >= coreMap.size()) {
                    result.diagnosticMessage =
                        "Join Core vertex mapping failed.";
                    return result;
                }
                polygon.vertexIndices.push_back(coreMap[vertex.vertexId]);
            }
        }
        polygon.type = polygon.vertexIndices.size() == 3U
            ? PolygonType::Triangle : PolygonType::Quad;
        result.polygons.push_back(std::move(polygon));
    }

    std::map<EdgeKey, std::vector<std::size_t>> edgeFaces;
    std::set<std::vector<std::size_t>> canonicalPolygons;
    std::vector<std::vector<std::size_t>> faceNeighbors(
        result.polygons.size());
    for (std::size_t faceId = 0U;
         faceId < result.polygons.size();
         ++faceId) {
        const CombinedPolygon& polygon = result.polygons[faceId];
        if (polygon.vertexIndices.size() < 3U) {
            ++result.metrics.zeroAreaPolygonCount;
            continue;
        }
        std::set<std::size_t> unique(
            polygon.vertexIndices.begin(), polygon.vertexIndices.end());
        if (unique.size() != polygon.vertexIndices.size()) {
            ++result.metrics.zeroAreaPolygonCount;
            continue;
        }
        std::vector<std::size_t> canonical(unique.begin(), unique.end());
        if (!canonicalPolygons.insert(canonical).second) {
            ++result.metrics.duplicatePolygonCount;
        }
        std::vector<Vec3> points;
        for (const std::size_t vertex : polygon.vertexIndices) {
            if (vertex >= result.vertices.size() ||
                !result.vertices[vertex].position.finite()) {
                ++result.metrics.zeroAreaPolygonCount;
                points.clear();
                break;
            }
            points.push_back(result.vertices[vertex].position);
        }
        if (points.empty() ||
            polygonArea(points, epsilon) <= epsilon) {
            ++result.metrics.zeroAreaPolygonCount;
        }
        if (polygon.type == PolygonType::Triangle) {
            ++result.metrics.triangleCount;
        } else if (polygon.type == PolygonType::Quad) {
            ++result.metrics.quadCount;
        } else {
            ++result.metrics.nGonCount;
        }
        if (polygon.region == CombinedPolygonRegion::InterfaceJoin &&
            polygon.type == PolygonType::NGon) {
            ++result.metrics.joinNGonCount;
        }
        for (std::size_t corner = 0U;
             corner < polygon.vertexIndices.size();
             ++corner) {
            edgeFaces[edgeKey(
                polygon.vertexIndices[corner],
                polygon.vertexIndices[
                    (corner + 1U) % polygon.vertexIndices.size()])]
                .push_back(faceId);
        }
    }
    std::map<std::size_t, std::vector<std::size_t>> boundaryAdjacency;
    for (const auto& [edge, faces] : edgeFaces) {
        if (faces.size() > 2U) {
            ++result.metrics.nonManifoldEdgeCount;
        } else if (faces.size() == 2U) {
            faceNeighbors[faces[0]].push_back(faces[1]);
            faceNeighbors[faces[1]].push_back(faces[0]);
        } else if (faces.size() == 1U) {
            boundaryAdjacency[edge.first].push_back(edge.second);
            boundaryAdjacency[edge.second].push_back(edge.first);
        }
    }
    std::set<std::size_t> unvisitedFaces;
    for (std::size_t face = 0U; face < result.polygons.size(); ++face) {
        unvisitedFaces.insert(face);
    }
    while (!unvisitedFaces.empty()) {
        ++result.metrics.connectedComponentCount;
        std::queue<std::size_t> pending;
        pending.push(*unvisitedFaces.begin());
        unvisitedFaces.erase(unvisitedFaces.begin());
        while (!pending.empty()) {
            const std::size_t face = pending.front();
            pending.pop();
            for (const std::size_t adjacent : faceNeighbors[face]) {
                if (unvisitedFaces.erase(adjacent) != 0U) {
                    pending.push(adjacent);
                }
            }
        }
    }
    for (auto& [vertex, neighbors] : boundaryAdjacency) {
        (void)vertex;
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(
            std::unique(neighbors.begin(), neighbors.end()),
            neighbors.end());
        if (neighbors.size() != 2U) {
            ++result.metrics.nonManifoldEdgeCount;
        }
    }
    std::set<EdgeKey> unusedBoundary;
    for (const auto& [vertex, neighbors] : boundaryAdjacency) {
        for (const std::size_t neighbor : neighbors) {
            unusedBoundary.insert(edgeKey(vertex, neighbor));
        }
    }
    while (!unusedBoundary.empty()) {
        const std::size_t start = unusedBoundary.begin()->first;
        std::size_t previous = kInvalidIndex;
        std::size_t current = start;
        ResultBoundaryLoop loop;
        loop.closed = true;
        std::size_t guard = 0U;
        do {
            if (++guard > boundaryAdjacency.size() + 1U) {
                ++result.metrics.nonManifoldEdgeCount;
                break;
            }
            loop.vertexIndices.push_back(current);
            const auto found = boundaryAdjacency.find(current);
            if (found == boundaryAdjacency.end() ||
                found->second.size() != 2U) {
                ++result.metrics.nonManifoldEdgeCount;
                break;
            }
            const std::size_t next =
                found->second.front() == previous
                ? found->second.back() : found->second.front();
            if (unusedBoundary.erase(edgeKey(current, next)) == 0U) {
                ++result.metrics.nonManifoldEdgeCount;
                break;
            }
            previous = current;
            current = next;
        } while (current != start);
        result.boundaryLoops.push_back(std::move(loop));
    }
    result.metrics.outerBoundaryLoopCount = result.boundaryLoops.size();

    std::set<EdgeKey> expectedFixedEdges;
    const ScaffoldBoundaryLoop& fixed = mesh.fixedOuterBoundary();
    for (std::size_t index = 0U;
         index < fixed.vertexIndices.size();
         ++index) {
        const MutableVertexId mutableId = fixed.vertexIndices[index];
        const MutableVertexId nextId =
            fixed.vertexIndices[
                (index + 1U) % fixed.vertexIndices.size()];
        if (mutableId >= scaffoldMap.size() ||
            nextId >= scaffoldMap.size()) {
            result.diagnosticMessage =
                "Fixed Boundary mapping is invalid.";
            return result;
        }
        const std::size_t vertex = scaffoldMap[mutableId];
        const std::size_t next = scaffoldMap[nextId];
        result.fixedBoundaryVertexIndices.push_back(vertex);
        expectedFixedEdges.insert(edgeKey(vertex, next));
        const MutableVertex& sourceVertex = mesh.vertices()[mutableId];
        result.metrics.maximumFixedBoundaryDisplacement = std::max(
            result.metrics.maximumFixedBoundaryDisplacement,
            (sourceVertex.position - sourceVertex.sourcePosition).length());
    }
    std::set<EdgeKey> actualBoundaryEdges;
    for (const auto& [edge, faces] : edgeFaces) {
        if (faces.size() == 1U) {
            actualBoundaryEdges.insert(edge);
        }
    }

    std::vector<EdgeKey> crossDomainEdges;
    for (const CombinedPolygon& polygon : result.polygons) {
        if (polygon.region != CombinedPolygonRegion::InterfaceJoin) {
            continue;
        }
        for (std::size_t index = 0U;
             index < polygon.vertexIndices.size();
             ++index) {
            const std::size_t first = polygon.vertexIndices[index];
            const std::size_t second =
                polygon.vertexIndices[
                    (index + 1U) % polygon.vertexIndices.size()];
            if (result.vertices[first].origin !=
                result.vertices[second].origin) {
                crossDomainEdges.push_back(edgeKey(first, second));
            }
        }
    }
    std::sort(crossDomainEdges.begin(), crossDomainEdges.end());
    crossDomainEdges.erase(
        std::unique(crossDomainEdges.begin(), crossDomainEdges.end()),
        crossDomainEdges.end());
    for (std::size_t first = 0U;
         first < crossDomainEdges.size();
         ++first) {
        for (std::size_t second = first + 1U;
             second < crossDomainEdges.size();
             ++second) {
            const EdgeKey& a = crossDomainEdges[first];
            const EdgeKey& b = crossDomainEdges[second];
            if (a.first == b.first || a.first == b.second ||
                a.second == b.first || a.second == b.second) {
                continue;
            }
            const SegmentDistance distance = segmentDistance(
                result.vertices[a.first].position,
                result.vertices[a.second].position,
                result.vertices[b.first].position,
                result.vertices[b.second].position,
                epsilon);
            if (distance.distance <= epsilon * 10.0 &&
                distance.firstParameter > 1.0e-6 &&
                distance.firstParameter < 1.0 - 1.0e-6 &&
                distance.secondParameter > 1.0e-6 &&
                distance.secondParameter < 1.0 - 1.0e-6) {
                ++result.metrics.boundaryCrossingCount;
            }
        }
    }
    result.success =
        result.metrics.connectedComponentCount == 1U &&
        result.metrics.outerBoundaryLoopCount == 1U &&
        result.metrics.nonManifoldEdgeCount == 0U &&
        result.metrics.zeroAreaPolygonCount == 0U &&
        result.metrics.duplicatePolygonCount == 0U &&
        result.metrics.boundaryCrossingCount == 0U &&
        result.metrics.joinNGonCount == 0U &&
        result.metrics.maximumFixedBoundaryDisplacement == 0.0 &&
        actualBoundaryEdges == expectedFixedEdges;
    result.diagnosticMessage = result.success
        ? "Combined Scaffold + Join + Core topology is valid; only the Fixed Source Boundary remains external."
        : "Combined topology failed a hard R7 invariant.";
    result.signature = combinedSignature(result);
    return result;
}

}  // namespace

CoreInterfaceJoinResult CoreInterfaceJoinSolver::join(
    const RemeshInput& input,
    const RegionComponent& component,
    const SourceTransitionScaffold& scaffold,
    const AdaptedScaffoldResult& adaptation,
    const CoreRemeshResult& core,
    const CoreInterfaceJoinSettings& settings) const noexcept
{
    CoreInterfaceJoinResult result;
    result.componentId = component.componentId;
    result.scaffoldStatus = adaptation.status;
    result.core = core;
    const Clock::time_point totalStart = Clock::now();
    const auto fail = [&](CoreJoinStatus status,
                          const std::string& message) {
        result.status = status;
        result.diagnosticMessage = message;
        result.timings.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return result;
    };
    try {
        (void)scaffold;
        if (!adaptation.validResult() ||
            !adaptation.adaptedMesh.valid()) {
            return fail(
                CoreJoinStatus::InterfaceInvalid,
                "R7 requires a valid R6 Success or Partial Scaffold.");
        }
        if (!core.success()) {
            return fail(
                core.status == CoreGenerationStatus::CoreBoundaryInvalid
                    ? CoreJoinStatus::CoreBoundaryInvalid
                    : CoreJoinStatus::CoreGenerationFailed,
                core.diagnosticMessage);
        }
        const Clock::time_point descriptorStart = Clock::now();
        result.interfaceBefore = describeInterface(
            input, adaptation.adaptedMesh, settings.geometryEpsilon);
        result.timings.interfaceDescriptorMilliseconds =
            elapsedMilliseconds(descriptorStart);
        if (!result.interfaceBefore.success()) {
            return fail(
                CoreJoinStatus::InterfaceInvalid,
                result.interfaceBefore.diagnosticMessage);
        }

        const Clock::time_point reconciliationStart = Clock::now();
        result.reconciliation = reconcileInterface(
            input, adaptation, core, settings);
        result.timings.reconciliationMilliseconds =
            elapsedMilliseconds(reconciliationStart);
        if (!result.reconciliation.usable()) {
            return fail(
                CoreJoinStatus::ReconciliationFailed,
                result.reconciliation.diagnosticMessage);
        }

        const Clock::time_point joinStart = Clock::now();
        result.join = buildJoin(
            input,
            result.reconciliation.interface,
            core.boundary,
            settings);
        result.timings.correspondenceMilliseconds =
            result.join.correspondenceMilliseconds;
        result.timings.joinMilliseconds = std::max(
            0.0,
            elapsedMilliseconds(joinStart) -
                result.timings.correspondenceMilliseconds);
        if (!result.join.success()) {
            return fail(
                CoreJoinStatus::JoinFailed,
                result.join.diagnosticMessage);
        }

        const Clock::time_point validationStart = Clock::now();
        result.combined = combineAndValidate(
            result.reconciliation.mesh,
            core,
            result.join,
            settings.geometryEpsilon);
        result.timings.combinedValidationMilliseconds =
            elapsedMilliseconds(validationStart);
        if (!result.combined.success) {
            return fail(
                CoreJoinStatus::CombinedValidationFailed,
                result.combined.diagnosticMessage);
        }
        result.status =
            adaptation.status == ScaffoldAdaptationStatus::Partial ||
            result.reconciliation.status == ReconciliationStatus::Partial
            ? CoreJoinStatus::Partial : CoreJoinStatus::Success;
        result.signature = result.combined.signature;
        std::ostringstream diagnostic;
        diagnostic
            << "R7 experimental Core Interface Join "
            << coreJoinStatusName(result.status)
            << "; Scaffold="
            << result.reconciliation.scaffoldCountBefore << "->"
            << result.reconciliation.scaffoldCountAfter
            << "; Core=" << core.boundary.vertices.size()
            << "; Join Q/T=" << result.join.quadCount << "/"
            << result.join.triangleCount
            << "; seam=" << result.join.correspondence.coreSeamOffset
            << "; reversed="
            << (result.join.correspondence.coreOrderReversed
                ? "yes" : "no")
            << "; Fixed displacement="
            << result.combined.metrics.maximumFixedBoundaryDisplacement;
        result.diagnosticMessage = diagnostic.str();
        result.timings.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return result;
    } catch (const std::exception& exception) {
        return fail(
            CoreJoinStatus::JoinFailed,
            std::string("R7 Join exception: ") + exception.what());
    } catch (...) {
        return fail(
            CoreJoinStatus::JoinFailed,
            "R7 Join raised an unknown exception.");
    }
}

const char* coreJoinStatusName(CoreJoinStatus status) noexcept
{
    switch (status) {
    case CoreJoinStatus::Success: return "Success";
    case CoreJoinStatus::Partial: return "Partial";
    case CoreJoinStatus::CoreGenerationFailed:
        return "CoreGenerationFailed";
    case CoreJoinStatus::CoreBoundaryInvalid:
        return "CoreBoundaryInvalid";
    case CoreJoinStatus::InterfaceInvalid: return "InterfaceInvalid";
    case CoreJoinStatus::CorrespondenceFailed:
        return "CorrespondenceFailed";
    case CoreJoinStatus::ReconciliationFailed:
        return "ReconciliationFailed";
    case CoreJoinStatus::JoinFailed: return "JoinFailed";
    case CoreJoinStatus::CombinedValidationFailed:
        return "CombinedValidationFailed";
    }
    return "Unknown";
}

const char* interfaceJoinStatusName(
    InterfaceJoinStatus status) noexcept
{
    switch (status) {
    case InterfaceJoinStatus::Success: return "Success";
    case InterfaceJoinStatus::InvalidInput: return "InvalidInput";
    case InterfaceJoinStatus::NoCorrespondence:
        return "NoCorrespondence";
    case InterfaceJoinStatus::ExcessiveMismatch:
        return "ExcessiveMismatch";
    case InterfaceJoinStatus::NoFeasibleJoin:
        return "NoFeasibleJoin";
    case InterfaceJoinStatus::ValidationFailed:
        return "ValidationFailed";
    }
    return "Unknown";
}

const char* reconciliationStatusName(
    ReconciliationStatus status) noexcept
{
    switch (status) {
    case ReconciliationStatus::Success: return "Success";
    case ReconciliationStatus::Partial: return "Partial";
    case ReconciliationStatus::InvalidInput: return "InvalidInput";
    case ReconciliationStatus::OperationFailed:
        return "OperationFailed";
    case ReconciliationStatus::ExcessiveMismatch:
        return "ExcessiveMismatch";
    }
    return "Unknown";
}

const char* joinTriangleReasonName(
    JoinTriangleReason reason) noexcept
{
    switch (reason) {
    case JoinTriangleReason::InterfaceCountMismatch:
        return "InterfaceCountMismatch";
    case JoinTriangleReason::ParityTermination:
        return "ParityTermination";
    case JoinTriangleReason::FlowTermination:
        return "FlowTermination";
    case JoinTriangleReason::JoinFallback: return "JoinFallback";
    }
    return "Unknown";
}

}  // namespace directional_retopo::solver
