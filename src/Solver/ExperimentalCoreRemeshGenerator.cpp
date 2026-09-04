#include "Solver/ExperimentalCoreRemeshGenerator.h"

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Remesh/AutoRemesherAdapter.h"
#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"
#include "Remesh/SurfaceConformer.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace directional_retopo::solver {
namespace {

using Clock = std::chrono::steady_clock;
using EdgeKey = std::pair<std::size_t, std::size_t>;

double elapsedMilliseconds(Clock::time_point start) noexcept
{
    return std::chrono::duration<double, std::milli>(
        Clock::now() - start).count();
}

EdgeKey edgeKey(std::size_t first, std::size_t second) noexcept
{
    return first < second ? EdgeKey(first, second) : EdgeKey(second, first);
}

MPoint toMPoint(const Vec3& value) noexcept
{
    return MPoint(value.x, value.y, value.z);
}

MVector toMVector(const Vec3& value) noexcept
{
    return MVector(value.x, value.y, value.z);
}

Vec3 toPortable(const MPoint& value) noexcept
{
    return {value.x, value.y, value.z};
}

Vec3 toPortable(const MVector& value) noexcept
{
    return {value.x, value.y, value.z};
}

DirectionFieldData makeDirectionField(const RemeshInput& input)
{
    DirectionFieldData result;
    result.perFace.resize(input.directionField.size());
    for (std::size_t index = 0U; index < input.directionField.size(); ++index) {
        const FaceDirection& source = input.directionField[index];
        FaceDirectionField& destination = result.perFace[index];
        destination.normal = toMVector(source.normal);
        destination.uDirection = toMVector(source.uDirection);
        destination.vDirection = toMVector(source.vDirection);
        destination.constraintWeight = source.paintConstraintWeight;
        destination.topologyGuidanceWeight = source.topologyGuidanceWeight;
        destination.hasPaintConstraint = source.paintConstraintWeight > 0.0;
        destination.valid = source.valid;
    }
    return result;
}

DensityFieldData makeDensityField(const RemeshInput& input)
{
    DensityFieldData result;
    result.perFace.resize(input.densityField.size());
    for (std::size_t index = 0U; index < input.densityField.size(); ++index) {
        const FaceDensity& source = input.densityField[index];
        directional_retopo::FaceDensity& destination = result.perFace[index];
        destination.targetEdgeLength = source.effectiveTargetEdgeLength;
        destination.baseTargetEdgeLength =
            source.requestedTargetEdgeLength;
        destination.scaleU = source.scaleU;
        destination.scaleV = source.scaleV;
        destination.curvatureLimited = source.curvatureConstrained;
        destination.valid = source.valid;
    }
    return result;
}

bool extractSourceBoundary(
    const RemeshInput& input,
    const std::vector<std::size_t>& faceIndices,
    std::vector<OrderedBoundaryLoop>& loops,
    std::string& diagnostic)
{
    const std::unordered_set<std::size_t> faceSet(
        faceIndices.begin(), faceIndices.end());
    std::vector<std::size_t> boundaryEdges;
    std::map<std::size_t, std::vector<std::size_t>> vertexEdges;
    for (std::size_t edgeIndex = 0U;
         edgeIndex < input.sourceMesh.edges.size();
         ++edgeIndex) {
        const SourceEdge& edge = input.sourceMesh.edges[edgeIndex];
        std::size_t inside = 0U;
        for (const std::size_t faceIndex : edge.faceIndices) {
            inside += faceSet.count(faceIndex);
        }
        if (inside != 1U) {
            continue;
        }
        boundaryEdges.push_back(edgeIndex);
        vertexEdges[edge.vertexIndices[0]].push_back(edgeIndex);
        vertexEdges[edge.vertexIndices[1]].push_back(edgeIndex);
    }
    if (boundaryEdges.empty()) {
        diagnostic = "Core source domain has no boundary.";
        return false;
    }
    for (auto& [vertex, edges] : vertexEdges) {
        (void)vertex;
        std::sort(edges.begin(), edges.end());
        if (edges.size() != 2U) {
            diagnostic =
                "Core source boundary is open, branched, or non-manifold.";
            return false;
        }
    }

    std::set<std::size_t> unused(
        boundaryEdges.begin(), boundaryEdges.end());
    while (!unused.empty()) {
        const std::size_t firstEdgeId = *unused.begin();
        const SourceEdge& firstEdge = input.sourceMesh.edges[firstEdgeId];
        const std::size_t start = std::min(
            firstEdge.vertexIndices[0], firstEdge.vertexIndices[1]);
        std::size_t current = start;
        std::size_t previousEdge = kInvalidIndex;
        OrderedBoundaryLoop loop;
        loop.closed = true;
        std::size_t guard = 0U;
        do {
            if (++guard > boundaryEdges.size() + 1U) {
                diagnostic = "Core source boundary traversal exceeded guard.";
                return false;
            }
            loop.vertexIndices.push_back(current);
            const std::vector<std::size_t>& incident = vertexEdges[current];
            std::size_t nextEdge = incident.front();
            if (nextEdge == previousEdge) {
                nextEdge = incident.back();
            } else if (previousEdge == kInvalidIndex &&
                       incident.back() < nextEdge) {
                nextEdge = incident.back();
            }
            if (unused.erase(nextEdge) == 0U) {
                diagnostic =
                    "Core source boundary traversal repeated an edge.";
                return false;
            }
            loop.edgeIndices.push_back(nextEdge);
            const SourceEdge& edge = input.sourceMesh.edges[nextEdge];
            const std::size_t next = edge.vertexIndices[0] == current
                ? edge.vertexIndices[1]
                : edge.vertexIndices[0];
            previousEdge = nextEdge;
            current = next;
        } while (current != start);
        if (loop.vertexIndices.size() < 3U) {
            diagnostic = "Core source boundary contains fewer than 3 vertices.";
            return false;
        }
        for (const std::size_t vertex : loop.vertexIndices) {
            loop.sourceVertexIds.push_back(
                input.sourceMesh.vertices[vertex].sourceVertexId);
        }
        for (const std::size_t edge : loop.edgeIndices) {
            loop.sourceEdgeIds.push_back(
                input.sourceMesh.edges[edge].sourceEdgeId);
            loop.touchesOriginalMeshBoundary =
                loop.touchesOriginalMeshBoundary ||
                input.sourceMesh.edges[edge].originalMeshBoundary;
        }
        loops.push_back(std::move(loop));
    }
    return true;
}

bool buildCorePatch(
    const RemeshInput& input,
    const RegionComponent& component,
    const std::vector<std::size_t>& faceIndices,
    TriangulatedPatch& patch,
    std::string& diagnostic)
{
    patch = TriangulatedPatch();
    patch.componentId = component.componentId;
    patch.purpose = TriangulatedPatch::Purpose::InnerRemeshCore;
    if (faceIndices.empty()) {
        diagnostic = "Core source domain contains no faces.";
        return false;
    }
    std::vector<OrderedBoundaryLoop> sourceLoops;
    if (!extractSourceBoundary(input, faceIndices, sourceLoops, diagnostic) ||
        sourceLoops.size() != 1U || !sourceLoops.front().closed) {
        if (diagnostic.empty()) {
            diagnostic =
                "R7 Core source domain must have one closed boundary.";
        }
        return false;
    }

    std::unordered_set<std::size_t> boundaryVertices(
        sourceLoops.front().vertexIndices.begin(),
        sourceLoops.front().vertexIndices.end());
    std::unordered_map<std::size_t, std::size_t> localBySource;
    const auto localVertex =
        [&](std::size_t sourceIndex, std::size_t& localIndex) {
            if (sourceIndex >= input.sourceMesh.vertices.size()) {
                return false;
            }
            const auto found = localBySource.find(sourceIndex);
            if (found != localBySource.end()) {
                localIndex = found->second;
                return true;
            }
            localIndex = patch.vertices.size();
            localBySource.emplace(sourceIndex, localIndex);
            patch.vertices.push_back({
                toMPoint(input.sourceMesh.vertices[sourceIndex].position),
                static_cast<int>(
                    input.sourceMesh.vertices[sourceIndex].sourceVertexId),
                boundaryVertices.count(sourceIndex) != 0U});
            return true;
        };

    patch.sourceFaceToTriangleIndices.resize(input.sourceMesh.faces.size());
    for (const std::size_t faceIndex : faceIndices) {
        if (faceIndex >= input.sourceMesh.faces.size()) {
            diagnostic = "Core source domain contains an invalid face.";
            return false;
        }
        const SourceFace& face = input.sourceMesh.faces[faceIndex];
        for (const std::size_t triangleIndex : face.triangleIndices) {
            if (triangleIndex >= input.sourceMesh.triangles.size()) {
                diagnostic = "Core source face has invalid triangulation.";
                return false;
            }
            PatchTriangle triangle;
            triangle.sourceFaceId =
                static_cast<int>(face.sourceFaceId);
            for (std::size_t corner = 0U; corner < 3U; ++corner) {
                if (!localVertex(
                        input.sourceMesh.triangles[triangleIndex]
                            .vertexIndices[corner],
                        triangle.vertexIndices[corner])) {
                    diagnostic =
                        "Core source triangle has an invalid vertex.";
                    return false;
                }
            }
            patch.sourceFaceToTriangleIndices[faceIndex].push_back(
                patch.triangles.size());
            patch.triangles.push_back(triangle);
        }
    }

    PatchBoundaryLoop patchLoop;
    patchLoop.closed = true;
    for (const std::size_t sourceVertex :
         sourceLoops.front().vertexIndices) {
        std::size_t local = kInvalidIndex;
        if (!localVertex(sourceVertex, local)) {
            diagnostic = "Core source boundary vertex mapping failed.";
            return false;
        }
        patchLoop.vertexIndices.push_back(local);
        patchLoop.sourceVertexIds.push_back(static_cast<int>(
            input.sourceMesh.vertices[sourceVertex].sourceVertexId));
    }
    for (const std::size_t sourceEdge :
         sourceLoops.front().edgeIndices) {
        const SourceEdge& edge = input.sourceMesh.edges[sourceEdge];
        std::size_t first = kInvalidIndex;
        std::size_t second = kInvalidIndex;
        if (!localVertex(edge.vertexIndices[0], first) ||
            !localVertex(edge.vertexIndices[1], second)) {
            diagnostic = "Core source boundary edge mapping failed.";
            return false;
        }
        patchLoop.sourceEdgeIds.push_back(
            static_cast<int>(edge.sourceEdgeId));
        patch.boundaryEdges.push_back({
            {first, second},
            static_cast<int>(edge.sourceEdgeId),
            edge.originalMeshBoundary});
    }
    patch.boundaryLoops.push_back(std::move(patchLoop));
    patch.diagnosticMessage =
        "R7 Core-only source domain built from cached triangulation.";
    if (patch.vertices.size() < 3U || patch.triangles.empty()) {
        diagnostic = "Core source domain is too small.";
        return false;
    }
    return true;
}

std::vector<std::size_t> insetCoreFaces(
    const RemeshInput& input,
    const RegionComponent& component,
    unsigned int rings)
{
    std::unordered_set<std::size_t> current(
        component.coreFaceIndices.begin(), component.coreFaceIndices.end());
    for (unsigned int ring = 0U; ring < rings && !current.empty(); ++ring) {
        std::vector<std::size_t> boundary;
        for (const std::size_t faceIndex : current) {
            bool isBoundary = false;
            for (const std::size_t adjacent :
                 input.sourceMesh.faces[faceIndex].adjacentFaceIndices) {
                if (current.count(adjacent) == 0U) {
                    isBoundary = true;
                    break;
                }
            }
            if (!isBoundary) {
                for (const std::size_t edgeIndex :
                     input.sourceMesh.faces[faceIndex].edgeIndices) {
                    if (input.sourceMesh.edges[edgeIndex].faceIndices.size() <
                        2U) {
                        isBoundary = true;
                        break;
                    }
                }
            }
            if (isBoundary) {
                boundary.push_back(faceIndex);
            }
        }
        if (boundary.empty()) {
            break;
        }
        for (const std::size_t face : boundary) {
            current.erase(face);
        }
    }
    std::vector<std::size_t> result(current.begin(), current.end());
    std::sort(result.begin(), result.end());
    return result;
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

SurfacePointMapping mapToSource(
    const RemeshInput& input,
    const Vec3& position,
    SourceId preferredFaceId,
    const std::vector<std::size_t>& permittedFaceIndices,
    double epsilon)
{
    SurfacePointMapping best;
    std::vector<std::size_t> candidateTriangles;
    const std::size_t preferred =
        sourceFaceIndex(input.sourceMesh, preferredFaceId);
    if (preferred != kInvalidIndex) {
        candidateTriangles =
            input.sourceMesh.faces[preferred].triangleIndices;
        for (const std::size_t adjacent :
             input.sourceMesh.faces[preferred].adjacentFaceIndices) {
            const auto& triangles =
                input.sourceMesh.faces[adjacent].triangleIndices;
            candidateTriangles.insert(
                candidateTriangles.end(), triangles.begin(), triangles.end());
        }
    }
    if (candidateTriangles.empty()) {
        for (const std::size_t faceIndex : permittedFaceIndices) {
            if (faceIndex >= input.sourceMesh.faces.size()) {
                continue;
            }
            const auto& triangles = input.sourceMesh.faces[faceIndex].triangleIndices;
            candidateTriangles.insert(candidateTriangles.end(), triangles.begin(), triangles.end());
        }
    }
    std::sort(candidateTriangles.begin(), candidateTriangles.end());
    candidateTriangles.erase(
        std::unique(candidateTriangles.begin(), candidateTriangles.end()),
        candidateTriangles.end());
    for (const std::size_t triangleIndex : candidateTriangles) {
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
        if (!best.valid || distance < best.surfaceDistance) {
            const Vec3 normal = (b - a).cross(c - a).normalized(epsilon);
            if (normal.squaredLength() <= epsilon * epsilon) {
                continue;
            }
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


bool restoreOpenCoreBoundary(
    QuadPatchResult& result,
    double epsilon,
    std::size_t& removedPolygonCount,
    std::string& diagnostic)
{
    removedPolygonCount = 0U;
    if (result.conformedVertices.empty() || result.polygons.empty() ||
        result.sourceMappings.size() != result.conformedVertices.size()) {
        diagnostic =
            "Core closure filtering requires conformed vertices and local Source mappings.";
        return false;
    }
    std::vector<std::vector<std::size_t>> kept;
    kept.reserve(result.polygons.size());
    for (const std::vector<std::size_t>& polygon : result.polygons) {
        if (polygon.size() < 3U) {
            ++removedPolygonCount;
            continue;
        }
        std::set<std::size_t> unique;
        MVector areaNormal;
        MVector sourceNormal;
        bool valid = true;
        for (const std::size_t vertexId : polygon) {
            if (vertexId >= result.conformedVertices.size() ||
                vertexId >= result.sourceMappings.size() ||
                !unique.insert(vertexId).second) {
                valid = false;
                break;
            }
            sourceNormal += result.sourceMappings[vertexId].sourceNormal;
        }
        if (valid) {
            const MPoint& origin = result.conformedVertices[polygon.front()];
            for (std::size_t corner = 1U;
                 corner + 1U < polygon.size(); ++corner) {
                areaNormal +=
                    (result.conformedVertices[polygon[corner]] - origin) ^
                    (result.conformedVertices[polygon[corner + 1U]] - origin);
            }
        }
        if (!valid || areaNormal.length() <= epsilon ||
            sourceNormal.length() <= epsilon) {
            ++removedPolygonCount;
            continue;
        }
        areaNormal.normalize();
        sourceNormal.normalize();
        if (areaNormal * sourceNormal <= 1.0e-6) {
            ++removedPolygonCount;
            continue;
        }
        kept.push_back(polygon);
    }
    if (kept.empty()) {
        diagnostic =
            "Core closure filtering removed every extracted polygon.";
        return false;
    }
    result.polygons = std::move(kept);
    result.quadCount = 0U;
    result.triangleCount = 0U;
    result.nGonCount = 0U;
    for (const auto& polygon : result.polygons) {
        if (polygon.size() == 3U) {
            ++result.triangleCount;
        } else if (polygon.size() == 4U) {
            ++result.quadCount;
        } else {
            ++result.nGonCount;
        }
    }
    result.nonQuadCount = result.triangleCount + result.nGonCount;
    diagnostic = removedPolygonCount == 0U
        ? "Core extraction already exposed an open boundary."
        : "Removed oppositely oriented QuadExtractor closure faces to restore the experimental Core boundary.";
    return true;
}
CoreBoundaryDescriptor extractCoreBoundary(
    const RemeshInput& input,
    CoreRemeshResult& result,
    double epsilon)
{
    (void)input;
    CoreBoundaryDescriptor boundary;
    std::map<EdgeKey, std::size_t> edgeUse;
    std::map<EdgeKey, EdgeKey> directed;
    std::map<std::size_t, std::vector<std::size_t>> faceAdjacency;
    std::map<EdgeKey, std::vector<std::size_t>> edgeFaces;
    for (std::size_t faceId = 0U;
         faceId < result.polygons.size();
         ++faceId) {
        const ResultPolygon& polygon = result.polygons[faceId];
        if (polygon.vertexIndices.size() < 3U) {
            boundary.status = CoreBoundaryStatus::InvalidGeometry;
            boundary.diagnosticMessage = "Core polygon has fewer than 3 vertices.";
            return boundary;
        }
        for (std::size_t corner = 0U;
             corner < polygon.vertexIndices.size();
             ++corner) {
            const std::size_t first = polygon.vertexIndices[corner];
            const std::size_t second =
                polygon.vertexIndices[
                    (corner + 1U) % polygon.vertexIndices.size()];
            if (first >= result.vertices.size() ||
                second >= result.vertices.size() || first == second) {
                boundary.status = CoreBoundaryStatus::InvalidGeometry;
                boundary.diagnosticMessage =
                    "Core polygon has an invalid edge.";
                return boundary;
            }
            const EdgeKey key = edgeKey(first, second);
            ++edgeUse[key];
            directed[key] = {first, second};
            edgeFaces[key].push_back(faceId);
        }
    }
    for (const auto& [edge, faces] : edgeFaces) {
        if (faces.size() > 2U) {
            boundary.status = CoreBoundaryStatus::NonManifold;
            boundary.diagnosticMessage =
                "Core result contains a non-manifold edge.";
            return boundary;
        }
        if (faces.size() == 2U) {
            faceAdjacency[faces[0]].push_back(faces[1]);
            faceAdjacency[faces[1]].push_back(faces[0]);
        }
    }
    std::set<std::size_t> unvisitedFaces;
    for (std::size_t face = 0U; face < result.polygons.size(); ++face) {
        unvisitedFaces.insert(face);
    }
    while (!unvisitedFaces.empty()) {
        ++result.connectedComponentCount;
        std::queue<std::size_t> pending;
        pending.push(*unvisitedFaces.begin());
        unvisitedFaces.erase(unvisitedFaces.begin());
        while (!pending.empty()) {
            const std::size_t face = pending.front();
            pending.pop();
            for (const std::size_t adjacent : faceAdjacency[face]) {
                if (unvisitedFaces.erase(adjacent) != 0U) {
                    pending.push(adjacent);
                }
            }
        }
    }
    if (result.connectedComponentCount != 1U) {
        boundary.status = CoreBoundaryStatus::DisconnectedCore;
        boundary.diagnosticMessage =
            "Core result contains multiple polygon components.";
        return boundary;
    }

    std::set<EdgeKey> boundaryEdges;
    std::map<std::size_t, std::vector<std::size_t>> adjacency;
    for (const auto& [edge, useCount] : edgeUse) {
        if (useCount == 1U) {
            boundaryEdges.insert(edge);
            adjacency[edge.first].push_back(edge.second);
            adjacency[edge.second].push_back(edge.first);
        }
    }
    if (boundaryEdges.empty()) {
        boundary.status = CoreBoundaryStatus::NoBoundary;
        boundary.diagnosticMessage = "Core result exposes no boundary.";
        return boundary;
    }
    for (auto& [vertex, neighbors] : adjacency) {
        (void)vertex;
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(
            std::unique(neighbors.begin(), neighbors.end()),
            neighbors.end());
        if (neighbors.size() != 2U) {
            ++boundary.boundaryDegreeViolationCount;
        }
    }
    if (boundary.boundaryDegreeViolationCount != 0U) {
        boundary.status = CoreBoundaryStatus::BranchedBoundary;
        boundary.diagnosticMessage =
            "Core boundary degree is not exactly 2.";
        return boundary;
    }

    std::set<EdgeKey> unused = boundaryEdges;
    std::vector<std::vector<std::size_t>> loops;
    while (!unused.empty()) {
        const std::size_t start = unused.begin()->first;
        std::size_t previous = kInvalidIndex;
        std::size_t current = start;
        std::vector<std::size_t> loop;
        std::size_t guard = 0U;
        do {
            if (++guard > boundaryEdges.size() + 1U) {
                boundary.status = CoreBoundaryStatus::OpenBoundary;
                boundary.diagnosticMessage =
                    "Core boundary traversal exceeded guard.";
                return boundary;
            }
            loop.push_back(current);
            const std::vector<std::size_t>& neighbors = adjacency[current];
            std::size_t next = neighbors.front();
            if (previous != kInvalidIndex) {
                next = neighbors.front() == previous
                    ? neighbors.back() : neighbors.front();
            } else {
                const EdgeKey firstKey = edgeKey(current, neighbors.front());
                const EdgeKey secondKey = edgeKey(current, neighbors.back());
                const bool firstForward =
                    directed[firstKey].first == current;
                const bool secondForward =
                    directed[secondKey].first == current;
                if (!firstForward && secondForward) {
                    next = neighbors.back();
                }
            }
            if (unused.erase(edgeKey(current, next)) == 0U) {
                boundary.status = CoreBoundaryStatus::OpenBoundary;
                boundary.diagnosticMessage =
                    "Core boundary traversal repeated an edge.";
                return boundary;
            }
            previous = current;
            current = next;
        } while (current != start);
        loops.push_back(std::move(loop));
    }
    boundary.boundaryLoopCount = loops.size();
    boundary.holeCount = loops.size() > 1U ? loops.size() - 1U : 0U;
    if (loops.size() != 1U) {
        boundary.status = CoreBoundaryStatus::MultipleBoundaryLoops;
        boundary.diagnosticMessage =
            "R7 Core result must expose exactly one closed boundary.";
        return boundary;
    }

    boundary.orderedVertexIds = loops.front();
    boundary.closed = true;
    std::vector<double> cumulative(boundary.orderedVertexIds.size(), 0.0);
    for (std::size_t index = 0U;
         index < boundary.orderedVertexIds.size();
         ++index) {
        const std::size_t next =
            (index + 1U) % boundary.orderedVertexIds.size();
        const double length =
            (result.vertices[boundary.orderedVertexIds[next]] -
             result.vertices[boundary.orderedVertexIds[index]]).length();
        if (!std::isfinite(length) || length <= epsilon) {
            boundary.status = CoreBoundaryStatus::InvalidGeometry;
            boundary.diagnosticMessage =
                "Core boundary contains a zero-length edge.";
            return boundary;
        }
        boundary.totalArcLength += length;
        if (next != 0U) {
            cumulative[next] = boundary.totalArcLength;
        }
    }
    if (!(boundary.totalArcLength > epsilon)) {
        boundary.status = CoreBoundaryStatus::InvalidGeometry;
        boundary.diagnosticMessage = "Core boundary has zero arc length.";
        return boundary;
    }
    boundary.vertices.reserve(boundary.orderedVertexIds.size());
    for (std::size_t index = 0U;
         index < boundary.orderedVertexIds.size();
         ++index) {
        const std::size_t previous =
            (index + boundary.orderedVertexIds.size() - 1U) %
            boundary.orderedVertexIds.size();
        const std::size_t next =
            (index + 1U) % boundary.orderedVertexIds.size();
        const std::size_t vertexId = boundary.orderedVertexIds[index];
        CoreBoundaryVertexDescriptor descriptor;
        descriptor.coreVertexId = vertexId;
        descriptor.orderedBoundaryIndex = index;
        descriptor.position = result.vertices[vertexId];
        descriptor.tangent =
            (result.vertices[boundary.orderedVertexIds[next]] -
             result.vertices[boundary.orderedVertexIds[previous]])
                .normalized(epsilon);
        descriptor.normalizedArcLength =
            cumulative[index] / boundary.totalArcLength;
        descriptor.surface = result.sourceMappings[vertexId];
        if (!descriptor.surface.valid || !descriptor.tangent.finite()) {
            boundary.status = CoreBoundaryStatus::InvalidGeometry;
            boundary.diagnosticMessage =
                "Core boundary surface mapping is incomplete.";
            return boundary;
        }
        boundary.vertices.push_back(descriptor);
    }
    boundary.status = CoreBoundaryStatus::Success;
    boundary.diagnosticMessage =
        "One ordered, closed, manifold Core boundary extracted.";
    return boundary;
}

}  // namespace

CoreRemeshResult ExperimentalCoreRemeshGenerator::generate(
    const RemeshInput& input,
    const RegionComponent& component,
    const ExperimentalCoreRemeshSettings& settings) const noexcept
{
    CoreRemeshResult result;
    result.componentId = component.componentId;
    const Clock::time_point totalStart = Clock::now();
    const auto fail = [&](CoreGenerationStatus status,
                          const std::string& message) {
        result.status = status;
        result.diagnosticMessage = message;
        result.signature = coreRemeshResultSignature(result);
        result.timings.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return result;
    };
    try {
        std::string inputDiagnostic;
        if (!input.valid(&inputDiagnostic) ||
            component.coreFaceIndices.empty()) {
            return fail(
                CoreGenerationStatus::InvalidInput,
                "R7 Core generation input is invalid: " +
                    inputDiagnostic);
        }

        const Clock::time_point patchStart = Clock::now();
        std::vector<std::size_t> selectedFaces =
            insetCoreFaces(input, component, settings.insetSourceFaceRings);
        TriangulatedPatch patch;
        std::string diagnostic;
        bool built = false;
        if (selectedFaces.size() >= settings.minimumInsetFaceCount) {
            built = buildCorePatch(
                input, component, selectedFaces, patch, diagnostic);
            result.usedInsetDomain = built;
        }
        if (!built) {
            selectedFaces = component.coreFaceIndices;
            std::sort(selectedFaces.begin(), selectedFaces.end());
            diagnostic.clear();
            built = buildCorePatch(
                input, component, selectedFaces, patch, diagnostic);
            result.usedInsetDomain = false;
        }
        result.timings.patchBuildMilliseconds =
            elapsedMilliseconds(patchStart);
        if (!built) {
            return fail(
                CoreGenerationStatus::PatchBuildFailed, diagnostic);
        }
        result.sourceFaceIndices = selectedFaces;

        const DirectionFieldData direction = makeDirectionField(input);
        const DensityFieldData density = makeDensityField(input);
        AutoRemesherAdapter adapter;
        AutoRemesherInput solverInput;
        ParameterizationResult parameterization;
        QuadPatchResult quadResult;

        if (!adapter.buildInput(
                patch, direction, density, solverInput, diagnostic)) {
            return fail(
                CoreGenerationStatus::InvalidInput, diagnostic);
        }
        const Clock::time_point parameterizationStart = Clock::now();
        if (!adapter.parameterize(
                solverInput, parameterization, diagnostic)) {
            result.timings.parameterizationMilliseconds =
                elapsedMilliseconds(parameterizationStart);
            return fail(
                CoreGenerationStatus::ParameterizationFailed, diagnostic);
        }
        result.timings.parameterizationMilliseconds =
            elapsedMilliseconds(parameterizationStart);

        const Clock::time_point extractionStart = Clock::now();
        if (!adapter.extractQuads(
                solverInput, parameterization, quadResult, diagnostic)) {
            result.timings.extractionMilliseconds =
                elapsedMilliseconds(extractionStart);
            return fail(
                CoreGenerationStatus::ExtractionFailed, diagnostic);
        }
        result.timings.extractionMilliseconds =
            elapsedMilliseconds(extractionStart);

        SurfaceConformer conformer;
        SurfaceConformerSettings conformSettings = conformer.settings();
        conformSettings.projectResultBoundaryToSourceBoundary =
            settings.projectBoundaryToSourcePolyline;
        conformSettings.lockResultBoundaryDuringRelax = true;
        conformSettings.geometryEpsilon = settings.geometryEpsilon;
        conformer.setSettings(conformSettings);
        const Clock::time_point conformationStart = Clock::now();
        if (!conformer.conform(patch, quadResult, diagnostic)) {
            result.timings.conformationMilliseconds =
                elapsedMilliseconds(conformationStart);
            return fail(
                CoreGenerationStatus::SurfaceConformationFailed,
                diagnostic);
        }
        result.timings.conformationMilliseconds =
            elapsedMilliseconds(conformationStart);

        const Clock::time_point validationStart = Clock::now();
        std::size_t removedClosurePolygons = 0U;
        if (!restoreOpenCoreBoundary(
                quadResult,
                settings.geometryEpsilon,
                removedClosurePolygons,
                diagnostic)) {
            result.timings.validationMilliseconds =
                elapsedMilliseconds(validationStart);
            return fail(CoreGenerationStatus::ValidationFailed, diagnostic);
        }
        result.timings.validationMilliseconds =
            elapsedMilliseconds(validationStart);

        result.rawVertices.reserve(quadResult.rawVertices.size());
        for (const MPoint& vertex : quadResult.rawVertices) {
            result.rawVertices.push_back(toPortable(vertex));
        }
        result.vertices.reserve(quadResult.conformedVertices.size());
        for (const MPoint& vertex : quadResult.conformedVertices) {
            result.vertices.push_back(toPortable(vertex));
        }
        for (const std::vector<std::size_t>& source :
             quadResult.polygons) {
            ResultPolygon polygon;
            polygon.vertexIndices = source;
            polygon.type = source.size() == 3U
                ? PolygonType::Triangle
                : (source.size() == 4U
                    ? PolygonType::Quad : PolygonType::NGon);
            polygon.region = PolygonRegion::Core;
            polygon.triangleReason = TriangleReason::SolverFallback;
            result.polygons.push_back(std::move(polygon));
        }
        result.quadCount = quadResult.quadCount;
        result.triangleCount = quadResult.triangleCount;
        result.nGonCount = quadResult.nGonCount;

        const Clock::time_point mappingStart = Clock::now();
        result.sourceMappings.resize(result.vertices.size());
        for (std::size_t vertexId = 0U;
             vertexId < result.vertices.size();
             ++vertexId) {
            SourceId preferredFace = kInvalidSourceId;
            if (vertexId < quadResult.sourceMappings.size()) {
                preferredFace =
                    quadResult.sourceMappings[vertexId].sourceFaceId;
            }
            result.sourceMappings[vertexId] = mapToSource(
                input,
                result.vertices[vertexId],
                preferredFace,
                result.sourceFaceIndices,
                settings.geometryEpsilon);
            if (!result.sourceMappings[vertexId].valid) {
                return fail(
                    CoreGenerationStatus::SurfaceConformationFailed,
                    "Core vertex could not be mapped to a local Source triangle.");
            }
        }
        result.boundary = extractCoreBoundary(
            input, result, settings.geometryEpsilon);
        result.timings.boundaryMappingMilliseconds =
            elapsedMilliseconds(mappingStart);
        if (!result.boundary.success()) {
            result.status = CoreGenerationStatus::CoreBoundaryInvalid;
            result.diagnosticMessage =
                result.boundary.diagnosticMessage;
            result.signature = coreRemeshResultSignature(result);
            result.timings.totalMilliseconds =
                elapsedMilliseconds(totalStart);
            return result;
        }

        result.status = CoreGenerationStatus::Success;
        std::ostringstream message;
        message
            << "R7 experimental Core generated; vertices="
            << result.vertices.size()
            << "; polygons=" << result.polygons.size()
            << "; boundary=" << result.boundary.vertices.size()
            << "; inset=" << (result.usedInsetDomain ? "yes" : "no")
            << "; removed closure polygons=" << removedClosurePolygons
            << "; source correspondence=complete.";
        result.diagnosticMessage = message.str();
        result.signature = coreRemeshResultSignature(result);
        result.timings.totalMilliseconds =
            elapsedMilliseconds(totalStart);
        return result;
    } catch (const std::exception& exception) {
        return fail(
            CoreGenerationStatus::InvalidInput,
            std::string("R7 Core generation exception: ") +
                exception.what());
    } catch (...) {
        return fail(
            CoreGenerationStatus::InvalidInput,
            "R7 Core generation raised an unknown exception.");
    }
}

}  // namespace directional_retopo::solver
