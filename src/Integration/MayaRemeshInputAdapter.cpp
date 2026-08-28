#include "Integration/MayaRemeshInputAdapter.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace directional_retopo {
namespace {

solver::Vec3 toPortable(const MPoint& value) noexcept
{
    return {value.x, value.y, value.z};
}

solver::Vec3 toPortable(const MVector& value) noexcept
{
    return {value.x, value.y, value.z};
}

bool validIndex(int value, std::size_t size) noexcept
{
    return value >= 0 && static_cast<std::size_t>(value) < size;
}

std::vector<int> buildTransitionDepths(
    const PaintRegionComponent& component,
    const std::vector<MeshFaceTopology>& faces)
{
    std::vector<int> depths(faces.size(), -1);
    std::unordered_set<int> componentFaces;
    componentFaces.reserve(component.allFaceIds.size());
    for (const int faceId : component.allFaceIds) {
        if (validIndex(faceId, faces.size())) {
            componentFaces.insert(faceId);
        }
    }

    std::queue<int> frontier;
    for (const int faceId : component.coreFaceIds) {
        if (componentFaces.find(faceId) == componentFaces.end()) {
            continue;
        }
        depths[static_cast<std::size_t>(faceId)] = 0;
        frontier.push(faceId);
    }

    while (!frontier.empty()) {
        const int faceId = frontier.front();
        frontier.pop();
        const int nextDepth = depths[static_cast<std::size_t>(faceId)] + 1;
        for (const int neighborId : faces[static_cast<std::size_t>(faceId)].adjacentFaceIds) {
            if (componentFaces.find(neighborId) == componentFaces.end() ||
                depths[static_cast<std::size_t>(neighborId)] >= 0) {
                continue;
            }
            depths[static_cast<std::size_t>(neighborId)] = nextDepth;
            frontier.push(neighborId);
        }
    }
    return depths;
}

}  // namespace

bool MayaRemeshInputAdapter::build(
    const MeshTopologyCache& topology,
    const PaintRegionData& region,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField,
    const solver::RemeshSettings& settings,
    solver::RemeshInput& output,
    std::string& diagnostic)
{
    output = solver::RemeshInput();
    diagnostic.clear();

    const std::vector<MPoint>& positions = topology.worldVertexPositions();
    const std::vector<MeshEdgeTopology>& edges = topology.edges();
    const std::vector<MeshFaceTopology>& faces = topology.faces();
    if (positions.empty() || faces.empty()) {
        diagnostic = "The Maya topology cache is empty.";
        return false;
    }
    if (directionField.perFace.size() != faces.size() ||
        densityField.perFace.size() != faces.size()) {
        diagnostic = "Direction or density field size does not match the cached Maya face count.";
        return false;
    }

    solver::SourceMeshSnapshot& snapshot = output.sourceMesh;
    snapshot.vertices.resize(positions.size());
    snapshot.edges.resize(edges.size());
    snapshot.faces.resize(faces.size());

    for (std::size_t vertexIndex = 0; vertexIndex < positions.size(); ++vertexIndex) {
        solver::SourceVertex& destination = snapshot.vertices[vertexIndex];
        destination.position = toPortable(positions[vertexIndex]);
        destination.sourceVertexId = static_cast<solver::SourceId>(vertexIndex);
    }

    for (std::size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
        const MeshEdgeTopology& source = edges[edgeIndex];
        if (!validIndex(source.vertexIds[0], positions.size()) ||
            !validIndex(source.vertexIds[1], positions.size())) {
            diagnostic = "The Maya topology cache contains an invalid edge vertex ID.";
            return false;
        }
        solver::SourceEdge& destination = snapshot.edges[edgeIndex];
        destination.vertexIndices = {
            static_cast<std::size_t>(source.vertexIds[0]),
            static_cast<std::size_t>(source.vertexIds[1])};
        destination.sourceEdgeId = static_cast<solver::SourceId>(edgeIndex);
        destination.length = source.worldLength;
        destination.originalMeshBoundary = source.originalMeshBoundary;
        for (const int faceId : source.faceIds) {
            if (validIndex(faceId, faces.size())) {
                destination.faceIndices.push_back(static_cast<std::size_t>(faceId));
            }
        }
        snapshot.vertices[destination.vertexIndices[0]].edgeIndices.push_back(edgeIndex);
        snapshot.vertices[destination.vertexIndices[1]].edgeIndices.push_back(edgeIndex);
        snapshot.vertices[destination.vertexIndices[0]].adjacentVertexIndices.push_back(
            destination.vertexIndices[1]);
        snapshot.vertices[destination.vertexIndices[1]].adjacentVertexIndices.push_back(
            destination.vertexIndices[0]);
    }

    std::vector<solver::Vec3> accumulatedVertexNormals(positions.size());
    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const MeshFaceTopology& source = faces[faceIndex];
        solver::SourceFace& destination = snapshot.faces[faceIndex];
        destination.sourceFaceId = static_cast<solver::SourceId>(faceIndex);
        destination.center = toPortable(source.worldCenter);
        destination.normal = toPortable(source.worldNormal).normalized();
        destination.geometryValid = source.worldGeometryValid;

        for (const int vertexId : source.vertexIds) {
            if (!validIndex(vertexId, positions.size())) {
                diagnostic = "The Maya topology cache contains an invalid face vertex ID.";
                return false;
            }
            const std::size_t vertexIndex = static_cast<std::size_t>(vertexId);
            destination.vertexIndices.push_back(vertexIndex);
            snapshot.vertices[vertexIndex].faceIndices.push_back(faceIndex);
            accumulatedVertexNormals[vertexIndex] += destination.normal;
        }
        for (const int edgeId : source.edgeIds) {
            if (!validIndex(edgeId, edges.size())) {
                diagnostic = "The Maya topology cache contains an invalid face edge ID.";
                return false;
            }
            destination.edgeIndices.push_back(static_cast<std::size_t>(edgeId));
        }
        for (const int adjacentFaceId : source.adjacentFaceIds) {
            if (validIndex(adjacentFaceId, faces.size())) {
                destination.adjacentFaceIndices.push_back(
                    static_cast<std::size_t>(adjacentFaceId));
            }
        }
        for (const std::array<int, 3>& triangle : source.triangleVertexIds) {
            if (!validIndex(triangle[0], positions.size()) ||
                !validIndex(triangle[1], positions.size()) ||
                !validIndex(triangle[2], positions.size())) {
                diagnostic = "The Maya topology cache contains an invalid triangle vertex ID.";
                return false;
            }
            const std::size_t triangleIndex = snapshot.triangles.size();
            destination.triangleIndices.push_back(triangleIndex);
            snapshot.triangles.push_back({
                {static_cast<std::size_t>(triangle[0]),
                 static_cast<std::size_t>(triangle[1]),
                 static_cast<std::size_t>(triangle[2])},
                faceIndex});
        }
    }

    for (std::size_t vertexIndex = 0; vertexIndex < snapshot.vertices.size(); ++vertexIndex) {
        snapshot.vertices[vertexIndex].normal = accumulatedVertexNormals[vertexIndex].normalized();
    }

    output.directionField.resize(faces.size());
    output.densityField.resize(faces.size());
    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const FaceDirectionField& sourceDirection = directionField.perFace[faceIndex];
        solver::FaceDirection& destinationDirection = output.directionField[faceIndex];
        destinationDirection.normal = toPortable(sourceDirection.normal);
        destinationDirection.uDirection = toPortable(sourceDirection.uDirection);
        destinationDirection.vDirection = toPortable(sourceDirection.vDirection);
        destinationDirection.paintConstraintWeight = sourceDirection.constraintWeight;
        destinationDirection.topologyGuidanceWeight = sourceDirection.topologyGuidanceWeight;
        destinationDirection.valid = sourceDirection.valid;

        const directional_retopo::FaceDensity& sourceDensity = densityField.perFace[faceIndex];
        solver::FaceDensity& destinationDensity = output.densityField[faceIndex];
        destinationDensity.requestedTargetEdgeLength =
            sourceDensity.baseTargetEdgeLength > 0.0
                ? sourceDensity.baseTargetEdgeLength
                : sourceDensity.targetEdgeLength;
        destinationDensity.effectiveTargetEdgeLength = sourceDensity.targetEdgeLength;
        destinationDensity.scaleU = sourceDensity.scaleU;
        destinationDensity.scaleV = sourceDensity.scaleV;
        destinationDensity.curvatureConstrained = sourceDensity.curvatureLimited;
        destinationDensity.valid = sourceDensity.valid;
    }

    output.components.reserve(region.components.size());
    for (std::size_t componentIndex = 0;
         componentIndex < region.components.size();
         ++componentIndex) {
        const PaintRegionComponent& source = region.components[componentIndex];
        solver::RegionComponent destination;
        destination.componentId = componentIndex;
        const auto appendFaces = [&faces](
                                     const std::vector<int>& sourceIds,
                                     std::vector<std::size_t>& destinationIds) {
            for (const int faceId : sourceIds) {
                if (validIndex(faceId, faces.size())) {
                    destinationIds.push_back(static_cast<std::size_t>(faceId));
                }
            }
        };
        appendFaces(source.coreFaceIds, destination.coreFaceIndices);
        appendFaces(source.transitionFaceIds, destination.transitionFaceIndices);
        appendFaces(source.allFaceIds, destination.allFaceIndices);
        destination.transitionRingDepthByFace = buildTransitionDepths(source, faces);

        destination.fixedBoundaryLoops.reserve(source.boundaryLoops.size());
        for (const BoundaryLoop& sourceLoop : source.boundaryLoops) {
            solver::OrderedBoundaryLoop loop;
            loop.closed = sourceLoop.closed;
            loop.touchesOriginalMeshBoundary = sourceLoop.touchesOriginalMeshBoundary;
            for (const int vertexId : sourceLoop.vertexIds) {
                if (!validIndex(vertexId, positions.size())) {
                    diagnostic = "A Paint Region boundary contains an invalid vertex ID.";
                    return false;
                }
                loop.vertexIndices.push_back(static_cast<std::size_t>(vertexId));
                loop.sourceVertexIds.push_back(static_cast<solver::SourceId>(vertexId));
            }
            for (const int edgeId : sourceLoop.edgeIds) {
                if (!validIndex(edgeId, edges.size())) {
                    diagnostic = "A Paint Region boundary contains an invalid edge ID.";
                    return false;
                }
                loop.edgeIndices.push_back(static_cast<std::size_t>(edgeId));
                loop.sourceEdgeIds.push_back(static_cast<solver::SourceId>(edgeId));
            }
            destination.fixedBoundaryLoops.push_back(std::move(loop));
        }
        output.components.push_back(std::move(destination));
    }

    output.settings = settings;
    if (!output.valid(&diagnostic)) {
        return false;
    }
    return true;
}

}  // namespace directional_retopo
