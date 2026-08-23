#include "Mesh/MeshTopologyCache.h"

#include <maya/MFnMesh.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshEdge.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MPointArray.h>

#include <algorithm>
#include <cmath>

namespace directional_retopo {

MStatus MeshTopologyCache::setTarget(const MDagPath& meshPath)
{
    clear();

    MStatus status;
    MFnMesh mesh(meshPath, &status);
    if (!status) {
        return status;
    }

    meshPath_ = meshPath;
    meshHandle_ = MObjectHandle(meshPath.node());
    topologyDirty_ = true;
    geometryDirty_ = true;
    status = rebuildTopology();
    if (!status) {
        clear();
        return status;
    }
    status = updateWorldGeometry();
    if (!status) {
        clear();
    }
    return status;
}

void MeshTopologyCache::clear() noexcept
{
    meshPath_ = MDagPath();
    meshHandle_ = MObjectHandle();
    worldVertexPositions_.clear();
    edges_.clear();
    faces_.clear();
    vertexEdgeIds_.clear();
    cachedInclusiveMatrix_ = MMatrix();
    cachedVertexCount_ = -1;
    cachedEdgeCount_ = -1;
    cachedFaceCount_ = -1;
    topologyDirty_ = true;
    geometryDirty_ = true;
}

bool MeshTopologyCache::hasTarget() const noexcept
{
    return meshHandle_.isValid() && meshHandle_.isAlive() && meshPath_.isValid();
}

MStatus MeshTopologyCache::ensureCurrent()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status = validateTopologyCounts();
    if (!status) {
        return status;
    }
    if (topologyDirty_) {
        status = rebuildTopology();
        if (!status) {
            return status;
        }
    }

    // Formal solves happen only on mouse release. Reading current points here
    // keeps deformed vertices and transforms current without reparsing topology.
    geometryDirty_ = true;
    return updateWorldGeometry();
}

void MeshTopologyCache::invalidateTopology() noexcept
{
    topologyDirty_ = true;
    geometryDirty_ = true;
}

void MeshTopologyCache::invalidateGeometry() noexcept
{
    geometryDirty_ = true;
}

const MDagPath& MeshTopologyCache::meshPath() const noexcept
{
    return meshPath_;
}

const std::vector<MPoint>& MeshTopologyCache::worldVertexPositions() const noexcept
{
    return worldVertexPositions_;
}

const std::vector<MeshEdgeTopology>& MeshTopologyCache::edges() const noexcept
{
    return edges_;
}

const std::vector<MeshFaceTopology>& MeshTopologyCache::faces() const noexcept
{
    return faces_;
}

const std::vector<std::vector<int>>& MeshTopologyCache::vertexEdgeIds() const noexcept
{
    return vertexEdgeIds_;
}

std::uint64_t MeshTopologyCache::topologyBuildCount() const noexcept
{
    return topologyBuildCount_;
}

std::uint64_t MeshTopologyCache::geometryBuildCount() const noexcept
{
    return geometryBuildCount_;
}

MStatus MeshTopologyCache::rebuildTopology()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    const int vertexCount = mesh.numVertices(&status);
    if (!status || vertexCount < 0) {
        return status ? MS::kFailure : status;
    }
    const int edgeCount = mesh.numEdges(&status);
    if (!status || edgeCount < 0) {
        return status ? MS::kFailure : status;
    }
    const int faceCount = mesh.numPolygons(&status);
    if (!status || faceCount < 0) {
        return status ? MS::kFailure : status;
    }

    std::vector<MeshEdgeTopology> newEdges(static_cast<std::size_t>(edgeCount));
    std::vector<MeshFaceTopology> newFaces(static_cast<std::size_t>(faceCount));
    std::vector<std::vector<int>> newVertexEdgeIds(
        static_cast<std::size_t>(vertexCount));

    MIntArray faceTriangleCounts;
    MIntArray triangleVertexIds;
    status = mesh.getTriangles(faceTriangleCounts, triangleVertexIds);
    if (!status || faceTriangleCounts.length() != static_cast<unsigned int>(faceCount)) {
        return status ? MS::kFailure : status;
    }
    std::size_t triangleVertexOffset = 0;
    for (int faceId = 0; faceId < faceCount; ++faceId) {
        const int triangleCount = faceTriangleCounts[static_cast<unsigned int>(faceId)];
        if (triangleCount < 0) {
            return MS::kFailure;
        }
        MeshFaceTopology& face = newFaces[static_cast<std::size_t>(faceId)];
        face.triangleVertexIds.reserve(static_cast<std::size_t>(triangleCount));
        for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
            if (triangleVertexOffset + 3U > triangleVertexIds.length()) {
                return MS::kFailure;
            }
            std::array<int, 3> triangle = {
                triangleVertexIds[static_cast<unsigned int>(triangleVertexOffset)],
                triangleVertexIds[static_cast<unsigned int>(triangleVertexOffset + 1U)],
                triangleVertexIds[static_cast<unsigned int>(triangleVertexOffset + 2U)]};
            triangleVertexOffset += 3U;
            if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
                triangle[0] >= vertexCount || triangle[1] >= vertexCount ||
                triangle[2] >= vertexCount) {
                return MS::kFailure;
            }
            face.triangleVertexIds.push_back(triangle);
        }
    }
    if (triangleVertexOffset != triangleVertexIds.length()) {
        return MS::kFailure;
    }

    MObject component = MObject::kNullObj;
    MItMeshEdge edgeIterator(meshPath_, component, &status);
    if (!status) {
        return status;
    }
    for (; !edgeIterator.isDone(&status) && status; edgeIterator.next()) {
        const int edgeId = edgeIterator.index(&status);
        if (!status || edgeId < 0 || edgeId >= edgeCount) {
            return status ? MS::kFailure : status;
        }

        int vertexIds[2] = {-1, -1};
        status = mesh.getEdgeVertices(edgeId, vertexIds);
        if (!status || vertexIds[0] < 0 || vertexIds[1] < 0 ||
            vertexIds[0] >= vertexCount || vertexIds[1] >= vertexCount) {
            return status ? MS::kFailure : status;
        }

        MeshEdgeTopology& edge = newEdges[static_cast<std::size_t>(edgeId)];
        edge.vertexIds = {vertexIds[0], vertexIds[1]};
        newVertexEdgeIds[static_cast<std::size_t>(vertexIds[0])].push_back(edgeId);
        newVertexEdgeIds[static_cast<std::size_t>(vertexIds[1])].push_back(edgeId);

        MIntArray connectedFaces;
        (void)edgeIterator.getConnectedFaces(connectedFaces, &status);
        if (!status) {
            return status;
        }
        edge.faceIds.reserve(connectedFaces.length());
        for (unsigned int index = 0; index < connectedFaces.length(); ++index) {
            const int faceId = connectedFaces[index];
            if (faceId >= 0 && faceId < faceCount) {
                edge.faceIds.push_back(faceId);
            }
        }
        std::sort(edge.faceIds.begin(), edge.faceIds.end());
        edge.faceIds.erase(
            std::unique(edge.faceIds.begin(), edge.faceIds.end()),
            edge.faceIds.end());
        edge.originalMeshBoundary = edge.faceIds.size() == 1;
    }
    if (!status) {
        return status;
    }

    MItMeshPolygon faceIterator(meshPath_, component, &status);
    if (!status) {
        return status;
    }
    for (; !faceIterator.isDone(&status) && status; faceIterator.next()) {
        const int faceId = static_cast<int>(faceIterator.index(&status));
        if (!status || faceId < 0 || faceId >= faceCount) {
            return status ? MS::kFailure : status;
        }

        MeshFaceTopology& face = newFaces[static_cast<std::size_t>(faceId)];
        MIntArray vertexIds;
        status = faceIterator.getVertices(vertexIds);
        if (!status) {
            return status;
        }
        face.vertexIds.reserve(vertexIds.length());
        for (unsigned int index = 0; index < vertexIds.length(); ++index) {
            const int vertexId = vertexIds[index];
            if (vertexId >= 0 && vertexId < vertexCount) {
                face.vertexIds.push_back(vertexId);
            }
        }

        MIntArray edgeIds;
        status = faceIterator.getEdges(edgeIds);
        if (!status) {
            return status;
        }
        face.edgeIds.reserve(edgeIds.length());
        for (unsigned int index = 0; index < edgeIds.length(); ++index) {
            const int edgeId = edgeIds[index];
            if (edgeId >= 0 && edgeId < edgeCount) {
                face.edgeIds.push_back(edgeId);
            }
        }
    }
    if (!status) {
        return status;
    }

    for (int faceId = 0; faceId < faceCount; ++faceId) {
        MeshFaceTopology& face = newFaces[static_cast<std::size_t>(faceId)];
        for (const int edgeId : face.edgeIds) {
            const MeshEdgeTopology& edge = newEdges[static_cast<std::size_t>(edgeId)];
            for (const int adjacentFaceId : edge.faceIds) {
                if (adjacentFaceId != faceId) {
                    face.adjacentFaceIds.push_back(adjacentFaceId);
                }
            }
        }
        std::sort(face.adjacentFaceIds.begin(), face.adjacentFaceIds.end());
        face.adjacentFaceIds.erase(
            std::unique(face.adjacentFaceIds.begin(), face.adjacentFaceIds.end()),
            face.adjacentFaceIds.end());
    }

    edges_ = std::move(newEdges);
    faces_ = std::move(newFaces);
    vertexEdgeIds_ = std::move(newVertexEdgeIds);
    cachedVertexCount_ = vertexCount;
    cachedEdgeCount_ = edgeCount;
    cachedFaceCount_ = faceCount;
    topologyDirty_ = false;
    geometryDirty_ = true;
    ++topologyBuildCount_;
    return MS::kSuccess;
}

MStatus MeshTopologyCache::updateWorldGeometry()
{
    if (!hasTarget()) {
        return MS::kFailure;
    }

    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    MPointArray objectPoints;
    status = mesh.getPoints(objectPoints, MSpace::kObject);
    if (!status) {
        return status;
    }
    if (objectPoints.length() != vertexEdgeIds_.size()) {
        topologyDirty_ = true;
        status = rebuildTopology();
        return status ? updateWorldGeometry() : status;
    }

    const MMatrix inclusiveMatrix = meshPath_.inclusiveMatrix(&status);
    if (!status) {
        return status;
    }
    worldVertexPositions_.resize(objectPoints.length());
    for (unsigned int index = 0; index < objectPoints.length(); ++index) {
        worldVertexPositions_[index] = objectPoints[index] * inclusiveMatrix;
    }

    for (MeshEdgeTopology& edge : edges_) {
        const int first = edge.vertexIds[0];
        const int second = edge.vertexIds[1];
        if (first < 0 || second < 0 ||
            static_cast<std::size_t>(first) >= worldVertexPositions_.size() ||
            static_cast<std::size_t>(second) >= worldVertexPositions_.size()) {
            return MS::kFailure;
        }
        edge.worldLength =
            (worldVertexPositions_[static_cast<std::size_t>(second)] -
             worldVertexPositions_[static_cast<std::size_t>(first)])
                .length();
        if (!std::isfinite(edge.worldLength)) {
            return MS::kFailure;
        }
    }

    for (MeshFaceTopology& face : faces_) {
        face.worldCenter = MPoint::origin;
        face.worldNormal = MVector::zero;
        face.worldGeometryValid = false;
        if (face.vertexIds.size() < 3) {
            continue;
        }

        bool validVertices = true;
        double centerX = 0.0;
        double centerY = 0.0;
        double centerZ = 0.0;
        for (const int vertexId : face.vertexIds) {
            if (vertexId < 0 ||
                static_cast<std::size_t>(vertexId) >= worldVertexPositions_.size()) {
                validVertices = false;
                break;
            }
            const MPoint& vertex =
                worldVertexPositions_[static_cast<std::size_t>(vertexId)];
            centerX += vertex.x;
            centerY += vertex.y;
            centerZ += vertex.z;
        }
        if (!validVertices) {
            continue;
        }
        const double inverseVertexCount =
            1.0 / static_cast<double>(face.vertexIds.size());
        face.worldCenter = MPoint(
            centerX * inverseVertexCount,
            centerY * inverseVertexCount,
            centerZ * inverseVertexCount);

        MVector areaNormal = MVector::zero;
        for (std::size_t index = 0; index < face.vertexIds.size(); ++index) {
            const std::size_t nextIndex = (index + 1) % face.vertexIds.size();
            const MPoint& current = worldVertexPositions_[static_cast<std::size_t>(
                face.vertexIds[index])];
            const MPoint& next = worldVertexPositions_[static_cast<std::size_t>(
                face.vertexIds[nextIndex])];
            areaNormal += (current - face.worldCenter) ^ (next - face.worldCenter);
        }
        if (areaNormal.length() <= 1.0e-12) {
            continue;
        }
        areaNormal.normalize();
        face.worldNormal = areaNormal;
        face.worldGeometryValid = true;
    }

    cachedInclusiveMatrix_ = inclusiveMatrix;
    geometryDirty_ = false;
    ++geometryBuildCount_;
    return MS::kSuccess;
}

MStatus MeshTopologyCache::validateTopologyCounts()
{
    MStatus status;
    MFnMesh mesh(meshPath_, &status);
    if (!status) {
        return status;
    }
    const int vertexCount = mesh.numVertices(&status);
    if (!status) {
        return status;
    }
    const int edgeCount = mesh.numEdges(&status);
    if (!status) {
        return status;
    }
    const int faceCount = mesh.numPolygons(&status);
    if (!status) {
        return status;
    }
    if (vertexCount != cachedVertexCount_ || edgeCount != cachedEdgeCount_ ||
        faceCount != cachedFaceCount_) {
        invalidateTopology();
    }
    return MS::kSuccess;
}

}  // namespace directional_retopo
