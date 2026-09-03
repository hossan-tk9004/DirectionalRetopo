#include "Solver/LocalMutablePatchMesh.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace directional_retopo::solver {
namespace {

constexpr double kGeometryEpsilon = 1.0e-12;

template <typename T>
bool contains(const std::vector<T>& values, T value)
{
    return std::find(values.begin(), values.end(), value) != values.end();
}

template <typename T>
void appendUnique(std::vector<T>& values, T value)
{
    if (!contains(values, value)) {
        values.push_back(value);
    }
}

template <typename T>
void eraseValue(std::vector<T>& values, T value)
{
    values.erase(std::remove(values.begin(), values.end(), value), values.end());
}

template <typename T>
void normalizeIds(std::vector<T>& values)
{
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

std::pair<MutableVertexId, MutableVertexId> edgeKey(
    MutableVertexId first,
    MutableVertexId second)
{
    return std::minmax(first, second);
}

Vec3 polygonAreaVector(
    const std::vector<MutableVertexId>& polygon,
    const std::vector<MutableVertex>& vertices)
{
    Vec3 area;
    for (std::size_t index = 0U; index < polygon.size(); ++index) {
        const Vec3& current = vertices[polygon[index]].position;
        const Vec3& next = vertices[polygon[(index + 1U) % polygon.size()]].position;
        area += current.cross(next);
    }
    return area * 0.5;
}

std::vector<MutableVertexId> compactPolygon(
    const std::vector<MutableVertexId>& polygon)
{
    std::vector<MutableVertexId> result;
    for (const MutableVertexId vertexId : polygon) {
        if (result.empty() || result.back() != vertexId) {
            result.push_back(vertexId);
        }
    }
    if (result.size() > 1U && result.front() == result.back()) {
        result.pop_back();
    }
    return result;
}

std::size_t hashMix(std::size_t seed, std::uint64_t value) noexcept
{
    return seed ^ (static_cast<std::size_t>(value) + 0x9e3779b97f4a7c15ULL +
                   (seed << 6U) + (seed >> 2U));
}

std::uint64_t doubleBits(double value) noexcept
{
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

MutableEdgeId findActiveEdge(
    const std::vector<MutableEdge>& edges,
    MutableVertexId first,
    MutableVertexId second)
{
    const auto key = edgeKey(first, second);
    for (const MutableEdge& edge : edges) {
        if (!edge.deleted && edgeKey(edge.vertex0, edge.vertex1) == key) {
            return edge.id;
        }
    }
    return kInvalidIndex;
}

MutableOperationResult failure(
    MutableOperationType type,
    MutableOperationId operationId,
    MutableOperationStatus status,
    const std::string& message)
{
    MutableOperationResult result;
    result.status = status;
    result.diagnosticMessage = message;
    result.changes.id = operationId;
    result.changes.type = type;
    return result;
}

}  // namespace

LocalMutablePatchMesh LocalMutablePatchMesh::fromScaffold(
    const SourceTransitionScaffold& scaffold,
    unsigned int requestedBlendWidth,
    std::string* diagnostic)
{
    LocalMutablePatchMesh mesh;
    mesh.diagnostics_.requestedBlendWidth = requestedBlendWidth;
    if (!scaffold.success() || scaffold.fixedOuterBoundaryLoops.size() != 1U ||
        scaffold.innerInterfaceLoops.size() != 1U) {
        if (diagnostic != nullptr) {
            *diagnostic = "A successful single-boundary R4 scaffold is required.";
        }
        return mesh;
    }

    mesh.vertices_.reserve(scaffold.vertices.size());
    for (const ScaffoldVertex& source : scaffold.vertices) {
        MutableVertex vertex;
        vertex.id = source.localIndex;
        vertex.position = source.position;
        vertex.sourcePosition = source.position;
        vertex.normal = source.normal;
        vertex.fixedOuterBoundary = hasClassification(
            source.classification,
            ScaffoldVertexClassification::FixedOuterBoundary);
        vertex.innerInterface = hasClassification(
            source.classification,
            ScaffoldVertexClassification::InnerInterface);
        vertex.sourceVertexId = source.sourceVertexId;
        mesh.vertices_.push_back(std::move(vertex));
    }

    mesh.edges_.reserve(scaffold.edges.size());
    for (const ScaffoldEdge& source : scaffold.edges) {
        MutableEdge edge;
        edge.id = source.localIndex;
        edge.vertex0 = source.vertexIndices[0];
        edge.vertex1 = source.vertexIndices[1];
        edge.faceIds = source.faceIndices;
        edge.fixedOuterBoundary =
            source.classification == ScaffoldEdgeClassification::FixedOuterBoundary;
        edge.innerInterface =
            source.classification == ScaffoldEdgeClassification::InnerInterface;
        edge.sourceEdgeId = source.sourceEdgeId;
        mesh.edges_.push_back(std::move(edge));
    }

    mesh.faces_.reserve(scaffold.faces.size());
    for (const ScaffoldFace& source : scaffold.faces) {
        MutableFace face;
        face.id = source.localIndex;
        face.vertexIds = source.vertexIndices;
        face.edgeIds = source.edgeIndices;
        face.sourceFaceIds.push_back(source.sourceFaceId);
        face.minimumRingDepth = source.transitionRingDepth;
        face.maximumRingDepth = source.transitionRingDepth;
        mesh.faces_.push_back(std::move(face));
    }

    for (MutableEdge& edge : mesh.edges_) {
        appendUnique(mesh.vertices_[edge.vertex0].edgeIds, edge.id);
        appendUnique(mesh.vertices_[edge.vertex1].edgeIds, edge.id);
    }
    for (const MutableFace& face : mesh.faces_) {
        for (const MutableVertexId vertexId : face.vertexIds) {
            appendUnique(mesh.vertices_[vertexId].faceIds, face.id);
        }
    }

    mesh.fixedOuterBoundary_ = scaffold.fixedOuterBoundaryLoops.front();
    mesh.orderedInnerInterfaceVertices_ =
        scaffold.innerInterfaceLoops.front().vertexIndices;
    mesh.orderedInnerInterfaceEdges_ =
        scaffold.innerInterfaceLoops.front().edgeIndices;
    mesh.mutableVertexIdBySourceIndex_ = scaffold.localVertexIndexBySource;
    mesh.mutableEdgeIdBySourceIndex_ = scaffold.localEdgeIndexBySource;
    mesh.mutableFaceIdBySourceIndex_ = scaffold.localFaceIndexBySource;
    mesh.refreshDiagnostics();

    std::string reason;
    if (!mesh.valid(&reason)) {
        if (diagnostic != nullptr) {
            *diagnostic = reason;
        }
        return LocalMutablePatchMesh();
    }
    if (diagnostic != nullptr) {
        *diagnostic = mesh.diagnostics_.ringDepthMismatch
            ? "Mutable copy created; requested/actual ring-depth mismatch retained as a warning."
            : "Mutable copy created without altering the R4 scaffold.";
    }
    return mesh;
}

void LocalMutablePatchMesh::refreshDiagnostics()
{
    std::size_t sourceVertices = 0U;
    std::size_t activeVertices = 0U;
    std::size_t sourceEdges = 0U;
    std::size_t activeEdges = 0U;
    std::size_t sourceFaces = 0U;
    std::size_t activeFaces = 0U;
    unsigned int maximumDepth = 0U;
    double maximumDisplacement = 0.0;
    for (const MutableVertex& vertex : vertices_) {
        if (!vertex.deleted) {
            ++activeVertices;
            sourceVertices += vertex.sourceVertexId != kInvalidSourceId ? 1U : 0U;
            if (vertex.fixedOuterBoundary) {
                maximumDisplacement = std::max(
                    maximumDisplacement,
                    (vertex.position - vertex.sourcePosition).length());
            }
        }
    }
    for (const MutableEdge& edge : edges_) {
        if (!edge.deleted) {
            ++activeEdges;
            sourceEdges += edge.sourceEdgeId != kInvalidSourceId ? 1U : 0U;
        }
    }
    for (const MutableFace& face : faces_) {
        if (!face.deleted) {
            ++activeFaces;
            sourceFaces += !face.sourceFaceIds.empty() ? 1U : 0U;
            if (face.maximumRingDepth >= 0) {
                maximumDepth = std::max(
                    maximumDepth,
                    static_cast<unsigned int>(face.maximumRingDepth));
            }
        }
    }
    diagnostics_.actualMaximumRingDepth = maximumDepth;
    diagnostics_.ringDepthMismatch =
        maximumDepth != diagnostics_.requestedBlendWidth;
    diagnostics_.vertexSourceCoverage = activeVertices == 0U
        ? 0.0
        : static_cast<double>(sourceVertices) / static_cast<double>(activeVertices);
    diagnostics_.edgeSourceCoverage = activeEdges == 0U
        ? 0.0
        : static_cast<double>(sourceEdges) / static_cast<double>(activeEdges);
    diagnostics_.faceSourceCoverage = activeFaces == 0U
        ? 0.0
        : static_cast<double>(sourceFaces) / static_cast<double>(activeFaces);
    diagnostics_.maximumFixedBoundaryDisplacement = maximumDisplacement;
}

bool LocalMutablePatchMesh::valid(std::string* diagnostic) const
{
    const auto reject = [diagnostic](const std::string& message) {
        if (diagnostic != nullptr) { *diagnostic = message; }
        return false;
    };
    if (vertices_.empty() || edges_.empty() || faces_.empty()) {
        return reject("Mutable patch is empty.");
    }
    std::map<std::pair<MutableVertexId, MutableVertexId>, MutableEdgeId> edgeMap;
    for (std::size_t index = 0U; index < vertices_.size(); ++index) {
        const MutableVertex& vertex = vertices_[index];
        if (vertex.id != index) { return reject("Stable vertex ID slot mismatch."); }
        if (vertex.deleted) { continue; }
        if (!vertex.position.finite() || !vertex.normal.finite()) {
            return reject("Active vertex has non-finite geometry.");
        }
        if (vertex.fixedOuterBoundary &&
            (vertex.position - vertex.sourcePosition).squaredLength() != 0.0) {
            return reject("Fixed outer-boundary vertex was displaced.");
        }
        for (const MutableEdgeId edgeId : vertex.edgeIds) {
            if (edgeId >= edges_.size() || edges_[edgeId].deleted ||
                (edges_[edgeId].vertex0 != vertex.id && edges_[edgeId].vertex1 != vertex.id)) {
                return reject("Vertex-to-edge adjacency is inconsistent.");
            }
        }
        for (const MutableFaceId faceId : vertex.faceIds) {
            if (faceId >= faces_.size() || faces_[faceId].deleted ||
                !contains(faces_[faceId].vertexIds, vertex.id)) {
                return reject("Vertex-to-face adjacency is inconsistent.");
            }
        }
    }
    for (std::size_t index = 0U; index < edges_.size(); ++index) {
        const MutableEdge& edge = edges_[index];
        if (edge.id != index) { return reject("Stable edge ID slot mismatch."); }
        if (edge.deleted) { continue; }
        if (edge.vertex0 >= vertices_.size() || edge.vertex1 >= vertices_.size() ||
            vertices_[edge.vertex0].deleted || vertices_[edge.vertex1].deleted ||
            edge.vertex0 == edge.vertex1) {
            return reject("Active edge has an invalid endpoint.");
        }
        if ((vertices_[edge.vertex0].position - vertices_[edge.vertex1].position).length() <=
            kGeometryEpsilon) {
            return reject("Active edge has zero length.");
        }
        if (edge.faceIds.empty() || edge.faceIds.size() > 2U) {
            return reject("Active edge has an invalid incident-face count.");
        }
        if (edge.faceIds.size() == 1U &&
            !edge.fixedOuterBoundary && !edge.innerInterface) {
            return reject("Operation created an unintended open Transition boundary.");
        }
        if (edge.faceIds.size() == 2U &&
            (edge.fixedOuterBoundary || edge.innerInterface)) {
            return reject("Boundary-role edge unexpectedly has two Transition faces.");
        }
        if (!edgeMap.emplace(edgeKey(edge.vertex0, edge.vertex1), edge.id).second) {
            return reject("Duplicate active edge endpoints were found.");
        }
        if (!contains(vertices_[edge.vertex0].edgeIds, edge.id) ||
            !contains(vertices_[edge.vertex1].edgeIds, edge.id)) {
            return reject("Edge-to-vertex adjacency is inconsistent.");
        }
        for (const MutableFaceId faceId : edge.faceIds) {
            if (faceId >= faces_.size() || faces_[faceId].deleted ||
                !contains(faces_[faceId].edgeIds, edge.id)) {
                return reject("Edge-to-face adjacency is inconsistent.");
            }
        }
    }
    for (std::size_t index = 0U; index < faces_.size(); ++index) {
        const MutableFace& face = faces_[index];
        if (face.id != index) { return reject("Stable face ID slot mismatch."); }
        if (face.deleted) { continue; }
        if (face.vertexIds.size() < 3U || face.edgeIds.size() != face.vertexIds.size()) {
            return reject("Active face has fewer than three corners or bad edge count.");
        }
        std::set<MutableVertexId> uniqueVertices;
        for (std::size_t corner = 0U; corner < face.vertexIds.size(); ++corner) {
            const MutableVertexId first = face.vertexIds[corner];
            const MutableVertexId second = face.vertexIds[(corner + 1U) % face.vertexIds.size()];
            if (first >= vertices_.size() || vertices_[first].deleted ||
                !uniqueVertices.insert(first).second) {
                return reject("Active face contains an invalid or repeated vertex.");
            }
            const MutableEdgeId edgeId = face.edgeIds[corner];
            if (edgeId >= edges_.size() || edges_[edgeId].deleted ||
                edgeKey(edges_[edgeId].vertex0, edges_[edgeId].vertex1) != edgeKey(first, second)) {
                return reject("Face edge order does not match vertex order.");
            }
            if (!contains(vertices_[first].faceIds, face.id)) {
                return reject("Face-to-vertex adjacency is inconsistent.");
            }
        }
        if (polygonAreaVector(face.vertexIds, vertices_).length() <= kGeometryEpsilon) {
            return reject("Active face has zero 3D area.");
        }
    }
    if (!fixedOuterBoundary_.closed || fixedOuterBoundary_.vertexIndices.size() < 3U ||
        fixedOuterBoundary_.edgeIndices.size() != fixedOuterBoundary_.vertexIndices.size()) {
        return reject("Fixed outer boundary is not one ordered closed loop.");
    }
    for (std::size_t item = 0U; item < fixedOuterBoundary_.vertexIndices.size(); ++item) {
        const MutableVertexId vertexId = fixedOuterBoundary_.vertexIndices[item];
        const MutableVertexId nextVertexId = fixedOuterBoundary_.vertexIndices[
            (item + 1U) % fixedOuterBoundary_.vertexIndices.size()];
        const MutableEdgeId edgeId = fixedOuterBoundary_.edgeIndices[item];
        if (vertexId >= vertices_.size() || vertices_[vertexId].deleted ||
            !vertices_[vertexId].fixedOuterBoundary || edgeId >= edges_.size() ||
            edges_[edgeId].deleted || !edges_[edgeId].fixedOuterBoundary ||
            edgeKey(edges_[edgeId].vertex0, edges_[edgeId].vertex1) !=
                edgeKey(vertexId, nextVertexId)) {
            return reject("Fixed outer-boundary identity was not preserved.");
        }
    }
    if (orderedInnerInterfaceVertices_.size() < 3U ||
        orderedInnerInterfaceEdges_.size() != orderedInnerInterfaceVertices_.size()) {
        return reject("Inner interface is not one ordered closed loop.");
    }
    for (std::size_t item = 0U; item < orderedInnerInterfaceVertices_.size(); ++item) {
        const MutableVertexId first = orderedInnerInterfaceVertices_[item];
        const MutableVertexId second = orderedInnerInterfaceVertices_[
            (item + 1U) % orderedInnerInterfaceVertices_.size()];
        const MutableEdgeId edgeId = orderedInnerInterfaceEdges_[item];
        if (first >= vertices_.size() || second >= vertices_.size() ||
            vertices_[first].deleted || vertices_[second].deleted ||
            !vertices_[first].innerInterface || !vertices_[second].innerInterface ||
            edgeId >= edges_.size() || edges_[edgeId].deleted || !edges_[edgeId].innerInterface ||
            edgeKey(edges_[edgeId].vertex0, edges_[edgeId].vertex1) != edgeKey(first, second)) {
            return reject("Inner-interface ordering or degree is inconsistent.");
        }
    }
    std::size_t activeInterfaceEdges = 0U;
    std::set<MutableVertexId> activeInterfaceVertices;
    for (const MutableEdge& edge : edges_) {
        if (!edge.deleted && edge.innerInterface) {
            ++activeInterfaceEdges;
            activeInterfaceVertices.insert(edge.vertex0);
            activeInterfaceVertices.insert(edge.vertex1);
        }
    }
    if (activeInterfaceEdges != orderedInnerInterfaceEdges_.size() ||
        activeInterfaceVertices.size() != orderedInnerInterfaceVertices_.size()) {
        return reject("Inner interface contains an untracked branch or extra loop.");
    }
    for (std::size_t sourceIndex = 0U;
         sourceIndex < mutableVertexIdBySourceIndex_.size(); ++sourceIndex) {
        const MutableVertexId id = mutableVertexIdBySourceIndex_[sourceIndex];
        if (id != kInvalidIndex &&
            (id >= vertices_.size() ||
             vertices_[id].sourceVertexId == kInvalidSourceId)) {
            return reject("Source-to-mutable vertex inverse mapping is invalid.");
        }
    }
    for (std::size_t sourceIndex = 0U;
         sourceIndex < mutableEdgeIdBySourceIndex_.size(); ++sourceIndex) {
        const MutableEdgeId id = mutableEdgeIdBySourceIndex_[sourceIndex];
        if (id != kInvalidIndex &&
            (id >= edges_.size() || edges_[id].sourceEdgeId == kInvalidSourceId)) {
            return reject("Source-to-mutable edge inverse mapping is invalid.");
        }
    }
    for (std::size_t sourceIndex = 0U;
         sourceIndex < mutableFaceIdBySourceIndex_.size(); ++sourceIndex) {
        const MutableFaceId id = mutableFaceIdBySourceIndex_[sourceIndex];
        if (id != kInvalidIndex &&
            (id >= faces_.size() || faces_[id].sourceFaceIds.empty())) {
            return reject("Source-to-mutable face inverse mapping is invalid.");
        }
    }
    if (diagnostic != nullptr) {
        *diagnostic = diagnostics_.ringDepthMismatch
            ? "Valid mutable patch (ring-depth mismatch is diagnostic only)."
            : "Valid mutable patch.";
    }
    return true;
}

std::uint64_t LocalMutablePatchMesh::signature() const noexcept
{
    std::size_t hash = 0xcbf29ce484222325ULL;
    const auto mix = [&hash](std::uint64_t value) { hash = hashMix(hash, value); };
    mix(vertices_.size()); mix(edges_.size()); mix(faces_.size()); mix(nextOperationId_);
    for (const MutableVertex& value : vertices_) {
        mix(value.id); mix(value.deleted); mix(value.fixedOuterBoundary); mix(value.innerInterface);
        mix(doubleBits(value.position.x)); mix(doubleBits(value.position.y)); mix(doubleBits(value.position.z));
        mix(static_cast<std::uint64_t>(value.sourceVertexId)); mix(value.parentEdgeId);
        mix(value.createdByOperation);
        for (const MutableEdgeId id : value.edgeIds) { mix(id); }
        for (const MutableFaceId id : value.faceIds) { mix(id); }
    }
    for (const MutableEdge& value : edges_) {
        mix(value.id); mix(value.deleted); mix(value.fixedOuterBoundary); mix(value.innerInterface);
        mix(value.vertex0); mix(value.vertex1); mix(static_cast<std::uint64_t>(value.sourceEdgeId));
        mix(value.parentEdgeId); mix(value.createdByOperation);
        for (const MutableFaceId id : value.faceIds) { mix(id); }
    }
    for (const MutableFace& value : faces_) {
        mix(value.id); mix(value.deleted); mix(value.createdByOperation);
        mix(static_cast<std::uint64_t>(value.minimumRingDepth));
        mix(static_cast<std::uint64_t>(value.maximumRingDepth));
        for (const MutableVertexId id : value.vertexIds) { mix(id); }
        for (const MutableEdgeId id : value.edgeIds) { mix(id); }
        for (const SourceId id : value.sourceFaceIds) { mix(static_cast<std::uint64_t>(id)); }
    }
    for (const MutableVertexId id : orderedInnerInterfaceVertices_) { mix(id); }
    for (const MutableEdgeId id : orderedInnerInterfaceEdges_) { mix(id); }
    for (const MutableOperationRecord& value : operationLineage_) {
        mix(value.id); mix(static_cast<std::uint64_t>(value.type));
    }
    return static_cast<std::uint64_t>(hash);
}

const char* mutableOperationStatusName(MutableOperationStatus status) noexcept
{
    switch (status) {
    case MutableOperationStatus::Success: return "Success";
    case MutableOperationStatus::InvalidElement: return "InvalidElement";
    case MutableOperationStatus::InvalidParameter: return "InvalidParameter";
    case MutableOperationStatus::FixedBoundaryViolation: return "FixedBoundaryViolation";
    case MutableOperationStatus::UnsupportedConfiguration: return "UnsupportedConfiguration";
    case MutableOperationStatus::WouldCreateDegenerateFace: return "WouldCreateDegenerateFace";
    case MutableOperationStatus::WouldCreateDuplicateEdge: return "WouldCreateDuplicateEdge";
    case MutableOperationStatus::WouldBecomeNonManifold: return "WouldBecomeNonManifold";
    case MutableOperationStatus::GeometryInvalid: return "GeometryInvalid";
    case MutableOperationStatus::InnerInterfaceInvalid: return "InnerInterfaceInvalid";
    case MutableOperationStatus::ValidationFailed: return "ValidationFailed";
    }
    return "Unknown";
}

MutableOperationResult LocalMutablePatchMesh::moveVertex(
    MutableVertexId vertexId, const Vec3& position)
{
    LocalMutablePatchMesh candidate = *this;
    MutableOperationResult result = candidate.executeMoveVertex(
        vertexId, position, candidate.nextOperationId_);
    if (!result.success()) { return result; }
    std::string reason;
    candidate.refreshDiagnostics();
    if (!candidate.valid(&reason)) {
        return failure(result.changes.type, result.changes.id,
            MutableOperationStatus::ValidationFailed, reason);
    }
    ++candidate.nextOperationId_;
    candidate.operationLineage_.push_back(result.changes);
    *this = std::move(candidate);
    return result;
}

MutableOperationResult LocalMutablePatchMesh::splitEdge(
    MutableEdgeId edgeId, double parameter)
{
    LocalMutablePatchMesh candidate = *this;
    MutableOperationResult result = candidate.executeSplitEdge(
        edgeId, parameter, candidate.nextOperationId_);
    if (!result.success()) { return result; }
    std::string reason;
    candidate.refreshDiagnostics();
    if (!candidate.valid(&reason)) {
        return failure(result.changes.type, result.changes.id,
            MutableOperationStatus::ValidationFailed, reason);
    }
    ++candidate.nextOperationId_;
    candidate.operationLineage_.push_back(result.changes);
    *this = std::move(candidate);
    return result;
}

MutableOperationResult LocalMutablePatchMesh::collapseEdgeToEndpoint(
    MutableEdgeId edgeId, MutableVertexId endpointToKeep)
{
    LocalMutablePatchMesh candidate = *this;
    MutableOperationResult result = candidate.executeCollapseEdge(
        edgeId, endpointToKeep, candidate.nextOperationId_);
    if (!result.success()) { return result; }
    std::string reason;
    candidate.refreshDiagnostics();
    if (!candidate.valid(&reason)) {
        return failure(result.changes.type, result.changes.id,
            MutableOperationStatus::ValidationFailed, reason);
    }
    ++candidate.nextOperationId_;
    candidate.operationLineage_.push_back(result.changes);
    *this = std::move(candidate);
    return result;
}

MutableOperationResult LocalMutablePatchMesh::dissolveEdge(MutableEdgeId edgeId)
{
    LocalMutablePatchMesh candidate = *this;
    MutableOperationResult result = candidate.executeDissolveEdge(
        edgeId, candidate.nextOperationId_);
    if (!result.success()) { return result; }
    std::string reason;
    candidate.refreshDiagnostics();
    if (!candidate.valid(&reason)) {
        return failure(result.changes.type, result.changes.id,
            MutableOperationStatus::ValidationFailed, reason);
    }
    ++candidate.nextOperationId_;
    candidate.operationLineage_.push_back(result.changes);
    *this = std::move(candidate);
    return result;
}

MutableOperationResult LocalMutablePatchMesh::flipTriangleEdge(MutableEdgeId edgeId)
{
    LocalMutablePatchMesh candidate = *this;
    MutableOperationResult result = candidate.executeFlipEdge(
        edgeId, candidate.nextOperationId_);
    if (!result.success()) { return result; }
    std::string reason;
    candidate.refreshDiagnostics();
    if (!candidate.valid(&reason)) {
        return failure(result.changes.type, result.changes.id,
            MutableOperationStatus::ValidationFailed, reason);
    }
    ++candidate.nextOperationId_;
    candidate.operationLineage_.push_back(result.changes);
    *this = std::move(candidate);
    return result;
}

bool LocalMutablePatchMesh::rebuildAffectedTopology(
    const std::vector<MutableFaceId>& affectedFaces,
    MutableOperationRecord& changes,
    std::string& diagnostic)
{
    std::vector<MutableFaceId> faceIds = affectedFaces;
    normalizeIds(faceIds);
    std::vector<MutableEdgeId> oldEdges;
    for (const MutableFaceId faceId : faceIds) {
        if (faceId >= faces_.size()) {
            diagnostic = "Affected face ID is invalid.";
            return false;
        }
        MutableFace& face = faces_[faceId];
        for (const MutableEdgeId edgeId : face.edgeIds) {
            if (edgeId < edges_.size()) {
                if (edges_[edgeId].vertex0 < vertices_.size()) {
                    eraseValue(vertices_[edges_[edgeId].vertex0].faceIds, faceId);
                }
                if (edges_[edgeId].vertex1 < vertices_.size()) {
                    eraseValue(vertices_[edges_[edgeId].vertex1].faceIds, faceId);
                }
                eraseValue(edges_[edgeId].faceIds, faceId);
                appendUnique(oldEdges, edgeId);
            }
        }
        for (const MutableVertexId vertexId : face.vertexIds) {
            if (vertexId < vertices_.size()) {
                eraseValue(vertices_[vertexId].faceIds, faceId);
            }
        }
        face.edgeIds.clear();
    }

    for (const MutableFaceId faceId : faceIds) {
        MutableFace& face = faces_[faceId];
        if (face.deleted) { continue; }
        for (const MutableVertexId vertexId : face.vertexIds) {
            if (vertexId >= vertices_.size() || vertices_[vertexId].deleted) {
                diagnostic = "Edited face references a deleted vertex.";
                return false;
            }
            appendUnique(vertices_[vertexId].faceIds, faceId);
        }
        for (std::size_t corner = 0U; corner < face.vertexIds.size(); ++corner) {
            const MutableVertexId first = face.vertexIds[corner];
            const MutableVertexId second =
                face.vertexIds[(corner + 1U) % face.vertexIds.size()];
            MutableEdgeId edgeId = findActiveEdge(edges_, first, second);
            if (edgeId == kInvalidIndex) {
                MutableEdge edge;
                edge.id = edges_.size();
                edge.vertex0 = first;
                edge.vertex1 = second;
                edge.origin = MutableElementOrigin::Derived;
                edge.innerInterface =
                    vertices_[first].innerInterface && vertices_[second].innerInterface;
                edge.createdByOperation = changes.id;
                edgeId = edge.id;
                edges_.push_back(std::move(edge));
                appendUnique(vertices_[first].edgeIds, edgeId);
                appendUnique(vertices_[second].edgeIds, edgeId);
                changes.createdEdgeIds.push_back(edgeId);
            }
            appendUnique(edges_[edgeId].faceIds, faceId);
            face.edgeIds.push_back(edgeId);
        }
        appendUnique(changes.modifiedFaceIds, faceId);
    }

    for (const MutableEdgeId edgeId : oldEdges) {
        MutableEdge& edge = edges_[edgeId];
        if (!edge.deleted && edge.faceIds.empty()) {
            if (edge.fixedOuterBoundary) {
                diagnostic = "Operation attempted to remove a fixed outer-boundary edge.";
                return false;
            }
            edge.deleted = true;
            eraseValue(vertices_[edge.vertex0].edgeIds, edgeId);
            eraseValue(vertices_[edge.vertex1].edgeIds, edgeId);
            appendUnique(changes.deletedEdgeIds, edgeId);
        } else if (!edge.deleted) {
            appendUnique(changes.modifiedEdgeIds, edgeId);
        }
    }
    for (MutableEdge& edge : edges_) {
        if (!edge.deleted && edge.faceIds.size() > 2U) {
            diagnostic = "Operation would create a non-manifold edge.";
            return false;
        }
    }
    normalizeIds(changes.createdEdgeIds);
    normalizeIds(changes.deletedEdgeIds);
    normalizeIds(changes.modifiedEdgeIds);
    normalizeIds(changes.modifiedFaceIds);
    return true;
}

bool LocalMutablePatchMesh::rebuildInnerInterface(std::string& diagnostic)
{
    std::map<MutableVertexId, std::vector<std::pair<MutableVertexId, MutableEdgeId>>> adjacency;
    for (MutableEdge& edge : edges_) {
        if (edge.deleted) { continue; }
        if (edge.innerInterface &&
            (!vertices_[edge.vertex0].innerInterface ||
             !vertices_[edge.vertex1].innerInterface)) {
            diagnostic = "Inner-interface edge lost an interface endpoint.";
            return false;
        }
        if (edge.innerInterface) {
            adjacency[edge.vertex0].push_back({edge.vertex1, edge.id});
            adjacency[edge.vertex1].push_back({edge.vertex0, edge.id});
        }
    }
    if (adjacency.size() < 3U) {
        diagnostic = "Inner interface has fewer than three active vertices.";
        return false;
    }
    for (auto& item : adjacency) {
        std::sort(item.second.begin(), item.second.end());
        if (item.second.size() != 2U) {
            diagnostic = "Inner interface is open or branched.";
            return false;
        }
    }

    const MutableVertexId start = adjacency.begin()->first;
    MutableVertexId previous = kInvalidIndex;
    MutableVertexId current = start;
    std::vector<MutableVertexId> vertices;
    std::vector<MutableEdgeId> edges;
    do {
        vertices.push_back(current);
        const auto& neighbors = adjacency[current];
        const auto next = previous == kInvalidIndex || neighbors[0].first != previous
            ? neighbors[0] : neighbors[1];
        edges.push_back(next.second);
        previous = current;
        current = next.first;
        if (vertices.size() > adjacency.size()) {
            diagnostic = "Inner interface traversal did not close.";
            return false;
        }
    } while (current != start);
    if (vertices.size() != adjacency.size()) {
        diagnostic = "Inner interface contains multiple loops.";
        return false;
    }
    orderedInnerInterfaceVertices_ = std::move(vertices);
    orderedInnerInterfaceEdges_ = std::move(edges);
    return true;
}

MutableOperationResult LocalMutablePatchMesh::executeMoveVertex(
    MutableVertexId vertexId,
    const Vec3& position,
    MutableOperationId operationId)
{
    if (vertexId >= vertices_.size() || vertices_[vertexId].deleted) {
        return failure(MutableOperationType::MoveVertex, operationId,
            MutableOperationStatus::InvalidElement, "Vertex is not active.");
    }
    if (vertices_[vertexId].fixedOuterBoundary) {
        return failure(MutableOperationType::MoveVertex, operationId,
            MutableOperationStatus::FixedBoundaryViolation,
            "Fixed outer-boundary vertices cannot be moved.");
    }
    if (!position.finite()) {
        return failure(MutableOperationType::MoveVertex, operationId,
            MutableOperationStatus::InvalidParameter, "Position is not finite.");
    }
    vertices_[vertexId].position = position;
    MutableOperationResult result;
    result.status = MutableOperationStatus::Success;
    result.diagnosticMessage = "Vertex moved transactionally.";
    result.changes.id = operationId;
    result.changes.type = MutableOperationType::MoveVertex;
    result.changes.modifiedVertexIds.push_back(vertexId);
    return result;
}

MutableOperationResult LocalMutablePatchMesh::executeSplitEdge(
    MutableEdgeId edgeId,
    double parameter,
    MutableOperationId operationId)
{
    if (edgeId >= edges_.size() || edges_[edgeId].deleted) {
        return failure(MutableOperationType::SplitEdge, operationId,
            MutableOperationStatus::InvalidElement, "Edge is not active.");
    }
    const MutableEdge sourceEdge = edges_[edgeId];
    if (sourceEdge.fixedOuterBoundary) {
        return failure(MutableOperationType::SplitEdge, operationId,
            MutableOperationStatus::FixedBoundaryViolation,
            "Fixed outer-boundary edges cannot be split.");
    }
    if (!std::isfinite(parameter) || parameter <= 0.0 || parameter >= 1.0) {
        return failure(MutableOperationType::SplitEdge, operationId,
            MutableOperationStatus::InvalidParameter,
            "Split parameter must be finite and strictly inside the edge.");
    }
    MutableOperationResult result;
    result.status = MutableOperationStatus::Success;
    result.changes.id = operationId;
    result.changes.type = MutableOperationType::SplitEdge;

    MutableVertex vertex;
    vertex.id = vertices_.size();
    vertex.position = vertices_[sourceEdge.vertex0].position * (1.0 - parameter) +
        vertices_[sourceEdge.vertex1].position * parameter;
    vertex.sourcePosition = vertex.position;
    vertex.normal = (vertices_[sourceEdge.vertex0].normal * (1.0 - parameter) +
        vertices_[sourceEdge.vertex1].normal * parameter).normalized();
    vertex.innerInterface = sourceEdge.innerInterface;
    vertex.origin = MutableElementOrigin::Derived;
    vertex.parentEdgeId = edgeId;
    vertex.parentSourceEdgeId = sourceEdge.sourceEdgeId;
    vertex.parentEdgeParameter = parameter;
    vertex.createdByOperation = operationId;
    vertices_.push_back(std::move(vertex));
    result.changes.createdVertexIds.push_back(vertices_.back().id);

    for (const MutableFaceId faceId : sourceEdge.faceIds) {
        MutableFace& face = faces_[faceId];
        for (std::size_t corner = 0U; corner < face.vertexIds.size(); ++corner) {
            const MutableVertexId first = face.vertexIds[corner];
            const MutableVertexId second =
                face.vertexIds[(corner + 1U) % face.vertexIds.size()];
            if (edgeKey(first, second) ==
                edgeKey(sourceEdge.vertex0, sourceEdge.vertex1)) {
                face.vertexIds.insert(
                    face.vertexIds.begin() + static_cast<std::ptrdiff_t>(corner + 1U),
                    vertices_.back().id);
                break;
            }
        }
    }
    std::string reason;
    if (!rebuildAffectedTopology(sourceEdge.faceIds, result.changes, reason)) {
        return failure(MutableOperationType::SplitEdge, operationId,
            MutableOperationStatus::WouldBecomeNonManifold, reason);
    }
    for (const MutableEdgeId createdId : result.changes.createdEdgeIds) {
        MutableEdge& edge = edges_[createdId];
        if (contains(std::vector<MutableVertexId>{sourceEdge.vertex0, sourceEdge.vertex1},
                     edge.vertex0) ||
            contains(std::vector<MutableVertexId>{sourceEdge.vertex0, sourceEdge.vertex1},
                     edge.vertex1)) {
            edge.parentEdgeId = edgeId;
            edge.parentSourceEdgeId = sourceEdge.sourceEdgeId;
            edge.innerInterface = sourceEdge.innerInterface;
        }
    }
    if (sourceEdge.innerInterface && !rebuildInnerInterface(reason)) {
        return failure(MutableOperationType::SplitEdge, operationId,
            MutableOperationStatus::InnerInterfaceInvalid, reason);
    }
    result.diagnosticMessage = "Edge split committed; source lineage retained.";
    return result;
}

MutableOperationResult LocalMutablePatchMesh::executeCollapseEdge(
    MutableEdgeId edgeId,
    MutableVertexId endpointToKeep,
    MutableOperationId operationId)
{
    if (edgeId >= edges_.size() || edges_[edgeId].deleted) {
        return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
            MutableOperationStatus::InvalidElement, "Edge is not active.");
    }
    const MutableEdge sourceEdge = edges_[edgeId];
    if (endpointToKeep != sourceEdge.vertex0 && endpointToKeep != sourceEdge.vertex1) {
        return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
            MutableOperationStatus::InvalidParameter,
            "Collapse destination must be one endpoint of the edge.");
    }
    const MutableVertexId removed = endpointToKeep == sourceEdge.vertex0
        ? sourceEdge.vertex1 : sourceEdge.vertex0;
    if (sourceEdge.fixedOuterBoundary ||
        vertices_[endpointToKeep].fixedOuterBoundary ||
        vertices_[removed].fixedOuterBoundary) {
        return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
            MutableOperationStatus::FixedBoundaryViolation,
            "Collapse cannot delete or alter a fixed outer-boundary element.");
    }
    if (vertices_[removed].innerInterface && !sourceEdge.innerInterface) {
        return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
            MutableOperationStatus::InnerInterfaceInvalid,
            "An inner-interface vertex can only collapse along its interface.");
    }

    MutableOperationResult result;
    result.status = MutableOperationStatus::Success;
    result.changes.id = operationId;
    result.changes.type = MutableOperationType::CollapseEdgeToEndpoint;
    const std::vector<MutableEdgeId> removedIncidentEdges =
        vertices_[removed].edgeIds;
    std::vector<MutableFaceId> affected = vertices_[removed].faceIds;
    for (const MutableFaceId faceId : vertices_[endpointToKeep].faceIds) {
        appendUnique(affected, faceId);
    }
    for (const MutableFaceId faceId : affected) {
        MutableFace& face = faces_[faceId];
        if (face.deleted) { continue; }
        for (MutableVertexId& vertexId : face.vertexIds) {
            if (vertexId == removed) { vertexId = endpointToKeep; }
        }
        face.vertexIds = compactPolygon(face.vertexIds);
        std::set<MutableVertexId> unique(face.vertexIds.begin(), face.vertexIds.end());
        if (unique.size() < 3U) {
            if (face.vertexIds.size() <= 2U) {
                face.deleted = true;
                result.changes.deletedFaceIds.push_back(faceId);
            } else {
                return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
                    MutableOperationStatus::WouldCreateDegenerateFace,
                    "Collapse would create a degenerate face.");
            }
        } else if (unique.size() != face.vertexIds.size()) {
            return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
                MutableOperationStatus::WouldCreateDuplicateEdge,
                "Collapse would repeat a vertex inside a face.");
        }
    }
    vertices_[endpointToKeep].innerInterface =
        vertices_[endpointToKeep].innerInterface || vertices_[removed].innerInterface;
    std::string reason;
    if (!rebuildAffectedTopology(affected, result.changes, reason)) {
        return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
            MutableOperationStatus::WouldBecomeNonManifold, reason);
    }
    for (const MutableEdgeId createdId : result.changes.createdEdgeIds) {
        MutableEdge& created = edges_[createdId];
        const MutableVertexId other = created.vertex0 == endpointToKeep
            ? created.vertex1 : created.vertex0;
        for (const MutableEdgeId parentId : removedIncidentEdges) {
            const MutableEdge& parent = edges_[parentId];
            if (edgeKey(parent.vertex0, parent.vertex1) == edgeKey(removed, other)) {
                created.parentEdgeId = parentId;
                created.parentSourceEdgeId = parent.sourceEdgeId != kInvalidSourceId
                    ? parent.sourceEdgeId : parent.parentSourceEdgeId;
                break;
            }
        }
    }
    vertices_[removed].deleted = true;
    vertices_[removed].edgeIds.clear();
    vertices_[removed].faceIds.clear();
    result.changes.deletedVertexIds.push_back(removed);
    result.changes.modifiedVertexIds.push_back(endpointToKeep);
    if (sourceEdge.innerInterface && !rebuildInnerInterface(reason)) {
        return failure(MutableOperationType::CollapseEdgeToEndpoint, operationId,
            MutableOperationStatus::InnerInterfaceInvalid, reason);
    }
    normalizeIds(result.changes.deletedFaceIds);
    result.diagnosticMessage =
        "Edge collapsed to an existing endpoint without relocating it.";
    return result;
}

MutableOperationResult LocalMutablePatchMesh::executeDissolveEdge(
    MutableEdgeId edgeId,
    MutableOperationId operationId)
{
    if (edgeId >= edges_.size() || edges_[edgeId].deleted) {
        return failure(MutableOperationType::DissolveEdge, operationId,
            MutableOperationStatus::InvalidElement, "Edge is not active.");
    }
    const MutableEdge sourceEdge = edges_[edgeId];
    if (sourceEdge.fixedOuterBoundary) {
        return failure(MutableOperationType::DissolveEdge, operationId,
            MutableOperationStatus::FixedBoundaryViolation,
            "Fixed outer-boundary edges cannot be dissolved.");
    }
    if (sourceEdge.innerInterface) {
        return failure(MutableOperationType::DissolveEdge, operationId,
            MutableOperationStatus::InnerInterfaceInvalid,
            "R5 does not dissolve the ordered inner-interface loop.");
    }
    if (sourceEdge.faceIds.size() != 2U) {
        return failure(MutableOperationType::DissolveEdge, operationId,
            MutableOperationStatus::UnsupportedConfiguration,
            "Dissolve requires exactly two incident faces.");
    }
    const MutableFaceId keepId = std::min(sourceEdge.faceIds[0], sourceEdge.faceIds[1]);
    const MutableFaceId removeId = std::max(sourceEdge.faceIds[0], sourceEdge.faceIds[1]);
    const MutableFace& first = faces_[keepId];
    const MutableFace& second = faces_[removeId];

    std::map<MutableVertexId, std::vector<MutableVertexId>> boundary;
    const auto addBoundary = [&boundary, &sourceEdge](const MutableFace& face) {
        for (std::size_t corner = 0U; corner < face.vertexIds.size(); ++corner) {
            const MutableVertexId a = face.vertexIds[corner];
            const MutableVertexId b = face.vertexIds[(corner + 1U) % face.vertexIds.size()];
            if (edgeKey(a, b) == edgeKey(sourceEdge.vertex0, sourceEdge.vertex1)) {
                continue;
            }
            boundary[a].push_back(b);
            boundary[b].push_back(a);
        }
    };
    addBoundary(first);
    addBoundary(second);
    for (auto& item : boundary) {
        normalizeIds(item.second);
        if (item.second.size() != 2U) {
            return failure(MutableOperationType::DissolveEdge, operationId,
                MutableOperationStatus::WouldCreateDegenerateFace,
                "Incident polygons do not form one simple dissolve boundary.");
        }
    }
    const MutableVertexId start = boundary.begin()->first;
    MutableVertexId previous = kInvalidIndex;
    MutableVertexId current = start;
    std::vector<MutableVertexId> merged;
    do {
        merged.push_back(current);
        const auto& neighbors = boundary[current];
        const MutableVertexId next =
            previous == kInvalidIndex || neighbors[0] != previous
                ? neighbors[0] : neighbors[1];
        previous = current;
        current = next;
        if (merged.size() > boundary.size()) {
            return failure(MutableOperationType::DissolveEdge, operationId,
                MutableOperationStatus::WouldCreateDegenerateFace,
                "Merged dissolve boundary did not close.");
        }
    } while (current != start);
    if (merged.size() != boundary.size() ||
        polygonAreaVector(merged, vertices_).length() <= kGeometryEpsilon) {
        return failure(MutableOperationType::DissolveEdge, operationId,
            MutableOperationStatus::WouldCreateDegenerateFace,
            "Dissolve would create a zero-area polygon.");
    }

    MutableOperationResult result;
    result.status = MutableOperationStatus::Success;
    result.changes.id = operationId;
    result.changes.type = MutableOperationType::DissolveEdge;
    MutableFace& keep = faces_[keepId];
    MutableFace& removedFace = faces_[removeId];
    keep.vertexIds = std::move(merged);
    keep.origin = MutableElementOrigin::Derived;
    keep.createdByOperation = operationId;
    keep.minimumRingDepth = std::min(keep.minimumRingDepth, removedFace.minimumRingDepth);
    keep.maximumRingDepth = std::max(keep.maximumRingDepth, removedFace.maximumRingDepth);
    for (const SourceId sourceId : removedFace.sourceFaceIds) {
        appendUnique(keep.sourceFaceIds, sourceId);
    }
    std::sort(keep.sourceFaceIds.begin(), keep.sourceFaceIds.end());
    removedFace.deleted = true;
    result.changes.deletedFaceIds.push_back(removeId);
    std::string reason;
    if (!rebuildAffectedTopology({keepId, removeId}, result.changes, reason)) {
        return failure(MutableOperationType::DissolveEdge, operationId,
            MutableOperationStatus::WouldBecomeNonManifold, reason);
    }
    result.diagnosticMessage =
        keep.vertexIds.size() > 4U
            ? "Edge dissolved; source-derived n-gon retained for later adaptation."
            : "Edge dissolved into one source-derived polygon.";
    return result;
}

MutableOperationResult LocalMutablePatchMesh::executeFlipEdge(
    MutableEdgeId edgeId,
    MutableOperationId operationId)
{
    if (edgeId >= edges_.size() || edges_[edgeId].deleted) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::InvalidElement, "Edge is not active.");
    }
    const MutableEdge sourceEdge = edges_[edgeId];
    if (sourceEdge.fixedOuterBoundary) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::FixedBoundaryViolation,
            "Fixed outer-boundary edges cannot be flipped.");
    }
    if (sourceEdge.innerInterface) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::InnerInterfaceInvalid,
            "Inner-interface edges cannot be flipped.");
    }
    if (sourceEdge.faceIds.size() != 2U ||
        faces_[sourceEdge.faceIds[0]].vertexIds.size() != 3U ||
        faces_[sourceEdge.faceIds[1]].vertexIds.size() != 3U) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::UnsupportedConfiguration,
            "Edge flip requires two active source triangles.");
    }
    MutableFace& first = faces_[sourceEdge.faceIds[0]];
    MutableFace& second = faces_[sourceEdge.faceIds[1]];
    MutableVertexId u = sourceEdge.vertex0;
    MutableVertexId v = sourceEdge.vertex1;
    MutableVertexId a = kInvalidIndex;
    MutableVertexId b = kInvalidIndex;
    for (std::size_t corner = 0U; corner < first.vertexIds.size(); ++corner) {
        if (first.vertexIds[corner] == u &&
            first.vertexIds[(corner + 1U) % 3U] == v) {
            a = first.vertexIds[(corner + 2U) % 3U];
            break;
        }
        if (first.vertexIds[corner] == v &&
            first.vertexIds[(corner + 1U) % 3U] == u) {
            std::swap(u, v);
            a = first.vertexIds[(corner + 2U) % 3U];
            break;
        }
    }
    for (const MutableVertexId vertexId : second.vertexIds) {
        if (vertexId != u && vertexId != v) { b = vertexId; break; }
    }
    if (a == kInvalidIndex || b == kInvalidIndex || a == b) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::WouldCreateDegenerateFace,
            "Triangle pair has no valid opposite diagonal.");
    }
    const MutableEdgeId existing = findActiveEdge(edges_, a, b);
    if (existing != kInvalidIndex && existing != edgeId) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::WouldCreateDuplicateEdge,
            "Opposite diagonal already exists.");
    }
    const std::vector<MutableVertexId> firstVertices = {a, u, b};
    const std::vector<MutableVertexId> secondVertices = {a, b, v};
    if (polygonAreaVector(firstVertices, vertices_).length() <= kGeometryEpsilon ||
        polygonAreaVector(secondVertices, vertices_).length() <= kGeometryEpsilon) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::WouldCreateDegenerateFace,
            "Flipped diagonal would create a zero-area triangle.");
    }
    first.vertexIds = firstVertices;
    second.vertexIds = secondVertices;
    first.origin = MutableElementOrigin::Derived;
    second.origin = MutableElementOrigin::Derived;
    first.createdByOperation = operationId;
    second.createdByOperation = operationId;
    MutableOperationResult result;
    result.status = MutableOperationStatus::Success;
    result.changes.id = operationId;
    result.changes.type = MutableOperationType::FlipTriangleEdge;
    std::string reason;
    if (!rebuildAffectedTopology(sourceEdge.faceIds, result.changes, reason)) {
        return failure(MutableOperationType::FlipTriangleEdge, operationId,
            MutableOperationStatus::WouldBecomeNonManifold, reason);
    }
    for (const MutableEdgeId createdId : result.changes.createdEdgeIds) {
        MutableEdge& created = edges_[createdId];
        if (edgeKey(created.vertex0, created.vertex1) == edgeKey(a, b)) {
            created.parentEdgeId = edgeId;
            created.parentSourceEdgeId = sourceEdge.sourceEdgeId != kInvalidSourceId
                ? sourceEdge.sourceEdgeId : sourceEdge.parentSourceEdgeId;
        }
    }
    result.diagnosticMessage = "Triangle-triangle diagonal flipped safely.";
    return result;
}

}  // namespace directional_retopo::solver
