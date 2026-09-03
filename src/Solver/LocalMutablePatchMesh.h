#pragma once

#include "Solver/SourceTransitionScaffold.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace directional_retopo::solver {

using MutableVertexId = std::size_t;
using MutableEdgeId = std::size_t;
using MutableFaceId = std::size_t;
using MutableOperationId = std::uint64_t;

enum class MutableElementOrigin
{
    Source,
    Derived,
    Synthetic,
};

enum class MutableOperationType
{
    None,
    MoveVertex,
    SplitEdge,
    CollapseEdgeToEndpoint,
    DissolveEdge,
    FlipTriangleEdge,
};

enum class MutableOperationStatus
{
    Success,
    InvalidElement,
    InvalidParameter,
    FixedBoundaryViolation,
    UnsupportedConfiguration,
    WouldCreateDegenerateFace,
    WouldCreateDuplicateEdge,
    WouldBecomeNonManifold,
    GeometryInvalid,
    InnerInterfaceInvalid,
    ValidationFailed,
};

struct MutableVertex final
{
    MutableVertexId id = kInvalidIndex;
    bool deleted = false;
    bool fixedOuterBoundary = false;
    bool innerInterface = false;
    Vec3 position;
    Vec3 sourcePosition;
    Vec3 normal;
    MutableElementOrigin origin = MutableElementOrigin::Source;
    SourceId sourceVertexId = kInvalidSourceId;
    MutableEdgeId parentEdgeId = kInvalidIndex;
    SourceId parentSourceEdgeId = kInvalidSourceId;
    double parentEdgeParameter = 0.0;
    MutableOperationId createdByOperation = 0U;
    std::vector<MutableEdgeId> edgeIds;
    std::vector<MutableFaceId> faceIds;
};

struct MutableEdge final
{
    MutableEdgeId id = kInvalidIndex;
    bool deleted = false;
    bool fixedOuterBoundary = false;
    bool innerInterface = false;
    MutableVertexId vertex0 = kInvalidIndex;
    MutableVertexId vertex1 = kInvalidIndex;
    MutableElementOrigin origin = MutableElementOrigin::Source;
    SourceId sourceEdgeId = kInvalidSourceId;
    MutableEdgeId parentEdgeId = kInvalidIndex;
    SourceId parentSourceEdgeId = kInvalidSourceId;
    MutableOperationId createdByOperation = 0U;
    std::vector<MutableFaceId> faceIds;
};

struct MutableFace final
{
    MutableFaceId id = kInvalidIndex;
    bool deleted = false;
    MutableElementOrigin origin = MutableElementOrigin::Source;
    std::vector<MutableVertexId> vertexIds;
    std::vector<MutableEdgeId> edgeIds;
    std::vector<SourceId> sourceFaceIds;
    int minimumRingDepth = -1;
    int maximumRingDepth = -1;
    MutableOperationId createdByOperation = 0U;
};

struct MutableOperationRecord final
{
    MutableOperationId id = 0U;
    MutableOperationType type = MutableOperationType::None;
    std::vector<MutableVertexId> createdVertexIds;
    std::vector<MutableEdgeId> createdEdgeIds;
    std::vector<MutableFaceId> createdFaceIds;
    std::vector<MutableVertexId> deletedVertexIds;
    std::vector<MutableEdgeId> deletedEdgeIds;
    std::vector<MutableFaceId> deletedFaceIds;
    std::vector<MutableVertexId> modifiedVertexIds;
    std::vector<MutableEdgeId> modifiedEdgeIds;
    std::vector<MutableFaceId> modifiedFaceIds;
};

struct MutableOperationResult final
{
    MutableOperationStatus status = MutableOperationStatus::ValidationFailed;
    std::string diagnosticMessage;
    MutableOperationRecord changes;

    [[nodiscard]] bool success() const noexcept
    {
        return status == MutableOperationStatus::Success;
    }
};

struct MutablePatchDiagnostics final
{
    unsigned int requestedBlendWidth = 0U;
    unsigned int actualMaximumRingDepth = 0U;
    bool ringDepthMismatch = false;
    double vertexSourceCoverage = 0.0;
    double edgeSourceCoverage = 0.0;
    double faceSourceCoverage = 0.0;
    double maximumFixedBoundaryDisplacement = 0.0;
};

class LocalMutablePatchMesh final
{
public:
    [[nodiscard]] static LocalMutablePatchMesh fromScaffold(
        const SourceTransitionScaffold& scaffold,
        unsigned int requestedBlendWidth,
        std::string* diagnostic = nullptr);

    [[nodiscard]] bool valid(std::string* diagnostic = nullptr) const;
    [[nodiscard]] std::uint64_t signature() const noexcept;

    [[nodiscard]] MutableOperationResult moveVertex(
        MutableVertexId vertexId,
        const Vec3& position);
    [[nodiscard]] MutableOperationResult splitEdge(
        MutableEdgeId edgeId,
        double parameter = 0.5);
    [[nodiscard]] MutableOperationResult collapseEdgeToEndpoint(
        MutableEdgeId edgeId,
        MutableVertexId endpointToKeep);
    [[nodiscard]] MutableOperationResult dissolveEdge(MutableEdgeId edgeId);
    [[nodiscard]] MutableOperationResult flipTriangleEdge(MutableEdgeId edgeId);

    [[nodiscard]] const std::vector<MutableVertex>& vertices() const noexcept
    {
        return vertices_;
    }
    [[nodiscard]] const std::vector<MutableEdge>& edges() const noexcept
    {
        return edges_;
    }
    [[nodiscard]] const std::vector<MutableFace>& faces() const noexcept
    {
        return faces_;
    }
    [[nodiscard]] const ScaffoldBoundaryLoop& fixedOuterBoundary() const noexcept
    {
        return fixedOuterBoundary_;
    }
    [[nodiscard]] const std::vector<MutableVertexId>& orderedInnerInterfaceVertices() const noexcept
    {
        return orderedInnerInterfaceVertices_;
    }
    [[nodiscard]] const std::vector<MutableEdgeId>& orderedInnerInterfaceEdges() const noexcept
    {
        return orderedInnerInterfaceEdges_;
    }
    [[nodiscard]] const std::vector<MutableOperationRecord>& operationLineage() const noexcept
    {
        return operationLineage_;
    }
    [[nodiscard]] const MutablePatchDiagnostics& diagnostics() const noexcept
    {
        return diagnostics_;
    }
    [[nodiscard]] MutableVertexId vertexIdFromSourceIndex(
        std::size_t sourceIndex) const noexcept
    {
        return sourceIndex < mutableVertexIdBySourceIndex_.size()
            ? mutableVertexIdBySourceIndex_[sourceIndex] : kInvalidIndex;
    }
    [[nodiscard]] MutableEdgeId edgeIdFromSourceIndex(
        std::size_t sourceIndex) const noexcept
    {
        return sourceIndex < mutableEdgeIdBySourceIndex_.size()
            ? mutableEdgeIdBySourceIndex_[sourceIndex] : kInvalidIndex;
    }
    [[nodiscard]] MutableFaceId faceIdFromSourceIndex(
        std::size_t sourceIndex) const noexcept
    {
        return sourceIndex < mutableFaceIdBySourceIndex_.size()
            ? mutableFaceIdBySourceIndex_[sourceIndex] : kInvalidIndex;
    }

private:
    [[nodiscard]] MutableOperationResult executeMoveVertex(
        MutableVertexId vertexId,
        const Vec3& position,
        MutableOperationId operationId);
    [[nodiscard]] MutableOperationResult executeSplitEdge(
        MutableEdgeId edgeId,
        double parameter,
        MutableOperationId operationId);
    [[nodiscard]] MutableOperationResult executeCollapseEdge(
        MutableEdgeId edgeId,
        MutableVertexId endpointToKeep,
        MutableOperationId operationId);
    [[nodiscard]] MutableOperationResult executeDissolveEdge(
        MutableEdgeId edgeId,
        MutableOperationId operationId);
    [[nodiscard]] MutableOperationResult executeFlipEdge(
        MutableEdgeId edgeId,
        MutableOperationId operationId);

    [[nodiscard]] bool rebuildAffectedTopology(
        const std::vector<MutableFaceId>& affectedFaces,
        MutableOperationRecord& changes,
        std::string& diagnostic);
    [[nodiscard]] bool rebuildInnerInterface(std::string& diagnostic);
    void refreshDiagnostics();

    std::vector<MutableVertex> vertices_;
    std::vector<MutableEdge> edges_;
    std::vector<MutableFace> faces_;
    ScaffoldBoundaryLoop fixedOuterBoundary_;
    std::vector<MutableVertexId> orderedInnerInterfaceVertices_;
    std::vector<MutableEdgeId> orderedInnerInterfaceEdges_;
    std::vector<std::size_t> mutableVertexIdBySourceIndex_;
    std::vector<std::size_t> mutableEdgeIdBySourceIndex_;
    std::vector<std::size_t> mutableFaceIdBySourceIndex_;
    std::vector<MutableOperationRecord> operationLineage_;
    MutablePatchDiagnostics diagnostics_;
    MutableOperationId nextOperationId_ = 1U;
};

[[nodiscard]] const char* mutableOperationStatusName(
    MutableOperationStatus status) noexcept;

}  // namespace directional_retopo::solver
