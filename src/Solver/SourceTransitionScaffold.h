#pragma once

#include "Solver/RemeshContract.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace directional_retopo::solver {

enum class ScaffoldStatus
{
    Success,
    InvalidInput,
    MissingOuterBoundary,
    MultipleOuterBoundaries,
    OuterBoundaryInvalid,
    MissingInnerInterface,
    MultipleInnerInterfaces,
    OpenInnerInterface,
    BranchedInnerInterface,
    NonManifoldTransition,
    CoreDisconnected,
    RingDepthInvalid,
    AmbiguousTransitionTopology,
};

enum class ScaffoldVertexClassification : std::uint8_t
{
    None = 0U,
    FixedOuterBoundary = 1U << 0U,
    TransitionInterior = 1U << 1U,
    InnerInterface = 1U << 2U,
};

enum class ScaffoldEdgeClassification
{
    FixedOuterBoundary,
    TransitionInterior,
    InnerInterface,
};

struct ScaffoldVertex final
{
    std::size_t localIndex = kInvalidIndex;
    std::size_t sourceVertexIndex = kInvalidIndex;
    SourceId sourceVertexId = kInvalidSourceId;
    Vec3 position;
    Vec3 normal;
    ScaffoldVertexClassification classification =
        ScaffoldVertexClassification::None;
};

struct ScaffoldEdge final
{
    std::size_t localIndex = kInvalidIndex;
    std::size_t sourceEdgeIndex = kInvalidIndex;
    SourceId sourceEdgeId = kInvalidSourceId;
    std::array<std::size_t, 2> vertexIndices = {
        kInvalidIndex,
        kInvalidIndex};
    std::vector<std::size_t> faceIndices;
    ScaffoldEdgeClassification classification =
        ScaffoldEdgeClassification::TransitionInterior;
};

struct ScaffoldFace final
{
    std::size_t localIndex = kInvalidIndex;
    std::size_t sourceFaceIndex = kInvalidIndex;
    SourceId sourceFaceId = kInvalidSourceId;
    std::vector<std::size_t> vertexIndices;
    std::vector<std::size_t> edgeIndices;
    int transitionRingDepth = -1;
    PolygonType polygonType = PolygonType::NGon;
};

struct ScaffoldBoundaryLoop final
{
    std::vector<std::size_t> vertexIndices;
    std::vector<std::size_t> edgeIndices;
    std::vector<std::size_t> sourceVertexIndices;
    std::vector<std::size_t> sourceEdgeIndices;
    std::vector<SourceId> sourceVertexIds;
    std::vector<SourceId> sourceEdgeIds;
    bool closed = false;
};

struct ScaffoldDiagnostics final
{
    std::size_t transitionRingCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t quadCount = 0U;
    std::size_t nGonCount = 0U;
    std::size_t nonManifoldEdgeCount = 0U;
    std::size_t openInterfaceCount = 0U;
    std::size_t branchedInterfaceVertexCount = 0U;
    std::size_t invalidReferenceCount = 0U;
    std::size_t ringDepthWarningCount = 0U;
    double vertexMappingCoverage = 0.0;
    double edgeMappingCoverage = 0.0;
    double faceMappingCoverage = 0.0;
    double fixedBoundaryVertexCoverage = 0.0;
    double fixedBoundaryEdgeCoverage = 0.0;
    double maximumFixedBoundaryDisplacement = 0.0;
    double extractionMilliseconds = 0.0;
};

struct SourceTransitionScaffold final
{
    std::size_t componentId = 0U;
    ScaffoldStatus status = ScaffoldStatus::InvalidInput;
    std::string diagnosticMessage;

    std::vector<ScaffoldVertex> vertices;
    std::vector<ScaffoldEdge> edges;
    std::vector<ScaffoldFace> faces;
    std::vector<ScaffoldBoundaryLoop> fixedOuterBoundaryLoops;
    std::vector<ScaffoldBoundaryLoop> innerInterfaceLoops;

    // Stable inverse mappings indexed by SourceMeshSnapshot array index.
    // Entries outside this scaffold are kInvalidIndex.
    std::vector<std::size_t> localVertexIndexBySource;
    std::vector<std::size_t> localEdgeIndexBySource;
    std::vector<std::size_t> localFaceIndexBySource;

    ScaffoldDiagnostics diagnostics;

    [[nodiscard]] bool success() const noexcept
    {
        return status == ScaffoldStatus::Success;
    }
};

class SourceTransitionScaffoldExtractor final
{
public:
    [[nodiscard]] SourceTransitionScaffold extract(
        const SourceMeshSnapshot& sourceMesh,
        const RegionComponent& component,
        const RemeshSettings& settings) const;
};

[[nodiscard]] const char* scaffoldStatusName(ScaffoldStatus status) noexcept;

[[nodiscard]] std::uint64_t sourceTransitionScaffoldSignature(
    const SourceTransitionScaffold& scaffold) noexcept;

[[nodiscard]] bool hasClassification(
    ScaffoldVertexClassification value,
    ScaffoldVertexClassification classification) noexcept;

}  // namespace directional_retopo::solver
