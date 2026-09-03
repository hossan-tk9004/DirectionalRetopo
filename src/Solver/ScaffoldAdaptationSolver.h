#pragma once

#include "Solver/LocalMutablePatchMesh.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace directional_retopo::solver {

enum class ScaffoldAdaptationStatus
{
    Success,
    Partial,
    Failure,
};

enum class ScaffoldAdaptationStopReason
{
    Converged,
    NoImprovingCandidate,
    OperationBudgetReached,
    SurfaceConstraintReached,
    InterfaceMinimumReached,
    UnsupportedSplit,
    InvalidInput,
    ValidationFailure,
};

enum class ScaffoldCandidateType
{
    CollapseEdgeToEndpoint,
    SplitSourceLineageEdge,
    FlipTriangleEdge,
};

struct ScaffoldAdaptationSettings final
{
    unsigned int maxOperations = 32U;
    unsigned int maxPasses = 32U;
    std::size_t maximumCandidatesPerPass = 48U;
    double minimumCostImprovement = 1.0e-5;
    double collapseLengthRatioThreshold = 0.90;
    double splitLengthRatioThreshold = 1.50;
    double maximumSurfaceDistanceRatio = 0.40;
    double maximumSurfaceErrorIncreaseRatio = 0.05;
    std::size_t minimumInterfaceVertexCount = 3U;
    bool enableCollapse = true;
    bool enableSourceLineageSplit = true;
    bool enableTriangleFlip = false;
    bool enableDissolve = false;
};

struct ScaffoldAdaptationCost final
{
    double density = 0.0;
    double direction = 0.0;
    double surface = 0.0;
    double topologyQuality = 0.0;
    double valencePenalty = 0.0;
    double sourcePreservation = 0.0;
    double total = 0.0;
};

struct ScaffoldAdaptationMetrics final
{
    std::size_t vertexCount = 0U;
    std::size_t edgeCount = 0U;
    std::size_t faceCount = 0U;
    std::size_t innerInterfaceVertexCount = 0U;
    std::size_t approximateDesiredInterfaceCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t quadCount = 0U;
    std::size_t nGonCount = 0U;
    std::size_t nonManifoldCount = 0U;
    std::size_t zeroAreaCount = 0U;
    double meanDensityError = 0.0;
    double maximumDensityError = 0.0;
    double meanDirectionDeviationDegrees = 0.0;
    double maximumDirectionDeviationDegrees = 0.0;
    double meanSurfaceError = 0.0;
    double maximumSurfaceError = 0.0;
    double maximumFixedBoundaryDisplacement = 0.0;
    ScaffoldAdaptationCost cost;
};

struct ScaffoldCandidateDiagnostics final
{
    std::size_t generated = 0U;
    std::size_t simulated = 0U;
    std::size_t valid = 0U;
    std::size_t rejectedByLinkCondition = 0U;
    std::size_t rejectedByTopology = 0U;
    std::size_t rejectedBySurface = 0U;
    std::size_t rejectedWithoutImprovement = 0U;
};

struct ScaffoldAdaptationOperation final
{
    ScaffoldCandidateType type =
        ScaffoldCandidateType::CollapseEdgeToEndpoint;
    MutableEdgeId edgeId = kInvalidIndex;
    MutableVertexId endpointToKeep = kInvalidIndex;
    double splitParameter = 0.5;
    double costBefore = 0.0;
    double costAfter = 0.0;
    MutableOperationRecord lineage;
    std::string description;
};

struct AdaptedScaffoldResult final
{
    ScaffoldAdaptationStatus status = ScaffoldAdaptationStatus::Failure;
    ScaffoldAdaptationStopReason stopReason =
        ScaffoldAdaptationStopReason::InvalidInput;
    std::string diagnosticMessage;
    LocalMutablePatchMesh adaptedMesh;
    ScaffoldAdaptationMetrics before;
    ScaffoldAdaptationMetrics after;
    ScaffoldCandidateDiagnostics candidates;
    std::vector<ScaffoldAdaptationOperation> operations;
    std::vector<MutableVertexId> innerInterfaceBefore;
    std::vector<MutableVertexId> innerInterfaceAfter;
    unsigned int passes = 0U;
    double adaptationMilliseconds = 0.0;
    std::uint64_t finalSignature = 0U;

    [[nodiscard]] bool validResult() const noexcept
    {
        return status != ScaffoldAdaptationStatus::Failure;
    }
};

class ScaffoldAdaptationSolver final
{
public:
    [[nodiscard]] AdaptedScaffoldResult adapt(
        const SourceTransitionScaffold& scaffold,
        const LocalMutablePatchMesh& mutablePatch,
        const SourceMeshSnapshot& sourceMesh,
        const RegionComponent& component,
        const std::vector<FaceDirection>& directionField,
        const std::vector<FaceDensity>& densityField,
        const RemeshSettings& remeshSettings,
        const ScaffoldAdaptationSettings& settings = {}) const;
};

[[nodiscard]] const char* scaffoldAdaptationStatusName(
    ScaffoldAdaptationStatus status) noexcept;

[[nodiscard]] const char* scaffoldAdaptationStopReasonName(
    ScaffoldAdaptationStopReason reason) noexcept;

[[nodiscard]] const char* scaffoldCandidateTypeName(
    ScaffoldCandidateType type) noexcept;

}  // namespace directional_retopo::solver
