#pragma once

#include "Solver/CoreRemeshResult.h"
#include "Solver/ScaffoldAdaptationSolver.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace directional_retopo::solver {

enum class InterfaceDescriptorStatus
{
    Success,
    Missing,
    Open,
    Branched,
    InvalidGeometry,
    SurfaceMappingFailed,
};

struct AdaptedInterfaceVertexDescriptor final
{
    MutableVertexId mutableVertexId = kInvalidIndex;
    std::size_t orderedBoundaryIndex = kInvalidIndex;
    Vec3 position;
    Vec3 tangent;
    Vec3 normal;
    double normalizedArcLength = 0.0;
    int minimumRingDepth = -1;
    bool fixedOuterBoundary = false;
    SurfacePointMapping surface;
};

struct AdaptedInterfaceDescriptor final
{
    InterfaceDescriptorStatus status = InterfaceDescriptorStatus::Missing;
    std::vector<MutableVertexId> orderedVertexIds;
    std::vector<MutableEdgeId> orderedEdgeIds;
    std::vector<AdaptedInterfaceVertexDescriptor> vertices;
    bool closed = false;
    double totalArcLength = 0.0;
    std::string diagnosticMessage;

    [[nodiscard]] bool success() const noexcept
    {
        return status == InterfaceDescriptorStatus::Success;
    }
};

struct InterfaceCorrespondenceEntry final
{
    std::size_t scaffoldOrderIndex = kInvalidIndex;
    std::size_t coreOrderIndex = kInvalidIndex;
    double scaffoldNormalizedArcLength = 0.0;
    double coreNormalizedArcLength = 0.0;
    double positionDistance = 0.0;
    double tangentDeviation = 0.0;
    double normalDeviation = 0.0;
};

struct InterfaceCorrespondence final
{
    bool success = false;
    bool coreOrderReversed = false;
    std::size_t coreSeamOffset = 0U;
    double totalCost = std::numeric_limits<double>::infinity();
    std::size_t monotonicViolationCount = 0U;
    std::vector<InterfaceCorrespondenceEntry> entries;
    std::string diagnosticMessage;
};

enum class ReconciliationStatus
{
    Success,
    Partial,
    InvalidInput,
    OperationFailed,
    ExcessiveMismatch,
};

struct InterfaceReconciliationResult final
{
    ReconciliationStatus status = ReconciliationStatus::InvalidInput;
    LocalMutablePatchMesh mesh;
    AdaptedInterfaceDescriptor interface;
    std::size_t scaffoldCountBefore = 0U;
    std::size_t coreCount = 0U;
    std::size_t scaffoldCountAfter = 0U;
    std::size_t absoluteDifferenceBefore = 0U;
    std::size_t absoluteDifferenceAfter = 0U;
    double countRatioBefore = 0.0;
    double countRatioAfter = 0.0;
    bool parityMatchedBefore = false;
    bool parityMatchedAfter = false;
    std::size_t splitCount = 0U;
    std::size_t collapseCount = 0U;
    std::vector<MutableOperationRecord> operations;
    std::string diagnosticMessage;

    [[nodiscard]] bool usable() const noexcept
    {
        return status == ReconciliationStatus::Success ||
            status == ReconciliationStatus::Partial;
    }
};

enum class JoinTriangleReason
{
    InterfaceCountMismatch,
    ParityTermination,
    FlowTermination,
    JoinFallback,
};

enum class JoinVertexDomain
{
    Scaffold,
    Core,
};

struct JoinVertexReference final
{
    JoinVertexDomain domain = JoinVertexDomain::Scaffold;
    std::size_t vertexId = kInvalidIndex;
};

struct InterfaceJoinFace final
{
    std::vector<JoinVertexReference> vertices;
    JoinTriangleReason triangleReason =
        JoinTriangleReason::InterfaceCountMismatch;
    double cost = 0.0;
};

enum class InterfaceJoinStatus
{
    Success,
    InvalidInput,
    NoCorrespondence,
    ExcessiveMismatch,
    NoFeasibleJoin,
    ValidationFailed,
};

struct InterfaceJoinResult final
{
    InterfaceJoinStatus status = InterfaceJoinStatus::InvalidInput;
    InterfaceCorrespondence correspondence;
    std::vector<InterfaceJoinFace> faces;
    std::size_t triangleCount = 0U;
    std::size_t quadCount = 0U;
    std::size_t nGonCount = 0U;
    std::size_t seamCandidatesTested = 0U;
    std::size_t feasibleSeamCount = 0U;
    std::size_t rejectedGeometryCandidateCount = 0U;
    double correspondenceMilliseconds = 0.0;
    double meanSurfaceError = 0.0;
    double maximumSurfaceError = 0.0;
    double meanDirectionDeviationDegrees = 0.0;
    double totalCost = std::numeric_limits<double>::infinity();
    std::string diagnosticMessage;

    [[nodiscard]] bool success() const noexcept
    {
        return status == InterfaceJoinStatus::Success;
    }
};

enum class CombinedVertexOrigin
{
    Scaffold,
    Core,
};

struct CombinedVertex final
{
    Vec3 position;
    CombinedVertexOrigin origin = CombinedVertexOrigin::Scaffold;
    std::size_t sourceLocalId = kInvalidIndex;
    SurfacePointMapping surface;
    bool fixedOuterBoundary = false;
};

enum class CombinedPolygonRegion
{
    TransitionScaffold,
    InterfaceJoin,
    Core,
};

struct CombinedPolygon final
{
    std::vector<std::size_t> vertexIndices;
    PolygonType type = PolygonType::NGon;
    CombinedPolygonRegion region =
        CombinedPolygonRegion::TransitionScaffold;
    JoinTriangleReason triangleReason =
        JoinTriangleReason::JoinFallback;
};

struct CombinedRemeshMetrics final
{
    std::size_t connectedComponentCount = 0U;
    std::size_t outerBoundaryLoopCount = 0U;
    std::size_t nonManifoldEdgeCount = 0U;
    std::size_t zeroAreaPolygonCount = 0U;
    std::size_t duplicatePolygonCount = 0U;
    std::size_t boundaryCrossingCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t quadCount = 0U;
    std::size_t nGonCount = 0U;
    std::size_t joinNGonCount = 0U;
    double maximumFixedBoundaryDisplacement = 0.0;
};

struct CombinedRemeshResult final
{
    bool success = false;
    std::vector<CombinedVertex> vertices;
    std::vector<CombinedPolygon> polygons;
    std::vector<ResultBoundaryLoop> boundaryLoops;
    std::vector<std::size_t> fixedBoundaryVertexIndices;
    CombinedRemeshMetrics metrics;
    std::uint64_t signature = 0U;
    std::string diagnosticMessage;
};

enum class CoreJoinStatus
{
    Success,
    Partial,
    CoreGenerationFailed,
    CoreBoundaryInvalid,
    InterfaceInvalid,
    CorrespondenceFailed,
    ReconciliationFailed,
    JoinFailed,
    CombinedValidationFailed,
};

struct CoreInterfaceJoinSettings final
{
    std::size_t maximumReconciliationOperations = 32U;
    std::size_t residualTriangleBudget = 2U;
    std::size_t maximumSeamCandidates = 32U;
    std::size_t maximumResidualTriangleCount = 32U;
    std::size_t maximumDpStatesPerCandidate = 250000U;
    std::size_t maximumTotalDpStates = 10000U;
    std::size_t minimumLoopVertexCount = 3U;
    double maximumJoinCountRatio = 4.0;
    double maximumResidualTriangleFraction = 0.35;
    double trianglePenalty = 4.0;
    double geometryEpsilon = 1.0e-12;
    double relativeAreaEpsilon = 1.0e-8;
    double maximumJoinSurfaceDistanceRatio = 0.75;
    double densityCostWeight = 1.0;
    double directionCostWeight = 1.5;
    double surfaceCostWeight = 12.0;
    double qualityCostWeight = 1.0;
};

struct CoreInterfaceJoinTimings final
{
    double interfaceDescriptorMilliseconds = 0.0;
    double correspondenceMilliseconds = 0.0;
    double reconciliationMilliseconds = 0.0;
    double joinMilliseconds = 0.0;
    double combinedValidationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
};

struct CoreInterfaceJoinResult final
{
    std::size_t componentId = 0U;
    CoreJoinStatus status = CoreJoinStatus::InterfaceInvalid;
    ScaffoldAdaptationStatus scaffoldStatus =
        ScaffoldAdaptationStatus::Failure;
    CoreRemeshResult core;
    AdaptedInterfaceDescriptor interfaceBefore;
    InterfaceReconciliationResult reconciliation;
    InterfaceJoinResult join;
    CombinedRemeshResult combined;
    CoreInterfaceJoinTimings timings;
    std::uint64_t signature = 0U;
    std::string diagnosticMessage;

    [[nodiscard]] bool usable() const noexcept
    {
        return status == CoreJoinStatus::Success ||
            status == CoreJoinStatus::Partial;
    }
};

class CoreInterfaceJoinSolver final
{
public:
    [[nodiscard]] CoreInterfaceJoinResult join(
        const RemeshInput& input,
        const RegionComponent& component,
        const SourceTransitionScaffold& scaffold,
        const AdaptedScaffoldResult& adaptation,
        const CoreRemeshResult& core,
        const CoreInterfaceJoinSettings& settings = {}) const noexcept;
};

[[nodiscard]] const char* coreJoinStatusName(CoreJoinStatus status) noexcept;
[[nodiscard]] const char* interfaceJoinStatusName(
    InterfaceJoinStatus status) noexcept;
[[nodiscard]] const char* reconciliationStatusName(
    ReconciliationStatus status) noexcept;
[[nodiscard]] const char* joinTriangleReasonName(
    JoinTriangleReason reason) noexcept;

}  // namespace directional_retopo::solver
