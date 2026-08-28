#pragma once

#include "Solver/PortableMath.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace directional_retopo::solver {

using SourceId = std::int64_t;
constexpr SourceId kInvalidSourceId = -1;
constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();

enum class TopologyPolicy
{
    StrictAllQuads,
    QuadDominant,
};

enum class TrianglePolicy
{
    Disallow,
    MinimalNecessary,
};

enum class SolveStatus
{
    Success,
    Failed,
};

enum class FailureCode
{
    Success,
    InvalidInput,
    RegionTooSmall,
    PatchBuildFailed,
    ParameterizationFailed,
    QuadExtractionFailed,
    SurfaceConformationFailed,
    InnerBoundaryInvalid,
    BoundaryLoopMismatch,
    TransitionBuildFailed,
    FinalValidationFailed,
    ZeroAreaPolygon,
    BoundaryCrossing,
    NonManifoldResult,
    UnknownFailure,
};

enum class PolygonType
{
    Triangle,
    Quad,
    NGon,
};

enum class PolygonRegion
{
    Core,
    Transition,
};

enum class TriangleReason
{
    BoundaryCountMismatch,
    BoundaryParity,
    DensityTransition,
    FlowTermination,
    SmallHoleRepair,
    SolverFallback,
};

struct SourceVertex final
{
    Vec3 position;
    Vec3 normal;
    SourceId sourceVertexId = kInvalidSourceId;
    std::vector<std::size_t> adjacentVertexIndices;
    std::vector<std::size_t> edgeIndices;
    std::vector<std::size_t> faceIndices;
};

struct SourceEdge final
{
    std::array<std::size_t, 2> vertexIndices = {kInvalidIndex, kInvalidIndex};
    std::vector<std::size_t> faceIndices;
    SourceId sourceEdgeId = kInvalidSourceId;
    double length = 0.0;
    bool originalMeshBoundary = false;
};

struct SourceTriangle final
{
    std::array<std::size_t, 3> vertexIndices = {
        kInvalidIndex,
        kInvalidIndex,
        kInvalidIndex};
    std::size_t faceIndex = kInvalidIndex;
};

struct SourceFace final
{
    std::vector<std::size_t> vertexIndices;
    std::vector<std::size_t> edgeIndices;
    std::vector<std::size_t> adjacentFaceIndices;
    std::vector<std::size_t> triangleIndices;
    Vec3 center;
    Vec3 normal;
    SourceId sourceFaceId = kInvalidSourceId;
    bool geometryValid = false;
};

struct SourceMeshSnapshot final
{
    std::vector<SourceVertex> vertices;
    std::vector<SourceEdge> edges;
    std::vector<SourceFace> faces;
    std::vector<SourceTriangle> triangles;

    [[nodiscard]] bool valid(std::string* reason = nullptr) const noexcept;
};

struct OrderedBoundaryLoop final
{
    std::vector<std::size_t> vertexIndices;
    std::vector<std::size_t> edgeIndices;
    std::vector<SourceId> sourceVertexIds;
    std::vector<SourceId> sourceEdgeIds;
    bool closed = false;
    bool touchesOriginalMeshBoundary = false;
};

struct RegionComponent final
{
    std::size_t componentId = 0;
    std::vector<std::size_t> coreFaceIndices;
    std::vector<std::size_t> transitionFaceIndices;
    std::vector<std::size_t> allFaceIndices;
    // Indexed by SourceMeshSnapshot face index. Core=0, Transition=1..N,
    // outside=-1. This is also the future Source Scaffold ring coordinate.
    std::vector<int> transitionRingDepthByFace;
    std::vector<OrderedBoundaryLoop> fixedBoundaryLoops;
};

struct FaceDirection final
{
    Vec3 normal;
    Vec3 uDirection;
    Vec3 vDirection;
    double paintConstraintWeight = 0.0;
    double topologyGuidanceWeight = 0.0;
    bool valid = false;
};

struct FaceDensity final
{
    double requestedTargetEdgeLength = 0.0;
    double effectiveTargetEdgeLength = 0.0;
    double scaleU = 1.0;
    double scaleV = 1.0;
    bool curvatureConstrained = false;
    bool valid = false;
};

struct RemeshSettings final
{
    unsigned int topologyBlendWidth = 2U;
    TopologyPolicy topologyPolicy = TopologyPolicy::QuadDominant;
    TrianglePolicy trianglePolicy = TrianglePolicy::MinimalNecessary;
    unsigned int maximumRetryAttempts = 3U;
    double geometryEpsilon = 1.0e-12;
    double areaEpsilon = 1.0e-12;
    bool retainDebugResults = true;
};

struct RemeshInput final
{
    SourceMeshSnapshot sourceMesh;
    std::vector<RegionComponent> components;
    // Both arrays are indexed by SourceMeshSnapshot face index.
    std::vector<FaceDirection> directionField;
    std::vector<FaceDensity> densityField;
    RemeshSettings settings;

    [[nodiscard]] bool valid(std::string* reason = nullptr) const noexcept;
};

struct ResultSourceMapping final
{
    std::size_t sourceTriangleIndex = kInvalidIndex;
    SourceId sourceFaceId = kInvalidSourceId;
    double surfaceDistance = std::numeric_limits<double>::infinity();
};

struct ResultPolygon final
{
    std::vector<std::size_t> vertexIndices;
    PolygonType type = PolygonType::NGon;
    PolygonRegion region = PolygonRegion::Core;
    TriangleReason triangleReason = TriangleReason::SolverFallback;
};

struct ResultBoundaryLoop final
{
    std::vector<std::size_t> vertexIndices;
    bool closed = false;
};

struct FixedBoundaryMapping final
{
    std::size_t resultVertexIndex = kInvalidIndex;
    SourceId sourceVertexId = kInvalidSourceId;
    Vec3 sourcePosition;
};

struct QualityMetrics final
{
    std::size_t quadCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t nGonCount = 0U;
    std::size_t boundaryCrossingCount = 0U;
    std::size_t nonManifoldEdgeCount = 0U;
    std::size_t zeroAreaPolygonCount = 0U;
    double maximumBoundaryDisplacement = 0.0;
    double meanSurfaceDistance = 0.0;
    double p95SurfaceDistance = 0.0;
    double maximumSurfaceDistance = 0.0;
    double meanCoreDirectionDeviationDegrees = 0.0;
    double maximumCoreDirectionDeviationDegrees = 0.0;
    double requestedCoreEdgeLength = 0.0;
    double actualCoreEdgeLength = 0.0;
};

struct TimingMetrics final
{
    double patchBuildMilliseconds = 0.0;
    double parameterizationMilliseconds = 0.0;
    double extractionMilliseconds = 0.0;
    double conformationMilliseconds = 0.0;
    double transitionMilliseconds = 0.0;
    double validationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
};

struct ComponentResult final
{
    std::size_t componentId = 0U;
    SolveStatus status = SolveStatus::Failed;
    FailureCode failureCode = FailureCode::UnknownFailure;
    std::string failedStage;
    std::string diagnosticMessage;
    std::vector<Vec3> rawVertices;
    std::vector<Vec3> vertices;
    std::vector<ResultPolygon> polygons;
    std::vector<ResultBoundaryLoop> boundaryLoops;
    std::vector<ResultSourceMapping> sourceMappings;
    std::vector<FixedBoundaryMapping> fixedBoundaryMappings;
    QualityMetrics quality;
    TimingMetrics timings;
    unsigned int retryCount = 0U;
    std::string retryReason;
    bool debugOnly = false;
};

struct RemeshResult final
{
    SolveStatus status = SolveStatus::Failed;
    FailureCode failureCode = FailureCode::UnknownFailure;
    std::vector<ComponentResult> components;
    std::vector<ComponentResult> debugComponents;
    std::vector<std::string> warnings;
    TimingMetrics timings;

    [[nodiscard]] bool success() const noexcept
    {
        return status == SolveStatus::Success;
    }
};

[[nodiscard]] const char* failureCodeName(FailureCode code) noexcept;

}  // namespace directional_retopo::solver
