#pragma once

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace directional_retopo {

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

enum class TriangleReason
{
    BoundaryCountMismatch,
    BoundaryParity,
    DensityTransition,
    FlowTermination,
    SolverFallback,
};

enum class ResultPolygonRegion
{
    Core,
    TransitionCollar,
};

struct TriangleDiagnostic final
{
    std::size_t polygonIndex = std::numeric_limits<std::size_t>::max();
    TriangleReason reason = TriangleReason::SolverFallback;
};

struct BoundaryLockedPatchDiagnostic final
{
    bool success = false;
    unsigned int topologyBlendWidth = 0U;
    std::size_t fixedBoundaryVertexCount = 0U;
    std::size_t fixedBoundaryEdgeCount = 0U;
    std::size_t innerBoundaryVertexCount = 0U;
    std::size_t innerBoundaryEdgeCount = 0U;
    std::size_t collarQuadCount = 0U;
    std::size_t collarTriangleCount = 0U;
    std::size_t coreQuadCount = 0U;
    std::size_t coreTriangleCount = 0U;
    std::size_t boundaryCrossingCount = 0U;
    double maximumSourceBoundaryDisplacement = 0.0;
    bool outerBoundaryTopologySimple = false;
    bool innerBoundaryTopologySimple = false;
    std::size_t outerBoundaryTrueIntersectionCount = 0U;
    std::size_t innerBoundaryTrueIntersectionCount = 0U;
    std::size_t selectedSeamOffset = 0U;
    bool innerOrderReversed = false;
    double boundaryAlignmentCost = 0.0;
};

struct ResultBoundaryLoop final
{
    std::vector<std::size_t> vertexIndices;
    bool closed = false;
    double totalLength = 0.0;
};

enum class BoundaryWinding
{
    Unknown,
    Aligned,
    Reversed,
};

struct BoundaryVertexCorrespondence final
{
    std::size_t resultVertexIndex = std::numeric_limits<std::size_t>::max();
    std::size_t resultOrderIndex = std::numeric_limits<std::size_t>::max();
    int sourceEdgeId = -1;
    int sourceVertex0 = -1;
    int sourceVertex1 = -1;
    double sourceEdgeParameter = 0.0;
    double resultNormalizedParameter = 0.0;
    double sourceNormalizedParameter = 0.0;
    double sourceUnwrappedParameter = 0.0;
    MPoint resultPositionBeforeConformation;
    MPoint sourcePosition;
    double distanceBeforeConformation = 0.0;
    double distanceAfterConformation = 0.0;
};

struct RequiredBoundaryAnchor final
{
    int sourceVertexId = -1;
    std::size_t sourceLoopVertexIndex = std::numeric_limits<std::size_t>::max();
    double normalizedArcLength = 0.0;
    std::size_t resultEdgeIndex = std::numeric_limits<std::size_t>::max();
    std::size_t resultVertex0 = std::numeric_limits<std::size_t>::max();
    std::size_t resultVertex1 = std::numeric_limits<std::size_t>::max();
    double boundaryCornerAngleRadians = 0.0;
    double shortcutDistance = 0.0;
    MPoint sourcePosition;
    bool requiresResultSplit = true;
};

struct BoundaryLoopCorrespondence final
{
    std::size_t sourceLoopIndex = std::numeric_limits<std::size_t>::max();
    std::size_t resultLoopIndex = std::numeric_limits<std::size_t>::max();
    bool sourceClosed = false;
    bool resultClosed = false;
    bool closedStateMatches = false;
    BoundaryWinding winding = BoundaryWinding::Unknown;
    double sourceSeamParameter = 0.0;
    std::size_t resultSeamOffset = 0U;
    bool resultOrderReversed = false;
    bool windingAlignedAfterConformation = false;
    bool orderedMappingValid = false;
    std::size_t sourceVertexCount = 0U;
    std::size_t sourceEdgeCount = 0U;
    std::size_t resultVertexCount = 0U;
    std::size_t resultEdgeCount = 0U;
    long long vertexCountDifference = 0;
    double sourceTotalArcLength = 0.0;
    double resultTotalArcLengthBefore = 0.0;
    double resultTotalArcLengthAfter = 0.0;
    double meanDistanceBefore = 0.0;
    double maximumDistanceBefore = 0.0;
    double meanDistanceAfter = 0.0;
    double maximumDistanceAfter = 0.0;
    std::size_t monotonicViolationCount = 0U;
    std::size_t selfIntersectionCount = 0U;
    std::size_t sourceCrossingCount = 0U;
    std::size_t duplicatedParameterCount = 0U;
    std::size_t zeroLengthBoundaryEdgeCount = 0U;
    std::vector<MPoint> sourcePolylinePositions;
    std::vector<BoundaryVertexCorrespondence> vertices;
    std::vector<RequiredBoundaryAnchor> requiredBoundaryAnchors;
};

struct ResultVertexSourceMapping final
{
    std::size_t patchTriangleIndex = std::numeric_limits<std::size_t>::max();
    int sourceFaceId = -1;
    MPoint rawPosition;
    MPoint projectedPosition;
    MVector sourceNormal;
    double projectionDistance = 0.0;
    double surfaceDistance = std::numeric_limits<double>::infinity();
};

struct FidelityBounds final
{
    MPoint minimum;
    MPoint maximum;
    bool valid = false;
};

struct SurfaceFidelityMetrics final
{
    double sourceArea = 0.0;
    double rawQuadArea = 0.0;
    double rawAreaRatio = 0.0;
    double conformedArea = 0.0;
    double conformedAreaRatio = 0.0;
    double sourceAverageEdgeLength = 0.0;
    double sourceMedianEdgeLength = 0.0;
    double meanRawSurfaceDistance = 0.0;
    double maximumRawSurfaceDistance = 0.0;
    double meanProjectionDistance = 0.0;
    double maximumProjectionDistance = 0.0;
    double meanConformedSurfaceDistance = 0.0;
    double maximumConformedSurfaceDistance = 0.0;
    bool distanceQualityWarning = false;
    FidelityBounds sourceBounds;
    FidelityBounds rawBounds;
    FidelityBounds conformedBounds;
};

struct BoundaryComparisonDiagnostic final
{
    std::size_t sourceLoopCount = 0;
    std::size_t resultLoopCount = 0;
    std::size_t sourceVertexCount = 0;
    std::size_t resultVertexCount = 0;
    std::size_t sourceEdgeCount = 0;
    std::size_t resultEdgeCount = 0;
    long long vertexCountDifference = 0;
    double sourceTotalLength = 0.0;
    double resultTotalLength = 0.0;
    double meanNearestDistance = 0.0;
    double maximumNearestDistance = 0.0;
    bool orientationAligned = true;
    bool closedStateMatches = true;
    bool correspondenceComplete = false;
    std::size_t alignedLoopCount = 0U;
    std::size_t reversedLoopCount = 0U;
    std::size_t monotonicViolationCount = 0U;
    std::size_t crossingCount = 0U;
    std::size_t requiredBoundaryAnchorCount = 0U;
    std::size_t requiredResultSplitCount = 0U;
    bool orderedMappingValid = false;
};

struct QuadPatchResult final
{
    std::size_t componentId = 0;
    bool success = false;
    bool boundaryLocked = false;
    bool debugPreviewAvailable = false;
    bool debugInnerResultOnly = false;
    TopologyPolicy topologyPolicy = TopologyPolicy::QuadDominant;
    TrianglePolicy trianglePolicy = TrianglePolicy::MinimalNecessary;
    std::vector<MPoint> rawVertices;
    std::vector<MPoint> conformedVertices;
    std::vector<std::vector<std::size_t>> polygons;
    std::vector<ResultPolygonRegion> polygonRegions;
    std::vector<ResultBoundaryLoop> boundaryLoops;
    std::vector<BoundaryLoopCorrespondence> boundaryCorrespondences;
    std::vector<ResultVertexSourceMapping> rawSourceMappings;
    std::vector<ResultVertexSourceMapping> sourceMappings;
    std::vector<std::size_t> fixedBoundaryVertexIndices;
    std::vector<TriangleDiagnostic> triangleDiagnostics;
    std::size_t quadCount = 0;
    std::size_t triangleCount = 0;
    std::size_t nGonCount = 0;
    std::size_t nonQuadCount = 0;
    double targetEdgeLength = 0.0;
    double maximumSurfaceDistance = 0.0;
    SurfaceFidelityMetrics fidelity;
    BoundaryComparisonDiagnostic boundaryDiagnostic;
    BoundaryLockedPatchDiagnostic boundaryLockedDiagnostic;
    std::string diagnosticMessage;

    void clear() noexcept
    {
        *this = QuadPatchResult();
    }
};

struct QuadSolveTimings final
{
    double patchBuildMilliseconds = 0.0;
    double parameterizationMilliseconds = 0.0;
    double extractionMilliseconds = 0.0;
    double conformationMilliseconds = 0.0;
    double validationMilliseconds = 0.0;
    double collarBuildMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
};

struct QuadComponentSolveReport final
{
    std::size_t componentId = 0;
    std::size_t patchVertexCount = 0;
    std::size_t patchTriangleCount = 0;
    bool parameterizationSuccess = false;
    bool extractionSuccess = false;
    bool conformationSuccess = false;
    bool boundaryLockedCollarSuccess = false;
    bool boundaryLockedCollarAttempted = false;
    bool finalConformationAttempted = false;
    bool finalValidationAttempted = false;
    bool finalConformationSuccess = false;
    bool finalValidationSuccess = false;
    double maximumGuidanceDeviationDegrees = 0.0;
    double minimumEffectiveTargetEdgeLength = 0.0;
    double meanEffectiveTargetEdgeLength = 0.0;
    double maximumEffectiveTargetEdgeLength = 0.0;
    std::size_t curvatureLimitedTriangleCount = 0U;
    QuadSolveTimings timings;
    QuadPatchResult result;
    std::string diagnosticMessage;
};

}  // namespace directional_retopo
