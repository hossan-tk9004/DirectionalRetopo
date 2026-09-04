#pragma once

#include "Solver/RemeshContract.h"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace directional_retopo::solver {

enum class CoreGenerationStatus
{
    Success,
    InvalidInput,
    PatchBuildFailed,
    ParameterizationFailed,
    ExtractionFailed,
    SurfaceConformationFailed,
    ValidationFailed,
    CoreBoundaryInvalid,
};

enum class CoreBoundaryStatus
{
    Success,
    NoBoundary,
    MultipleBoundaryLoops,
    OpenBoundary,
    BranchedBoundary,
    NonManifold,
    DisconnectedCore,
    InvalidGeometry,
};

struct SurfacePointMapping final
{
    std::size_t sourceTriangleIndex = kInvalidIndex;
    SourceId sourceFaceId = kInvalidSourceId;
    Vec3 barycentric;
    Vec3 sourcePosition;
    Vec3 sourceNormal;
    double surfaceDistance = std::numeric_limits<double>::infinity();
    bool valid = false;
};

struct CoreBoundaryVertexDescriptor final
{
    std::size_t coreVertexId = kInvalidIndex;
    std::size_t orderedBoundaryIndex = kInvalidIndex;
    Vec3 position;
    Vec3 tangent;
    double normalizedArcLength = 0.0;
    SurfacePointMapping surface;
};

struct CoreBoundaryDescriptor final
{
    CoreBoundaryStatus status = CoreBoundaryStatus::InvalidGeometry;
    std::vector<std::size_t> orderedVertexIds;
    std::vector<CoreBoundaryVertexDescriptor> vertices;
    bool closed = false;
    double totalArcLength = 0.0;
    std::size_t boundaryDegreeViolationCount = 0U;
    std::size_t boundaryLoopCount = 0U;
    std::size_t holeCount = 0U;
    std::string diagnosticMessage;

    [[nodiscard]] bool success() const noexcept
    {
        return status == CoreBoundaryStatus::Success;
    }
};

struct CoreRemeshTimings final
{
    double patchBuildMilliseconds = 0.0;
    double parameterizationMilliseconds = 0.0;
    double extractionMilliseconds = 0.0;
    double conformationMilliseconds = 0.0;
    double boundaryMappingMilliseconds = 0.0;
    double validationMilliseconds = 0.0;
    double totalMilliseconds = 0.0;
};

struct CoreRemeshResult final
{
    std::size_t componentId = 0U;
    CoreGenerationStatus status = CoreGenerationStatus::InvalidInput;
    std::string diagnosticMessage;
    std::vector<Vec3> rawVertices;
    std::vector<Vec3> vertices;
    std::vector<ResultPolygon> polygons;
    std::vector<SurfacePointMapping> sourceMappings;
    std::vector<std::size_t> sourceFaceIndices;
    CoreBoundaryDescriptor boundary;
    std::size_t connectedComponentCount = 0U;
    std::size_t quadCount = 0U;
    std::size_t triangleCount = 0U;
    std::size_t nGonCount = 0U;
    bool usedInsetDomain = false;
    CoreRemeshTimings timings;
    std::uint64_t signature = 0U;

    [[nodiscard]] bool success() const noexcept
    {
        return status == CoreGenerationStatus::Success && boundary.success();
    }
};

[[nodiscard]] const char* coreGenerationStatusName(
    CoreGenerationStatus status) noexcept;

[[nodiscard]] const char* coreBoundaryStatusName(
    CoreBoundaryStatus status) noexcept;

[[nodiscard]] std::uint64_t coreRemeshResultSignature(
    const CoreRemeshResult& result) noexcept;

}  // namespace directional_retopo::solver
