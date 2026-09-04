#include "Solver/CoreRemeshResult.h"

#include <cstring>

namespace directional_retopo::solver {
namespace {

std::uint64_t fnv(std::uint64_t hash, std::uint64_t value) noexcept
{
    hash ^= value;
    hash *= 1099511628211ULL;
    return hash;
}

std::uint64_t bits(double value) noexcept
{
    std::uint64_t result = 0U;
    static_assert(sizeof(result) == sizeof(value));
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

}  // namespace

const char* coreGenerationStatusName(CoreGenerationStatus status) noexcept
{
    switch (status) {
    case CoreGenerationStatus::Success: return "Success";
    case CoreGenerationStatus::InvalidInput: return "InvalidInput";
    case CoreGenerationStatus::PatchBuildFailed: return "PatchBuildFailed";
    case CoreGenerationStatus::ParameterizationFailed:
        return "ParameterizationFailed";
    case CoreGenerationStatus::ExtractionFailed: return "ExtractionFailed";
    case CoreGenerationStatus::SurfaceConformationFailed:
        return "SurfaceConformationFailed";
    case CoreGenerationStatus::ValidationFailed: return "ValidationFailed";
    case CoreGenerationStatus::CoreBoundaryInvalid:
        return "CoreBoundaryInvalid";
    }
    return "Unknown";
}

const char* coreBoundaryStatusName(CoreBoundaryStatus status) noexcept
{
    switch (status) {
    case CoreBoundaryStatus::Success: return "Success";
    case CoreBoundaryStatus::NoBoundary: return "NoBoundary";
    case CoreBoundaryStatus::MultipleBoundaryLoops:
        return "MultipleBoundaryLoops";
    case CoreBoundaryStatus::OpenBoundary: return "OpenBoundary";
    case CoreBoundaryStatus::BranchedBoundary: return "BranchedBoundary";
    case CoreBoundaryStatus::NonManifold: return "NonManifold";
    case CoreBoundaryStatus::DisconnectedCore: return "DisconnectedCore";
    case CoreBoundaryStatus::InvalidGeometry: return "InvalidGeometry";
    }
    return "Unknown";
}

std::uint64_t coreRemeshResultSignature(
    const CoreRemeshResult& result) noexcept
{
    std::uint64_t hash = 1469598103934665603ULL;
    const auto add = [&hash](std::uint64_t value) { hash = fnv(hash, value); };
    add(static_cast<std::uint64_t>(result.status));
    add(result.componentId);
    add(result.vertices.size());
    for (const Vec3& vertex : result.vertices) {
        add(bits(vertex.x));
        add(bits(vertex.y));
        add(bits(vertex.z));
    }
    add(result.polygons.size());
    for (const ResultPolygon& polygon : result.polygons) {
        add(static_cast<std::uint64_t>(polygon.type));
        add(polygon.vertexIndices.size());
        for (const std::size_t vertex : polygon.vertexIndices) {
            add(vertex);
        }
    }
    add(static_cast<std::uint64_t>(result.boundary.status));
    add(result.boundary.orderedVertexIds.size());
    for (const std::size_t vertex : result.boundary.orderedVertexIds) {
        add(vertex);
    }
    return hash;
}

}  // namespace directional_retopo::solver
