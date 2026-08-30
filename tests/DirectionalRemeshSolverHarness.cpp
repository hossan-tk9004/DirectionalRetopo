#include "Solver/DirectionalRemeshSolver.h"
#include "Solver/RemeshCapture.h"
#include <filesystem>
#include <fstream>
#include <iterator>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace dr = directional_retopo;
namespace solver = directional_retopo::solver;

namespace {

constexpr double kPi = 3.14159265358979323846;

enum class Expectation
{
    ExpectedSuccess,
    ExpectedKnownFailure,
};

struct Fixture final
{
    std::string name;
    solver::RemeshInput input;
    Expectation expectation = Expectation::ExpectedKnownFailure;
};

struct RunSummary final
{
    bool success = false;
    solver::FailureCode failureCode = solver::FailureCode::UnknownFailure;
    std::uint64_t hash = 0U;
    solver::QualityMetrics quality;
    double milliseconds = 0.0;
    std::string diagnostic;
};

struct InputStats final
{
    std::size_t sourceVertexCount = 0U;
    std::size_t sourceFaceCount = 0U;
    std::size_t sourceTriangleCount = 0U;
    std::size_t coreFaceCount = 0U;
    std::size_t transitionFaceCount = 0U;
    std::size_t allFaceCount = 0U;
    std::size_t fixedBoundaryLoopCount = 0U;
    std::size_t fixedBoundaryVertexCount = 0U;
    int maximumTransitionDepth = -1;
    std::size_t validDirectionCount = 0U;
    double meanPaintWeight = 0.0;
    double meanTopologyWeight = 0.0;
    std::size_t validDensityCount = 0U;
    std::size_t curvatureConstrainedCount = 0U;
    double minimumRequestedEdgeLength = 0.0;
    double meanRequestedEdgeLength = 0.0;
    double maximumRequestedEdgeLength = 0.0;
    double minimumEffectiveEdgeLength = 0.0;
    double meanEffectiveEdgeLength = 0.0;
    double maximumEffectiveEdgeLength = 0.0;
};

solver::Vec3 normalized(const solver::Vec3& value)
{
    return value.normalized();
}

solver::SourceMeshSnapshot buildMesh(
    const std::vector<solver::Vec3>& positions,
    const std::vector<std::vector<std::size_t>>& polygons)
{
    solver::SourceMeshSnapshot mesh;
    mesh.vertices.resize(positions.size());
    for (std::size_t vertexIndex = 0U; vertexIndex < positions.size(); ++vertexIndex) {
        mesh.vertices[vertexIndex].position = positions[vertexIndex];
        mesh.vertices[vertexIndex].sourceVertexId = static_cast<solver::SourceId>(vertexIndex);
    }
    mesh.faces.resize(polygons.size());
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> edgeMap;

    for (std::size_t faceIndex = 0U; faceIndex < polygons.size(); ++faceIndex) {
        const std::vector<std::size_t>& polygon = polygons[faceIndex];
        solver::SourceFace& face = mesh.faces[faceIndex];
        face.sourceFaceId = static_cast<solver::SourceId>(faceIndex);
        face.vertexIndices = polygon;
        solver::Vec3 center;
        for (const std::size_t vertexIndex : polygon) {
            center += positions[vertexIndex];
            mesh.vertices[vertexIndex].faceIndices.push_back(faceIndex);
        }
        face.center = center / static_cast<double>(polygon.size());
        face.normal = normalized(
            (positions[polygon[1]] - positions[polygon[0]])
                .cross(positions[polygon[2]] - positions[polygon[0]]));
        face.geometryValid = face.normal.squaredLength() > 0.0;

        for (std::size_t corner = 0U; corner < polygon.size(); ++corner) {
            const std::size_t first = polygon[corner];
            const std::size_t second = polygon[(corner + 1U) % polygon.size()];
            const auto key = std::minmax(first, second);
            const std::pair<std::size_t, std::size_t> edgeKey(key.first, key.second);
            auto found = edgeMap.find(edgeKey);
            if (found == edgeMap.end()) {
                const std::size_t edgeIndex = mesh.edges.size();
                found = edgeMap.emplace(edgeKey, edgeIndex).first;
                solver::SourceEdge edge;
                edge.vertexIndices = {edgeKey.first, edgeKey.second};
                edge.sourceEdgeId = static_cast<solver::SourceId>(edgeIndex);
                edge.length = (positions[edgeKey.second] - positions[edgeKey.first]).length();
                mesh.edges.push_back(edge);
                mesh.vertices[edgeKey.first].edgeIndices.push_back(edgeIndex);
                mesh.vertices[edgeKey.second].edgeIndices.push_back(edgeIndex);
                mesh.vertices[edgeKey.first].adjacentVertexIndices.push_back(edgeKey.second);
                mesh.vertices[edgeKey.second].adjacentVertexIndices.push_back(edgeKey.first);
            }
            face.edgeIndices.push_back(found->second);
            mesh.edges[found->second].faceIndices.push_back(faceIndex);
        }

        const auto appendTriangle = [&](std::size_t a, std::size_t b, std::size_t c) {
            const std::size_t triangleIndex = mesh.triangles.size();
            mesh.triangles.push_back({{a, b, c}, faceIndex});
            face.triangleIndices.push_back(triangleIndex);
        };
        if (polygon.size() == 3U) {
            appendTriangle(polygon[0], polygon[1], polygon[2]);
        } else {
            for (std::size_t corner = 1U; corner + 1U < polygon.size(); ++corner) {
                appendTriangle(polygon[0], polygon[corner], polygon[corner + 1U]);
            }
        }
    }

    for (solver::SourceEdge& edge : mesh.edges) {
        edge.originalMeshBoundary = edge.faceIndices.size() == 1U;
        if (edge.faceIndices.size() == 2U) {
            mesh.faces[edge.faceIndices[0]].adjacentFaceIndices.push_back(edge.faceIndices[1]);
            mesh.faces[edge.faceIndices[1]].adjacentFaceIndices.push_back(edge.faceIndices[0]);
        }
    }
    for (std::size_t vertexIndex = 0U; vertexIndex < mesh.vertices.size(); ++vertexIndex) {
        solver::Vec3 normal;
        for (const std::size_t faceIndex : mesh.vertices[vertexIndex].faceIndices) {
            normal += mesh.faces[faceIndex].normal;
        }
        mesh.vertices[vertexIndex].normal = normalized(normal);
    }
    return mesh;
}

std::size_t findEdge(
    const solver::SourceMeshSnapshot& mesh,
    std::size_t first,
    std::size_t second)
{
    for (std::size_t edgeIndex : mesh.vertices[first].edgeIndices) {
        const solver::SourceEdge& edge = mesh.edges[edgeIndex];
        if ((edge.vertexIndices[0] == first && edge.vertexIndices[1] == second) ||
            (edge.vertexIndices[0] == second && edge.vertexIndices[1] == first)) {
            return edgeIndex;
        }
    }
    return solver::kInvalidIndex;
}

void finalizeInput(
    solver::RemeshInput& input,
    const std::vector<std::size_t>& coreFaces,
    const std::vector<std::size_t>& boundaryVertices,
    double targetEdgeLength,
    unsigned int blendWidth)
{
    solver::RegionComponent component;
    component.componentId = 0U;
    component.coreFaceIndices = coreFaces;
    component.transitionRingDepthByFace.assign(input.sourceMesh.faces.size(), -1);
    std::set<std::size_t> coreSet(coreFaces.begin(), coreFaces.end());
    for (std::size_t faceIndex = 0U; faceIndex < input.sourceMesh.faces.size(); ++faceIndex) {
        component.allFaceIndices.push_back(faceIndex);
        if (coreSet.count(faceIndex) == 0U) {
            component.transitionFaceIndices.push_back(faceIndex);
        }
    }
    std::queue<std::size_t> frontier;
    for (std::size_t faceIndex : coreFaces) {
        component.transitionRingDepthByFace[faceIndex] = 0;
        frontier.push(faceIndex);
    }
    while (!frontier.empty()) {
        const std::size_t faceIndex = frontier.front();
        frontier.pop();
        const int depth = component.transitionRingDepthByFace[faceIndex] + 1;
        for (std::size_t adjacent : input.sourceMesh.faces[faceIndex].adjacentFaceIndices) {
            if (component.transitionRingDepthByFace[adjacent] < 0) {
                component.transitionRingDepthByFace[adjacent] = depth;
                frontier.push(adjacent);
            }
        }
    }
    solver::OrderedBoundaryLoop boundary;
    boundary.closed = true;
    boundary.vertexIndices = boundaryVertices;
    for (std::size_t index = 0U; index < boundaryVertices.size(); ++index) {
        const std::size_t first = boundaryVertices[index];
        const std::size_t second = boundaryVertices[(index + 1U) % boundaryVertices.size()];
        const std::size_t edgeIndex = findEdge(input.sourceMesh, first, second);
        boundary.edgeIndices.push_back(edgeIndex);
        boundary.sourceVertexIds.push_back(input.sourceMesh.vertices[first].sourceVertexId);
        boundary.sourceEdgeIds.push_back(input.sourceMesh.edges[edgeIndex].sourceEdgeId);
        boundary.touchesOriginalMeshBoundary = boundary.touchesOriginalMeshBoundary ||
            input.sourceMesh.edges[edgeIndex].originalMeshBoundary;
    }
    component.fixedBoundaryLoops.push_back(std::move(boundary));
    input.components.push_back(std::move(component));

    input.directionField.resize(input.sourceMesh.faces.size());
    input.densityField.resize(input.sourceMesh.faces.size());
    for (std::size_t faceIndex = 0U; faceIndex < input.sourceMesh.faces.size(); ++faceIndex) {
        const solver::Vec3 normal = input.sourceMesh.faces[faceIndex].normal;
        solver::Vec3 u = solver::Vec3{1.0, 0.0, 0.0} -
            normal * normal.dot(solver::Vec3{1.0, 0.0, 0.0});
        if (u.squaredLength() < 1.0e-12) {
            u = solver::Vec3{0.0, 1.0, 0.0} -
                normal * normal.dot(solver::Vec3{0.0, 1.0, 0.0});
        }
        u = normalized(u);
        input.directionField[faceIndex] = {
            normal, u, normalized(normal.cross(u)),
            coreSet.count(faceIndex) != 0U ? 1.0 : 0.25,
            coreSet.count(faceIndex) != 0U ? 0.0 : 0.5,
            true};
        input.densityField[faceIndex] = {
            targetEdgeLength,
            targetEdgeLength,
            1.0,
            1.0,
            false,
            true};
    }
    input.settings.topologyBlendWidth = std::max(blendWidth, 1U);
    input.settings.maximumRetryAttempts = 3U;
}

solver::RemeshInput makeGrid(
    std::size_t columns,
    std::size_t rows,
    std::size_t coreInset,
    double targetEdgeLength,
    unsigned int blendWidth,
    const std::string& surface)
{
    solver::RemeshInput input;
    std::vector<solver::Vec3> positions;
    positions.reserve((columns + 1U) * (rows + 1U));
    for (std::size_t row = 0U; row <= rows; ++row) {
        for (std::size_t column = 0U; column <= columns; ++column) {
            const double x = static_cast<double>(column) - static_cast<double>(columns) * 0.5;
            const double y = static_cast<double>(row) - static_cast<double>(rows) * 0.5;
            double z = 0.0;
            if (surface == "curved") {
                z = 0.045 * (x * x + 0.5 * y * y);
            } else if (surface == "chest") {
                z = 2.5 * std::exp(-(x * x + 1.4 * y * y) / 18.0);
            } else if (surface == "cloth") {
                z = 0.55 * std::sin(0.75 * x) * std::cos(0.6 * y);
            }
            positions.push_back({x, y, z});
        }
    }
    std::vector<std::vector<std::size_t>> polygons;
    for (std::size_t row = 0U; row < rows; ++row) {
        for (std::size_t column = 0U; column < columns; ++column) {
            const std::size_t a = row * (columns + 1U) + column;
            const std::size_t b = a + 1U;
            const std::size_t d = (row + 1U) * (columns + 1U) + column;
            const std::size_t c = d + 1U;
            polygons.push_back({a, b, c, d});
        }
    }
    input.sourceMesh = buildMesh(positions, polygons);
    std::vector<std::size_t> coreFaces;
    for (std::size_t row = coreInset; row + coreInset < rows; ++row) {
        for (std::size_t column = coreInset; column + coreInset < columns; ++column) {
            coreFaces.push_back(row * columns + column);
        }
    }
    if (coreFaces.empty()) {
        coreFaces.push_back((rows / 2U) * columns + columns / 2U);
    }
    std::vector<std::size_t> boundary;
    for (std::size_t column = 0U; column <= columns; ++column) {
        boundary.push_back(column);
    }
    for (std::size_t row = 1U; row <= rows; ++row) {
        boundary.push_back(row * (columns + 1U) + columns);
    }
    for (std::size_t column = columns; column-- > 0U;) {
        boundary.push_back(rows * (columns + 1U) + column);
    }
    for (std::size_t row = rows; row-- > 1U;) {
        boundary.push_back(row * (columns + 1U));
    }
    finalizeInput(input, coreFaces, boundary, targetEdgeLength, blendWidth);
    return input;
}

solver::RemeshInput makeOddBoundary()
{
    constexpr std::size_t segments = 9U;
    solver::RemeshInput input;
    std::vector<solver::Vec3> positions;
    positions.push_back({0.0, 0.0, 0.2});
    for (std::size_t ring = 0U; ring < 2U; ++ring) {
        const double radius = ring == 0U ? 2.0 : 4.0;
        for (std::size_t index = 0U; index < segments; ++index) {
            const double angle = 2.0 * kPi * static_cast<double>(index) /
                static_cast<double>(segments);
            positions.push_back({radius * std::cos(angle), radius * std::sin(angle),
                                 0.08 * radius * radius});
        }
    }
    std::vector<std::vector<std::size_t>> polygons;
    for (std::size_t index = 0U; index < segments; ++index) {
        const std::size_t next = (index + 1U) % segments;
        polygons.push_back({0U, 1U + index, 1U + next});
    }
    for (std::size_t index = 0U; index < segments; ++index) {
        const std::size_t next = (index + 1U) % segments;
        polygons.push_back({1U + index, 1U + segments + index,
                            1U + segments + next, 1U + next});
    }
    input.sourceMesh = buildMesh(positions, polygons);
    std::vector<std::size_t> coreFaces;
    for (std::size_t index = 0U; index < segments; ++index) {
        coreFaces.push_back(index);
    }
    std::vector<std::size_t> boundary;
    for (std::size_t index = 0U; index < segments; ++index) {
        boundary.push_back(1U + segments + index);
    }
    finalizeInput(input, coreFaces, boundary, 1.0, 2U);
    return input;
}

std::uint64_t fnv(std::uint64_t hash, std::uint64_t value)
{
    constexpr std::uint64_t prime = 1099511628211ULL;
    for (unsigned int byte = 0U; byte < 8U; ++byte) {
        hash ^= (value >> (byte * 8U)) & 0xffU;
        hash *= prime;
    }
    return hash;
}

std::vector<std::size_t> canonicalPolygon(const std::vector<std::size_t>& polygon)
{
    std::vector<std::size_t> best;
    for (bool reverse : {false, true}) {
        for (std::size_t offset = 0U; offset < polygon.size(); ++offset) {
            std::vector<std::size_t> candidate;
            for (std::size_t index = 0U; index < polygon.size(); ++index) {
                const std::size_t position = reverse
                    ? (offset + polygon.size() - index) % polygon.size()
                    : (offset + index) % polygon.size();
                candidate.push_back(polygon[position]);
            }
            if (best.empty() || candidate < best) {
                best = std::move(candidate);
            }
        }
    }
    return best;
}

std::uint64_t canonicalHash(const solver::RemeshResult& result)
{
    std::uint64_t hash = 1469598103934665603ULL;
    hash = fnv(hash, static_cast<std::uint64_t>(result.status));
    hash = fnv(hash, static_cast<std::uint64_t>(result.failureCode));
    for (const solver::ComponentResult& component : result.components) {
        hash = fnv(hash, component.componentId);
        hash = fnv(hash, static_cast<std::uint64_t>(component.status));
        hash = fnv(hash, static_cast<std::uint64_t>(component.failureCode));
        for (const solver::Vec3& vertex : component.vertices) {
            for (const double value : {vertex.x, vertex.y, vertex.z}) {
                hash = fnv(hash, static_cast<std::uint64_t>(
                    static_cast<std::int64_t>(std::llround(value * 1.0e6))));
            }
        }
        std::vector<std::vector<std::size_t>> polygons;
        for (const solver::ResultPolygon& polygon : component.polygons) {
            polygons.push_back(canonicalPolygon(polygon.vertexIndices));
        }
        std::sort(polygons.begin(), polygons.end());
        for (const std::vector<std::size_t>& polygon : polygons) {
            hash = fnv(hash, polygon.size());
            for (const std::size_t index : polygon) {
                hash = fnv(hash, index);
            }
        }
    }
    return hash;
}

solver::QualityMetrics aggregateQuality(const solver::RemeshResult& result)
{
    solver::QualityMetrics aggregate;
    double totalWeight = 0.0;
    for (const solver::ComponentResult& component : result.components) {
        const solver::QualityMetrics& quality = component.quality;
        const double weight = static_cast<double>(
            quality.quadCount + quality.triangleCount + quality.nGonCount);
        aggregate.quadCount += quality.quadCount;
        aggregate.triangleCount += quality.triangleCount;
        aggregate.nGonCount += quality.nGonCount;
        aggregate.boundaryCrossingCount += quality.boundaryCrossingCount;
        aggregate.nonManifoldEdgeCount += quality.nonManifoldEdgeCount;
        aggregate.zeroAreaPolygonCount += quality.zeroAreaPolygonCount;
        aggregate.maximumBoundaryDisplacement = std::max(
            aggregate.maximumBoundaryDisplacement,
            quality.maximumBoundaryDisplacement);
        aggregate.p95SurfaceDistance = std::max(
            aggregate.p95SurfaceDistance,
            quality.p95SurfaceDistance);
        aggregate.maximumSurfaceDistance = std::max(
            aggregate.maximumSurfaceDistance,
            quality.maximumSurfaceDistance);
        aggregate.maximumCoreDirectionDeviationDegrees = std::max(
            aggregate.maximumCoreDirectionDeviationDegrees,
            quality.maximumCoreDirectionDeviationDegrees);
        if (weight > 0.0) {
            aggregate.meanSurfaceDistance += quality.meanSurfaceDistance * weight;
            aggregate.meanCoreDirectionDeviationDegrees +=
                quality.meanCoreDirectionDeviationDegrees * weight;
            aggregate.requestedCoreEdgeLength +=
                quality.requestedCoreEdgeLength * weight;
            aggregate.actualCoreEdgeLength += quality.actualCoreEdgeLength * weight;
            totalWeight += weight;
        }
    }
    if (totalWeight > 0.0) {
        aggregate.meanSurfaceDistance /= totalWeight;
        aggregate.meanCoreDirectionDeviationDegrees /= totalWeight;
        aggregate.requestedCoreEdgeLength /= totalWeight;
        aggregate.actualCoreEdgeLength /= totalWeight;
    }
    return aggregate;
}

bool validateSuccess(const solver::RemeshResult& result, std::string& failure)
{
    for (const solver::ComponentResult& component : result.components) {
        if (component.status != solver::SolveStatus::Success) {
            continue;
        }
        for (const solver::Vec3& vertex : component.vertices) {
            if (!vertex.finite()) {
                failure = "non-finite output vertex";
                return false;
            }
        }
        if (component.quality.maximumBoundaryDisplacement > 1.0e-9) {
            failure = "fixed boundary displacement is non-zero";
            return false;
        }
        if (component.quality.boundaryCrossingCount != 0U) {
            failure = "boundary crossing count is non-zero";
            return false;
        }
        if (component.quality.nonManifoldEdgeCount != 0U ||
            component.quality.zeroAreaPolygonCount != 0U ||
            component.quality.nGonCount != 0U) {
            failure = "topology quality invariant failed";
            return false;
        }
    }
    return true;
}

std::vector<Fixture> fixtures()
{
    return {
        {"plane_fine", makeGrid(8U, 8U, 2U, 0.5, 2U, "plane"), Expectation::ExpectedKnownFailure},
        {"plane_coarse", makeGrid(8U, 8U, 2U, 2.0, 2U, "plane"), Expectation::ExpectedKnownFailure},
        {"curved_fine", makeGrid(10U, 8U, 2U, 0.5, 2U, "curved"), Expectation::ExpectedKnownFailure},
        {"curved_coarse", makeGrid(10U, 8U, 2U, 2.0, 2U, "curved"), Expectation::ExpectedKnownFailure},
        {"small_region", makeGrid(3U, 3U, 1U, 1.0, 1U, "plane"), Expectation::ExpectedKnownFailure},
        {"blend_width_1", makeGrid(8U, 8U, 2U, 1.0, 1U, "plane"), Expectation::ExpectedKnownFailure},
        {"odd_boundary", makeOddBoundary(), Expectation::ExpectedKnownFailure},
        {"dense_boundary_coarse_core", makeGrid(12U, 12U, 3U, 3.0, 2U, "plane"), Expectation::ExpectedKnownFailure},
        {"chest_like_bump", makeGrid(12U, 10U, 3U, 0.8, 3U, "chest"), Expectation::ExpectedKnownFailure},
        {"cloth_fold", makeGrid(12U, 10U, 3U, 0.7, 3U, "cloth"), Expectation::ExpectedKnownFailure},
    };
}

const char* expectationName(Expectation expectation)
{
    return expectation == Expectation::ExpectedSuccess
        ? "ExpectedSuccess"
        : "ExpectedKnownFailure";
}
bool readBytes(const std::filesystem::path& path, std::string& bytes)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return false;
    }
    bytes.assign(
        std::istreambuf_iterator<char>(stream),
        std::istreambuf_iterator<char>());
    return stream.good() || stream.eof();
}

bool roundTripFixture(
    const Fixture& fixture,
    solver::RemeshInput& replayInput,
    std::string& failure)
{
    const std::filesystem::path firstPath =
        std::filesystem::current_path() / (fixture.name + ".roundtrip-a.drinput");
    const std::filesystem::path secondPath =
        std::filesystem::current_path() / (fixture.name + ".roundtrip-b.drinput");
    const auto cleanup = [&]() {
        std::error_code ignored;
        std::filesystem::remove(firstPath, ignored);
        std::filesystem::remove(secondPath, ignored);
    };

    solver::RemeshCaptureRecord original;
    original.label = fixture.name;
    original.input = fixture.input;

    std::string diagnostic;
    if (!solver::saveRemeshCapture(firstPath.string(), original, diagnostic)) {
        failure = diagnostic;
        cleanup();
        return false;
    }
    solver::RemeshCaptureRecord loaded;
    if (!solver::loadRemeshCapture(firstPath.string(), loaded, diagnostic)) {
        failure = diagnostic;
        cleanup();
        return false;
    }
    if (solver::remeshInputSignature(fixture.input) !=
        solver::remeshInputSignature(loaded.input)) {
        failure = "RemeshInput signature changed across serialization.";
        cleanup();
        return false;
    }
    if (loaded.hasExpectedResult) {
        failure = "Input-only capture unexpectedly gained a solve expectation.";
        cleanup();
        return false;
    }

    dr::DirectionalRemeshSolver captureSolver;
    loaded.hasExpectedResult = true;
    loaded.expectedResult = solver::summarizeResult(captureSolver.solve(loaded.input));
    if (!solver::saveRemeshCapture(firstPath.string(), loaded, diagnostic)) {
        failure = diagnostic;
        cleanup();
        return false;
    }
    solver::RemeshCaptureRecord loadedWithExpectation;
    if (!solver::loadRemeshCapture(
            firstPath.string(), loadedWithExpectation, diagnostic) ||
        !loadedWithExpectation.hasExpectedResult) {
        failure = diagnostic.empty()
            ? "Captured solve expectation was lost during serialization."
            : diagnostic;
        cleanup();
        return false;
    }
    dr::DirectionalRemeshSolver replaySolver;
    const solver::RemeshResult replayResult =
        replaySolver.solve(loadedWithExpectation.input);
    if (!solver::replayMatchesCapture(
            loadedWithExpectation.expectedResult,
            replayResult,
            diagnostic)) {
        failure = "Captured result parity failed after round-trip: " + diagnostic;
        cleanup();
        return false;
    }
    if (!solver::saveRemeshCapture(
            secondPath.string(), loadedWithExpectation, diagnostic)) {
        failure = diagnostic;
        cleanup();
        return false;
    }
    std::string firstBytes;
    std::string secondBytes;
    if (!readBytes(firstPath, firstBytes) || !readBytes(secondPath, secondBytes) ||
        firstBytes != secondBytes) {
        failure = "RemeshInput capture bytes are not deterministic after round-trip.";
        cleanup();
        return false;
    }
    replayInput = std::move(loadedWithExpectation.input);
    cleanup();
    return true;
}

InputStats summarizeInput(const solver::RemeshInput& input)
{
    InputStats stats;
    stats.sourceVertexCount = input.sourceMesh.vertices.size();
    stats.sourceFaceCount = input.sourceMesh.faces.size();
    stats.sourceTriangleCount = input.sourceMesh.triangles.size();
    for (const solver::RegionComponent& component : input.components) {
        stats.coreFaceCount += component.coreFaceIndices.size();
        stats.transitionFaceCount += component.transitionFaceIndices.size();
        stats.allFaceCount += component.allFaceIndices.size();
        stats.fixedBoundaryLoopCount += component.fixedBoundaryLoops.size();
        for (const solver::OrderedBoundaryLoop& loop : component.fixedBoundaryLoops) {
            stats.fixedBoundaryVertexCount += loop.vertexIndices.size();
        }
        for (const int depth : component.transitionRingDepthByFace) {
            stats.maximumTransitionDepth = std::max(stats.maximumTransitionDepth, depth);
        }
    }
    for (const solver::FaceDirection& direction : input.directionField) {
        if (!direction.valid) {
            continue;
        }
        ++stats.validDirectionCount;
        stats.meanPaintWeight += direction.paintConstraintWeight;
        stats.meanTopologyWeight += direction.topologyGuidanceWeight;
    }
    if (stats.validDirectionCount > 0U) {
        stats.meanPaintWeight /= static_cast<double>(stats.validDirectionCount);
        stats.meanTopologyWeight /= static_cast<double>(stats.validDirectionCount);
    }
    stats.minimumRequestedEdgeLength = std::numeric_limits<double>::infinity();
    stats.minimumEffectiveEdgeLength = std::numeric_limits<double>::infinity();
    for (const solver::FaceDensity& density : input.densityField) {
        if (!density.valid) {
            continue;
        }
        ++stats.validDensityCount;
        stats.curvatureConstrainedCount += density.curvatureConstrained ? 1U : 0U;
        stats.minimumRequestedEdgeLength = std::min(
            stats.minimumRequestedEdgeLength, density.requestedTargetEdgeLength);
        stats.meanRequestedEdgeLength += density.requestedTargetEdgeLength;
        stats.maximumRequestedEdgeLength = std::max(
            stats.maximumRequestedEdgeLength, density.requestedTargetEdgeLength);
        stats.minimumEffectiveEdgeLength = std::min(
            stats.minimumEffectiveEdgeLength, density.effectiveTargetEdgeLength);
        stats.meanEffectiveEdgeLength += density.effectiveTargetEdgeLength;
        stats.maximumEffectiveEdgeLength = std::max(
            stats.maximumEffectiveEdgeLength, density.effectiveTargetEdgeLength);
    }
    if (stats.validDensityCount > 0U) {
        stats.meanRequestedEdgeLength /= static_cast<double>(stats.validDensityCount);
        stats.meanEffectiveEdgeLength /= static_cast<double>(stats.validDensityCount);
    } else {
        stats.minimumRequestedEdgeLength = 0.0;
        stats.minimumEffectiveEdgeLength = 0.0;
    }
    return stats;
}

std::string captureDensityModeName(const std::filesystem::path& path)
{
    std::string name = path.filename().string();
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char value) {
        return static_cast<char>(std::tolower(value));
    });
    if (name.find("manualdensity") != std::string::npos) {
        return "Manual";
    }
    if (name.find("autodensity") != std::string::npos) {
        return "Auto";
    }
    return "Unknown";
}

bool inspectCaptureFile(const std::filesystem::path& path)
{
    solver::RemeshCaptureRecord capture;
    std::string diagnostic;
    if (!solver::loadRemeshCapture(path.string(), capture, diagnostic)) {
        std::cerr << "CAPTURE_ERROR|" << path.filename().string() << '|'
                  << diagnostic << '\n';
        return false;
    }
    const InputStats stats = summarizeInput(capture.input);
    std::size_t expectedVertices = 0U;
    std::size_t expectedPolygons = 0U;
    std::size_t expectedQuads = 0U;
    std::size_t expectedTriangles = 0U;
    std::size_t expectedNGons = 0U;
    for (const solver::CapturedComponentSummary& component :
         capture.expectedResult.components) {
        expectedVertices += component.vertexCount;
        expectedPolygons += component.polygonCount;
        expectedQuads += component.quality.quadCount;
        expectedTriangles += component.quality.triangleCount;
        expectedNGons += component.quality.nGonCount;
    }
    std::cout << "CAPTURE|" << path.filename().string()
              << "|mode=" << captureDensityModeName(path)
              << "|metadata=" << (capture.hasExpectedResult
                  ? (capture.expectedResult.status == solver::SolveStatus::Success
                      ? "ExpectedSuccess" : "ExpectedKnownFailure")
                  : "InputOnly")
              << "|failure=" << (capture.hasExpectedResult
                  ? solver::failureCodeName(capture.expectedResult.failureCode)
                  : "None")
              << "|sourceVertices=" << stats.sourceVertexCount
              << "|sourceFaces=" << stats.sourceFaceCount
              << "|coreFaces=" << stats.coreFaceCount
              << "|transitionFaces=" << stats.transitionFaceCount
              << "|fixedLoops=" << stats.fixedBoundaryLoopCount
              << "|fixedVertices=" << stats.fixedBoundaryVertexCount
              << "|requested=" << stats.minimumRequestedEdgeLength << '/'
              << stats.meanRequestedEdgeLength << '/'
              << stats.maximumRequestedEdgeLength
              << "|effective=" << stats.minimumEffectiveEdgeLength << '/'
              << stats.meanEffectiveEdgeLength << '/'
              << stats.maximumEffectiveEdgeLength
              << "|blend=" << capture.input.settings.topologyBlendWidth
              << "|expectedV/P/Q/T/N=" << expectedVertices << '/'
              << expectedPolygons << '/' << expectedQuads << '/'
              << expectedTriangles << '/' << expectedNGons
              << "|signature=0x" << std::hex
              << capture.expectedResult.topologySignature << std::dec << '\n';
    return true;
}

void printInputStats(const solver::RemeshInput& input)
{
    const InputStats stats = summarizeInput(input);
    std::cout << std::defaultfloat << std::setprecision(6)
              << "  input: faces/triangles=" << stats.sourceFaceCount << '/'
              << stats.sourceTriangleCount
              << ", core/transition/all=" << stats.coreFaceCount << '/'
              << stats.transitionFaceCount << '/' << stats.allFaceCount
              << ", ringDepthMax=" << stats.maximumTransitionDepth
              << ", fixedLoops/vertices=" << stats.fixedBoundaryLoopCount << '/'
              << stats.fixedBoundaryVertexCount
              << ", directionValid paint/topologyMean=" << stats.validDirectionCount
              << ' ' << stats.meanPaintWeight << '/' << stats.meanTopologyWeight
              << ", densityValid curvature requested/effectiveMean="
              << stats.validDensityCount << ' ' << stats.curvatureConstrainedCount
              << ' ' << stats.meanRequestedEdgeLength << '/'
              << stats.meanEffectiveEdgeLength << '\n';
}

bool replayCaptureFile(
    const std::filesystem::path& path,
    double targetEdgeLengthOverride,
    unsigned int repeatCount)
{
    solver::RemeshCaptureRecord capture;
    std::string diagnostic;
    if (!solver::loadRemeshCapture(path.string(), capture, diagnostic)) {
        std::cerr << path.string() << ": " << diagnostic << '\n';
        return false;
    }
    if (targetEdgeLengthOverride > 0.0) {
        for (solver::FaceDensity& density : capture.input.densityField) {
            if (!density.valid) {
                continue;
            }
            density.requestedTargetEdgeLength = targetEdgeLengthOverride;
            density.effectiveTargetEdgeLength = targetEdgeLengthOverride;
            density.scaleU = 1.0;
            density.scaleV = 1.0;
            density.curvatureConstrained = false;
        }
    }
    printInputStats(capture.input);
    dr::DirectionalRemeshSolver remesher;
    std::vector<RunSummary> runs;
    bool passed = true;
    for (unsigned int repeat = 0U; repeat < repeatCount; ++repeat) {
        const auto start = std::chrono::steady_clock::now();
        const solver::RemeshResult result = remesher.solve(capture.input);
        RunSummary run;
        run.success = result.success();
        run.failureCode = result.failureCode;
        run.hash = canonicalHash(result);
        run.milliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        if (!result.components.empty()) {
            run.quality = aggregateQuality(result);
            run.diagnostic = result.components.front().diagnosticMessage;
        }
        if (capture.hasExpectedResult && !(targetEdgeLengthOverride > 0.0) &&
            !solver::replayMatchesCapture(capture.expectedResult, result, diagnostic)) {
            std::cerr << path.string() << " replay " << (repeat + 1U)
                      << " parity failure: " << diagnostic << '\n';
            passed = false;
        }
        if (result.success()) {
            std::string validationFailure;
            if (!validateSuccess(result, validationFailure)) {
                std::cerr << path.string() << " replay " << (repeat + 1U)
                          << " invariant failure: " << validationFailure << '\n';
                passed = false;
            }
        }
        runs.push_back(std::move(run));
    }
    const RunSummary& first = runs.front();
    const bool deterministic = std::all_of(
        runs.begin(), runs.end(), [&first](const RunSummary& candidate) {
            return candidate.success == first.success &&
                candidate.failureCode == first.failureCode &&
                candidate.hash == first.hash;
        });
    passed = passed && deterministic;

    const solver::QualityMetrics& q = first.quality;
    std::cout << std::defaultfloat << std::setprecision(6)
              << path.filename().string() << " | MayaCapture | "
              << (capture.hasExpectedResult
                  ? (capture.expectedResult.status == solver::SolveStatus::Success
                      ? "ExpectedSuccess" : "ExpectedFailure")
                  : "InputOnly")
              << " | " << (first.success ? "Success" : "Failure")
              << " | " << solver::failureCodeName(first.failureCode)
              << " | " << q.quadCount << '/' << q.triangleCount << '/' << q.nGonCount
              << " | " << q.maximumBoundaryDisplacement << '/'
              << q.boundaryCrossingCount
              << " | " << q.meanCoreDirectionDeviationDegrees << '/'
              << q.maximumCoreDirectionDeviationDegrees
              << " | " << q.requestedCoreEdgeLength << '/' << q.actualCoreEdgeLength
              << " | " << q.meanSurfaceDistance << '/' << q.p95SurfaceDistance
              << '/' << q.maximumSurfaceDistance
              << " | 0x" << std::hex << first.hash << std::dec
              << " | " << std::fixed << std::setprecision(2)
              << first.milliseconds << " ms"
              << (deterministic ? " | deterministic" : " | NON-DETERMINISTIC")
              << '\n';
    return passed;
}


}  // namespace

int main(int argc, char** argv)
{
    bool probe = false;
    bool inspectOnly = false;
    double targetEdgeLengthOverride = 0.0;
    unsigned int replayRepeatCount = 5U;
    std::vector<std::filesystem::path> replayFiles;
    for (int index = 1; index < argc; ++index) {
        const std::string argument(argv[index]);
        if (argument == "--probe") {
            probe = true;
        } else if (argument == "--inspect-only") {
            inspectOnly = true;
        } else if (argument == "--replay" && index + 1 < argc) {
            replayFiles.emplace_back(argv[++index]);
        } else if (argument == "--captured-dir" && index + 1 < argc) {
            const std::filesystem::path directory(argv[++index]);
            std::error_code error;
            if (std::filesystem::exists(directory, error)) {
                for (const std::filesystem::directory_entry& entry :
                     std::filesystem::directory_iterator(directory)) {
                    if (entry.is_regular_file() && entry.path().extension() == ".drinput") {
                        replayFiles.push_back(entry.path());
                    }
                }
                std::sort(replayFiles.begin(), replayFiles.end());
            }
        } else if (argument == "--target-edge-length" && index + 1 < argc) {
            targetEdgeLengthOverride = std::stod(argv[++index]);
            if (!std::isfinite(targetEdgeLengthOverride) || targetEdgeLengthOverride <= 0.0) {
                std::cerr << "--target-edge-length must be finite and greater than zero.\n";
                return 2;
            }
        } else if (argument == "--repeat" && index + 1 < argc) {
            replayRepeatCount = static_cast<unsigned int>(std::stoul(argv[++index]));
            if (replayRepeatCount == 0U) {
                std::cerr << "--repeat must be greater than zero.\n";
                return 2;
            }
        } else {
            std::cerr << "Usage: DirectionalRemeshSolverHarness [--probe] "
                         "[--inspect-only] "
                         "[--replay file.drinput] [--captured-dir directory] "
                         "[--target-edge-length value] [--repeat count]\n";
            return 2;
        }
    }
    if (!replayFiles.empty()) {
        bool replayPassed = true;
        if (inspectOnly) {
            std::cout << "Captured RemeshInput audit\n";
            for (const std::filesystem::path& path : replayFiles) {
                replayPassed = inspectCaptureFile(path) && replayPassed;
            }
            std::cout << (replayPassed ? "PASS" : "FAIL") << '\n';
            return replayPassed ? 0 : 1;
        }
        std::cout << "Captured RemeshInput replay baseline\n";
        for (const std::filesystem::path& path : replayFiles) {
            replayPassed = replayCaptureFile(
                path, targetEdgeLengthOverride, replayRepeatCount) && replayPassed;
        }
        std::cout << (replayPassed ? "PASS" : "FAIL") << '\n';
        return replayPassed ? 0 : 1;
    }

    dr::DirectionalRemeshSolver solver;
    bool allPassed = true;
    std::cout << "DirectionalRemeshSolver deterministic baseline\n";
    std::cout << "Fixture | Expected | Actual | Code | Q/T/N | Surface mean/p95/max | "
                 "Direction mean/max | Density requested/actual | Boundary | ms | hash\n";

    for (Fixture fixture : fixtures()) {
        solver::RemeshInput replayInput;
        std::string roundTripFailure;
        if (!roundTripFixture(fixture, replayInput, roundTripFailure)) {
            std::cerr << fixture.name << " serialization failure: "
                      << roundTripFailure << '\n';
            allPassed = false;
            continue;
        }
        std::vector<RunSummary> runs;
        for (unsigned int repeat = 0U; repeat < 5U; ++repeat) {
            const auto start = std::chrono::steady_clock::now();
            const solver::RemeshResult result = solver.solve(replayInput);
            RunSummary summary;
            summary.success = result.success();
            summary.failureCode = result.failureCode;
            summary.hash = canonicalHash(result);
            summary.milliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (!result.components.empty()) {
                summary.quality = aggregateQuality(result);
                summary.diagnostic = result.components.front().diagnosticMessage;
            } else if (!result.warnings.empty()) {
                summary.diagnostic = result.warnings.front();
            }
            if (summary.success) {
                std::string validationFailure;
                if (!validateSuccess(result, validationFailure)) {
                    std::cerr << fixture.name << " invariant failure: "
                              << validationFailure << '\n';
                    allPassed = false;
                }
            }
            runs.push_back(std::move(summary));
        }
        const RunSummary& first = runs.front();
        const bool repeatable = std::all_of(
            runs.begin(), runs.end(), [&first](const RunSummary& candidate) {
                return candidate.success == first.success &&
                    candidate.failureCode == first.failureCode &&
                    candidate.hash == first.hash;
            });
        const bool expectationMatches = fixture.expectation == Expectation::ExpectedSuccess
            ? first.success
            : !first.success;
        if (!probe && (!expectationMatches || !repeatable)) {
            allPassed = false;
        }
        const solver::QualityMetrics& q = first.quality;
        std::cout << std::defaultfloat << std::setprecision(6)
                  << fixture.name << " | " << expectationName(fixture.expectation)
                  << " | " << (first.success ? "Success" : "KnownFailure")
                  << " | " << solver::failureCodeName(first.failureCode)
                  << " | " << q.quadCount << '/' << q.triangleCount << '/' << q.nGonCount
                  << " | " << q.meanSurfaceDistance << '/' << q.p95SurfaceDistance
                  << '/' << q.maximumSurfaceDistance
                  << " | " << q.meanCoreDirectionDeviationDegrees << '/'
                  << q.maximumCoreDirectionDeviationDegrees
                  << " | " << q.requestedCoreEdgeLength << '/' << q.actualCoreEdgeLength
                  << " | " << q.maximumBoundaryDisplacement << '/'
                  << q.boundaryCrossingCount
                  << " | " << std::fixed << std::setprecision(2)
                  << first.milliseconds << " | 0x" << std::hex << first.hash
                  << std::dec << (repeatable ? "" : " NON-DETERMINISTIC") << '\n';
        printInputStats(replayInput);
        if (!first.success) {
            std::cout << "  diagnostic: " << first.diagnostic << '\n';
        }
    }
    if (probe) {
        std::cout << "Probe mode: expectations were not enforced.\n";
        return allPassed ? 0 : 1;
    }
    std::cout << (allPassed ? "PASS" : "FAIL") << '\n';
    return allPassed ? 0 : 1;
}
