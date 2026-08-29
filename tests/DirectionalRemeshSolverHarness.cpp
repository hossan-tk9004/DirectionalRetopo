#include "Solver/DirectionalRemeshSolver.h"

#include <algorithm>
#include <array>
#include <chrono>
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

}  // namespace

int main(int argc, char** argv)
{
    const bool probe = argc > 1 && std::string(argv[1]) == "--probe";
    dr::DirectionalRemeshSolver solver;
    bool allPassed = true;
    std::cout << "DirectionalRemeshSolver deterministic baseline\n";
    std::cout << "Fixture | Expected | Actual | Code | Q/T/N | Surface mean/p95/max | "
                 "Direction mean/max | Density requested/actual | Boundary | ms | hash\n";

    for (Fixture fixture : fixtures()) {
        std::vector<RunSummary> runs;
        for (unsigned int repeat = 0U; repeat < 5U; ++repeat) {
            const auto start = std::chrono::steady_clock::now();
            const solver::RemeshResult result = solver.solve(fixture.input);
            RunSummary summary;
            summary.success = result.success();
            summary.failureCode = result.failureCode;
            summary.hash = canonicalHash(result);
            summary.milliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (!result.components.empty()) {
                summary.quality = result.components.front().quality;
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
        std::cout << fixture.name << " | " << expectationName(fixture.expectation)
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
