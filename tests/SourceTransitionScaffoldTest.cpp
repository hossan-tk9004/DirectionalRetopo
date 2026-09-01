#include "Solver/RemeshCapture.h"
#include "Solver/SourceTransitionScaffold.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace solver = directional_retopo::solver;
namespace {

constexpr double kPi = 3.14159265358979323846;

solver::SourceMeshSnapshot buildMesh(
    const std::vector<solver::Vec3>& positions,
    const std::vector<std::vector<std::size_t>>& polygons)
{
    solver::SourceMeshSnapshot mesh;
    mesh.vertices.resize(positions.size());
    for (std::size_t vertexIndex = 0U; vertexIndex < positions.size(); ++vertexIndex) {
        mesh.vertices[vertexIndex].position = positions[vertexIndex];
        mesh.vertices[vertexIndex].sourceVertexId =
            static_cast<solver::SourceId>(vertexIndex);
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
        face.normal = ((positions[polygon[1]] - positions[polygon[0]])
                           .cross(positions[polygon[2]] - positions[polygon[0]]))
                          .normalized();
        face.geometryValid = face.normal.squaredLength() > 0.0;
        for (std::size_t corner = 0U; corner < polygon.size(); ++corner) {
            const std::size_t first = polygon[corner];
            const std::size_t second = polygon[(corner + 1U) % polygon.size()];
            const auto pair = std::minmax(first, second);
            const std::pair<std::size_t, std::size_t> key(pair.first, pair.second);
            auto found = edgeMap.find(key);
            if (found == edgeMap.end()) {
                const std::size_t edgeIndex = mesh.edges.size();
                found = edgeMap.emplace(key, edgeIndex).first;
                solver::SourceEdge edge;
                edge.vertexIndices = {key.first, key.second};
                edge.sourceEdgeId = static_cast<solver::SourceId>(edgeIndex);
                edge.length = (positions[key.second] - positions[key.first]).length();
                mesh.edges.push_back(edge);
                mesh.vertices[key.first].edgeIndices.push_back(edgeIndex);
                mesh.vertices[key.second].edgeIndices.push_back(edgeIndex);
                mesh.vertices[key.first].adjacentVertexIndices.push_back(key.second);
                mesh.vertices[key.second].adjacentVertexIndices.push_back(key.first);
            }
            face.edgeIndices.push_back(found->second);
            mesh.edges[found->second].faceIndices.push_back(faceIndex);
        }
        const auto addTriangle = [&](std::size_t a, std::size_t b, std::size_t c) {
            const std::size_t triangleIndex = mesh.triangles.size();
            mesh.triangles.push_back({{a, b, c}, faceIndex});
            face.triangleIndices.push_back(triangleIndex);
        };
        if (polygon.size() == 3U) {
            addTriangle(polygon[0], polygon[1], polygon[2]);
        } else {
            for (std::size_t corner = 1U; corner + 1U < polygon.size(); ++corner) {
                addTriangle(polygon[0], polygon[corner], polygon[corner + 1U]);
            }
        }
    }
    for (solver::SourceEdge& edge : mesh.edges) {
        edge.originalMeshBoundary = edge.faceIndices.size() == 1U;
        if (edge.faceIndices.size() == 2U) {
            mesh.faces[edge.faceIndices[0]].adjacentFaceIndices.push_back(
                edge.faceIndices[1]);
            mesh.faces[edge.faceIndices[1]].adjacentFaceIndices.push_back(
                edge.faceIndices[0]);
        }
    }
    for (solver::SourceVertex& vertex : mesh.vertices) {
        solver::Vec3 normal;
        for (const std::size_t faceIndex : vertex.faceIndices) {
            normal += mesh.faces[faceIndex].normal;
        }
        vertex.normal = normal.normalized();
    }
    return mesh;
}

std::size_t findEdge(
    const solver::SourceMeshSnapshot& mesh,
    std::size_t first,
    std::size_t second)
{
    for (const std::size_t edgeIndex : mesh.vertices[first].edgeIndices) {
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
    const std::set<std::size_t> coreSet(coreFaces.begin(), coreFaces.end());
    for (std::size_t faceIndex = 0U;
         faceIndex < input.sourceMesh.faces.size();
         ++faceIndex) {
        component.allFaceIndices.push_back(faceIndex);
        if (coreSet.count(faceIndex) == 0U) {
            component.transitionFaceIndices.push_back(faceIndex);
        }
    }
    std::queue<std::size_t> frontier;
    for (const std::size_t faceIndex : coreFaces) {
        component.transitionRingDepthByFace[faceIndex] = 0;
        frontier.push(faceIndex);
    }
    while (!frontier.empty()) {
        const std::size_t faceIndex = frontier.front();
        frontier.pop();
        const int depth = component.transitionRingDepthByFace[faceIndex] + 1;
        for (const std::size_t adjacent :
             input.sourceMesh.faces[faceIndex].adjacentFaceIndices) {
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
        const std::size_t second =
            boundaryVertices[(index + 1U) % boundaryVertices.size()];
        const std::size_t edgeIndex = findEdge(input.sourceMesh, first, second);
        boundary.edgeIndices.push_back(edgeIndex);
        boundary.sourceVertexIds.push_back(
            input.sourceMesh.vertices[first].sourceVertexId);
        boundary.sourceEdgeIds.push_back(
            input.sourceMesh.edges[edgeIndex].sourceEdgeId);
        boundary.touchesOriginalMeshBoundary =
            boundary.touchesOriginalMeshBoundary ||
            input.sourceMesh.edges[edgeIndex].originalMeshBoundary;
    }
    component.fixedBoundaryLoops.push_back(std::move(boundary));
    input.components.push_back(std::move(component));
    input.directionField.resize(input.sourceMesh.faces.size());
    input.densityField.resize(input.sourceMesh.faces.size());
    for (std::size_t faceIndex = 0U;
         faceIndex < input.sourceMesh.faces.size();
         ++faceIndex) {
        const solver::Vec3 normal = input.sourceMesh.faces[faceIndex].normal;
        solver::Vec3 u = solver::Vec3{1.0, 0.0, 0.0} -
            normal * normal.dot(solver::Vec3{1.0, 0.0, 0.0});
        if (u.squaredLength() < 1.0e-12) {
            u = solver::Vec3{0.0, 1.0, 0.0} -
                normal * normal.dot(solver::Vec3{0.0, 1.0, 0.0});
        }
        u = u.normalized();
        input.directionField[faceIndex] = {
            normal, u, normal.cross(u).normalized(),
            coreSet.count(faceIndex) != 0U ? 1.0 : 0.25,
            coreSet.count(faceIndex) != 0U ? 0.0 : 0.5,
            true};
        input.densityField[faceIndex] = {
            targetEdgeLength, targetEdgeLength, 1.0, 1.0, false, true};
    }
    input.settings.topologyBlendWidth = std::max(blendWidth, 1U);
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
    for (std::size_t row = 0U; row <= rows; ++row) {
        for (std::size_t column = 0U; column <= columns; ++column) {
            const double x = static_cast<double>(column) -
                static_cast<double>(columns) * 0.5;
            const double y = static_cast<double>(row) -
                static_cast<double>(rows) * 0.5;
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
        for (std::size_t column = coreInset;
             column + coreInset < columns;
             ++column) {
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
            const double angle =
                2.0 * kPi * static_cast<double>(index) /
                static_cast<double>(segments);
            positions.push_back({
                radius * std::cos(angle),
                radius * std::sin(angle),
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
        polygons.push_back({
            1U + index,
            1U + segments + index,
            1U + segments + next,
            1U + next});
    }
    input.sourceMesh = buildMesh(positions, polygons);
    std::vector<std::size_t> coreFaces;
    std::vector<std::size_t> boundary;
    for (std::size_t index = 0U; index < segments; ++index) {
        coreFaces.push_back(index);
        boundary.push_back(1U + segments + index);
    }
    finalizeInput(input, coreFaces, boundary, 1.0, 2U);
    return input;
}

solver::RemeshInput makeMixedPolygonBoundary()
{
    constexpr std::size_t segments = 8U;
    solver::RemeshInput input;
    std::vector<solver::Vec3> positions;
    positions.push_back({0.0, 0.0, 0.15});
    for (std::size_t ring = 0U; ring < 2U; ++ring) {
        const double radius = ring == 0U ? 2.0 : 4.0;
        for (std::size_t index = 0U; index < segments; ++index) {
            const double angle =
                2.0 * kPi * static_cast<double>(index) /
                static_cast<double>(segments);
            positions.push_back({
                radius * std::cos(angle),
                radius * std::sin(angle),
                0.025 * radius * radius});
        }
    }
    std::vector<std::vector<std::size_t>> polygons;
    for (std::size_t index = 0U; index < segments; ++index) {
        const std::size_t next = (index + 1U) % segments;
        polygons.push_back({0U, 1U + index, 1U + next});
    }
    // One n-gon spans two Transition sectors, the next sector is split into
    // two source triangles, and the remaining sectors retain source quads.
    polygons.push_back({
        1U, 1U + segments, 2U + segments, 3U + segments, 3U, 2U});
    polygons.push_back({3U, 3U + segments, 4U + segments});
    polygons.push_back({3U, 4U + segments, 4U});
    for (std::size_t index = 3U; index < segments; ++index) {
        const std::size_t next = (index + 1U) % segments;
        polygons.push_back({
            1U + index,
            1U + segments + index,
            1U + segments + next,
            1U + next});
    }
    input.sourceMesh = buildMesh(positions, polygons);
    std::vector<std::size_t> coreFaces;
    std::vector<std::size_t> boundary;
    for (std::size_t index = 0U; index < segments; ++index) {
        coreFaces.push_back(index);
        boundary.push_back(1U + segments + index);
    }
    finalizeInput(input, coreFaces, boundary, 1.0, 2U);
    return input;
}

struct Fixture final
{
    std::string name;
    solver::RemeshInput input;
};

std::vector<Fixture> proceduralFixtures()
{
    return {
        {"plane_fine", makeGrid(8U, 8U, 2U, 0.5, 2U, "plane")},
        {"plane_coarse", makeGrid(8U, 8U, 2U, 2.0, 2U, "plane")},
        {"curved_fine", makeGrid(10U, 8U, 2U, 0.5, 2U, "curved")},
        {"curved_coarse", makeGrid(10U, 8U, 2U, 2.0, 2U, "curved")},
        {"small_region", makeGrid(3U, 3U, 1U, 1.0, 1U, "plane")},
        {"blend_width_1", makeGrid(8U, 8U, 2U, 1.0, 1U, "plane")},
        {"odd_boundary", makeOddBoundary()},
        {"dense_boundary_coarse_core",
         makeGrid(12U, 12U, 3U, 3.0, 2U, "plane")},
        {"chest_like_bump", makeGrid(12U, 10U, 3U, 0.8, 3U, "chest")},
        {"cloth_fold", makeGrid(12U, 10U, 3U, 0.7, 3U, "cloth")},
    };
}

bool validateSuccess(
    const solver::SourceMeshSnapshot& source,
    const solver::RegionComponent& component,
    const solver::SourceTransitionScaffold& scaffold,
    std::string& reason)
{
    if (!scaffold.success() ||
        scaffold.fixedOuterBoundaryLoops.size() != 1U ||
        scaffold.innerInterfaceLoops.size() != 1U) {
        reason = "successful scaffold does not contain exactly two boundary roles";
        return false;
    }
    if (std::abs(scaffold.diagnostics.maximumFixedBoundaryDisplacement) > 1.0e-12 ||
        scaffold.diagnostics.vertexMappingCoverage != 1.0 ||
        scaffold.diagnostics.edgeMappingCoverage != 1.0 ||
        scaffold.diagnostics.faceMappingCoverage != 1.0 ||
        scaffold.diagnostics.fixedBoundaryVertexCoverage != 1.0 ||
        scaffold.diagnostics.fixedBoundaryEdgeCoverage != 1.0) {
        reason = "mapping coverage or immutable boundary invariant failed";
        return false;
    }
    if (scaffold.faces.size() != component.transitionFaceIndices.size()) {
        reason = "local face count differs from Transition face count";
        return false;
    }
    for (const solver::ScaffoldVertex& vertex : scaffold.vertices) {
        if (vertex.sourceVertexIndex >= source.vertices.size() ||
            scaffold.localVertexIndexBySource[vertex.sourceVertexIndex] !=
                vertex.localIndex ||
            vertex.position.x != source.vertices[vertex.sourceVertexIndex].position.x ||
            vertex.position.y != source.vertices[vertex.sourceVertexIndex].position.y ||
            vertex.position.z != source.vertices[vertex.sourceVertexIndex].position.z) {
            reason = "vertex source/local mapping is not exact";
            return false;
        }
    }
    for (const solver::ScaffoldEdge& edge : scaffold.edges) {
        if (edge.sourceEdgeIndex >= source.edges.size() ||
            scaffold.localEdgeIndexBySource[edge.sourceEdgeIndex] != edge.localIndex) {
            reason = "edge source/local mapping is not exact";
            return false;
        }
    }
    for (const solver::ScaffoldFace& face : scaffold.faces) {
        if (face.sourceFaceIndex >= source.faces.size() ||
            scaffold.localFaceIndexBySource[face.sourceFaceIndex] != face.localIndex) {
            reason = "face source/local mapping is not exact";
            return false;
        }
        const solver::SourceFace& sourceFace = source.faces[face.sourceFaceIndex];
        if (face.vertexIndices.size() != sourceFace.vertexIndices.size() ||
            face.edgeIndices.size() != sourceFace.edgeIndices.size()) {
            reason = "original polygon topology was replaced by triangulation";
            return false;
        }
        for (std::size_t item = 0U; item < sourceFace.edgeIndices.size(); ++item) {
            const std::size_t localEdge = face.edgeIndices[item];
            if (localEdge >= scaffold.edges.size() ||
                scaffold.edges[localEdge].sourceEdgeIndex !=
                    sourceFace.edgeIndices[item]) {
                reason = "face contains an edge not present in original polygon topology";
                return false;
            }
        }
    }
    const auto validateLoop =
        [&source, &scaffold, &reason](const solver::ScaffoldBoundaryLoop& loop) {
            if (!loop.closed || loop.vertexIndices.size() < 3U ||
                loop.edgeIndices.size() != loop.vertexIndices.size()) {
                reason = "boundary loop is not an ordered closed cycle";
                return false;
            }
            for (std::size_t item = 0U; item < loop.vertexIndices.size(); ++item) {
                const std::size_t sourceVertex = loop.sourceVertexIndices[item];
                const std::size_t sourceNext =
                    loop.sourceVertexIndices[(item + 1U) % loop.sourceVertexIndices.size()];
                const std::size_t sourceEdge = loop.sourceEdgeIndices[item];
                if (sourceEdge >= source.edges.size() ||
                    findEdge(source, sourceVertex, sourceNext) != sourceEdge ||
                    scaffold.localVertexIndexBySource[sourceVertex] !=
                        loop.vertexIndices[item] ||
                    scaffold.localEdgeIndexBySource[sourceEdge] !=
                        loop.edgeIndices[item]) {
                    reason = "boundary traversal/source mapping mismatch";
                    return false;
                }
            }
            return true;
        };
    return validateLoop(scaffold.fixedOuterBoundaryLoops.front()) &&
        validateLoop(scaffold.innerInterfaceLoops.front());
}

struct EvaluationTotals final
{
    std::size_t fixtures = 0U;
    std::size_t components = 0U;
    std::size_t success = 0U;
    std::size_t structuredFailure = 0U;
    double milliseconds = 0.0;
};

bool evaluateInput(
    const std::string& label,
    const solver::RemeshInput& input,
    bool requireSuccess,
    EvaluationTotals& totals)
{
    ++totals.fixtures;
    const std::uint64_t immutableBefore = solver::remeshInputSignature(input);
    solver::SourceTransitionScaffoldExtractor extractor;
    bool valid = true;
    for (const solver::RegionComponent& component : input.components) {
        ++totals.components;
        solver::SourceTransitionScaffold scaffold =
            extractor.extract(input.sourceMesh, component, input.settings);
        totals.milliseconds += scaffold.diagnostics.extractionMilliseconds;
        const std::uint64_t signature =
            solver::sourceTransitionScaffoldSignature(scaffold);
        for (int repeat = 0; repeat < 5; ++repeat) {
            const solver::SourceTransitionScaffold repeated =
                extractor.extract(input.sourceMesh, component, input.settings);
            if (solver::sourceTransitionScaffoldSignature(repeated) != signature ||
                repeated.status != scaffold.status) {
                std::cerr << "[FAIL] " << label
                          << " nondeterministic scaffold extraction\n";
                valid = false;
            }
        }
        if (scaffold.success()) {
            ++totals.success;
            std::string reason;
            if (!validateSuccess(input.sourceMesh, component, scaffold, reason)) {
                std::cerr << "[FAIL] " << label << " " << reason << "\n";
                valid = false;
            }
        } else {
            ++totals.structuredFailure;
            if (requireSuccess ||
                scaffold.status == solver::ScaffoldStatus::InvalidInput) {
                std::cerr << "[FAIL] " << label << " unexpected "
                          << solver::scaffoldStatusName(scaffold.status)
                          << ": " << scaffold.diagnosticMessage << "\n";
                valid = false;
            }
        }
        const std::size_t outerVertices =
            scaffold.fixedOuterBoundaryLoops.empty()
                ? 0U
                : scaffold.fixedOuterBoundaryLoops.front().vertexIndices.size();
        const std::size_t innerVertices =
            scaffold.innerInterfaceLoops.empty()
                ? 0U
                : scaffold.innerInterfaceLoops.front().vertexIndices.size();
        std::cout << std::left << std::setw(55) << label
                  << " c=" << component.componentId
                  << " status=" << std::setw(27)
                  << solver::scaffoldStatusName(scaffold.status)
                  << " transition=" << std::setw(4) << scaffold.faces.size()
                  << " rings=" << std::setw(2)
                  << scaffold.diagnostics.transitionRingCount
                  << " outer=" << std::setw(4) << outerVertices
                  << " inner=" << std::setw(4) << innerVertices
                  << " poly=" << scaffold.diagnostics.triangleCount << "/"
                  << scaffold.diagnostics.quadCount << "/"
                  << scaffold.diagnostics.nGonCount
                  << " map=" << std::fixed << std::setprecision(2)
                  << scaffold.diagnostics.faceMappingCoverage
                  << " ms=" << std::setprecision(3)
                  << scaffold.diagnostics.extractionMilliseconds << "\n";
    }
    if (solver::remeshInputSignature(input) != immutableBefore) {
        std::cerr << "[FAIL] " << label << " input was mutated\n";
        valid = false;
    }
    return valid;
}

bool verifyStructuredFailures()
{
    solver::SourceTransitionScaffoldExtractor extractor;
    solver::RemeshInput mixed = makeMixedPolygonBoundary();
    const solver::SourceTransitionScaffold mixedResult = extractor.extract(
        mixed.sourceMesh, mixed.components.front(), mixed.settings);
    if (!mixedResult.success() ||
        mixedResult.diagnostics.triangleCount != 2U ||
        mixedResult.diagnostics.quadCount != 5U ||
        mixedResult.diagnostics.nGonCount != 1U) {
        std::cerr << "[FAIL] mixed source triangle/quad/n-gon classification\n";
        return false;
    }

    solver::RemeshInput multipleOuter =
        makeGrid(6U, 6U, 2U, 1.0, 2U, "plane");
    multipleOuter.components.front().fixedBoundaryLoops.push_back(
        multipleOuter.components.front().fixedBoundaryLoops.front());
    const solver::SourceTransitionScaffold multipleResult = extractor.extract(
        multipleOuter.sourceMesh,
        multipleOuter.components.front(),
        multipleOuter.settings);
    if (multipleResult.status !=
        solver::ScaffoldStatus::MultipleOuterBoundaries) {
        std::cerr << "[FAIL] multiple outer boundary was not classified\n";
        return false;
    }

    solver::RemeshInput badRing =
        makeGrid(6U, 6U, 2U, 1.0, 2U, "plane");
    const std::size_t transitionFace =
        badRing.components.front().transitionFaceIndices.front();
    badRing.components.front().transitionRingDepthByFace[transitionFace] = 99;
    const solver::SourceTransitionScaffold ringResult = extractor.extract(
        badRing.sourceMesh, badRing.components.front(), badRing.settings);
    if (ringResult.status != solver::ScaffoldStatus::RingDepthInvalid) {
        std::cerr << "[FAIL] invalid ring adjacency was not classified\n";
        return false;
    }

    solver::RemeshInput disconnected =
        makeGrid(6U, 6U, 2U, 1.0, 2U, "plane");
    solver::RegionComponent& disconnectedComponent =
        disconnected.components.front();
    const std::size_t isolatedFace =
        disconnectedComponent.transitionFaceIndices.front();
    disconnectedComponent.transitionFaceIndices.erase(
        disconnectedComponent.transitionFaceIndices.begin());
    disconnectedComponent.coreFaceIndices.push_back(isolatedFace);
    disconnectedComponent.transitionRingDepthByFace[isolatedFace] = 0;
    const solver::SourceTransitionScaffold disconnectedResult =
        extractor.extract(
            disconnected.sourceMesh,
            disconnectedComponent,
            disconnected.settings);
    if (disconnectedResult.status != solver::ScaffoldStatus::CoreDisconnected) {
        std::cerr << "[FAIL] disconnected Core was not classified\n";
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    bool valid = verifyStructuredFailures();
    EvaluationTotals proceduralTotals;
    std::cout << "[R4] Procedural Source Transition Scaffold matrix\n";
    for (const Fixture& fixture : proceduralFixtures()) {
        valid = evaluateInput(
                    "procedural/" + fixture.name,
                    fixture.input,
                    true,
                    proceduralTotals) &&
            valid;
    }

    const std::filesystem::path captureDirectory =
        argc > 1 ? std::filesystem::path(argv[1])
                 : std::filesystem::path("tests/fixtures/captured");
    std::vector<std::filesystem::path> capturePaths;
    if (std::filesystem::exists(captureDirectory)) {
        for (const std::filesystem::directory_entry& entry :
             std::filesystem::directory_iterator(captureDirectory)) {
            if (entry.is_regular_file() &&
                entry.path().extension() == ".drinput") {
                capturePaths.push_back(entry.path());
            }
        }
    }
    std::sort(capturePaths.begin(), capturePaths.end());
    if (capturePaths.size() != 12U) {
        std::cerr << "[FAIL] expected 12 captured fixtures, found "
                  << capturePaths.size() << "\n";
        valid = false;
    }

    EvaluationTotals capturedTotals;
    std::cout << "[R4] Captured Maya Source Transition Scaffold matrix\n";
    for (const std::filesystem::path& path : capturePaths) {
        solver::RemeshCaptureRecord capture;
        std::string diagnostic;
        if (!solver::loadRemeshCapture(path.string(), capture, diagnostic)) {
            std::cerr << "[FAIL] cannot load " << path.filename().string()
                      << ": " << diagnostic << "\n";
            valid = false;
            continue;
        }
        valid = evaluateInput(
                    "captured/" + path.filename().string(),
                    capture.input,
                    false,
                    capturedTotals) &&
            valid;
    }

    std::cout << "[R4] procedural fixtures=" << proceduralTotals.fixtures
              << " components=" << proceduralTotals.components
              << " success=" << proceduralTotals.success
              << " structuredFailure=" << proceduralTotals.structuredFailure
              << " extractionMs=" << std::fixed << std::setprecision(3)
              << proceduralTotals.milliseconds << "\n";
    std::cout << "[R4] captured fixtures=" << capturedTotals.fixtures
              << " components=" << capturedTotals.components
              << " success=" << capturedTotals.success
              << " structuredFailure=" << capturedTotals.structuredFailure
              << " extractionMs=" << capturedTotals.milliseconds << "\n";
    std::cout << "[R4] immutable=yes deterministicRepeats=5 "
              << "originalPolygonEdgesOnly=yes\n";
    return valid ? 0 : 1;
}
