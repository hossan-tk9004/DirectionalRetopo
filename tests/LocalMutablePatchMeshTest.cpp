#include "Solver/LocalMutablePatchMesh.h"
#include "Solver/RemeshCapture.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace solver = directional_retopo::solver;
namespace {

using Pair = std::pair<std::size_t, std::size_t>;

Pair key(std::size_t first, std::size_t second)
{
    return std::minmax(first, second);
}

solver::SourceTransitionScaffold makeScaffold(
    const std::vector<solver::Vec3>& positions,
    const std::vector<std::vector<std::size_t>>& polygons,
    const std::vector<std::size_t>& fixedLoop,
    const std::vector<std::size_t>& innerLoop)
{
    solver::SourceTransitionScaffold scaffold;
    scaffold.status = solver::ScaffoldStatus::Success;
    scaffold.vertices.resize(positions.size());
    std::set<std::size_t> fixedVertices(fixedLoop.begin(), fixedLoop.end());
    std::set<std::size_t> innerVertices(innerLoop.begin(), innerLoop.end());
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        solver::ScaffoldVertex& vertex = scaffold.vertices[index];
        vertex.localIndex = index;
        vertex.sourceVertexIndex = index;
        vertex.sourceVertexId = static_cast<solver::SourceId>(1000U + index);
        vertex.position = positions[index];
        vertex.normal = {0.0, 0.0, 1.0};
        vertex.classification = fixedVertices.count(index) != 0U
            ? solver::ScaffoldVertexClassification::FixedOuterBoundary
            : innerVertices.count(index) != 0U
                ? solver::ScaffoldVertexClassification::InnerInterface
                : solver::ScaffoldVertexClassification::TransitionInterior;
    }
    std::map<Pair, std::size_t> edgeMap;
    scaffold.faces.resize(polygons.size());
    for (std::size_t faceIndex = 0U; faceIndex < polygons.size(); ++faceIndex) {
        solver::ScaffoldFace& face = scaffold.faces[faceIndex];
        face.localIndex = faceIndex;
        face.sourceFaceIndex = faceIndex;
        face.sourceFaceId = static_cast<solver::SourceId>(3000U + faceIndex);
        face.vertexIndices = polygons[faceIndex];
        face.transitionRingDepth = faceIndex < 4U ? 2 : 1;
        face.polygonType = polygons[faceIndex].size() == 3U
            ? solver::PolygonType::Triangle
            : polygons[faceIndex].size() == 4U
                ? solver::PolygonType::Quad
                : solver::PolygonType::NGon;
        for (std::size_t corner = 0U; corner < polygons[faceIndex].size(); ++corner) {
            const std::size_t a = polygons[faceIndex][corner];
            const std::size_t b = polygons[faceIndex][
                (corner + 1U) % polygons[faceIndex].size()];
            const Pair pair = key(a, b);
            auto found = edgeMap.find(pair);
            if (found == edgeMap.end()) {
                solver::ScaffoldEdge edge;
                edge.localIndex = scaffold.edges.size();
                edge.sourceEdgeIndex = edge.localIndex;
                edge.sourceEdgeId = static_cast<solver::SourceId>(2000U + edge.localIndex);
                edge.vertexIndices = {pair.first, pair.second};
                scaffold.edges.push_back(edge);
                found = edgeMap.emplace(pair, edge.localIndex).first;
            }
            face.edgeIndices.push_back(found->second);
            scaffold.edges[found->second].faceIndices.push_back(faceIndex);
        }
    }
    const auto makeLoop = [&scaffold, &edgeMap](
                              const std::vector<std::size_t>& vertices,
                              solver::ScaffoldEdgeClassification classification) {
        solver::ScaffoldBoundaryLoop loop;
        loop.closed = true;
        loop.vertexIndices = vertices;
        loop.sourceVertexIndices = vertices;
        for (std::size_t index = 0U; index < vertices.size(); ++index) {
            const std::size_t edgeId = edgeMap.at(key(
                vertices[index], vertices[(index + 1U) % vertices.size()]));
            loop.edgeIndices.push_back(edgeId);
            loop.sourceEdgeIndices.push_back(edgeId);
            loop.sourceVertexIds.push_back(scaffold.vertices[vertices[index]].sourceVertexId);
            loop.sourceEdgeIds.push_back(scaffold.edges[edgeId].sourceEdgeId);
            scaffold.edges[edgeId].classification = classification;
        }
        return loop;
    };
    scaffold.fixedOuterBoundaryLoops.push_back(makeLoop(
        fixedLoop, solver::ScaffoldEdgeClassification::FixedOuterBoundary));
    scaffold.innerInterfaceLoops.push_back(makeLoop(
        innerLoop, solver::ScaffoldEdgeClassification::InnerInterface));
    scaffold.localVertexIndexBySource.resize(scaffold.vertices.size());
    scaffold.localEdgeIndexBySource.resize(scaffold.edges.size());
    scaffold.localFaceIndexBySource.resize(scaffold.faces.size());
    for (std::size_t index = 0U; index < scaffold.vertices.size(); ++index) {
        scaffold.localVertexIndexBySource[index] = index;
    }
    for (std::size_t index = 0U; index < scaffold.edges.size(); ++index) {
        scaffold.localEdgeIndexBySource[index] = index;
    }
    for (std::size_t index = 0U; index < scaffold.faces.size(); ++index) {
        scaffold.localFaceIndexBySource[index] = index;
    }
    return scaffold;
}

solver::SourceTransitionScaffold annulus(
    bool triangulated,
    bool mixed = false,
    bool oppositeDiagonalExists = false)
{
    std::vector<solver::Vec3> positions;
    for (const double radius : {3.0, 2.0, 1.0}) {
        positions.push_back({-radius, -radius, 0.0});
        positions.push_back({radius, -radius, 0.0});
        positions.push_back({radius, radius, 0.0});
        positions.push_back({-radius, radius, 0.0});
    }
    std::vector<std::vector<std::size_t>> polygons;
    for (std::size_t index = 0U; index < 4U; ++index) {
        const std::size_t next = (index + 1U) % 4U;
        polygons.push_back({index, next, 4U + next, 4U + index});
    }
    for (std::size_t index = 0U; index < 4U; ++index) {
        const std::size_t next = (index + 1U) % 4U;
        if (triangulated || (mixed && index == 0U)) {
            polygons.push_back({4U + index, 4U + next, 8U + next});
            polygons.push_back({4U + index, 8U + next, 8U + index});
        } else {
            polygons.push_back({4U + index, 4U + next, 8U + next, 8U + index});
        }
    }
    if (oppositeDiagonalExists) {
        positions.push_back({0.0, -3.0, 1.0});
        positions.push_back({0.0, -3.0, -1.0});
        polygons.push_back({5U, 8U, 12U});
        polygons.push_back({8U, 5U, 13U});
        polygons.push_back({5U, 12U, 13U});
        polygons.push_back({8U, 13U, 12U});
    }
    return makeScaffold(positions, polygons, {0U, 1U, 2U, 3U},
        {8U, 9U, 10U, 11U});
}

solver::LocalMutablePatchMesh mutableAnnulus(
    bool triangulated,
    bool mixed = false,
    bool oppositeDiagonalExists = false)
{
    std::string reason;
    solver::LocalMutablePatchMesh mesh = solver::LocalMutablePatchMesh::fromScaffold(
        annulus(triangulated, mixed, oppositeDiagonalExists), 2U, &reason);
    if (!mesh.valid()) {
        std::cerr << "[FAIL] synthetic mutable annulus: " << reason << "\n";
    }
    return mesh;
}

solver::SourceTransitionScaffold annulusWithSourceNGon()
{
    std::vector<solver::Vec3> positions;
    for (const double radius : {3.0, 2.0, 1.0}) {
        positions.push_back({-radius, -radius, 0.0});
        positions.push_back({radius, -radius, 0.0});
        positions.push_back({radius, radius, 0.0});
        positions.push_back({-radius, radius, 0.0});
    }
    std::vector<std::vector<std::size_t>> polygons = {
        {0U, 1U, 2U, 6U, 5U, 4U},
        {2U, 3U, 7U, 6U},
        {3U, 0U, 4U, 7U}};
    for (std::size_t index = 0U; index < 4U; ++index) {
        const std::size_t next = (index + 1U) % 4U;
        polygons.push_back({4U + index, 4U + next, 8U + next, 8U + index});
    }
    return makeScaffold(positions, polygons, {0U, 1U, 2U, 3U},
        {8U, 9U, 10U, 11U});
}

std::size_t findEdge(
    const solver::LocalMutablePatchMesh& mesh,
    const std::function<bool(const solver::MutableEdge&)>& predicate)
{
    for (const solver::MutableEdge& edge : mesh.edges()) {
        if (!edge.deleted && predicate(edge)) { return edge.id; }
    }
    return solver::kInvalidIndex;
}

bool expectFailurePreserves(
    solver::LocalMutablePatchMesh& mesh,
    const std::function<solver::MutableOperationResult()>& operation,
    const std::string& label)
{
    const std::uint64_t before = mesh.signature();
    const solver::MutableOperationResult result = operation();
    if (result.success() || mesh.signature() != before || !mesh.valid()) {
        std::cerr << "[FAIL] rollback " << label << " status="
                  << solver::mutableOperationStatusName(result.status) << "\n";
        return false;
    }
    return true;
}

bool testCopyAndFixedBoundary()
{
    bool ok = true;
    solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
    const solver::MutablePatchDiagnostics& diagnostics = mesh.diagnostics();
    if (!mesh.valid() || diagnostics.vertexSourceCoverage != 1.0 ||
        diagnostics.edgeSourceCoverage != 1.0 ||
        diagnostics.faceSourceCoverage != 1.0 ||
        diagnostics.maximumFixedBoundaryDisplacement != 0.0 ||
        diagnostics.requestedBlendWidth != 2U ||
        diagnostics.actualMaximumRingDepth != 2U) {
        std::cerr << "[FAIL] no-op copy/mapping diagnostics\n";
        ok = false;
    }
    for (std::size_t sourceIndex = 0U; sourceIndex < mesh.vertices().size(); ++sourceIndex) {
        if (mesh.vertexIdFromSourceIndex(sourceIndex) != sourceIndex) {
            std::cerr << "[FAIL] source vertex inverse lookup\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh warningCopy =
            solver::LocalMutablePatchMesh::fromScaffold(annulus(false), 5U);
        if (!warningCopy.valid() || !warningCopy.diagnostics().ringDepthMismatch ||
            warningCopy.diagnostics().requestedBlendWidth != 5U ||
            warningCopy.diagnostics().actualMaximumRingDepth != 2U) {
            std::cerr << "[FAIL] requested/actual ring-depth warning semantics\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh nGonCopy =
            solver::LocalMutablePatchMesh::fromScaffold(annulusWithSourceNGon(), 2U);
        bool retained = false;
        for (const solver::MutableFace& face : nGonCopy.faces()) {
            retained = retained || (!face.deleted && face.vertexIds.size() == 6U);
        }
        if (!nGonCopy.valid() || !retained) {
            std::cerr << "[FAIL] source n-gon no-op retention\n";
            ok = false;
        }
    }
    const std::size_t fixedEdge = findEdge(mesh, [](const solver::MutableEdge& edge) {
        return edge.fixedOuterBoundary;
    });
    const std::size_t fixedVertex = mesh.edges()[fixedEdge].vertex0;
    ok = expectFailurePreserves(mesh, [&]() {
        return mesh.moveVertex(fixedVertex, {99.0, 99.0, 99.0});
    }, "move fixed vertex") && ok;
    ok = expectFailurePreserves(mesh, [&]() {
        return mesh.splitEdge(fixedEdge);
    }, "split fixed edge") && ok;
    ok = expectFailurePreserves(mesh, [&]() {
        return mesh.collapseEdgeToEndpoint(fixedEdge, fixedVertex);
    }, "collapse fixed edge") && ok;
    ok = expectFailurePreserves(mesh, [&]() {
        return mesh.dissolveEdge(fixedEdge);
    }, "dissolve fixed edge") && ok;
    ok = expectFailurePreserves(mesh, [&]() {
        return mesh.flipTriangleEdge(fixedEdge);
    }, "flip fixed edge") && ok;
    return ok && mesh.diagnostics().maximumFixedBoundaryDisplacement == 0.0;
}

bool testSplitOperations()
{
    bool ok = true;
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
        const std::size_t diagonal = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                edge.faceIds.size() == 2U &&
                mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
                mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
        });
        const auto result = mesh.splitEdge(diagonal, 0.4);
        if (!result.success() || result.changes.createdVertexIds.size() != 1U ||
            result.changes.createdEdgeIds.size() < 2U || !mesh.valid()) {
            std::cerr << "[FAIL] split triangle pair\n";
            ok = false;
        }
        if (result.success() &&
            (!mesh.edges()[diagonal].deleted ||
             result.changes.createdEdgeIds.front() <= diagonal)) {
            std::cerr << "[FAIL] split tombstone/monotonic ID rule\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t quadEdge = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                edge.faceIds.size() == 2U &&
                mesh.faces()[edge.faceIds[0]].vertexIds.size() == 4U &&
                mesh.faces()[edge.faceIds[1]].vertexIds.size() == 4U;
        });
        if (!mesh.splitEdge(quadEdge).success() || !mesh.valid()) {
            std::cerr << "[FAIL] split quad pair\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t interfaceEdge = findEdge(mesh, [](const solver::MutableEdge& edge) {
            return edge.innerInterface;
        });
        const std::size_t before = mesh.orderedInnerInterfaceVertices().size();
        if (!mesh.splitEdge(interfaceEdge).success() ||
            mesh.orderedInnerInterfaceVertices().size() != before + 1U ||
            !mesh.valid()) {
            std::cerr << "[FAIL] split/rebuild inner interface\n";
            ok = false;
        }
    }
    return ok;
}

bool testCollapseOperations()
{
    bool ok = true;
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                !mesh.vertices()[edge.vertex0].fixedOuterBoundary &&
                !mesh.vertices()[edge.vertex1].fixedOuterBoundary &&
                !mesh.vertices()[edge.vertex0].innerInterface &&
                !mesh.vertices()[edge.vertex1].innerInterface;
        });
        if (!mesh.collapseEdgeToEndpoint(edgeId, mesh.edges()[edgeId].vertex0).success() ||
            !mesh.valid()) {
            std::cerr << "[FAIL] valid interior collapse\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t edgeId = findEdge(mesh, [](const solver::MutableEdge& edge) {
            return edge.innerInterface;
        });
        const std::size_t before = mesh.orderedInnerInterfaceVertices().size();
        if (!mesh.collapseEdgeToEndpoint(edgeId, mesh.edges()[edgeId].vertex0).success() ||
            mesh.orderedInnerInterfaceVertices().size() + 1U != before ||
            !mesh.valid()) {
            std::cerr << "[FAIL] valid inner-interface collapse\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t invalid = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.innerInterface && !edge.fixedOuterBoundary &&
                (mesh.vertices()[edge.vertex0].innerInterface ||
                 mesh.vertices()[edge.vertex1].innerInterface);
        });
        const std::size_t keep = mesh.vertices()[mesh.edges()[invalid].vertex0].innerInterface
            ? mesh.edges()[invalid].vertex1 : mesh.edges()[invalid].vertex0;
        ok = expectFailurePreserves(mesh, [&]() {
            return mesh.collapseEdgeToEndpoint(invalid, keep);
        }, "collapse interface across non-interface edge") && ok;
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
        const std::size_t diagonal = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                edge.faceIds.size() == 2U &&
                mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
                mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
        });
        ok = expectFailurePreserves(mesh, [&]() {
            return mesh.collapseEdgeToEndpoint(diagonal, mesh.edges()[diagonal].vertex0);
        }, "collapse that opens transition topology") && ok;
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                !mesh.vertices()[edge.vertex0].fixedOuterBoundary;
        });
        ok = expectFailurePreserves(mesh, [&]() {
            return mesh.moveVertex(
                mesh.edges()[edgeId].vertex0,
                mesh.vertices()[mesh.edges()[edgeId].vertex1].position);
        }, "move that creates zero-length edge") && ok;
    }
    return ok;
}

bool testDissolveAndFlip()
{
    bool ok = true;
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
        const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                edge.faceIds.size() == 2U &&
                mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
                mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
        });
        if (!mesh.dissolveEdge(edgeId).success() || !mesh.valid()) {
            std::cerr << "[FAIL] triangle-triangle dissolve\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false, true);
        const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            if (edge.fixedOuterBoundary || edge.innerInterface || edge.faceIds.size() != 2U) {
                return false;
            }
            const std::size_t a = mesh.faces()[edge.faceIds[0]].vertexIds.size();
            const std::size_t b = mesh.faces()[edge.faceIds[1]].vertexIds.size();
            return (a == 3U && b == 4U) || (a == 4U && b == 3U);
        });
        const auto result = mesh.dissolveEdge(edgeId);
        bool fiveGon = false;
        for (const solver::MutableFace& face : mesh.faces()) {
            fiveGon = fiveGon || (!face.deleted && face.vertexIds.size() == 5U);
        }
        if (!result.success() || !fiveGon || !mesh.valid()) {
            std::cerr << "[FAIL] triangle-quad dissolve into source n-gon\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
        const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface &&
                edge.faceIds.size() == 2U &&
                mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
                mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
        });
        const auto flipped = mesh.flipTriangleEdge(edgeId);
        if (!flipped.success() || !mesh.valid()) {
            std::cerr << "[FAIL] triangle diagonal flip: "
                      << flipped.diagnosticMessage << "\n";
            ok = false;
        }
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
        const std::size_t nonTriangle = findEdge(mesh, [](const solver::MutableEdge& edge) {
            return !edge.fixedOuterBoundary && !edge.innerInterface && edge.faceIds.size() == 2U;
        });
        ok = expectFailurePreserves(mesh, [&]() {
            return mesh.flipTriangleEdge(nonTriangle);
        }, "flip non-triangle pair") && ok;
    }
    {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(true, false, true);
        const std::size_t duplicate = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
            if (edge.fixedOuterBoundary || edge.innerInterface || edge.faceIds.size() != 2U ||
                mesh.faces()[edge.faceIds[0]].vertexIds.size() != 3U ||
                mesh.faces()[edge.faceIds[1]].vertexIds.size() != 3U) {
                return false;
            }
            std::vector<std::size_t> opposite;
            for (const std::size_t faceId : edge.faceIds) {
                for (const std::size_t vertexId : mesh.faces()[faceId].vertexIds) {
                    if (vertexId != edge.vertex0 && vertexId != edge.vertex1) {
                        opposite.push_back(vertexId);
                    }
                }
            }
            return opposite.size() == 2U &&
                findEdge(mesh, [&opposite](const solver::MutableEdge& other) {
                    return key(other.vertex0, other.vertex1) ==
                        key(opposite[0], opposite[1]);
                }) != solver::kInvalidIndex;
        });
        ok = expectFailurePreserves(mesh, [&]() {
            return mesh.flipTriangleEdge(duplicate);
        }, "flip when opposite diagonal exists") && ok;
    }
    return ok;
}

bool applyDeterministicSequence(solver::LocalMutablePatchMesh& mesh)
{
    const std::size_t diagonal = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
        return !edge.fixedOuterBoundary && !edge.innerInterface &&
            edge.faceIds.size() == 2U &&
            mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
            mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
    });
    const auto flipped = mesh.flipTriangleEdge(diagonal);
    if (!flipped.success() || flipped.changes.createdEdgeIds.empty()) {
        std::cerr << "[FAIL] sequence flip: " << flipped.diagnosticMessage << "\n";
        return false;
    }
    const auto split = mesh.splitEdge(flipped.changes.createdEdgeIds.front(), 0.5);
    if (!split.success() || split.changes.createdVertexIds.empty()) { return false; }
    const std::size_t newVertex = split.changes.createdVertexIds.front();
    const std::size_t child = findEdge(mesh, [newVertex](const solver::MutableEdge& edge) {
        return !edge.fixedOuterBoundary && !edge.innerInterface &&
            (edge.vertex0 == newVertex || edge.vertex1 == newVertex);
    });
    const std::size_t keep = mesh.edges()[child].vertex0 == newVertex
        ? mesh.edges()[child].vertex1 : mesh.edges()[child].vertex0;
    if (!mesh.collapseEdgeToEndpoint(child, keep).success()) { return false; }
    const std::size_t remainingDiagonal = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
        return !edge.fixedOuterBoundary && !edge.innerInterface &&
            edge.faceIds.size() == 2U &&
            mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
            mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
    });
    return remainingDiagonal != solver::kInvalidIndex &&
        mesh.dissolveEdge(remainingDiagonal).success() && mesh.valid() &&
        mesh.diagnostics().maximumFixedBoundaryDisplacement == 0.0;
}

bool testSequenceAndDeterminism()
{
    std::uint64_t expected = 0U;
    for (int repeat = 0; repeat < 5; ++repeat) {
        solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
        if (!applyDeterministicSequence(mesh)) {
            std::cerr << "[FAIL] mixed operation sequence\n";
            return false;
        }
        if (mesh.operationLineage().size() != 4U) {
            std::cerr << "[FAIL] operation lineage record count\n";
            return false;
        }
        if (repeat == 0) {
            expected = mesh.signature();
        } else if (mesh.signature() != expected) {
            std::cerr << "[FAIL] operation sequence is nondeterministic\n";
            return false;
        }
    }
    return true;
}

struct CaptureTotals final
{
    std::size_t files = 0U;
    std::size_t components = 0U;
    std::size_t probes = 0U;
    double copyMilliseconds = 0.0;
    double validationMilliseconds = 0.0;
    double probeMilliseconds = 0.0;
};

bool testCapturedFixtures(
    const std::filesystem::path& directory,
    CaptureTotals& totals)
{
    std::vector<std::filesystem::path> paths;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".drinput") {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    totals.files = paths.size();
    bool ok = paths.size() == 12U;
    solver::SourceTransitionScaffoldExtractor extractor;
    for (const std::filesystem::path& path : paths) {
        solver::RemeshCaptureRecord capture;
        std::string reason;
        if (!solver::loadRemeshCapture(path.string(), capture, reason)) {
            std::cerr << "[FAIL] capture load " << path.filename().string()
                      << ": " << reason << "\n";
            ok = false;
            continue;
        }
        for (const solver::RegionComponent& component : capture.input.components) {
            ++totals.components;
            const solver::SourceTransitionScaffold scaffold = extractor.extract(
                capture.input.sourceMesh, component, capture.input.settings);
            if (!scaffold.success()) {
                std::cerr << "[FAIL] R4 scaffold for " << path.filename().string()
                          << ": " << scaffold.diagnosticMessage << "\n";
                ok = false;
                continue;
            }
            const auto copyStart = std::chrono::steady_clock::now();
            solver::LocalMutablePatchMesh mesh =
                solver::LocalMutablePatchMesh::fromScaffold(
                    scaffold, capture.input.settings.topologyBlendWidth, &reason);
            totals.copyMilliseconds += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - copyStart).count();
            const auto validationStart = std::chrono::steady_clock::now();
            const bool valid = mesh.valid(&reason);
            totals.validationMilliseconds += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - validationStart).count();
            if (!valid || mesh.diagnostics().maximumFixedBoundaryDisplacement != 0.0 ||
                mesh.diagnostics().vertexSourceCoverage != 1.0 ||
                mesh.diagnostics().edgeSourceCoverage != 1.0 ||
                mesh.diagnostics().faceSourceCoverage != 1.0) {
                std::cerr << "[FAIL] mutable copy " << path.filename().string()
                          << ": " << reason << "\n";
                ok = false;
                continue;
            }
            const std::uint64_t sourceSignature =
                solver::sourceTransitionScaffoldSignature(scaffold);
            const solver::SourceTransitionScaffold repeatedScaffold = extractor.extract(
                capture.input.sourceMesh, component, capture.input.settings);
            solver::LocalMutablePatchMesh repeated =
                solver::LocalMutablePatchMesh::fromScaffold(
                    repeatedScaffold, capture.input.settings.topologyBlendWidth);
            if (solver::sourceTransitionScaffoldSignature(repeatedScaffold) != sourceSignature ||
                repeated.signature() != mesh.signature()) {
                std::cerr << "[FAIL] deterministic copy " << path.filename().string() << "\n";
                ok = false;
            }
            const std::size_t probeEdge = findEdge(mesh, [](const solver::MutableEdge& edge) {
                return !edge.fixedOuterBoundary;
            });
            if (probeEdge == solver::kInvalidIndex) {
                std::cerr << "[FAIL] no mutable probe edge " << path.filename().string() << "\n";
                ok = false;
                continue;
            }
            const auto probeStart = std::chrono::steady_clock::now();
            const solver::MutableOperationResult probe = mesh.splitEdge(probeEdge, 0.5);
            totals.probeMilliseconds += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - probeStart).count();
            if (!probe.success() || !mesh.valid() ||
                mesh.diagnostics().maximumFixedBoundaryDisplacement != 0.0) {
                std::cerr << "[FAIL] captured split probe "
                          << path.filename().string() << ": "
                          << probe.diagnosticMessage << "\n";
                ok = false;
            } else {
                ++totals.probes;
            }
        }
    }
    if (totals.components != 20U) {
        std::cerr << "[FAIL] expected 20 captured components, found "
                  << totals.components << "\n";
        ok = false;
    }
    return ok;
}

struct OperationTimings final
{
    double split = 0.0;
    double collapse = 0.0;
    double dissolve = 0.0;
    double flip = 0.0;
};

OperationTimings timedSyntheticOperations()
{
    OperationTimings timings;
    constexpr int repeats = 50;
    const auto measure = [](const std::function<void()>& operation) {
        const auto start = std::chrono::steady_clock::now();
        operation();
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    };
    for (int repeat = 0; repeat < repeats; ++repeat) {
        {
            solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
            const std::size_t edgeId = findEdge(mesh, [](const solver::MutableEdge& edge) {
                return edge.innerInterface;
            });
            timings.split += measure([&]() { (void)mesh.splitEdge(edgeId); });
        }
        {
            solver::LocalMutablePatchMesh mesh = mutableAnnulus(false);
            const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
                return !edge.fixedOuterBoundary && !edge.innerInterface &&
                    !mesh.vertices()[edge.vertex0].innerInterface &&
                    !mesh.vertices()[edge.vertex1].innerInterface;
            });
            timings.collapse += measure([&]() {
                (void)mesh.collapseEdgeToEndpoint(edgeId, mesh.edges()[edgeId].vertex0);
            });
        }
        {
            solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
            const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
                return !edge.fixedOuterBoundary && !edge.innerInterface &&
                    edge.faceIds.size() == 2U &&
                    mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
                    mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
            });
            timings.dissolve += measure([&]() { (void)mesh.dissolveEdge(edgeId); });
        }
        {
            solver::LocalMutablePatchMesh mesh = mutableAnnulus(true);
            const std::size_t edgeId = findEdge(mesh, [&mesh](const solver::MutableEdge& edge) {
                return !edge.fixedOuterBoundary && !edge.innerInterface &&
                    edge.faceIds.size() == 2U &&
                    mesh.faces()[edge.faceIds[0]].vertexIds.size() == 3U &&
                    mesh.faces()[edge.faceIds[1]].vertexIds.size() == 3U;
            });
            timings.flip += measure([&]() { (void)mesh.flipTriangleEdge(edgeId); });
        }
    }
    timings.split /= repeats;
    timings.collapse /= repeats;
    timings.dissolve /= repeats;
    timings.flip /= repeats;
    return timings;
}

}  // namespace

int main(int argc, char** argv)
{
    bool ok = testCopyAndFixedBoundary();
    ok = testSplitOperations() && ok;
    ok = testCollapseOperations() && ok;
    ok = testDissolveAndFlip() && ok;
    ok = testSequenceAndDeterminism() && ok;

    const std::filesystem::path captureDirectory =
        argc > 1 ? std::filesystem::path(argv[1])
                 : std::filesystem::path("tests/fixtures/captured");
    CaptureTotals totals;
    ok = testCapturedFixtures(captureDirectory, totals) && ok;
    const OperationTimings operationTimings = timedSyntheticOperations();
    std::cout << "[R5] LocalMutablePatchMesh captured files=" << totals.files
              << " components=" << totals.components
              << " safeSplitProbes=" << totals.probes
              << " copyMs=" << totals.copyMilliseconds
              << " validateMs=" << totals.validationMilliseconds
              << " probeMs=" << totals.probeMilliseconds << "\n";
    std::cout << "[R5] split/collapse/dissolve/flip sequence repeats=5 deterministic=yes"
              << " fixedBoundaryDisplacement=0 averageMs(split/collapse/dissolve/flip)="
              << operationTimings.split << "/" << operationTimings.collapse << "/"
              << operationTimings.dissolve << "/" << operationTimings.flip << "\n";
    std::cout << "[R5] productionSolverIntegration=disabled sourceMeshMutation=none\n";
    return ok ? 0 : 1;
}
