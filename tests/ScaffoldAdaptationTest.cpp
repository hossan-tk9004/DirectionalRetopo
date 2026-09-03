#include "Solver/RemeshCapture.h"
#include "Solver/ScaffoldAdaptationSolver.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace solver = directional_retopo::solver;
namespace {

constexpr double kPi = 3.14159265358979323846;
using EdgeKey = std::pair<std::size_t, std::size_t>;

EdgeKey edgeKey(std::size_t first, std::size_t second)
{
    return std::minmax(first, second);
}

solver::SourceMeshSnapshot buildMesh(
    const std::vector<solver::Vec3>& positions,
    const std::vector<std::vector<std::size_t>>& polygons)
{
    solver::SourceMeshSnapshot mesh;
    mesh.vertices.resize(positions.size());
    for (std::size_t index = 0U; index < positions.size(); ++index) {
        mesh.vertices[index].position = positions[index];
        mesh.vertices[index].sourceVertexId =
            static_cast<solver::SourceId>(index);
    }
    std::map<EdgeKey, std::size_t> edgeMap;
    mesh.faces.resize(polygons.size());
    for (std::size_t faceIndex = 0U; faceIndex < polygons.size(); ++faceIndex) {
        solver::SourceFace& face = mesh.faces[faceIndex];
        face.vertexIndices = polygons[faceIndex];
        face.sourceFaceId = static_cast<solver::SourceId>(faceIndex);
        solver::Vec3 center;
        solver::Vec3 area;
        for (std::size_t corner = 0U;
             corner < face.vertexIndices.size();
             ++corner) {
            const std::size_t first = face.vertexIndices[corner];
            const std::size_t second = face.vertexIndices[
                (corner + 1U) % face.vertexIndices.size()];
            center += positions[first];
            area += positions[first].cross(positions[second]);
            mesh.vertices[first].faceIndices.push_back(faceIndex);
            const EdgeKey key = edgeKey(first, second);
            auto found = edgeMap.find(key);
            if (found == edgeMap.end()) {
                solver::SourceEdge edge;
                edge.vertexIndices = {key.first, key.second};
                edge.sourceEdgeId =
                    static_cast<solver::SourceId>(mesh.edges.size());
                edge.length =
                    (positions[key.second] - positions[key.first]).length();
                found = edgeMap.emplace(key, mesh.edges.size()).first;
                mesh.edges.push_back(edge);
                mesh.vertices[key.first].edgeIndices.push_back(found->second);
                mesh.vertices[key.second].edgeIndices.push_back(found->second);
                mesh.vertices[key.first].adjacentVertexIndices.push_back(key.second);
                mesh.vertices[key.second].adjacentVertexIndices.push_back(key.first);
            }
            face.edgeIndices.push_back(found->second);
            mesh.edges[found->second].faceIndices.push_back(faceIndex);
        }
        face.center = center / static_cast<double>(face.vertexIndices.size());
        face.normal = area.normalized();
        face.geometryValid = face.normal.squaredLength() > 0.0;
        for (std::size_t corner = 1U;
             corner + 1U < face.vertexIndices.size();
             ++corner) {
            const std::size_t triangleIndex = mesh.triangles.size();
            mesh.triangles.push_back({
                {face.vertexIndices[0],
                 face.vertexIndices[corner],
                 face.vertexIndices[corner + 1U]},
                faceIndex});
            face.triangleIndices.push_back(triangleIndex);
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
    const EdgeKey wanted = edgeKey(first, second);
    for (const std::size_t edgeIndex : mesh.vertices[first].edgeIndices) {
        const solver::SourceEdge& edge = mesh.edges[edgeIndex];
        if (edgeKey(edge.vertexIndices[0], edge.vertexIndices[1]) == wanted) {
            return edgeIndex;
        }
    }
    return solver::kInvalidIndex;
}

double surfaceHeight(
    const std::string& surface,
    double x,
    double y)
{
    if (surface == "curved") {
        return 0.08 * (x * x + 0.55 * y * y);
    }
    if (surface == "chest") {
        return 1.8 * std::exp(-(x * x + 1.3 * y * y) / 10.0);
    }
    if (surface == "cloth") {
        return 0.45 * std::sin(1.1 * x) * std::cos(0.75 * y);
    }
    return 0.0;
}

solver::RemeshInput makeRadialFixture(
    std::size_t segments,
    unsigned int transitionRings,
    double targetEdgeLength,
    const std::string& surface,
    bool rotateField = false)
{
    solver::RemeshInput input;
    std::vector<solver::Vec3> positions = {
        {0.0, 0.0, surfaceHeight(surface, 0.0, 0.0)}};
    const unsigned int rings = std::max(1U, transitionRings) + 1U;
    for (unsigned int ring = 0U; ring < rings; ++ring) {
        const double radius = static_cast<double>(ring + 1U);
        for (std::size_t index = 0U; index < segments; ++index) {
            const double angle = 2.0 * kPi *
                static_cast<double>(index) /
                static_cast<double>(segments);
            const double x = radius * std::cos(angle);
            const double y = radius * std::sin(angle);
            positions.push_back({x, y, surfaceHeight(surface, x, y)});
        }
    }

    const auto vertex = [segments](unsigned int ring, std::size_t index) {
        return 1U + static_cast<std::size_t>(ring) * segments +
            index % segments;
    };
    std::vector<std::vector<std::size_t>> polygons;
    for (std::size_t index = 0U; index < segments; ++index) {
        polygons.push_back({
            0U, vertex(0U, index), vertex(0U, index + 1U)});
    }
    for (unsigned int ring = 0U; ring + 1U < rings; ++ring) {
        for (std::size_t index = 0U; index < segments; ++index) {
            polygons.push_back({
                vertex(ring, index),
                vertex(ring + 1U, index),
                vertex(ring + 1U, index + 1U),
                vertex(ring, index + 1U)});
        }
    }
    input.sourceMesh = buildMesh(positions, polygons);

    solver::RegionComponent component;
    component.componentId = 0U;
    component.transitionRingDepthByFace.assign(
        input.sourceMesh.faces.size(), -1);
    for (std::size_t faceIndex = 0U;
         faceIndex < segments;
         ++faceIndex) {
        component.coreFaceIndices.push_back(faceIndex);
        component.allFaceIndices.push_back(faceIndex);
        component.transitionRingDepthByFace[faceIndex] = 0;
    }
    for (unsigned int ring = 0U; ring + 1U < rings; ++ring) {
        for (std::size_t index = 0U; index < segments; ++index) {
            const std::size_t faceIndex =
                segments + static_cast<std::size_t>(ring) * segments + index;
            component.transitionFaceIndices.push_back(faceIndex);
            component.allFaceIndices.push_back(faceIndex);
            component.transitionRingDepthByFace[faceIndex] =
                static_cast<int>(ring + 1U);
        }
    }

    solver::OrderedBoundaryLoop boundary;
    boundary.closed = true;
    const unsigned int outerRing = rings - 1U;
    for (std::size_t index = 0U; index < segments; ++index) {
        const std::size_t first = vertex(outerRing, index);
        const std::size_t second = vertex(outerRing, index + 1U);
        const std::size_t edgeIndex =
            findEdge(input.sourceMesh, first, second);
        boundary.vertexIndices.push_back(first);
        boundary.edgeIndices.push_back(edgeIndex);
        boundary.sourceVertexIds.push_back(
            input.sourceMesh.vertices[first].sourceVertexId);
        boundary.sourceEdgeIds.push_back(
            input.sourceMesh.edges[edgeIndex].sourceEdgeId);
    }
    component.fixedBoundaryLoops.push_back(std::move(boundary));
    input.components.push_back(std::move(component));
    input.settings.topologyBlendWidth = std::max(1U, transitionRings);

    input.directionField.resize(input.sourceMesh.faces.size());
    input.densityField.resize(input.sourceMesh.faces.size());
    const solver::Vec3 worldDirection = rotateField
        ? solver::Vec3{1.0, 1.0, 0.0}.normalized()
        : solver::Vec3{1.0, 0.0, 0.0};
    for (std::size_t faceIndex = 0U;
         faceIndex < input.sourceMesh.faces.size();
         ++faceIndex) {
        const solver::Vec3 normal = input.sourceMesh.faces[faceIndex].normal;
        solver::Vec3 u =
            (worldDirection - normal * worldDirection.dot(normal)).normalized();
        if (u.squaredLength() == 0.0) {
            u = solver::Vec3{0.0, 1.0, 0.0};
        }
        input.directionField[faceIndex] = {
            normal,
            u,
            normal.cross(u).normalized(),
            faceIndex < segments ? 1.0 : 0.5,
            faceIndex < segments ? 0.0 : 0.35,
            true};
        input.densityField[faceIndex] = {
            targetEdgeLength,
            targetEdgeLength,
            1.0,
            1.0,
            false,
            true};
    }
    return input;
}

struct Fixture final
{
    std::string name;
    solver::RemeshInput input;
    bool requireTopologyReduction = false;
    bool requireDensityImprovement = false;
    bool requireDirectionNonRegression = false;
};

std::vector<Fixture> proceduralFixtures()
{
    return {
        {"plane_fine",
         makeRadialFixture(12U, 2U, 0.30, "plane")},
        {"plane_coarse",
         makeRadialFixture(16U, 2U, 2.20, "plane"),
         true, true, false},
        {"curved_fine",
         makeRadialFixture(12U, 2U, 0.35, "curved")},
        {"curved_coarse",
         makeRadialFixture(16U, 2U, 2.00, "curved"),
         true, true, false},
        {"small_region",
         makeRadialFixture(4U, 1U, 2.00, "plane")},
        {"blend_width_1",
         makeRadialFixture(12U, 1U, 1.60, "plane")},
        {"odd_boundary",
         makeRadialFixture(9U, 2U, 1.60, "curved")},
        {"dense_boundary_coarse_core",
         makeRadialFixture(24U, 3U, 3.00, "plane"),
         true, true, false},
        {"chest_like_bump",
         makeRadialFixture(18U, 3U, 1.10, "chest")},
        {"cloth_fold",
         makeRadialFixture(18U, 3U, 1.00, "cloth")},
        {"direction_rotated",
         makeRadialFixture(16U, 2U, 1.10, "plane", true),
         false, false, true},
        {"aggressive_coarse",
         makeRadialFixture(20U, 1U, 8.00, "curved")},
    };
}

bool sameDeterministicResult(
    const solver::AdaptedScaffoldResult& first,
    const solver::AdaptedScaffoldResult& second)
{
    if (first.status != second.status ||
        first.stopReason != second.stopReason ||
        first.finalSignature != second.finalSignature ||
        first.operations.size() != second.operations.size() ||
        first.after.meanDensityError != second.after.meanDensityError ||
        first.after.meanDirectionDeviationDegrees !=
            second.after.meanDirectionDeviationDegrees ||
        first.after.maximumSurfaceError !=
            second.after.maximumSurfaceError) {
        return false;
    }
    for (std::size_t index = 0U; index < first.operations.size(); ++index) {
        const solver::ScaffoldAdaptationOperation& left =
            first.operations[index];
        const solver::ScaffoldAdaptationOperation& right =
            second.operations[index];
        if (left.type != right.type ||
            left.edgeId != right.edgeId ||
            left.endpointToKeep != right.endpointToKeep ||
            left.splitParameter != right.splitParameter ||
            left.description != right.description) {
            return false;
        }
    }
    return true;
}

bool fixedBoundaryUnchanged(
    const solver::SourceTransitionScaffold& scaffold,
    const solver::AdaptedScaffoldResult& result)
{
    if (scaffold.fixedOuterBoundaryLoops.size() != 1U ||
        result.after.maximumFixedBoundaryDisplacement != 0.0) {
        return false;
    }
    const solver::ScaffoldBoundaryLoop& source =
        scaffold.fixedOuterBoundaryLoops.front();
    const solver::ScaffoldBoundaryLoop& adapted =
        result.adaptedMesh.fixedOuterBoundary();
    if (source.vertexIndices != adapted.vertexIndices ||
        source.edgeIndices != adapted.edgeIndices) {
        return false;
    }
    for (const std::size_t vertexId : adapted.vertexIndices) {
        if (vertexId >= result.adaptedMesh.vertices().size()) {
            return false;
        }
        const solver::MutableVertex& vertex =
            result.adaptedMesh.vertices()[vertexId];
        if (vertex.deleted || !vertex.fixedOuterBoundary ||
            (vertex.position - vertex.sourcePosition).squaredLength() != 0.0) {
            return false;
        }
    }
    for (const std::size_t edgeId : adapted.edgeIndices) {
        if (edgeId >= result.adaptedMesh.edges().size() ||
            result.adaptedMesh.edges()[edgeId].deleted ||
            !result.adaptedMesh.edges()[edgeId].fixedOuterBoundary) {
            return false;
        }
    }
    return true;
}

struct Totals final
{
    std::size_t fixtures = 0U;
    std::size_t components = 0U;
    std::size_t success = 0U;
    std::size_t partial = 0U;
    std::size_t failure = 0U;
    std::size_t legacyTransitionFailureFixtures = 0U;
    std::size_t legacyTransitionFailureComponents = 0U;
    std::size_t legacyFailureAdaptationValid = 0U;
    std::size_t candidates = 0U;
    std::size_t operations = 0U;
    double milliseconds = 0.0;
};

bool expectedLegacyTransitionFailure(
    const solver::RemeshCaptureRecord& capture,
    std::size_t componentId)
{
    (void)componentId;
    return capture.hasExpectedResult &&
        capture.expectedResult.status == solver::SolveStatus::Failed &&
        capture.expectedResult.failureCode ==
            solver::FailureCode::TransitionBuildFailed;
}

bool evaluateComponent(
    const std::string& label,
    const solver::RemeshInput& input,
    const solver::RegionComponent& component,
    const Fixture* expectations,
    bool legacyTransitionFailure,
    Totals& totals)
{
    ++totals.components;
    solver::SourceTransitionScaffoldExtractor extractor;
    const solver::SourceTransitionScaffold scaffold = extractor.extract(
        input.sourceMesh, component, input.settings);
    if (!scaffold.success()) {
        std::cerr << "[FAIL] " << label << " R4 scaffold: "
                  << scaffold.diagnosticMessage << "\n";
        ++totals.failure;
        return false;
    }
    const std::uint64_t scaffoldSignature =
        solver::sourceTransitionScaffoldSignature(scaffold);
    std::string diagnostic;
    const solver::LocalMutablePatchMesh mutablePatch =
        solver::LocalMutablePatchMesh::fromScaffold(
            scaffold,
            input.settings.topologyBlendWidth,
            &diagnostic);
    if (!mutablePatch.valid(&diagnostic)) {
        std::cerr << "[FAIL] " << label << " R5 copy: "
                  << diagnostic << "\n";
        ++totals.failure;
        return false;
    }

    solver::ScaffoldAdaptationSettings adaptationSettings;
    adaptationSettings.maxOperations =
        expectations != nullptr ? 20U : 10U;
    adaptationSettings.maxPasses =
        adaptationSettings.maxOperations;
    adaptationSettings.maximumCandidatesPerPass = 32U;
    solver::ScaffoldAdaptationSolver adaptationSolver;
    const solver::AdaptedScaffoldResult result = adaptationSolver.adapt(
        scaffold,
        mutablePatch,
        input.sourceMesh,
        component,
        input.directionField,
        input.densityField,
        input.settings,
        adaptationSettings);
    totals.candidates += result.candidates.generated;
    totals.operations += result.operations.size();
    totals.milliseconds += result.adaptationMilliseconds;
    if (result.status == solver::ScaffoldAdaptationStatus::Success) {
        ++totals.success;
    } else if (result.status == solver::ScaffoldAdaptationStatus::Partial) {
        ++totals.partial;
    } else {
        ++totals.failure;
    }
    if (legacyTransitionFailure) {
        ++totals.legacyTransitionFailureComponents;
        if (result.validResult()) {
            ++totals.legacyFailureAdaptationValid;
        }
    }

    bool ok = result.validResult() &&
        result.adaptedMesh.valid(&diagnostic) &&
        fixedBoundaryUnchanged(scaffold, result) &&
        result.after.nonManifoldCount == 0U &&
        result.after.zeroAreaCount == 0U &&
        solver::sourceTransitionScaffoldSignature(scaffold) ==
            scaffoldSignature;
    if (expectations != nullptr) {
        if (expectations->requireTopologyReduction &&
            !(result.after.vertexCount < result.before.vertexCount)) {
            std::cerr << "[FAIL] " << label
                      << " did not reduce coarse topology\n";
            ok = false;
        }
        if (expectations->requireDensityImprovement &&
            !(result.after.meanDensityError <
              result.before.meanDensityError)) {
            std::cerr << "[FAIL] " << label
                      << " did not improve density error\n";
            ok = false;
        }
        if (expectations->requireDirectionNonRegression &&
            result.after.meanDirectionDeviationDegrees >
                result.before.meanDirectionDeviationDegrees + 1.0e-9) {
            std::cerr << "[FAIL] " << label
                      << " worsened 4-RoSy direction alignment\n";
            ok = false;
        }
    }

    for (int repeat = 1; repeat < 5; ++repeat) {
        const solver::AdaptedScaffoldResult repeated =
            adaptationSolver.adapt(
                scaffold,
                mutablePatch,
                input.sourceMesh,
                component,
                input.directionField,
                input.densityField,
                input.settings,
                adaptationSettings);
        if (!sameDeterministicResult(result, repeated)) {
            std::cerr << "[FAIL] " << label
                      << " adaptation is nondeterministic at repeat "
                      << repeat + 1 << "\n";
            ok = false;
        }
    }

    std::cout << std::left << std::setw(67) << label
              << " status=" << std::setw(7)
              << solver::scaffoldAdaptationStatusName(result.status)
              << " stop=" << std::setw(25)
              << solver::scaffoldAdaptationStopReasonName(result.stopReason)
              << " v/e/f=" << result.before.vertexCount << "/"
              << result.before.edgeCount << "/" << result.before.faceCount
              << "->" << result.after.vertexCount << "/"
              << result.after.edgeCount << "/" << result.after.faceCount
              << " interface=" << result.before.innerInterfaceVertexCount
              << "->" << result.after.innerInterfaceVertexCount
              << "~" << result.after.approximateDesiredInterfaceCount
              << " density=" << std::fixed << std::setprecision(4)
              << result.before.meanDensityError << "->"
              << result.after.meanDensityError
              << " direction=" << std::setprecision(2)
              << result.before.meanDirectionDeviationDegrees << "->"
              << result.after.meanDirectionDeviationDegrees
              << " surfaceMax=" << std::setprecision(6)
              << result.after.maximumSurfaceError
              << " ops=" << result.operations.size()
              << " candidates=" << result.candidates.generated
              << " ms=" << std::setprecision(3)
              << result.adaptationMilliseconds << "\n";
    if (!ok) {
        std::cerr << "[FAIL] " << label << ": "
                  << result.diagnosticMessage << "; validation="
                  << diagnostic << "\n";
    }
    return ok;
}

}  // namespace

int main(int argc, char** argv)
{
    bool valid = true;
    Totals procedural;
    const std::vector<Fixture> fixtures = proceduralFixtures();
    procedural.fixtures = fixtures.size();
    std::cout << "[R6] Procedural field-aware scaffold adaptation matrix\n";
    for (const Fixture& fixture : fixtures) {
        for (const solver::RegionComponent& component :
             fixture.input.components) {
            valid = evaluateComponent(
                        "procedural/" + fixture.name,
                        fixture.input,
                        component,
                        &fixture,
                        false,
                        procedural) &&
                valid;
        }
    }

    const std::filesystem::path captureDirectory =
        argc > 1 ? std::filesystem::path(argv[1])
                 : std::filesystem::path("tests/fixtures/captured");
    std::vector<std::filesystem::path> paths;
    if (std::filesystem::exists(captureDirectory)) {
        for (const std::filesystem::directory_entry& entry :
             std::filesystem::directory_iterator(captureDirectory)) {
            if (entry.is_regular_file() &&
                entry.path().extension() == ".drinput") {
                paths.push_back(entry.path());
            }
        }
    }
    std::sort(paths.begin(), paths.end());
    if (paths.size() != 12U) {
        std::cerr << "[FAIL] expected 12 captured fixtures, found "
                  << paths.size() << "\n";
        valid = false;
    }

    Totals captured;
    captured.fixtures = paths.size();
    std::cout << "[R6] Captured Maya field-aware scaffold adaptation matrix\n";
    for (const std::filesystem::path& path : paths) {
        solver::RemeshCaptureRecord capture;
        std::string diagnostic;
        if (!solver::loadRemeshCapture(
                path.string(), capture, diagnostic)) {
            std::cerr << "[FAIL] capture load "
                      << path.filename().string() << ": "
                      << diagnostic << "\n";
            valid = false;
            continue;
        }
        const std::uint64_t inputSignature =
            solver::remeshInputSignature(capture.input);
        bool fixtureHasLegacyTransitionFailure = false;
        for (const solver::RegionComponent& component :
             capture.input.components) {
            const bool legacyFailure =
                expectedLegacyTransitionFailure(
                    capture, component.componentId);
            fixtureHasLegacyTransitionFailure =
                fixtureHasLegacyTransitionFailure || legacyFailure;
            valid = evaluateComponent(
                        "captured/" + path.filename().string() +
                            "/c" + std::to_string(component.componentId),
                        capture.input,
                        component,
                        nullptr,
                        legacyFailure,
                        captured) &&
                valid;
        }
        if (fixtureHasLegacyTransitionFailure) {
            ++captured.legacyTransitionFailureFixtures;
        }
        if (solver::remeshInputSignature(capture.input) !=
            inputSignature) {
            std::cerr << "[FAIL] immutable capture input changed: "
                      << path.filename().string() << "\n";
            valid = false;
        }
    }
    if (captured.components != 20U) {
        std::cerr << "[FAIL] expected 20 captured components, found "
                  << captured.components << "\n";
        valid = false;
    }
    if (captured.legacyTransitionFailureFixtures != 7U) {
        std::cerr << "[FAIL] expected 7 captured legacy Transition failures, found "
                  << captured.legacyTransitionFailureFixtures << " fixtures\n";
        valid = false;
    }
    if (captured.legacyFailureAdaptationValid !=
        captured.legacyTransitionFailureComponents) {
        std::cerr << "[FAIL] an R6 adaptation failed only because the legacy "
                     "Transition path failed\n";
        valid = false;
    }

    const auto printTotals = [](const char* label, const Totals& totals) {
        std::cout << "[R6] " << label
                  << " fixtures=" << totals.fixtures
                  << " components=" << totals.components
                  << " success=" << totals.success
                  << " partial=" << totals.partial
                  << " failure=" << totals.failure
                  << " candidates=" << totals.candidates
                  << " operations=" << totals.operations
                  << " adaptationMs=" << std::fixed
                  << std::setprecision(3) << totals.milliseconds
                  << "\n";
    };
    printTotals("procedural", procedural);
    printTotals("captured", captured);
    std::cout << "[R6] captured legacy TransitionBuildFailed="
              << captured.legacyTransitionFailureFixtures
              << " fixtures/" << captured.legacyTransitionFailureComponents
              << " components R6-valid="
              << captured.legacyFailureAdaptationValid << "\n";
    std::cout << "[R6] deterministicRepeats=5 fixedBoundaryDisplacement=0 "
                 "productionPathConnected=no dissolveAutomatic=no\n";
    return valid ? 0 : 1;
}
