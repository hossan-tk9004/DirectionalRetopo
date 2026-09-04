#include "Solver/CoreInterfaceJoinSolver.h"
#include "Solver/ExperimentalCoreRemeshGenerator.h"
#include "Solver/RemeshCapture.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace solver = directional_retopo::solver;
namespace {

constexpr double kPi = 3.14159265358979323846;
using EdgeKey = std::pair<std::size_t, std::size_t>;

class ScopedCerrSilencer final
{
public:
    ScopedCerrSilencer()
        : previous_(std::cerr.rdbuf(sink_.rdbuf()))
    {
    }

    ~ScopedCerrSilencer()
    {
        std::cerr.rdbuf(previous_);
    }

    ScopedCerrSilencer(const ScopedCerrSilencer&) = delete;
    ScopedCerrSilencer& operator=(const ScopedCerrSilencer&) = delete;

private:
    std::ostringstream sink_;
    std::streambuf* previous_ = nullptr;
};

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
        for (std::size_t corner = 0U; corner < face.vertexIndices.size(); ++corner) {
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
                edge.sourceEdgeId = static_cast<solver::SourceId>(mesh.edges.size());
                edge.length = (positions[key.second] - positions[key.first]).length();
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
        for (std::size_t corner = 1U; corner + 1U < face.vertexIndices.size(); ++corner) {
            const std::size_t triangleIndex = mesh.triangles.size();
            mesh.triangles.push_back({
                {face.vertexIndices[0], face.vertexIndices[corner],
                 face.vertexIndices[corner + 1U]}, faceIndex});
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

double height(const std::string& surface, double x, double y)
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

solver::RemeshInput radialFixture(
    std::size_t segments,
    unsigned int transitionRings,
    double targetEdgeLength,
    const std::string& surface)
{
    solver::RemeshInput input;
    std::vector<solver::Vec3> positions = {
        {0.0, 0.0, height(surface, 0.0, 0.0)}};
    const unsigned int rings = std::max(1U, transitionRings) + 1U;
    for (unsigned int ring = 0U; ring < rings; ++ring) {
        const double radius = static_cast<double>(ring + 1U);
        for (std::size_t index = 0U; index < segments; ++index) {
            const double angle = 2.0 * kPi * static_cast<double>(index) /
                static_cast<double>(segments);
            const double x = radius * std::cos(angle);
            const double y = radius * std::sin(angle);
            positions.push_back({x, y, height(surface, x, y)});
        }
    }
    const auto vertex = [segments](unsigned int ring, std::size_t index) {
        return 1U + static_cast<std::size_t>(ring) * segments + index % segments;
    };
    std::vector<std::vector<std::size_t>> polygons;
    for (std::size_t index = 0U; index < segments; ++index) {
        polygons.push_back({0U, vertex(0U, index), vertex(0U, index + 1U)});
    }
    for (unsigned int ring = 0U; ring + 1U < rings; ++ring) {
        for (std::size_t index = 0U; index < segments; ++index) {
            polygons.push_back({vertex(ring, index), vertex(ring + 1U, index),
                                vertex(ring + 1U, index + 1U),
                                vertex(ring, index + 1U)});
        }
    }
    input.sourceMesh = buildMesh(positions, polygons);
    solver::RegionComponent component;
    component.componentId = 0U;
    component.transitionRingDepthByFace.assign(input.sourceMesh.faces.size(), -1);
    for (std::size_t face = 0U; face < segments; ++face) {
        component.coreFaceIndices.push_back(face);
        component.allFaceIndices.push_back(face);
        component.transitionRingDepthByFace[face] = 0;
    }
    for (unsigned int ring = 0U; ring + 1U < rings; ++ring) {
        for (std::size_t index = 0U; index < segments; ++index) {
            const std::size_t face = segments +
                static_cast<std::size_t>(ring) * segments + index;
            component.transitionFaceIndices.push_back(face);
            component.allFaceIndices.push_back(face);
            component.transitionRingDepthByFace[face] = static_cast<int>(ring + 1U);
        }
    }
    solver::OrderedBoundaryLoop boundary;
    boundary.closed = true;
    const unsigned int outerRing = rings - 1U;
    for (std::size_t index = 0U; index < segments; ++index) {
        const std::size_t first = vertex(outerRing, index);
        const std::size_t second = vertex(outerRing, index + 1U);
        const std::size_t edge = findEdge(input.sourceMesh, first, second);
        boundary.vertexIndices.push_back(first);
        boundary.edgeIndices.push_back(edge);
        boundary.sourceVertexIds.push_back(input.sourceMesh.vertices[first].sourceVertexId);
        boundary.sourceEdgeIds.push_back(input.sourceMesh.edges[edge].sourceEdgeId);
    }
    component.fixedBoundaryLoops.push_back(std::move(boundary));
    input.components.push_back(std::move(component));
    input.settings.topologyBlendWidth = std::max(1U, transitionRings);
    input.directionField.resize(input.sourceMesh.faces.size());
    input.densityField.resize(input.sourceMesh.faces.size());
    for (std::size_t face = 0U; face < input.sourceMesh.faces.size(); ++face) {
        const solver::Vec3 normal = input.sourceMesh.faces[face].normal;
        solver::Vec3 u = (solver::Vec3{1.0, 0.0, 0.0} -
            normal * normal.x).normalized();
        if (u.squaredLength() == 0.0) {
            u = {0.0, 1.0, 0.0};
        }
        input.directionField[face] = {
            normal, u, normal.cross(u).normalized(),
            face < segments ? 1.0 : 0.5,
            face < segments ? 0.0 : 0.35, true};
        input.densityField[face] = {
            targetEdgeLength, targetEdgeLength, 1.0, 1.0, false, true};
    }
    return input;
}

solver::SurfacePointMapping mappingFor(
    const solver::RemeshInput& input,
    std::size_t faceIndex,
    const solver::Vec3& barycentric)
{
    solver::SurfacePointMapping mapping;
    const solver::SourceFace& face = input.sourceMesh.faces[faceIndex];
    const std::size_t triangleIndex = face.triangleIndices.front();
    const solver::SourceTriangle& triangle = input.sourceMesh.triangles[triangleIndex];
    const solver::Vec3 a = input.sourceMesh.vertices[triangle.vertexIndices[0]].position;
    const solver::Vec3 b = input.sourceMesh.vertices[triangle.vertexIndices[1]].position;
    const solver::Vec3 c = input.sourceMesh.vertices[triangle.vertexIndices[2]].position;
    mapping.sourceTriangleIndex = triangleIndex;
    mapping.sourceFaceId = face.sourceFaceId;
    mapping.barycentric = barycentric;
    mapping.sourcePosition = a * barycentric.x + b * barycentric.y + c * barycentric.z;
    mapping.sourceNormal = (b - a).cross(c - a).normalized();
    mapping.surfaceDistance = 0.0;
    mapping.valid = true;
    return mapping;
}

solver::CoreRemeshResult syntheticCore(
    const solver::RemeshInput& input,
    const solver::RegionComponent& component,
    std::size_t boundaryCount)
{
    solver::CoreRemeshResult core;
    core.componentId = component.componentId;
    boundaryCount = std::max<std::size_t>(3U, boundaryCount);
    const std::size_t sourceSegments = component.coreFaceIndices.size();
    for (std::size_t index = 0U; index < boundaryCount; ++index) {
        const double phase = static_cast<double>(index) *
            static_cast<double>(sourceSegments) / static_cast<double>(boundaryCount);
        const std::size_t segment = static_cast<std::size_t>(std::floor(phase)) %
            sourceSegments;
        const double fraction = phase - std::floor(phase);
        const double radial = 0.55;
        const solver::Vec3 barycentric{
            1.0 - radial, radial * (1.0 - fraction), radial * fraction};
        const std::size_t faceIndex = component.coreFaceIndices[segment];
        const solver::SurfacePointMapping mapping = mappingFor(input, faceIndex, barycentric);
        core.vertices.push_back(mapping.sourcePosition);
        core.rawVertices.push_back(mapping.sourcePosition);
        core.sourceMappings.push_back(mapping);
    }
    const std::size_t centerId = core.vertices.size();
    const solver::SurfacePointMapping center = mappingFor(
        input, component.coreFaceIndices.front(), {1.0, 0.0, 0.0});
    core.vertices.push_back(center.sourcePosition);
    core.rawVertices.push_back(center.sourcePosition);
    core.sourceMappings.push_back(center);
    for (std::size_t index = 0U; index < boundaryCount; ++index) {
        solver::ResultPolygon polygon;
        polygon.vertexIndices = {centerId, index, (index + 1U) % boundaryCount};
        polygon.type = solver::PolygonType::Triangle;
        core.polygons.push_back(std::move(polygon));
        core.sourceFaceIndices.push_back(
            component.coreFaceIndices[index * sourceSegments / boundaryCount]);
    }
    core.connectedComponentCount = 1U;
    core.triangleCount = boundaryCount;
    core.boundary.status = solver::CoreBoundaryStatus::Success;
    core.boundary.closed = true;
    core.boundary.boundaryLoopCount = 1U;
    core.boundary.orderedVertexIds.resize(boundaryCount);
    std::vector<double> cumulative(boundaryCount, 0.0);
    for (std::size_t index = 0U; index < boundaryCount; ++index) {
        core.boundary.orderedVertexIds[index] = index;
        if (index + 1U < boundaryCount) {
            cumulative[index + 1U] = cumulative[index] +
                (core.vertices[index + 1U] - core.vertices[index]).length();
        }
    }
    core.boundary.totalArcLength = cumulative.back() +
        (core.vertices.front() - core.vertices.back()).length();
    for (std::size_t index = 0U; index < boundaryCount; ++index) {
        solver::CoreBoundaryVertexDescriptor vertex;
        vertex.coreVertexId = index;
        vertex.orderedBoundaryIndex = index;
        vertex.position = core.vertices[index];
        vertex.tangent = (core.vertices[(index + 1U) % boundaryCount] -
            core.vertices[(index + boundaryCount - 1U) % boundaryCount]).normalized();
        vertex.normalizedArcLength = cumulative[index] / core.boundary.totalArcLength;
        vertex.surface = core.sourceMappings[index];
        core.boundary.vertices.push_back(vertex);
    }
    core.boundary.diagnosticMessage = "Synthetic one-loop conformed Core.";
    core.status = solver::CoreGenerationStatus::Success;
    core.diagnosticMessage = "Synthetic portable Core generated.";
    core.signature = solver::coreRemeshResultSignature(core);
    return core;
}

struct Prepared final
{
    solver::SourceTransitionScaffold scaffold;
    solver::AdaptedScaffoldResult adaptation;
};

Prepared prepare(
    const solver::RemeshInput& input,
    const solver::RegionComponent& component)
{
    Prepared result;
    solver::SourceTransitionScaffoldExtractor extractor;
    result.scaffold = extractor.extract(input.sourceMesh, component, input.settings);
    std::string diagnostic;
    const solver::LocalMutablePatchMesh mutablePatch =
        solver::LocalMutablePatchMesh::fromScaffold(
            result.scaffold, input.settings.topologyBlendWidth, &diagnostic);
    solver::ScaffoldAdaptationSettings settings;
    settings.maxOperations = 20U;
    settings.maxPasses = 20U;
    settings.maximumCandidatesPerPass = 32U;
    solver::ScaffoldAdaptationSolver adapter;
    result.adaptation = adapter.adapt(
        result.scaffold, mutablePatch, input.sourceMesh, component,
        input.directionField, input.densityField, input.settings, settings);
    return result;
}

bool sameResult(
    const solver::CoreInterfaceJoinResult& first,
    const solver::CoreInterfaceJoinResult& second)
{
    if (first.reconciliation.operations.size() !=
        second.reconciliation.operations.size()) {
        return false;
    }
    for (std::size_t index = 0U;
         index < first.reconciliation.operations.size(); ++index) {
        const solver::MutableOperationRecord& left =
            first.reconciliation.operations[index];
        const solver::MutableOperationRecord& right =
            second.reconciliation.operations[index];
        if (left.id != right.id || left.type != right.type ||
            left.createdVertexIds != right.createdVertexIds ||
            left.createdEdgeIds != right.createdEdgeIds ||
            left.createdFaceIds != right.createdFaceIds ||
            left.deletedVertexIds != right.deletedVertexIds ||
            left.deletedEdgeIds != right.deletedEdgeIds ||
            left.deletedFaceIds != right.deletedFaceIds ||
            left.modifiedVertexIds != right.modifiedVertexIds ||
            left.modifiedEdgeIds != right.modifiedEdgeIds ||
            left.modifiedFaceIds != right.modifiedFaceIds) {
            return false;
        }
    }
    return first.status == second.status &&
        first.signature == second.signature &&
        first.join.correspondence.coreSeamOffset ==
            second.join.correspondence.coreSeamOffset &&
        first.join.correspondence.coreOrderReversed ==
            second.join.correspondence.coreOrderReversed &&
        first.join.triangleCount == second.join.triangleCount &&
        first.join.quadCount == second.join.quadCount &&
        first.combined.metrics.nonManifoldEdgeCount ==
            second.combined.metrics.nonManifoldEdgeCount;
}

bool hardInvariants(const solver::CoreInterfaceJoinResult& result)
{
    return result.usable() && result.combined.success &&
        result.combined.metrics.maximumFixedBoundaryDisplacement == 0.0 &&
        result.combined.metrics.nonManifoldEdgeCount == 0U &&
        result.combined.metrics.zeroAreaPolygonCount == 0U &&
        result.combined.metrics.boundaryCrossingCount == 0U &&
        result.combined.metrics.joinNGonCount == 0U &&
        result.combined.metrics.outerBoundaryLoopCount == 1U;
}

bool runProcedural(
    const std::string& name,
    solver::RemeshInput input,
    int coreCountDelta,
    bool allowStructuredFailure,
    bool requireAllQuad)
{
    const solver::RegionComponent& component = input.components.front();
    const Prepared prepared = prepare(input, component);
    if (!prepared.scaffold.success() || !prepared.adaptation.validResult()) {
        std::cerr << "[FAIL] " << name << " R6 preparation failed\n";
        return false;
    }
    const std::size_t interfaceCount =
        prepared.adaptation.adaptedMesh.orderedInnerInterfaceVertices().size();
    const std::size_t requestedCount = static_cast<std::size_t>(std::max(
        3, static_cast<int>(interfaceCount) + coreCountDelta));
    const solver::CoreRemeshResult core = syntheticCore(input, component, requestedCount);
    solver::CoreInterfaceJoinSettings settings;
    settings.maximumReconciliationOperations = 64U;
    solver::CoreInterfaceJoinSolver joiner;
    const solver::CoreInterfaceJoinResult result = joiner.join(
        input, component, prepared.scaffold, prepared.adaptation, core, settings);
    bool valid = allowStructuredFailure ?
        result.status != solver::CoreJoinStatus::Success || hardInvariants(result) :
        hardInvariants(result);
    if (requireAllQuad && result.usable() && result.join.triangleCount != 0U) {
        valid = false;
    }
    for (int repeat = 1; repeat < 5; ++repeat) {
        const solver::CoreInterfaceJoinResult repeated = joiner.join(
            input, component, prepared.scaffold, prepared.adaptation, core, settings);
        if (!sameResult(result, repeated)) {
            std::cerr << "[FAIL] " << name << " nondeterministic repeat "
                      << repeat + 1 << "\n";
            valid = false;
        }
    }
    std::cout << std::left << std::setw(34) << name
              << " R6=" << solver::scaffoldAdaptationStatusName(prepared.adaptation.status)
              << " S=" << result.reconciliation.scaffoldCountBefore << "->"
              << result.reconciliation.scaffoldCountAfter
              << " C=" << core.boundary.vertices.size()
              << " ops(s/c)=" << result.reconciliation.splitCount << "/"
              << result.reconciliation.collapseCount
              << " join(Q/T)=" << result.join.quadCount << "/"
              << result.join.triangleCount
              << " status=" << solver::coreJoinStatusName(result.status)
              << " surfaceMax=" << std::fixed << std::setprecision(6)
              << result.join.maximumSurfaceError << "\n";
    if (!valid) {
        std::cerr << "[FAIL] " << name << ": " << result.diagnosticMessage
                  << "; join=" << result.join.diagnosticMessage << "\n";
    }
    return valid;
}

struct CapturedTotals final
{
    std::size_t fixtures = 0U;
    std::size_t components = 0U;
    std::size_t success = 0U;
    std::size_t partial = 0U;
    std::size_t failure = 0U;
    std::size_t legacyFailureFixtures = 0U;
    std::size_t legacyFailureComponents = 0U;
    std::size_t legacyFailureUsable = 0U;
    std::size_t successControlComponents = 0U;
    std::size_t successControlUsable = 0U;
    std::size_t r6SuccessToUsable = 0U;
    std::size_t r6PartialToUsable = 0U;
    double coreMilliseconds = 0.0;
    double coreBoundaryMappingMilliseconds = 0.0;
    double correspondenceMilliseconds = 0.0;
    double totalR7Milliseconds = 0.0;
    double reconciliationMilliseconds = 0.0;
    double joinMilliseconds = 0.0;
    bool capturedDeterminismVerified = false;
};

bool runCaptured(
    const std::filesystem::path& directory,
    CapturedTotals& totals)
{
    std::vector<std::filesystem::path> paths;
    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".drinput") {
            paths.push_back(entry.path());
        }
    }
    std::sort(paths.begin(), paths.end());
    totals.fixtures = paths.size();
    bool valid = paths.size() == 12U;
    solver::ExperimentalCoreRemeshGenerator generator;
    solver::CoreInterfaceJoinSolver joiner;
    for (const std::filesystem::path& path : paths) {
        solver::RemeshCaptureRecord capture;
        std::string diagnostic;
        if (!solver::loadRemeshCapture(path.string(), capture, diagnostic)) {
            std::cerr << "[FAIL] " << path.filename().string() << ": "
                      << diagnostic << "\n";
            valid = false;
            continue;
        }
        const bool legacyFailure = capture.hasExpectedResult &&
            capture.expectedResult.status == solver::SolveStatus::Failed &&
            capture.expectedResult.failureCode == solver::FailureCode::TransitionBuildFailed;
        if (legacyFailure) {
            ++totals.legacyFailureFixtures;
        }
        for (const solver::RegionComponent& component : capture.input.components) {
            ++totals.components;
            const Prepared prepared = prepare(capture.input, component);
            if (!prepared.scaffold.success() || !prepared.adaptation.validResult()) {
                std::cerr << "[FAIL] captured R6 preparation "
                          << path.filename().string() << "/c" << component.componentId << "\n";
                valid = false;
                continue;
            }
            solver::CoreRemeshResult core;
            {
                ScopedCerrSilencer silenceUpstreamDiagnostics;
                core = generator.generate(capture.input, component);
            }
            totals.coreMilliseconds += core.timings.totalMilliseconds;
            totals.coreBoundaryMappingMilliseconds +=
                core.timings.boundaryMappingMilliseconds;
            solver::CoreInterfaceJoinResult result;
            if (core.success()) {
                result = joiner.join(capture.input, component, prepared.scaffold,
                                     prepared.adaptation, core);
            } else {
                result.status = core.status == solver::CoreGenerationStatus::CoreBoundaryInvalid
                    ? solver::CoreJoinStatus::CoreBoundaryInvalid
                    : solver::CoreJoinStatus::CoreGenerationFailed;
                result.core = core;
                result.diagnosticMessage = core.diagnosticMessage;
            }
            totals.reconciliationMilliseconds += result.timings.reconciliationMilliseconds;
            totals.correspondenceMilliseconds +=
                result.timings.correspondenceMilliseconds;
            totals.totalR7Milliseconds += core.timings.totalMilliseconds + result.timings.totalMilliseconds;
            totals.joinMilliseconds += result.timings.joinMilliseconds;
            if (result.status == solver::CoreJoinStatus::Success) {
                ++totals.success;
            } else if (result.status == solver::CoreJoinStatus::Partial) {
                ++totals.partial;
            } else {
                ++totals.failure;
            }
            const bool usable = hardInvariants(result);
            if (legacyFailure) {
                ++totals.legacyFailureComponents;
                totals.legacyFailureUsable += usable ? 1U : 0U;
            } else {
                ++totals.successControlComponents;
                totals.successControlUsable += usable ? 1U : 0U;
            }
            if (prepared.adaptation.status == solver::ScaffoldAdaptationStatus::Success) {
                totals.r6SuccessToUsable += usable ? 1U : 0U;
            } else {
                totals.r6PartialToUsable += usable ? 1U : 0U;
            }
            if (result.usable() && !usable) {
                valid = false;
            }
            if (core.success() && result.usable() &&
                !totals.capturedDeterminismVerified) {
                for (int repeat = 1; repeat < 5; ++repeat) {
                    const solver::CoreInterfaceJoinResult repeated = joiner.join(
                        capture.input, component, prepared.scaffold,
                        prepared.adaptation, core);
                    if (!sameResult(result, repeated)) {
                        std::cerr << "[FAIL] captured nondeterminism "
                                  << path.filename().string() << "/c"
                                  << component.componentId << " repeat " << repeat + 1 << "\n";
                        valid = false;
                    }
                }
                totals.capturedDeterminismVerified = true;
            }
            std::cout << "captured/" << path.filename().string() << "/c"
                      << component.componentId
                      << " R6=" << solver::scaffoldAdaptationStatusName(prepared.adaptation.status)
                      << " Core=" << solver::coreGenerationStatusName(core.status)
                      << " R6I=" << prepared.adaptation.before.innerInterfaceVertexCount
                      << "->" << prepared.adaptation.after.innerInterfaceVertexCount
                      << " R7I=" << result.reconciliation.scaffoldCountAfter
                      << " loops=" << core.boundary.boundaryLoopCount
                      << " coreV/P/B=" << core.vertices.size() << "/"
                      << core.polygons.size() << "/" << core.boundary.vertices.size()
                      << " R7=" << solver::coreJoinStatusName(result.status)
                      << " S=" << result.reconciliation.scaffoldCountBefore << "->"
                      << result.reconciliation.scaffoldCountAfter
                      << " Q/T=" << result.join.quadCount << "/"
                      << result.join.triangleCount
                      << " fixed=" << result.combined.metrics.maximumFixedBoundaryDisplacement
                      << " diagnostic=" << result.diagnosticMessage << "\n";
        }
    }
    valid = valid && totals.components == 20U &&
        totals.legacyFailureFixtures == 7U && totals.capturedDeterminismVerified;
    return valid;
}

}  // namespace

int main(int argc, char** argv)
{
    bool valid = true;
    std::cout << "[R7] Procedural Adapted Scaffold <-> Core Join matrix\n";
    valid = runProcedural("plane_fine/equal", radialFixture(12U, 2U, 0.30, "plane"), 0, false, true) && valid;
    valid = runProcedural("plane_coarse/small_diff", radialFixture(16U, 2U, 2.20, "plane"), -2, false, false) && valid;
    valid = runProcedural("curved_fine", radialFixture(12U, 2U, 0.35, "curved"), 0, false, true) && valid;
    valid = runProcedural("curved_coarse", radialFixture(16U, 2U, 2.00, "curved"), -2, false, false) && valid;
    valid = runProcedural("small_region", radialFixture(4U, 1U, 2.00, "plane"), -1, false, false) && valid;
    valid = runProcedural("blend_width_1", radialFixture(12U, 1U, 1.60, "plane"), -1, false, false) && valid;
    valid = runProcedural("odd_boundary", radialFixture(9U, 2U, 1.60, "curved"), -1, false, false) && valid;
    valid = runProcedural("dense_boundary_coarse_core", radialFixture(24U, 3U, 3.00, "plane"), -4, false, false) && valid;
    valid = runProcedural("chest_like_bump", radialFixture(18U, 3U, 1.10, "chest"), -2, false, false) && valid;
    valid = runProcedural("cloth_fold", radialFixture(18U, 3U, 1.00, "cloth"), -2, false, false) && valid;
    valid = runProcedural("large_count_difference", radialFixture(40U, 2U, 4.00, "plane"), -30, true, false) && valid;

    const std::filesystem::path captures = argc > 1
        ? std::filesystem::path(argv[1])
        : std::filesystem::path("tests/fixtures/captured");
    CapturedTotals totals;
    valid = runCaptured(captures, totals) && valid;
    std::cout << "[R7] captured fixtures=" << totals.fixtures
              << " components=" << totals.components
              << " success=" << totals.success
              << " partial=" << totals.partial
              << " failure=" << totals.failure
              << " legacyFailure=" << totals.legacyFailureUsable << "/"
              << totals.legacyFailureComponents
              << " controls=" << totals.successControlUsable << "/"
              << totals.successControlComponents
              << " R6SuccessUsable=" << totals.r6SuccessToUsable
              << " R6PartialUsable=" << totals.r6PartialToUsable
              << " coreMs=" << std::fixed << std::setprecision(3)
              << totals.coreMilliseconds
              << " coreBoundaryMapMs="
              << totals.coreBoundaryMappingMilliseconds
              << " reconcileMs=" << totals.reconciliationMilliseconds
              << " joinMs=" << totals.joinMilliseconds
              << " correspondenceMs=" << totals.correspondenceMilliseconds
              << " totalR7Ms=" << totals.totalR7Milliseconds
              << " capturedDeterminism="
              << (totals.capturedDeterminismVerified ? "5x-pass" : "missing") << "\n";
    std::cout << "[R7] deterministicRepeats=5 productionPathConnected=no "
                 "fixedBoundaryExpected=0 joinNGonExpected=0\n";
    return valid ? 0 : 1;
}
