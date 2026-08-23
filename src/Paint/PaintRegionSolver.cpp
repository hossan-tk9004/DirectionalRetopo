#include "Paint/PaintRegionSolver.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace directional_retopo {
namespace {

constexpr double kDistanceComparisonEpsilon = 1.0e-12;

struct VertexQueueEntry final
{
    double normalizedDistance = 0.0;
    double sourceRadius = 1.0;
    int vertexId = -1;
};

struct FartherVertexFirst final
{
    bool operator()(const VertexQueueEntry& left, const VertexQueueEntry& right) const
    {
        return left.normalizedDistance > right.normalizedDistance;
    }
};

float evaluateInfluence(double normalizedDistance, PaintFalloff falloff)
{
    switch (falloff) {
    case PaintFalloff::Linear:
        return static_cast<float>(std::clamp(1.0 - normalizedDistance, 0.0, 1.0));
    }
    return 0.0F;
}

}  // namespace

const PaintRegionSolverSettings& PaintRegionSolver::settings() const noexcept
{
    return settings_;
}

void PaintRegionSolver::setSettings(const PaintRegionSolverSettings& settings) noexcept
{
    settings_ = settings;
    settings_.transitionRings = std::clamp(
        settings_.transitionRings,
        PaintRegionSolverSettings::kMinimumTransitionRings,
        PaintRegionSolverSettings::kMaximumTransitionRings);
    settings_.coreInfluenceThreshold =
        std::clamp(settings_.coreInfluenceThreshold, 0.0F, 1.0F);
    settings_.minimumUsableRadius = std::max(settings_.minimumUsableRadius, 0.0);
}

bool PaintRegionSolver::solve(
    const StrokeData& processedStroke,
    const MeshTopologyCache& topology,
    PaintRegionData& output,
    PaintRegionSolveMetrics* metrics) const
{
    output.clear();
    PaintRegionSolveMetrics localMetrics;
    const std::vector<MPoint>& positions = topology.worldVertexPositions();
    const std::vector<MeshEdgeTopology>& edges = topology.edges();
    const std::vector<MeshFaceTopology>& faces = topology.faces();
    const std::vector<std::vector<int>>& vertexEdges = topology.vertexEdgeIds();
    if (processedStroke.empty() || positions.empty() || faces.empty() ||
        vertexEdges.size() != positions.size()) {
        if (metrics != nullptr) {
            *metrics = localMetrics;
        }
        return false;
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> normalizedDistances(positions.size(), infinity);
    std::vector<double> winningSourceRadii(positions.size(), 0.0);
    std::vector<unsigned char> directHitFaces(faces.size(), 0U);
    std::priority_queue<
        VertexQueueEntry,
        std::vector<VertexQueueEntry>,
        FartherVertexFirst>
        pendingVertices;

    for (const StrokeSample& sample : processedStroke.samples()) {
        if (sample.faceId < 0 || static_cast<std::size_t>(sample.faceId) >= faces.size() ||
            !std::isfinite(sample.radius) || sample.radius <= settings_.minimumUsableRadius) {
            continue;
        }
        ++localMetrics.validSampleCount;
        directHitFaces[static_cast<std::size_t>(sample.faceId)] = 1U;

        const double sourceRadius = sample.radius;
        for (const int vertexId : faces[static_cast<std::size_t>(sample.faceId)].vertexIds) {
            if (vertexId < 0 || static_cast<std::size_t>(vertexId) >= positions.size()) {
                continue;
            }
            const double normalizedDistance =
                (positions[static_cast<std::size_t>(vertexId)] - sample.position).length() /
                sourceRadius;
            if (!std::isfinite(normalizedDistance) || normalizedDistance > 1.0) {
                continue;
            }
            const std::size_t vertexIndex = static_cast<std::size_t>(vertexId);
            if (normalizedDistance + kDistanceComparisonEpsilon <
                normalizedDistances[vertexIndex]) {
                normalizedDistances[vertexIndex] = normalizedDistance;
                winningSourceRadii[vertexIndex] = sourceRadius;
                pendingVertices.push({normalizedDistance, sourceRadius, vertexId});
            }
        }
    }

    while (!pendingVertices.empty()) {
        const VertexQueueEntry entry = pendingVertices.top();
        pendingVertices.pop();
        if (entry.vertexId < 0 ||
            static_cast<std::size_t>(entry.vertexId) >= normalizedDistances.size()) {
            continue;
        }
        const std::size_t vertexIndex = static_cast<std::size_t>(entry.vertexId);
        if (entry.normalizedDistance > normalizedDistances[vertexIndex] +
                kDistanceComparisonEpsilon ||
            entry.normalizedDistance > 1.0) {
            continue;
        }
        ++localMetrics.visitedVertexCount;

        for (const int edgeId : vertexEdges[vertexIndex]) {
            if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
                continue;
            }
            ++localMetrics.expandedEdgeCount;
            const MeshEdgeTopology& edge = edges[static_cast<std::size_t>(edgeId)];
            const int neighborId = edge.vertexIds[0] == entry.vertexId
                ? edge.vertexIds[1]
                : edge.vertexIds[0];
            if (neighborId < 0 || static_cast<std::size_t>(neighborId) >= positions.size()) {
                continue;
            }
            const double candidateDistance = entry.normalizedDistance +
                edge.worldLength / entry.sourceRadius;
            if (!std::isfinite(candidateDistance) || candidateDistance > 1.0) {
                continue;
            }
            const std::size_t neighborIndex = static_cast<std::size_t>(neighborId);
            if (candidateDistance + kDistanceComparisonEpsilon <
                normalizedDistances[neighborIndex]) {
                normalizedDistances[neighborIndex] = candidateDistance;
                winningSourceRadii[neighborIndex] = entry.sourceRadius;
                pendingVertices.push({candidateDistance, entry.sourceRadius, neighborId});
            }
        }
    }

    output.vertexInfluence.assign(positions.size(), 0.0F);
    for (std::size_t vertexIndex = 0; vertexIndex < positions.size(); ++vertexIndex) {
        if (std::isfinite(normalizedDistances[vertexIndex]) &&
            winningSourceRadii[vertexIndex] > settings_.minimumUsableRadius) {
            output.vertexInfluence[vertexIndex] =
                evaluateInfluence(normalizedDistances[vertexIndex], settings_.falloff);
        }
    }

    output.faceInfluence.assign(faces.size(), 0.0F);
    std::vector<unsigned char> coreFaceFlags(faces.size(), 0U);
    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        float influence = 0.0F;
        for (const int vertexId : faces[faceIndex].vertexIds) {
            if (vertexId >= 0 &&
                static_cast<std::size_t>(vertexId) < output.vertexInfluence.size()) {
                influence = std::max(
                    influence,
                    output.vertexInfluence[static_cast<std::size_t>(vertexId)]);
            }
        }
        output.faceInfluence[faceIndex] = influence;
        if (influence > settings_.coreInfluenceThreshold || directHitFaces[faceIndex] != 0U) {
            coreFaceFlags[faceIndex] = 1U;
        }
    }

    // Extract face-connected Core components. Disconnected surfaces remain
    // separate and no bridge polygons are synthesized.
    std::vector<int> coreOwner(faces.size(), -1);
    for (std::size_t seedFace = 0; seedFace < faces.size(); ++seedFace) {
        if (coreFaceFlags[seedFace] == 0U || coreOwner[seedFace] >= 0) {
            continue;
        }
        const int componentId = static_cast<int>(output.components.size());
        output.components.emplace_back();
        PaintRegionComponent& component = output.components.back();
        std::deque<int> pendingFaces;
        pendingFaces.push_back(static_cast<int>(seedFace));
        coreOwner[seedFace] = componentId;
        while (!pendingFaces.empty()) {
            const int faceId = pendingFaces.front();
            pendingFaces.pop_front();
            component.coreFaceIds.push_back(faceId);
            for (const int adjacentFaceId :
                 faces[static_cast<std::size_t>(faceId)].adjacentFaceIds) {
                if (adjacentFaceId < 0 ||
                    static_cast<std::size_t>(adjacentFaceId) >= faces.size()) {
                    continue;
                }
                const std::size_t adjacentIndex = static_cast<std::size_t>(adjacentFaceId);
                if (coreFaceFlags[adjacentIndex] != 0U && coreOwner[adjacentIndex] < 0) {
                    coreOwner[adjacentIndex] = componentId;
                    pendingFaces.push_back(adjacentFaceId);
                }
            }
        }
        std::sort(component.coreFaceIds.begin(), component.coreFaceIds.end());
    }

    // Multi-source face-ring expansion assigns every transition face to one
    // Core component. Components stay distinct even if their transition bands meet.
    std::vector<int> completeOwner = coreOwner;
    std::vector<int> ringDistance(faces.size(), -1);
    using FaceExpansionEntry = std::tuple<int, int, int>;  // ring, owner, face
    std::priority_queue<
        FaceExpansionEntry,
        std::vector<FaceExpansionEntry>,
        std::greater<FaceExpansionEntry>>
        expansionQueue;
    for (std::size_t faceIndex = 0; faceIndex < coreOwner.size(); ++faceIndex) {
        if (coreOwner[faceIndex] >= 0) {
            ringDistance[faceIndex] = 0;
            expansionQueue.emplace(
                0,
                coreOwner[faceIndex],
                static_cast<int>(faceIndex));
        }
    }
    while (!expansionQueue.empty()) {
        const auto [ring, owner, faceId] = expansionQueue.top();
        expansionQueue.pop();
        const std::size_t faceIndex = static_cast<std::size_t>(faceId);
        if (ring != ringDistance[faceIndex] || owner != completeOwner[faceIndex]) {
            continue;
        }
        const int nextRing = ring + 1;
        if (nextRing > settings_.transitionRings) {
            continue;
        }
        for (const int adjacentFaceId :
             faces[faceIndex].adjacentFaceIds) {
            if (adjacentFaceId < 0 ||
                static_cast<std::size_t>(adjacentFaceId) >= faces.size()) {
                continue;
            }
            const std::size_t adjacentIndex = static_cast<std::size_t>(adjacentFaceId);
            if (coreOwner[adjacentIndex] >= 0) {
                continue;
            }
            if (ringDistance[adjacentIndex] < 0 ||
                nextRing < ringDistance[adjacentIndex] ||
                (nextRing == ringDistance[adjacentIndex] &&
                 owner < completeOwner[adjacentIndex])) {
                ringDistance[adjacentIndex] = nextRing;
                completeOwner[adjacentIndex] = owner;
                expansionQueue.emplace(nextRing, owner, adjacentFaceId);
            }
        }
    }

    for (std::size_t faceIndex = 0; faceIndex < completeOwner.size(); ++faceIndex) {
        const int owner = completeOwner[faceIndex];
        if (owner < 0 || static_cast<std::size_t>(owner) >= output.components.size()) {
            continue;
        }
        PaintRegionComponent& component = output.components[static_cast<std::size_t>(owner)];
        component.allFaceIds.push_back(static_cast<int>(faceIndex));
        if (coreOwner[faceIndex] < 0) {
            component.transitionFaceIds.push_back(static_cast<int>(faceIndex));
        }
    }

    for (PaintRegionComponent& component : output.components) {
        std::sort(component.transitionFaceIds.begin(), component.transitionFaceIds.end());
        std::sort(component.allFaceIds.begin(), component.allFaceIds.end());
        boundaryExtractor_.extract(topology, component);
    }

    if (metrics != nullptr) {
        *metrics = localMetrics;
    }
    return !output.components.empty();
}

}  // namespace directional_retopo
