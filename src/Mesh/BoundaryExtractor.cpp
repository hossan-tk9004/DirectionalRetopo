#include "Mesh/BoundaryExtractor.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace directional_retopo {
namespace {

int otherVertex(const BoundaryEdge& edge, int vertexId)
{
    if (edge.vertexIds[0] == vertexId) {
        return edge.vertexIds[1];
    }
    if (edge.vertexIds[1] == vertexId) {
        return edge.vertexIds[0];
    }
    return -1;
}

}  // namespace

void BoundaryExtractor::extract(
    const MeshTopologyCache& topology,
    PaintRegionComponent& component) const
{
    component.boundaryVertexIds.clear();
    component.boundaryEdges.clear();
    component.boundaryLoops.clear();
    component.hasAmbiguousBoundary = false;

    const std::vector<MeshFaceTopology>& faces = topology.faces();
    const std::vector<MeshEdgeTopology>& edges = topology.edges();
    std::vector<unsigned char> regionFaceFlags(faces.size(), 0U);
    for (const int faceId : component.allFaceIds) {
        if (faceId >= 0 && static_cast<std::size_t>(faceId) < faces.size()) {
            regionFaceFlags[static_cast<std::size_t>(faceId)] = 1U;
        }
    }

    std::unordered_set<int> candidateEdgeIds;
    for (const int faceId : component.allFaceIds) {
        if (faceId < 0 || static_cast<std::size_t>(faceId) >= faces.size()) {
            continue;
        }
        for (const int edgeId : faces[static_cast<std::size_t>(faceId)].edgeIds) {
            candidateEdgeIds.insert(edgeId);
        }
    }

    std::vector<int> sortedEdgeIds(candidateEdgeIds.begin(), candidateEdgeIds.end());
    std::sort(sortedEdgeIds.begin(), sortedEdgeIds.end());
    for (const int edgeId : sortedEdgeIds) {
        if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
            continue;
        }

        const MeshEdgeTopology& topologyEdge = edges[static_cast<std::size_t>(edgeId)];
        std::vector<int> insideFaces;
        std::vector<int> outsideFaces;
        for (const int faceId : topologyEdge.faceIds) {
            if (faceId >= 0 && static_cast<std::size_t>(faceId) < regionFaceFlags.size() &&
                regionFaceFlags[static_cast<std::size_t>(faceId)] != 0U) {
                insideFaces.push_back(faceId);
            } else {
                outsideFaces.push_back(faceId);
            }
        }

        const bool atOpenMeshBoundary =
            topologyEdge.originalMeshBoundary && insideFaces.size() == 1;
        const bool separatesRegion =
            !insideFaces.empty() && !outsideFaces.empty();
        if (!atOpenMeshBoundary && !separatesRegion) {
            continue;
        }

        BoundaryEdge boundaryEdge;
        boundaryEdge.edgeId = edgeId;
        boundaryEdge.vertexIds = topologyEdge.vertexIds;
        boundaryEdge.insideFaceId = insideFaces.empty() ? -1 : insideFaces.front();
        boundaryEdge.outsideFaceId = outsideFaces.empty() ? -1 : outsideFaces.front();
        boundaryEdge.isOriginalMeshBoundary = topologyEdge.originalMeshBoundary;
        boundaryEdge.ambiguous = topologyEdge.faceIds.size() > 2 ||
            insideFaces.size() != 1 || outsideFaces.size() > 1;
        component.hasAmbiguousBoundary =
            component.hasAmbiguousBoundary || boundaryEdge.ambiguous;
        component.boundaryEdges.push_back(boundaryEdge);
    }

    std::unordered_map<int, std::vector<std::size_t>> vertexToBoundaryEdges;
    for (std::size_t edgeIndex = 0; edgeIndex < component.boundaryEdges.size(); ++edgeIndex) {
        const BoundaryEdge& edge = component.boundaryEdges[edgeIndex];
        vertexToBoundaryEdges[edge.vertexIds[0]].push_back(edgeIndex);
        vertexToBoundaryEdges[edge.vertexIds[1]].push_back(edgeIndex);
    }
    component.boundaryVertexIds.reserve(vertexToBoundaryEdges.size());
    for (auto& entry : vertexToBoundaryEdges) {
        component.boundaryVertexIds.push_back(entry.first);
        std::sort(
            entry.second.begin(),
            entry.second.end(),
            [&component](std::size_t left, std::size_t right) {
                return component.boundaryEdges[left].edgeId <
                    component.boundaryEdges[right].edgeId;
            });
        if (entry.second.size() != 2) {
            component.hasAmbiguousBoundary = true;
        }
    }
    std::sort(component.boundaryVertexIds.begin(), component.boundaryVertexIds.end());

    std::vector<unsigned char> visited(component.boundaryEdges.size(), 0U);
    const auto walkBoundary = [&component, &vertexToBoundaryEdges, &visited](
                                  int startVertex,
                                  std::size_t startEdgeIndex) {
        BoundaryLoop loop;
        int currentVertex = startVertex;
        std::size_t currentEdgeIndex = startEdgeIndex;
        loop.vertexIds.push_back(currentVertex);

        for (std::size_t step = 0; step < component.boundaryEdges.size(); ++step) {
            if (currentEdgeIndex >= visited.size() || visited[currentEdgeIndex] != 0U) {
                loop.ambiguous = true;
                break;
            }

            visited[currentEdgeIndex] = 1U;
            const BoundaryEdge& edge = component.boundaryEdges[currentEdgeIndex];
            loop.edgeIds.push_back(edge.edgeId);
            loop.touchesOriginalMeshBoundary =
                loop.touchesOriginalMeshBoundary || edge.isOriginalMeshBoundary;
            loop.ambiguous = loop.ambiguous || edge.ambiguous;

            const int nextVertex = otherVertex(edge, currentVertex);
            if (nextVertex < 0) {
                loop.ambiguous = true;
                break;
            }
            loop.vertexIds.push_back(nextVertex);
            if (nextVertex == startVertex) {
                loop.closed = !loop.touchesOriginalMeshBoundary && !loop.ambiguous;
                break;
            }

            const auto incidentIterator = vertexToBoundaryEdges.find(nextVertex);
            if (incidentIterator == vertexToBoundaryEdges.end()) {
                break;
            }
            std::vector<std::size_t> unvisitedEdges;
            for (const std::size_t candidate : incidentIterator->second) {
                if (visited[candidate] == 0U) {
                    unvisitedEdges.push_back(candidate);
                }
            }
            if (unvisitedEdges.empty()) {
                break;
            }
            if (unvisitedEdges.size() > 1) {
                // Continue deterministically through one branch. Remaining
                // branches are emitted by subsequent walks.
                loop.ambiguous = true;
            }
            currentVertex = nextVertex;
            currentEdgeIndex = unvisitedEdges.front();
        }

        if (loop.touchesOriginalMeshBoundary) {
            // Even if original open-boundary edges form a graph cycle, expose
            // this as a stitch boundary chain for downstream patch solvers.
            loop.closed = false;
        }
        if (loop.closed && loop.vertexIds.size() > 1U &&
            loop.vertexIds.front() == loop.vertexIds.back()) {
            // Closed loops store unique cycle vertices; closure is represented by closed.
            loop.vertexIds.pop_back();
        }
        return loop;
    };

    // Open endpoints and branches first, so ambiguous graphs become finite,
    // deterministic chains instead of arbitrary cycles.
    for (const int vertexId : component.boundaryVertexIds) {
        const auto& incidentEdges = vertexToBoundaryEdges[vertexId];
        bool touchesOpenBoundary = false;
        for (const std::size_t edgeIndex : incidentEdges) {
            touchesOpenBoundary = touchesOpenBoundary ||
                component.boundaryEdges[edgeIndex].isOriginalMeshBoundary;
        }
        if (incidentEdges.size() == 2 && !touchesOpenBoundary) {
            continue;
        }
        for (const std::size_t edgeIndex : incidentEdges) {
            if (visited[edgeIndex] == 0U) {
                BoundaryLoop loop = walkBoundary(vertexId, edgeIndex);
                component.hasAmbiguousBoundary =
                    component.hasAmbiguousBoundary || loop.ambiguous;
                component.boundaryLoops.push_back(std::move(loop));
            }
        }
    }

    // Remaining degree-two subgraphs are ordinary closed loops.
    for (std::size_t edgeIndex = 0; edgeIndex < component.boundaryEdges.size(); ++edgeIndex) {
        if (visited[edgeIndex] != 0U) {
            continue;
        }
        BoundaryLoop loop = walkBoundary(
            component.boundaryEdges[edgeIndex].vertexIds[0],
            edgeIndex);
        component.hasAmbiguousBoundary =
            component.hasAmbiguousBoundary || loop.ambiguous;
        component.boundaryLoops.push_back(std::move(loop));
    }
}

}  // namespace directional_retopo
