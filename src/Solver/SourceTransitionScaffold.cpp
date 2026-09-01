#include "Solver/SourceTransitionScaffold.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <set>
#include <tuple>
#include <utility>

namespace directional_retopo::solver {
namespace {

using Clock = std::chrono::steady_clock;

ScaffoldVertexClassification operator|(
    ScaffoldVertexClassification left,
    ScaffoldVertexClassification right) noexcept
{
    return static_cast<ScaffoldVertexClassification>(
        static_cast<std::uint8_t>(left) |
        static_cast<std::uint8_t>(right));
}

PolygonType polygonType(std::size_t vertexCount) noexcept
{
    if (vertexCount == 3U) {
        return PolygonType::Triangle;
    }
    if (vertexCount == 4U) {
        return PolygonType::Quad;
    }
    return PolygonType::NGon;
}

bool edgeConnects(
    const SourceEdge& edge,
    std::size_t first,
    std::size_t second) noexcept
{
    return (edge.vertexIndices[0] == first && edge.vertexIndices[1] == second) ||
        (edge.vertexIndices[0] == second && edge.vertexIndices[1] == first);
}

double elapsedMilliseconds(Clock::time_point start) noexcept
{
    return std::chrono::duration<double, std::milli>(
        Clock::now() - start).count();
}

std::uint64_t fnv(std::uint64_t hash, std::uint64_t value) noexcept
{
    constexpr std::uint64_t prime = 1099511628211ULL;
    for (unsigned int byte = 0U; byte < 8U; ++byte) {
        hash ^= (value >> (byte * 8U)) & 0xffU;
        hash *= prime;
    }
    return hash;
}

std::uint64_t doubleBits(double value) noexcept
{
    std::uint64_t bits = 0U;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

}  // namespace

bool hasClassification(
    ScaffoldVertexClassification value,
    ScaffoldVertexClassification classification) noexcept
{
    return (static_cast<std::uint8_t>(value) &
            static_cast<std::uint8_t>(classification)) != 0U;
}

SourceTransitionScaffold SourceTransitionScaffoldExtractor::extract(
    const SourceMeshSnapshot& sourceMesh,
    const RegionComponent& component,
    const RemeshSettings& settings) const
{
    const Clock::time_point start = Clock::now();
    SourceTransitionScaffold scaffold;
    scaffold.componentId = component.componentId;
    scaffold.localVertexIndexBySource.assign(
        sourceMesh.vertices.size(), kInvalidIndex);
    scaffold.localEdgeIndexBySource.assign(
        sourceMesh.edges.size(), kInvalidIndex);
    scaffold.localFaceIndexBySource.assign(
        sourceMesh.faces.size(), kInvalidIndex);

    const auto fail = [&scaffold, start](
                          ScaffoldStatus status,
                          const std::string& message) {
        scaffold.status = status;
        scaffold.diagnosticMessage = message;
        scaffold.diagnostics.extractionMilliseconds =
            elapsedMilliseconds(start);
        return scaffold;
    };

    if (sourceMesh.vertices.empty() || sourceMesh.edges.empty() ||
        sourceMesh.faces.empty() || component.coreFaceIndices.empty() ||
        component.transitionFaceIndices.empty() ||
        component.allFaceIndices.empty() ||
        !(settings.geometryEpsilon > 0.0) ||
        !std::isfinite(settings.geometryEpsilon)) {
        return fail(
            ScaffoldStatus::InvalidInput,
            "Source scaffold input is empty or has an invalid geometry tolerance.");
    }

    const std::size_t faceCount = sourceMesh.faces.size();
    std::vector<unsigned char> core(faceCount, 0U);
    std::vector<unsigned char> transition(faceCount, 0U);
    std::vector<unsigned char> complete(faceCount, 0U);
    const auto markFaces = [&scaffold, faceCount](
                               const std::vector<std::size_t>& indices,
                               std::vector<unsigned char>& flags) {
        for (const std::size_t index : indices) {
            if (index >= faceCount || flags[index] != 0U) {
                ++scaffold.diagnostics.invalidReferenceCount;
                return false;
            }
            flags[index] = 1U;
        }
        return true;
    };
    if (!markFaces(component.coreFaceIndices, core) ||
        !markFaces(component.transitionFaceIndices, transition) ||
        !markFaces(component.allFaceIndices, complete)) {
        return fail(
            ScaffoldStatus::InvalidInput,
            "Region component contains an invalid or duplicate face index.");
    }
    for (std::size_t faceIndex = 0U; faceIndex < faceCount; ++faceIndex) {
        if (core[faceIndex] != 0U && transition[faceIndex] != 0U) {
            return fail(
                ScaffoldStatus::InvalidInput,
                "Core and Transition face sets overlap.");
        }
        const bool classified =
            core[faceIndex] != 0U || transition[faceIndex] != 0U;
        if ((complete[faceIndex] != 0U) != classified) {
            return fail(
                ScaffoldStatus::InvalidInput,
                "Complete Region faces do not equal Core plus Transition faces.");
        }
    }

    if (component.transitionRingDepthByFace.size() != faceCount) {
        return fail(
            ScaffoldStatus::RingDepthInvalid,
            "Transition ring-depth array does not match the source face count.");
    }
    std::set<int> transitionDepths;
    for (std::size_t faceIndex = 0U; faceIndex < faceCount; ++faceIndex) {
        const int depth = component.transitionRingDepthByFace[faceIndex];
        if (core[faceIndex] != 0U && depth != 0) {
            return fail(
                ScaffoldStatus::RingDepthInvalid,
                "Core face ring depth must be zero.");
        }
        if (transition[faceIndex] != 0U) {
            if (depth <= 0) {
                return fail(
                    ScaffoldStatus::RingDepthInvalid,
                    "Every Transition face requires a positive Core-relative ring depth.");
            }
            transitionDepths.insert(depth);
            if (static_cast<unsigned int>(depth) > settings.topologyBlendWidth) {
                ++scaffold.diagnostics.ringDepthWarningCount;
            }
        }
    }
    scaffold.diagnostics.transitionRingCount = transitionDepths.size();

    std::vector<std::size_t> transitionFaceIndices =
        component.transitionFaceIndices;
    std::sort(transitionFaceIndices.begin(), transitionFaceIndices.end());
    std::set<std::size_t> scaffoldEdgeIndices;
    std::set<std::size_t> scaffoldVertexIndices;
    for (const std::size_t faceIndex : transitionFaceIndices) {
        const SourceFace& face = sourceMesh.faces[faceIndex];
        if (face.vertexIndices.size() < 3U ||
            face.edgeIndices.size() != face.vertexIndices.size() ||
            !face.center.finite() || !face.normal.finite() ||
            face.sourceFaceId == kInvalidSourceId) {
            return fail(
                ScaffoldStatus::InvalidInput,
                "Transition face has invalid original polygon topology or geometry.");
        }
        for (std::size_t corner = 0U; corner < face.vertexIndices.size(); ++corner) {
            const std::size_t vertexIndex = face.vertexIndices[corner];
            const std::size_t nextVertexIndex =
                face.vertexIndices[(corner + 1U) % face.vertexIndices.size()];
            const std::size_t edgeIndex = face.edgeIndices[corner];
            if (vertexIndex >= sourceMesh.vertices.size() ||
                nextVertexIndex >= sourceMesh.vertices.size() ||
                edgeIndex >= sourceMesh.edges.size()) {
                ++scaffold.diagnostics.invalidReferenceCount;
                return fail(
                    ScaffoldStatus::InvalidInput,
                    "Transition polygon contains an invalid source reference.");
            }
            const SourceVertex& vertex = sourceMesh.vertices[vertexIndex];
            const SourceEdge& edge = sourceMesh.edges[edgeIndex];
            if (!vertex.position.finite() || !vertex.normal.finite() ||
                vertex.sourceVertexId == kInvalidSourceId ||
                edge.sourceEdgeId == kInvalidSourceId ||
                !edgeConnects(edge, vertexIndex, nextVertexIndex)) {
                return fail(
                    ScaffoldStatus::InvalidInput,
                    "Transition polygon does not match original source edges.");
            }
            const double edgeLength =
                (sourceMesh.vertices[edge.vertexIndices[1]].position -
                 sourceMesh.vertices[edge.vertexIndices[0]].position).length();
            if (!std::isfinite(edgeLength) ||
                edgeLength <= settings.geometryEpsilon) {
                return fail(
                    ScaffoldStatus::InvalidInput,
                    "Transition scaffold contains a zero-length source edge.");
            }
            if (edge.faceIndices.size() > 2U) {
                ++scaffold.diagnostics.nonManifoldEdgeCount;
                return fail(
                    ScaffoldStatus::NonManifoldTransition,
                    "Transition scaffold contains a non-manifold source edge.");
            }
            scaffoldVertexIndices.insert(vertexIndex);
            scaffoldEdgeIndices.insert(edgeIndex);
        }
    }

    // The inner Core must be one connected source-polygon component. Use
    // original source edges only; triangulation diagonals are intentionally
    // absent from SourceFace::edgeIndices.
    std::vector<std::size_t> pendingCore;
    pendingCore.push_back(component.coreFaceIndices.front());
    std::vector<unsigned char> visitedCore(faceCount, 0U);
    visitedCore[pendingCore.front()] = 1U;
    std::size_t visitedCoreCount = 0U;
    while (!pendingCore.empty()) {
        const std::size_t faceIndex = pendingCore.back();
        pendingCore.pop_back();
        ++visitedCoreCount;
        for (const std::size_t edgeIndex : sourceMesh.faces[faceIndex].edgeIndices) {
            if (edgeIndex >= sourceMesh.edges.size()) {
                return fail(
                    ScaffoldStatus::InvalidInput,
                    "Core face contains an invalid source edge reference.");
            }
            for (const std::size_t adjacentFace : sourceMesh.edges[edgeIndex].faceIndices) {
                if (adjacentFace < faceCount && core[adjacentFace] != 0U &&
                    visitedCore[adjacentFace] == 0U) {
                    visitedCore[adjacentFace] = 1U;
                    pendingCore.push_back(adjacentFace);
                }
            }
        }
    }
    if (visitedCoreCount != component.coreFaceIndices.size()) {
        return fail(
            ScaffoldStatus::CoreDisconnected,
            "R4 supports one connected Core component per scaffold.");
    }

    if (component.fixedBoundaryLoops.empty()) {
        return fail(
            ScaffoldStatus::MissingOuterBoundary,
            "Transition scaffold requires one Fixed Source Boundary loop.");
    }
    if (component.fixedBoundaryLoops.size() != 1U) {
        return fail(
            ScaffoldStatus::MultipleOuterBoundaries,
            "R4 supports exactly one Fixed Source Boundary loop.");
    }

    const OrderedBoundaryLoop& sourceOuter = component.fixedBoundaryLoops.front();
    if (!sourceOuter.closed || sourceOuter.vertexIndices.size() < 3U ||
        sourceOuter.edgeIndices.size() != sourceOuter.vertexIndices.size() ||
        sourceOuter.sourceVertexIds.size() != sourceOuter.vertexIndices.size() ||
        sourceOuter.sourceEdgeIds.size() != sourceOuter.edgeIndices.size()) {
        return fail(
            ScaffoldStatus::OuterBoundaryInvalid,
            "Fixed Source Boundary must be one ordered closed polygon-edge loop.");
    }

    std::set<std::size_t> fixedOuterEdges;
    std::set<std::size_t> fixedOuterVertices;
    for (std::size_t item = 0U; item < sourceOuter.vertexIndices.size(); ++item) {
        const std::size_t vertexIndex = sourceOuter.vertexIndices[item];
        const std::size_t nextVertexIndex =
            sourceOuter.vertexIndices[(item + 1U) % sourceOuter.vertexIndices.size()];
        const std::size_t edgeIndex = sourceOuter.edgeIndices[item];
        if (vertexIndex >= sourceMesh.vertices.size() ||
            edgeIndex >= sourceMesh.edges.size() ||
            !edgeConnects(sourceMesh.edges[edgeIndex], vertexIndex, nextVertexIndex) ||
            sourceMesh.vertices[vertexIndex].sourceVertexId != sourceOuter.sourceVertexIds[item] ||
            sourceMesh.edges[edgeIndex].sourceEdgeId != sourceOuter.sourceEdgeIds[item] ||
            fixedOuterVertices.count(vertexIndex) != 0U ||
            fixedOuterEdges.count(edgeIndex) != 0U) {
            return fail(
                ScaffoldStatus::OuterBoundaryInvalid,
                "Fixed Source Boundary ordering or source-ID mapping is invalid.");
        }
        fixedOuterVertices.insert(vertexIndex);
        fixedOuterEdges.insert(edgeIndex);
        scaffoldVertexIndices.insert(vertexIndex);
        scaffoldEdgeIndices.insert(edgeIndex);
    }

    // Allocate stable local indices in ascending source-snapshot order.
    for (const std::size_t sourceVertexIndex : scaffoldVertexIndices) {
        const SourceVertex& sourceVertex = sourceMesh.vertices[sourceVertexIndex];
        ScaffoldVertex local;
        local.localIndex = scaffold.vertices.size();
        local.sourceVertexIndex = sourceVertexIndex;
        local.sourceVertexId = sourceVertex.sourceVertexId;
        local.position = sourceVertex.position;
        local.normal = sourceVertex.normal;
        local.classification = ScaffoldVertexClassification::TransitionInterior;
        scaffold.localVertexIndexBySource[sourceVertexIndex] = local.localIndex;
        scaffold.vertices.push_back(local);
    }
    for (const std::size_t sourceFaceIndex : transitionFaceIndices) {
        const SourceFace& sourceFace = sourceMesh.faces[sourceFaceIndex];
        ScaffoldFace local;
        local.localIndex = scaffold.faces.size();
        local.sourceFaceIndex = sourceFaceIndex;
        local.sourceFaceId = sourceFace.sourceFaceId;
        local.transitionRingDepth = component.transitionRingDepthByFace[sourceFaceIndex];
        local.polygonType = polygonType(sourceFace.vertexIndices.size());
        for (const std::size_t vertexIndex : sourceFace.vertexIndices) {
            local.vertexIndices.push_back(scaffold.localVertexIndexBySource[vertexIndex]);
        }
        scaffold.localFaceIndexBySource[sourceFaceIndex] = local.localIndex;
        scaffold.faces.push_back(std::move(local));
        switch (polygonType(sourceFace.vertexIndices.size())) {
        case PolygonType::Triangle: ++scaffold.diagnostics.triangleCount; break;
        case PolygonType::Quad: ++scaffold.diagnostics.quadCount; break;
        case PolygonType::NGon: ++scaffold.diagnostics.nGonCount; break;
        }
    }

    std::set<std::size_t> innerInterfaceEdges;
    for (const std::size_t sourceEdgeIndex : scaffoldEdgeIndices) {
        const SourceEdge& sourceEdge = sourceMesh.edges[sourceEdgeIndex];
        std::size_t transitionSides = 0U;
        std::size_t coreSides = 0U;
        std::set<std::size_t> uniqueFaces;
        for (const std::size_t faceIndex : sourceEdge.faceIndices) {
            if (faceIndex >= faceCount || !uniqueFaces.insert(faceIndex).second) {
                return fail(
                    ScaffoldStatus::InvalidInput,
                    "Source edge contains an invalid or duplicate incident face.");
            }
            transitionSides += transition[faceIndex] != 0U ? 1U : 0U;
            coreSides += core[faceIndex] != 0U ? 1U : 0U;
        }

        ScaffoldEdgeClassification classification =
            ScaffoldEdgeClassification::TransitionInterior;
        if (fixedOuterEdges.count(sourceEdgeIndex) != 0U) {
            if (transitionSides != 1U || coreSides != 0U) {
                return fail(
                    ScaffoldStatus::OuterBoundaryInvalid,
                    "Fixed Source Boundary edge is not the outer edge of Transition.");
            }
            classification = ScaffoldEdgeClassification::FixedOuterBoundary;
        } else if (transitionSides == 1U && coreSides == 1U) {
            classification = ScaffoldEdgeClassification::InnerInterface;
            innerInterfaceEdges.insert(sourceEdgeIndex);
        } else if (transitionSides == 2U && coreSides == 0U) {
            classification = ScaffoldEdgeClassification::TransitionInterior;
        } else {
            return fail(
                ScaffoldStatus::AmbiguousTransitionTopology,
                "Transition edge is neither Fixed Boundary, Transition interior, nor Core interface.");
        }

        ScaffoldEdge local;
        local.localIndex = scaffold.edges.size();
        local.sourceEdgeIndex = sourceEdgeIndex;
        local.sourceEdgeId = sourceEdge.sourceEdgeId;
        local.classification = classification;
        local.vertexIndices = {
            scaffold.localVertexIndexBySource[sourceEdge.vertexIndices[0]],
            scaffold.localVertexIndexBySource[sourceEdge.vertexIndices[1]]};
        if (local.vertexIndices[0] == kInvalidIndex ||
            local.vertexIndices[1] == kInvalidIndex) {
            return fail(
                ScaffoldStatus::InvalidInput,
                "Transition edge endpoint is missing from the local vertex mapping.");
        }
        for (const std::size_t sourceFaceIndex : sourceEdge.faceIndices) {
            if (transition[sourceFaceIndex] != 0U) {
                const std::size_t localFace =
                    scaffold.localFaceIndexBySource[sourceFaceIndex];
                if (localFace == kInvalidIndex) {
                    return fail(
                        ScaffoldStatus::InvalidInput,
                        "Transition edge face is missing from the local face mapping.");
                }
                local.faceIndices.push_back(localFace);
            }
        }
        scaffold.localEdgeIndexBySource[sourceEdgeIndex] = local.localIndex;
        scaffold.edges.push_back(std::move(local));
    }

    for (ScaffoldFace& localFace : scaffold.faces) {
        for (const std::size_t sourceEdgeIndex :
             sourceMesh.faces[localFace.sourceFaceIndex].edgeIndices) {
            const std::size_t localEdge =
                scaffold.localEdgeIndexBySource[sourceEdgeIndex];
            if (localEdge == kInvalidIndex) {
                return fail(
                    ScaffoldStatus::InvalidInput,
                    "Transition face edge is missing from the local edge mapping.");
            }
            localFace.edgeIndices.push_back(localEdge);
        }
    }

    ScaffoldBoundaryLoop fixedLoop;
    fixedLoop.closed = true;
    for (std::size_t item = 0U; item < sourceOuter.vertexIndices.size(); ++item) {
        const std::size_t sourceVertexIndex = sourceOuter.vertexIndices[item];
        const std::size_t sourceEdgeIndex = sourceOuter.edgeIndices[item];
        const std::size_t localVertex =
            scaffold.localVertexIndexBySource[sourceVertexIndex];
        const std::size_t localEdge =
            scaffold.localEdgeIndexBySource[sourceEdgeIndex];
        if (localVertex == kInvalidIndex || localEdge == kInvalidIndex) {
            return fail(
                ScaffoldStatus::OuterBoundaryInvalid,
                "Fixed Source Boundary mapping is incomplete.");
        }
        fixedLoop.vertexIndices.push_back(localVertex);
        fixedLoop.edgeIndices.push_back(localEdge);
        fixedLoop.sourceVertexIndices.push_back(sourceVertexIndex);
        fixedLoop.sourceEdgeIndices.push_back(sourceEdgeIndex);
        fixedLoop.sourceVertexIds.push_back(sourceOuter.sourceVertexIds[item]);
        fixedLoop.sourceEdgeIds.push_back(sourceOuter.sourceEdgeIds[item]);
        scaffold.vertices[localVertex].classification =
            scaffold.vertices[localVertex].classification |
            ScaffoldVertexClassification::FixedOuterBoundary;
        if (scaffold.edges[localEdge].classification !=
            ScaffoldEdgeClassification::FixedOuterBoundary) {
            return fail(
                ScaffoldStatus::OuterBoundaryInvalid,
                "Fixed Source Boundary edge classification is inconsistent.");
        }
        const double displacement =
            (scaffold.vertices[localVertex].position -
             sourceMesh.vertices[sourceVertexIndex].position).length();
        scaffold.diagnostics.maximumFixedBoundaryDisplacement =
            std::max(scaffold.diagnostics.maximumFixedBoundaryDisplacement,
                     displacement);
    }
    scaffold.fixedOuterBoundaryLoops.push_back(std::move(fixedLoop));

    if (innerInterfaceEdges.empty()) {
        return fail(
            ScaffoldStatus::MissingInnerInterface,
            "Transition scaffold has no Transition-to-Core interface.");
    }

    using InterfaceConnection = std::pair<std::size_t, std::size_t>;
    std::vector<std::vector<InterfaceConnection>> interfaceAdjacency(
        sourceMesh.vertices.size());
    for (const std::size_t edgeIndex : innerInterfaceEdges) {
        const SourceEdge& edge = sourceMesh.edges[edgeIndex];
        interfaceAdjacency[edge.vertexIndices[0]].push_back(
            {edge.vertexIndices[1], edgeIndex});
        interfaceAdjacency[edge.vertexIndices[1]].push_back(
            {edge.vertexIndices[0], edgeIndex});
    }
    std::vector<std::size_t> interfaceVertices;
    for (std::size_t vertexIndex = 0U;
         vertexIndex < interfaceAdjacency.size();
         ++vertexIndex) {
        const std::size_t degree = interfaceAdjacency[vertexIndex].size();
        if (degree == 0U) {
            continue;
        }
        interfaceVertices.push_back(vertexIndex);
        if (degree == 1U) {
            ++scaffold.diagnostics.openInterfaceCount;
        } else if (degree != 2U) {
            ++scaffold.diagnostics.branchedInterfaceVertexCount;
        }
    }
    if (scaffold.diagnostics.branchedInterfaceVertexCount != 0U) {
        return fail(
            ScaffoldStatus::BranchedInnerInterface,
            "Inner interface is branched or non-manifold.");
    }
    if (scaffold.diagnostics.openInterfaceCount != 0U) {
        return fail(
            ScaffoldStatus::OpenInnerInterface,
            "R4 supports a closed Inner Interface loop.");
    }

    const auto sourceVertexKey = [&sourceMesh](std::size_t vertexIndex) {
        return std::pair<SourceId, std::size_t>(
            sourceMesh.vertices[vertexIndex].sourceVertexId, vertexIndex);
    };
    const std::size_t startVertex = *std::min_element(
        interfaceVertices.begin(),
        interfaceVertices.end(),
        [&sourceVertexKey](std::size_t left, std::size_t right) {
            return sourceVertexKey(left) < sourceVertexKey(right);
        });

    const auto transitionEdgeForward =
        [&sourceMesh, &transition](
            std::size_t edgeIndex,
            std::size_t from,
            std::size_t to) {
            const SourceEdge& edge = sourceMesh.edges[edgeIndex];
            for (const std::size_t faceIndex : edge.faceIndices) {
                if (transition[faceIndex] == 0U) {
                    continue;
                }
                const SourceFace& face = sourceMesh.faces[faceIndex];
                for (std::size_t corner = 0U;
                     corner < face.edgeIndices.size();
                     ++corner) {
                    if (face.edgeIndices[corner] == edgeIndex) {
                        return face.vertexIndices[corner] == from &&
                            face.vertexIndices[(corner + 1U) %
                                               face.vertexIndices.size()] == to;
                    }
                }
            }
            return false;
        };

    const std::vector<InterfaceConnection>& startConnections =
        interfaceAdjacency[startVertex];
    InterfaceConnection firstConnection = startConnections.front();
    const bool firstForward = transitionEdgeForward(
        firstConnection.second, startVertex, firstConnection.first);
    const bool secondForward = transitionEdgeForward(
        startConnections[1].second, startVertex, startConnections[1].first);
    if (firstForward == secondForward) {
        return fail(
            ScaffoldStatus::AmbiguousTransitionTopology,
            "Inner Interface winding cannot be derived from Transition polygon order.");
    }
    if (!firstForward) {
        firstConnection = startConnections[1];
    }

    ScaffoldBoundaryLoop innerLoop;
    innerLoop.closed = true;
    std::set<std::size_t> traversedEdges;
    std::size_t previousVertex = kInvalidIndex;
    std::size_t currentVertex = startVertex;
    InterfaceConnection nextConnection = firstConnection;
    while (true) {
        const std::size_t edgeIndex = nextConnection.second;
        const std::size_t nextVertex = nextConnection.first;
        if (!traversedEdges.insert(edgeIndex).second) {
            return fail(
                ScaffoldStatus::MultipleInnerInterfaces,
                "Inner Interface traversal repeated an edge before closing.");
        }
        const std::size_t localVertex =
            scaffold.localVertexIndexBySource[currentVertex];
        const std::size_t localEdge =
            scaffold.localEdgeIndexBySource[edgeIndex];
        if (localVertex == kInvalidIndex || localEdge == kInvalidIndex) {
            return fail(
                ScaffoldStatus::InvalidInput,
                "Inner Interface mapping is incomplete.");
        }
        innerLoop.vertexIndices.push_back(localVertex);
        innerLoop.edgeIndices.push_back(localEdge);
        innerLoop.sourceVertexIndices.push_back(currentVertex);
        innerLoop.sourceEdgeIndices.push_back(edgeIndex);
        innerLoop.sourceVertexIds.push_back(
            sourceMesh.vertices[currentVertex].sourceVertexId);
        innerLoop.sourceEdgeIds.push_back(
            sourceMesh.edges[edgeIndex].sourceEdgeId);
        scaffold.vertices[localVertex].classification =
            scaffold.vertices[localVertex].classification |
            ScaffoldVertexClassification::InnerInterface;
        if (scaffold.edges[localEdge].classification !=
            ScaffoldEdgeClassification::InnerInterface) {
            return fail(
                ScaffoldStatus::AmbiguousTransitionTopology,
                "Inner Interface edge classification is inconsistent.");
        }

        previousVertex = currentVertex;
        currentVertex = nextVertex;
        if (currentVertex == startVertex) {
            break;
        }
        const std::vector<InterfaceConnection>& connections =
            interfaceAdjacency[currentVertex];
        bool found = false;
        for (const InterfaceConnection& connection : connections) {
            if (connection.first != previousVertex &&
                traversedEdges.count(connection.second) == 0U) {
                nextConnection = connection;
                found = true;
                break;
            }
        }
        if (!found) {
            return fail(
                ScaffoldStatus::OpenInnerInterface,
                "Inner Interface traversal ended before closing.");
        }
    }
    if (traversedEdges.size() != innerInterfaceEdges.size()) {
        return fail(
            ScaffoldStatus::MultipleInnerInterfaces,
            "R4 supports exactly one closed Inner Interface loop.");
    }
    scaffold.innerInterfaceLoops.push_back(std::move(innerLoop));

    // Core-relative ring depth must change by no more than one across an
    // original source edge.  This validates the RegionComponent semantics
    // without consulting triangulation adjacency.
    for (const SourceEdge& edge : sourceMesh.edges) {
        std::vector<int> depths;
        for (const std::size_t faceIndex : edge.faceIndices) {
            if (faceIndex < faceCount && complete[faceIndex] != 0U) {
                depths.push_back(component.transitionRingDepthByFace[faceIndex]);
            }
        }
        if (depths.size() == 2U && std::abs(depths[0] - depths[1]) > 1) {
            return fail(
                ScaffoldStatus::RingDepthInvalid,
                "Adjacent Complete Region faces have a non-contiguous ring depth.");
        }
    }

    scaffold.diagnostics.vertexMappingCoverage =
        scaffold.vertices.empty()
            ? 0.0
            : static_cast<double>(scaffoldVertexIndices.size()) /
                  static_cast<double>(scaffold.vertices.size());
    scaffold.diagnostics.edgeMappingCoverage =
        scaffold.edges.empty()
            ? 0.0
            : static_cast<double>(scaffoldEdgeIndices.size()) /
                  static_cast<double>(scaffold.edges.size());
    scaffold.diagnostics.faceMappingCoverage =
        scaffold.faces.empty()
            ? 0.0
            : static_cast<double>(transitionFaceIndices.size()) /
                  static_cast<double>(scaffold.faces.size());
    scaffold.diagnostics.fixedBoundaryVertexCoverage =
        sourceOuter.vertexIndices.empty()
            ? 0.0
            : static_cast<double>(
                  scaffold.fixedOuterBoundaryLoops.front().vertexIndices.size()) /
                  static_cast<double>(sourceOuter.vertexIndices.size());
    scaffold.diagnostics.fixedBoundaryEdgeCoverage =
        sourceOuter.edgeIndices.empty()
            ? 0.0
            : static_cast<double>(
                  scaffold.fixedOuterBoundaryLoops.front().edgeIndices.size()) /
                  static_cast<double>(sourceOuter.edgeIndices.size());
    scaffold.diagnostics.extractionMilliseconds = elapsedMilliseconds(start);
    scaffold.status = ScaffoldStatus::Success;
    scaffold.diagnosticMessage =
        "Source Transition Scaffold extracted from immutable original polygon topology.";
    return scaffold;
}

const char* scaffoldStatusName(ScaffoldStatus status) noexcept
{
    switch (status) {
    case ScaffoldStatus::Success: return "Success";
    case ScaffoldStatus::InvalidInput: return "InvalidInput";
    case ScaffoldStatus::MissingOuterBoundary: return "MissingOuterBoundary";
    case ScaffoldStatus::MultipleOuterBoundaries: return "MultipleOuterBoundaries";
    case ScaffoldStatus::OuterBoundaryInvalid: return "OuterBoundaryInvalid";
    case ScaffoldStatus::MissingInnerInterface: return "MissingInnerInterface";
    case ScaffoldStatus::MultipleInnerInterfaces: return "MultipleInnerInterfaces";
    case ScaffoldStatus::OpenInnerInterface: return "OpenInnerInterface";
    case ScaffoldStatus::BranchedInnerInterface: return "BranchedInnerInterface";
    case ScaffoldStatus::NonManifoldTransition: return "NonManifoldTransition";
    case ScaffoldStatus::CoreDisconnected: return "CoreDisconnected";
    case ScaffoldStatus::RingDepthInvalid: return "RingDepthInvalid";
    case ScaffoldStatus::AmbiguousTransitionTopology:
        return "AmbiguousTransitionTopology";
    }
    return "Unknown";
}

std::uint64_t sourceTransitionScaffoldSignature(
    const SourceTransitionScaffold& scaffold) noexcept
{
    std::uint64_t hash = 1469598103934665603ULL;
    const auto add = [&hash](std::uint64_t value) {
        hash = fnv(hash, value);
    };
    add(static_cast<std::uint64_t>(scaffold.status));
    add(scaffold.componentId);
    add(scaffold.vertices.size());
    for (const ScaffoldVertex& vertex : scaffold.vertices) {
        add(vertex.localIndex);
        add(vertex.sourceVertexIndex);
        add(static_cast<std::uint64_t>(vertex.sourceVertexId));
        add(doubleBits(vertex.position.x));
        add(doubleBits(vertex.position.y));
        add(doubleBits(vertex.position.z));
        add(static_cast<std::uint64_t>(vertex.classification));
    }
    add(scaffold.edges.size());
    for (const ScaffoldEdge& edge : scaffold.edges) {
        add(edge.localIndex);
        add(edge.sourceEdgeIndex);
        add(static_cast<std::uint64_t>(edge.sourceEdgeId));
        add(edge.vertexIndices[0]);
        add(edge.vertexIndices[1]);
        add(static_cast<std::uint64_t>(edge.classification));
        for (const std::size_t faceIndex : edge.faceIndices) {
            add(faceIndex);
        }
    }
    add(scaffold.faces.size());
    for (const ScaffoldFace& face : scaffold.faces) {
        add(face.localIndex);
        add(face.sourceFaceIndex);
        add(static_cast<std::uint64_t>(face.sourceFaceId));
        add(static_cast<std::uint64_t>(face.transitionRingDepth));
        add(static_cast<std::uint64_t>(face.polygonType));
        for (const std::size_t vertexIndex : face.vertexIndices) {
            add(vertexIndex);
        }
        for (const std::size_t edgeIndex : face.edgeIndices) {
            add(edgeIndex);
        }
    }
    const auto addLoops = [&add](
                              const std::vector<ScaffoldBoundaryLoop>& loops) {
        add(loops.size());
        for (const ScaffoldBoundaryLoop& loop : loops) {
            add(loop.closed ? 1U : 0U);
            add(loop.sourceVertexIndices.size());
            for (const std::size_t index : loop.sourceVertexIndices) {
                add(index);
            }
            for (const std::size_t index : loop.sourceEdgeIndices) {
                add(index);
            }
            for (const SourceId id : loop.sourceVertexIds) {
                add(static_cast<std::uint64_t>(id));
            }
            for (const SourceId id : loop.sourceEdgeIds) {
                add(static_cast<std::uint64_t>(id));
            }
        }
    };
    addLoops(scaffold.fixedOuterBoundaryLoops);
    addLoops(scaffold.innerInterfaceLoops);
    return hash;
}

}  // namespace directional_retopo::solver
