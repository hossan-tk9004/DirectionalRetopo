#include "Remesh/BoundaryCompatibilityDensity.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <vector>

namespace directional_retopo {
namespace {

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    const std::size_t middle = values.size() / 2U;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    double result = values[middle];
    if (values.size() % 2U == 0U) {
        result = (*std::max_element(values.begin(), values.begin() + middle) +
                  result) * 0.5;
    }
    return result;
}

double loopLength(
    const TriangulatedPatch& patch,
    const PatchBoundaryLoop& loop)
{
    if (loop.vertexIndices.size() < 2U) {
        return 0.0;
    }
    double result = 0.0;
    for (std::size_t index = 1U; index < loop.vertexIndices.size(); ++index) {
        result += (
            patch.vertices[loop.vertexIndices[index]].position -
            patch.vertices[loop.vertexIndices[index - 1U]].position).length();
    }
    if (loop.closed) {
        result += (
            patch.vertices[loop.vertexIndices.front()].position -
            patch.vertices[loop.vertexIndices.back()].position).length();
    }
    return result;
}

}  // namespace

double BoundaryCompatibilityDensity::computeCompatibilityTarget(
    double requestedCoreTargetEdgeLength,
    double sourceBoundaryMedianEdgeLength,
    double innerBoundaryArcLength,
    std::size_t sourceBoundaryVertexCount,
    unsigned int topologyBlendWidth) noexcept
{
    if (!(requestedCoreTargetEdgeLength > 0.0) ||
        !std::isfinite(requestedCoreTargetEdgeLength)) {
        return 0.0;
    }
    const double blend =
        static_cast<double>(std::max(topologyBlendWidth, 1U));
    const std::size_t desiredInterfaceSegments = std::max<std::size_t>(
        3U,
        static_cast<std::size_t>(std::ceil(
            static_cast<double>(std::max<std::size_t>(
                sourceBoundaryVertexCount,
                3U)) / blend)));
    const double arcSamplingTarget = innerBoundaryArcLength > 0.0
        ? innerBoundaryArcLength /
            static_cast<double>(desiredInterfaceSegments)
        : sourceBoundaryMedianEdgeLength;
    const double compatibilityTarget = std::max(
        sourceBoundaryMedianEdgeLength,
        arcSamplingTarget);
    return compatibilityTarget > 0.0
        ? std::min(
            requestedCoreTargetEdgeLength,
            compatibilityTarget)
        : requestedCoreTargetEdgeLength;
}

BoundaryCompatibilityDensityResult BoundaryCompatibilityDensity::build(
    const TriangulatedPatch& completeSourcePatch,
    const TriangulatedPatch& innerPatch,
    const MeshTopologyCache& topology,
    const DensityFieldData& requestedDensity,
    unsigned int topologyBlendWidth)
{
    BoundaryCompatibilityDensityResult result;
    result.densityField = requestedDensity;
    result.interfaceBandRings = std::max(topologyBlendWidth, 1U);
    if (innerPatch.empty() || requestedDensity.empty()) {
        result.diagnosticMessage =
            "Compatibility Density requires a valid Inner Patch and Density Field.";
        return result;
    }

    std::unordered_set<int> innerFaces;
    std::vector<double> requestedTargets;
    for (const PatchTriangle& triangle : innerPatch.triangles) {
        if (!innerFaces.insert(triangle.sourceFaceId).second) {
            continue;
        }
        const FaceDensity* density =
            requestedDensity.face(triangle.sourceFaceId);
        if (density != nullptr && density->valid &&
            std::isfinite(density->targetEdgeLength) &&
            density->targetEdgeLength > 0.0) {
            requestedTargets.push_back(density->targetEdgeLength);
        }
    }
    result.requestedCoreTargetEdgeLength =
        median(std::move(requestedTargets));
    if (!(result.requestedCoreTargetEdgeLength > 0.0)) {
        result.diagnosticMessage =
            "Compatibility Density found no valid requested Core target.";
        return result;
    }

    std::vector<double> sourceBoundaryLengths;
    for (const PatchBoundaryEdge& edge : completeSourcePatch.boundaryEdges) {
        if (edge.vertexIndices[0] >= completeSourcePatch.vertices.size() ||
            edge.vertexIndices[1] >= completeSourcePatch.vertices.size()) {
            continue;
        }
        const double length = (
            completeSourcePatch.vertices[edge.vertexIndices[1]].position -
            completeSourcePatch.vertices[edge.vertexIndices[0]].position).length();
        if (std::isfinite(length) && length > 0.0) {
            sourceBoundaryLengths.push_back(length);
        }
    }
    result.sourceBoundaryMedianEdgeLength =
        median(std::move(sourceBoundaryLengths));

    std::size_t sourceBoundaryVertexCount = 0U;
    for (const PatchBoundaryLoop& loop : completeSourcePatch.boundaryLoops) {
        sourceBoundaryVertexCount += loop.vertexIndices.size();
    }
    double innerBoundaryLength = 0.0;
    for (const PatchBoundaryLoop& loop : innerPatch.boundaryLoops) {
        innerBoundaryLength += loopLength(innerPatch, loop);
    }

    result.effectiveInterfaceTargetEdgeLength =
        computeCompatibilityTarget(
            result.requestedCoreTargetEdgeLength,
            result.sourceBoundaryMedianEdgeLength,
            innerBoundaryLength,
            sourceBoundaryVertexCount,
            topologyBlendWidth);

    std::unordered_set<int> bandFaces;
    std::vector<int> frontier;
    for (const PatchBoundaryEdge& edge : innerPatch.boundaryEdges) {
        if (edge.sourceEdgeId < 0 ||
            static_cast<std::size_t>(edge.sourceEdgeId) >=
                topology.edges().size()) {
            continue;
        }
        for (const int faceId :
             topology.edges()[static_cast<std::size_t>(edge.sourceEdgeId)].faceIds) {
            if (innerFaces.find(faceId) != innerFaces.end() &&
                bandFaces.insert(faceId).second) {
                frontier.push_back(faceId);
            }
        }
    }

    for (unsigned int ring = 1U;
         ring < result.interfaceBandRings && !frontier.empty();
         ++ring) {
        std::vector<int> nextFrontier;
        for (const int faceId : frontier) {
            if (faceId < 0 ||
                static_cast<std::size_t>(faceId) >= topology.faces().size()) {
                continue;
            }
            for (const int adjacent :
                 topology.faces()[static_cast<std::size_t>(faceId)]
                     .adjacentFaceIds) {
                if (innerFaces.find(adjacent) == innerFaces.end() ||
                    !bandFaces.insert(adjacent).second) {
                    continue;
                }
                nextFrontier.push_back(adjacent);
            }
        }
        frontier = std::move(nextFrontier);
    }

    for (const int faceId : bandFaces) {
        if (faceId < 0 ||
            static_cast<std::size_t>(faceId) >=
                result.densityField.perFace.size()) {
            continue;
        }
        FaceDensity& density =
            result.densityField.perFace[static_cast<std::size_t>(faceId)];
        if (!density.valid) {
            continue;
        }
        density.targetEdgeLength = std::min(
            density.targetEdgeLength,
            result.effectiveInterfaceTargetEdgeLength);
    }
    result.interfaceBandFaceCount = bandFaces.size();
    result.success = true;

    std::ostringstream message;
    message << "Requested Core target "
            << result.requestedCoreTargetEdgeLength
            << "; effective Interface target "
            << result.effectiveInterfaceTargetEdgeLength
            << "; source Boundary median "
            << result.sourceBoundaryMedianEdgeLength
            << "; compatibility band " << result.interfaceBandRings
            << " rings/" << result.interfaceBandFaceCount
            << " faces. Core-center Density remains unchanged.";
    result.diagnosticMessage = message.str();
    return result;
}

}  // namespace directional_retopo
