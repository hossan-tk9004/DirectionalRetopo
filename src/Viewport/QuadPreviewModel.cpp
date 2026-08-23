#include "Viewport/QuadPreviewModel.h"

#include <algorithm>
#include <set>
#include <utility>

namespace directional_retopo {
namespace {

MPointArray buildLinePoints(
    const std::vector<QuadPatchResult>& results,
    bool useConformedVertices)
{
    MPointArray points;
    for (const QuadPatchResult& result : results) {
        if (!result.success && !result.debugPreviewAvailable) {
            continue;
        }
        if (useConformedVertices && result.debugInnerResultOnly) {
            continue;
        }
        const std::vector<MPoint>& vertices = useConformedVertices
            ? result.conformedVertices
            : result.rawVertices;
        std::set<std::pair<std::size_t, std::size_t>> uniqueEdges;
        for (const std::vector<std::size_t>& polygon : result.polygons) {
            for (std::size_t index = 0; index < polygon.size(); ++index) {
                const std::size_t first = polygon[index];
                const std::size_t second = polygon[(index + 1U) % polygon.size()];
                if (first >= vertices.size() || second >= vertices.size() ||
                    first == second) {
                    continue;
                }
                uniqueEdges.insert(first < second
                    ? std::make_pair(first, second)
                    : std::make_pair(second, first));
            }
        }
        for (const auto& edge : uniqueEdges) {
            points.append(vertices[edge.first]);
            points.append(vertices[edge.second]);
        }
    }
    return points;
}

MPointArray buildTaggedLinePoints(
    const std::vector<QuadPatchResult>& results,
    bool trianglesOnly)
{
    MPointArray points;
    for (const QuadPatchResult& result : results) {
        if (!result.success) {
            continue;
        }
        for (std::size_t polygonIndex = 0U;
             polygonIndex < result.polygons.size();
             ++polygonIndex) {
            const std::vector<std::size_t>& polygon = result.polygons[polygonIndex];
            const bool selected = trianglesOnly
                ? polygon.size() == 3U
                : (polygonIndex < result.polygonRegions.size() &&
                   result.polygonRegions[polygonIndex] ==
                       ResultPolygonRegion::TransitionCollar);
            if (!selected) {
                continue;
            }
            for (std::size_t edge = 0U; edge < polygon.size(); ++edge) {
                const std::size_t first = polygon[edge];
                const std::size_t second = polygon[(edge + 1U) % polygon.size()];
                if (first < result.conformedVertices.size() &&
                    second < result.conformedVertices.size() && first != second) {
                    points.append(result.conformedVertices[first]);
                    points.append(result.conformedVertices[second]);
                }
            }
        }
    }

    return points;
}
void appendPolyline(
    MPointArray& points,
    const std::vector<MPoint>& polyline,
    bool closed)
{
    if (polyline.size() < 2U) {
        return;
    }
    for (std::size_t index = 1U; index < polyline.size(); ++index) {
        points.append(polyline[index - 1U]);
        points.append(polyline[index]);
    }
    if (closed) {
        points.append(polyline.back());
        points.append(polyline.front());
    }
}

MPointArray buildSourceBoundaryLinePoints(
    const std::vector<QuadPatchResult>& results)
{
    MPointArray points;
    for (const QuadPatchResult& result : results) {
        if (!result.success) {
            continue;
        }
        for (const BoundaryLoopCorrespondence& correspondence :
             result.boundaryCorrespondences) {
            appendPolyline(
                points,
                correspondence.sourcePolylinePositions,
                correspondence.sourceClosed);
        }
    }
    return points;
}

MPointArray buildResultBoundaryLinePoints(
    const std::vector<QuadPatchResult>& results)
{
    MPointArray points;
    for (const QuadPatchResult& result : results) {
        if (!result.success && !result.debugPreviewAvailable) {
            continue;
        }
        for (const ResultBoundaryLoop& loop : result.boundaryLoops) {
            std::vector<MPoint> polyline;
            polyline.reserve(loop.vertexIndices.size());
            for (const std::size_t vertexIndex : loop.vertexIndices) {
                if (vertexIndex < result.conformedVertices.size()) {
                    polyline.push_back(result.conformedVertices[vertexIndex]);
                }
            }
            appendPolyline(points, polyline, loop.closed);
        }
    }
    return points;
}

MPointArray buildBoundaryCorrespondenceLinePoints(
    const std::vector<QuadPatchResult>& results)
{
    MPointArray points;
    for (const QuadPatchResult& result : results) {
        if (!result.success) {
            continue;
        }
        for (const BoundaryLoopCorrespondence& correspondence :
             result.boundaryCorrespondences) {
            for (const BoundaryVertexCorrespondence& vertex :
                 correspondence.vertices) {
                if (vertex.resultVertexIndex >= result.conformedVertices.size()) {
                    continue;
                }
                points.append(result.conformedVertices[vertex.resultVertexIndex]);
                points.append(vertex.sourcePosition);
            }
        }
    }
    return points;
}

MPointArray buildRequiredBoundaryAnchorPoints(
    const std::vector<QuadPatchResult>& results)
{
    MPointArray points;
    for (const QuadPatchResult& result : results) {
        if (!result.success) {
            continue;
        }
        for (const BoundaryLoopCorrespondence& correspondence :
             result.boundaryCorrespondences) {
            for (const RequiredBoundaryAnchor& anchor :
                 correspondence.requiredBoundaryAnchors) {
                points.append(anchor.sourcePosition);
            }
        }
    }
    return points;
}

void sanitize(QuadPreviewVisualizationSettings& settings)
{
    settings.rawWireLineWidth = std::max(settings.rawWireLineWidth, 0.1F);
    settings.rawWireOpacity = std::clamp(settings.rawWireOpacity, 0.0F, 1.0F);
    settings.conformedWireLineWidth =
        std::max(settings.conformedWireLineWidth, 0.1F);
    settings.conformedWireOpacity =
        std::clamp(settings.conformedWireOpacity, 0.0F, 1.0F);
    settings.sourceBoundaryLineWidth =
        std::max(settings.sourceBoundaryLineWidth, 0.1F);
    settings.sourceBoundaryOpacity =
        std::clamp(settings.sourceBoundaryOpacity, 0.0F, 1.0F);
    settings.resultBoundaryLineWidth =
        std::max(settings.resultBoundaryLineWidth, 0.1F);
    settings.resultBoundaryOpacity =
        std::clamp(settings.resultBoundaryOpacity, 0.0F, 1.0F);
    settings.boundaryCorrespondenceLineWidth =
        std::max(settings.boundaryCorrespondenceLineWidth, 0.1F);
    settings.boundaryCorrespondenceOpacity =
        std::clamp(settings.boundaryCorrespondenceOpacity, 0.0F, 1.0F);
    settings.requiredBoundaryAnchorPointSize =
        std::max(settings.requiredBoundaryAnchorPointSize, 1.0F);
    settings.requiredBoundaryAnchorOpacity =
        std::clamp(settings.requiredBoundaryAnchorOpacity, 0.0F, 1.0F);
    settings.transitionCollarLineWidth =
        std::max(settings.transitionCollarLineWidth, 0.1F);
    settings.transitionCollarOpacity =
        std::clamp(settings.transitionCollarOpacity, 0.0F, 1.0F);
    settings.trianglePolygonLineWidth = std::max(settings.trianglePolygonLineWidth, 0.1F);
    settings.trianglePolygonOpacity = std::clamp(settings.trianglePolygonOpacity, 0.0F, 1.0F);
}

}  // namespace

void QuadPreviewModel::clear() noexcept
{
    std::scoped_lock lock(mutex_);
    rawWorldLinePoints_.clear();
    conformedWorldLinePoints_.clear();
    transitionCollarWorldLinePoints_.clear();
    triangleWorldLinePoints_.clear();
    sourceBoundaryWorldLinePoints_.clear();
    resultBoundaryWorldLinePoints_.clear();
    boundaryCorrespondenceWorldLinePoints_.clear();
    requiredBoundaryAnchorWorldPoints_.clear();
    ++generation_;
}

void QuadPreviewModel::setResults(
    const std::vector<QuadPatchResult>& results,
    const QuadPreviewVisualizationSettings& settings)
{
    MPointArray rawPoints = buildLinePoints(results, false);
    MPointArray conformedPoints = buildLinePoints(results, true);
    MPointArray transitionCollarPoints = buildTaggedLinePoints(results, false);
    MPointArray trianglePoints = buildTaggedLinePoints(results, true);
    MPointArray sourceBoundaryPoints = buildSourceBoundaryLinePoints(results);
    MPointArray resultBoundaryPoints = buildResultBoundaryLinePoints(results);
    MPointArray correspondencePoints =
        buildBoundaryCorrespondenceLinePoints(results);
    MPointArray anchorPoints = buildRequiredBoundaryAnchorPoints(results);

    std::scoped_lock lock(mutex_);
    rawWorldLinePoints_ = std::move(rawPoints);
    conformedWorldLinePoints_ = std::move(conformedPoints);
    transitionCollarWorldLinePoints_ = std::move(transitionCollarPoints);
    triangleWorldLinePoints_ = std::move(trianglePoints);
    sourceBoundaryWorldLinePoints_ = std::move(sourceBoundaryPoints);
    resultBoundaryWorldLinePoints_ = std::move(resultBoundaryPoints);
    boundaryCorrespondenceWorldLinePoints_ = std::move(correspondencePoints);
    requiredBoundaryAnchorWorldPoints_ = std::move(anchorPoints);
    settings_ = settings;
    sanitize(settings_);
    ++generation_;
}

void QuadPreviewModel::setSettings(
    const QuadPreviewVisualizationSettings& settings) noexcept
{
    std::scoped_lock lock(mutex_);
    settings_ = settings;
    sanitize(settings_);
    ++generation_;
}

bool QuadPreviewModel::snapshot(QuadPreviewSnapshot& snapshot) const
{
    std::scoped_lock lock(mutex_);
    snapshot.visible = settings_.showQuadPreview &&
        ((settings_.showRawQuadPreview && rawWorldLinePoints_.length() >= 2U) ||
         (settings_.showConformedQuadPreview &&
          conformedWorldLinePoints_.length() >= 2U) ||
         (settings_.showTransitionCollar &&
          transitionCollarWorldLinePoints_.length() >= 2U) ||
         (settings_.showTrianglePolygons &&
          triangleWorldLinePoints_.length() >= 2U) ||
         (settings_.showSourceBoundary &&
          sourceBoundaryWorldLinePoints_.length() >= 2U) ||
         (settings_.showResultBoundary &&
          resultBoundaryWorldLinePoints_.length() >= 2U) ||
         (settings_.showBoundaryCorrespondence &&
          boundaryCorrespondenceWorldLinePoints_.length() >= 2U) ||
         (settings_.showRequiredBoundaryAnchors &&
          requiredBoundaryAnchorWorldPoints_.length() >= 1U));
    snapshot.rawWorldLinePoints = rawWorldLinePoints_;
    snapshot.conformedWorldLinePoints = conformedWorldLinePoints_;
    snapshot.transitionCollarWorldLinePoints = transitionCollarWorldLinePoints_;
    snapshot.triangleWorldLinePoints = triangleWorldLinePoints_;
    snapshot.sourceBoundaryWorldLinePoints = sourceBoundaryWorldLinePoints_;
    snapshot.resultBoundaryWorldLinePoints = resultBoundaryWorldLinePoints_;
    snapshot.boundaryCorrespondenceWorldLinePoints =
        boundaryCorrespondenceWorldLinePoints_;
    snapshot.requiredBoundaryAnchorWorldPoints =
        requiredBoundaryAnchorWorldPoints_;
    snapshot.style = settings_;
    return snapshot.visible;
}

std::uint64_t QuadPreviewModel::generation() const noexcept
{
    std::scoped_lock lock(mutex_);
    return generation_;
}

}  // namespace directional_retopo
