#include "Field/DensityFieldBuilder.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace directional_retopo {
namespace {

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    return values.size() % 2 == 0
        ? (values[middle - 1] + values[middle]) * 0.5
        : values[middle];
}

std::vector<double> robustLengths(
    const std::vector<double>& values,
    std::size_t minimumCount,
    double madMultiplier,
    double epsilon)
{
    std::vector<double> finiteValues;
    finiteValues.reserve(values.size());
    for (const double value : values) {
        if (std::isfinite(value) && value > epsilon) {
            finiteValues.push_back(value);
        }
    }
    if (finiteValues.size() < minimumCount) {
        return finiteValues;
    }

    const double center = median(finiteValues);
    std::vector<double> deviations;
    deviations.reserve(finiteValues.size());
    for (const double value : finiteValues) {
        deviations.push_back(std::abs(value - center));
    }
    const double mad = median(deviations);
    if (mad <= epsilon || !std::isfinite(mad)) {
        return finiteValues;
    }

    const double robustSigma = 1.4826 * mad;
    const double maximumDeviation = madMultiplier * robustSigma;
    std::vector<double> filtered;
    filtered.reserve(finiteValues.size());
    for (const double value : finiteValues) {
        if (std::abs(value - center) <= maximumDeviation) {
            filtered.push_back(value);
        }
    }
    return filtered.size() >= minimumCount ? filtered : finiteValues;
}

struct ComponentReference final
{
    double edgeLength = 0.0;
    DensityFallback fallback = DensityFallback::None;
    std::vector<int> referenceEdgeIds;
};

}  // namespace

const DensityFieldBuilderSettings& DensityFieldBuilder::settings() const noexcept
{
    return settings_;
}

void DensityFieldBuilder::setSettings(
    const DensityFieldBuilderSettings& settings) noexcept
{
    settings_ = settings;
    settings_.minimumUsableEdgeLength =
        std::max(settings_.minimumUsableEdgeLength, 0.0);
    settings_.manualTargetEdgeLength =
        std::max(settings_.manualTargetEdgeLength, settings_.minimumUsableEdgeLength);
    settings_.edgeLengthScale =
        std::max(settings_.edgeLengthScale, settings_.minimumUsableEdgeLength);
    settings_.outsideReferenceFaceRings =
        std::max(settings_.outsideReferenceFaceRings, 0);
    settings_.minimumReferenceEdgeCount =
        std::max<std::size_t>(settings_.minimumReferenceEdgeCount, 1);
    settings_.outlierMadMultiplier = std::max(settings_.outlierMadMultiplier, 0.0);
    settings_.transitionBoundaryBlend =
        std::clamp(settings_.transitionBoundaryBlend, 0.0, 1.0);
    settings_.boundaryBlend = std::clamp(settings_.boundaryBlend, 0.0, 1.0);
    settings_.minimumTargetEdgeLength = std::max(
        settings_.minimumTargetEdgeLength,
        settings_.minimumUsableEdgeLength);
    settings_.maximumCurvatureRefinementFactor = std::max(
        settings_.maximumCurvatureRefinementFactor,
        1.0);
    curvatureEstimator_.setSettings(settings_.curvature);
    settings_.curvature = curvatureEstimator_.settings();
}

bool DensityFieldBuilder::build(
    const PaintRegionData& region,
    const MeshTopologyCache& topology,
    DensityFieldData& output,
    DensityFieldBuildMetrics* metrics) const
{
    return build(
        region,
        topology.faces(),
        topology.edges(),
        output,
        metrics);
}

bool DensityFieldBuilder::build(
    const PaintRegionData& region,
    const std::vector<MeshFaceTopology>& faces,
    const std::vector<MeshEdgeTopology>& edges,
    DensityFieldData& output,
    DensityFieldBuildMetrics* metrics) const
{
    output.clear();
    DensityFieldBuildMetrics localMetrics;
    localMetrics.mode = settings_.mode;
    localMetrics.edgeLengthScale = settings_.edgeLengthScale;
    if (region.components.empty() || faces.empty() || edges.empty()) {
        if (metrics != nullptr) {
            *metrics = localMetrics;
        }
        return false;
    }

    output.mode = settings_.mode;
    output.perFace.resize(faces.size());
    std::vector<unsigned char> regionFaceFlags(faces.size(), 0U);
    std::vector<int> componentByFace(faces.size(), -1);
    std::vector<unsigned char> coreFaceFlags(faces.size(), 0U);
    std::vector<unsigned char> boundaryFaceFlags(faces.size(), 0U);
    for (std::size_t componentIndex = 0;
         componentIndex < region.components.size();
         ++componentIndex) {
        const PaintRegionComponent& component = region.components[componentIndex];
        for (const int faceId : component.allFaceIds) {
            if (faceId >= 0 && static_cast<std::size_t>(faceId) < faces.size()) {
                regionFaceFlags[static_cast<std::size_t>(faceId)] = 1U;
                componentByFace[static_cast<std::size_t>(faceId)] =
                    static_cast<int>(componentIndex);
            }
        }
        for (const int faceId : component.coreFaceIds) {
            if (faceId >= 0 && static_cast<std::size_t>(faceId) < faces.size()) {
                coreFaceFlags[static_cast<std::size_t>(faceId)] = 1U;
            }
        }
        for (const BoundaryEdge& boundaryEdge : component.boundaryEdges) {
            if (boundaryEdge.insideFaceId >= 0 &&
                static_cast<std::size_t>(boundaryEdge.insideFaceId) < faces.size()) {
                boundaryFaceFlags[static_cast<std::size_t>(
                    boundaryEdge.insideFaceId)] = 1U;
            }
        }
    }

    std::vector<ComponentReference> componentReferences(region.components.size());
    std::unordered_set<int> allReferenceEdgeIds;
    DensityFallback worstFallback = DensityFallback::None;
    for (std::size_t componentIndex = 0;
         componentIndex < region.components.size();
         ++componentIndex) {
        const PaintRegionComponent& component = region.components[componentIndex];
        ComponentReference& reference = componentReferences[componentIndex];
        std::unordered_set<int> outsideFaces;
        std::deque<std::pair<int, int>> pendingOutsideFaces;
        for (const BoundaryEdge& boundaryEdge : component.boundaryEdges) {
            if (boundaryEdge.edgeId < 0 ||
                static_cast<std::size_t>(boundaryEdge.edgeId) >= edges.size()) {
                continue;
            }
            for (const int faceId : edges[static_cast<std::size_t>(
                     boundaryEdge.edgeId)].faceIds) {
                if (faceId >= 0 && static_cast<std::size_t>(faceId) < faces.size() &&
                    regionFaceFlags[static_cast<std::size_t>(faceId)] == 0U &&
                    outsideFaces.insert(faceId).second) {
                    pendingOutsideFaces.emplace_back(faceId, 1);
                }
            }
        }

        std::unordered_set<int> referenceEdgeSet;
        while (!pendingOutsideFaces.empty()) {
            const auto [faceId, ring] = pendingOutsideFaces.front();
            pendingOutsideFaces.pop_front();
            for (const int edgeId : faces[static_cast<std::size_t>(faceId)].edgeIds) {
                referenceEdgeSet.insert(edgeId);
            }
            if (ring >= settings_.outsideReferenceFaceRings) {
                continue;
            }
            for (const int adjacentFaceId :
                 faces[static_cast<std::size_t>(faceId)].adjacentFaceIds) {
                if (adjacentFaceId >= 0 &&
                    static_cast<std::size_t>(adjacentFaceId) < faces.size() &&
                    regionFaceFlags[static_cast<std::size_t>(adjacentFaceId)] == 0U &&
                    outsideFaces.insert(adjacentFaceId).second) {
                    pendingOutsideFaces.emplace_back(adjacentFaceId, ring + 1);
                }
            }
        }

        auto lengthsForEdges = [&edges, this](const std::unordered_set<int>& edgeIds) {
            std::vector<double> lengths;
            lengths.reserve(edgeIds.size());
            for (const int edgeId : edgeIds) {
                if (edgeId >= 0 && static_cast<std::size_t>(edgeId) < edges.size()) {
                    const double length = edges[static_cast<std::size_t>(edgeId)].worldLength;
                    if (std::isfinite(length) &&
                        length > settings_.minimumUsableEdgeLength) {
                        lengths.push_back(length);
                    }
                }
            }
            return lengths;
        };

        std::vector<double> lengths = robustLengths(
            lengthsForEdges(referenceEdgeSet),
            settings_.minimumReferenceEdgeCount,
            settings_.outlierMadMultiplier,
            settings_.minimumUsableEdgeLength);
        if (lengths.size() < settings_.minimumReferenceEdgeCount) {
            reference.fallback = DensityFallback::BoundaryEdges;
            referenceEdgeSet.clear();
            for (const BoundaryEdge& boundaryEdge : component.boundaryEdges) {
                if (boundaryEdge.edgeId >= 0 &&
                    static_cast<std::size_t>(boundaryEdge.edgeId) < edges.size()) {
                    referenceEdgeSet.insert(boundaryEdge.edgeId);
                }
            }
            lengths = robustLengths(
                lengthsForEdges(referenceEdgeSet),
                1,
                settings_.outlierMadMultiplier,
                settings_.minimumUsableEdgeLength);
        }
        if (lengths.empty()) {
            reference.fallback = DensityFallback::LocalRegionEdges;
            referenceEdgeSet.clear();
            for (const int faceId : component.allFaceIds) {
                if (faceId < 0 || static_cast<std::size_t>(faceId) >= faces.size()) {
                    continue;
                }
                for (const int edgeId : faces[static_cast<std::size_t>(faceId)].edgeIds) {
                    referenceEdgeSet.insert(edgeId);
                }
            }
            lengths = robustLengths(
                lengthsForEdges(referenceEdgeSet),
                1,
                settings_.outlierMadMultiplier,
                settings_.minimumUsableEdgeLength);
        }
        if (lengths.empty()) {
            reference.fallback = DensityFallback::ManualDefault;
            reference.edgeLength = settings_.manualTargetEdgeLength;
            referenceEdgeSet.clear();
        } else {
            reference.edgeLength = median(lengths);
        }
        if (!std::isfinite(reference.edgeLength) ||
            reference.edgeLength <= settings_.minimumUsableEdgeLength) {
            reference.edgeLength = settings_.manualTargetEdgeLength;
            reference.fallback = DensityFallback::ManualDefault;
        }
        reference.referenceEdgeIds.assign(
            referenceEdgeSet.begin(),
            referenceEdgeSet.end());
        for (const int edgeId : reference.referenceEdgeIds) {
            allReferenceEdgeIds.insert(edgeId);
        }
        worstFallback = static_cast<int>(reference.fallback) >
                static_cast<int>(worstFallback)
            ? reference.fallback
            : worstFallback;
    }

    localMetrics.referenceEdgeCount = allReferenceEdgeIds.size();
    localMetrics.fallback = worstFallback;
    std::vector<double> allReferenceLengths;
    allReferenceLengths.reserve(allReferenceEdgeIds.size());
    for (const int edgeId : allReferenceEdgeIds) {
        if (edgeId >= 0 && static_cast<std::size_t>(edgeId) < edges.size()) {
            allReferenceLengths.push_back(edges[static_cast<std::size_t>(edgeId)].worldLength);
        }
    }
    allReferenceLengths = robustLengths(
        allReferenceLengths,
        1,
        settings_.outlierMadMultiplier,
        settings_.minimumUsableEdgeLength);
    localMetrics.medianReferenceEdgeLength = allReferenceLengths.empty()
        ? settings_.manualTargetEdgeLength
        : median(allReferenceLengths);

    std::vector<FaceCurvature> faceCurvatures;
    CurvatureEstimateMetrics curvatureMetrics;
    (void)curvatureEstimator_.estimate(
        faces,
        edges,
        faceCurvatures,
        &curvatureMetrics);
    localMetrics.maximumCurvatureIndicator =
        curvatureMetrics.maximumIndicator;
    localMetrics.meanCurvatureIndicator = curvatureMetrics.meanIndicator;

    std::vector<int> transitionDepth(faces.size(), -1);
    std::vector<int> maximumDepth(region.components.size(), 0);
    std::deque<int> depthQueue;
    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        if (coreFaceFlags[faceIndex] != 0U) {
            transitionDepth[faceIndex] = 0;
            depthQueue.push_back(static_cast<int>(faceIndex));
        }
    }
    while (!depthQueue.empty()) {
        const int faceId = depthQueue.front();
        depthQueue.pop_front();
        const int componentId = componentByFace[static_cast<std::size_t>(faceId)];
        for (const int adjacentFaceId :
             faces[static_cast<std::size_t>(faceId)].adjacentFaceIds) {
            if (adjacentFaceId < 0 ||
                static_cast<std::size_t>(adjacentFaceId) >= faces.size() ||
                componentByFace[static_cast<std::size_t>(adjacentFaceId)] != componentId ||
                transitionDepth[static_cast<std::size_t>(adjacentFaceId)] >= 0) {
                continue;
            }
            const int nextDepth =
                transitionDepth[static_cast<std::size_t>(faceId)] + 1;
            transitionDepth[static_cast<std::size_t>(adjacentFaceId)] = nextDepth;
            if (componentId >= 0) {
                maximumDepth[static_cast<std::size_t>(componentId)] = std::max(
                    maximumDepth[static_cast<std::size_t>(componentId)],
                    nextDepth);
            }
            depthQueue.push_back(adjacentFaceId);
        }
    }

    double minimumTarget = std::numeric_limits<double>::infinity();
    double targetSum = 0.0;
    double maximumTarget = 0.0;
    double minimumCurvatureTarget = std::numeric_limits<double>::infinity();
    double curvatureSourceEdgeLengthSum = 0.0;
    for (std::size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {
        const int componentId = componentByFace[faceIndex];
        if (componentId < 0) {
            continue;
        }
        const ComponentReference& reference =
            componentReferences[static_cast<std::size_t>(componentId)];
        const double surroundingTarget =
            reference.edgeLength * settings_.edgeLengthScale;
        double coreTarget = settings_.mode == DensityMode::Auto
            ? surroundingTarget
            : settings_.manualTargetEdgeLength * settings_.edgeLengthScale;
        double curvatureTarget = surroundingTarget;
        double curvatureIndicator = 0.0;
        double localSurfaceEdgeLength = 0.0;
        bool curvatureLimited = false;
        if (settings_.mode == DensityMode::Auto &&
            faceIndex < faceCurvatures.size() &&
            faceCurvatures[faceIndex].valid) {
            const FaceCurvature& curvature = faceCurvatures[faceIndex];
            curvatureIndicator = curvature.indicator;
            localSurfaceEdgeLength = curvature.localEdgeLength;
            if (curvatureIndicator > settings_.minimumUsableEdgeLength) {
                constexpr double kPi = 3.14159265358979323846;
                const double desiredVariation =
                    settings_.curvature.desiredNormalVariationDegrees *
                    kPi / 180.0;
                const double unconstrainedCurvatureTarget =
                    desiredVariation / curvatureIndicator;
                const double relativeFloor = surroundingTarget /
                    settings_.maximumCurvatureRefinementFactor;
                curvatureTarget = std::max({
                    unconstrainedCurvatureTarget,
                    relativeFloor,
                    settings_.minimumTargetEdgeLength});
                curvatureTarget = std::min(curvatureTarget, surroundingTarget);
                curvatureLimited = curvatureTarget < surroundingTarget *
                    (1.0 - 1.0e-6);
                if (curvatureLimited) {
                    coreTarget = curvatureTarget;
                }
            }
        }
        const int componentMaximumDepth =
            maximumDepth[static_cast<std::size_t>(componentId)];
        const double transitionRatio = componentMaximumDepth > 0 &&
            transitionDepth[faceIndex] > 0
            ? std::clamp(
                  static_cast<double>(transitionDepth[faceIndex]) /
                      static_cast<double>(componentMaximumDepth),
                  0.0,
                  1.0)
            : 0.0;
        double blend = transitionRatio * settings_.transitionBoundaryBlend;
        if (boundaryFaceFlags[faceIndex] != 0U) {
            blend = std::max(blend, settings_.boundaryBlend);
        }
        blend = std::clamp(blend, 0.0, 1.0);

        FaceDensity& density = output.perFace[faceIndex];
        density.referenceEdgeLength = reference.edgeLength;
        density.baseTargetEdgeLength = surroundingTarget;
        density.curvatureTargetEdgeLength = curvatureTarget;
        density.curvatureIndicator = curvatureIndicator;
        density.localSurfaceEdgeLength = localSurfaceEdgeLength;
        const double transitionTarget =
            coreTarget * (1.0 - blend) + surroundingTarget * blend;
        // Existing-topology blending must not undo the geometric sampling cap:
        // even a Boundary face needs enough samples to represent its curvature.
        density.targetEdgeLength = settings_.mode == DensityMode::Auto
            ? std::min(transitionTarget, curvatureTarget)
            : transitionTarget;
        density.curvatureLimited = curvatureLimited &&
            density.targetEdgeLength < surroundingTarget * (1.0 - 1.0e-6);
        density.curvatureRefinementFactor = density.targetEdgeLength >
                settings_.minimumUsableEdgeLength
            ? surroundingTarget / density.targetEdgeLength
            : 1.0;
        density.autoDerived = settings_.mode == DensityMode::Auto;
        density.valid = std::isfinite(density.targetEdgeLength) &&
            density.targetEdgeLength > settings_.minimumUsableEdgeLength;
        if (!density.valid) {
            continue;
        }
        density.scaleU = density.targetEdgeLength / reference.edgeLength;
        density.scaleV = density.scaleU;
        minimumTarget = std::min(minimumTarget, density.targetEdgeLength);
        targetSum += density.targetEdgeLength;
        maximumTarget = std::max(maximumTarget, density.targetEdgeLength);
        if (density.curvatureLimited) {
            ++localMetrics.curvatureConstrainedFaceCount;
            minimumCurvatureTarget = std::min(
                minimumCurvatureTarget,
                density.targetEdgeLength);
            curvatureSourceEdgeLengthSum += density.localSurfaceEdgeLength;
            localMetrics.maximumAppliedCurvatureRefinementFactor = std::max(
                localMetrics.maximumAppliedCurvatureRefinementFactor,
                density.curvatureRefinementFactor);
        }
    }

    localMetrics.minimumTargetEdgeLength = std::isfinite(minimumTarget)
        ? minimumTarget
        : 0.0;
    const std::size_t validTargetCount = static_cast<std::size_t>(std::count_if(
        output.perFace.begin(),
        output.perFace.end(),
        [](const FaceDensity& density) { return density.valid; }));
    localMetrics.meanTargetEdgeLength = validTargetCount > 0U
        ? targetSum / static_cast<double>(validTargetCount)
        : 0.0;
    localMetrics.maximumTargetEdgeLength = maximumTarget;
    localMetrics.minimumCurvatureTargetEdgeLength =
        std::isfinite(minimumCurvatureTarget) ? minimumCurvatureTarget : 0.0;
    localMetrics.meanCurvatureRegionSourceEdgeLength =
        localMetrics.curvatureConstrainedFaceCount > 0U
        ? curvatureSourceEdgeLengthSum /
            static_cast<double>(localMetrics.curvatureConstrainedFaceCount)
        : 0.0;
    if (metrics != nullptr) {
        *metrics = localMetrics;
    }
    return std::isfinite(minimumTarget) && maximumTarget > 0.0;
}

}  // namespace directional_retopo
