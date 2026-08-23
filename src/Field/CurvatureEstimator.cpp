#include "Field/CurvatureEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace directional_retopo {
namespace {

constexpr double kPi = 3.14159265358979323846;

double radians(double degrees)
{
    return degrees * kPi / 180.0;
}

double median(std::vector<double> values)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    return values.size() % 2U == 0U
        ? (values[middle - 1U] + values[middle]) * 0.5
        : values[middle];
}

}  // namespace

const CurvatureEstimatorSettings& CurvatureEstimator::settings() const noexcept
{
    return settings_;
}

void CurvatureEstimator::setSettings(
    const CurvatureEstimatorSettings& settings) noexcept
{
    settings_ = settings;
    settings_.desiredNormalVariationDegrees = std::clamp(
        settings_.desiredNormalVariationDegrees,
        0.1,
        90.0);
    settings_.minimumSignificantDihedralDegrees = std::clamp(
        settings_.minimumSignificantDihedralDegrees,
        0.0,
        settings_.desiredNormalVariationDegrees);
    settings_.rmsWeight = std::max(settings_.rmsWeight, 0.0);
    settings_.peakWeight = std::max(settings_.peakWeight, 0.0);
    const double weightSum = settings_.rmsWeight + settings_.peakWeight;
    if (weightSum <= 1.0e-12) {
        settings_.rmsWeight = 1.0;
        settings_.peakWeight = 0.0;
    } else {
        settings_.rmsWeight /= weightSum;
        settings_.peakWeight /= weightSum;
    }
    settings_.neighborSpreadIterations =
        std::max(settings_.neighborSpreadIterations, 0);
    settings_.neighborSpreadStrength = std::clamp(
        settings_.neighborSpreadStrength,
        0.0,
        1.0);
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 0.0);
}

bool CurvatureEstimator::estimate(
    const MeshTopologyCache& topology,
    std::vector<FaceCurvature>& output,
    CurvatureEstimateMetrics* metrics) const
{
    return estimate(topology.faces(), topology.edges(), output, metrics);
}

bool CurvatureEstimator::estimate(
    const std::vector<MeshFaceTopology>& faces,
    const std::vector<MeshEdgeTopology>& edges,
    std::vector<FaceCurvature>& output,
    CurvatureEstimateMetrics* metrics) const
{
    output.assign(faces.size(), FaceCurvature());
    CurvatureEstimateMetrics localMetrics;
    if (faces.empty() || edges.empty()) {
        if (metrics != nullptr) {
            *metrics = localMetrics;
        }
        return false;
    }

    const double significantAngle = radians(
        settings_.minimumSignificantDihedralDegrees);
    for (std::size_t faceIndex = 0U; faceIndex < faces.size(); ++faceIndex) {
        const MeshFaceTopology& face = faces[faceIndex];
        if (!face.worldGeometryValid ||
            face.worldNormal.length() <= settings_.geometryEpsilon) {
            continue;
        }
        std::vector<double> edgeLengths;
        std::vector<double> dihedrals;
        std::vector<double> curvatureSamples;
        edgeLengths.reserve(face.edgeIds.size());
        dihedrals.reserve(face.edgeIds.size());
        curvatureSamples.reserve(face.edgeIds.size());
        for (const int edgeId : face.edgeIds) {
            if (edgeId < 0 || static_cast<std::size_t>(edgeId) >= edges.size()) {
                continue;
            }
            const MeshEdgeTopology& edge = edges[static_cast<std::size_t>(edgeId)];
            if (!std::isfinite(edge.worldLength) ||
                edge.worldLength <= settings_.geometryEpsilon) {
                continue;
            }
            edgeLengths.push_back(edge.worldLength);
            for (const int adjacentFaceId : edge.faceIds) {
                if (adjacentFaceId < 0 ||
                    static_cast<std::size_t>(adjacentFaceId) >= faces.size() ||
                    static_cast<std::size_t>(adjacentFaceId) == faceIndex) {
                    continue;
                }
                const MeshFaceTopology& adjacent =
                    faces[static_cast<std::size_t>(adjacentFaceId)];
                if (!adjacent.worldGeometryValid ||
                    adjacent.worldNormal.length() <= settings_.geometryEpsilon) {
                    continue;
                }
                const double angle = std::acos(std::clamp(
                    face.worldNormal * adjacent.worldNormal,
                    -1.0,
                    1.0));
                if (!std::isfinite(angle)) {
                    continue;
                }
                dihedrals.push_back(angle);
                curvatureSamples.push_back(
                    angle >= significantAngle ? angle / edge.worldLength : 0.0);
            }
        }
        if (edgeLengths.empty() || curvatureSamples.empty()) {
            continue;
        }

        double squaredSum = 0.0;
        double peak = 0.0;
        for (const double sample : curvatureSamples) {
            squaredSum += sample * sample;
            peak = std::max(peak, sample);
        }
        const double rms = std::sqrt(
            squaredSum / static_cast<double>(curvatureSamples.size()));
        FaceCurvature& curvature = output[faceIndex];
        curvature.indicator = settings_.rmsWeight * rms +
            settings_.peakWeight * peak;
        curvature.localEdgeLength = median(edgeLengths);
        curvature.maximumDihedralRadians = dihedrals.empty()
            ? 0.0
            : *std::max_element(dihedrals.begin(), dihedrals.end());
        curvature.meanDihedralRadians = dihedrals.empty()
            ? 0.0
            : std::accumulate(dihedrals.begin(), dihedrals.end(), 0.0) /
                static_cast<double>(dihedrals.size());
        curvature.valid = std::isfinite(curvature.indicator) &&
            std::isfinite(curvature.localEdgeLength) &&
            curvature.localEdgeLength > settings_.geometryEpsilon;
    }

    for (int iteration = 0;
         iteration < settings_.neighborSpreadIterations;
         ++iteration) {
        const std::vector<FaceCurvature> source = output;
        for (std::size_t faceIndex = 0U; faceIndex < faces.size(); ++faceIndex) {
            if (!source[faceIndex].valid) {
                continue;
            }
            double neighborSum = 0.0;
            std::size_t neighborCount = 0U;
            for (const int adjacentFaceId : faces[faceIndex].adjacentFaceIds) {
                if (adjacentFaceId >= 0 &&
                    static_cast<std::size_t>(adjacentFaceId) < source.size() &&
                    source[static_cast<std::size_t>(adjacentFaceId)].valid) {
                    neighborSum += source[static_cast<std::size_t>(
                        adjacentFaceId)].indicator;
                    ++neighborCount;
                }
            }
            if (neighborCount == 0U) {
                continue;
            }
            const double neighborMean =
                neighborSum / static_cast<double>(neighborCount);
            const double spread = source[faceIndex].indicator *
                    (1.0 - settings_.neighborSpreadStrength) +
                neighborMean * settings_.neighborSpreadStrength;
            // Preserve real peaks while spreading part of them by one ring.
            output[faceIndex].indicator = std::max(
                source[faceIndex].indicator,
                spread);
        }
    }

    double indicatorSum = 0.0;
    localMetrics.minimumIndicator = std::numeric_limits<double>::infinity();
    for (const FaceCurvature& curvature : output) {
        if (!curvature.valid) {
            continue;
        }
        ++localMetrics.validFaceCount;
        indicatorSum += curvature.indicator;
        localMetrics.minimumIndicator = std::min(
            localMetrics.minimumIndicator,
            curvature.indicator);
        localMetrics.maximumIndicator = std::max(
            localMetrics.maximumIndicator,
            curvature.indicator);
        if (curvature.indicator > settings_.geometryEpsilon) {
            ++localMetrics.significantFaceCount;
        }
    }
    if (!std::isfinite(localMetrics.minimumIndicator)) {
        localMetrics.minimumIndicator = 0.0;
    }
    localMetrics.meanIndicator = localMetrics.validFaceCount > 0U
        ? indicatorSum / static_cast<double>(localMetrics.validFaceCount)
        : 0.0;
    if (metrics != nullptr) {
        *metrics = localMetrics;
    }
    return localMetrics.validFaceCount > 0U;
}

}  // namespace directional_retopo
