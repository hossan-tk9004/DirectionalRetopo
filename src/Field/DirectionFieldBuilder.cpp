#include "Field/DirectionFieldBuilder.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <vector>

namespace directional_retopo {
namespace {

struct FaceQueueEntry final
{
    double distance = 0.0;
    int localFaceIndex = -1;
};

struct FartherFaceFirst final
{
    bool operator()(const FaceQueueEntry& left, const FaceQueueEntry& right) const
    {
        return left.distance > right.distance;
    }
};

bool validatesFrame(
    const MVector& normal,
    const MVector& uDirection,
    const MVector& vDirection,
    double tolerance)
{
    return std::abs(normal.length() - 1.0) <= tolerance &&
        std::abs(uDirection.length() - 1.0) <= tolerance &&
        std::abs(vDirection.length() - 1.0) <= tolerance &&
        std::abs(uDirection * normal) <= tolerance &&
        std::abs(vDirection * normal) <= tolerance &&
        std::abs(uDirection * vDirection) <= tolerance;
}

}  // namespace

const DirectionFieldBuilderSettings& DirectionFieldBuilder::settings() const noexcept
{
    return settings_;
}

void DirectionFieldBuilder::setSettings(
    const DirectionFieldBuilderSettings& settings) noexcept
{
    settings_ = settings;
    settings_.directionSmoothStrength =
        std::clamp(settings_.directionSmoothStrength, 0.0, 1.0);
    settings_.directHitConstraintWeight =
        std::clamp(settings_.directHitConstraintWeight, 0.0, 1.0);
    settings_.minimumCoreInfluenceForConstraint =
        std::clamp(settings_.minimumCoreInfluenceForConstraint, 0.0, 1.0);
    settings_.coreTopologyGuidanceWeight =
        std::clamp(settings_.coreTopologyGuidanceWeight, 0.0, 1.0);
    settings_.transitionTopologyGuidanceWeight =
        std::clamp(settings_.transitionTopologyGuidanceWeight, 0.0, 1.0);
    settings_.boundaryTopologyGuidanceWeight =
        std::clamp(settings_.boundaryTopologyGuidanceWeight, 0.0, 1.0);
    settings_.singularityEpsilon = std::max(settings_.singularityEpsilon, 0.0);
    settings_.geometryEpsilon = std::max(settings_.geometryEpsilon, 0.0);
    settings_.validationTolerance = std::max(settings_.validationTolerance, 0.0);
}

bool DirectionFieldBuilder::build(
    const StrokeData& processedStroke,
    const PaintRegionData& region,
    const MeshTopologyCache& topology,
    DirectionFieldData& output,
    DirectionFieldBuildMetrics* metrics) const
{
    output.clear();
    DirectionFieldBuildMetrics localMetrics;
    const auto& faces = topology.faces();
    if (processedStroke.empty() || region.components.empty() || faces.empty()) {
        if (metrics != nullptr) {
            *metrics = localMetrics;
        }
        return false;
    }

    output.perFace.resize(faces.size());
    const std::vector<FaceTangentBasis> bases = CrossFieldMath::buildFaceBases(
        topology,
        settings_.geometryEpsilon);
    std::vector<int> componentByFace(faces.size(), -1);
    std::vector<unsigned char> coreFaceFlags(faces.size(), 0U);
    std::vector<unsigned char> boundaryFaceFlags(faces.size(), 0U);
    std::vector<int> regionFaceIds;
    regionFaceIds.reserve(region.totalFaceCount());

    for (std::size_t componentIndex = 0;
         componentIndex < region.components.size();
         ++componentIndex) {
        const PaintRegionComponent& component = region.components[componentIndex];
        for (const int faceId : component.allFaceIds) {
            if (faceId >= 0 && static_cast<std::size_t>(faceId) < faces.size()) {
                componentByFace[static_cast<std::size_t>(faceId)] =
                    static_cast<int>(componentIndex);
                regionFaceIds.push_back(faceId);
            }
        }
        for (const int faceId : component.coreFaceIds) {
            if (faceId >= 0 && static_cast<std::size_t>(faceId) < faces.size()) {
                coreFaceFlags[static_cast<std::size_t>(faceId)] = 1U;
            }
        }
        for (const BoundaryEdge& edge : component.boundaryEdges) {
            if (edge.insideFaceId >= 0 &&
                static_cast<std::size_t>(edge.insideFaceId) < faces.size()) {
                boundaryFaceFlags[static_cast<std::size_t>(edge.insideFaceId)] = 1U;
            }
        }
    }
    std::sort(regionFaceIds.begin(), regionFaceIds.end());
    regionFaceIds.erase(
        std::unique(regionFaceIds.begin(), regionFaceIds.end()),
        regionFaceIds.end());
    localMetrics.regionFaceCount = regionFaceIds.size();
    if (regionFaceIds.empty()) {
        if (metrics != nullptr) {
            *metrics = localMetrics;
        }
        return false;
    }

    std::vector<int> faceToLocalIndex(faces.size(), -1);
    for (std::size_t localIndex = 0; localIndex < regionFaceIds.size(); ++localIndex) {
        faceToLocalIndex[static_cast<std::size_t>(regionFaceIds[localIndex])] =
            static_cast<int>(localIndex);
    }

    // Face-ring distance from Core provides a smooth topology-guidance ramp
    // through Transition while keeping each Region component independent.
    std::vector<int> transitionDepth(faces.size(), -1);
    std::vector<int> maximumTransitionDepth(region.components.size(), 0);
    std::deque<int> depthQueue;
    for (const int faceId : regionFaceIds) {
        if (coreFaceFlags[static_cast<std::size_t>(faceId)] != 0U) {
            transitionDepth[static_cast<std::size_t>(faceId)] = 0;
            depthQueue.push_back(faceId);
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
                maximumTransitionDepth[static_cast<std::size_t>(componentId)] =
                    std::max(
                        maximumTransitionDepth[static_cast<std::size_t>(componentId)],
                        nextDepth);
            }
            depthQueue.push_back(adjacentFaceId);
        }
    }

    std::vector<double> constraintSumX(faces.size(), 0.0);
    std::vector<double> constraintSumY(faces.size(), 0.0);
    std::vector<double> constraintWeight(faces.size(), 0.0);
    const double infinity = std::numeric_limits<double>::infinity();

    // Each processed sample performs a radius-bounded walk over Core faces.
    // The transported direction is accumulated in each destination face's
    // local four-angle coordinates, never as an averaged world vector.
    for (const StrokeSample& sample : processedStroke.samples()) {
        if (sample.faceId < 0 ||
            static_cast<std::size_t>(sample.faceId) >= faces.size() ||
            coreFaceFlags[static_cast<std::size_t>(sample.faceId)] == 0U ||
            !std::isfinite(sample.radius) || sample.radius <= settings_.geometryEpsilon) {
            continue;
        }
        const int sourceLocalIndex =
            faceToLocalIndex[static_cast<std::size_t>(sample.faceId)];
        if (sourceLocalIndex < 0 ||
            !bases[static_cast<std::size_t>(sample.faceId)].valid) {
            continue;
        }
        const MVector sourceDirection = CrossFieldMath::projectToTangent(
            sample.direction,
            bases[static_cast<std::size_t>(sample.faceId)].normal,
            settings_.geometryEpsilon);
        if (sourceDirection.length() <= settings_.geometryEpsilon) {
            continue;
        }

        std::vector<double> distances(regionFaceIds.size(), infinity);
        std::vector<MVector> transportedDirections(
            regionFaceIds.size(),
            MVector::zero);
        std::priority_queue<
            FaceQueueEntry,
            std::vector<FaceQueueEntry>,
            FartherFaceFirst>
            pendingFaces;
        distances[static_cast<std::size_t>(sourceLocalIndex)] = 0.0;
        transportedDirections[static_cast<std::size_t>(sourceLocalIndex)] =
            sourceDirection;
        pendingFaces.push({0.0, sourceLocalIndex});

        while (!pendingFaces.empty()) {
            const FaceQueueEntry entry = pendingFaces.top();
            pendingFaces.pop();
            if (entry.localFaceIndex < 0 ||
                static_cast<std::size_t>(entry.localFaceIndex) >= regionFaceIds.size() ||
                entry.distance > distances[static_cast<std::size_t>(entry.localFaceIndex)] ||
                entry.distance > sample.radius) {
                continue;
            }
            const int faceId =
                regionFaceIds[static_cast<std::size_t>(entry.localFaceIndex)];
            for (const int adjacentFaceId :
                 faces[static_cast<std::size_t>(faceId)].adjacentFaceIds) {
                if (adjacentFaceId < 0 ||
                    static_cast<std::size_t>(adjacentFaceId) >= faces.size() ||
                    coreFaceFlags[static_cast<std::size_t>(adjacentFaceId)] == 0U ||
                    componentByFace[static_cast<std::size_t>(adjacentFaceId)] !=
                        componentByFace[static_cast<std::size_t>(faceId)]) {
                    continue;
                }
                const int adjacentLocalIndex =
                    faceToLocalIndex[static_cast<std::size_t>(adjacentFaceId)];
                if (adjacentLocalIndex < 0) {
                    continue;
                }
                const double stepDistance =
                    (bases[static_cast<std::size_t>(adjacentFaceId)].center -
                     bases[static_cast<std::size_t>(faceId)].center)
                        .length();
                const double candidateDistance = entry.distance + stepDistance;
                if (!std::isfinite(candidateDistance) || candidateDistance > sample.radius ||
                    candidateDistance >=
                        distances[static_cast<std::size_t>(adjacentLocalIndex)]) {
                    continue;
                }
                MVector transported;
                if (!CrossFieldMath::transportDirection(
                        topology,
                        bases,
                        faceId,
                        adjacentFaceId,
                        transportedDirections[static_cast<std::size_t>(
                            entry.localFaceIndex)],
                        transported,
                        settings_.geometryEpsilon)) {
                    continue;
                }
                distances[static_cast<std::size_t>(adjacentLocalIndex)] =
                    candidateDistance;
                transportedDirections[static_cast<std::size_t>(adjacentLocalIndex)] =
                    transported;
                pendingFaces.push({candidateDistance, adjacentLocalIndex});
            }
        }

        const double sampleWeight = std::max(sample.weight, 0.0);
        for (std::size_t localIndex = 0; localIndex < regionFaceIds.size(); ++localIndex) {
            const int faceId = regionFaceIds[localIndex];
            if (coreFaceFlags[static_cast<std::size_t>(faceId)] == 0U ||
                !std::isfinite(distances[localIndex]) ||
                distances[localIndex] > sample.radius) {
                continue;
            }
            const double radialWeight = std::clamp(
                1.0 - distances[localIndex] / sample.radius,
                0.0,
                1.0);
            const double faceInfluence =
                static_cast<std::size_t>(faceId) < region.faceInfluence.size()
                ? static_cast<double>(region.faceInfluence[static_cast<std::size_t>(faceId)])
                : 0.0;
            double weight = sampleWeight * radialWeight * std::max(
                faceInfluence,
                settings_.minimumCoreInfluenceForConstraint);
            if (faceId == sample.faceId) {
                weight = std::max(weight, settings_.directHitConstraintWeight);
            }
            const CrossFieldValue cross = CrossFieldMath::encode(
                transportedDirections[localIndex],
                bases[static_cast<std::size_t>(faceId)],
                settings_.geometryEpsilon);
            if (!cross.valid || weight <= settings_.singularityEpsilon) {
                continue;
            }
            constraintSumX[static_cast<std::size_t>(faceId)] += cross.x * weight;
            constraintSumY[static_cast<std::size_t>(faceId)] += cross.y * weight;
            constraintWeight[static_cast<std::size_t>(faceId)] = std::max(
                constraintWeight[static_cast<std::size_t>(faceId)],
                std::clamp(weight, 0.0, 1.0));
        }
    }

    std::vector<CrossFieldValue> paintCross(faces.size());
    std::vector<CrossFieldValue> topologyCross(faces.size());
    std::vector<double> topologyWeight(faces.size(), 0.0);
    std::vector<CrossFieldValue> current(faces.size());
    for (const int faceId : regionFaceIds) {
        const std::size_t faceIndex = static_cast<std::size_t>(faceId);
        paintCross[faceIndex] = CrossFieldMath::normalized(
            constraintSumX[faceIndex],
            constraintSumY[faceIndex],
            settings_.singularityEpsilon);
        if (paintCross[faceIndex].valid) {
            ++localMetrics.paintConstrainedFaceCount;
        } else {
            constraintWeight[faceIndex] = 0.0;
        }

        double topologyConfidence = 0.0;
        topologyCross[faceIndex] = CrossFieldMath::existingTopologyOrientation(
            topology,
            bases[faceIndex],
            faceId,
            topologyConfidence,
            settings_.geometryEpsilon);
        const int componentId = componentByFace[faceIndex];
        const int maximumDepth = componentId >= 0
            ? maximumTransitionDepth[static_cast<std::size_t>(componentId)]
            : 0;
        const double transitionRatio = maximumDepth > 0 &&
            transitionDepth[faceIndex] > 0
            ? std::clamp(
                  static_cast<double>(transitionDepth[faceIndex]) /
                      static_cast<double>(maximumDepth),
                  0.0,
                  1.0)
            : 0.0;
        double guidanceWeight = coreFaceFlags[faceIndex] != 0U
            ? settings_.coreTopologyGuidanceWeight
            : settings_.transitionTopologyGuidanceWeight * transitionRatio;
        if (boundaryFaceFlags[faceIndex] != 0U) {
            guidanceWeight = std::max(
                guidanceWeight,
                settings_.boundaryTopologyGuidanceWeight);
        }
        topologyWeight[faceIndex] = std::clamp(
            guidanceWeight * topologyConfidence,
            0.0,
            1.0);
        current[faceIndex] = paintCross[faceIndex].valid
            ? paintCross[faceIndex]
            : topologyCross[faceIndex];
    }

    std::vector<CrossFieldValue> next = current;
    for (unsigned int iteration = 0;
         iteration < settings_.directionSmoothingIterations;
         ++iteration) {
        for (const int faceId : regionFaceIds) {
            const std::size_t faceIndex = static_cast<std::size_t>(faceId);
            double neighborX = 0.0;
            double neighborY = 0.0;
            std::size_t neighborCount = 0;
            for (const int adjacentFaceId : faces[faceIndex].adjacentFaceIds) {
                if (adjacentFaceId < 0 ||
                    static_cast<std::size_t>(adjacentFaceId) >= faces.size() ||
                    componentByFace[static_cast<std::size_t>(adjacentFaceId)] !=
                        componentByFace[faceIndex]) {
                    continue;
                }
                const CrossFieldValue transported = CrossFieldMath::transportCross(
                    topology,
                    bases,
                    adjacentFaceId,
                    faceId,
                    current[static_cast<std::size_t>(adjacentFaceId)],
                    settings_.geometryEpsilon);
                if (!transported.valid) {
                    continue;
                }
                neighborX += transported.x;
                neighborY += transported.y;
                ++neighborCount;
            }
            const CrossFieldValue neighborCross = neighborCount > 0
                ? CrossFieldMath::normalized(
                      neighborX,
                      neighborY,
                      settings_.singularityEpsilon)
                : CrossFieldValue();

            const double paintWeight = constraintWeight[faceIndex];
            const double existingWeight =
                (1.0 - paintWeight) * topologyWeight[faceIndex];
            const double availableSmoothWeight =
                (1.0 - paintWeight) * (1.0 - topologyWeight[faceIndex]);
            const double neighborWeight = availableSmoothWeight *
                settings_.directionSmoothStrength;
            const double selfWeight = availableSmoothWeight *
                (1.0 - settings_.directionSmoothStrength);

            double sumX = 0.0;
            double sumY = 0.0;
            if (paintCross[faceIndex].valid) {
                sumX += paintCross[faceIndex].x * paintWeight;
                sumY += paintCross[faceIndex].y * paintWeight;
            }
            if (topologyCross[faceIndex].valid) {
                sumX += topologyCross[faceIndex].x * existingWeight;
                sumY += topologyCross[faceIndex].y * existingWeight;
            }
            if (neighborCross.valid) {
                sumX += neighborCross.x * neighborWeight;
                sumY += neighborCross.y * neighborWeight;
            }
            if (current[faceIndex].valid) {
                sumX += current[faceIndex].x * selfWeight;
                sumY += current[faceIndex].y * selfWeight;
            }
            next[faceIndex] = CrossFieldMath::normalized(
                sumX,
                sumY,
                settings_.singularityEpsilon);
        }
        current.swap(next);
    }
    localMetrics.smoothingIterations = settings_.directionSmoothingIterations;

    for (const int faceId : regionFaceIds) {
        const std::size_t faceIndex = static_cast<std::size_t>(faceId);
        FaceDirectionField& field = output.perFace[faceIndex];
        field.constraintWeight = constraintWeight[faceIndex];
        field.topologyGuidanceWeight = topologyWeight[faceIndex];
        field.hasPaintConstraint = paintCross[faceIndex].valid;
        if (!bases[faceIndex].valid || !current[faceIndex].valid) {
            ++localMetrics.invalidFaceCount;
            continue;
        }

        field.normal = bases[faceIndex].normal;
        field.uDirection = CrossFieldMath::decode(
            current[faceIndex],
            bases[faceIndex],
            settings_.geometryEpsilon);
        field.vDirection = field.normal ^ field.uDirection;
        if (field.vDirection.length() > settings_.geometryEpsilon) {
            field.vDirection.normalize();
        }
        field.uDirection = field.vDirection ^ field.normal;
        if (field.uDirection.length() > settings_.geometryEpsilon) {
            field.uDirection.normalize();
        }
        field.valid = validatesFrame(
            field.normal,
            field.uDirection,
            field.vDirection,
            settings_.validationTolerance);
        if (!field.valid) {
            field.uDirection = MVector::zero;
            field.vDirection = MVector::zero;
            ++localMetrics.invalidFaceCount;
        }
    }

    if (metrics != nullptr) {
        *metrics = localMetrics;
    }
    return localMetrics.regionFaceCount > localMetrics.invalidFaceCount;
}

}  // namespace directional_retopo
