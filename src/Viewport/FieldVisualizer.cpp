#include "Viewport/FieldVisualizer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace directional_retopo {
namespace {

std::size_t adaptiveStride(
    std::size_t itemCount,
    int configuredStride,
    std::size_t maximumCount)
{
    const std::size_t baseStride = static_cast<std::size_t>(
        std::max(configuredStride, 1));
    if (maximumCount == 0 || itemCount <= maximumCount) {
        return baseStride;
    }
    const std::size_t countStride =
        (itemCount + maximumCount - 1) / maximumCount;
    return std::max(baseStride, countStride);
}

MColor lerpColor(const MColor& first, const MColor& second, double amount)
{
    const float t = static_cast<float>(std::clamp(amount, 0.0, 1.0));
    return MColor(
        first.r + (second.r - first.r) * t,
        first.g + (second.g - first.g) * t,
        first.b + (second.b - first.b) * t,
        first.a + (second.a - first.a) * t);
}

}  // namespace

void FieldVisualizer::clear() noexcept
{
    directionLinePoints_.clear();
    densityPoints_.clear();
    densityColors_.clear();
}

void FieldVisualizer::setSettings(
    const FieldVisualizationSettings& settings) noexcept
{
    settings_ = settings;
    settings_.directionFieldLineWidth =
        std::max(settings_.directionFieldLineWidth, 0.1F);
    settings_.densityPointSize = std::max(settings_.densityPointSize, 0.1F);
    settings_.fieldDisplayStride = std::max(settings_.fieldDisplayStride, 1);
    settings_.densityDisplayStride = std::max(settings_.densityDisplayStride, 1);
    settings_.directionGlyphTargetLengthRatio =
        std::max(settings_.directionGlyphTargetLengthRatio, 0.0);
    settings_.surfaceOffsetLengthRatio =
        std::max(settings_.surfaceOffsetLengthRatio, 0.0);
    settings_.minimumSurfaceOffset = std::max(settings_.minimumSurfaceOffset, 0.0);
}

const FieldVisualizationSettings& FieldVisualizer::settings() const noexcept
{
    return settings_;
}

void FieldVisualizer::setData(
    const MeshTopologyCache& topology,
    const PaintRegionData& region,
    const DirectionFieldData& directionField,
    const DensityFieldData& densityField)
{
    clear();
    const auto& faces = topology.faces();
    std::vector<int> regionFaceIds;
    regionFaceIds.reserve(region.totalFaceCount());
    for (const PaintRegionComponent& component : region.components) {
        regionFaceIds.insert(
            regionFaceIds.end(),
            component.allFaceIds.begin(),
            component.allFaceIds.end());
    }
    std::sort(regionFaceIds.begin(), regionFaceIds.end());
    regionFaceIds.erase(
        std::unique(regionFaceIds.begin(), regionFaceIds.end()),
        regionFaceIds.end());

    const std::size_t directionStride = adaptiveStride(
        regionFaceIds.size(),
        settings_.fieldDisplayStride,
        settings_.maxFieldGlyphCount);
    const std::size_t densityStride = adaptiveStride(
        regionFaceIds.size(),
        settings_.densityDisplayStride,
        settings_.maxDensityGlyphCount);

    double minimumDensity = std::numeric_limits<double>::infinity();
    double maximumDensity = 0.0;
    for (const int faceId : regionFaceIds) {
        if (faceId < 0 || static_cast<std::size_t>(faceId) >= densityField.perFace.size()) {
            continue;
        }
        const FaceDensity& density = densityField.perFace[static_cast<std::size_t>(faceId)];
        if (density.valid) {
            minimumDensity = std::min(minimumDensity, density.targetEdgeLength);
            maximumDensity = std::max(maximumDensity, density.targetEdgeLength);
        }
    }

    std::size_t directionOrdinal = 0;
    std::size_t densityOrdinal = 0;
    for (const int faceId : regionFaceIds) {
        if (faceId < 0 || static_cast<std::size_t>(faceId) >= faces.size()) {
            continue;
        }
        const std::size_t faceIndex = static_cast<std::size_t>(faceId);
        const MeshFaceTopology& face = faces[faceIndex];
        const FaceDirectionField* direction = faceIndex < directionField.perFace.size()
            ? &directionField.perFace[faceIndex]
            : nullptr;
        const FaceDensity* density = faceIndex < densityField.perFace.size()
            ? &densityField.perFace[faceIndex]
            : nullptr;

        if (direction != nullptr && direction->valid) {
            if ((directionOrdinal++ % directionStride) == 0U) {
                const double targetLength = density != nullptr && density->valid
                    ? density->targetEdgeLength
                    : 1.0;
                const double halfLength = targetLength *
                    settings_.directionGlyphTargetLengthRatio * 0.5;
                const double offset = std::max(
                    targetLength * settings_.surfaceOffsetLengthRatio,
                    settings_.minimumSurfaceOffset);
                const MPoint center = face.worldCenter + direction->normal * offset;
                directionLinePoints_.append(
                    center - direction->uDirection * halfLength);
                directionLinePoints_.append(
                    center + direction->uDirection * halfLength);
                directionLinePoints_.append(
                    center - direction->vDirection * halfLength);
                directionLinePoints_.append(
                    center + direction->vDirection * halfLength);
            }
        }

        if (density != nullptr && density->valid) {
            if ((densityOrdinal++ % densityStride) == 0U) {
                const double offset = std::max(
                    density->targetEdgeLength *
                        settings_.surfaceOffsetLengthRatio * 1.5,
                    settings_.minimumSurfaceOffset);
                densityPoints_.append(face.worldCenter + face.worldNormal * offset);
                const double denominator = maximumDensity - minimumDensity;
                const double normalizedDensity = denominator > 1.0e-12
                    ? (density->targetEdgeLength - minimumDensity) / denominator
                    : 0.5;
                densityColors_.append(lerpColor(
                    settings_.densityLowColor,
                    settings_.densityHighColor,
                    normalizedDensity));
            }
        }
    }
}

void FieldVisualizer::draw(MHWRender::MUIDrawManager& drawManager) const
{
    drawManager.setDepthPriority(settings_.fieldDepthPriority);
    if (settings_.showDirectionField && directionLinePoints_.length() >= 2) {
        drawManager.setColor(settings_.directionFieldColor);
        drawManager.setLineWidth(settings_.directionFieldLineWidth);
        (void)drawManager.lineList(directionLinePoints_, false);
    }
    if (settings_.showDensityField && densityPoints_.length() > 0 &&
        densityColors_.length() == densityPoints_.length()) {
        drawManager.setPointSize(settings_.densityPointSize);
        drawManager.mesh(
            MHWRender::MUIDrawManager::kPoints,
            densityPoints_,
            nullptr,
            &densityColors_);
    }
}

}  // namespace directional_retopo
