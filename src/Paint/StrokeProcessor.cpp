#include "Paint/StrokeProcessor.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace directional_retopo {
namespace {

constexpr double kDirectionEpsilon = 1.0e-10;

double strokeLength(const std::vector<StrokeSample>& samples)
{
    double length = 0.0;
    for (std::size_t index = 1; index < samples.size(); ++index) {
        length += (samples[index].position - samples[index - 1].position).length();
    }
    return length;
}

std::size_t trimmedSampleCount(
    const std::vector<StrokeSample>& samples,
    double trimDistance,
    std::size_t minimumSamples)
{
    if (samples.size() <= minimumSamples || trimDistance <= 0.0) {
        return samples.size();
    }

    std::size_t endExclusive = samples.size();
    double removedDistance = 0.0;

    while (endExclusive > minimumSamples) {
        const double segmentLength =
            (samples[endExclusive - 1].position - samples[endExclusive - 2].position).length();
        const double candidateDistance = removedDistance + segmentLength;

        if (candidateDistance > trimDistance &&
            std::abs(trimDistance - removedDistance) <=
                std::abs(candidateDistance - trimDistance)) {
            break;
        }

        --endExclusive;
        removedDistance = candidateDistance;
        if (removedDistance >= trimDistance) {
            break;
        }
    }

    return endExclusive;
}

MVector projectToTangentPlane(const MVector& vector, const MVector& normal)
{
    if (normal.length() <= kDirectionEpsilon) {
        return vector;
    }
    return vector - normal * (vector * normal);
}

void calculateWindowDirections(
    std::vector<StrokeSample>& samples,
    std::size_t windowRadius)
{
    if (samples.size() < 2) {
        return;
    }

    const std::size_t effectiveWindow = std::max<std::size_t>(windowRadius, 1);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const std::size_t first = index > effectiveWindow ? index - effectiveWindow : 0;
        const std::size_t last =
            std::min(samples.size() - 1, index + effectiveWindow);

        MVector direction = samples[last].position - samples[first].position;
        direction = projectToTangentPlane(direction, samples[index].normal);
        if (direction.length() > kDirectionEpsilon) {
            direction.normalize();
            samples[index].direction = direction;
        } else {
            samples[index].direction = MVector::zero;
        }
    }
}

void smoothDirections(
    std::vector<StrokeSample>& samples,
    unsigned int passes,
    double blend)
{
    if (samples.size() < 2 || passes == 0) {
        return;
    }

    const double clampedBlend = std::clamp(blend, 0.0, 1.0);
    std::vector<MVector> source(samples.size());
    std::vector<MVector> destination(samples.size());

    for (std::size_t index = 0; index < samples.size(); ++index) {
        source[index] = samples[index].direction;
    }

    for (unsigned int pass = 0; pass < passes; ++pass) {
        for (std::size_t index = 0; index < samples.size(); ++index) {
            const MVector current = source[index];
            if (current.length() <= kDirectionEpsilon) {
                destination[index] = current;
                continue;
            }

            MVector average = current * 2.0;
            double weight = 2.0;

            if (index > 0 && source[index - 1].length() > kDirectionEpsilon) {
                MVector previous = source[index - 1];
                if ((previous * current) < 0.0) {
                    previous *= -1.0;
                }
                average += previous;
                weight += 1.0;
            }
            if (index + 1 < samples.size() &&
                source[index + 1].length() > kDirectionEpsilon) {
                MVector next = source[index + 1];
                if ((next * current) < 0.0) {
                    next *= -1.0;
                }
                average += next;
                weight += 1.0;
            }

            average /= weight;
            average = projectToTangentPlane(average, samples[index].normal);
            if (average.length() > kDirectionEpsilon) {
                average.normalize();
            } else {
                average = current;
            }

            MVector smoothed = current * (1.0 - clampedBlend) + average * clampedBlend;
            smoothed = projectToTangentPlane(smoothed, samples[index].normal);
            if (smoothed.length() > kDirectionEpsilon) {
                smoothed.normalize();
            }
            destination[index] = smoothed;
        }
        source.swap(destination);
    }

    for (std::size_t index = 0; index < samples.size(); ++index) {
        samples[index].direction = source[index];
    }
}

}  // namespace

const StrokeProcessingSettings& StrokeProcessor::settings() const noexcept
{
    return settings_;
}

void StrokeProcessor::setSettings(const StrokeProcessingSettings& settings) noexcept
{
    settings_ = settings;
}

StrokeData StrokeProcessor::process(
    const StrokeData& rawStroke,
    double sampleSpacing,
    double brushRadius,
    bool trimEndpoint) const
{
    StrokeData processedStroke;
    if (rawStroke.empty()) {
        return processedStroke;
    }

    std::vector<StrokeSample> samples = rawStroke.samples();
    if (trimEndpoint && samples.size() > 1) {
        const double totalLength = strokeLength(samples);
        const double requestedTrimDistance = std::max(
            sampleSpacing * settings_.endTrimSpacingMultiplier,
            brushRadius * settings_.endTrimRadiusMultiplier);
        const double trimDistance = std::min(
            requestedTrimDistance,
            totalLength * settings_.maximumEndTrimLengthRatio);
        const std::size_t minimumSamples = std::min(
            settings_.minimumSamplesAfterTrim,
            samples.size());
        samples.resize(trimmedSampleCount(samples, trimDistance, minimumSamples));
    }

    calculateWindowDirections(samples, settings_.directionWindowRadius);
    smoothDirections(
        samples,
        settings_.directionSmoothingPasses,
        settings_.directionSmoothingBlend);

    processedStroke.replaceSamples(std::move(samples));
    return processedStroke;
}

}  // namespace directional_retopo
