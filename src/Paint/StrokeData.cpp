#include "Paint/StrokeData.h"

#include <utility>

namespace directional_retopo {

void StrokeData::clear() noexcept
{
    samples_.clear();
}

bool StrokeData::empty() const noexcept
{
    return samples_.empty();
}

std::size_t StrokeData::size() const noexcept
{
    return samples_.size();
}

const StrokeSample& StrokeData::back() const
{
    return samples_.back();
}

const std::vector<StrokeSample>& StrokeData::samples() const noexcept
{
    return samples_;
}

bool StrokeData::append(StrokeSample sample, double minimumSpacing)
{
    constexpr double kNormalEpsilon = 1.0e-10;
    if (sample.normal.length() > kNormalEpsilon) {
        sample.normal.normalize();
    }

    if (samples_.empty()) {
        samples_.push_back(std::move(sample));
        return true;
    }

    const StrokeSample& previous = samples_.back();
    const MVector displacement = sample.position - previous.position;
    if (displacement.length() < minimumSpacing) {
        return false;
    }

    samples_.push_back(std::move(sample));
    return true;
}

void StrokeData::replaceSamples(std::vector<StrokeSample> samples)
{
    samples_ = std::move(samples);
}

}  // namespace directional_retopo
