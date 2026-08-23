#pragma once

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <cstddef>
#include <vector>

namespace directional_retopo {

struct StrokeSample
{
    MPoint position;
    MVector normal;
    MVector direction;
    double weight = 1.0;
    double radius = 1.0;
    int faceId = -1;
    int triangleId = -1;
    float barycentric1 = 0.0F;
    float barycentric2 = 0.0F;
};

class StrokeData final
{
public:
    void clear() noexcept;

    [[nodiscard]] bool empty() const noexcept;
    [[nodiscard]] std::size_t size() const noexcept;

    [[nodiscard]] const StrokeSample& back() const;
    [[nodiscard]] const std::vector<StrokeSample>& samples() const noexcept;

    bool append(StrokeSample sample, double minimumSpacing);
    void replaceSamples(std::vector<StrokeSample> samples);

private:
    std::vector<StrokeSample> samples_;
};

}  // namespace directional_retopo
