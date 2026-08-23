#pragma once

#include <maya/MVector.h>

#include <cstddef>
#include <vector>

namespace directional_retopo {

struct FaceDirectionField final
{
    MVector normal;
    MVector uDirection;
    MVector vDirection;
    double constraintWeight = 0.0;
    double topologyGuidanceWeight = 0.0;
    bool hasPaintConstraint = false;
    bool valid = false;
};

struct DirectionFieldData final
{
    // Indexed by the original polygon face ID. A future triangulation can
    // safely copy a polygon face's entry to each child triangle.
    std::vector<FaceDirectionField> perFace;

    void clear() noexcept
    {
        perFace.clear();
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return perFace.empty();
    }

    [[nodiscard]] const FaceDirectionField* face(int faceId) const noexcept
    {
        return faceId >= 0 && static_cast<std::size_t>(faceId) < perFace.size()
            ? &perFace[static_cast<std::size_t>(faceId)]
            : nullptr;
    }
};

struct DirectionFieldBuildMetrics final
{
    std::size_t regionFaceCount = 0;
    std::size_t paintConstrainedFaceCount = 0;
    std::size_t invalidFaceCount = 0;
    unsigned int smoothingIterations = 0;
};

}  // namespace directional_retopo
