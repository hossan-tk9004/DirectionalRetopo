#pragma once

#include "Remesh/LocalPatch.h"
#include "Remesh/QuadPatchResult.h"

#include <cstddef>
#include <string>
#include <vector>

namespace directional_retopo {

struct OrderedBoundaryCorrespondenceSettings final
{
    double geometryEpsilon = 1.0e-10;
    double tangentCostWeight = 0.20;
    double arcLengthCostWeight = 0.35;
    double boundaryCornerAngleRadians = 0.35;
    unsigned int samplesPerResultVertex = 6U;
    std::size_t maximumCandidateSamples = 8192U;
    std::size_t seamCandidateCount = 4U;
};

class OrderedBoundaryCorrespondence final
{
public:
    [[nodiscard]] const OrderedBoundaryCorrespondenceSettings& settings()
        const noexcept;
    void setSettings(
        const OrderedBoundaryCorrespondenceSettings& settings) noexcept;

    // Solves one Source/Result loop pair as an ordered sequence. The Result
    // loop metadata may be cyclically shifted or reversed, but polygon
    // connectivity and vertex counts are never changed.
    bool solve(
        const TriangulatedPatch& patch,
        std::size_t sourceLoopIndex,
        ResultBoundaryLoop& resultLoop,
        std::vector<MPoint>& resultVertices,
        BoundaryLoopCorrespondence& correspondence,
        std::string& diagnostic) const;

private:
    OrderedBoundaryCorrespondenceSettings settings_;
};

}  // namespace directional_retopo
