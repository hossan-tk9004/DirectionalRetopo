#pragma once

#include "Remesh/LocalPatch.h"
#include "Remesh/OrderedBoundaryCorrespondence.h"
#include "Remesh/QuadPatchResult.h"

#include <string>

namespace directional_retopo {

struct BoundaryConformerSettings final
{
    double geometryEpsilon = 1.0e-10;
};

class BoundaryConformer final
{
public:
    [[nodiscard]] const BoundaryConformerSettings& settings() const noexcept;
    void setSettings(const BoundaryConformerSettings& settings) noexcept;

    // Repositions only generated boundary vertices. Vertex/edge counts and
    // polygon connectivity are never changed.
    bool conform(
        const TriangulatedPatch& patch,
        QuadPatchResult& result,
        std::string& diagnostic) const;

private:
    BoundaryConformerSettings settings_;
    OrderedBoundaryCorrespondence orderedCorrespondence_;
};

}  // namespace directional_retopo
