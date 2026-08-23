#pragma once

#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"
#include "Remesh/LocalPatch.h"

#include <cstddef>

namespace directional_retopo {

struct PatchTriangulatorSettings final
{
    std::size_t minimumVertexCount = 4;
    std::size_t minimumTriangleCount = 2;
};

class PatchTriangulator final
{
public:
    [[nodiscard]] const PatchTriangulatorSettings& settings() const noexcept;
    void setSettings(const PatchTriangulatorSettings& settings) noexcept;

    [[nodiscard]] PatchBuildResult build(
        const PaintRegionData& region,
        const MeshTopologyCache& topology) const;
    [[nodiscard]] PatchBuildResult buildInnerCores(
        const PaintRegionData& region,
        const MeshTopologyCache& topology) const;


private:
    bool buildComponent(
        std::size_t componentId,
        const PaintRegionComponent& component,
        const MeshTopologyCache& topology,
        TriangulatedPatch& patch,
        std::string& failure) const;

    PatchTriangulatorSettings settings_;
};

}  // namespace directional_retopo

