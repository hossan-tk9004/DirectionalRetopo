#pragma once

#include "Field/DensityFieldData.h"
#include "Field/DirectionFieldData.h"
#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"
#include "Viewport/VisualizationSettings.h"

#include <maya/MColorArray.h>
#include <maya/MPointArray.h>
#include <maya/MUIDrawManager.h>

namespace directional_retopo {

class FieldVisualizer final
{
public:
    void clear() noexcept;
    void setSettings(const FieldVisualizationSettings& settings) noexcept;
    [[nodiscard]] const FieldVisualizationSettings& settings() const noexcept;

    void setData(
        const MeshTopologyCache& topology,
        const PaintRegionData& region,
        const DirectionFieldData& directionField,
        const DensityFieldData& densityField);

    void draw(MHWRender::MUIDrawManager& drawManager) const;

private:
    FieldVisualizationSettings settings_;
    MPointArray directionLinePoints_;
    MPointArray densityPoints_;
    MColorArray densityColors_;
};

}  // namespace directional_retopo
