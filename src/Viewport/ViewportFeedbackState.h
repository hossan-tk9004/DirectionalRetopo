#pragma once

#include "Paint/StrokeData.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <utility>

namespace directional_retopo {

struct ActiveStrokeVisualizationState final
{
    bool visible = false;
    StrokeData stroke;

    void set(StrokeData newStroke)
    {
        stroke = std::move(newStroke);
        visible = !stroke.empty();
    }

    void clear() noexcept
    {
        visible = false;
        stroke.clear();
    }
};

struct DirectionVisualizationState final
{
    bool visible = false;
    MPoint position;
    MVector normal;
    MVector direction;
    double radius = 1.0;

    void set(const StrokeSample& sample)
    {
        visible = true;
        position = sample.position;
        normal = sample.normal;
        direction = sample.direction;
        radius = sample.radius;
    }

    void clear() noexcept
    {
        visible = false;
    }
};

struct ViewportFeedbackState final
{
    ActiveStrokeVisualizationState activeStroke;
    DirectionVisualizationState direction;

    void clearInteraction() noexcept
    {
        activeStroke.clear();
        direction.clear();
    }

    void clearStroke() noexcept
    {
        activeStroke.clear();
        direction.clear();
    }
};

}  // namespace directional_retopo
