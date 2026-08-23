#pragma once

#include "Viewport/VisualizationSettings.h"

#include <maya/MPoint.h>
#include <maya/MVector.h>

#include <mutex>

namespace directional_retopo {

enum class BrushCursorMode
{
    Hidden,
    Hover,
    RadiusAdjust,
};

struct BrushCursorSnapshot final
{
    bool visible = false;
    bool requiresFreshRaycast = true;
    bool cameraSuppressed = false;
    BrushCursorMode mode = BrushCursorMode::Hidden;
    short screenX = 0;
    short screenY = 0;
    MPoint worldPosition;
    MVector surfaceNormal;
    double radius = 1.0;
    BrushVisualizationSettings style;

    [[nodiscard]] bool drawable() const noexcept
    {
        return visible && !requiresFreshRaycast && !cameraSuppressed &&
            mode != BrushCursorMode::Hidden;
    }
};

class BrushCursorModel final
{
public:
    void reset(
        double radius,
        const BrushVisualizationSettings& style)
    {
        std::scoped_lock lock(mutex_);
        snapshot_ = BrushCursorSnapshot();
        snapshot_.radius = radius;
        snapshot_.style = style;
    }

    void setFreshHit(
        short screenX,
        short screenY,
        const MPoint& worldPosition,
        const MVector& surfaceNormal,
        double radius,
        BrushCursorMode mode)
    {
        std::scoped_lock lock(mutex_);
        snapshot_.screenX = screenX;
        snapshot_.screenY = screenY;
        snapshot_.worldPosition = worldPosition;
        snapshot_.surfaceNormal = surfaceNormal;
        snapshot_.radius = radius;
        snapshot_.requiresFreshRaycast = false;
        snapshot_.mode = mode;
        snapshot_.visible = !snapshot_.cameraSuppressed;
    }

    void beginRadiusAdjust()
    {
        std::scoped_lock lock(mutex_);
        if (snapshot_.requiresFreshRaycast || snapshot_.cameraSuppressed) {
            snapshot_.visible = false;
            snapshot_.mode = BrushCursorMode::Hidden;
            return;
        }
        snapshot_.mode = BrushCursorMode::RadiusAdjust;
        snapshot_.visible = true;
    }

    void endRadiusAdjust()
    {
        std::scoped_lock lock(mutex_);
        invalidateForFreshRaycastLocked();
    }

    void setRadius(double radius)
    {
        std::scoped_lock lock(mutex_);
        snapshot_.radius = radius;
    }

    void setCameraSuppressed(bool suppressed)
    {
        std::scoped_lock lock(mutex_);
        snapshot_.cameraSuppressed = suppressed;
        invalidateForFreshRaycastLocked();
        snapshot_.cameraSuppressed = suppressed;
    }

    void invalidateForFreshRaycast()
    {
        std::scoped_lock lock(mutex_);
        invalidateForFreshRaycastLocked();
    }

    [[nodiscard]] bool hasFreshHit() const
    {
        std::scoped_lock lock(mutex_);
        return !snapshot_.requiresFreshRaycast && !snapshot_.cameraSuppressed;
    }

    [[nodiscard]] BrushCursorSnapshot snapshot() const
    {
        std::scoped_lock lock(mutex_);
        return snapshot_;
    }

private:
    void invalidateForFreshRaycastLocked() noexcept
    {
        snapshot_.visible = false;
        snapshot_.requiresFreshRaycast = true;
        snapshot_.mode = BrushCursorMode::Hidden;
        snapshot_.screenX = 0;
        snapshot_.screenY = 0;
        snapshot_.worldPosition = MPoint();
        snapshot_.surfaceNormal = MVector();
    }

    mutable std::mutex mutex_;
    BrushCursorSnapshot snapshot_;
};

}  // namespace directional_retopo
