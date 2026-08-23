#pragma once

#include <maya/MColor.h>
#include <maya/MHWGeometry.h>

#include <cstddef>

namespace directional_retopo {

struct TargetVisualizationSettings final
{
    bool showTargetWireframe = true;
    MColor targetWireColor = MColor(0.55F, 0.55F, 0.55F, 1.0F);
    float targetWireOpacity = 0.35F;
    float targetWireLineWidth = 1.0F;
    unsigned int targetWireDepthPriority =
        MHWRender::MRenderItem::sActiveWireDepthPriority;
    double targetWireNormalOffsetRatio = 1.0e-5;
};

struct RegionVisualizationSettings final
{
    bool showRegionPreview = true;
    bool showFinalRegion = true;
    MColor provisionalColor = MColor(0.55F, 0.20F, 0.15F, 1.0F);
    float provisionalOpacity = 0.14F;
    MColor coreColor = MColor(0.90F, 0.28F, 0.08F, 1.0F);
    float coreOpacity = 0.21F;
    MColor transitionColor = MColor(0.95F, 0.62F, 0.16F, 1.0F);
    float transitionOpacity = 0.10F;
    MColor boundaryColor = MColor(1.0F, 0.82F, 0.18F, 1.0F);
    float boundaryOpacity = 0.90F;
    float boundaryLineWidth = 1.5F;
    unsigned int fillDepthPriority =
        MHWRender::MRenderItem::sDormantWireDepthPriority;
    unsigned int boundaryDepthPriority =
        MHWRender::MRenderItem::sActiveWireDepthPriority + 1U;
    double regionNormalOffsetRatio = 5.0e-6;
    double boundaryNormalOffsetRatio = 1.5e-5;
};

struct BrushVisualizationSettings final
{
    MColor brushColor = MColor(0.25F, 1.0F, 0.35F, 1.0F);
    float brushLineWidth = 2.0F;
    unsigned int brushDepthPriority =
        MHWRender::MRenderItem::sActiveLineDepthPriority;
    int circleSegments = 48;
    double surfaceOffsetRadiusRatio = 0.002;
    double minimumSurfaceOffset = 1.0e-5;
};

struct StrokeVisualizationSettings final
{
    MColor strokeColor = MColor(0.10F, 0.75F, 1.0F, 1.0F);
    float strokeLineWidth = 3.0F;
    float singleSamplePointSize = 5.0F;
    unsigned int strokeDepthPriority =
        MHWRender::MRenderItem::sSelectionDepthPriority;
    double surfaceOffsetRadiusRatio = 0.002;
    double minimumSurfaceOffset = 1.0e-5;
};

struct DirectionVisualizationSettings final
{
    MColor directionColor = MColor(1.0F, 0.8F, 0.1F, 1.0F);
    float directionLineWidth = 3.0F;
    unsigned int directionDepthPriority =
        MHWRender::MRenderItem::sSelectionDepthPriority;
    double lengthRadiusRatio = 0.75;
    double surfaceOffsetRadiusRatio = 0.002;
    double minimumSurfaceOffset = 1.0e-5;
};

struct FieldVisualizationSettings final
{
    bool showDirectionField = true;
    bool showDensityField = false;
    MColor directionFieldColor = MColor(0.20F, 0.95F, 0.82F, 1.0F);
    MColor densityLowColor = MColor(0.20F, 0.45F, 1.0F, 1.0F);
    MColor densityHighColor = MColor(1.0F, 0.30F, 0.65F, 1.0F);
    float directionFieldLineWidth = 1.25F;
    float densityPointSize = 4.0F;
    unsigned int fieldDepthPriority =
        MHWRender::MRenderItem::sActiveWireDepthPriority + 2U;
    int fieldDisplayStride = 1;
    int densityDisplayStride = 1;
    std::size_t maxFieldGlyphCount = 1500;
    std::size_t maxDensityGlyphCount = 1500;
    double directionGlyphTargetLengthRatio = 0.35;
    double surfaceOffsetLengthRatio = 0.01;
    double minimumSurfaceOffset = 1.0e-5;
};

struct QuadPreviewVisualizationSettings final
{
    bool showQuadPreview = true;
    bool showRawQuadPreview = false;
    bool showConformedQuadPreview = true;
    bool showSourceBoundary = false;
    bool showResultBoundary = false;
    bool showBoundaryCorrespondence = false;
    bool showRequiredBoundaryAnchors = false;
    bool showTransitionCollar = true;
    bool showTrianglePolygons = true;
    MColor rawWireColor = MColor(1.0F, 0.32F, 0.12F, 1.0F);
    float rawWireOpacity = 0.70F;
    float rawWireLineWidth = 1.25F;
    MColor conformedWireColor = MColor(0.10F, 1.0F, 0.72F, 1.0F);
    float conformedWireOpacity = 0.95F;
    float conformedWireLineWidth = 2.0F;
    MColor sourceBoundaryColor = MColor(1.0F, 0.76F, 0.12F, 1.0F);
    float sourceBoundaryOpacity = 0.95F;
    float sourceBoundaryLineWidth = 2.5F;
    MColor resultBoundaryColor = MColor(1.0F, 0.18F, 0.72F, 1.0F);
    float resultBoundaryOpacity = 0.95F;
    float resultBoundaryLineWidth = 2.5F;
    MColor boundaryCorrespondenceColor = MColor(0.25F, 0.85F, 1.0F, 1.0F);
    float boundaryCorrespondenceOpacity = 0.75F;
    float boundaryCorrespondenceLineWidth = 1.0F;
    MColor requiredBoundaryAnchorColor = MColor(1.0F, 0.05F, 0.05F, 1.0F);
    float requiredBoundaryAnchorOpacity = 1.0F;
    float requiredBoundaryAnchorPointSize = 8.0F;
    MColor transitionCollarColor = MColor(0.35F, 0.70F, 1.0F, 1.0F);
    float transitionCollarOpacity = 0.90F;
    float transitionCollarLineWidth = 2.25F;
    MColor trianglePolygonColor = MColor(1.0F, 0.30F, 0.08F, 1.0F);
    float trianglePolygonOpacity = 1.0F;
    float trianglePolygonLineWidth = 3.0F;

    unsigned int wireDepthPriority =
        MHWRender::MRenderItem::sActiveLineDepthPriority;
};

struct ViewportVisualizationSettings final
{
    TargetVisualizationSettings target;
    RegionVisualizationSettings region;
    BrushVisualizationSettings brush;
    StrokeVisualizationSettings stroke;
    DirectionVisualizationSettings direction;
    FieldVisualizationSettings field;
    QuadPreviewVisualizationSettings quadPreview;
};

}  // namespace directional_retopo
