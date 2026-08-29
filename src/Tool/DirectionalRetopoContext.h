#pragma once

#include "Brush/BrushSettings.h"
#include "Brush/RayCaster.h"
#include "Field/DensityFieldBuilder.h"
#include "Field/DirectionFieldBuilder.h"
#include "Field/QuadSolveInput.h"
#include "Mesh/MeshTopologyCache.h"
#include "Paint/PaintRegionData.h"
#include "Paint/PaintRegionSolver.h"
#include "Paint/RegionPreviewCalculator.h"
#include "Paint/StrokeData.h"
#include "Paint/StrokeProcessor.h"
#include "Remesh/AutoRemesherAdapter.h"
#include "Remesh/BoundaryLockedPatchBuilder.h"
#include "Remesh/PatchTriangulator.h"
#include "Remesh/QuadPatchResult.h"
#include "Solver/DirectionalRemeshSolver.h"
#include "Viewport/BrushCursorModel.h"
#include "Viewport/DirectionalRetopoBrushCursor.h"
#include "Viewport/DirectionalRetopoQuadPreview.h"
#include "Viewport/DirectionalRetopoTargetDisplay.h"
#include "Viewport/FieldVisualizer.h"
#include "Viewport/RegionVisualizer.h"
#include "Viewport/QuadPreviewModel.h"
#include "Viewport/TargetDisplayModel.h"
#include "Viewport/ViewportFeedbackState.h"
#include "Viewport/VisualizationSettings.h"

#include <maya/MFrameContext.h>
#include <maya/MMessage.h>
#include <maya/MPxContext.h>
#include <maya/MSelectionList.h>
#include <maya/MStatus.h>
#include <maya/MUIDrawManager.h>

#include <memory>
#include <unordered_set>
#include <vector>

namespace directional_retopo {

class DirectionalRetopoContext final : public MPxContext
{
public:
    DirectionalRetopoContext();
    ~DirectionalRetopoContext() override;

    void toolOnSetup(MEvent& event) override;
    void toolOffCleanup() override;

    MStatus doPress(
        MEvent& event,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext) override;
    MStatus doDrag(
        MEvent& event,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext) override;
    MStatus doRelease(
        MEvent& event,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext) override;
    MStatus doPtrMoved(
        MEvent& event,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext) override;
    MStatus drawFeedback(
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext) override;

    MStatus doEnterRegion(MEvent& event) override;
    MStatus doExitRegion(MEvent& event) override;
    void getClassName(MString& name) const override;

    [[nodiscard]] double brushRadius() const noexcept;
    void setBrushRadius(double radius);
    [[nodiscard]] bool radiusAdjustMode() const noexcept;
    void setRadiusAdjustMode(bool enabled);
    [[nodiscard]] DensityMode densityMode() const noexcept;
    void setDensityMode(DensityMode mode);
    [[nodiscard]] double manualTargetEdgeLength() const noexcept;
    void setManualTargetEdgeLength(double edgeLength);
    [[nodiscard]] double densityEdgeLengthScale() const noexcept;
    void setDensityEdgeLengthScale(double scale);
    [[nodiscard]] int topologyBlendWidth() const noexcept;
    void setTopologyBlendWidth(int rings);
    [[nodiscard]] bool showDirectionField() const noexcept;
    void setShowDirectionField(bool show);
    [[nodiscard]] bool showDensityField() const noexcept;
    void setShowDensityField(bool show);
    [[nodiscard]] bool showQuadPreview() const noexcept;
    void setShowQuadPreview(bool show);
    [[nodiscard]] bool showRawQuadPreview() const noexcept;
    void setShowRawQuadPreview(bool show);
    [[nodiscard]] bool showConformedQuadPreview() const noexcept;
    void setShowConformedQuadPreview(bool show);
    [[nodiscard]] bool showSourceBoundary() const noexcept;
    void setShowSourceBoundary(bool show);
    [[nodiscard]] bool showResultBoundary() const noexcept;
    void setShowResultBoundary(bool show);
    [[nodiscard]] bool showBoundaryCorrespondence() const noexcept;
    void setShowBoundaryCorrespondence(bool show);
    [[nodiscard]] bool showRequiredBoundaryAnchors() const noexcept;
    void setShowRequiredBoundaryAnchors(bool show);
    void resetToolSettings();
    [[nodiscard]] QuadSolveInput quadSolveInput() const noexcept;

private:
    static void alternateContextTimerCallback(
        float elapsedTime,
        float lastTime,
        void* clientData);

    bool acquireTargetFromSelection(bool displayErrors);
    bool hasValidTarget(bool displayErrors);
    void restoreOriginalSelection() noexcept;
    bool castEventToTarget(MEvent& event, SurfaceHit& hit);
    void setBrushCursorFromHit(MEvent& event, const SurfaceHit& hit);
    bool appendSurfaceHit(const SurfaceHit& hit, double minimumSpacing);
    void updateCursorHit(MEvent& event);
    void updateActiveStrokeVisualization();
    void finalizeProcessedStroke();
    void generateFinalPaintRegion();
    void generateFinalFields();
    void generateQuadPreview();
    void clearFinalFields() noexcept;
    void clearQuadPreview() noexcept;
    void resetRegionPreview();
    void invalidateTransientForCamera();
    void invalidateHoverFeedback(bool refreshViewport);
    void installAlternateContextMonitor();
    void removeAlternateContextMonitor() noexcept;
    void pollAlternateContext();
    void updateAlternateContextState(bool alternateContextActive);
    void requestBrushCursorRefresh();
    void requestQuadPreviewRefresh();
    void requestFeedbackRefresh();

    BrushSettings brushSettings_;
    RayCaster rayCaster_;
    ViewportVisualizationSettings visualizationSettings_;
    MeshTopologyCache meshTopologyCache_;
    PaintRegionSolver paintRegionSolver_;
    PaintRegionData finalPaintRegion_;
    DirectionFieldBuilder directionFieldBuilder_;
    DirectionFieldData directionFieldData_;
    DensityFieldBuilder densityFieldBuilder_;
    DensityFieldData densityFieldData_;
    RegionPreviewCalculator regionPreviewCalculator_;
    RegionVisualizer regionVisualizer_;
    FieldVisualizer fieldVisualizer_;
    PatchTriangulator patchTriangulator_;
    AutoRemesherAdapter autoRemesherAdapter_;
    BoundaryLockedPatchBuilder boundaryLockedPatchBuilder_;
    DirectionalRemeshSolver remeshSolver_;
    std::vector<TriangulatedPatch> triangulatedPatches_;
    std::vector<QuadComponentSolveReport> quadSolveReports_;
    std::vector<QuadPatchResult> quadPatchResults_;
    std::vector<QuadPatchResult> quadDebugPatchResults_;
    std::shared_ptr<BrushCursorModel> brushCursorModel_;
    BrushCursorDrawable brushCursorDrawable_;
    std::shared_ptr<TargetDisplayModel> targetDisplayModel_;
    TargetDisplayDrawable targetDisplayDrawable_;
    std::shared_ptr<QuadPreviewModel> quadPreviewModel_;
    QuadPreviewDrawable quadPreviewDrawable_;
    ViewportFeedbackState feedbackState_;
    StrokeData rawStroke_;
    StrokeData processedStroke_;
    StrokeProcessor strokeProcessor_;
    std::unordered_set<int> regionFaceIds_;
    bool strokeActive_ = false;
    bool radiusAdjustMode_ = false;
    bool radiusAdjustDragActive_ = false;
    short radiusAdjustStartX_ = 0;
    double radiusAdjustStartRadius_ = BrushSettings::kDefaultRadius;
    MSelectionList originalSelection_;
    bool originalSelectionSaved_ = false;
    MCallbackId alternateContextTimerCallbackId_ = 0;
    bool alternateContextObserved_ = false;
    bool toolActive_ = false;
};

}  // namespace directional_retopo
