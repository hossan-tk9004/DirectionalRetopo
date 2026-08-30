#include "Tool/DirectionalRetopoContext.h"

#include "Integration/LegacyPreviewAdapter.h"
#include "Integration/MayaRemeshInputAdapter.h"
#include "Solver/RemeshCapture.h"
#include "Tool/PythonRuntimeBridge.h"

#include <maya/M3dView.h>
#include <maya/MColor.h>
#include <maya/MEvent.h>
#include <maya/MFn.h>
#include <maya/MFnDagNode.h>
#include <maya/MGlobal.h>
#include <maya/MHWGeometry.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>
#include <maya/MPointArray.h>
#include <maya/MSelectionList.h>
#include <maya/MTimerMessage.h>
#include <maya/MToolsInfo.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>

namespace directional_retopo {
namespace {

constexpr char kToolTitle[] = "Directional Retopo";
constexpr char kHelpText[] =
    "Left-drag to paint a stroke. Hold B and drag left/right to change brush radius.";
constexpr double kDirectionEpsilon = 1.0e-10;
constexpr double kRadiusPixelsPerDoubling = 100.0;
constexpr double kMaximumBrushRadius = 1.0e6;
constexpr double kMinimumInteractiveManualQuadLimit = 1000.0;
constexpr double kMaximumInteractiveManualQuadLimit = 5000.0;
constexpr double kMaximumInteractiveManualRefinementRatio = 8.0;
constexpr float kAlternateContextPollIntervalSeconds = 1.0F / 60.0F;

void displayTargetWarning(const char* detail)
{
    MString message("[DirectionalRetopo] ");
    message += detail;
    MGlobal::displayWarning(message);
}

bool isRenderableMeshPath(const MDagPath& path)
{
    if (!path.hasFn(MFn::kMesh)) {
        return false;
    }

    MStatus status;
    MFnDagNode dagNode(path, &status);
    return status && !dagNode.isIntermediateObject(&status) && status;
}

bool resolveMeshShape(const MDagPath& selectedPath, MDagPath& meshPath)
{
    if (isRenderableMeshPath(selectedPath)) {
        meshPath = selectedPath;
        return true;
    }

    if (!selectedPath.hasFn(MFn::kTransform)) {
        return false;
    }

    unsigned int shapeCount = 0;
    if (!selectedPath.numberOfShapesDirectlyBelow(shapeCount)) {
        return false;
    }

    unsigned int meshCount = 0;
    MDagPath resolvedPath;
    for (unsigned int shapeIndex = 0; shapeIndex < shapeCount; ++shapeIndex) {
        MDagPath candidate = selectedPath;
        if (!candidate.extendToShapeDirectlyBelow(shapeIndex)) {
            continue;
        }
        if (!isRenderableMeshPath(candidate)) {
            continue;
        }

        resolvedPath = candidate;
        ++meshCount;
    }

    if (meshCount != 1) {
        return false;
    }

    meshPath = resolvedPath;
    return true;
}

const char* densityModeName(DensityMode mode)
{
    return mode == DensityMode::Manual ? "Manual" : "Auto";
}

const char* densityFallbackName(DensityFallback fallback)
{
    switch (fallback) {
    case DensityFallback::None:
        return "None";
    case DensityFallback::BoundaryEdges:
        return "Boundary edges";
    case DensityFallback::LocalRegionEdges:
        return "Local region edges";
    case DensityFallback::ManualDefault:
        return "Manual default";
    }
    return "Unknown";
}

struct DensitySolveEstimate final
{
    std::size_t faceCount = 0U;
    double requestedMinimum = 0.0;
    double requestedMean = 0.0;
    double requestedMaximum = 0.0;
    double effectiveMinimum = 0.0;
    double effectiveMean = 0.0;
    double effectiveMaximum = 0.0;
    double meanSourceEdgeLength = 0.0;
    double meanSolverScale = 0.0;
    double estimatedQuadCount = 0.0;
};

DensitySolveEstimate estimateDensitySolve(const solver::RemeshInput& input)
{
    DensitySolveEstimate result;
    double requestedSum = 0.0;
    double effectiveSum = 0.0;
    double sourceEdgeSum = 0.0;
    std::unordered_set<std::size_t> regionFaces;
    std::unordered_set<std::size_t> regionEdges;
    for (const solver::RegionComponent& component : input.components) {
        regionFaces.insert(
            component.allFaceIndices.begin(),
            component.allFaceIndices.end());
    }

    result.requestedMinimum = std::numeric_limits<double>::infinity();
    result.effectiveMinimum = std::numeric_limits<double>::infinity();
    for (const std::size_t faceIndex : regionFaces) {
        if (faceIndex >= input.sourceMesh.faces.size() ||
            faceIndex >= input.densityField.size()) {
            continue;
        }
        const solver::FaceDensity& density = input.densityField[faceIndex];
        if (!density.valid ||
            !(density.requestedTargetEdgeLength > 0.0) ||
            !(density.effectiveTargetEdgeLength > 0.0) ||
            !std::isfinite(density.requestedTargetEdgeLength) ||
            !std::isfinite(density.effectiveTargetEdgeLength)) {
            continue;
        }
        ++result.faceCount;
        requestedSum += density.requestedTargetEdgeLength;
        effectiveSum += density.effectiveTargetEdgeLength;
        result.requestedMinimum = std::min(
            result.requestedMinimum, density.requestedTargetEdgeLength);
        result.requestedMaximum = std::max(
            result.requestedMaximum, density.requestedTargetEdgeLength);
        result.effectiveMinimum = std::min(
            result.effectiveMinimum, density.effectiveTargetEdgeLength);
        result.effectiveMaximum = std::max(
            result.effectiveMaximum, density.effectiveTargetEdgeLength);

        const solver::SourceFace& face = input.sourceMesh.faces[faceIndex];
        regionEdges.insert(face.edgeIndices.begin(), face.edgeIndices.end());
        for (const std::size_t triangleIndex : face.triangleIndices) {
            if (triangleIndex >= input.sourceMesh.triangles.size()) {
                continue;
            }
            const solver::SourceTriangle& triangle =
                input.sourceMesh.triangles[triangleIndex];
            const solver::Vec3& a =
                input.sourceMesh.vertices[triangle.vertexIndices[0]].position;
            const solver::Vec3& b =
                input.sourceMesh.vertices[triangle.vertexIndices[1]].position;
            const solver::Vec3& c =
                input.sourceMesh.vertices[triangle.vertexIndices[2]].position;
            const double area = 0.5 * (b - a).cross(c - a).length();
            result.estimatedQuadCount += area /
                (density.effectiveTargetEdgeLength *
                 density.effectiveTargetEdgeLength);
        }
    }
    for (const std::size_t edgeIndex : regionEdges) {
        if (edgeIndex < input.sourceMesh.edges.size()) {
            sourceEdgeSum += input.sourceMesh.edges[edgeIndex].length;
        }
    }
    if (result.faceCount > 0U) {
        result.requestedMean = requestedSum /
            static_cast<double>(result.faceCount);
        result.effectiveMean = effectiveSum /
            static_cast<double>(result.faceCount);
    } else {
        result.requestedMinimum = 0.0;
        result.effectiveMinimum = 0.0;
    }
    if (!regionEdges.empty()) {
        result.meanSourceEdgeLength = sourceEdgeSum /
            static_cast<double>(regionEdges.size());
        if (result.meanSourceEdgeLength > 0.0) {
            result.meanSolverScale =
                result.effectiveMean / result.meanSourceEdgeLength;
        }
    }
    return result;
}

const char* boundaryWindingName(BoundaryWinding winding)
{
    switch (winding) {
    case BoundaryWinding::Aligned:
        return "Aligned";
    case BoundaryWinding::Reversed:
        return "Reversed";
    case BoundaryWinding::Unknown:
        break;
    }
    return "Unknown";
}

}  // namespace

DirectionalRetopoContext::DirectionalRetopoContext()
{
    setTitleString(kToolTitle);
    setHelpString(kHelpText);
    regionVisualizer_.setSettings(visualizationSettings_.region);
    fieldVisualizer_.setSettings(visualizationSettings_.field);
}

DirectionalRetopoContext::~DirectionalRetopoContext()
{
    removeAlternateContextMonitor();
    if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }
    brushCursorDrawable_.destroy();
    brushCursorModel_.reset();
    if (targetDisplayModel_) {
        targetDisplayModel_->clear();
    }
    targetDisplayDrawable_.destroy();
    targetDisplayModel_.reset();
    clearQuadPreview();
    quadPreviewDrawable_.destroy();
    quadPreviewModel_.reset();
    regionVisualizer_.clear();
    regionPreviewCalculator_.clear();
    finalPaintRegion_.clear();
    clearFinalFields();
    meshTopologyCache_.clear();
    rayCaster_.clearTarget();
    restoreOriginalSelection();
}

void DirectionalRetopoContext::toolOnSetup(MEvent& event)
{
    MPxContext::toolOnSetup(event);
    setHelpString(kHelpText);
    restoreOriginalSelection();
    regionVisualizer_.clear();
    regionPreviewCalculator_.clear();
    finalPaintRegion_.clear();
    clearFinalFields();
    meshTopologyCache_.clear();
    if (targetDisplayModel_) {
        targetDisplayModel_->clear();
    }
    targetDisplayDrawable_.destroy();
    targetDisplayModel_.reset();
    clearQuadPreview();
    quadPreviewDrawable_.destroy();
    quadPreviewModel_.reset();
    rayCaster_.clearTarget();
    rawStroke_.clear();
    processedStroke_.clear();
    regionFaceIds_.clear();
    feedbackState_.clearInteraction();
    strokeActive_ = false;
    radiusAdjustMode_ = false;
    radiusAdjustDragActive_ = false;
    brushCursorDrawable_.destroy();
    brushCursorModel_ = std::make_shared<BrushCursorModel>();
    brushCursorModel_->reset(
        brushSettings_.radius(),
        visualizationSettings_.brush);
    const MStatus cursorStatus = brushCursorDrawable_.create(brushCursorModel_);
    if (!cursorStatus) {
        displayTargetWarning(
            "The Brush Cursor VP2 drawable could not be created; "
            "painting remains available without a Brush Circle.");
    }
    targetDisplayModel_ = std::make_shared<TargetDisplayModel>();
    const MStatus targetDisplayStatus =
        targetDisplayDrawable_.create(targetDisplayModel_);
    if (!targetDisplayStatus) {
        displayTargetWarning(
            "The Target Display VP2 drawable could not be created; "
            "painting remains available without the custom target wireframe.");
    }
    quadPreviewModel_ = std::make_shared<QuadPreviewModel>();
    quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    const MStatus quadPreviewStatus =
        quadPreviewDrawable_.create(quadPreviewModel_);
    if (!quadPreviewStatus) {
        displayTargetWarning(
            "The Quad Preview VP2 drawable could not be created; "
            "quad solving remains available without Viewport Preview.");
    }
    toolActive_ = true;
    alternateContextObserved_ = inAlternateContext();
    installAlternateContextMonitor();

    MGlobal::displayInfo("[DirectionalRetopo] Tool activated");
    if (acquireTargetFromSelection(true)) {
        updateCursorHit(event);
    }
    requestFeedbackRefresh();

    const MStatus runtimeStatus = activatePythonRuntime();
    if (!runtimeStatus) {
        displayTargetWarning(
            "B-key runtime activation failed; Brush painting remains available.");
    }
}

void DirectionalRetopoContext::toolOffCleanup()
{
    const MStatus runtimeStatus = deactivatePythonRuntime();
    if (!runtimeStatus) {
        displayTargetWarning("B-key runtime cleanup failed.");
    }

    toolActive_ = false;
    removeAlternateContextMonitor();
    strokeActive_ = false;
    radiusAdjustMode_ = false;
    radiusAdjustDragActive_ = false;
    if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }
    rawStroke_.clear();
    processedStroke_.clear();
    regionFaceIds_.clear();
    feedbackState_.clearInteraction();
    regionVisualizer_.clear();
    regionPreviewCalculator_.clear();
    finalPaintRegion_.clear();
    clearFinalFields();
    meshTopologyCache_.clear();
    if (targetDisplayModel_) {
        targetDisplayModel_->clear();
    }
    clearQuadPreview();
    rayCaster_.clearTarget();
    brushCursorDrawable_.destroy();
    brushCursorModel_.reset();
    targetDisplayDrawable_.destroy();
    targetDisplayModel_.reset();
    quadPreviewDrawable_.destroy();
    quadPreviewModel_.reset();
    restoreOriginalSelection();
    requestFeedbackRefresh();
    MPxContext::toolOffCleanup();
}

MStatus DirectionalRetopoContext::doPress(
    MEvent& event,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext)
{
    const bool alternateContextActive = inAlternateContext();
    updateAlternateContextState(alternateContextActive);
    if (alternateContextActive) {
        (void)MPxContext::doPress(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    MStatus status;
    if (event.mouseButton(&status) != MEvent::kLeftMouse || !status) {
        (void)MPxContext::doPress(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    if (radiusAdjustMode_) {
        short y = 0;
        if (!event.getPosition(radiusAdjustStartX_, y)) {
            (void)MPxContext::doPress(event, drawManager, frameContext);
            return MS::kSuccess;
        }

        strokeActive_ = false;
        radiusAdjustDragActive_ = true;
        radiusAdjustStartRadius_ = brushSettings_.radius();

        feedbackState_.clearStroke();
        if (brushCursorModel_ && !brushCursorModel_->hasFreshHit()) {
            updateCursorHit(event);
        }
        if (brushCursorModel_) {
            brushCursorModel_->beginRadiusAdjust();
        }
        requestFeedbackRefresh();
        (void)MPxContext::doPress(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    rawStroke_.clear();
    feedbackState_.clearStroke();
    resetRegionPreview();
    strokeActive_ = false;

    if (!hasValidTarget(true)) {
        requestFeedbackRefresh();
        (void)MPxContext::doPress(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    strokeActive_ = true;
    MGlobal::displayInfo("[DirectionalRetopo] Stroke begin");
    captureArmedAtStrokeStart_ = !pendingRemeshCapturePath_.empty();
    if (captureArmedAtStrokeStart_) {
        std::ostringstream captureMessage;
        captureMessage << "[DirectionalRetopo][CaptureDebug]\n"
                       << "Stroke begin\n"
                       << "Capture armed: true\n"
                       << "Capture path: " << pendingRemeshCapturePath_;
        MGlobal::displayInfo(MString(captureMessage.str().c_str()));
    }

    SurfaceHit hit;
    if (castEventToTarget(event, hit)) {
        setBrushCursorFromHit(event, hit);
        appendSurfaceHit(hit, 0.0);
    } else if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }

    requestFeedbackRefresh();
    (void)MPxContext::doPress(event, drawManager, frameContext);
    return MS::kSuccess;
}

MStatus DirectionalRetopoContext::doDrag(
    MEvent& event,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext)
{
    const bool alternateContextActive = inAlternateContext();
    updateAlternateContextState(alternateContextActive);
    if (alternateContextActive) {
        (void)MPxContext::doDrag(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    MStatus status;
    if (event.mouseButton(&status) != MEvent::kLeftMouse || !status) {
        (void)MPxContext::doDrag(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    if (radiusAdjustDragActive_) {
        short x = 0;
        short y = 0;
        if (!event.getPosition(x, y)) {
            (void)MPxContext::doDrag(event, drawManager, frameContext);
            return MS::kSuccess;
        }

        const double horizontalDelta = static_cast<double>(x - radiusAdjustStartX_);
        const double scale = std::pow(2.0, horizontalDelta / kRadiusPixelsPerDoubling);
        setBrushRadius(std::clamp(
            radiusAdjustStartRadius_ * scale,
            BrushSettings::kMinimumRadius,
            kMaximumBrushRadius));
        (void)MPxContext::doDrag(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    if (!strokeActive_) {
        (void)MPxContext::doDrag(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    SurfaceHit hit;
    if (castEventToTarget(event, hit)) {
        setBrushCursorFromHit(event, hit);
        appendSurfaceHit(hit, brushSettings_.sampleSpacing());
    } else if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }

    requestFeedbackRefresh();
    (void)MPxContext::doDrag(event, drawManager, frameContext);
    return MS::kSuccess;
}

MStatus DirectionalRetopoContext::doRelease(
    MEvent& event,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext)
{
    const bool alternateContextActive = inAlternateContext();
    updateAlternateContextState(alternateContextActive);
    if (alternateContextActive) {
        (void)MPxContext::doRelease(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    if (radiusAdjustDragActive_) {
        radiusAdjustDragActive_ = false;
        requestFeedbackRefresh();
        (void)MPxContext::doRelease(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    if (!strokeActive_) {
        (void)MPxContext::doRelease(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    strokeActive_ = false;
    finalizeProcessedStroke();
    generateFinalPaintRegion();
    const std::string message =
        "[DirectionalRetopo] Stroke end: " + std::to_string(rawStroke_.size()) +
        " raw / " + std::to_string(processedStroke_.size()) + " processed samples";
    MGlobal::displayInfo(MString(message.c_str()));
    if (captureArmedAtStrokeStart_) {
        std::ostringstream captureMessage;
        captureMessage << "[DirectionalRetopo][CaptureDebug]\n"
                       << "Stroke end\n"
                       << "Capture flag cleared: "
                       << (pendingRemeshCapturePath_.empty() ? "true" : "false");
        MGlobal::displayInfo(MString(captureMessage.str().c_str()));
        captureArmedAtStrokeStart_ = false;
    }
    requestFeedbackRefresh();
    (void)MPxContext::doRelease(event, drawManager, frameContext);
    return MS::kSuccess;
}

MStatus DirectionalRetopoContext::doPtrMoved(
    MEvent& event,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext)
{
    const bool alternateContextActive = inAlternateContext();
    updateAlternateContextState(alternateContextActive);
    if (alternateContextActive) {
        (void)MPxContext::doPtrMoved(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    // Radius adjustment owns the single cursor's center. Pointer movement in
    // this state changes only the radius and must not run the hover ray cast.
    if (radiusAdjustMode_ || radiusAdjustDragActive_) {
        (void)MPxContext::doPtrMoved(event, drawManager, frameContext);
        return MS::kSuccess;
    }

    if (!hasValidTarget(false)) {
        invalidateHoverFeedback(true);
        (void)MPxContext::doPtrMoved(event, drawManager, frameContext);
        return MS::kSuccess;
    }
    updateCursorHit(event);
    requestBrushCursorRefresh();
    (void)MPxContext::doPtrMoved(event, drawManager, frameContext);
    return MS::kSuccess;
}

MStatus DirectionalRetopoContext::drawFeedback(
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& /*frameContext*/)
{
    const bool alternateContextActive = inAlternateContext();

    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);

    // Target Wireframe is owned by its persistent DrawOverride. Context
    // feedback retains only Region Preview and transient Stroke/Direction.
    (void)regionVisualizer_.draw(drawManager);
    fieldVisualizer_.draw(drawManager);

    const ActiveStrokeVisualizationState& strokeState = feedbackState_.activeStroke;
    const StrokeVisualizationSettings& strokeStyle = visualizationSettings_.stroke;
    if (!alternateContextActive && strokeState.visible && !strokeState.stroke.empty()) {
        MPointArray strokePoints;
        for (const StrokeSample& sample : strokeState.stroke.samples()) {
            const double offset = std::max(
                sample.radius * strokeStyle.surfaceOffsetRadiusRatio,
                strokeStyle.minimumSurfaceOffset);
            strokePoints.append(sample.position + sample.normal * offset);
        }

        drawManager.setColor(strokeStyle.strokeColor);
        drawManager.setDepthPriority(strokeStyle.strokeDepthPriority);
        drawManager.setLineWidth(strokeStyle.strokeLineWidth);
        if (strokePoints.length() == 1) {
            drawManager.setPointSize(strokeStyle.singleSamplePointSize);
            drawManager.point(strokePoints[0]);
        } else {
            (void)drawManager.lineStrip(strokePoints, false);
        }
    }

    const DirectionVisualizationState& directionState = feedbackState_.direction;
    const DirectionVisualizationSettings& directionStyle =
        visualizationSettings_.direction;
    if (!alternateContextActive && directionState.visible &&
        directionState.direction.length() > kDirectionEpsilon) {
        const double offset = std::max(
            directionState.radius * directionStyle.surfaceOffsetRadiusRatio,
            directionStyle.minimumSurfaceOffset);
        const MPoint directionStart =
            directionState.position + directionState.normal * offset;
        const MPoint directionEnd = directionStart + directionState.direction *
            (directionState.radius * directionStyle.lengthRadiusRatio);
        drawManager.setColor(directionStyle.directionColor);
        drawManager.setDepthPriority(directionStyle.directionDepthPriority);
        drawManager.setLineWidth(directionStyle.directionLineWidth);
        drawManager.line(directionStart, directionEnd);
    }

    drawManager.endDrawable();
    return MS::kSuccess;
}

MStatus DirectionalRetopoContext::doEnterRegion(MEvent& event)
{
    const MStatus status = setHelpString(kHelpText);
    const bool alternateContextActive = inAlternateContext();
    updateAlternateContextState(alternateContextActive);
    if (alternateContextActive) {
        return status;
    }

    if (radiusAdjustMode_ || radiusAdjustDragActive_) {
        return status;
    }

    if (!hasValidTarget(false)) {
        invalidateHoverFeedback(true);
        return status;
    }
    updateCursorHit(event);
    requestBrushCursorRefresh();
    return status;
}

MStatus DirectionalRetopoContext::doExitRegion(MEvent& event)
{
    invalidateHoverFeedback(true);
    return MPxContext::doExitRegion(event);
}

void DirectionalRetopoContext::getClassName(MString& name) const
{
    name.set("DirectionalRetopoContext");
}

double DirectionalRetopoContext::brushRadius() const noexcept
{
    return brushSettings_.radius();
}

void DirectionalRetopoContext::setBrushRadius(double radius)
{
    brushSettings_.setRadius(radius);
    if (brushCursorModel_) {
        brushCursorModel_->setRadius(brushSettings_.radius());
    }
    requestBrushCursorRefresh();
}

bool DirectionalRetopoContext::radiusAdjustMode() const noexcept
{
    return radiusAdjustMode_;
}

void DirectionalRetopoContext::setRadiusAdjustMode(bool enabled)
{
    bool contextFeedbackChanged = false;
    if (enabled && strokeActive_) {
        strokeActive_ = false;
        finalizeProcessedStroke();
        contextFeedbackChanged = true;
    }

    radiusAdjustMode_ = enabled;
    if (enabled) {
        if (brushCursorModel_) {
            brushCursorModel_->setRadius(brushSettings_.radius());
            brushCursorModel_->beginRadiusAdjust();
        }
    } else {
        radiusAdjustDragActive_ = false;
        radiusAdjustStartX_ = 0;
        if (brushCursorModel_) {
            // B Release never restores the anchored hit. The next normal
            // pointer event is required to produce a fresh hover ray cast.
            brushCursorModel_->endRadiusAdjust();
        }
    }
    if (contextFeedbackChanged) {
        requestFeedbackRefresh();
    } else {
        requestBrushCursorRefresh();
    }
}

DensityMode DirectionalRetopoContext::densityMode() const noexcept
{
    return densityFieldBuilder_.settings().mode;
}

void DirectionalRetopoContext::setDensityMode(DensityMode mode)
{
    const auto changeStart = std::chrono::steady_clock::now();
    const DensityMode previousMode = densityFieldBuilder_.settings().mode;
    const std::uint64_t invocationCountBefore = remeshSolverInvocationCount_;
    if (previousMode == mode) {
        std::ostringstream message;
        message << "[DirectionalRetopo][DensityDebug]\n"
                << "Mode change: " << densityModeName(previousMode) << " -> "
                << densityModeName(mode) << " (no-op)\n"
                << "Manual target: " << manualTargetEdgeLength() << "\n"
                << "Edge scale: " << densityEdgeLengthScale() << "\n"
                << "Preview rebuild requested: false\n"
                << "Solver invoked: false\n"
                << "Solver invocation count: " << remeshSolverInvocationCount_ << "\n"
                << "Mode change total: 0.00 ms";
        MGlobal::displayInfo(MString(message.str().c_str()));
        return;
    }

    DensityFieldBuilderSettings settings = densityFieldBuilder_.settings();
    settings.mode = mode;
    densityFieldBuilder_.setSettings(settings);
    if (!finalPaintRegion_.components.empty()) {
        // A Tool Settings edit must stay interactive.  Refresh the lightweight
        // fields and invalidate the stale Preview, but defer the synchronous
        // remesh solve until the next committed Stroke.
        generateFinalFields(false);
    }
    requestFeedbackRefresh();

    const double elapsedMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - changeStart).count();
    const std::uint64_t invocationDelta =
        remeshSolverInvocationCount_ - invocationCountBefore;
    std::ostringstream message;
    message << "[DirectionalRetopo][DensityDebug]\n"
            << "Mode change: " << densityModeName(previousMode) << " -> "
            << densityModeName(mode) << "\n"
            << "Manual target: " << manualTargetEdgeLength() << "\n"
            << "Edge scale: " << densityEdgeLengthScale() << "\n"
            << "Preview rebuild requested: false\n"
            << "Solver invoked: " << (invocationDelta != 0U ? "true" : "false") << "\n"
            << "Solver invocation count: " << remeshSolverInvocationCount_ << "\n"
            << "Mode change total: " << std::fixed << std::setprecision(2)
            << elapsedMilliseconds << " ms";
    MGlobal::displayInfo(MString(message.str().c_str()));
}

double DirectionalRetopoContext::manualTargetEdgeLength() const noexcept
{
    return densityFieldBuilder_.settings().manualTargetEdgeLength;
}

void DirectionalRetopoContext::setManualTargetEdgeLength(double edgeLength)
{
    if (manualTargetEdgeLength() == edgeLength) {
        return;
    }
    DensityFieldBuilderSettings settings = densityFieldBuilder_.settings();
    settings.manualTargetEdgeLength = edgeLength;
    densityFieldBuilder_.setSettings(settings);
    if (!finalPaintRegion_.components.empty()) {
        generateFinalFields(false);
    }
    requestFeedbackRefresh();
}

double DirectionalRetopoContext::densityEdgeLengthScale() const noexcept
{
    return densityFieldBuilder_.settings().edgeLengthScale;
}

void DirectionalRetopoContext::setDensityEdgeLengthScale(double scale)
{
    if (densityEdgeLengthScale() == scale) {
        return;
    }
    DensityFieldBuilderSettings settings = densityFieldBuilder_.settings();
    settings.edgeLengthScale = scale;
    densityFieldBuilder_.setSettings(settings);
    if (!finalPaintRegion_.components.empty()) {
        generateFinalFields(false);
    }
    requestFeedbackRefresh();
}

int DirectionalRetopoContext::topologyBlendWidth() const noexcept
{
    return paintRegionSolver_.settings().transitionRings;
}

void DirectionalRetopoContext::setTopologyBlendWidth(int rings)
{
    PaintRegionSolverSettings settings = paintRegionSolver_.settings();
    settings.transitionRings = rings;
    paintRegionSolver_.setSettings(settings);
    if (!processedStroke_.empty()) {
        // Transition width is shared by Paint Region expansion, Direction Field
        // topology guidance and Density Field boundary interpolation. Rebuild
        // only on a committed Tool Settings/context-command edit.
        generateFinalPaintRegion();
    }
    requestFeedbackRefresh();
}

bool DirectionalRetopoContext::showDirectionField() const noexcept
{
    return visualizationSettings_.field.showDirectionField;
}

void DirectionalRetopoContext::setShowDirectionField(bool show)
{
    visualizationSettings_.field.showDirectionField = show;
    fieldVisualizer_.setSettings(visualizationSettings_.field);
    requestFeedbackRefresh();
}

bool DirectionalRetopoContext::showDensityField() const noexcept
{
    return visualizationSettings_.field.showDensityField;
}

void DirectionalRetopoContext::setShowDensityField(bool show)
{
    visualizationSettings_.field.showDensityField = show;
    fieldVisualizer_.setSettings(visualizationSettings_.field);
    requestFeedbackRefresh();
}

bool DirectionalRetopoContext::showQuadPreview() const noexcept
{
    return visualizationSettings_.quadPreview.showQuadPreview;
}

void DirectionalRetopoContext::setShowQuadPreview(bool show)
{
    visualizationSettings_.quadPreview.showQuadPreview = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

bool DirectionalRetopoContext::showRawQuadPreview() const noexcept
{
    return visualizationSettings_.quadPreview.showRawQuadPreview;
}

void DirectionalRetopoContext::setShowRawQuadPreview(bool show)
{
    visualizationSettings_.quadPreview.showRawQuadPreview = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

bool DirectionalRetopoContext::showConformedQuadPreview() const noexcept
{
    return visualizationSettings_.quadPreview.showConformedQuadPreview;
}

void DirectionalRetopoContext::setShowConformedQuadPreview(bool show)
{
    visualizationSettings_.quadPreview.showConformedQuadPreview = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

bool DirectionalRetopoContext::showSourceBoundary() const noexcept
{
    return visualizationSettings_.quadPreview.showSourceBoundary;
}

void DirectionalRetopoContext::setShowSourceBoundary(bool show)
{
    visualizationSettings_.quadPreview.showSourceBoundary = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

bool DirectionalRetopoContext::showResultBoundary() const noexcept
{
    return visualizationSettings_.quadPreview.showResultBoundary;
}

void DirectionalRetopoContext::setShowResultBoundary(bool show)
{
    visualizationSettings_.quadPreview.showResultBoundary = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

bool DirectionalRetopoContext::showBoundaryCorrespondence() const noexcept
{
    return visualizationSettings_.quadPreview.showBoundaryCorrespondence;
}

void DirectionalRetopoContext::setShowBoundaryCorrespondence(bool show)
{
    visualizationSettings_.quadPreview.showBoundaryCorrespondence = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

bool DirectionalRetopoContext::showRequiredBoundaryAnchors() const noexcept
{
    return visualizationSettings_.quadPreview.showRequiredBoundaryAnchors;
}

void DirectionalRetopoContext::setShowRequiredBoundaryAnchors(bool show)
{
    visualizationSettings_.quadPreview.showRequiredBoundaryAnchors = show;
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    requestQuadPreviewRefresh();
}

const std::string& DirectionalRetopoContext::pendingRemeshCapturePath() const noexcept
{
    return pendingRemeshCapturePath_;
}

void DirectionalRetopoContext::setPendingRemeshCapturePath(std::string path)
{
    pendingRemeshCapturePath_ = std::move(path);
    std::ostringstream message;
    message << "[DirectionalRetopo][CaptureDebug]\n"
            << "Capture armed: "
            << (!pendingRemeshCapturePath_.empty() ? "true" : "false") << "\n"
            << "Capture path: " << pendingRemeshCapturePath_;
    MGlobal::displayInfo(MString(message.str().c_str()));
}

void DirectionalRetopoContext::resetToolSettings()
{
    brushSettings_ = BrushSettings();
    paintRegionSolver_.setSettings(PaintRegionSolverSettings());
    densityFieldBuilder_.setSettings(DensityFieldBuilderSettings());
    const ViewportVisualizationSettings defaults;
    visualizationSettings_.quadPreview = defaults.quadPreview;
    if (brushCursorModel_) {
        brushCursorModel_->setRadius(brushSettings_.radius());
    }
    if (quadPreviewModel_) {
        quadPreviewModel_->setSettings(visualizationSettings_.quadPreview);
    }
    if (!processedStroke_.empty()) {
        generateFinalPaintRegion();
    }
    requestFeedbackRefresh();
    requestBrushCursorRefresh();
    requestQuadPreviewRefresh();
}

QuadSolveInput DirectionalRetopoContext::quadSolveInput() const noexcept
{
    return QuadSolveInput{
        &finalPaintRegion_,
        &directionFieldData_,
        &densityFieldData_};
}

bool DirectionalRetopoContext::acquireTargetFromSelection(bool displayErrors)
{
    MSelectionList selection;
    MStatus status = MGlobal::getActiveSelectionList(selection);
    if (!status || selection.length() != 1) {
        rayCaster_.clearTarget();
        if (displayErrors) {
            displayTargetWarning("Select exactly one polygon mesh before painting.");
        }
        return false;
    }

    originalSelection_ = selection;
    originalSelectionSaved_ = true;

    MDagPath selectedPath;
    MObject component;
    status = selection.getDagPath(0, selectedPath, component);
    if (!status) {
        rayCaster_.clearTarget();
        if (displayErrors) {
            displayTargetWarning("The selected item is not a polygon mesh DAG object.");
        }
        return false;
    }

    MDagPath meshPath;
    if (!resolveMeshShape(selectedPath, meshPath)) {
        rayCaster_.clearTarget();
        if (displayErrors) {
            displayTargetWarning(
                "The selection must resolve to one non-intermediate polygon mesh shape.");
        }
        return false;
    }

    status = rayCaster_.setTarget(meshPath);
    if (!status) {
        if (displayErrors) {
            displayTargetWarning("The target mesh could not be initialized for ray casting.");
        }
        return false;
    }

    status = targetDisplayModel_
        ? targetDisplayModel_->setTarget(meshPath, visualizationSettings_.target)
        : MS::kFailure;
    if (!status) {
        if (targetDisplayModel_) {
            targetDisplayModel_->clear();
        }
        if (displayErrors) {
            displayTargetWarning(
                "The target visualization cache could not be initialized; "
                "painting will continue without the custom target wireframe.");
        }
    }

    status = regionPreviewCalculator_.setTarget(meshPath);
    if (!status) {
        regionPreviewCalculator_.clear();
        if (displayErrors) {
            displayTargetWarning(
                "The provisional region calculation cache could not be initialized; "
                "painting will continue without Region Preview calculation.");
        }
    }

    status = meshTopologyCache_.setTarget(meshPath);
    if (!status) {
        meshTopologyCache_.clear();
        if (displayErrors) {
            displayTargetWarning(
                "The Paint Region topology cache could not be initialized; "
                "interactive painting remains available without a Final Region solve.");
        }
    }

    status = regionVisualizer_.setTarget(meshPath);
    if (!status) {
        regionVisualizer_.clear();
        if (displayErrors) {
            displayTargetWarning(
                "The Region Preview renderer could not be initialized; "
                "brush and stroke feedback remain available.");
        }
    }

    status = MGlobal::clearSelectionList();
    if (!status) {
        regionVisualizer_.clear();
        regionPreviewCalculator_.clear();
        finalPaintRegion_.clear();
        clearFinalFields();
        meshTopologyCache_.clear();
        if (targetDisplayModel_) {
            targetDisplayModel_->clear();
        }
        rayCaster_.clearTarget();
        if (displayErrors) {
            displayTargetWarning("Maya's active selection could not be cleared safely.");
        }
        return false;
    }

    return true;
}

bool DirectionalRetopoContext::hasValidTarget(bool displayErrors)
{
    // Target visualization is optional feedback. Ray casting owns the target
    // required by painting and must remain usable if visualization degrades.
    if (rayCaster_.hasTarget()) {
        return true;
    }

    feedbackState_.clearInteraction();
    if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }
    if (displayErrors) {
        displayTargetWarning("The stored target mesh is no longer available.");
    }
    return false;
}

void DirectionalRetopoContext::restoreOriginalSelection() noexcept
{
    if (!originalSelectionSaved_) {
        return;
    }

    MSelectionList restorableSelection;
    const unsigned int itemCount = originalSelection_.length();
    for (unsigned int index = 0; index < itemCount; ++index) {
        MDagPath dagPath;
        MObject component;
        if (originalSelection_.getDagPath(index, dagPath, component) == MS::kSuccess &&
            dagPath.isValid()) {
            const MObjectHandle nodeHandle(dagPath.node());
            if (nodeHandle.isValid() && nodeHandle.isAlive()) {
                if (restorableSelection.add(dagPath, component) != MS::kSuccess) {
                    (void)restorableSelection.add(dagPath);
                }
            }
            continue;
        }

        MObject dependencyNode;
        if (originalSelection_.getDependNode(index, dependencyNode) == MS::kSuccess) {
            const MObjectHandle nodeHandle(dependencyNode);
            if (nodeHandle.isValid() && nodeHandle.isAlive()) {
                (void)restorableSelection.add(dependencyNode);
            }
        }
    }

    (void)MGlobal::setActiveSelectionList(restorableSelection, MGlobal::kReplaceList);
    originalSelection_.clear();
    originalSelectionSaved_ = false;
}

bool DirectionalRetopoContext::castEventToTarget(MEvent& event, SurfaceHit& hit)
{
    short x = 0;
    short y = 0;
    if (!event.getPosition(x, y)) {
        return false;
    }
    return rayCaster_.castFromViewport(x, y, hit);
}

void DirectionalRetopoContext::setBrushCursorFromHit(
    MEvent& event,
    const SurfaceHit& hit)
{
    short x = 0;
    short y = 0;
    if (!event.getPosition(x, y)) {
        if (brushCursorModel_) {
            brushCursorModel_->invalidateForFreshRaycast();
        }
        return;
    }

    if (!brushCursorModel_) {
        return;
    }
    brushCursorModel_->setFreshHit(
        x,
        y,
        hit.position,
        hit.normal,
        brushSettings_.radius(),
        radiusAdjustMode_ ? BrushCursorMode::RadiusAdjust : BrushCursorMode::Hover);
}

bool DirectionalRetopoContext::appendSurfaceHit(
    const SurfaceHit& hit,
    double minimumSpacing)
{
    StrokeSample sample;
    sample.position = hit.position;
    sample.normal = hit.normal;
    sample.weight = 1.0;
    sample.radius = brushSettings_.radius();
    sample.faceId = hit.faceId;
    sample.triangleId = hit.triangleId;
    sample.barycentric1 = hit.barycentric1;
    sample.barycentric2 = hit.barycentric2;
    const bool appended = rawStroke_.append(sample, minimumSpacing);
    if (appended) {
        updateActiveStrokeVisualization();
        if (regionPreviewCalculator_.addFacesForSample(sample, regionFaceIds_)) {
            regionVisualizer_.setFaceIds(regionFaceIds_);
        }
    }
    return appended;
}

void DirectionalRetopoContext::updateCursorHit(MEvent& event)
{
    SurfaceHit hit;
    if (castEventToTarget(event, hit)) {
        setBrushCursorFromHit(event, hit);
    } else if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }
}

void DirectionalRetopoContext::updateActiveStrokeVisualization()
{
    StrokeData activeStroke = strokeProcessor_.process(
        rawStroke_,
        brushSettings_.sampleSpacing(),
        brushSettings_.radius(),
        false);
    feedbackState_.activeStroke.set(std::move(activeStroke));

    if (!feedbackState_.activeStroke.stroke.empty()) {
        const StrokeSample& latest = feedbackState_.activeStroke.stroke.back();
        if (latest.direction.length() > kDirectionEpsilon) {
            feedbackState_.direction.set(latest);
        } else {
            feedbackState_.direction.clear();
        }
    } else {
        feedbackState_.direction.clear();
    }
}

void DirectionalRetopoContext::finalizeProcessedStroke()
{
    processedStroke_ = strokeProcessor_.process(
        rawStroke_,
        brushSettings_.sampleSpacing(),
        brushSettings_.radius(),
        true);
    feedbackState_.clearStroke();
}

void DirectionalRetopoContext::generateFinalPaintRegion()
{
    finalPaintRegion_.clear();
    clearFinalFields();
    if (processedStroke_.empty()) {
        regionVisualizer_.clearFaceIds();
        return;
    }

    const auto solveStart = std::chrono::steady_clock::now();
    const MStatus cacheStatus = meshTopologyCache_.ensureCurrent();
    if (!cacheStatus) {
        regionVisualizer_.clearFaceIds();
        displayTargetWarning(
            "Final Region generation failed because the topology cache is unavailable.");
        return;
    }

    PaintRegionSolveMetrics metrics;
    const bool solved = paintRegionSolver_.solve(
        processedStroke_,
        meshTopologyCache_,
        finalPaintRegion_,
        &metrics);
    const auto solveEnd = std::chrono::steady_clock::now();
    const double elapsedMilliseconds =
        std::chrono::duration<double, std::milli>(solveEnd - solveStart).count();
    if (!solved) {
        regionVisualizer_.clearFaceIds();
        displayTargetWarning(
            "Final Region generation produced no components; the target mesh was not modified.");
        return;
    }

    regionVisualizer_.setFinalRegion(finalPaintRegion_);
    std::ostringstream message;
    message << "[DirectionalRetopo] Region generated\n"
            << "Core faces: " << finalPaintRegion_.coreFaceCount() << '\n'
            << "Transition faces: " << finalPaintRegion_.transitionFaceCount() << '\n'
            << "Total faces: " << finalPaintRegion_.totalFaceCount() << '\n'
            << "Components: " << finalPaintRegion_.components.size() << '\n'
            << "Boundary loops/chains: " << finalPaintRegion_.boundaryLoopCount() << '\n'
            << "Boundary edges: " << finalPaintRegion_.boundaryEdgeCount() << '\n'
            << "Dijkstra vertices/edge visits: " << metrics.visitedVertexCount << '/'
            << metrics.expandedEdgeCount << '\n'
            << "Solve time: " << std::fixed << std::setprecision(2)
            << elapsedMilliseconds << " ms";
    MGlobal::displayInfo(MString(message.str().c_str()));
    if (finalPaintRegion_.hasAmbiguousBoundary()) {
        displayTargetWarning(
            "The Final Region contains a branching or non-manifold boundary; "
            "ordered finite chains were retained for inspection.");
    }
    generateFinalFields();
}

void DirectionalRetopoContext::generateFinalFields(bool generatePreview)
{
    directionFieldData_.clear();
    densityFieldData_.clear();
    fieldVisualizer_.clear();
    clearQuadPreview();
    if (finalPaintRegion_.components.empty() || processedStroke_.empty()) {
        return;
    }

    const auto directionStart = std::chrono::steady_clock::now();
    DirectionFieldBuildMetrics directionMetrics;
    const bool directionGenerated = directionFieldBuilder_.build(
        processedStroke_,
        finalPaintRegion_,
        meshTopologyCache_,
        directionFieldData_,
        &directionMetrics);
    const auto directionEnd = std::chrono::steady_clock::now();
    const double directionMilliseconds =
        std::chrono::duration<double, std::milli>(directionEnd - directionStart).count();

    std::ostringstream directionMessage;
    directionMessage << "[DirectionalRetopo] Direction field generated\n"
                     << "Faces: " << directionMetrics.regionFaceCount << '\n'
                     << "Paint constrained: "
                     << directionMetrics.paintConstrainedFaceCount << '\n'
                     << "Invalid: " << directionMetrics.invalidFaceCount << '\n'
                     << "Iterations: " << directionMetrics.smoothingIterations << '\n'
                     << "Solve time: " << std::fixed << std::setprecision(2)
                     << directionMilliseconds << " ms";
    MGlobal::displayInfo(MString(directionMessage.str().c_str()));
    if (!directionGenerated) {
        displayTargetWarning(
            "Direction Field contains no valid Region faces; Density generation will continue.");
    }

    const auto densityStart = std::chrono::steady_clock::now();
    DensityFieldBuildMetrics densityMetrics;
    const bool densityGenerated = densityFieldBuilder_.build(
        finalPaintRegion_,
        meshTopologyCache_,
        densityFieldData_,
        &densityMetrics);
    const auto densityEnd = std::chrono::steady_clock::now();
    const double densityMilliseconds =
        std::chrono::duration<double, std::milli>(densityEnd - densityStart).count();

    std::ostringstream densityMessage;
    densityMessage << "[DirectionalRetopo] Density field generated\n"
                   << "Mode: " << densityModeName(densityMetrics.mode) << '\n'
                   << "Reference edges: " << densityMetrics.referenceEdgeCount << '\n'
                   << "Median edge length: " << std::fixed << std::setprecision(4)
                   << densityMetrics.medianReferenceEdgeLength << '\n'
                   << "Edge Length Scale: " << densityMetrics.edgeLengthScale << '\n'
                   << "Effective target range/mean: "
                   << densityMetrics.minimumTargetEdgeLength << " - "
                   << densityMetrics.maximumTargetEdgeLength << " / "
                   << densityMetrics.meanTargetEdgeLength << '\n'
                   << "Curvature constrained faces: "
                   << densityMetrics.curvatureConstrainedFaceCount << '\n'
                   << "Curvature minimum target: "
                   << densityMetrics.minimumCurvatureTargetEdgeLength << '\n'
                   << "Curvature indicator mean/max: "
                   << densityMetrics.meanCurvatureIndicator << " / "
                   << densityMetrics.maximumCurvatureIndicator << '\n'
                   << "High-curvature source edge mean: "
                   << densityMetrics.meanCurvatureRegionSourceEdgeLength << '\n'
                   << "Maximum curvature refinement: "
                   << densityMetrics.maximumAppliedCurvatureRefinementFactor
                   << "x\n"
                   << "Fallback: " << densityFallbackName(densityMetrics.fallback) << '\n'
                   << "Solve time: " << std::setprecision(2)
                   << densityMilliseconds << " ms";
    MGlobal::displayInfo(MString(densityMessage.str().c_str()));
    if (!densityGenerated) {
        displayTargetWarning("Density Field generation produced no valid face values.");
    }

    fieldVisualizer_.setData(
        meshTopologyCache_,
        finalPaintRegion_,
        directionFieldData_,
        densityFieldData_);
    if (directionGenerated && densityGenerated && generatePreview) {
        generateQuadPreview();
    }
}

void DirectionalRetopoContext::generateQuadPreview()
{
    clearQuadPreview();
    if (finalPaintRegion_.components.empty() ||
        directionFieldData_.empty() || densityFieldData_.empty()) {
        return;
    }

    solver::RemeshSettings settings;
    settings.topologyBlendWidth = static_cast<unsigned int>(std::max(
        paintRegionSolver_.settings().transitionRings,
        PaintRegionSolverSettings::kMinimumTransitionRings));
    settings.topologyPolicy = solver::TopologyPolicy::QuadDominant;
    settings.trianglePolicy = solver::TrianglePolicy::MinimalNecessary;
    settings.maximumRetryAttempts = 3U;
    settings.retainDebugResults = true;

    const auto adapterStart = std::chrono::steady_clock::now();
    solver::RemeshInput input;
    std::string adapterDiagnostic;
    if (!MayaRemeshInputAdapter::build(
            meshTopologyCache_, finalPaintRegion_, directionFieldData_,
            densityFieldData_, settings, input, adapterDiagnostic)) {
        displayTargetWarning(adapterDiagnostic.c_str());
        requestQuadPreviewRefresh();
        return;
    }
    const double adapterMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - adapterStart).count();
    const std::string capturePath = pendingRemeshCapturePath_;
    solver::RemeshCaptureRecord captureRecord;
    bool captureInputSaved = false;
    if (!capturePath.empty()) {
        std::ostringstream inputMessage;
        inputMessage << "[DirectionalRetopo][CaptureDebug]\n"
                     << "RemeshInput complete\n"
                     << "Capture armed: true\n"
                     << "Density mode: " << densityModeName(densityMode()) << "\n"
                     << "Serialization requested: true\n"
                     << "Capture path: " << capturePath;
        MGlobal::displayInfo(MString(inputMessage.str().c_str()));

        // One-shot capture is consumed by the next valid portable input.  It
        // is deliberately serialized before any solver safety check so a
        // valid Manual input remains capturable even when solving is skipped.
        pendingRemeshCapturePath_.clear();
        captureRecord.label = capturePath;
        captureRecord.input = input;
        std::string captureDiagnostic;
        captureInputSaved = solver::saveRemeshCapture(
            capturePath, captureRecord, captureDiagnostic);
        std::ostringstream resultMessage;
        resultMessage << "[DirectionalRetopo][CaptureDebug]\n"
                      << "Serialization complete\n"
                      << "Capture success: "
                      << (captureInputSaved ? "true" : "false") << "\n"
                      << "Saved path: " << capturePath << "\n"
                      << "Capture flag cleared: true";
        if (captureInputSaved) {
            resultMessage << "\nInput signature: 0x" << std::hex
                          << solver::remeshInputSignature(input);
            MGlobal::displayInfo(MString(resultMessage.str().c_str()));
        } else {
            resultMessage << "\nDiagnostic: " << captureDiagnostic;
            displayTargetWarning(resultMessage.str().c_str());
        }
    }
    const DensitySolveEstimate densityEstimate = estimateDensitySolve(input);
    const double interactiveManualQuadLimit = std::clamp(
        static_cast<double>(densityEstimate.faceCount) *
            kMaximumInteractiveManualRefinementRatio,
        kMinimumInteractiveManualQuadLimit,
        kMaximumInteractiveManualQuadLimit);
    std::ostringstream densityDebug;
    densityDebug << "[DirectionalRetopo][DensityDebug]\n"
                 << "Mode: " << densityModeName(densityMode()) << "\n"
                 << "Manual target (GUI/Context): " << manualTargetEdgeLength() << "\n"
                 << "Edge scale: " << densityEdgeLengthScale() << "\n"
                 << "RemeshInput requested min/mean/max: "
                 << densityEstimate.requestedMinimum << " / "
                 << densityEstimate.requestedMean << " / "
                 << densityEstimate.requestedMaximum << "\n"
                 << "RemeshInput effective min/mean/max: "
                 << densityEstimate.effectiveMinimum << " / "
                 << densityEstimate.effectiveMean << " / "
                 << densityEstimate.effectiveMaximum << "\n"
                 << "Source edge mean: " << densityEstimate.meanSourceEdgeLength << "\n"
                 << "Solver scale mean: " << densityEstimate.meanSolverScale << "\n"
                 << "Estimated output quads: " << densityEstimate.estimatedQuadCount;
    MGlobal::displayInfo(MString(densityDebug.str().c_str()));

    if (densityMode() == DensityMode::Manual &&
        densityEstimate.estimatedQuadCount >
            interactiveManualQuadLimit &&
        !captureInputSaved) {
        std::ostringstream warning;
        warning << "Manual Target Edge Length " << manualTargetEdgeLength()
                << " requests approximately " << std::fixed << std::setprecision(0)
                << densityEstimate.estimatedQuadCount
                << " quads for this Paint Region (interactive safety limit "
                << interactiveManualQuadLimit
                << "). Solver invocation was skipped to keep Maya responsive. "
                   "Use a larger absolute Manual Target Edge Length or a smaller Paint Region.";
        displayTargetWarning(warning.str().c_str());
        requestQuadPreviewRefresh();
        return;
    }
    if (densityMode() == DensityMode::Manual &&
        densityEstimate.estimatedQuadCount >
            interactiveManualQuadLimit &&
        captureInputSaved) {
        std::ostringstream captureWarning;
        captureWarning
            << "[DirectionalRetopo][CaptureDebug]\n"
            << "Manual interactive safety limit bypassed for this one-shot capture.\n"
            << "Estimated output quads: " << std::fixed << std::setprecision(0)
            << densityEstimate.estimatedQuadCount << "\n"
            << "Interactive safety limit: " << interactiveManualQuadLimit << "\n"
            << "The Tool Settings value was not changed.";
        MGlobal::displayInfo(MString(captureWarning.str().c_str()));
    }

    const auto solverStart = std::chrono::steady_clock::now();
    ++remeshSolverInvocationCount_;
    const solver::RemeshResult solveResult = remeshSolver_.solve(input);
    const double solverMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solverStart).count();
    const auto previewConversionStart = std::chrono::steady_clock::now();
    if (captureInputSaved) {
        captureRecord.hasExpectedResult = true;
        captureRecord.expectedResult = solver::summarizeResult(solveResult);
        std::string captureDiagnostic;
        if (!solver::saveRemeshCapture(capturePath, captureRecord, captureDiagnostic)) {
            displayTargetWarning(captureDiagnostic.c_str());
        } else {
            std::ostringstream captureMessage;
            captureMessage << "[DirectionalRetopo] Added Maya solve expectation to capture\n"
                           << "Status: " << (solveResult.success() ? "Success" : "Failure")
                           << "\nFailure code: "
                           << solver::failureCodeName(solveResult.failureCode);
            MGlobal::displayInfo(MString(captureMessage.str().c_str()));
        }
    }
    for (const std::string& warning : solveResult.warnings) {
        displayTargetWarning(warning.c_str());
    }
    quadPatchResults_.reserve(solveResult.components.size());
    quadDebugPatchResults_.reserve(solveResult.debugComponents.size());
    for (const solver::ComponentResult& component : solveResult.components) {
        std::ostringstream message;
        message << "[DirectionalRetopo] Remesh solver facade\n"
                << "Component: " << component.componentId << '\n'
                << "Status: "
                << (component.status == solver::SolveStatus::Success
                    ? "success" : "failure") << '\n'
                << "Failure code: "
                << solver::failureCodeName(component.failureCode) << '\n'
                << "Retry attempts/reason: " << component.retryCount
                << " / " << component.retryReason << '\n'
                << "Quads/Triangles/N-gons: "
                << component.quality.quadCount << " / "
                << component.quality.triangleCount << " / "
                << component.quality.nGonCount << '\n'
                << "Fixed Boundary maximum displacement: "
                << component.quality.maximumBoundaryDisplacement << '\n'
                << "Boundary crossings: "
                << component.quality.boundaryCrossingCount << '\n'
                << "Surface mean/p95/max distance: "
                << component.quality.meanSurfaceDistance << " / "
                << component.quality.p95SurfaceDistance << " / "
                << component.quality.maximumSurfaceDistance << '\n'
                << "Patch/Parameterization/Extraction: "
                << component.timings.patchBuildMilliseconds << " / "
                << component.timings.parameterizationMilliseconds << " / "
                << component.timings.extractionMilliseconds << " ms\n"
                << "Conformation/Transition/Validation: "
                << component.timings.conformationMilliseconds << " / "
                << component.timings.transitionMilliseconds << " / "
                << component.timings.validationMilliseconds << " ms\n"
                << "Total time: " << component.timings.totalMilliseconds
                << " ms\nDiagnostic: " << component.diagnosticMessage;
        MGlobal::displayInfo(MString(message.str().c_str()));
        if (component.status == solver::SolveStatus::Success) {
            quadPatchResults_.push_back(LegacyPreviewAdapter::convert(component, input));
        } else {
            std::ostringstream warning;
            warning << "Remesh component " << component.componentId
                    << " failed safely at " << component.failedStage
                    << ": " << component.diagnosticMessage;
            displayTargetWarning(warning.str().c_str());
        }
    }
    for (const solver::ComponentResult& component : solveResult.debugComponents) {
        quadDebugPatchResults_.push_back(LegacyPreviewAdapter::convert(component, input));
    }

    std::vector<QuadPatchResult> previewResults = quadPatchResults_;
    previewResults.insert(
        previewResults.end(),
        quadDebugPatchResults_.begin(),
        quadDebugPatchResults_.end());
    if (previewResults.empty()) {
        if (quadPreviewModel_) {
            quadPreviewModel_->clear();
        }
        displayTargetWarning(
            "Remesh solver produced no valid Preview; the Target Mesh was not modified.");
    } else if (quadPreviewModel_) {
        quadPreviewModel_->setResults(previewResults, visualizationSettings_.quadPreview);
        if (quadPatchResults_.empty()) {
            displayTargetWarning(
                "Boundary-Locked Preview failed; Raw Inner Result remains available "
                "for debug display and the Target Mesh was not modified.");
        }
    }
    requestQuadPreviewRefresh();
    const double previewConversionMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - previewConversionStart).count();
    solver::TimingMetrics aggregateTimings;
    for (const solver::ComponentResult& component : solveResult.components) {
        aggregateTimings.patchBuildMilliseconds +=
            component.timings.patchBuildMilliseconds;
        aggregateTimings.parameterizationMilliseconds +=
            component.timings.parameterizationMilliseconds;
        aggregateTimings.extractionMilliseconds +=
            component.timings.extractionMilliseconds;
        aggregateTimings.conformationMilliseconds +=
            component.timings.conformationMilliseconds;
        aggregateTimings.transitionMilliseconds +=
            component.timings.transitionMilliseconds;
        aggregateTimings.validationMilliseconds +=
            component.timings.validationMilliseconds;
    }
    std::ostringstream timingMessage;
    timingMessage << "[DirectionalRetopo][Timing]\n"
                  << "Input Adapter: " << std::fixed << std::setprecision(2)
                  << adapterMilliseconds << " ms\n"
                  << "Solver total: " << solverMilliseconds << " ms\n"
                  << "Patch build: " << aggregateTimings.patchBuildMilliseconds << " ms\n"
                  << "Parameterization: "
                  << aggregateTimings.parameterizationMilliseconds << " ms\n"
                  << "Quad extraction: "
                  << aggregateTimings.extractionMilliseconds << " ms\n"
                  << "Inner/final conformation: "
                  << aggregateTimings.conformationMilliseconds << " ms\n"
                  << "Boundary processing: "
                  << aggregateTimings.transitionMilliseconds << " ms\n"
                  << "Final validation: "
                  << aggregateTimings.validationMilliseconds << " ms\n"
                  << "Preview conversion: " << previewConversionMilliseconds << " ms\n"
                  << "Solver invocation count: " << remeshSolverInvocationCount_;
    MGlobal::displayInfo(MString(timingMessage.str().c_str()));

#if 0
    // R1-R3 baseline reference only. The executable path above is owned by
    // DirectionalRemeshSolver. Remove this reference block when R4 begins.
    clearQuadPreview();
    if (finalPaintRegion_.components.empty() ||
        directionFieldData_.empty() || densityFieldData_.empty()) {
        return;
    }

    const auto patchStart = std::chrono::steady_clock::now();
    PatchBuildResult patchBuild = patchTriangulator_.build(
        finalPaintRegion_,
        meshTopologyCache_);
    const unsigned int topologyBlendWidth = static_cast<unsigned int>(
        std::max(
            paintRegionSolver_.settings().transitionRings,
            PaintRegionSolverSettings::kMinimumTransitionRings));
    PatchBuildResult innerCoreBuild = patchTriangulator_.buildInnerCores(
        finalPaintRegion_,
        meshTopologyCache_,
        0U);
    PatchBuildResult adaptiveInnerCoreBuild;
    if (topologyBlendWidth > 1U) {
        adaptiveInnerCoreBuild = patchTriangulator_.buildInnerCores(
            finalPaintRegion_,
            meshTopologyCache_,
            topologyBlendWidth - 1U);
    }
    const auto patchEnd = std::chrono::steady_clock::now();
    const double patchBuildMilliseconds =
        std::chrono::duration<double, std::milli>(patchEnd - patchStart).count();
    for (const PatchBuildFailure& failure : patchBuild.failures) {
        std::ostringstream message;
        message << "Quad solve component " << failure.componentId
                << " skipped: " << failure.message;
        displayTargetWarning(message.str().c_str());
    }
    for (const PatchBuildFailure& failure : innerCoreBuild.failures) {
        std::ostringstream message;
        message << "Inner Core component " << failure.componentId
                << " skipped: " << failure.message;
        displayTargetWarning(message.str().c_str());
    }

    triangulatedPatches_ = std::move(patchBuild.patches);
    if (triangulatedPatches_.empty()) {
        displayTargetWarning(
            "No valid triangulated component is available for the Quad solver.");
        requestQuadPreviewRefresh();
        return;
    }

    const double patchTimeShare = patchBuildMilliseconds /
        static_cast<double>(triangulatedPatches_.size());
    quadSolveReports_.reserve(triangulatedPatches_.size());
    quadPatchResults_.reserve(triangulatedPatches_.size());
    quadDebugPatchResults_.reserve(triangulatedPatches_.size());
    BoundaryLockedPatchBuilderSettings lockedSettings =
        boundaryLockedPatchBuilder_.settings();
    lockedSettings.topologyBlendWidth = topologyBlendWidth;
    boundaryLockedPatchBuilder_.setSettings(lockedSettings);
    for (const TriangulatedPatch& patch : triangulatedPatches_) {
        const auto findComponentPatch = [&patch](
            const PatchBuildResult& build) -> const TriangulatedPatch* {
            const auto found = std::find_if(
                build.patches.begin(),
                build.patches.end(),
                [&patch](const TriangulatedPatch& candidate) {
                    return candidate.componentId == patch.componentId;
                });
            return found == build.patches.end() ? nullptr : &*found;
        };
        const TriangulatedPatch* standardInner =
            findComponentPatch(innerCoreBuild);
        const TriangulatedPatch* adaptiveInner =
            findComponentPatch(adaptiveInnerCoreBuild);
        if (standardInner == nullptr && adaptiveInner == nullptr) {
            std::ostringstream warning;
            warning << "No valid Inner Remesh Core for component "
                    << patch.componentId
                    << " after normal and adaptive domain construction.";
            displayTargetWarning(warning.str().c_str());
            continue;
        }

        BoundaryCompatibilityDensityResult standardCompatibility;
        if (standardInner != nullptr) {
            standardCompatibility = BoundaryCompatibilityDensity::build(
                patch,
                *standardInner,
                meshTopologyCache_,
                densityFieldData_,
                topologyBlendWidth);
        }
        BoundaryCompatibilityDensityResult adaptiveCompatibility;
        if (adaptiveInner != nullptr) {
            adaptiveCompatibility = BoundaryCompatibilityDensity::build(
                patch,
                *adaptiveInner,
                meshTopologyCache_,
                densityFieldData_,
                topologyBlendWidth);
        }

        struct SolveAttempt final
        {
            const TriangulatedPatch* patch = nullptr;
            const DensityFieldData* density = nullptr;
            const BoundaryCompatibilityDensityResult* compatibility = nullptr;
            const char* reason = nullptr;
        };
        std::vector<SolveAttempt> attempts;
        if (standardInner != nullptr) {
            attempts.push_back({
                standardInner,
                &densityFieldData_,
                nullptr,
                "Requested Core Density"});
            if (standardCompatibility.success) {
                attempts.push_back({
                    standardInner,
                    &standardCompatibility.densityField,
                    &standardCompatibility,
                    "Boundary Compatibility Density"});
            }
        }
        const bool adaptiveIsDistinct =
            adaptiveInner != nullptr &&
            (standardInner == nullptr ||
             adaptiveInner->triangles.size() >
                 standardInner->triangles.size());
        if (adaptiveIsDistinct && adaptiveCompatibility.success) {
            attempts.push_back({
                adaptiveInner,
                &adaptiveCompatibility.densityField,
                &adaptiveCompatibility,
                "Adaptive Inner Solve Region + Compatibility Density"});
        }
        if (attempts.empty()) {
            displayTargetWarning(
                "No finite Boundary-Locked retry attempt could be constructed.");
            continue;
        }
        if (attempts.size() > 3U) {
            attempts.resize(3U);
        }

        QuadComponentSolveReport report;
        QuadPatchResult lastInnerDebugResult;
        bool finalValid = false;
        std::ostringstream retryHistory;
        for (std::size_t attemptIndex = 0U;
             attemptIndex < attempts.size();
             ++attemptIndex) {
            const SolveAttempt& attempt = attempts[attemptIndex];
            report = autoRemesherAdapter_.solve(
                *attempt.patch,
                directionFieldData_,
                *attempt.density);
            report.retryAttemptCount = attemptIndex + 1U;
            report.retryReason = attempt.reason;
            std::unordered_set<int> innerSourceFaces;
            for (const PatchTriangle& triangle : attempt.patch->triangles) {
                if (triangle.sourceFaceId >= 0) {
                    innerSourceFaces.insert(triangle.sourceFaceId);
                }
            }
            report.innerSolveFaceCount = innerSourceFaces.size();
            report.requestedCoreTargetEdgeLength =
                attempt.compatibility != nullptr
                ? attempt.compatibility->requestedCoreTargetEdgeLength
                : report.meanEffectiveTargetEdgeLength;
            report.effectiveInterfaceTargetEdgeLength =
                attempt.compatibility != nullptr
                ? attempt.compatibility->effectiveInterfaceTargetEdgeLength
                : report.meanEffectiveTargetEdgeLength;
            report.timings.patchBuildMilliseconds = patchTimeShare;
            report.timings.totalMilliseconds += patchTimeShare;

            if (!report.result.success) {
                retryHistory << "Attempt " << (attemptIndex + 1U)
                             << " (" << attempt.reason << "): "
                             << report.diagnosticMessage << ' ';
                continue;
            }

            const auto collarStart = std::chrono::steady_clock::now();
            const std::string innerDiagnostic = report.diagnosticMessage;
            lastInnerDebugResult = report.result;
            QuadPatchResult boundaryLockedResult;
            std::string boundaryLockedDiagnostic;
            const bool collarBuilt = boundaryLockedPatchBuilder_.build(
                patch,
                report.result,
                directionFieldData_,
                *attempt.density,
                boundaryLockedResult,
                boundaryLockedDiagnostic);
            report.boundaryLockedCollarAttempted = true;
            report.boundaryLockedCollarSuccess = collarBuilt;
            bool finalConformed = false;
            bool finalValidated = false;
            if (collarBuilt) {
                boundaryLockedResult.boundaryLockedDiagnostic
                    .requestedCoreTargetEdgeLength =
                    report.requestedCoreTargetEdgeLength;
                boundaryLockedResult.boundaryLockedDiagnostic
                    .effectiveInterfaceTargetEdgeLength =
                    report.effectiveInterfaceTargetEdgeLength;
                boundaryLockedResult.boundaryLockedDiagnostic
                    .innerSolveFaceCount =
                    report.innerSolveFaceCount;
                report.finalConformationAttempted = true;
                finalConformed = autoRemesherAdapter_.conformToSurface(
                    patch,
                    boundaryLockedResult,
                    boundaryLockedDiagnostic);
                report.finalConformationSuccess = finalConformed;
                if (finalConformed) {
                    report.finalValidationAttempted = true;
                    finalValidated = autoRemesherAdapter_.validateResult(
                        patch,
                        boundaryLockedResult,
                        boundaryLockedDiagnostic);
                    report.finalValidationSuccess = finalValidated;
                }
            }
            finalValid = collarBuilt && finalConformed && finalValidated;
            const auto collarEnd = std::chrono::steady_clock::now();
            report.timings.collarBuildMilliseconds =
                std::chrono::duration<double, std::milli>(
                    collarEnd - collarStart).count();
            report.timings.totalMilliseconds +=
                report.timings.collarBuildMilliseconds;
            if (finalValid) {
                report.result = std::move(boundaryLockedResult);
                report.diagnosticMessage =
                    innerDiagnostic + " | " + boundaryLockedDiagnostic;
                break;
            }

            retryHistory << "Attempt " << (attemptIndex + 1U)
                         << " (" << attempt.reason << "): "
                         << boundaryLockedDiagnostic << ' ';
        }

        if (!finalValid) {
            report.result.clear();
            report.result.componentId = patch.componentId;
            if (!lastInnerDebugResult.rawVertices.empty()) {
                lastInnerDebugResult.success = false;
                lastInnerDebugResult.debugPreviewAvailable = true;
                lastInnerDebugResult.debugInnerResultOnly = true;
                quadDebugPatchResults_.push_back(
                    std::move(lastInnerDebugResult));
            }
            report.diagnosticMessage =
                "Boundary-Locked retry matrix exhausted: " +
                retryHistory.str();
        }

        std::ostringstream message;
        message << "[DirectionalRetopo] Quad solve\n"
                << "Component: " << report.componentId << '\n'
                << "Patch vertices: " << report.patchVertexCount << '\n'
                << "Patch triangles: " << report.patchTriangleCount << '\n'
                << "Patch diagnostic: " << patch.diagnosticMessage << '\n'
                << "Parameterization: "
                << (report.parameterizationSuccess ? "success" : "failure") << '\n'
                << "Quad extraction: "
                << (report.extractionSuccess ? "success" : "failure") << '\n'
                << "Inner surface conformation: "
                << (report.conformationSuccess ? "success" : "failure") << '\n'
                << "Boundary-Locked collar build: "
                << (!report.boundaryLockedCollarAttempted ? "not run" :
                    (report.boundaryLockedCollarSuccess ? "success" : "failure")) << '\n'
                << "Final surface conformation: "
                << (!report.finalConformationAttempted ? "not run" :
                    (report.finalConformationSuccess ? "success" : "failure")) << '\n'
                << "Final validation: "
                << (!report.finalValidationAttempted ? "not run" :
                    (report.finalValidationSuccess ? "success" : "failure")) << '\n'
                << "Effective target min/mean/max: "
                << report.minimumEffectiveTargetEdgeLength << " / "
                << report.meanEffectiveTargetEdgeLength << " / "
                << report.maximumEffectiveTargetEdgeLength << '\n'
                << "Curvature-limited triangles: "
                << report.curvatureLimitedTriangleCount << '\n'
                << "Guidance max deviation: " << std::fixed << std::setprecision(2)
                << report.maximumGuidanceDeviationDegrees << " deg\n"
                << "Retry attempts/reason: " << report.retryAttemptCount
                << " / " << report.retryReason << '\n'
                << "Requested Core target: "
                << report.requestedCoreTargetEdgeLength << '\n'
                << "Effective Interface target: "
                << report.effectiveInterfaceTargetEdgeLength << '\n'
                << "Inner solve triangles: "
                << report.innerSolveFaceCount << '\n';
        if (report.result.success) {
            const BoundaryComparisonDiagnostic& boundary =
                report.result.boundaryDiagnostic;
            const SurfaceFidelityMetrics& fidelity = report.result.fidelity;
            const BoundaryLockedPatchDiagnostic& locked =
                report.result.boundaryLockedDiagnostic;
            message << "Result vertices: "
                    << report.result.conformedVertices.size() << '\n'
                    << "Result polygons: " << report.result.polygons.size() << '\n'
                    << "Quads: " << report.result.quadCount << '\n'
                    << "Non-quads: " << report.result.nonQuadCount << '\n'
                    << "Triangles: " << report.result.triangleCount << '\n'
                    << "N-gons: " << report.result.nGonCount << '\n'
                    << "Quad percentage: "
                    << (100.0 * static_cast<double>(report.result.quadCount) /
                        static_cast<double>(report.result.polygons.size())) << "\n"
                    << "Fixed Boundary vertices/edges: "
                    << locked.fixedBoundaryVertexCount << " / "
                    << locked.fixedBoundaryEdgeCount << '\n'
                    << "Fixed Boundary maximum displacement: "
                    << locked.maximumSourceBoundaryDisplacement << '\n'
                    << "Inner Remesh Boundary vertices/edges: "
                    << locked.innerBoundaryVertexCount << " / "
                    << locked.innerBoundaryEdgeCount << '\n'
                    << "Transition Collar width/quads/triangles: "
                    << locked.topologyBlendWidth << " / "
                    << locked.collarQuadCount << " / "
                    << locked.collarTriangleCount << '\n'
                    << "Core quads/triangles: "
                    << locked.coreQuadCount << " / "
                    << locked.coreTriangleCount << '\n'
                    << "Boundary-Locked crossings: "
                    << locked.boundaryCrossingCount << '\n'
                    << "Outer Boundary validation vertices/topology/3D intersections: "
                    << locked.fixedBoundaryVertexCount << " / "
                    << (locked.outerBoundaryTopologySimple ? "simple" : "invalid")
                    << " / " << locked.outerBoundaryTrueIntersectionCount << '\n'
                    << "Inner Boundary validation vertices/topology/3D intersections: "
                    << locked.innerBoundaryVertexCount << " / "
                    << (locked.innerBoundaryTopologySimple ? "simple" : "invalid")
                    << " / " << locked.innerBoundaryTrueIntersectionCount << '\n'
                    << "Transition Collar seam/reversed/alignment cost: "
                    << locked.selectedSeamOffset << " / "
                    << (locked.innerOrderReversed ? "yes" : "no") << " / "
                    << locked.boundaryAlignmentCost << '\n'
                    << "Inner loop raw/primary/secondary/tiny/repaired: "
                    << locked.rawInnerBoundaryLoopCount << " / "
                    << locked.primaryInnerLoopCount << " / "
                    << locked.secondaryHoleLoopCount << " / "
                    << locked.tinyArtifactLoopCount << " / "
                    << locked.holeRepairCount << '\n'
                    << "Collar candidates tested/DP feasible/final valid: "
                    << locked.seamCandidatesTested << " / "
                    << locked.dpFeasibleCandidateCount << " / "
                    << locked.geometryValidCandidateCount << '\n'
                    << "Rejected zero-area/sliver DP moves: "
                    << locked.rejectedZeroAreaCandidateCount << " / "
                    << locked.rejectedSliverCandidateCount << '\n'
                    << "Inner solve source triangles: "
                    << locked.innerSolveFaceCount << '\n'
                    << "Source boundary vertices: "
                    << boundary.sourceVertexCount << '\n'
                    << "Result boundary vertices: "
                    << boundary.resultVertexCount << '\n'
                    << "Source/Result boundary edges: "
                    << boundary.sourceEdgeCount << " -> "
                    << boundary.resultEdgeCount << '\n'
                    << "Boundary vertex count difference: "
                    << boundary.vertexCountDifference << '\n'
                    << "Boundary loops: " << boundary.sourceLoopCount << " -> "
                    << boundary.resultLoopCount << '\n'
                    << "Boundary length: " << boundary.sourceTotalLength << " -> "
                    << boundary.resultTotalLength << '\n'
                    << "Boundary conformed mean/max distance: "
                    << boundary.meanNearestDistance << " / "
                    << boundary.maximumNearestDistance << '\n'
                    << "Boundary winding aligned/reversed: "
                    << boundary.alignedLoopCount << " / "
                    << boundary.reversedLoopCount << '\n'
                    << "Boundary closed state matches: "
                    << (boundary.closedStateMatches ? "yes" : "no") << '\n'
                    << "Boundary correspondence complete: "
                    << (boundary.correspondenceComplete ? "yes" : "no") << '\n'
                    << "Boundary ordered mapping valid: "
                    << (boundary.orderedMappingValid ? "yes" : "no") << '\n'
                    << "Boundary monotonic violations: "
                    << boundary.monotonicViolationCount << '\n'
                    << "Boundary crossings: "
                    << boundary.crossingCount << '\n'
                    << "Required boundary anchors/splits: "
                    << boundary.requiredBoundaryAnchorCount << " / "
                    << boundary.requiredResultSplitCount << '\n'
                    << "sourceArea: " << fidelity.sourceArea << '\n'
                    << "rawQuadArea: " << fidelity.rawQuadArea << '\n'
                    << "rawAreaRatio: " << fidelity.rawAreaRatio << '\n'
                    << "conformedArea: " << fidelity.conformedArea << '\n'
                    << "conformedAreaRatio: " << fidelity.conformedAreaRatio << '\n'
                    << "meanRawSurfaceDistance: "
                    << fidelity.meanRawSurfaceDistance << '\n'
                    << "maxRawSurfaceDistance: "
                    << fidelity.maximumRawSurfaceDistance << '\n'
                    << "meanProjectionDistance: "
                    << fidelity.meanProjectionDistance << '\n'
                    << "maxProjectionDistance: "
                    << fidelity.maximumProjectionDistance << '\n'
                    << "meanConformedSurfaceDistance: "
                    << fidelity.meanConformedSurfaceDistance << '\n'
                    << "maxConformedSurfaceDistance: "
                    << fidelity.maximumConformedSurfaceDistance << '\n'
                    << "sourceAverageEdgeLength: "
                    << fidelity.sourceAverageEdgeLength << '\n'
                    << "sourceMedianEdgeLength: "
                    << fidelity.sourceMedianEdgeLength << '\n';
            const auto appendBounds = [&message](
                const char* label,
                const FidelityBounds& bounds) {
                if (!bounds.valid) {
                    return;
                }
                message << label << " bbox: ("
                        << bounds.minimum.x << ", "
                        << bounds.minimum.y << ", "
                        << bounds.minimum.z << ") - ("
                        << bounds.maximum.x << ", "
                        << bounds.maximum.y << ", "
                        << bounds.maximum.z << ")\n";
            };
            appendBounds("Source", fidelity.sourceBounds);
            appendBounds("Raw", fidelity.rawBounds);
            appendBounds("Conformed", fidelity.conformedBounds);
            for (const BoundaryLoopCorrespondence& correspondence :
                 report.result.boundaryCorrespondences) {
                message << "Boundary correspondence "
                        << correspondence.sourceLoopIndex << " -> "
                        << correspondence.resultLoopIndex << ": source "
                        << correspondence.sourceVertexCount << "v/"
                        << correspondence.sourceEdgeCount << "e, result "
                        << correspondence.resultVertexCount << "v/"
                        << correspondence.resultEdgeCount << "e, arc "
                        << correspondence.sourceTotalArcLength << " -> "
                        << correspondence.resultTotalArcLengthAfter
                        << ", count delta "
                        << correspondence.vertexCountDifference
                        << ", mean/max "
                        << correspondence.meanDistanceAfter << '/'
                        << correspondence.maximumDistanceAfter
                        << ", winding "
                        << boundaryWindingName(correspondence.winding)
                        << ", seam result/source "
                        << correspondence.resultSeamOffset << '/'
                        << correspondence.sourceSeamParameter
                        << ", monotonic/crossing "
                        << correspondence.monotonicViolationCount << '/'
                        << (correspondence.selfIntersectionCount +
                            correspondence.sourceCrossingCount)
                        << ", required anchors "
                        << correspondence.requiredBoundaryAnchors.size()
                        << ", "
                        << (correspondence.sourceClosed ? "closed" : "open")
                        << " -> "
                        << (correspondence.resultClosed ? "closed" : "open")
                        << '\n';
            }
        }
        message << "Patch build time: " << report.timings.patchBuildMilliseconds
                << " ms\n"
                << "Parameterization time: "
                << report.timings.parameterizationMilliseconds << " ms\n"
                << "Quad extraction time: "
                << report.timings.extractionMilliseconds << " ms\n"
                << "Surface conformation time: "
                << report.timings.conformationMilliseconds << " ms\n"
                << "Validation time: "
                << report.timings.validationMilliseconds << " ms\n"
                << "Transition Collar build time: "
                << report.timings.collarBuildMilliseconds << " ms\n"
                << "Total time: " << report.timings.totalMilliseconds << " ms\n"
                << "Diagnostic: " << report.diagnosticMessage;
        MGlobal::displayInfo(MString(message.str().c_str()));

        if (report.result.success) {
            if (report.result.fidelity.distanceQualityWarning) {
                displayTargetWarning(
                    "Conformed Quad Preview exceeded the surface-distance quality "
                    "warning threshold; inspect this component before Phase 5.");
            }
            quadPatchResults_.push_back(report.result);
        } else {
            std::ostringstream warning;
            warning << "Quad solve component " << report.componentId
                    << " failed safely: " << report.diagnosticMessage;
            displayTargetWarning(warning.str().c_str());
        }
        quadSolveReports_.push_back(std::move(report));
    }

    std::vector<QuadPatchResult> previewResults = quadPatchResults_;
    previewResults.insert(
        previewResults.end(),
        quadDebugPatchResults_.begin(),
        quadDebugPatchResults_.end());
    if (previewResults.empty()) {
        if (quadPreviewModel_) {
            quadPreviewModel_->clear();
        }
        displayTargetWarning(
            "Quad solve produced no valid Preview; the Target Mesh was not modified.");
    } else if (quadPreviewModel_) {
        quadPreviewModel_->setResults(
            previewResults,
            visualizationSettings_.quadPreview);
        if (quadPatchResults_.empty()) {
            displayTargetWarning(
                "Boundary-Locked Preview failed; Raw Inner Result remains available "
                "for debug display and the Target Mesh was not modified.");
        }
    }
    requestQuadPreviewRefresh();
#endif
}

void DirectionalRetopoContext::clearFinalFields() noexcept
{
    directionFieldData_.clear();
    densityFieldData_.clear();
    fieldVisualizer_.clear();
    clearQuadPreview();
}

void DirectionalRetopoContext::clearQuadPreview() noexcept
{
    triangulatedPatches_.clear();
    quadSolveReports_.clear();
    quadPatchResults_.clear();
    quadDebugPatchResults_.clear();
    if (quadPreviewModel_) {
        quadPreviewModel_->clear();
        quadPreviewDrawable_.markDirty();
    }
}

void DirectionalRetopoContext::resetRegionPreview()
{
    regionFaceIds_.clear();
    finalPaintRegion_.clear();
    clearFinalFields();
    regionVisualizer_.clearFaceIds();
}

void DirectionalRetopoContext::invalidateTransientForCamera()
{
    if (strokeActive_) {
        finalizeProcessedStroke();
    }
    strokeActive_ = false;
    radiusAdjustDragActive_ = false;
    feedbackState_.clearInteraction();
    if (brushCursorModel_) {
        brushCursorModel_->setCameraSuppressed(true);
    }
}

void DirectionalRetopoContext::invalidateHoverFeedback(bool refreshViewport)
{
    if (brushCursorModel_) {
        brushCursorModel_->invalidateForFreshRaycast();
    }
    if (refreshViewport) {
        requestBrushCursorRefresh();
    }
}

void DirectionalRetopoContext::alternateContextTimerCallback(
    float /*elapsedTime*/,
    float /*lastTime*/,
    void* clientData)
{
    static_cast<DirectionalRetopoContext*>(clientData)->pollAlternateContext();
}

void DirectionalRetopoContext::installAlternateContextMonitor()
{
    removeAlternateContextMonitor();

    MStatus status;
    alternateContextTimerCallbackId_ = MTimerMessage::addTimerCallback(
        kAlternateContextPollIntervalSeconds,
        &DirectionalRetopoContext::alternateContextTimerCallback,
        this,
        &status);
    if (!status || alternateContextTimerCallbackId_ == 0) {
        alternateContextTimerCallbackId_ = 0;
        displayTargetWarning(
            "Camera alternate-context monitoring could not be installed; "
            "mouse-event fallback remains active.");
    }
}

void DirectionalRetopoContext::removeAlternateContextMonitor() noexcept
{
    if (alternateContextTimerCallbackId_ == 0) {
        return;
    }
    (void)MMessage::removeCallback(alternateContextTimerCallbackId_);
    alternateContextTimerCallbackId_ = 0;
}

void DirectionalRetopoContext::pollAlternateContext()
{
    if (!toolActive_) {
        return;
    }

    updateAlternateContextState(inAlternateContext());
}

void DirectionalRetopoContext::updateAlternateContextState(
    bool alternateContextActive)
{
    if (alternateContextActive == alternateContextObserved_) {
        return;
    }

    alternateContextObserved_ = alternateContextActive;
    if (alternateContextActive) {
        invalidateTransientForCamera();
    } else {
        // A camera matrix change invalidates the pre-camera world-space hit.
        // Camera exit never restores it; a fresh pointer event must ray-cast.
        if (brushCursorModel_) {
            brushCursorModel_->setCameraSuppressed(false);
        }
    }
    requestFeedbackRefresh();
}

void DirectionalRetopoContext::requestFeedbackRefresh()
{
    brushCursorDrawable_.markDirty();
    MToolsInfo::setDirtyFlag(*this);

    // scheduleRefreshAllViews() alone can defer MPxDrawOverride evaluation
    // until Maya's next unrelated viewport event. Force the active view after
    // both the Context and the cursor node have been dirtied so this frame
    // consumes the current model snapshot rather than the preceding one.
    MStatus viewStatus;
    M3dView activeView = M3dView::active3dView(&viewStatus);
    if (viewStatus) {
        (void)activeView.refresh(false, true);
    }
    (void)M3dView::scheduleRefreshAllViews();
}

void DirectionalRetopoContext::requestBrushCursorRefresh()
{
    // Hover and radius changes affect only the dedicated VP2 node. Keeping
    // MPxContext feedback clean avoids needlessly rebuilding Target/Region/
    // Stroke drawables on every pointer event.
    brushCursorDrawable_.markDirty();

    MStatus viewStatus;
    M3dView activeView = M3dView::active3dView(&viewStatus);
    if (viewStatus) {
        (void)activeView.refresh(false, true);
    } else {
        (void)M3dView::scheduleRefreshAllViews();
    }
}

void DirectionalRetopoContext::requestQuadPreviewRefresh()
{
    quadPreviewDrawable_.markDirty();
    (void)M3dView::scheduleRefreshAllViews();
}

}  // namespace directional_retopo
