#include "Tool/DirectionalRetopoContextCommand.h"

#include "Brush/BrushSettings.h"
#include "Tool/DirectionalRetopoContext.h"

#include <maya/MArgParser.h>
#include <maya/MGlobal.h>
#include <maya/MSyntax.h>

#include <cmath>

namespace directional_retopo {
namespace {

constexpr char kRadiusFlag[] = "-r";
constexpr char kRadiusFlagLong[] = "-radius";
constexpr char kRadiusAdjustModeFlag[] = "-ram";
constexpr char kRadiusAdjustModeFlagLong[] = "-radiusAdjustMode";
constexpr char kDensityModeFlag[] = "-dm";
constexpr char kDensityModeFlagLong[] = "-densityMode";
constexpr char kManualTargetEdgeLengthFlag[] = "-mtl";
constexpr char kManualTargetEdgeLengthFlagLong[] = "-manualTargetEdgeLength";
constexpr char kEdgeLengthScaleFlag[] = "-els";
constexpr char kEdgeLengthScaleFlagLong[] = "-edgeLengthScale";
constexpr char kTopologyBlendWidthFlag[] = "-tbw";
constexpr char kTopologyBlendWidthFlagLong[] = "-topologyBlendWidth";
constexpr char kShowDirectionFieldFlag[] = "-sdf";
constexpr char kShowDirectionFieldFlagLong[] = "-showDirectionField";
constexpr char kShowDensityFieldFlag[] = "-sde";
constexpr char kShowDensityFieldFlagLong[] = "-showDensityField";
constexpr char kShowQuadPreviewFlag[] = "-sqp";
constexpr char kShowQuadPreviewFlagLong[] = "-showQuadPreview";
constexpr char kShowRawQuadPreviewFlag[] = "-srq";
constexpr char kShowRawQuadPreviewFlagLong[] = "-showRawQuadPreview";
constexpr char kShowConformedQuadPreviewFlag[] = "-scq";
constexpr char kShowConformedQuadPreviewFlagLong[] = "-showConformedQuadPreview";
constexpr char kShowSourceBoundaryFlag[] = "-ssb";
constexpr char kShowSourceBoundaryFlagLong[] = "-showSourceBoundary";
constexpr char kShowResultBoundaryFlag[] = "-srb";
constexpr char kShowResultBoundaryFlagLong[] = "-showResultBoundary";
constexpr char kShowBoundaryCorrespondenceFlag[] = "-sbc";
constexpr char kShowBoundaryCorrespondenceFlagLong[] =
    "-showBoundaryCorrespondence";
constexpr char kShowBoundaryAnchorsFlag[] = "-sba";
constexpr char kShowBoundaryAnchorsFlagLong[] = "-showBoundaryAnchors";
constexpr char kCaptureRemeshInputFlag[] = "-cri";
constexpr char kCaptureRemeshInputFlagLong[] = "-captureNextRemeshInput";
constexpr char kResetSettingsFlag[] = "-rst";
constexpr char kResetSettingsFlagLong[] = "-resetSettings";

}  // namespace

void* DirectionalRetopoContextCommand::creator()
{
    return new DirectionalRetopoContextCommand();
}

MPxContext* DirectionalRetopoContextCommand::makeObj()
{
    context_ = new DirectionalRetopoContext();
    return context_;
}

MStatus DirectionalRetopoContextCommand::appendSyntax()
{
    MStatus status;
    MSyntax contextSyntax = syntax(&status);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(kRadiusFlag, kRadiusFlagLong, MSyntax::kDouble);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kRadiusAdjustModeFlag,
        kRadiusAdjustModeFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kDensityModeFlag,
        kDensityModeFlagLong,
        MSyntax::kString);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kManualTargetEdgeLengthFlag,
        kManualTargetEdgeLengthFlagLong,
        MSyntax::kDouble);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kEdgeLengthScaleFlag,
        kEdgeLengthScaleFlagLong,
        MSyntax::kDouble);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kTopologyBlendWidthFlag,
        kTopologyBlendWidthFlagLong,
        MSyntax::kLong);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowDirectionFieldFlag,
        kShowDirectionFieldFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowDensityFieldFlag,
        kShowDensityFieldFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowQuadPreviewFlag,
        kShowQuadPreviewFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowRawQuadPreviewFlag,
        kShowRawQuadPreviewFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowConformedQuadPreviewFlag,
        kShowConformedQuadPreviewFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowSourceBoundaryFlag,
        kShowSourceBoundaryFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowResultBoundaryFlag,
        kShowResultBoundaryFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowBoundaryCorrespondenceFlag,
        kShowBoundaryCorrespondenceFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kShowBoundaryAnchorsFlag,
        kShowBoundaryAnchorsFlagLong,
        MSyntax::kBoolean);
    if (!status) {
        return status;
    }
    status = contextSyntax.addFlag(
        kCaptureRemeshInputFlag,
        kCaptureRemeshInputFlagLong,
        MSyntax::kString);
    if (!status) {
        return status;
    }
    return contextSyntax.addFlag(
        kResetSettingsFlag,
        kResetSettingsFlagLong,
        MSyntax::kBoolean);
}

MStatus DirectionalRetopoContextCommand::doEditFlags()
{
    if (context_ == nullptr) {
        return MS::kFailure;
    }

    MStatus status;
    const MArgParser arguments = parser(&status);
    if (!status) {
        return status;
    }

    if (arguments.isFlagSet(kRadiusFlag)) {
        double radius = 0.0;
        status = arguments.getFlagArgument(kRadiusFlag, 0, radius);
        if (!status) {
            return status;
        }

        if (!std::isfinite(radius) || radius < BrushSettings::kMinimumRadius) {
            MGlobal::displayError(
                "[DirectionalRetopo] Radius must be a finite value greater than zero.");
            return MS::kInvalidParameter;
        }

        context_->setBrushRadius(radius);
    }

    if (arguments.isFlagSet(kRadiusAdjustModeFlag)) {
        bool enabled = false;
        status = arguments.getFlagArgument(kRadiusAdjustModeFlag, 0, enabled);
        if (!status) {
            return status;
        }
        context_->setRadiusAdjustMode(enabled);
    }

    if (arguments.isFlagSet(kDensityModeFlag)) {
        MString mode;
        status = arguments.getFlagArgument(kDensityModeFlag, 0, mode);
        if (!status) {
            return status;
        }
        mode.toLowerCase();
        if (mode == "manual") {
            context_->setDensityMode(DensityMode::Manual);
        } else if (mode == "auto") {
            context_->setDensityMode(DensityMode::Auto);
        } else {
            MGlobal::displayError(
                "[DirectionalRetopo] Density mode must be 'Manual' or 'Auto'.");
            return MS::kInvalidParameter;
        }
    }

    if (arguments.isFlagSet(kManualTargetEdgeLengthFlag)) {
        double edgeLength = 0.0;
        status = arguments.getFlagArgument(
            kManualTargetEdgeLengthFlag,
            0,
            edgeLength);
        if (!status) {
            return status;
        }
        if (!std::isfinite(edgeLength) || edgeLength <= 0.0) {
            MGlobal::displayError(
                "[DirectionalRetopo] Manual target edge length must be finite and positive.");
            return MS::kInvalidParameter;
        }
        context_->setManualTargetEdgeLength(edgeLength);
    }

    if (arguments.isFlagSet(kEdgeLengthScaleFlag)) {
        double scale = 0.0;
        status = arguments.getFlagArgument(kEdgeLengthScaleFlag, 0, scale);
        if (!status) {
            return status;
        }
        if (!std::isfinite(scale) || scale <= 0.0) {
            MGlobal::displayError(
                "[DirectionalRetopo] Edge length scale must be finite and positive.");
            return MS::kInvalidParameter;
        }
        context_->setDensityEdgeLengthScale(scale);
    }

    if (arguments.isFlagSet(kTopologyBlendWidthFlag)) {
        int rings = 0;
        status = arguments.getFlagArgument(kTopologyBlendWidthFlag, 0, rings);
        if (!status) {
            return status;
        }
        if (rings < 0 ||
            rings > PaintRegionSolverSettings::kMaximumTransitionRings) {
            MGlobal::displayError(
                "[DirectionalRetopo] Topology Blend Width must be between 1 and 10 rings.");
            return MS::kInvalidParameter;
        }
        context_->setTopologyBlendWidth(rings);
    }

    if (arguments.isFlagSet(kShowDirectionFieldFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(kShowDirectionFieldFlag, 0, show);
        if (!status) {
            return status;
        }
        context_->setShowDirectionField(show);
    }

    if (arguments.isFlagSet(kShowDensityFieldFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(kShowDensityFieldFlag, 0, show);
        if (!status) {
            return status;
        }
        context_->setShowDensityField(show);
    }

    if (arguments.isFlagSet(kShowQuadPreviewFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(kShowQuadPreviewFlag, 0, show);
        if (!status) {
            return status;
        }
        context_->setShowQuadPreview(show);
    }

    if (arguments.isFlagSet(kShowRawQuadPreviewFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(kShowRawQuadPreviewFlag, 0, show);
        if (!status) {
            return status;
        }
        context_->setShowRawQuadPreview(show);
    }

    if (arguments.isFlagSet(kShowConformedQuadPreviewFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(
            kShowConformedQuadPreviewFlag,
            0,
            show);
        if (!status) {
            return status;
        }
        context_->setShowConformedQuadPreview(show);
    }

    if (arguments.isFlagSet(kShowSourceBoundaryFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(kShowSourceBoundaryFlag, 0, show);
        if (!status) {
            return status;
        }
        context_->setShowSourceBoundary(show);
    }

    if (arguments.isFlagSet(kShowResultBoundaryFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(kShowResultBoundaryFlag, 0, show);
        if (!status) {
            return status;
        }
        context_->setShowResultBoundary(show);
    }

    if (arguments.isFlagSet(kShowBoundaryCorrespondenceFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(
            kShowBoundaryCorrespondenceFlag,
            0,
            show);
        if (!status) {
            return status;
        }
        context_->setShowBoundaryCorrespondence(show);
    }

    if (arguments.isFlagSet(kShowBoundaryAnchorsFlag)) {
        bool show = false;
        status = arguments.getFlagArgument(
            kShowBoundaryAnchorsFlag,
            0,
            show);
        if (!status) {
            return status;
        }
        context_->setShowRequiredBoundaryAnchors(show);
    }

    if (arguments.isFlagSet(kCaptureRemeshInputFlag)) {
        MString path;
        status = arguments.getFlagArgument(kCaptureRemeshInputFlag, 0, path);
        if (!status) {
            return status;
        }
        context_->setPendingRemeshCapturePath(path.asChar());
    }

    if (arguments.isFlagSet(kResetSettingsFlag)) {
        bool reset = false;
        status = arguments.getFlagArgument(kResetSettingsFlag, 0, reset);
        if (!status) {
            return status;
        }
        if (reset) {
            context_->resetToolSettings();
        }
    }

    return MS::kSuccess;
}

MStatus DirectionalRetopoContextCommand::doQueryFlags()
{
    if (context_ == nullptr) {
        return MS::kFailure;
    }

    MStatus status;
    const MArgParser arguments = parser(&status);
    if (!status) {
        return status;
    }

    if (arguments.isFlagSet(kRadiusFlag)) {
        return setResult(context_->brushRadius());
    }
    if (arguments.isFlagSet(kRadiusAdjustModeFlag)) {
        return setResult(context_->radiusAdjustMode());
    }
    if (arguments.isFlagSet(kDensityModeFlag)) {
        return setResult(MString(
            context_->densityMode() == DensityMode::Manual ? "Manual" : "Auto"));
    }
    if (arguments.isFlagSet(kManualTargetEdgeLengthFlag)) {
        return setResult(context_->manualTargetEdgeLength());
    }
    if (arguments.isFlagSet(kEdgeLengthScaleFlag)) {
        return setResult(context_->densityEdgeLengthScale());
    }
    if (arguments.isFlagSet(kTopologyBlendWidthFlag)) {
        return setResult(context_->topologyBlendWidth());
    }
    if (arguments.isFlagSet(kShowDirectionFieldFlag)) {
        return setResult(context_->showDirectionField());
    }
    if (arguments.isFlagSet(kShowDensityFieldFlag)) {
        return setResult(context_->showDensityField());
    }
    if (arguments.isFlagSet(kShowQuadPreviewFlag)) {
        return setResult(context_->showQuadPreview());
    }
    if (arguments.isFlagSet(kShowRawQuadPreviewFlag)) {
        return setResult(context_->showRawQuadPreview());
    }
    if (arguments.isFlagSet(kShowConformedQuadPreviewFlag)) {
        return setResult(context_->showConformedQuadPreview());
    }
    if (arguments.isFlagSet(kShowSourceBoundaryFlag)) {
        return setResult(context_->showSourceBoundary());
    }
    if (arguments.isFlagSet(kShowResultBoundaryFlag)) {
        return setResult(context_->showResultBoundary());
    }
    if (arguments.isFlagSet(kShowBoundaryCorrespondenceFlag)) {
        return setResult(context_->showBoundaryCorrespondence());
    }
    if (arguments.isFlagSet(kShowBoundaryAnchorsFlag)) {
        return setResult(context_->showRequiredBoundaryAnchors());
    }

    if (arguments.isFlagSet(kCaptureRemeshInputFlag)) {
        return setResult(MString(context_->pendingRemeshCapturePath().c_str()));
    }
    return MS::kSuccess;
}

}  // namespace directional_retopo
