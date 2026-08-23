#include "Plugin/PluginNames.h"
#include "Tool/DirectionalRetopoContextCommand.h"
#include "Tool/PythonRuntimeBridge.h"
#include "Tool/DirectionalRetopoToolCommand.h"
#include "Viewport/DirectionalRetopoBrushCursor.h"
#include "Viewport/DirectionalRetopoQuadPreview.h"
#include "Viewport/DirectionalRetopoTargetDisplay.h"

#include <maya/MDrawRegistry.h>
#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MStatus.h>

MStatus initializePlugin(MObject pluginObject)
{
    using namespace directional_retopo;

    MStatus status;
    MFnPlugin plugin(
        pluginObject,
        kPluginVendor,
        kPluginVersion,
        kRequiredMayaVersion,
        &status);

    if (!status) {
        status.perror("DirectionalRetopo: MFnPlugin initialization failed");
        return status;
    }

    status = plugin.registerNode(
        DirectionalRetopoBrushCursorShape::kTypeName,
        DirectionalRetopoBrushCursorShape::kTypeId,
        &DirectionalRetopoBrushCursorShape::creator,
        &DirectionalRetopoBrushCursorShape::initialize,
        MPxNode::kLocatorNode,
        &DirectionalRetopoBrushCursorShape::kDrawDbClassification);
    if (!status) {
        status.perror("DirectionalRetopo: register Brush Cursor node failed");
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        DirectionalRetopoBrushCursorShape::kDrawDbClassification,
        DirectionalRetopoBrushCursorShape::kDrawRegistrantId,
        &DirectionalRetopoBrushCursorDrawOverride::creator);
    if (!status) {
        status.perror("DirectionalRetopo: register Brush Cursor draw override failed");
        (void)plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
        return status;
    }

    status = plugin.registerNode(
        DirectionalRetopoTargetDisplayShape::kTypeName,
        DirectionalRetopoTargetDisplayShape::kTypeId,
        &DirectionalRetopoTargetDisplayShape::creator,
        &DirectionalRetopoTargetDisplayShape::initialize,
        MPxNode::kLocatorNode,
        &DirectionalRetopoTargetDisplayShape::kDrawDbClassification);
    if (!status) {
        status.perror("DirectionalRetopo: register Target Display node failed");
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoBrushCursorShape::kDrawDbClassification,
            DirectionalRetopoBrushCursorShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        DirectionalRetopoTargetDisplayShape::kDrawDbClassification,
        DirectionalRetopoTargetDisplayShape::kDrawRegistrantId,
        &DirectionalRetopoTargetDisplayDrawOverride::creator);
    if (!status) {
        status.perror("DirectionalRetopo: register Target Display draw override failed");
        (void)plugin.deregisterNode(DirectionalRetopoTargetDisplayShape::kTypeId);
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoBrushCursorShape::kDrawDbClassification,
            DirectionalRetopoBrushCursorShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
        return status;
    }

    status = plugin.registerNode(
        DirectionalRetopoQuadPreviewShape::kTypeName,
        DirectionalRetopoQuadPreviewShape::kTypeId,
        &DirectionalRetopoQuadPreviewShape::creator,
        &DirectionalRetopoQuadPreviewShape::initialize,
        MPxNode::kLocatorNode,
        &DirectionalRetopoQuadPreviewShape::kDrawDbClassification);
    if (!status) {
        status.perror("DirectionalRetopo: register Quad Preview node failed");
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoTargetDisplayShape::kDrawDbClassification,
            DirectionalRetopoTargetDisplayShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoTargetDisplayShape::kTypeId);
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoBrushCursorShape::kDrawDbClassification,
            DirectionalRetopoBrushCursorShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
        return status;
    }

    status = MHWRender::MDrawRegistry::registerDrawOverrideCreator(
        DirectionalRetopoQuadPreviewShape::kDrawDbClassification,
        DirectionalRetopoQuadPreviewShape::kDrawRegistrantId,
        &DirectionalRetopoQuadPreviewDrawOverride::creator);
    if (!status) {
        status.perror("DirectionalRetopo: register Quad Preview draw override failed");
        (void)plugin.deregisterNode(DirectionalRetopoQuadPreviewShape::kTypeId);
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoTargetDisplayShape::kDrawDbClassification,
            DirectionalRetopoTargetDisplayShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoTargetDisplayShape::kTypeId);
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoBrushCursorShape::kDrawDbClassification,
            DirectionalRetopoBrushCursorShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
        return status;
    }

    status = plugin.registerContextCommand(
        kContextCommandName,
        &DirectionalRetopoContextCommand::creator,
        kToolCommandName,
        &DirectionalRetopoToolCommand::creator,
        &DirectionalRetopoToolCommand::newSyntax);
    if (!status) {
        status.perror("DirectionalRetopo: registerContextCommand failed");
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoQuadPreviewShape::kDrawDbClassification,
            DirectionalRetopoQuadPreviewShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoQuadPreviewShape::kTypeId);
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoTargetDisplayShape::kDrawDbClassification,
            DirectionalRetopoTargetDisplayShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoTargetDisplayShape::kTypeId);
        (void)MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
            DirectionalRetopoBrushCursorShape::kDrawDbClassification,
            DirectionalRetopoBrushCursorShape::kDrawRegistrantId);
        (void)plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
        return status;
    }

    const MStatus toolSettingsStatus = installToolSettingsScripts();
    if (!toolSettingsStatus) {
        MGlobal::displayWarning(
            "[DirectionalRetopo] Maya Tool Settings scripts could not be sourced. "
            "The plug-in remains loaded, but the standard Tool Property Sheet "
            "will be unavailable until the repository scripts path is restored.");
    }

    MGlobal::displayInfo("DirectionalRetopo 0.1.0 loaded (Maya 2024.2 target).");
    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject pluginObject)
{
    using namespace directional_retopo;

    MStatus status;
    MFnPlugin plugin(
        pluginObject,
        kPluginVendor,
        kPluginVersion,
        kRequiredMayaVersion,
        &status);

    if (!status) {
        status.perror("DirectionalRetopo: MFnPlugin shutdown failed");
        return status;
    }

    (void)unloadPythonRuntime();
    BrushCursorDrawable::destroyAll();
    TargetDisplayDrawable::destroyAll();
    QuadPreviewDrawable::destroyAll();

    status = plugin.deregisterContextCommand(kContextCommandName, kToolCommandName);
    if (!status) {
        status.perror("DirectionalRetopo: deregisterContextCommand failed");
        return status;
    }

    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        DirectionalRetopoQuadPreviewShape::kDrawDbClassification,
        DirectionalRetopoQuadPreviewShape::kDrawRegistrantId);
    if (!status) {
        status.perror("DirectionalRetopo: deregister Quad Preview draw override failed");
        return status;
    }

    status = plugin.deregisterNode(DirectionalRetopoQuadPreviewShape::kTypeId);
    if (!status) {
        status.perror("DirectionalRetopo: deregister Quad Preview node failed");
        return status;
    }

    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        DirectionalRetopoTargetDisplayShape::kDrawDbClassification,
        DirectionalRetopoTargetDisplayShape::kDrawRegistrantId);
    if (!status) {
        status.perror("DirectionalRetopo: deregister Target Display draw override failed");
        return status;
    }

    status = plugin.deregisterNode(DirectionalRetopoTargetDisplayShape::kTypeId);
    if (!status) {
        status.perror("DirectionalRetopo: deregister Target Display node failed");
        return status;
    }

    status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator(
        DirectionalRetopoBrushCursorShape::kDrawDbClassification,
        DirectionalRetopoBrushCursorShape::kDrawRegistrantId);
    if (!status) {
        status.perror("DirectionalRetopo: deregister Brush Cursor draw override failed");
        return status;
    }

    status = plugin.deregisterNode(DirectionalRetopoBrushCursorShape::kTypeId);
    if (!status) {
        status.perror("DirectionalRetopo: deregister Brush Cursor node failed");
        return status;
    }

    MGlobal::displayInfo("DirectionalRetopo 0.1.0 unloaded.");
    return MS::kSuccess;
}
