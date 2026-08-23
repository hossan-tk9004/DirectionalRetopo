#include "Viewport/DirectionalRetopoTargetDisplay.h"

#include <maya/MDagModifier.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MHWGeometry.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <vector>

namespace directional_retopo {
namespace {

constexpr char kTransformName[] = "directionalRetopoTargetDisplay";
constexpr char kShapeName[] = "directionalRetopoTargetDisplayShape";

class TargetDisplayDrawData final : public MUserData
{
public:
    bool drawable = false;
    MPointArray worldLinePoints;
    MColor color;
    float lineWidth = 1.0F;
    unsigned int depthPriority = MHWRender::MRenderItem::sActiveWireDepthPriority;
};

void configureTemporaryNode(
    const MObject& object,
    const char* name,
    bool hideInOutliner)
{
    MStatus status;
    MFnDependencyNode node(object, &status);
    if (!status) {
        return;
    }
    (void)node.setName(name);
    (void)node.setDoNotWrite(true);

    MPlug historicallyInteresting = node.findPlug(
        "isHistoricallyInteresting",
        true,
        &status);
    if (status && !historicallyInteresting.isNull()) {
        (void)historicallyInteresting.setInt(0);
    }
    if (hideInOutliner) {
        MPlug hiddenInOutliner = node.findPlug("hiddenInOutliner", true, &status);
        if (status && !hiddenInOutliner.isNull()) {
            (void)hiddenInOutliner.setBool(true);
        }
    }
}

std::vector<MObject> targetDisplayNodesForDeletion()
{
    std::vector<MObject> nodes;
    MStatus status;
    MItDependencyNodes iterator(MFn::kPluginLocatorNode, &status);
    if (!status) {
        return nodes;
    }

    for (; !iterator.isDone(); iterator.next()) {
        const MObject shapeObject = iterator.thisNode(&status);
        if (!status || shapeObject.isNull()) {
            continue;
        }
        MFnDependencyNode dependencyNode(shapeObject, &status);
        if (!status || dependencyNode.typeId() !=
            DirectionalRetopoTargetDisplayShape::kTypeId) {
            continue;
        }

        MFnDagNode dagNode(shapeObject, &status);
        MObject objectToDelete = shapeObject;
        if (status && dagNode.parentCount() > 0) {
            const MObject parent = dagNode.parent(0, &status);
            if (status && !parent.isNull()) {
                objectToDelete = parent;
            }
        }
        const bool alreadyQueued = std::any_of(
            nodes.begin(),
            nodes.end(),
            [&objectToDelete](const MObject& object) {
                return object == objectToDelete;
            });
        if (!alreadyQueued) {
            nodes.push_back(objectToDelete);
        }
    }
    return nodes;
}

}  // namespace

const MTypeId DirectionalRetopoTargetDisplayShape::kTypeId(0x0013B2C2);
const MString DirectionalRetopoTargetDisplayShape::kTypeName(
    "directionalRetopoTargetDisplayShape");
const MString DirectionalRetopoTargetDisplayShape::kDrawDbClassification(
    "drawdb/geometry/directionalRetopoTargetDisplay");
const MString DirectionalRetopoTargetDisplayShape::kDrawRegistrantId(
    "DirectionalRetopoTargetDisplayPlugin");

void* DirectionalRetopoTargetDisplayShape::creator()
{
    return new DirectionalRetopoTargetDisplayShape();
}

MStatus DirectionalRetopoTargetDisplayShape::initialize()
{
    return MS::kSuccess;
}

bool DirectionalRetopoTargetDisplayShape::isBounded() const
{
    return false;
}

MBoundingBox DirectionalRetopoTargetDisplayShape::boundingBox() const
{
    return MBoundingBox();
}

void DirectionalRetopoTargetDisplayShape::setModel(
    std::shared_ptr<TargetDisplayModel> model)
{
    std::scoped_lock lock(modelMutex_);
    model_ = std::move(model);
}

bool DirectionalRetopoTargetDisplayShape::snapshot(
    TargetDisplaySnapshot& snapshot)
{
    std::shared_ptr<TargetDisplayModel> model;
    {
        std::scoped_lock lock(modelMutex_);
        model = model_;
    }
    return model && model->snapshot(snapshot);
}

MHWRender::MPxDrawOverride*
DirectionalRetopoTargetDisplayDrawOverride::creator(const MObject& object)
{
    return new DirectionalRetopoTargetDisplayDrawOverride(object);
}

DirectionalRetopoTargetDisplayDrawOverride::
    DirectionalRetopoTargetDisplayDrawOverride(const MObject& object)
    // Persistent Target Display is updated only when its model explicitly
    // marks this shape dirty. Brush Hover never enters that path.
    : MHWRender::MPxDrawOverride(object, nullptr, false)
{
}

MHWRender::DrawAPI
DirectionalRetopoTargetDisplayDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kOpenGLCoreProfile |
        MHWRender::kDirectX11;
}

bool DirectionalRetopoTargetDisplayDrawOverride::hasUIDrawables() const
{
    return true;
}

MMatrix DirectionalRetopoTargetDisplayDrawOverride::transform(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    // TargetDisplayModel supplies world-space line points.
    return MMatrix::identity;
}

bool DirectionalRetopoTargetDisplayDrawOverride::isBounded(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return false;
}

MBoundingBox DirectionalRetopoTargetDisplayDrawOverride::boundingBox(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return MBoundingBox();
}

bool DirectionalRetopoTargetDisplayDrawOverride::disableInternalBoundingBoxDraw() const
{
    return true;
}

bool DirectionalRetopoTargetDisplayDrawOverride::excludedFromPostEffects() const
{
    return true;
}

bool DirectionalRetopoTargetDisplayDrawOverride::wantUserSelection() const
{
    return false;
}

MUserData* DirectionalRetopoTargetDisplayDrawOverride::prepareForDraw(
    const MDagPath& objectPath,
    const MDagPath& /*cameraPath*/,
    const MHWRender::MFrameContext& /*frameContext*/,
    MUserData* oldData)
{
    auto* drawData = dynamic_cast<TargetDisplayDrawData*>(oldData);
    if (drawData == nullptr) {
        drawData = new TargetDisplayDrawData();
    }
    drawData->drawable = false;
    drawData->worldLinePoints.clear();

    MStatus status;
    MFnDependencyNode dependencyNode(objectPath.node(&status), &status);
    auto* targetShape = status
        ? dynamic_cast<DirectionalRetopoTargetDisplayShape*>(dependencyNode.userNode())
        : nullptr;
    TargetDisplaySnapshot snapshot;
    if (targetShape == nullptr || !targetShape->snapshot(snapshot) ||
        !snapshot.visible || snapshot.worldLinePoints.length() == 0) {
        return drawData;
    }

    drawData->drawable = true;
    drawData->worldLinePoints = snapshot.worldLinePoints;
    drawData->color = MColor(
        snapshot.style.targetWireColor.r,
        snapshot.style.targetWireColor.g,
        snapshot.style.targetWireColor.b,
        snapshot.style.targetWireOpacity);
    drawData->lineWidth = snapshot.style.targetWireLineWidth;
    drawData->depthPriority = snapshot.style.targetWireDepthPriority;
    return drawData;
}

void DirectionalRetopoTargetDisplayDrawOverride::addUIDrawables(
    const MDagPath& /*objectPath*/,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& /*frameContext*/,
    const MUserData* data)
{
    const auto* drawData = dynamic_cast<const TargetDisplayDrawData*>(data);
    if (drawData == nullptr || !drawData->drawable ||
        drawData->worldLinePoints.length() < 2) {
        return;
    }

    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.setColor(drawData->color);
    drawManager.setDepthPriority(drawData->depthPriority);
    drawManager.setLineWidth(drawData->lineWidth);
    // The Target Wireframe's only batched submit call site.
    (void)drawManager.lineList(drawData->worldLinePoints, false);
    drawManager.endDrawable();
}

TargetDisplayDrawable::~TargetDisplayDrawable()
{
    destroy();
}

MStatus TargetDisplayDrawable::create(
    const std::shared_ptr<TargetDisplayModel>& model)
{
    destroy();
    destroyAll();
    if (!model) {
        return MS::kInvalidParameter;
    }

    MStatus status;
    MDagModifier modifier;
    MObject transformObject = modifier.createNode(
        "transform",
        MObject::kNullObj,
        &status);
    if (!status) {
        return status;
    }
    MObject shapeObject = modifier.createNode(
        DirectionalRetopoTargetDisplayShape::kTypeId,
        transformObject,
        &status);
    if (!status) {
        return status;
    }
    status = modifier.doIt();
    if (!status) {
        return status;
    }

    transformHandle_ = MObjectHandle(transformObject);
    shapeHandle_ = MObjectHandle(shapeObject);
    configureTemporaryNode(transformObject, kTransformName, true);
    configureTemporaryNode(shapeObject, kShapeName, true);

    MFnDependencyNode dependencyNode(shapeObject, &status);
    auto* targetShape = status
        ? dynamic_cast<DirectionalRetopoTargetDisplayShape*>(dependencyNode.userNode())
        : nullptr;
    if (targetShape == nullptr) {
        destroy();
        return MS::kFailure;
    }
    targetShape->setModel(model);
    model->setDisplayShapeObject(shapeObject);
    markDirty();

    if (liveShapeCount() != 1U) {
        destroyAll();
        transformHandle_ = MObjectHandle();
        shapeHandle_ = MObjectHandle();
        return MS::kFailure;
    }
    return MS::kSuccess;
}

void TargetDisplayDrawable::destroy() noexcept
{
    MObject objectToDelete;
    if (transformHandle_.isValid() && transformHandle_.isAlive()) {
        objectToDelete = transformHandle_.object();
    } else if (shapeHandle_.isValid() && shapeHandle_.isAlive()) {
        objectToDelete = shapeHandle_.object();
    }
    transformHandle_ = MObjectHandle();
    shapeHandle_ = MObjectHandle();
    if (objectToDelete.isNull()) {
        return;
    }

    MDagModifier modifier;
    if (modifier.deleteNode(objectToDelete) == MS::kSuccess) {
        (void)modifier.doIt();
    }
}

void TargetDisplayDrawable::markDirty() const noexcept
{
    if (shapeHandle_.isValid() && shapeHandle_.isAlive()) {
        MHWRender::MRenderer::setGeometryDrawDirty(shapeHandle_.object(), true);
    }
}

bool TargetDisplayDrawable::exists() const noexcept
{
    return shapeHandle_.isValid() && shapeHandle_.isAlive();
}

void TargetDisplayDrawable::destroyAll() noexcept
{
    const std::vector<MObject> nodes = targetDisplayNodesForDeletion();
    if (nodes.empty()) {
        return;
    }
    MDagModifier modifier;
    bool hasDeletion = false;
    for (const MObject& object : nodes) {
        if (!object.isNull() && modifier.deleteNode(object) == MS::kSuccess) {
            hasDeletion = true;
        }
    }
    if (hasDeletion) {
        (void)modifier.doIt();
    }
}

unsigned int TargetDisplayDrawable::liveShapeCount() noexcept
{
    unsigned int count = 0;
    MStatus status;
    MItDependencyNodes iterator(MFn::kPluginLocatorNode, &status);
    if (!status) {
        return 0;
    }
    for (; !iterator.isDone(); iterator.next()) {
        const MObject object = iterator.thisNode(&status);
        if (!status || object.isNull()) {
            continue;
        }
        MFnDependencyNode node(object, &status);
        if (status && node.typeId() == DirectionalRetopoTargetDisplayShape::kTypeId) {
            ++count;
        }
    }
    return count;
}

}  // namespace directional_retopo
