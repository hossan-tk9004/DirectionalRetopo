#include "Viewport/DirectionalRetopoBrushCursor.h"

#include <maya/MDagModifier.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MHWGeometry.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MMatrix.h>
#include <maya/MPlug.h>
#include <maya/MPointArray.h>
#include <maya/MViewport2Renderer.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace directional_retopo {
namespace {

constexpr double kNormalEpsilon = 1.0e-12;
constexpr int kMinimumCircleSegments = 8;
constexpr char kTransformName[] = "directionalRetopoBrushCursor";
constexpr char kShapeName[] = "directionalRetopoBrushCursorShape";

class BrushCursorDrawData final : public MUserData
{
public:
    bool drawable = false;
    MPointArray circlePoints;
    MColor color;
    float lineWidth = 1.0F;
    unsigned int depthPriority = MHWRender::MRenderItem::sActiveLineDepthPriority;
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

std::vector<MObject> cursorNodesForDeletion()
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
            DirectionalRetopoBrushCursorShape::kTypeId) {
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

const MTypeId DirectionalRetopoBrushCursorShape::kTypeId(0x0013B2C1);
const MString DirectionalRetopoBrushCursorShape::kTypeName(
    "directionalRetopoBrushCursorShape");
const MString DirectionalRetopoBrushCursorShape::kDrawDbClassification(
    "drawdb/geometry/directionalRetopoBrushCursor");
const MString DirectionalRetopoBrushCursorShape::kDrawRegistrantId(
    "DirectionalRetopoBrushCursorPlugin");

void* DirectionalRetopoBrushCursorShape::creator()
{
    return new DirectionalRetopoBrushCursorShape();
}

MStatus DirectionalRetopoBrushCursorShape::initialize()
{
    return MS::kSuccess;
}

bool DirectionalRetopoBrushCursorShape::isBounded() const
{
    return false;
}

MBoundingBox DirectionalRetopoBrushCursorShape::boundingBox() const
{
    return MBoundingBox();
}

void DirectionalRetopoBrushCursorShape::setModel(
    std::shared_ptr<BrushCursorModel> model)
{
    std::scoped_lock lock(modelMutex_);
    model_ = std::move(model);
}

bool DirectionalRetopoBrushCursorShape::snapshot(
    BrushCursorSnapshot& snapshot) const
{
    std::shared_ptr<BrushCursorModel> model;
    {
        std::scoped_lock lock(modelMutex_);
        model = model_;
    }
    if (!model) {
        return false;
    }
    snapshot = model->snapshot();
    return true;
}

MHWRender::MPxDrawOverride*
DirectionalRetopoBrushCursorDrawOverride::creator(const MObject& object)
{
    return new DirectionalRetopoBrushCursorDrawOverride(object);
}

DirectionalRetopoBrushCursorDrawOverride::
    DirectionalRetopoBrushCursorDrawOverride(const MObject& object)
    : MHWRender::MPxDrawOverride(object, nullptr, true)
{
}

MHWRender::DrawAPI
DirectionalRetopoBrushCursorDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kOpenGLCoreProfile |
        MHWRender::kDirectX11;
}

bool DirectionalRetopoBrushCursorDrawOverride::hasUIDrawables() const
{
    return true;
}

MMatrix DirectionalRetopoBrushCursorDrawOverride::transform(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    // Circle points are prepared in world space. The temporary DAG transform
    // must not alter them even if a script happens to modify the helper.
    return MMatrix::identity;
}

bool DirectionalRetopoBrushCursorDrawOverride::isBounded(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return false;
}

MBoundingBox DirectionalRetopoBrushCursorDrawOverride::boundingBox(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return MBoundingBox();
}

bool DirectionalRetopoBrushCursorDrawOverride::disableInternalBoundingBoxDraw() const
{
    return true;
}

bool DirectionalRetopoBrushCursorDrawOverride::excludedFromPostEffects() const
{
    return true;
}

bool DirectionalRetopoBrushCursorDrawOverride::wantUserSelection() const
{
    return false;
}

MUserData* DirectionalRetopoBrushCursorDrawOverride::prepareForDraw(
    const MDagPath& objectPath,
    const MDagPath& /*cameraPath*/,
    const MHWRender::MFrameContext& /*frameContext*/,
    MUserData* oldData)
{
    auto* drawData = dynamic_cast<BrushCursorDrawData*>(oldData);
    if (drawData == nullptr) {
        drawData = new BrushCursorDrawData();
    }
    drawData->drawable = false;
    drawData->circlePoints.clear();

    MStatus status;
    MFnDependencyNode dependencyNode(objectPath.node(&status), &status);
    auto* cursorShape = status
        ? dynamic_cast<DirectionalRetopoBrushCursorShape*>(dependencyNode.userNode())
        : nullptr;
    BrushCursorSnapshot snapshot;
    if (cursorShape == nullptr || !cursorShape->snapshot(snapshot) ||
        !snapshot.drawable()) {
        return drawData;
    }

    MVector normal = snapshot.surfaceNormal;
    if (normal.length() <= kNormalEpsilon) {
        return drawData;
    }
    normal.normalize();

    const MVector reference = std::abs(normal.y) < 0.95
        ? MVector(0.0, 1.0, 0.0)
        : MVector(1.0, 0.0, 0.0);
    MVector tangent = normal ^ reference;
    if (tangent.length() <= kNormalEpsilon) {
        return drawData;
    }
    tangent.normalize();
    MVector bitangent = normal ^ tangent;
    bitangent.normalize();

    const double offset = std::max(
        snapshot.radius * snapshot.style.surfaceOffsetRadiusRatio,
        snapshot.style.minimumSurfaceOffset);
    const MPoint center = snapshot.worldPosition + normal * offset;
    const int segmentCount = std::max(
        snapshot.style.circleSegments,
        kMinimumCircleSegments);
    constexpr double kTwoPi = 6.28318530717958647692;
    for (int segment = 0; segment <= segmentCount; ++segment) {
        const double angle = kTwoPi * static_cast<double>(segment) /
            static_cast<double>(segmentCount);
        const MVector radial = tangent * std::cos(angle) +
            bitangent * std::sin(angle);
        drawData->circlePoints.append(center + radial * snapshot.radius);
    }

    drawData->drawable = true;
    drawData->color = snapshot.style.brushColor;
    drawData->lineWidth = snapshot.style.brushLineWidth;
    drawData->depthPriority = snapshot.style.brushDepthPriority;
    return drawData;
}

void DirectionalRetopoBrushCursorDrawOverride::addUIDrawables(
    const MDagPath& /*objectPath*/,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& /*frameContext*/,
    const MUserData* data)
{
    const auto* drawData = dynamic_cast<const BrushCursorDrawData*>(data);
    if (drawData == nullptr || !drawData->drawable ||
        drawData->circlePoints.length() < 4) {
        return;
    }

    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.setColor(drawData->color);
    drawManager.setDepthPriority(drawData->depthPriority);
    drawManager.setLineWidth(drawData->lineWidth);
    // The new renderer's only Brush Circle submit call site.
    (void)drawManager.lineStrip(drawData->circlePoints, false);
    drawManager.endDrawable();
}

BrushCursorDrawable::~BrushCursorDrawable()
{
    destroy();
}

MStatus BrushCursorDrawable::create(
    const std::shared_ptr<BrushCursorModel>& model)
{
    destroy();
    destroyAll();
    if (!model) {
        return MS::kInvalidParameter;
    }

    MStatus status;
    MDagModifier modifier;
    MObject transformObject = modifier.createNode("transform", MObject::kNullObj, &status);
    if (!status) {
        return status;
    }
    MObject shapeObject = modifier.createNode(
        DirectionalRetopoBrushCursorShape::kTypeId,
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
    auto* cursorShape = status
        ? dynamic_cast<DirectionalRetopoBrushCursorShape*>(dependencyNode.userNode())
        : nullptr;
    if (cursorShape == nullptr) {
        destroy();
        return MS::kFailure;
    }
    cursorShape->setModel(model);
    markDirty();

    if (liveShapeCount() != 1U) {
        destroyAll();
        transformHandle_ = MObjectHandle();
        shapeHandle_ = MObjectHandle();
        return MS::kFailure;
    }
    return MS::kSuccess;
}

void BrushCursorDrawable::destroy() noexcept
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

void BrushCursorDrawable::markDirty() const noexcept
{
    if (shapeHandle_.isValid() && shapeHandle_.isAlive()) {
        MHWRender::MRenderer::setGeometryDrawDirty(shapeHandle_.object(), true);
    }
}

bool BrushCursorDrawable::exists() const noexcept
{
    return shapeHandle_.isValid() && shapeHandle_.isAlive();
}

void BrushCursorDrawable::destroyAll() noexcept
{
    const std::vector<MObject> nodes = cursorNodesForDeletion();
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

unsigned int BrushCursorDrawable::liveShapeCount() noexcept
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
        if (status && node.typeId() == DirectionalRetopoBrushCursorShape::kTypeId) {
            ++count;
        }
    }
    return count;
}

}  // namespace directional_retopo
