#include "Viewport/DirectionalRetopoQuadPreview.h"

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

constexpr char kTransformName[] = "directionalRetopoQuadPreview";
constexpr char kShapeName[] = "directionalRetopoQuadPreviewShape";

class QuadPreviewDrawData final : public MUserData
{
public:
    bool drawable = false;
    MPointArray rawWorldLinePoints;
    MPointArray conformedWorldLinePoints;
    MPointArray transitionCollarWorldLinePoints;
    MPointArray triangleWorldLinePoints;
    MPointArray sourceBoundaryWorldLinePoints;
    MPointArray resultBoundaryWorldLinePoints;
    MPointArray boundaryCorrespondenceWorldLinePoints;
    MPointArray requiredBoundaryAnchorWorldPoints;
    MColor rawColor;
    MColor conformedColor;
    MColor transitionCollarColor;
    MColor triangleColor;
    MColor sourceBoundaryColor;
    MColor resultBoundaryColor;
    MColor boundaryCorrespondenceColor;
    MColor requiredBoundaryAnchorColor;
    float rawLineWidth = 1.0F;
    float conformedLineWidth = 1.0F;
    float transitionCollarLineWidth = 1.0F;
    float triangleLineWidth = 1.0F;
    float sourceBoundaryLineWidth = 1.0F;
    float resultBoundaryLineWidth = 1.0F;
    float boundaryCorrespondenceLineWidth = 1.0F;
    float requiredBoundaryAnchorPointSize = 1.0F;
    unsigned int depthPriority = MHWRender::MRenderItem::sActiveLineDepthPriority;
};

void configureTemporaryNode(const MObject& object, const char* name, bool hideInOutliner)
{
    MStatus status;
    MFnDependencyNode node(object, &status);
    if (!status) {
        return;
    }
    (void)node.setName(name);
    (void)node.setDoNotWrite(true);
    MPlug historicallyInteresting = node.findPlug("isHistoricallyInteresting", true, &status);
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

std::vector<MObject> previewNodesForDeletion()
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
            DirectionalRetopoQuadPreviewShape::kTypeId) {
            continue;
        }
        MFnDagNode dagNode(shapeObject, &status);
        MObject objectToDelete = shapeObject;
        if (status && dagNode.parentCount() > 0U) {
            const MObject parent = dagNode.parent(0, &status);
            if (status && !parent.isNull()) {
                objectToDelete = parent;
            }
        }
        if (std::find(nodes.begin(), nodes.end(), objectToDelete) == nodes.end()) {
            nodes.push_back(objectToDelete);
        }
    }
    return nodes;
}

}  // namespace

const MTypeId DirectionalRetopoQuadPreviewShape::kTypeId(0x0013B2C3);
const MString DirectionalRetopoQuadPreviewShape::kTypeName(
    "directionalRetopoQuadPreviewShape");
const MString DirectionalRetopoQuadPreviewShape::kDrawDbClassification(
    "drawdb/geometry/directionalRetopoQuadPreview");
const MString DirectionalRetopoQuadPreviewShape::kDrawRegistrantId(
    "DirectionalRetopoQuadPreviewPlugin");

void* DirectionalRetopoQuadPreviewShape::creator()
{
    return new DirectionalRetopoQuadPreviewShape();
}

MStatus DirectionalRetopoQuadPreviewShape::initialize()
{
    return MS::kSuccess;
}

bool DirectionalRetopoQuadPreviewShape::isBounded() const
{
    return false;
}

MBoundingBox DirectionalRetopoQuadPreviewShape::boundingBox() const
{
    return MBoundingBox();
}

void DirectionalRetopoQuadPreviewShape::setModel(
    std::shared_ptr<QuadPreviewModel> model)
{
    std::scoped_lock lock(modelMutex_);
    model_ = std::move(model);
}

bool DirectionalRetopoQuadPreviewShape::snapshot(
    QuadPreviewSnapshot& snapshot) const
{
    std::shared_ptr<QuadPreviewModel> model;
    {
        std::scoped_lock lock(modelMutex_);
        model = model_;
    }
    return model && model->snapshot(snapshot);
}

MHWRender::MPxDrawOverride*
DirectionalRetopoQuadPreviewDrawOverride::creator(const MObject& object)
{
    return new DirectionalRetopoQuadPreviewDrawOverride(object);
}

DirectionalRetopoQuadPreviewDrawOverride::DirectionalRetopoQuadPreviewDrawOverride(
    const MObject& object)
    : MHWRender::MPxDrawOverride(object, nullptr, false)
{
}

MHWRender::DrawAPI DirectionalRetopoQuadPreviewDrawOverride::supportedDrawAPIs() const
{
    return MHWRender::kOpenGL | MHWRender::kOpenGLCoreProfile |
        MHWRender::kDirectX11;
}

bool DirectionalRetopoQuadPreviewDrawOverride::hasUIDrawables() const
{
    return true;
}

MMatrix DirectionalRetopoQuadPreviewDrawOverride::transform(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return MMatrix::identity;
}

bool DirectionalRetopoQuadPreviewDrawOverride::isBounded(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return false;
}

MBoundingBox DirectionalRetopoQuadPreviewDrawOverride::boundingBox(
    const MDagPath& /*objectPath*/,
    const MDagPath& /*cameraPath*/) const
{
    return MBoundingBox();
}

bool DirectionalRetopoQuadPreviewDrawOverride::disableInternalBoundingBoxDraw() const
{
    return true;
}

bool DirectionalRetopoQuadPreviewDrawOverride::excludedFromPostEffects() const
{
    return true;
}

bool DirectionalRetopoQuadPreviewDrawOverride::wantUserSelection() const
{
    return false;
}

MUserData* DirectionalRetopoQuadPreviewDrawOverride::prepareForDraw(
    const MDagPath& objectPath,
    const MDagPath& /*cameraPath*/,
    const MHWRender::MFrameContext& /*frameContext*/,
    MUserData* oldData)
{
    auto* drawData = dynamic_cast<QuadPreviewDrawData*>(oldData);
    if (drawData == nullptr) {
        drawData = new QuadPreviewDrawData();
    }
    drawData->drawable = false;
    drawData->rawWorldLinePoints.clear();
    drawData->conformedWorldLinePoints.clear();
    drawData->transitionCollarWorldLinePoints.clear();
    drawData->triangleWorldLinePoints.clear();
    drawData->sourceBoundaryWorldLinePoints.clear();
    drawData->resultBoundaryWorldLinePoints.clear();
    drawData->boundaryCorrespondenceWorldLinePoints.clear();
    drawData->requiredBoundaryAnchorWorldPoints.clear();

    MStatus status;
    MFnDependencyNode dependencyNode(objectPath.node(&status), &status);
    auto* shape = status
        ? dynamic_cast<DirectionalRetopoQuadPreviewShape*>(dependencyNode.userNode())
        : nullptr;
    QuadPreviewSnapshot snapshot;
    if (shape == nullptr || !shape->snapshot(snapshot)) {
        return drawData;
    }
    drawData->drawable = true;
    if (snapshot.style.showRawQuadPreview) {
        drawData->rawWorldLinePoints = snapshot.rawWorldLinePoints;
    }
    if (snapshot.style.showConformedQuadPreview) {
        drawData->conformedWorldLinePoints = snapshot.conformedWorldLinePoints;
    }
    if (snapshot.style.showTransitionCollar) {
        drawData->transitionCollarWorldLinePoints = snapshot.transitionCollarWorldLinePoints;
    }
    if (snapshot.style.showTrianglePolygons) {
        drawData->triangleWorldLinePoints = snapshot.triangleWorldLinePoints;
    }
    if (snapshot.style.showSourceBoundary) {
        drawData->sourceBoundaryWorldLinePoints =
            snapshot.sourceBoundaryWorldLinePoints;
    }
    if (snapshot.style.showResultBoundary) {
        drawData->resultBoundaryWorldLinePoints =
            snapshot.resultBoundaryWorldLinePoints;
    }
    if (snapshot.style.showBoundaryCorrespondence) {
        drawData->boundaryCorrespondenceWorldLinePoints =
            snapshot.boundaryCorrespondenceWorldLinePoints;
    }
    if (snapshot.style.showRequiredBoundaryAnchors) {
        drawData->requiredBoundaryAnchorWorldPoints =
            snapshot.requiredBoundaryAnchorWorldPoints;
    }
    drawData->rawColor = MColor(
        snapshot.style.rawWireColor.r,
        snapshot.style.rawWireColor.g,
        snapshot.style.rawWireColor.b,
        snapshot.style.rawWireOpacity);
    drawData->conformedColor = MColor(
        snapshot.style.conformedWireColor.r,
        snapshot.style.conformedWireColor.g,
        snapshot.style.conformedWireColor.b,
        snapshot.style.conformedWireOpacity);
    drawData->transitionCollarColor = MColor(
        snapshot.style.transitionCollarColor.r,
        snapshot.style.transitionCollarColor.g,
        snapshot.style.transitionCollarColor.b,
        snapshot.style.transitionCollarOpacity);
    drawData->triangleColor = MColor(
        snapshot.style.trianglePolygonColor.r,
        snapshot.style.trianglePolygonColor.g,
        snapshot.style.trianglePolygonColor.b,
        snapshot.style.trianglePolygonOpacity);
    drawData->sourceBoundaryColor = MColor(
        snapshot.style.sourceBoundaryColor.r,
        snapshot.style.sourceBoundaryColor.g,
        snapshot.style.sourceBoundaryColor.b,
        snapshot.style.sourceBoundaryOpacity);
    drawData->resultBoundaryColor = MColor(
        snapshot.style.resultBoundaryColor.r,
        snapshot.style.resultBoundaryColor.g,
        snapshot.style.resultBoundaryColor.b,
        snapshot.style.resultBoundaryOpacity);
    drawData->boundaryCorrespondenceColor = MColor(
        snapshot.style.boundaryCorrespondenceColor.r,
        snapshot.style.boundaryCorrespondenceColor.g,
        snapshot.style.boundaryCorrespondenceColor.b,
        snapshot.style.boundaryCorrespondenceOpacity);
    drawData->requiredBoundaryAnchorColor = MColor(
        snapshot.style.requiredBoundaryAnchorColor.r,
        snapshot.style.requiredBoundaryAnchorColor.g,
        snapshot.style.requiredBoundaryAnchorColor.b,
        snapshot.style.requiredBoundaryAnchorOpacity);
    drawData->rawLineWidth = snapshot.style.rawWireLineWidth;
    drawData->conformedLineWidth = snapshot.style.conformedWireLineWidth;
    drawData->transitionCollarLineWidth = snapshot.style.transitionCollarLineWidth;
    drawData->triangleLineWidth = snapshot.style.trianglePolygonLineWidth;
    drawData->sourceBoundaryLineWidth =
        snapshot.style.sourceBoundaryLineWidth;
    drawData->resultBoundaryLineWidth =
        snapshot.style.resultBoundaryLineWidth;
    drawData->boundaryCorrespondenceLineWidth =
        snapshot.style.boundaryCorrespondenceLineWidth;
    drawData->requiredBoundaryAnchorPointSize =
        snapshot.style.requiredBoundaryAnchorPointSize;
    drawData->depthPriority = snapshot.style.wireDepthPriority;
    return drawData;
}

void DirectionalRetopoQuadPreviewDrawOverride::addUIDrawables(
    const MDagPath& /*objectPath*/,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& /*frameContext*/,
    const MUserData* data)
{
    const auto* drawData = dynamic_cast<const QuadPreviewDrawData*>(data);
    if (drawData == nullptr || !drawData->drawable) {
        return;
    }
    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    drawManager.setDepthPriority(drawData->depthPriority);
    if (drawData->rawWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->rawColor);
        drawManager.setLineWidth(drawData->rawLineWidth);
        (void)drawManager.lineList(drawData->rawWorldLinePoints, false);
    }
    if (drawData->conformedWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->conformedColor);
        drawManager.setLineWidth(drawData->conformedLineWidth);
        (void)drawManager.lineList(drawData->conformedWorldLinePoints, false);
    }
    if (drawData->transitionCollarWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->transitionCollarColor);
        drawManager.setLineWidth(drawData->transitionCollarLineWidth);
        (void)drawManager.lineList(drawData->transitionCollarWorldLinePoints, false);
    }
    if (drawData->triangleWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->triangleColor);
        drawManager.setLineWidth(drawData->triangleLineWidth);
        (void)drawManager.lineList(drawData->triangleWorldLinePoints, false);
    }
    if (drawData->sourceBoundaryWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->sourceBoundaryColor);
        drawManager.setLineWidth(drawData->sourceBoundaryLineWidth);
        (void)drawManager.lineList(
            drawData->sourceBoundaryWorldLinePoints,
            false);
    }
    if (drawData->resultBoundaryWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->resultBoundaryColor);
        drawManager.setLineWidth(drawData->resultBoundaryLineWidth);
        (void)drawManager.lineList(
            drawData->resultBoundaryWorldLinePoints,
            false);
    }
    if (drawData->boundaryCorrespondenceWorldLinePoints.length() >= 2U) {
        drawManager.setColor(drawData->boundaryCorrespondenceColor);
        drawManager.setLineWidth(drawData->boundaryCorrespondenceLineWidth);
        (void)drawManager.lineList(
            drawData->boundaryCorrespondenceWorldLinePoints,
            false);
    }
    if (drawData->requiredBoundaryAnchorWorldPoints.length() >= 1U) {
        drawManager.setColor(drawData->requiredBoundaryAnchorColor);
        drawManager.setPointSize(drawData->requiredBoundaryAnchorPointSize);
        drawManager.points(
            drawData->requiredBoundaryAnchorWorldPoints,
            false);
    }
    drawManager.endDrawable();
}

QuadPreviewDrawable::~QuadPreviewDrawable()
{
    destroy();
}

MStatus QuadPreviewDrawable::create(const std::shared_ptr<QuadPreviewModel>& model)
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
        DirectionalRetopoQuadPreviewShape::kTypeId,
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
    auto* shape = status
        ? dynamic_cast<DirectionalRetopoQuadPreviewShape*>(dependencyNode.userNode())
        : nullptr;
    if (shape == nullptr) {
        destroy();
        return MS::kFailure;
    }
    shape->setModel(model);
    markDirty();
    if (liveShapeCount() != 1U) {
        destroyAll();
        transformHandle_ = MObjectHandle();
        shapeHandle_ = MObjectHandle();
        return MS::kFailure;
    }
    return MS::kSuccess;
}

void QuadPreviewDrawable::destroy() noexcept
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

void QuadPreviewDrawable::markDirty() const noexcept
{
    if (shapeHandle_.isValid() && shapeHandle_.isAlive()) {
        MHWRender::MRenderer::setGeometryDrawDirty(shapeHandle_.object(), true);
    }
}

bool QuadPreviewDrawable::exists() const noexcept
{
    return shapeHandle_.isValid() && shapeHandle_.isAlive();
}

void QuadPreviewDrawable::destroyAll() noexcept
{
    const std::vector<MObject> nodes = previewNodesForDeletion();
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

unsigned int QuadPreviewDrawable::liveShapeCount() noexcept
{
    unsigned int count = 0U;
    MStatus status;
    MItDependencyNodes iterator(MFn::kPluginLocatorNode, &status);
    if (!status) {
        return count;
    }
    for (; !iterator.isDone(); iterator.next()) {
        const MObject object = iterator.thisNode(&status);
        if (!status || object.isNull()) {
            continue;
        }
        MFnDependencyNode node(object, &status);
        if (status && node.typeId() == DirectionalRetopoQuadPreviewShape::kTypeId) {
            ++count;
        }
    }
    return count;
}

}  // namespace directional_retopo
