#pragma once

#include "Viewport/QuadPreviewModel.h"

#include <maya/MObjectHandle.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>

#include <memory>
#include <mutex>

namespace directional_retopo {

class DirectionalRetopoQuadPreviewShape final : public MPxLocatorNode
{
public:
    static void* creator();
    static MStatus initialize();

    bool isBounded() const override;
    MBoundingBox boundingBox() const override;

    void setModel(std::shared_ptr<QuadPreviewModel> model);
    bool snapshot(QuadPreviewSnapshot& snapshot) const;

    static const MTypeId kTypeId;
    static const MString kTypeName;
    static const MString kDrawDbClassification;
    static const MString kDrawRegistrantId;

private:
    mutable std::mutex modelMutex_;
    std::shared_ptr<QuadPreviewModel> model_;
};

class DirectionalRetopoQuadPreviewDrawOverride final
    : public MHWRender::MPxDrawOverride
{
public:
    static MHWRender::MPxDrawOverride* creator(const MObject& object);

    MHWRender::DrawAPI supportedDrawAPIs() const override;
    bool hasUIDrawables() const override;
    MMatrix transform(
        const MDagPath& objectPath,
        const MDagPath& cameraPath) const override;
    bool isBounded(
        const MDagPath& objectPath,
        const MDagPath& cameraPath) const override;
    MBoundingBox boundingBox(
        const MDagPath& objectPath,
        const MDagPath& cameraPath) const override;
    bool disableInternalBoundingBoxDraw() const override;
    bool excludedFromPostEffects() const override;
    bool wantUserSelection() const override;

    MUserData* prepareForDraw(
        const MDagPath& objectPath,
        const MDagPath& cameraPath,
        const MHWRender::MFrameContext& frameContext,
        MUserData* oldData) override;
    void addUIDrawables(
        const MDagPath& objectPath,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext,
        const MUserData* data) override;

private:
    explicit DirectionalRetopoQuadPreviewDrawOverride(const MObject& object);
};

class QuadPreviewDrawable final
{
public:
    ~QuadPreviewDrawable();

    QuadPreviewDrawable(const QuadPreviewDrawable&) = delete;
    QuadPreviewDrawable& operator=(const QuadPreviewDrawable&) = delete;
    QuadPreviewDrawable() = default;

    MStatus create(const std::shared_ptr<QuadPreviewModel>& model);
    void destroy() noexcept;
    void markDirty() const noexcept;
    [[nodiscard]] bool exists() const noexcept;

    static void destroyAll() noexcept;
    [[nodiscard]] static unsigned int liveShapeCount() noexcept;

private:
    MObjectHandle transformHandle_;
    MObjectHandle shapeHandle_;
};

}  // namespace directional_retopo

