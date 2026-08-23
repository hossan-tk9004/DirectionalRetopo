#pragma once

#include "Paint/StrokeData.h"

#include <maya/MPxToolCommand.h>
#include <maya/MSyntax.h>

namespace directional_retopo {

class DirectionalRetopoToolCommand final : public MPxToolCommand
{
public:
    DirectionalRetopoToolCommand();
    ~DirectionalRetopoToolCommand() override = default;

    static void* creator();
    static MSyntax newSyntax();

    MStatus doIt(const MArgList& arguments) override;
    MStatus redoIt() override;
    MStatus undoIt() override;
    bool isUndoable() const override;
    MStatus finalize() override;

    void setStrokeData(const StrokeData& strokeData);

private:
    StrokeData strokeData_;
};

}  // namespace directional_retopo
