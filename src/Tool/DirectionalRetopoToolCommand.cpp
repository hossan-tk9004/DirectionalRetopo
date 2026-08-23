#include "Tool/DirectionalRetopoToolCommand.h"

#include "Plugin/PluginNames.h"

#include <maya/MArgList.h>

namespace directional_retopo {

DirectionalRetopoToolCommand::DirectionalRetopoToolCommand()
{
    setCommandString(kToolCommandName);
}

void* DirectionalRetopoToolCommand::creator()
{
    return new DirectionalRetopoToolCommand();
}

MSyntax DirectionalRetopoToolCommand::newSyntax()
{
    return MSyntax();
}

MStatus DirectionalRetopoToolCommand::doIt(const MArgList& /*arguments*/)
{
    return redoIt();
}

MStatus DirectionalRetopoToolCommand::redoIt()
{
    // Phase 1 records data only and deliberately makes no scene changes.
    return MS::kSuccess;
}

MStatus DirectionalRetopoToolCommand::undoIt()
{
    // Kept as the future one-stroke/one-undo integration point.
    return MS::kSuccess;
}

bool DirectionalRetopoToolCommand::isUndoable() const
{
    return true;
}

MStatus DirectionalRetopoToolCommand::finalize()
{
    MArgList journalArguments;
    MStatus status = journalArguments.addArg(commandString());
    if (!status) {
        return status;
    }
    return MPxToolCommand::doFinalize(journalArguments);
}

void DirectionalRetopoToolCommand::setStrokeData(const StrokeData& strokeData)
{
    strokeData_ = strokeData;
}

}  // namespace directional_retopo
