#pragma once

#include <maya/MStatus.h>

namespace directional_retopo {

MStatus activatePythonRuntime();
MStatus deactivatePythonRuntime();
MStatus unloadPythonRuntime();
MStatus installToolSettingsScripts();

}  // namespace directional_retopo
