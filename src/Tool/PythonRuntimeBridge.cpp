#include "Tool/PythonRuntimeBridge.h"

#include <maya/MGlobal.h>
#include <maya/MString.h>

namespace directional_retopo {
namespace {

MStatus callRuntimeFunction(const char* functionName)
{
    // Resolve the repository's scripts directory from the loaded .mll so a
    // Context activated through setToolTo/Tool History does not depend on a
    // previous Python helper call to populate sys.path.
    MString command(
        "import importlib as _dr_importlib, os as _dr_os, sys as _dr_sys\n"
        "import maya.cmds as _dr_cmds\n"
        "_dr_plugin_path = _dr_cmds.pluginInfo("
        "'DirectionalRetopo', query=True, path=True)\n"
        "_dr_scripts = _dr_os.path.normpath(_dr_os.path.join("
        "_dr_os.path.dirname(_dr_plugin_path), '..', '..', 'scripts'))\n"
        "_dr_scripts not in _dr_sys.path and "
        "_dr_sys.path.insert(0, _dr_scripts)\n"
        "_dr_runtime = _dr_importlib.import_module("
        "'directional_retopo.runtime')\n"
        "_dr_runtime.");
    command += functionName;
    command += "()";
    return MGlobal::executePythonCommand(command, false, false);
}

}  // namespace

MStatus activatePythonRuntime()
{
    return callRuntimeFunction("on_context_activated");
}

MStatus deactivatePythonRuntime()
{
    return callRuntimeFunction("on_context_deactivated");
}

MStatus unloadPythonRuntime()
{
    return callRuntimeFunction("on_plugin_unloaded");
}

MStatus installToolSettingsScripts()
{
    // Make Maya's standard <ContextClassName>Properties/Values procedures
    // available even when the plug-in is loaded directly from Plug-in
    // Manager rather than through directional_retopo.tool.load_plugin().
    const MString command(
        "import maya.cmds as _dr_cmds, maya.mel as _dr_mel, os as _dr_os\n"
        "_dr_plugin_path = _dr_cmds.pluginInfo("
        "'DirectionalRetopo', query=True, path=True)\n"
        "_dr_scripts = _dr_os.path.normpath(_dr_os.path.join("
        "_dr_os.path.dirname(_dr_plugin_path), '..', '..', 'scripts'))\n"
        "for _dr_name in ('DirectionalRetopoContextProperties.mel', "
        "'DirectionalRetopoContextValues.mel'):\n"
        "    _dr_path = _dr_os.path.join(_dr_scripts, _dr_name)\n"
        "    if not _dr_os.path.isfile(_dr_path):\n"
        "        raise RuntimeError('DirectionalRetopo Tool Settings script "
        "not found: ' + _dr_path)\n"
        "    _dr_path = _dr_path.replace(_dr_os.sep, '/')\n"
        "    _dr_mel.eval('source \"' + _dr_path + '\";')");
    return MGlobal::executePythonCommand(command, false, false);
}

}  // namespace directional_retopo
