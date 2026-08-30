"""Maya standalone registration smoke test for DirectionalRetopo."""

from pathlib import Path
import sys

import maya.standalone


def main() -> int:
    if len(sys.argv) != 2:
        raise RuntimeError("Expected the absolute DirectionalRetopo.mll path")

    plugin_path = Path(sys.argv[1]).resolve()
    if not plugin_path.is_file():
        raise FileNotFoundError(plugin_path)

    maya.standalone.initialize(name="python")
    try:
        import maya.cmds as cmds
        import maya.mel as mel

        loaded_name = cmds.loadPlugin(str(plugin_path), quiet=True)
        plugin_name = (
            loaded_name[0]
            if isinstance(loaded_name, (list, tuple))
            else loaded_name
        )
        if not cmds.pluginInfo(plugin_name, query=True, loaded=True):
            raise RuntimeError("Plug-in did not report a loaded state")
        command_kind = mel.eval('whatIs "directionalRetopoContext"') or ""
        if "Command" not in command_kind:
            raise RuntimeError("Context command was not registered")

        help_text = cmds.help("directionalRetopoContext") or ""
        required_flags = (
            "radius",
            "densityMode",
            "manualTargetEdgeLength",
            "edgeLengthScale",
            "topologyBlendWidth",
            "showDirectionField",
            "showDensityField",
            "showQuadPreview",
            "showRawQuadPreview",
            "showConformedQuadPreview",
            "showSourceBoundary",
            "showResultBoundary",
            "showBoundaryCorrespondence",
            "showBoundaryAnchors",
            "resetSettings",
            "captureNextRemeshInput",
        )
        missing = [flag for flag in required_flags if flag not in help_text]
        if missing:
            raise RuntimeError(f"Context help is missing flags: {missing}")

        for procedure_name in (
            "DirectionalRetopoContextProperties",
            "DirectionalRetopoContextValues",
        ):
            procedure_kind = mel.eval(f'whatIs "{procedure_name}"') or ""
            if "Mel procedure" not in procedure_kind:
                raise RuntimeError(
                    f"Tool Settings procedure was not sourced: {procedure_name}"
                )

        cmds.unloadPlugin(plugin_name)
        if cmds.pluginInfo(plugin_name, query=True, loaded=True):
            raise RuntimeError("Plug-in remained loaded after unload")
        print("Maya standalone plug-in load/unload: success")
        print(f"Context flags present: {', '.join(required_flags)}")
        print("Tool Settings MEL procedures: source success")
        return 0
    finally:
        maya.standalone.uninitialize()


if __name__ == "__main__":
    raise SystemExit(main())
