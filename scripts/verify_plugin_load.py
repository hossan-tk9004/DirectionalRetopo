"""Load and unload the Release plug-in from a running Maya 2024.2 session."""

from pathlib import Path

import maya.cmds as cmds


PROJECT_ROOT = Path(__file__).resolve().parents[1]
PLUGIN_PATH = PROJECT_ROOT / "build" / "Release" / "DirectionalRetopo.mll"


def verify_plugin_load() -> str:
    if not PLUGIN_PATH.is_file():
        raise FileNotFoundError(
            f"Plug-in not found: {PLUGIN_PATH}. Run build_maya2024.bat first."
        )

    loaded_result = cmds.loadPlugin(str(PLUGIN_PATH), quiet=True)
    plugin_name = loaded_result[0] if isinstance(loaded_result, (list, tuple)) else loaded_result

    if not cmds.pluginInfo(plugin_name, query=True, loaded=True):
        raise RuntimeError(f"Maya did not report {plugin_name} as loaded.")

    print(f"Verified Maya plug-in load: {plugin_name} ({PLUGIN_PATH})")
    cmds.unloadPlugin(plugin_name, force=True)
    print(f"Verified Maya plug-in unload: {plugin_name}")
    return plugin_name


if __name__ == "__main__":
    verify_plugin_load()
