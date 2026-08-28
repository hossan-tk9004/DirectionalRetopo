"""Load and activate the DirectionalRetopo Maya context."""

from pathlib import Path

import maya.cmds as cmds
import maya.mel as mel

from . import runtime


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PLUGIN_PATH = PROJECT_ROOT / "build" / "Release" / "DirectionalRetopo.mll"
PLUGIN_NAME = runtime.PLUGIN_NAME
CONTEXT_NAME = runtime.CONTEXT_NAME
CONTEXT_CLASS_NAME = "DirectionalRetopoContext"
TOOL_SETTINGS_SCRIPTS = (
    PROJECT_ROOT / "scripts" / "DirectionalRetopoContextProperties.mel",
    PROJECT_ROOT / "scripts" / "DirectionalRetopoContextValues.mel",
)


def _plugin_is_loaded() -> bool:
    try:
        return bool(cmds.pluginInfo(PLUGIN_NAME, query=True, loaded=True))
    except (RuntimeError, TypeError):
        return False


def _source_tool_settings_scripts() -> None:
    """Define the MEL hooks Maya requests for this context's Tool Settings."""
    for script_path in TOOL_SETTINGS_SCRIPTS:
        if not script_path.is_file():
            raise FileNotFoundError(f"Tool Settings script not found: {script_path}")
        mel.eval(f'source "{script_path.as_posix()}";')


def load_plugin() -> str:
    """Load the local Release build and return Maya's registered plug-in name."""
    if _plugin_is_loaded():
        plugin_name = PLUGIN_NAME
        loaded_path = cmds.pluginInfo(PLUGIN_NAME, query=True, path=True)
        if loaded_path and Path(loaded_path).resolve() != PLUGIN_PATH.resolve():
            raise RuntimeError(
                "A different DirectionalRetopo.mll is already loaded. "
                f"Loaded: {loaded_path}; expected: {PLUGIN_PATH}. "
                "Unload the existing plug-in before activating this build."
            )
    else:
        if not PLUGIN_PATH.is_file():
            raise FileNotFoundError(
                f"Plug-in not found: {PLUGIN_PATH}. Run build_maya2024.bat first."
            )

        result = cmds.loadPlugin(str(PLUGIN_PATH), quiet=True)
        plugin_name = result[0] if isinstance(result, (list, tuple)) else result

    _source_tool_settings_scripts()
    return plugin_name


def _context_class_name() -> str:
    try:
        return str(cmds.contextInfo(CONTEXT_NAME, query=True, c=True) or "")
    except RuntimeError:
        return ""


def _create_context() -> None:
    cmds.directionalRetopoContext(CONTEXT_NAME)
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError(
            f"DirectionalRetopo failed to create context {CONTEXT_NAME!r}."
        )


def _delete_context_for_recovery() -> None:
    """Remove a stale context left by a previous plug-in load."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        return
    if cmds.currentCtx() == CONTEXT_NAME:
        cmds.setToolTo("selectSuperContext")
    runtime.on_context_deactivated()
    cmds.deleteUI(CONTEXT_NAME)
    if cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError(
            f"Stale DirectionalRetopo context {CONTEXT_NAME!r} could not be deleted."
        )


def _ensure_compatible_context() -> None:
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        _create_context()
        return

    context_class = _context_class_name()
    if context_class and context_class != CONTEXT_CLASS_NAME:
        raise RuntimeError(
            f"Context name {CONTEXT_NAME!r} is owned by {context_class!r}, "
            f"not {CONTEXT_CLASS_NAME!r}."
        )

    try:
        # A same-name UI context can survive an unusual manual plug-in reload
        # while its MPxContextCommand instance is no longer live. Querying a
        # harmless property detects that state before the requested edit.
        cmds.directionalRetopoContext(CONTEXT_NAME, query=True, radius=True)
    except RuntimeError:
        _delete_context_for_recovery()
        _create_context()


def activate(radius: float = 1.0) -> str:
    """Create or reuse the context, set its radius, and make it current."""
    if radius <= 0.0:
        raise ValueError("radius must be greater than zero")

    load_plugin()

    _ensure_compatible_context()
    try:
        cmds.directionalRetopoContext(CONTEXT_NAME, edit=True, radius=radius)
    except RuntimeError as error:
        raise RuntimeError(
            "DirectionalRetopo context exists but rejected its radius edit. "
            "Unload the plug-in with tool.unload_plugin(), reload the current "
            "Release build, and try again."
        ) from error
    cmds.setToolTo(CONTEXT_NAME)
    return CONTEXT_NAME


def set_radius(radius: float) -> None:
    """Set the radius on the existing DirectionalRetopo context."""
    if radius <= 0.0:
        raise ValueError("radius must be greater than zero")
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(CONTEXT_NAME, edit=True, radius=radius)


def query_radius() -> float:
    """Return the current context radius."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return float(cmds.directionalRetopoContext(CONTEXT_NAME, query=True, radius=True))


def set_density_mode(mode: str) -> None:
    """Set Density Field mode to ``"Manual"`` or ``"Auto"``."""
    normalized_mode = mode.strip().lower()
    if normalized_mode not in {"manual", "auto"}:
        raise ValueError("mode must be 'Manual' or 'Auto'")
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(
        CONTEXT_NAME,
        edit=True,
        densityMode=normalized_mode,
    )


def query_density_mode() -> str:
    """Return the current Density Field mode."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return str(
        cmds.directionalRetopoContext(CONTEXT_NAME, query=True, densityMode=True)
    )


def set_manual_target_edge_length(edge_length: float) -> None:
    """Set Manual mode's target polygon edge length in world units."""
    if edge_length <= 0.0:
        raise ValueError("edge_length must be greater than zero")
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(
        CONTEXT_NAME,
        edit=True,
        manualTargetEdgeLength=edge_length,
    )


def query_manual_target_edge_length() -> float:
    """Return Manual mode's target polygon edge length."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return float(
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            query=True,
            manualTargetEdgeLength=True,
        )
    )


def set_edge_length_scale(scale: float) -> None:
    """Scale target edge length: 0.5 is finer and 2.0 is coarser."""
    if scale <= 0.0:
        raise ValueError("scale must be greater than zero")
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(CONTEXT_NAME, edit=True, edgeLengthScale=scale)


def query_edge_length_scale() -> float:
    """Return the current target edge-length multiplier."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return float(
        cmds.directionalRetopoContext(CONTEXT_NAME, query=True, edgeLengthScale=True)
    )


def set_topology_blend_width(rings: int) -> None:
    """Set the shared Direction/Density transition width in face rings (1-10)."""
    if isinstance(rings, bool) or not isinstance(rings, int):
        raise TypeError("rings must be an integer")
    if rings < 0 or rings > 10:
        raise ValueError("rings must be between 0 and 10; legacy 0 is clamped to 1")
    if rings == 0:
        rings = 1
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(
        CONTEXT_NAME,
        edit=True,
        topologyBlendWidth=rings,
    )


def query_topology_blend_width() -> int:
    """Return the Paint Region transition width in face rings."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return int(
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            query=True,
            topologyBlendWidth=True,
        )
    )


def set_field_visualization(
    *,
    show_direction: bool | None = None,
    show_density: bool | None = None,
) -> None:
    """Toggle cached Direction and Density debug feedback independently."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    if show_direction is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showDirectionField=bool(show_direction),
        )
    if show_density is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showDensityField=bool(show_density),
        )


def query_field_visualization() -> tuple[bool, bool]:
    """Return ``(show_direction, show_density)``."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return (
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showDirectionField=True,
            )
        ),
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showDensityField=True,
            )
        ),
    )


def set_quad_preview_visibility(show: bool) -> None:
    """Show or hide the cached Phase 4 Quad Preview drawable."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(
        CONTEXT_NAME,
        edit=True,
        showQuadPreview=bool(show),
    )


def query_quad_preview_visibility() -> bool:
    """Return whether the cached Phase 4 Quad Preview is visible."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return bool(
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            query=True,
            showQuadPreview=True,
        )
    )


def set_quad_preview_layers(
    *,
    show_raw: bool | None = None,
    show_conformed: bool | None = None,
    show_source_boundary: bool | None = None,
    show_result_boundary: bool | None = None,
) -> None:
    """Toggle Raw, Conformed, Source-boundary and Result-boundary layers."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    if show_raw is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showRawQuadPreview=bool(show_raw),
        )
    if show_conformed is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showConformedQuadPreview=bool(show_conformed),
        )
    if show_source_boundary is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showSourceBoundary=bool(show_source_boundary),
        )
    if show_result_boundary is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showResultBoundary=bool(show_result_boundary),
        )


def query_quad_preview_layers() -> tuple[bool, bool]:
    """Return ``(show_raw, show_conformed)`` for backward compatibility."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return (
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showRawQuadPreview=True,
            )
        ),
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showConformedQuadPreview=True,
            )
        ),
    )


def query_boundary_preview_layers() -> tuple[bool, bool]:
    """Return ``(show_source_boundary, show_result_boundary)``."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return (
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showSourceBoundary=True,
            )
        ),
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showResultBoundary=True,
            )
        ),
    )


def set_boundary_debug_visualization(
    *,
    show_correspondence: bool | None = None,
    show_anchors: bool | None = None,
) -> None:
    """Toggle ordered correspondence lines and required split anchors."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    if show_correspondence is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showBoundaryCorrespondence=bool(show_correspondence),
        )
    if show_anchors is not None:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            showBoundaryAnchors=bool(show_anchors),
        )


def query_boundary_debug_visualization() -> tuple[bool, bool]:
    """Return ``(show_correspondence, show_required_anchors)``."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return (
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showBoundaryCorrespondence=True,
            )
        ),
        bool(
            cmds.directionalRetopoContext(
                CONTEXT_NAME,
                query=True,
                showBoundaryAnchors=True,
            )
        ),
    )


def reset_settings() -> None:
    """Restore C++ defaults and rebuild the current Preview when available."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    cmds.directionalRetopoContext(CONTEXT_NAME, edit=True, resetSettings=True)


def set_radius_adjust_mode(enabled: bool) -> None:
    """Safely set the C++ Context's B-drag radius adjustment state."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    runtime.set_radius_adjust_mode(enabled)


def query_radius_adjust_mode() -> bool:
    """Return whether the C++ Context is currently in radius adjustment mode."""
    if not cmds.contextInfo(CONTEXT_NAME, exists=True):
        raise RuntimeError("DirectionalRetopo context does not exist; call activate() first")
    return bool(
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            query=True,
            radiusAdjustMode=True,
        )
    )


def deactivate() -> None:
    """Return Maya to its standard selection context."""
    try:
        cmds.setToolTo("selectSuperContext")
    finally:
        # C++ toolOffCleanup is the primary owner. This idempotent fallback
        # also cleans a stale filter if the context was already inactive.
        runtime.on_context_deactivated()


def unload_plugin() -> None:
    """Delete the custom context and unload the plug-in before rebuilding."""
    deactivate()
    runtime.on_plugin_unloaded()
    if cmds.contextInfo(CONTEXT_NAME, exists=True):
        cmds.deleteUI(CONTEXT_NAME)
    if _plugin_is_loaded():
        cmds.unloadPlugin(PLUGIN_NAME)
