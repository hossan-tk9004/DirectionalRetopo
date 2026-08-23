"""Per-activation Maya GUI runtime for the DirectionalRetopo context."""

import maya.cmds as cmds
from PySide2 import QtCore, QtWidgets


PLUGIN_NAME = "DirectionalRetopo"
CONTEXT_NAME = "directionalRetopoContext1"

_FILTER_ATTRIBUTE = "_directional_retopo_radius_event_filter"
_ACTIVE_ATTRIBUTE = "_directional_retopo_context_active"
_LEGACY_SCRIPT_JOB_ATTRIBUTE = "_directional_retopo_tool_changed_job"
_FILTER_OBJECT_NAME = "DirectionalRetopoBrushRadiusEventFilter"


def _application():
    return QtWidgets.QApplication.instance()


def _plugin_is_loaded() -> bool:
    try:
        return bool(cmds.pluginInfo(PLUGIN_NAME, query=True, loaded=True))
    except (RuntimeError, TypeError):
        return False


def _context_exists() -> bool:
    try:
        return bool(cmds.contextInfo(CONTEXT_NAME, exists=True))
    except (RuntimeError, TypeError):
        return False


def _context_is_current() -> bool:
    app = _application()
    if app is None or not bool(getattr(app, _ACTIVE_ATTRIBUTE, False)):
        return False
    try:
        return cmds.currentCtx() == CONTEXT_NAME
    except RuntimeError:
        return False


def set_radius_adjust_mode(enabled: bool) -> None:
    """Forward B-key state to the active C++ context."""
    if not _plugin_is_loaded() or not _context_exists():
        return
    try:
        cmds.directionalRetopoContext(
            CONTEXT_NAME,
            edit=True,
            radiusAdjustMode=bool(enabled),
        )
    except RuntimeError:
        # Cleanup must tolerate a context that is already being deleted or an
        # older development binary. Enabling remains strict.
        if enabled:
            raise


class _BrushRadiusEventFilter(QtCore.QObject):
    """Forward Maya-style B-key radius mode only while our context is active."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName(_FILTER_OBJECT_NAME)

    def eventFilter(self, watched, event):  # noqa: N802 - Qt virtual name
        del watched

        if not _context_is_current():
            return False

        event_type = event.type()
        if event_type in (
            QtCore.QEvent.ApplicationDeactivate,
            QtCore.QEvent.WindowDeactivate,
        ):
            set_radius_adjust_mode(False)
            return False

        if event_type not in (
            QtCore.QEvent.ShortcutOverride,
            QtCore.QEvent.KeyPress,
            QtCore.QEvent.KeyRelease,
        ):
            return False
        if event.key() != QtCore.Qt.Key_B:
            return False

        event.accept()
        if event_type == QtCore.QEvent.ShortcutOverride:
            return True
        if event.isAutoRepeat():
            return True

        set_radius_adjust_mode(event_type == QtCore.QEvent.KeyPress)
        return True


def _remove_legacy_script_job(app) -> None:
    script_job = getattr(app, _LEGACY_SCRIPT_JOB_ATTRIBUTE, None)
    if script_job is None:
        return
    try:
        if cmds.scriptJob(exists=script_job):
            cmds.scriptJob(kill=script_job, force=True)
    except RuntimeError:
        pass
    try:
        delattr(app, _LEGACY_SCRIPT_JOB_ATTRIBUTE)
    except AttributeError:
        pass


def _managed_filters(app):
    filters = []
    stored_filter = getattr(app, _FILTER_ATTRIBUTE, None)
    if stored_filter is not None:
        filters.append(stored_filter)

    for event_filter in app.findChildren(QtCore.QObject, _FILTER_OBJECT_NAME):
        if all(event_filter is not existing for existing in filters):
            filters.append(event_filter)
    return filters


def _remove_filters(app) -> None:
    for event_filter in _managed_filters(app):
        try:
            app.removeEventFilter(event_filter)
            event_filter.setParent(None)
            event_filter.deleteLater()
        except RuntimeError:
            # A module reload can leave a Python wrapper whose underlying Qt
            # object Maya has already destroyed.
            pass

    try:
        delattr(app, _FILTER_ATTRIBUTE)
    except AttributeError:
        pass
    _remove_legacy_script_job(app)


def on_context_activated() -> int:
    """Install exactly one filter for the current context activation."""
    app = _application()
    if app is None:
        return 0

    # Removing by both QApplication attribute and objectName cleans filters
    # created by an older module object before a development reload.
    _remove_filters(app)
    setattr(app, _ACTIVE_ATTRIBUTE, True)

    event_filter = _BrushRadiusEventFilter(app)
    app.installEventFilter(event_filter)
    setattr(app, _FILTER_ATTRIBUTE, event_filter)
    return filter_count()


def on_context_deactivated() -> int:
    """Remove all managed filters when the context becomes inactive."""
    app = _application()
    if app is None:
        return 0
    setattr(app, _ACTIVE_ATTRIBUTE, False)
    _remove_filters(app)
    return filter_count()


def on_plugin_unloaded() -> int:
    """Best-effort cleanup for plug-in unload and development reloads."""
    return on_context_deactivated()


def filter_count() -> int:
    """Return the number of live DirectionalRetopo filter objects in Qt."""
    app = _application()
    if app is None:
        return 0
    return len(app.findChildren(QtCore.QObject, _FILTER_OBJECT_NAME))
