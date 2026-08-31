"""Per-activation Maya GUI runtime for the DirectionalRetopo context."""

import maya.cmds as cmds
from PySide2 import QtCore, QtWidgets


PLUGIN_NAME = "DirectionalRetopo"
CONTEXT_NAME = "directionalRetopoContext1"

_FILTER_ATTRIBUTE = "_directional_retopo_radius_event_filter"
_ACTIVE_ATTRIBUTE = "_directional_retopo_context_active"
_LEGACY_SCRIPT_JOB_ATTRIBUTE = "_directional_retopo_tool_changed_job"
_FILTER_OBJECT_NAME = "DirectionalRetopoBrushRadiusEventFilter"
_RADIUS_CONTROL_NAME = "directionalRetopoRadiusFSG"


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


def _focus_widget(watched=None):
    app = _application()
    widget = app.focusWidget() if app is not None else None
    if widget is None and isinstance(watched, QtWidgets.QWidget):
        widget = watched
    return widget


def _widget_accepts_text_input(widget) -> bool:
    """Return whether *widget* or one of its parents edits text or numbers."""
    while widget is not None:
        if isinstance(
            widget,
            (
                QtWidgets.QLineEdit,
                QtWidgets.QTextEdit,
                QtWidgets.QPlainTextEdit,
                QtWidgets.QAbstractSpinBox,
            ),
        ):
            return True
        if isinstance(widget, QtWidgets.QComboBox) and widget.isEditable():
            return True
        widget = widget.parentWidget()
    return False


def _viewport_has_keyboard_focus(watched=None) -> bool:
    """Limit Maya-style B-drag handling to a focused model panel."""
    if _widget_accepts_text_input(_focus_widget(watched)):
        return False
    try:
        panel = cmds.getPanel(withFocus=True)
        return bool(panel) and cmds.getPanel(typeOf=panel) == "modelPanel"
    except (RuntimeError, TypeError, ValueError):
        return False


def _sync_tool_settings_radius() -> None:
    """Refresh the visible Radius control from the C++ context, if present."""
    if not _plugin_is_loaded() or not _context_exists():
        return
    try:
        if not cmds.floatSliderGrp(_RADIUS_CONTROL_NAME, exists=True):
            return
        radius = cmds.directionalRetopoContext(
            CONTEXT_NAME,
            query=True,
            radius=True,
        )
        cmds.floatSliderGrp(
            _RADIUS_CONTROL_NAME,
            edit=True,
            value=float(radius),
        )
    except (RuntimeError, TypeError, ValueError):
        # The Tool Settings layout may be rebuilding while the key is released.
        # Its normal Values callback will query the same C++ source of truth.
        pass


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
    if not enabled:
        _sync_tool_settings_radius()


class _BrushRadiusEventFilter(QtCore.QObject):
    """Forward Maya-style B-key radius mode only while our context is active."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName(_FILTER_OBJECT_NAME)
        self._b_held = False

    def release_radius_mode(self) -> None:
        if not self._b_held:
            return
        self._b_held = False
        set_radius_adjust_mode(False)

    def eventFilter(self, watched, event):  # noqa: N802 - Qt virtual name
        if not _context_is_current():
            self.release_radius_mode()
            return False

        event_type = event.type()
        if event_type in (
            QtCore.QEvent.ApplicationDeactivate,
            QtCore.QEvent.WindowDeactivate,
        ):
            self.release_radius_mode()
            return False

        if event_type not in (
            QtCore.QEvent.ShortcutOverride,
            QtCore.QEvent.KeyPress,
            QtCore.QEvent.KeyRelease,
        ):
            return False
        if event.key() != QtCore.Qt.Key_B:
            return False

        # A release must always close a radius gesture that began in the
        # Viewport, even if keyboard focus moved during the drag. A B key that
        # began in an editor remains entirely owned by that editor.
        if event_type == QtCore.QEvent.KeyRelease:
            if not self._b_held:
                return False
            event.accept()
            self.release_radius_mode()
            return True

        # QApplication-level filters also see Script Editor and Tool Settings
        # input. Only claim B while a model panel has keyboard focus.
        if not _viewport_has_keyboard_focus(watched):
            return False

        event.accept()
        if not self._b_held and not event.isAutoRepeat():
            # Maya may consume ShortcutOverride before a subsequent KeyPress
            # reaches this filter. Enter held mode on the first of either event
            # and de-duplicate the other event with _b_held.
            self._b_held = True
            set_radius_adjust_mode(True)
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
            release_radius_mode = getattr(event_filter, "release_radius_mode", None)
            if callable(release_radius_mode):
                release_radius_mode()
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
