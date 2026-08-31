"""Maya/PySide regression checks for focus-aware B-drag key forwarding."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_PATH = PROJECT_ROOT / "scripts"
if str(SCRIPTS_PATH) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_PATH))

from PySide2 import QtCore, QtWidgets  # noqa: E402
from directional_retopo import runtime  # noqa: E402


class _FakeKeyEvent:
    def __init__(self, event_type, key=QtCore.Qt.Key_B, auto_repeat=False):
        self._event_type = event_type
        self._key = key
        self._auto_repeat = auto_repeat
        self.accepted = False

    def type(self):
        return self._event_type

    def key(self):
        return self._key

    def isAutoRepeat(self):
        return self._auto_repeat

    def accept(self):
        self.accepted = True


class _FakeCmds:
    def __init__(self):
        self.radius_mode_transitions = []
        self.radius_control_values = []

    @staticmethod
    def pluginInfo(_name, query=False, loaded=False):
        assert query and loaded
        return True

    @staticmethod
    def contextInfo(_name, exists=False):
        assert exists
        return True

    def directionalRetopoContext(
        self,
        _name,
        edit=False,
        query=False,
        radiusAdjustMode=None,
        radius=False,
    ):
        if edit:
            self.radius_mode_transitions.append(bool(radiusAdjustMode))
            return None
        if query and radius:
            return 3.25
        raise AssertionError("Unexpected context command")

    def floatSliderGrp(self, name, exists=False, edit=False, value=None):
        assert name == runtime._RADIUS_CONTROL_NAME
        if exists:
            return True
        if edit:
            self.radius_control_values.append(float(value))
            return None
        raise AssertionError("Unexpected Radius control command")


def _exercise_event_routing():
    forwarded = []
    viewport_focused = [False]
    context_current = [True]
    original_context_is_current = runtime._context_is_current
    original_viewport_has_focus = runtime._viewport_has_keyboard_focus
    original_set_radius_adjust_mode = runtime.set_radius_adjust_mode
    try:
        runtime._context_is_current = lambda: context_current[0]
        runtime._viewport_has_keyboard_focus = lambda _watched=None: viewport_focused[0]
        runtime.set_radius_adjust_mode = lambda enabled: forwarded.append(bool(enabled))

        event_filter = runtime._BrushRadiusEventFilter()

        # Text input: ShortcutOverride, press, and release all pass through.
        shortcut = _FakeKeyEvent(QtCore.QEvent.ShortcutOverride)
        key_press = _FakeKeyEvent(QtCore.QEvent.KeyPress)
        key_release = _FakeKeyEvent(QtCore.QEvent.KeyRelease)
        assert not event_filter.eventFilter(None, shortcut)
        assert not event_filter.eventFilter(None, key_press)
        assert not event_filter.eventFilter(None, key_release)
        assert not shortcut.accepted and not key_press.accepted
        assert forwarded == []

        # Viewport: the first ShortcutOverride establishes held mode. A later
        # KeyPress is consumed but does not forward a duplicate transition.
        viewport_focused[0] = True
        shortcut = _FakeKeyEvent(QtCore.QEvent.ShortcutOverride)
        key_press = _FakeKeyEvent(QtCore.QEvent.KeyPress)
        assert event_filter.eventFilter(None, shortcut)
        assert shortcut.accepted
        assert forwarded == [True]
        assert event_filter.eventFilter(None, key_press)
        assert forwarded == [True]

        # Holding B beyond the OS keyboard-repeat delay can produce repeated
        # release/press pairs whose release is not a physical key-up. None of
        # these events may end or restart the active radius gesture.
        for _ in range(100):
            repeat_release = _FakeKeyEvent(
                QtCore.QEvent.KeyRelease,
                auto_repeat=True,
            )
            repeat_press = _FakeKeyEvent(
                QtCore.QEvent.KeyPress,
                auto_repeat=True,
            )
            assert event_filter.eventFilter(None, repeat_release)
            assert repeat_release.accepted
            assert event_filter.eventFilter(None, repeat_press)
            assert repeat_press.accepted
            assert event_filter._b_held
            assert forwarded == [True]

        # Only a real physical release closes the gesture, even if focus left
        # the Viewport while B remained held.
        viewport_focused[0] = False
        key_release = _FakeKeyEvent(QtCore.QEvent.KeyRelease)
        assert event_filter.eventFilter(None, key_release)
        assert key_release.accepted
        assert forwarded == [True, False]

        context_current[0] = False
        assert not event_filter.eventFilter(
            None, _FakeKeyEvent(QtCore.QEvent.KeyPress)
        )
        assert forwarded == [True, False]
    finally:
        runtime._context_is_current = original_context_is_current
        runtime._viewport_has_keyboard_focus = original_viewport_has_focus
        runtime.set_radius_adjust_mode = original_set_radius_adjust_mode


def _exercise_focus_detection():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    line_edit = QtWidgets.QLineEdit()
    plain_text = QtWidgets.QPlainTextEdit()
    spin_box = QtWidgets.QDoubleSpinBox()
    ordinary_widget = QtWidgets.QWidget()
    assert runtime._widget_accepts_text_input(line_edit)
    assert runtime._widget_accepts_text_input(plain_text)
    assert runtime._widget_accepts_text_input(spin_box.lineEdit())
    assert not runtime._widget_accepts_text_input(ordinary_widget)
    del app


def _exercise_tool_settings_sync():
    fake_cmds = _FakeCmds()
    original_cmds = runtime.cmds
    try:
        runtime.cmds = fake_cmds
        runtime.set_radius_adjust_mode(True)
        assert fake_cmds.radius_mode_transitions == [True]
        assert fake_cmds.radius_control_values == []
        runtime.set_radius_adjust_mode(False)
        assert fake_cmds.radius_mode_transitions == [True, False]
        assert fake_cmds.radius_control_values == [3.25]
    finally:
        runtime.cmds = original_cmds


def _exercise_filter_lifecycle():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    original_application = runtime._application
    original_set_radius_adjust_mode = runtime.set_radius_adjust_mode
    try:
        runtime._application = lambda: app
        runtime.set_radius_adjust_mode = lambda _enabled: None
        for _ in range(10):
            assert runtime.on_context_activated() == 1
            assert runtime.filter_count() == 1
            assert runtime.on_context_deactivated() == 0
            assert runtime.filter_count() == 0
    finally:
        runtime.on_context_deactivated()
        runtime._application = original_application
        runtime.set_radius_adjust_mode = original_set_radius_adjust_mode


def main():
    _exercise_event_routing()
    _exercise_focus_detection()
    _exercise_tool_settings_sync()
    _exercise_filter_lifecycle()
    print("DirectionalRetopo B-key event filter: PASS")


if __name__ == "__main__":
    main()
