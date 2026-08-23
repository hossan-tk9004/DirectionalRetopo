"""Maya-side regression test for stale custom-context recovery."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_PATH = PROJECT_ROOT / "scripts"
if str(SCRIPTS_PATH) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_PATH))

from directional_retopo import tool  # noqa: E402


class _FakeMel:
    @staticmethod
    def eval(_command):
        return None


class _FakeRuntime:
    def __init__(self):
        self.deactivation_count = 0

    def on_context_deactivated(self):
        self.deactivation_count += 1


class _FakeCmds:
    def __init__(self):
        self.context_exists = True
        self.context_stale = True
        self.current_context = tool.CONTEXT_NAME
        self.radius = None
        self.create_count = 0
        self.delete_count = 0

    @staticmethod
    def pluginInfo(_name, query=False, loaded=False, path=False):
        assert query
        if loaded:
            return True
        if path:
            return str(tool.PLUGIN_PATH)
        raise AssertionError("Unexpected pluginInfo query")

    def contextInfo(self, name, exists=False, query=False, c=False):
        assert name == tool.CONTEXT_NAME
        if exists:
            return self.context_exists
        if query and c:
            return tool.CONTEXT_CLASS_NAME
        raise AssertionError("Unexpected contextInfo query")

    def directionalRetopoContext(
        self,
        name,
        edit=False,
        query=False,
        radius=None,
    ):
        assert name == tool.CONTEXT_NAME
        if not edit and not query:
            self.context_exists = True
            self.context_stale = False
            self.create_count += 1
            return name
        if query and radius:
            if self.context_stale:
                raise RuntimeError()
            return self.radius if self.radius is not None else 1.0
        if edit and radius is not None:
            if self.context_stale:
                raise RuntimeError()
            self.radius = radius
            return None
        raise AssertionError("Unexpected directionalRetopoContext call")

    def currentCtx(self):
        return self.current_context

    def setToolTo(self, name):
        self.current_context = name

    def deleteUI(self, name):
        assert name == tool.CONTEXT_NAME
        self.context_exists = False
        self.context_stale = False
        self.delete_count += 1


def main():
    fake_cmds = _FakeCmds()
    fake_runtime = _FakeRuntime()
    original_cmds = tool.cmds
    original_mel = tool.mel
    original_runtime = tool.runtime
    try:
        tool.cmds = fake_cmds
        tool.mel = _FakeMel()
        tool.runtime = fake_runtime

        result = tool.activate(radius=1.25)

        assert result == tool.CONTEXT_NAME
        assert fake_cmds.delete_count == 1
        assert fake_cmds.create_count == 1
        assert fake_cmds.radius == 1.25
        assert fake_cmds.current_context == tool.CONTEXT_NAME
        assert fake_runtime.deactivation_count == 1
    finally:
        tool.cmds = original_cmds
        tool.mel = original_mel
        tool.runtime = original_runtime

    print("DirectionalRetopo stale-context recovery: PASS")


if __name__ == "__main__":
    main()
