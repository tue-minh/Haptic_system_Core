"""Entry-point tests: each documented launch style must reach QApplication.exec.

Regression guard for the relative-import trap: ``python unified_gui/app.py``
executes the file as a standalone script, so ``from . import theme`` would fail
unless the module re-enters itself through the package (see the ``__package__``
guard at the top of ``unified_gui/app.py``).

Each mode is probed in its own subprocess because a process may only host a
single QApplication, and the event loop is stubbed so the probe returns
immediately.
"""
from __future__ import annotations

import os
import subprocess
import sys
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

PROBE = r'''
import os, runpy, sys
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, r"{repo}")
from PyQt6 import QtWidgets


class StubApp(QtWidgets.QApplication):
    def exec(self):
        print("EXEC_REACHED")
        return 0


QtWidgets.QApplication = StubApp
mode = sys.argv[1]
try:
    if mode == "file":
        runpy.run_path(os.path.join(r"{repo}", "unified_gui", "app.py"), run_name="__main__")
    elif mode == "module":
        runpy.run_module("unified_gui.app", run_name="__main__")
    else:
        runpy.run_path(os.path.join(r"{repo}", "run_unified_gui.py"), run_name="__main__")
except SystemExit as exc:
    if exc.code not in (0, None):
        print("SYSTEMEXIT", exc.code)
        sys.exit(1)

# The probe asserts "the entry point boots", not "Qt shuts down cleanly":
# the event loop never ran, so just close the windows and skip interpreter
# finalisation (which can abort inside Qt on the offscreen platform).
for widget in QtWidgets.QApplication.topLevelWidgets():
    widget.close()
print("PROBE_OK")
sys.stdout.flush()
os._exit(0)
'''


class EntryPointTests(unittest.TestCase):
    def _probe(self, mode: str) -> subprocess.CompletedProcess:
        script = PROBE.format(repo=REPO_ROOT)
        return subprocess.run(
            [sys.executable, "-c", script, mode],
            capture_output=True, text=True, cwd=REPO_ROOT, timeout=180,
        )

    def _assert_mode(self, mode: str) -> None:
        try:
            proc = self._probe(mode)
        except subprocess.TimeoutExpired:
            self.fail(f"entry point {mode!r} did not exit within the timeout")
        self.assertEqual(
            proc.returncode, 0,
            msg=f"entry point {mode!r} failed:\n{proc.stdout}\n{proc.stderr}",
        )
        self.assertIn("EXEC_REACHED", proc.stdout,
                      msg=f"entry point {mode!r} never built the main window:\n{proc.stderr}")
        self.assertIn("PROBE_OK", proc.stdout)

    def test_direct_script_invocation(self):
        """python unified_gui/app.py"""
        self._assert_mode("file")

    def test_module_invocation(self):
        """python -m unified_gui.app"""
        self._assert_mode("module")

    def test_root_launcher(self):
        """python run_unified_gui.py"""
        self._assert_mode("launcher")


if __name__ == "__main__":
    unittest.main()
