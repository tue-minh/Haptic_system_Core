"""MotorWorker tests (Phase 3, 7.1, 7.2) using an in-memory fake ACSI client."""
from __future__ import annotations

import os
import sys
import time
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from unified_gui.motor import (  # noqa: E402
    ACCEL_MAX,
    ACCEL_MIN,
    VEL_MAX,
    VEL_MIN,
    MotorWorker,
    clamp,
)

from tue_acsi import AcsiTimeout  # noqa: E402


class FakeAcsiClient:
    """Records the calls it receives and answers like a healthy board."""

    def __init__(self, *, enabled: bool = False, fail_polls: int = 0):
        self.calls = []
        self.closed = False
        self._enabled = enabled
        self._fail_polls = fail_polls
        self._pos = {"x": 1.5, "y": -2.0, "z": 3.25}

    # -- SYS / STEP / SERVO ------------------------------------------------
    def sys_ping(self):
        return 1

    def sys_info(self):
        return {"board": "MM_Control", "fw": "1.0"}

    def step_info(self):
        return {"xy_steps_per_mm": 320, "z_steps_per_mm": 1600,
                "min_rate": 10, "max_rate": 20000}

    def servo_ping(self, sid=1):
        return 1

    def step_position(self):
        if self._fail_polls > 0:
            self._fail_polls -= 1
            raise AcsiTimeout("no response")
        return dict(self._pos)

    def step_status(self):
        return {"busy": 0, "enabled": 1 if self._enabled else 0}

    def servo_feedback(self, sid=1):
        return {"pos": 55, "spd": 0, "load": 12, "volt": 121, "temp": 31,
                "status": 0, "moving": 0, "cur": 0}

    def step_enable(self, on):
        self._enabled = bool(on)
        self.calls.append(("step_enable", bool(on)))

    def step_jog_x(self, vx, accel):
        self.calls.append(("step_jog_x", vx, accel))

    def step_jog_y(self, vy, accel):
        self.calls.append(("step_jog_y", vy, accel))

    def step_jog_z(self, vz, accel):
        self.calls.append(("step_jog_z", vz, accel))

    def step_stop(self):
        self.calls.append(("step_stop",))

    def step_abort(self):
        self.calls.append(("step_abort",))

    def servo_set_torque(self, on, sid=1):
        self.calls.append(("servo_set_torque", bool(on)))

    def servo_set_pos(self, pos, sid=1, speed=None, accel=None):
        self.calls.append(("servo_set_pos", int(pos), speed, accel))

    def close(self):
        self.closed = True


def wait_for(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class MotorWorkerTests(unittest.TestCase):
    def _worker(self, **kw):
        client = FakeAcsiClient(**kw)
        worker = MotorWorker("COM_TEST", client_factory=lambda *a, **k: client, poll_hz=50.0)
        worker.start()
        wait_for(lambda: worker.snapshot().connected)
        return worker, client

    def test_handshake_reports_board_and_geometry(self):
        worker, _client = self._worker()
        try:
            snap = worker.snapshot()
            self.assertTrue(snap.connected)
            self.assertEqual(snap.board, "MM_Control")
            self.assertEqual(snap.firmware, "1.0")
            self.assertEqual(snap.steps_per_mm_xy, 320)
            self.assertEqual(snap.steps_per_mm_z, 1600)
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_telemetry_poll_updates_snapshot(self):
        worker, _client = self._worker()
        try:
            self.assertTrue(wait_for(lambda: worker.snapshot().x == 1.5))
            snap = worker.snapshot()
            self.assertEqual(snap.y, -2.0)
            self.assertEqual(snap.z, 3.25)
            self.assertAlmostEqual(snap.servo_volt_v, 12.1, places=3)
            self.assertGreater(snap.poll_hz, 0.0)
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_enable_then_jog_is_forwarded(self):
        worker, client = self._worker()
        try:
            worker.enable(True)
            self.assertTrue(wait_for(lambda: worker.snapshot().enabled))
            worker.jog("x", 25.0, 300.0)
            self.assertTrue(wait_for(lambda: any(c[0] == "step_jog_x" for c in client.calls)))
            jog = [c for c in client.calls if c[0] == "step_jog_x"][-1]
            self.assertAlmostEqual(jog[1], 25.0)
            self.assertAlmostEqual(jog[2], 300.0)
            worker.jog_stop()
            self.assertTrue(wait_for(lambda: any(c[0] == "step_stop" for c in client.calls)))
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_jog_blocked_when_drivers_disabled(self):
        worker, client = self._worker(enabled=False)
        try:
            worker.jog("y", 10.0, 300.0)
            time.sleep(0.3)
            self.assertFalse(any(c[0] == "step_jog_y" for c in client.calls))
            self.assertIn("disabled", worker.snapshot().last_warning)
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_abort_is_urgent(self):
        worker, client = self._worker()
        try:
            worker.abort()
            self.assertTrue(wait_for(lambda: any(c[0] == "step_abort" for c in client.calls)))
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_servo_move_enables_torque_first(self):
        worker, client = self._worker()
        try:
            worker.servo_move(420, 800, 30)
            self.assertTrue(wait_for(lambda: any(c[0] == "servo_set_pos" for c in client.calls)))
            names = [c[0] for c in client.calls]
            self.assertIn("servo_set_torque", names)
            self.assertLess(names.index("servo_set_torque"), names.index("servo_set_pos"))
            move = [c for c in client.calls if c[0] == "servo_set_pos"][-1]
            self.assertEqual(move[1], 420)
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_repeated_failures_disconnect_worker(self):
        worker, _client = self._worker(fail_polls=100)
        try:
            self.assertTrue(wait_for(lambda: not worker.snapshot().connected, timeout=5))
            self.assertTrue(wait_for(lambda: not worker.is_alive(), timeout=5))
            self.assertIn("lost contact", worker.snapshot().error)
        finally:
            worker.shutdown()
            worker.join(timeout=2)

    def test_clamp_helper(self):
        self.assertEqual(clamp(0.0, VEL_MIN, VEL_MAX), VEL_MIN)
        self.assertEqual(clamp(1e6, ACCEL_MIN, ACCEL_MAX), ACCEL_MAX)
        self.assertEqual(clamp(25.0, VEL_MIN, VEL_MAX), 25.0)


if __name__ == "__main__":
    unittest.main()
