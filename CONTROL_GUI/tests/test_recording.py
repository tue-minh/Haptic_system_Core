"""Recording format tests (Phase 5, 7.4).

Verifies the master CSV schema and - critically - that the per-sensor files are
byte-compatible with the offline analyser ``plot_paxini_data.py``.
"""
from __future__ import annotations

import os
import sys
import tempfile
import time
import unittest

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from unified_gui import recording as rec  # noqa: E402
import paxini_usb_core as core  # noqa: E402
import plot_paxini_data as offline  # noqa: E402


class FakeSnapshot:
    def __init__(self, connected=True):
        self.connected = connected
        self.x, self.y, self.z = 1.0, 2.0, 3.0
        self.servo_pos = 55
        self.servo_volt_v = 12.1


class FakeLink:
    """Stand-in for SensorLink with the small surface the recorder uses."""

    def __init__(self, tag: str, port: str, frame_force: float = 1.5):
        self.tag = tag
        self.port = port
        self.n_points = 52
        self.model = "S2015_elite"
        self.state = "streaming"
        self.recorder = None
        self._frame = frame_force

    def raw_row(self):
        t = core.now()
        base = [t, 0.5, -0.25, self._frame, 1.2, 0.9, 0.6,
                0.1, 2.0, 30.0, 45.0, 7.0, self._frame, 4.0]
        return tuple(base)

    def raw_taxels(self):
        tax = np.zeros((52, 3), np.float32)
        tax[:, 2] = 0.05
        tax[7, 2] = self._frame
        return tax

    def attach_recorder(self, recorder):
        self.recorder = recorder

    def detach_recorder(self):
        recorder, self.recorder = self.recorder, None
        return recorder

    def pump(self, n: int = 1):
        if self.recorder is not None:
            for _ in range(n):
                self.recorder.push(self.raw_row(), time.time(), self.raw_taxels())


class RecordingFormatTests(unittest.TestCase):
    def test_master_schema_is_complete(self):
        cols = rec.master_columns()
        self.assertEqual(len(cols), 7 + 2 * (13 + 156))
        self.assertEqual(cols[:7], rec.LEAD_COLUMNS)
        self.assertIn("L_Fz", cols)
        self.assertIn("R_Fz", cols)
        self.assertEqual(cols.count("L_t52_Fz"), 1)
        self.assertEqual(cols.count("R_t1_Fx"), 1)

    def test_master_and_offline_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            link_l = FakeLink("L", "COM8")
            link_r = FakeLink("R", "COM13", frame_force=2.5)
            manager = rec.RecordingManager(lambda _m: None, tmp)

            started = manager.start(
                {"L": link_l, "R": link_r},
                motor_snapshot=lambda: FakeSnapshot(),
                rate_hz=120.0,
                motor_port="COM9",
            )
            self.assertTrue(started)

            deadline = time.time() + 1.2
            while time.time() < deadline:
                link_l.pump(20)
                link_r.pump(20)
                time.sleep(0.05)

            summary = manager.stop()
            self.assertGreater(summary["rows"], 0)
            self.assertGreater(summary["sensor_rows"], 0)

            # --- master file -------------------------------------------------
            master = os.path.join(tmp, f"data_record_{manager.stamp}.csv")
            self.assertTrue(os.path.isfile(master))
            with open(master, "r", encoding="utf-8") as fh:
                lines = [ln.rstrip("\n") for ln in fh]
            self.assertTrue(lines[0].startswith("# unified recording"))
            self.assertEqual(lines[1].split(","), rec.master_columns())
            data = np.genfromtxt(lines[2:], delimiter=",")
            self.assertEqual(data.shape[1], len(rec.master_columns()))
            self.assertGreater(data.shape[0], 10)
            self.assertTrue(np.all(np.diff(data[:, 0]) > 0))  # t_rel increasing
            self.assertTrue(np.all(np.isfinite(data[:, 0])))

            # --- per-sensor offline files -----------------------------------
            left = os.path.join(tmp, f"paxini_{manager.stamp}_L_COM8.csv")
            right = os.path.join(tmp, f"paxini_{manager.stamp}_R_COM13.csv")
            self.assertTrue(os.path.isfile(left))
            self.assertTrue(os.path.isfile(right))

            # plot_paxini_data auto-discovery must find this session
            sessions = offline.find_paxini_sessions([tmp])
            self.assertIn(manager.stamp, sessions)
            self.assertEqual(set(sessions[manager.stamp]), {"L", "R"})

            # ...and SensorDataset must parse it exactly as the GUI wrote it
            ds = offline.SensorDataset(left)
            self.assertEqual(ds.tag, "L")
            self.assertEqual(ds.port, "COM8")
            self.assertEqual(ds.taxels.shape[1:], (52, 3))
            self.assertGreater(ds.num_samples, 10)
            self.assertGreater(ds.sampling_rate, 0.0)
            self.assertAlmostEqual(float(ds.peak_fz_overall), 1.5, places=5)

            ds_r = offline.SensorDataset(right)
            self.assertEqual(ds_r.tag, "R")
            self.assertAlmostEqual(float(ds_r.peak_fz_overall), 2.5, places=5)

    def test_start_without_streaming_sensor_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            link = FakeLink("L", "COM8")
            link.state = "error"
            manager = rec.RecordingManager(lambda _m: None, tmp)
            self.assertFalse(manager.start({"L": link}, motor_snapshot=lambda: FakeSnapshot()))


if __name__ == "__main__":
    unittest.main()
