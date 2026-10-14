"""SensorLink tests (Phase 2.3, 4.3) with a fake worker - no hardware required."""
from __future__ import annotations

import os
import sys
import threading
import time
import unittest

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from unified_gui.sensors import SensorLink, STATE_DISCONNECTED, STATE_STREAMING  # noqa: E402
import paxini_usb_core as core  # noqa: E402


class FakeRing:
    def __init__(self):
        self.calls = []

    def window(self, t_from, cols):
        self.calls.append((t_from, list(cols)))
        return np.zeros((0, len(cols)))


class FakeWorker(threading.Thread):
    """Blocks in run() until stopped, mimicking a live streaming worker.

    A real worker detects its bus address on connect; this one pretends the
    board answered on 0x03 when asked to auto-detect.
    """

    def __init__(self, port, log, device_addr=0, model="S2015_elite", baudrate=1_000_000):
        super().__init__(daemon=True)
        self._stop_evt = threading.Event()
        self.port = port
        self.baudrate = baudrate
        self.requested_addr = device_addr
        self.device_addr = int(device_addr) or 0x03
        self.state = "streaming"
        self.error = ""
        self.frames = 30
        self.timeouts = 2
        self.rate_hz = 47.5
        self.n_points = 52
        self.model = model
        self.recorder = None
        self.ring = FakeRing()
        row = [0.0] * len(core.METRIC_COLUMNS)
        row[core.COL["t"]] = 1.0
        row[core.COL["Fx"]] = 0.5
        row[core.COL["Fy"]] = -0.25
        row[core.COL["Fz"]] = 3.0
        row[core.COL["F_mag"]] = 3.1
        row[core.COL["F_shear"]] = 0.6
        row[core.COL["mu"]] = 0.2
        row[core.COL["CoP_x"]] = 0.1
        row[core.COL["CoP_y"]] = 2.0
        row[core.COL["shear_dir"]] = 30.0
        row[core.COL["tilt"]] = 11.0
        row[core.COL["peak_idx"]] = 7.0
        row[core.COL["peak_Fz"]] = 1.4
        row[core.COL["contact_n"]] = 4.0
        self.latest_row = tuple(row)
        self.latest_taxels = np.ones((52, 3), np.float32) * 0.2
        self.tare_requests = []

    def run(self):
        self._stop_evt.wait()

    def stop(self):
        self._stop_evt.set()

    def request_tare(self, n=50):
        self.tare_requests.append(n)

    def reset_tare(self):
        self.tare_requests.append(0)


class FakeSerial:
    """Serial stand-in that only answers the calibration frame of one address.

    ``answer_after`` makes the board stay silent for the first N frames, which
    is how a still-booting board behaves.
    """

    def __init__(self, answer_addr=None, answer_after=0, response_len=15):
        self.answer_addr = answer_addr
        self.answer_after = answer_after
        self.response_len = response_len
        self.written = []
        self._rx = b""

    def reset_input_buffer(self):
        self._rx = b""

    def write(self, data):
        self.written.append(bytes(data))
        addr = data[4] if len(data) > 4 else None
        if (self.answer_addr is not None
                and addr == self.answer_addr
                and len(self.written) > self.answer_after):
            self._rx = core.RESP_HEAD + bytes(self.response_len - 2)
        return len(data)

    @property
    def in_waiting(self):
        return len(self._rx)

    def read(self, size=1):
        chunk, self._rx = self._rx[:size], self._rx[size:]
        return chunk


class DeviceAddressProbeTests(unittest.TestCase):
    """Auto-detection of the sensor bus address (Phase 2.3)."""

    def test_calibration_frame_targets_the_address_and_carries_an_lrc(self):
        for addr in (0x01, 0x05, 0x08):
            frame = core.build_calibration_frame(addr)
            self.assertEqual(len(frame), 15)
            self.assertEqual(frame[:2], core.REQ_HEAD)
            self.assertEqual(frame[4], addr)
            self.assertEqual(frame[-1], core.calc_usb_lrc(frame[:-1]))

    def test_finds_the_address_that_answers(self):
        ser = FakeSerial(answer_addr=0x03)
        self.assertEqual(core.probe_device_address(ser, passes=1, timeout=0.02), 0x03)

    def test_sweeps_every_candidate_address(self):
        ser = FakeSerial(answer_addr=None)
        self.assertIsNone(core.probe_device_address(ser, passes=1, timeout=0.01))
        self.assertEqual([f[4] for f in ser.written], list(core.PROBE_ADDRESSES))

    def test_returns_none_when_no_board_answers(self):
        ser = FakeSerial(answer_addr=None)
        self.assertIsNone(core.probe_device_address(ser, passes=1, timeout=0.01))

    def test_preferred_address_is_probed_first(self):
        ser = FakeSerial(answer_addr=0x05)
        self.assertEqual(
            core.probe_device_address(ser, preferred=0x05, passes=1, timeout=0.02), 0x05)
        self.assertEqual(ser.written[0][4], 0x05)
        # the rest of the range is still swept when the preference is wrong
        self.assertEqual(len(ser.written), 1)

    def test_stale_preference_still_finds_the_board(self):
        ser = FakeSerial(answer_addr=0x04)
        self.assertEqual(
            core.probe_device_address(ser, preferred=0x07, passes=1, timeout=0.02), 0x04)

    def test_resweeps_for_a_board_that_boots_late(self):
        late = FakeSerial(answer_addr=0x08, answer_after=8)
        self.assertIsNone(core.probe_device_address(late, passes=1, timeout=0.01))
        fresh = FakeSerial(answer_addr=0x08, answer_after=8)
        self.assertEqual(core.probe_device_address(fresh, passes=2, timeout=0.02), 0x08)

    def test_probe_logs_the_address_it_found(self):
        lines = []
        core.probe_device_address(FakeSerial(answer_addr=0x02), lines.append,
                                  passes=1, timeout=0.02)
        self.assertTrue(any("02" in line and "answered" in line for line in lines))


class SensorLinkTests(unittest.TestCase):
    def _link(self, tag="L", port="COM8", device_addr=0):
        holder = {}
        log = []
        link = SensorLink(tag, port, log.append, device_addr=device_addr,
                          worker_factory=lambda *a, **k: holder.setdefault("w", FakeWorker(*a, **k)))
        link.connect()
        return link, holder["w"], log

    def test_connect_creates_streaming_worker(self):
        link, worker, _log = self._link()
        try:
            self.assertTrue(link.is_active())
            self.assertEqual(link.state, STATE_STREAMING)
            # must match what paxini_usb_core opens the port with, or the board
            # never answers at all
            self.assertEqual(worker.baudrate, core.BAUDRATE)
            self.assertEqual(link.label, "Left")
            self.assertEqual(link.n_points, 52)
        finally:
            link.disconnect(wait=True)
        self.assertEqual(link.state, STATE_DISCONNECTED)

    def test_reading_snapshot(self):
        link, worker, _log = self._link()
        try:
            reading = link.reading()
            self.assertIsNotNone(reading)
            self.assertAlmostEqual(reading.fz, 3.0)
            self.assertEqual(reading.taxels.shape, (52, 3))
            # snapshots must be copies, not aliases of the worker buffer
            reading.taxels[0, 0] = 99.0
            self.assertNotEqual(float(worker.latest_taxels[0, 0]), 99.0)
        finally:
            link.disconnect(wait=True)

    def test_health_metrics(self):
        link, _worker, _log = self._link()
        try:
            health = link.health()
            self.assertEqual(health["frames"], 30.0)
            self.assertEqual(health["drops"], 2.0)
            self.assertAlmostEqual(health["drop_pct"], 100.0 * 2 / 32, places=3)
        finally:
            link.disconnect(wait=True)

    def test_tare_forwarding(self):
        link, worker, _log = self._link()
        try:
            link.request_tare(50)
            link.reset_tare()
            self.assertEqual(worker.tare_requests, [50, 0])
        finally:
            link.disconnect(wait=True)

    def test_disconnected_state_without_worker(self):
        link = SensorLink("R", "COM13", lambda _m: None)
        self.assertEqual(link.state, STATE_DISCONNECTED)
        self.assertFalse(link.is_active())
        self.assertIsNone(link.reading())

    def test_auto_detect_asks_the_core_for_address_zero(self):
        link, worker, log = self._link(device_addr=0)
        try:
            self.assertEqual(worker.requested_addr, 0)
            self.assertEqual(link.addr_text, "0x03")   # what the board answered on
            self.assertEqual(link.detected_addr, 0x03)
            self.assertTrue(any("auto-detect" in line for line in log))
        finally:
            link.disconnect(wait=True)

    def test_explicit_address_is_forwarded(self):
        link, worker, _log = self._link(device_addr=0x05)
        try:
            self.assertEqual(worker.requested_addr, 0x05)
            self.assertEqual(link.detected_addr, 0x05)
            self.assertEqual(link.addr_text, "0x05")
        finally:
            link.disconnect(wait=True)

    def test_addr_text_before_connect(self):
        link = SensorLink("L", "COM8", lambda _m: None)
        self.assertEqual(link.detected_addr, 0)
        self.assertEqual(link.addr_text, "auto")


class FakePort(FakeSerial):
    """FakeSerial that accepts the kwargs the worker passes to serial.Serial."""

    def __init__(self, *args, **kwargs):
        super().__init__(answer_addr=kwargs.pop("_answer_addr", None),
                         answer_after=kwargs.pop("_answer_after", 0))

    def close(self):
        pass


def wait_for(predicate, timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class WorkerDetectionTests(unittest.TestCase):
    """The worker must resolve the address on connect, or say why it could not."""

    def _run_worker(self, port: FakePort):
        import serial as pyserial

        original_serial, original_timeout = pyserial.Serial, core.PROBE_TIMEOUT_S
        pyserial.Serial = lambda *a, **k: port
        core.PROBE_TIMEOUT_S = 0.01          # keep the sweep fast in tests
        try:
            worker = core.UsbSensorWorker("COM_TEST", lambda _m: None)
            worker.start()
            return worker
        finally:
            pyserial.Serial, core.PROBE_TIMEOUT_S = original_serial, original_timeout

    def test_worker_errors_when_no_sensor_answers(self):
        worker = self._run_worker(FakePort(_answer_addr=None))
        try:
            self.assertTrue(wait_for(lambda: not worker.is_alive()))
            self.assertEqual(worker.state, "error")
            self.assertIn("no sensor answered", worker.error)
            self.assertEqual(worker.frames, 0)
        finally:
            worker.stop()
            worker.join(timeout=2)

    def test_worker_uses_the_detected_address(self):
        worker = self._run_worker(FakePort(_answer_addr=0x03))
        try:
            self.assertTrue(wait_for(lambda: worker.state == "streaming"))
            self.assertEqual(worker.device_addr, 0x03)
            self.assertEqual(worker.error, "")
        finally:
            worker.stop()
            worker.join(timeout=2)

    def test_worker_honours_a_pinned_address(self):
        port = FakePort(_answer_addr=0x05)
        import serial as pyserial

        original = pyserial.Serial
        pyserial.Serial = lambda *a, **k: port
        try:
            worker = core.UsbSensorWorker("COM_TEST", lambda _m: None, device_addr=0x05)
            worker.start()
            self.assertTrue(wait_for(lambda: worker.state == "streaming"))
        finally:
            pyserial.Serial = original
        try:
            self.assertEqual(worker.device_addr, 0x05)
            self.assertEqual(port.written[0][4], 0x05)   # pinned address probed first
        finally:
            worker.stop()
            worker.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
