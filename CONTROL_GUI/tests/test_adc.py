"""SPI ADC feedback tests: protocol codec, panel rendering, worker polling."""
from __future__ import annotations

import os
import sys
import time
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from unified_gui import _paths  # noqa: F401,E402  (installs tue_usb on sys.path)
from tue_usb import SpiAdcCtrlStatus, SpiAdcFeedback  # noqa: E402


def wait_for(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class CodecTests(unittest.TestCase):
    def test_all_channel_packing_round_trip(self):
        from tue_usb import Decoders

        payload = bytes.fromhex("23614589c7abef3d")
        channels, status = Decoders.sensor_adc_all(payload)
        self.assertEqual(channels, (0x123, 0x456, 0x789, 0xABC, 0xDEF))
        self.assertEqual(status, 0x3)

    def test_feedback_millivolts(self):
        fb = SpiAdcFeedback(channels=(0, 2048, 4095, 100, 200), seq=1,
                            status=0x01, rx_ok_cnt=2, crc_err_cnt=0)
        mv = fb.millivolts(3300)
        self.assertAlmostEqual(mv[1], 2048 * 3300.0 / 4095.0, places=3)
        self.assertAlmostEqual(mv[2], 3300.0, places=3)
        self.assertTrue(fb.adc_valid)
        self.assertFalse(fb.adc_error)


class AdcPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from PyQt6 import QtWidgets
        except Exception as exc:  # pragma: no cover
            raise unittest.SkipTest(f"PyQt6 unavailable: {exc}")
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def _snapshot(self, **kw):
        from unified_gui.motor import MotorSnapshot

        base = dict(
            connected=True, adc_available=True, adc_updated=True,
            adc_channels=(100, 200, 300, 400, 4095),
            adc_seq=7, adc_status=0x01, adc_valid=True, adc_error=False,
            adc_rx_ok=9, adc_crc_err=1, adc_hz=10.0,
            adc_enabled=True, adc_bus_id=1, adc_cs_idx=0, adc_period_ms=10,
        )
        base.update(kw)
        return MotorSnapshot(**base)

    def test_panel_renders_channels_and_status(self):
        from unified_gui.panels.adc import AdcPanel

        panel = AdcPanel()
        panel.update_telemetry(self._snapshot())
        self.assertEqual(panel.raw_labels[0].text().strip(), "100")
        self.assertEqual(panel.raw_labels[4].text().strip(), "4095")
        self.assertIn("mV", panel.mv_labels[1].text())
        self.assertEqual(panel.bars[4].value(), 4095)
        self.assertIn("seq   7", panel.lbl_status.text())
        self.assertIn("crc-err 1", panel.lbl_status.text())
        self.assertTrue(panel.btn_apply.isEnabled())

    def test_apply_emits_selected_configuration(self):
        from unified_gui.panels.adc import AdcPanel

        panel = AdcPanel()
        panel.update_telemetry(self._snapshot())
        fired = []
        panel.apply_requested.connect(lambda *a: fired.append(a))
        panel.chk_enable.setChecked(False)
        panel.cbo_cs.setCurrentIndex(panel.cbo_cs.findData(1))
        panel.cbo_period.setCurrentIndex(panel.cbo_period.findData(20))
        panel.btn_apply.click()
        self.assertEqual(fired, [(False, 1, 1, 20)])

    def test_unavailable_panel_clears(self):
        from unified_gui.panels.adc import AdcPanel

        panel = AdcPanel()
        panel.update_telemetry(self._snapshot(adc_available=False,
                                              adc_last_error="no firmware support"))
        self.assertEqual(panel.raw_labels[0].text(), "---")
        self.assertIn("no firmware support", panel.lbl_footer.text())
        self.assertFalse(panel.btn_read.isEnabled())


class _AdcFakeClient:
    """Minimal board stand-in that also answers the SPI ADC commands."""

    def __init__(self):
        self.calls = []
        self._cfg = SpiAdcCtrlStatus(enabled=True, bus_id=1, cs_idx=0,
                                     period_ms=10, total_frames=0)

    def ping(self, token=0):
        return token

    def get_info(self):
        return SimpleNamespace(hw_id=1, hw_rev=1, fw_major=1, fw_minor=0,
                               build_flags=0, version_string="1.0")

    def step_get_config(self):
        return SimpleNamespace(xy_steps_per_mm=320, z_steps_per_mm=1600,
                               min_rate_steps_s=10, max_rate_steps_s=20000)

    def step_get_pos_xy(self):
        return (0.0, 0.0)

    def step_get_pos_z(self):
        return 0.0

    def step_get_status(self):
        return SimpleNamespace(is_busy=False, is_enabled=True, queue_depth=0)

    def get_spi_adc_status(self):
        return self._cfg

    def configure_spi_adc(self, enable=True, bus_id=1, cs_idx=0, period_ms=10):
        self.calls.append(("configure_spi_adc", enable, bus_id, cs_idx, period_ms))
        self._cfg = SpiAdcCtrlStatus(enabled=bool(enable), bus_id=bus_id,
                                     cs_idx=cs_idx, period_ms=period_ms,
                                     total_frames=self._cfg.total_frames)
        return self._cfg

    def read_spi_adc(self, cs_idx=None):
        self.calls.append(("read_spi_adc", cs_idx))
        self._cfg.total_frames += 1
        return SpiAdcFeedback(channels=(10, 20, 30, 40, 50), seq=3,
                              status=0x01, rx_ok_cnt=5, crc_err_cnt=0)

    def disconnect(self):
        pass


class MotorWorkerAdcTests(unittest.TestCase):
    def test_handshake_polls_and_configures_adc(self):
        from unified_gui.motor import MotorWorker

        client = _AdcFakeClient()
        worker = MotorWorker("COM_ADC", client_factory=lambda *a, **k: client,
                             poll_hz=50.0)
        worker.start()
        try:
            self.assertTrue(wait_for(lambda: worker.snapshot().adc_updated))
            snap = worker.snapshot()
            self.assertTrue(snap.adc_available)
            self.assertEqual(tuple(snap.adc_channels), (10, 20, 30, 40, 50))
            self.assertEqual(snap.adc_seq, 3)

            worker.configure_spi_adc(True, 1, 1, 20)
            self.assertTrue(wait_for(
                lambda: worker.snapshot().adc_bus_id == 1
                and worker.snapshot().adc_cs_idx == 1
                and worker.snapshot().adc_period_ms == 20))
            self.assertIn(("configure_spi_adc", True, 1, 1, 20), client.calls)

            worker.read_spi_adc()
            self.assertTrue(wait_for(
                lambda: sum(1 for c in client.calls if c[0] == "read_spi_adc") > 1))
        finally:
            worker.shutdown()
            worker.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
