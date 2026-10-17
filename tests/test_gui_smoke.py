"""Offscreen GUI smoke test (Phase 6.1, 2.1) - builds the whole window headless.

Run directly or via ``python -m unittest discover -s tests -t .``.
"""
from __future__ import annotations

import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


class FakePort:
    def __init__(self, device, description=""):
        self.device = device
        self.description = description


class GuiSmokeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            from PyQt6 import QtWidgets
        except Exception as exc:  # pragma: no cover
            raise unittest.SkipTest(f"PyQt6 unavailable: {exc}")

        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        from unified_gui import theme

        cls.app.setStyleSheet(theme.STYLESHEET)

    def test_dependency_report_is_ready(self):
        from unified_gui.deps import verify

        report = verify()
        self.assertTrue(report.ok, msg=str(report.missing))
        self.assertTrue(report.qtagg_ok)

    def test_main_window_builds_and_ticks(self):
        from unified_gui.main_window import MainWindow

        window = MainWindow()
        try:
            # force a few refresh cycles without any hardware attached
            for _ in range(25):
                window._on_tick()
            self.assertIsNotNone(window.connection)
            self.assertIsNotNone(window.sensor_monitor)
            self.assertEqual(window.links, {})
            self.assertFalse(window.recorder.active)
            self.assertIsNone(window.haptic)
            # panes exist for both fingers even before connection
            self.assertEqual(set(window.taxel_map.maps), {"L", "R"})
            self.assertEqual(set(window.view3d._panes), {"L", "R"})
            # taxel map, 3D cloud and the 5-valve haptic map share the tabs
            self.assertEqual(window.surface_tabs.count(), 3)
            self.assertIsNotNone(window.valve_map)
            # the live map is the default view; the CPU-rendered 3D sits behind it
            self.assertIs(window.surface_tabs.currentWidget(), window.taxel_map)
            window.stepper.release_jogs()  # watchdog no-op when idle
        finally:
            window.close()

    def test_haptic_role_is_known(self):
        from unified_gui.panels.connection import DEFAULT_BAUD, ROLES, ROLE_LABELS

        self.assertIn("haptic", ROLES)
        self.assertEqual(DEFAULT_BAUD["haptic"], 1_000_000)
        self.assertIn("Haptic", ROLE_LABELS["haptic"])

    def test_valve_map_plots_five_valves(self):
        from unified_gui.haptic import HapticSnapshot, TueBinaryEncoder
        from unified_gui.valvemap import ValveMapView

        view = ValveMapView()
        self.assertEqual(len(view.scatter.data), 5)   # one point per valve

        payload = bytes([20, 20, 10, 255, 0, 0, 0x03, 0])
        snap = HapticSnapshot.from_payload(payload, 1.0, 0.25,
                                           frame=TueBinaryEncoder().encode(payload))
        view.update_snapshot(snap)
        self.assertEqual(len(view.scatter.data), 5)
        self.assertEqual(view.table.item(0, 0).text(), "A")
        self.assertEqual(view.table.item(3, 0).text(), "D")
        self.assertEqual(view.table.item(0, 2).text().strip(), "1.00")   # A: shear*1.0
        self.assertEqual(view.table.item(3, 2).text().strip(), "2.00")   # D: Fz_Left
        self.assertIn("AA 55", view.lbl_frame.text())

        view.update_snapshot(None)                    # offline -> dashes
        self.assertEqual(view.table.item(2, 2).text(), "---")

    def test_sensor_baud_matches_the_reference_reader(self):
        import paxini_usb_core as core
        from unified_gui.sensors import SENSOR_BAUDRATE
        from unified_gui.panels.connection import DEFAULT_BAUD

        # a mismatch here is exactly what stops the board answering at all
        self.assertEqual(SENSOR_BAUDRATE, core.BAUDRATE)
        self.assertEqual(DEFAULT_BAUD["sensor_l"], core.BAUDRATE)
        self.assertEqual(DEFAULT_BAUD["sensor_r"], core.BAUDRATE)

    def test_connection_panel_port_collision_guard(self):
        from unified_gui.panels.connection import ConnectionPanel, ROLES

        panel = ConnectionPanel()
        panel._ports = [FakePort("COM4", "Motor board"), FakePort("COM8", "Sensor L")]
        for role in ROLES:
            panel._fill_combo(panel.rows[role].combo)

        # assign the same device to two roles and mark one as connected
        panel.rows["motor"].combo.setCurrentIndex(
            panel.rows["motor"].combo.findData("COM4"))
        panel.rows["sensor_l"].combo.setCurrentIndex(
            panel.rows["sensor_l"].combo.findData("COM4"))
        panel.rows["sensor_r"].combo.setCurrentIndex(
            panel.rows["sensor_r"].combo.findData("COM8"))
        panel.set_state("motor", "ready", "streaming")

        self.assertEqual(panel.collision("sensor_l"), "motor")
        self.assertIsNone(panel.collision("sensor_r"))

        # a disconnected role does not block reuse
        panel.set_state("motor", "off", "-")
        self.assertIsNone(panel.collision("sensor_l"))

    def test_port_auto_assignment_spreads_ports_over_roles(self):
        from unified_gui.panels.connection import ConnectionPanel, ROLES

        panel = ConnectionPanel()
        panel._ports = [FakePort("COM4"), FakePort("COM8"), FakePort("COM13")]
        for role in ROLES:
            panel._fill_combo(panel.rows[role].combo)
        panel._auto_assign()
        self.assertEqual(panel.selected_port("motor"), "COM4")
        self.assertEqual(panel.selected_port("sensor_l"), "COM8")
        self.assertEqual(panel.selected_port("sensor_r"), "COM13")

        # an already-connected role keeps its port when ports are rescanned
        panel.set_state("motor", "ready", "streaming")
        panel.rows["motor"].combo.setCurrentIndex(
            panel.rows["motor"].combo.findData("COM4"))
        panel._auto_assign()
        self.assertEqual(panel.selected_port("motor"), "COM4")

    def test_taxel_map_plots_taxels_and_contact_point(self):
        import numpy as np
        from unified_gui.sensors import SensorReading
        from unified_gui.taxelmap import TaxelMapView, _lookup_table

        tax = np.zeros((52, 3), np.float32)
        tax[:, 2] = 0.1
        tax[7, 2] = 1.8
        reading = SensorReading(
            t=0.0, epoch=0.0, fx=0.3, fy=-0.2, fz=1.8, f_mag=1.9, f_shear=0.36,
            mu=0.2, cop_x=1.5, cop_y=6.2, shear_dir=-33.0, tilt=20.0,
            peak_taxel=8.0, peak_fz=1.8, contact_taxels=9.0, taxels=tax,
        )

        view = TaxelMapView({})
        pane = view.maps["L"]
        self.assertEqual(len(pane.scatter.data), 52)   # all taxels placed

        view.update_readings({"L": reading})
        self.assertEqual(len(pane.cop.data), 1)        # contact point shown
        brushes = pane.scatter.data["brush"]           # one brush per taxel
        self.assertEqual(len(brushes), 52)
        # the loaded taxel must actually be coloured, not left at the base LUT entry
        loaded = pane.scatter.data[7]["brush"].color().name()
        idle = pane.scatter.data[0]["brush"].color().name()
        self.assertNotEqual(loaded, idle)

        view.update_readings({"L": None})              # no data -> marker hidden
        self.assertEqual(len(pane.cop.data), 0)

        # any colour scale / unknown colormap must not break the widget
        view.set_full_scale(0.5)
        view.update_readings({"L": reading})
        self.assertEqual(len(_lookup_table("not-a-real-colormap")), 256)
        self.assertEqual(len(_lookup_table("inferno")), 256)

    def test_jog_pad_layout(self):
        """X/Y form a cross (X vertical, Y horizontal); Z is a column."""
        from PyQt6 import QtWidgets
        from unified_gui.panels.motor import JogButton, StepperPanel

        panel = StepperPanel(lambda _m: None)
        grid = None
        for i in range(panel.layout().count()):
            item = panel.layout().itemAt(i)
            if not isinstance(item, QtWidgets.QGridLayout):
                continue
            if any(isinstance(item.itemAt(j).widget(), JogButton)
                   for j in range(item.count())):
                grid = item
                break
        self.assertIsNotNone(grid, "jog grid not found")

        cells = {}
        for j in range(grid.count()):
            widget = grid.itemAt(j).widget()
            if isinstance(widget, JogButton):
                row, col, _rs, _cs = grid.getItemPosition(j)
                cells[(widget.axis, widget.sign)] = (row, col)

        self.assertEqual(cells[("x", -1)], (0, 1))   # X- top centre
        self.assertEqual(cells[("x", +1)], (2, 1))   # X+ bottom centre
        self.assertEqual(cells[("y", +1)], (1, 0))   # Y+ left
        self.assertEqual(cells[("y", -1)], (1, 2))   # Y- right
        self.assertEqual(cells[("z", -1)], (1, 4))   # Z- above Z+
        self.assertEqual(cells[("z", +1)], (2, 4))

        # the orderings themselves, independent of the exact cells
        self.assertLess(cells[("x", -1)][0], cells[("x", +1)][0], "X- must sit above X+")
        self.assertLess(cells[("y", +1)][1], cells[("y", -1)][1], "Y+ must sit left of Y-")
        self.assertLess(cells[("z", -1)][0], cells[("z", +1)][0], "Z- must sit above Z+")

        # X and Y share the cross; the centre of it stays empty
        self.assertNotIn((1, 1), set(cells.values()))

    def test_jog_button_watchdog_releases_on_leave(self):
        from PyQt6 import QtCore
        from PyQt6.QtCore import QPointF, Qt
        from PyQt6.QtGui import QMouseEvent
        from unified_gui.panels.motor import JogButton

        btn = JogButton("X +", "x", 1.0)
        events = []
        btn.jog_pressed.connect(lambda axis, sign: events.append(("press", axis, sign)))
        btn.jog_released.connect(lambda: events.append(("release",)))

        press = QMouseEvent(
            QtCore.QEvent.Type.MouseButtonPress, QPointF(1, 1), QPointF(1, 1),
            Qt.MouseButton.LeftButton, Qt.MouseButton.LeftButton,
            Qt.KeyboardModifier.NoModifier,
        )
        btn.mousePressEvent(press)
        self.assertTrue(btn.is_down())

        # pointer leaves the button while held -> watchdog must release the jog
        btn.leaveEvent(QtCore.QEvent(QtCore.QEvent.Type.Leave))
        self.assertFalse(btn.is_down())
        self.assertEqual(events, [("press", "x", 1.0), ("release",)])


if __name__ == "__main__":
    unittest.main()
