"""Haptic synthesis / framing / streaming tests (TODO_SEND_SENSOR_DATA_THROUGH_USB.md).

Covers the three building blocks without hardware:

* ``HapticPayloadSynthesizer`` against the acceptance vectors of TODO §7.1;
* ``TueBinaryEncoder`` frame length, envelope and CRC round-trip;
* ``Stm32HapticStreamer`` against a fake serial port (frame size + rate).
"""
from __future__ import annotations

import os
import sys
import time
import unittest

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import unified_gui.haptic as h  # noqa: E402
from tue_usb import Frame, FrameType, crc8_sae_j1850, is_reserved_can_id  # noqa: E402


def _side(fz=0.0, fy=0.0, fx=0.0, taxels=0.0, **extra):
    d = {"fz": fz, "fy": fy, "fx": fx, "taxels_fz": [taxels] * h.N_TAXELS}
    d.update(extra)
    return d


class CrcTests(unittest.TestCase):
    def test_reference_vector(self):
        self.assertEqual(crc8_sae_j1850(b"123456789"), 0x4B)

    def test_can_id_is_in_the_reserved_user_range(self):
        # 0x300 is why the streamer writes raw frames instead of using
        # TueUsbClient.transact(), which refuses reserved IDs.
        self.assertTrue(is_reserved_can_id(h.CAN_ID))


class SynthesizerTests(unittest.TestCase):
    def setUp(self):
        self.synth = h.HapticPayloadSynthesizer()

    def test_zero_idle(self):
        payload = self.synth.synthesize(_side(), _side(), now=0.0)
        self.assertEqual(len(payload), h.PAYLOAD_LEN)
        self.assertEqual(payload[:7], b"\x00" * 7)
        self.assertEqual(payload[7], 0)  # Frame_Counter

    def test_pinch_only(self):
        # fz_avg = (5.0 + 3.0) / 2 = 4.0 N -> 40 = 0x28 (Valve B)
        payload = self.synth.synthesize(_side(fz=5.0), _side(fz=3.0), now=0.0)
        self.assertEqual(payload[:7], bytes([0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x03]))

    def test_directional_x_and_y_mapping(self):
        # Move +X: Valve C blows; Move +Y: Valve D blows
        # Left: fx=2.0, fy=1.0; Right: fx=-2.0 (inverted!), fy=1.0
        # fx_net = (2.0 - (-2.0)) / 2 = +2.0 -> Valve C = 20, Valve A = 0
        # fy_net = (1.0 + 1.0) / 2 = +1.0 -> Valve D = 10, Valve E = 0
        # fz_avg = 6.0 N -> Valve B = 60
        left_pos = _side(fz=6.0, fx=2.0, fy=1.0)
        right_pos = _side(fz=6.0, fx=-2.0, fy=1.0)
        payload_pos = self.synth.synthesize(left_pos, right_pos, now=0.0)
        self.assertEqual(payload_pos[:7], bytes([0, 60, 20, 10, 0, 0, 0x03]))

        # Move opposite (-X, -Y): Valve A blows, Valve E blows
        # Left: fx=-2.0, fy=-1.0; Right: fx=2.0 (inverted!), fy=-1.0
        # fx_net = (-2.0 - 2.0) / 2 = -2.0 -> Valve A = 20, Valve C = 0
        # fy_net = (-1.0 + -1.0) / 2 = -1.0 -> Valve E = 10, Valve D = 0
        left_neg = _side(fz=6.0, fx=-2.0, fy=-1.0)
        right_neg = _side(fz=6.0, fx=2.0, fy=-1.0)
        payload_neg = self.synth.synthesize(left_neg, right_neg, now=0.0)
        self.assertEqual(payload_neg[:7], bytes([20, 60, 0, 0, 10, 0, 0x03]))

    def test_stale_side_is_zeroed(self):
        stale = _side(fz=5.0, fy=2.0, age_s=0.5)
        fresh = _side(fz=3.0, fy=1.0)
        payload = self.synth.synthesize(stale, fresh, now=0.0)
        self.assertEqual(payload[0], 0)     # Valve A = 0
        self.assertEqual(payload[1], 30)    # Valve B (Fz) = 30 from fresh
        self.assertEqual(payload[2], 0)     # Valve C = 0
        self.assertEqual(payload[3], 10)    # Valve D (+Fy) = 10 from fresh
        self.assertEqual(payload[4], 0)     # Valve E (-Fy) = 0
        self.assertFalse(payload[6] & h.FLAG_LEFT_CONTACT)
        self.assertTrue(payload[6] & h.FLAG_RIGHT_CONTACT)

    def test_missing_keys_and_short_taxels_are_safe(self):
        payload = self.synth.synthesize({}, {}, now=0.0)
        self.assertEqual(payload[:7], b"\x00" * 7)
        short = {"fz": 1.0, "taxels_fz": [1.0, 2.0]}
        long = {"taxels_fz": list(range(100))}
        payload = self.synth.synthesize(short, long, now=0.0)
        self.assertEqual(len(payload), h.PAYLOAD_LEN)

    def test_slip_flag_from_shear_ratio(self):
        # |Fy|/Fz = 2.0/1.0 = 2.0 > mu_s -> slip
        payload = self.synth.synthesize(_side(fz=1.0, fy=2.0), {}, now=0.0)
        self.assertTrue(payload[6] & h.FLAG_SLIP)

    def test_tare_flag(self):
        payload = self.synth.synthesize(_side(), _side(), tare_active=True, now=0.0)
        self.assertTrue(payload[6] & h.FLAG_TARE)

    def test_frame_counter_rolls_over(self):
        counters = [self.synth.synthesize(_side(), _side(), now=0.0)[7] for _ in range(256)]
        self.assertEqual(counters, list(range(256)))
        self.assertEqual(self.synth.synthesize(_side(), _side(), now=0.0)[7], 0)

    def test_rejects_bad_shear_axis(self):
        with self.assertRaises(ValueError):
            h.HapticPayloadSynthesizer(shear_axis="Z")


class EncoderTests(unittest.TestCase):
    def test_frame_envelope_round_trip(self):
        enc = h.TueBinaryEncoder()
        payload = bytes([0x32, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00])
        frame = enc.encode(payload)
        self.assertEqual(len(frame), h.FRAME_LEN)
        self.assertEqual(frame[0], h.SOF1)
        self.assertEqual(frame[1], h.SOF2)
        self.assertEqual(frame[-2], h.EOF1)
        self.assertEqual(frame[-1], h.EOF2)

        decoded = Frame.from_bytes(frame)  # validates the CRC-8
        self.assertEqual(int(decoded.frame_type), int(FrameType.REQ))
        self.assertEqual(int(decoded.can_id), h.CAN_ID)
        self.assertEqual(len(decoded.payload), h.DLC)
        self.assertEqual(decoded.payload, payload)
        self.assertEqual(decoded.seq, 0)

    def test_seq_increments_and_wraps(self):
        enc = h.TueBinaryEncoder()
        payload = b"\x00" * h.DLC
        self.assertEqual(enc.encode(payload)[8], 0)
        self.assertEqual(enc.encode(payload)[8], 1)
        for _ in range(254):
            enc.encode(payload)
        self.assertEqual(enc.encode(payload)[8], 0)

    def test_rejects_wrong_payload_length(self):
        enc = h.TueBinaryEncoder()
        with self.assertRaises(ValueError):
            enc.encode(b"\x00" * 7)


class SnapshotTests(unittest.TestCase):
    def test_valve_forces_from_payload(self):
        # Payload [A=10 (1.0N), B=20 (2.0N), C=10 (1.0N), D=30 (3.0N), E=30 (3.0N), 0, 0x03, 0]
        snap = h.HapticSnapshot.from_payload(bytes([10, 20, 10, 30, 30, 0, 0x03, 0]), 1.0, 0.5)
        forces = {v.name: v.force for v in snap.valves}
        self.assertAlmostEqual(forces["A"], 1.0, places=6)
        self.assertAlmostEqual(forces["B"], 2.0, places=6)
        self.assertAlmostEqual(forces["C"], 1.0, places=6)
        self.assertAlmostEqual(forces["D"], 3.0, places=6)
        self.assertAlmostEqual(forces["E"], 3.0, places=6)
        self.assertEqual(snap.flags, 0x03)
        self.assertEqual(snap.flag_names, ["L-contact", "R-contact"])
        self.assertEqual(snap.frame, b"")  # no wire frame supplied to the model


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


class StreamerTests(unittest.TestCase):
    def _run(self, seconds=0.5, period=0.01, left=None, right=None):
        left = left if left is not None else _side(fz=5.0, taxels=0.5)
        right = right if right is not None else _side(fz=3.0, taxels=0.5)
        fake = FakeSerial()
        orig = h.serial.Serial
        h.serial.Serial = lambda *a, **k: fake
        try:
            streamer = h.Stm32HapticStreamer("COM_TEST", period=period)
            streamer.update_sensors(left, right)
            streamer.start()
            # push fresh snapshots like the 30 Hz GUI tick, or the data ages
            # past stale_s and the synthesizer (correctly) zeroes it
            deadline = time.time() + seconds
            while time.time() < deadline:
                streamer.update_sensors(left, right)
                time.sleep(0.02)
            streamer.shutdown()
            streamer.join(timeout=2)
        finally:
            h.serial.Serial = orig
        return streamer, fake

    def test_writes_fixed_20_byte_frames(self):
        streamer, fake = self._run()
        # wall-clock rate is bounded by the host timer, so assert a floor only
        self.assertGreaterEqual(len(fake.writes), 15)
        self.assertTrue(all(len(w) == h.FRAME_LEN for w in fake.writes))
        self.assertTrue(fake.closed)
        self.assertFalse(streamer.is_alive())

        decoded = Frame.from_bytes(fake.writes[-1])
        self.assertEqual(int(decoded.can_id), h.CAN_ID)
        self.assertEqual(decoded.payload[0], 0)    # Valve A (Fx avg) 0.0 N
        self.assertEqual(decoded.payload[1], 40)   # Valve B (Fz avg) 4.0 N

    def test_snapshot_reports_state_and_rate(self):
        streamer, _fake = self._run()
        snap = streamer.snapshot()
        self.assertEqual(snap.state, "stopped")
        self.assertGreaterEqual(snap.sent, 15)

    def test_open_failure_sets_error_state(self):
        def _boom(*a, **k):
            raise OSError("no such port")

        orig = h.serial.Serial
        h.serial.Serial = _boom
        try:
            streamer = h.Stm32HapticStreamer("COM_MISSING", period=0.005)
            streamer.start()
            streamer.join(timeout=2)
        finally:
            h.serial.Serial = orig
        self.assertEqual(streamer.state, "error")
        self.assertIn("open failed", streamer.error)

    def test_shared_link_uses_the_writer_instead_of_a_port(self):
        """The real topology: haptic frames ride the motor worker's STM32 port."""
        frames = []

        def writer(data):
            frames.append(bytes(data))
            return len(data)

        opened = []
        orig = h.serial.Serial
        h.serial.Serial = lambda *a, **k: opened.append(a) or object()
        left, right = _side(fz=5.0, taxels=0.5), _side(fz=3.0, taxels=0.5)
        try:
            streamer = h.Stm32HapticStreamer("COM_SHARED", period=0.005, writer=writer)
            streamer.update_sensors(left, right)
            streamer.start()
            deadline = time.time() + 0.4
            while time.time() < deadline:
                streamer.update_sensors(left, right)
                time.sleep(0.02)
            streamer.shutdown()
            streamer.join(timeout=2)
        finally:
            h.serial.Serial = orig

        self.assertTrue(streamer.shared)
        self.assertEqual(opened, [], "shared mode must not open its own port")
        self.assertGreaterEqual(len(frames), 10)
        self.assertTrue(all(len(f) == h.FRAME_LEN for f in frames))
        decoded = Frame.from_bytes(frames[-1])
        self.assertEqual(int(decoded.can_id), h.CAN_ID)
        self.assertEqual(decoded.payload[0], 0)    # Valve A (Fx avg) 0.0 N
        self.assertEqual(decoded.payload[1], 40)   # Valve B (Fz avg) 4.0 N
        self.assertGreaterEqual(streamer.snapshot().sent, 10)

    def test_shared_writer_returning_zero_counts_as_not_sent(self):
        """write_frame() returns 0 while the port is closed; that is not an error."""
        streamer = h.Stm32HapticStreamer("COM_SHARED", period=0.005, writer=lambda _f: 0)
        streamer.update_sensors(_side(fz=1.0, taxels=0.5), _side())
        streamer.start()
        try:
            time.sleep(0.2)
        finally:
            streamer.shutdown()
            streamer.join(timeout=2)
        snap = streamer.snapshot()
        self.assertEqual(snap.sent, 0)
        self.assertEqual(snap.errors, 0)
        self.assertEqual(streamer.state, "stopped")


    def test_short_frame_write_counts_as_error(self):
        """A truncated frame must be reported, never silently accepted."""
        streamer = h.Stm32HapticStreamer("COM_SHORT", period=0.005, writer=lambda _f: 7)
        streamer.update_sensors(_side(fz=1.0, taxels=0.5), _side())
        streamer.start()
        try:
            time.sleep(0.12)
        finally:
            streamer.shutdown()
            streamer.join(timeout=2)
        snap = streamer.snapshot()
        self.assertEqual(snap.sent, 0)
        self.assertGreater(snap.errors, 0)
        self.assertEqual(streamer.state, "stopped")


if __name__ == "__main__":
    unittest.main()
