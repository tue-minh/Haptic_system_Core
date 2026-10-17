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
        payload = self.synth.synthesize(_side(fz=5.0), _side(fz=3.0), now=0.0)
        self.assertEqual(payload[:7], bytes([0x32, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x03]))

    def test_shear_at_tip(self):
        tip = _side(fz=2.0, fy=1.0)
        tip["taxels_fz"] = [0.0] * h.N_TAXELS
        for i in range(h.ZONE_TIP[0], h.ZONE_TIP[1]):
            tip["taxels_fz"][i] = 1.0
        payload = self.synth.synthesize(tip, tip, now=0.0)
        self.assertEqual(payload[:7], bytes([0x14, 0x14, 0x0A, 0xFF, 0x00, 0x00, 0x03]))

    def test_shear_axis_x_selects_fx(self):
        synth = h.HapticPayloadSynthesizer(shear_axis="X")
        payload = synth.synthesize(_side(fz=2.0, fx=2.0, fy=9.9), _side(fz=2.0, fx=1.0), now=0.0)
        self.assertEqual(payload[2], 0x0F)  # mean |Fx| = 1.5 N -> 15

    def test_stale_side_is_zeroed(self):
        stale = _side(fz=5.0, fy=2.0, age_s=0.5)
        fresh = _side(fz=3.0, fy=1.0)
        payload = self.synth.synthesize(stale, fresh, now=0.0)
        self.assertEqual(payload[0], 0)     # left contribution zeroed
        self.assertEqual(payload[1], 30)    # right intact
        self.assertFalse(payload[6] & h.FLAG_LEFT_CONTACT)
        self.assertTrue(payload[6] & h.FLAG_RIGHT_CONTACT)

    def test_missing_keys_and_short_taxels_are_safe(self):
        payload = self.synth.synthesize({}, {}, now=0.0)
        self.assertEqual(payload[:7], b"\x00" * 7)
        short = {"fz": 1.0, "taxels_fz": [1.0, 2.0]}
        long = {"taxels_fz": list(range(100))}
        payload = self.synth.synthesize(short, long, now=0.0)
        self.assertEqual(len(payload), h.PAYLOAD_LEN)

    def test_weighting_sums_to_255(self):
        tax = [0.0] * h.N_TAXELS
        tax[h.ZONE_MID[0]:h.ZONE_MID[1]] = [1.0] * (h.ZONE_MID[1] - h.ZONE_MID[0])
        tax[h.ZONE_BASE[0]:h.ZONE_BASE[1]] = [1.0] * (h.ZONE_BASE[1] - h.ZONE_BASE[0])
        payload = self.synth.synthesize(_side(fz=1.0, taxels=0.0, taxels_fz=tax), {}, now=0.0)
        w_a, w_b, w_c = payload[3], payload[4], payload[5]
        self.assertEqual(w_a, 0)
        self.assertEqual(w_a + w_b + w_c, 255)

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
        snap = h.HapticSnapshot.from_payload(bytes([20, 20, 10, 255, 0, 0, 0x03, 0]), 1.0, 0.5)
        forces = {v.name: v.force for v in snap.valves}
        # D/E carry the bilateral clamp force, A/B/C split the shear by weight
        self.assertAlmostEqual(forces["D"], 2.0, places=6)
        self.assertAlmostEqual(forces["E"], 2.0, places=6)
        self.assertAlmostEqual(forces["A"], 1.0, places=6)
        self.assertAlmostEqual(forces["B"], 0.0, places=6)
        self.assertAlmostEqual(forces["C"], 0.0, places=6)
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
        self.assertEqual(decoded.payload[0], 50)   # Fz_Left 5.0 N
        self.assertEqual(decoded.payload[1], 30)   # Fz_Right 3.0 N

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


if __name__ == "__main__":
    unittest.main()
