"""
Comprehensive unit tests for the TUE USB Binary Protocol library.
"""

import os
import sys
import unittest
import struct

# Ensure parent directory (python/) is in sys.path when test script is executed directly
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from tue_usb import (
    CanId,
    ErrorCode,
    Frame,
    FrameParser,
    FrameType,
    HapticPayload,
    LedState,
    PositionXYZ,
    ServoMode,
    ServoTorque,
    DcMode,
    DcInfo,
    DcChannelStatus,
    DcFaults,
    TueUsbClient,
    crc8_sae_j1850,
    crc8_selftest,
    is_reserved_can_id,
    mm_to_um,
    um_to_mm,
    Decoders,
    Encoders,
)


class TestProtocol(unittest.TestCase):

    def test_01_crc8_selftest(self):
        """Validate SAE J1850 CRC-8 implementation."""
        self.assertTrue(crc8_selftest())
        self.assertEqual(crc8_sae_j1850(b"123456789"), 0x4B)
        self.assertEqual(crc8_sae_j1850(b""), 0x00)

    def test_02_reserved_can_ids(self):
        """Firmware rule: no internal command may use 0x3..."""
        self.assertTrue(is_reserved_can_id(0x300))
        self.assertTrue(is_reserved_can_id(0x350))
        self.assertTrue(is_reserved_can_id(0x3FF))
        self.assertTrue(is_reserved_can_id(0x3000))
        self.assertTrue(is_reserved_can_id(0x3ABC))
        self.assertTrue(is_reserved_can_id(0x30000000))

        # Check that none of the defined CanId enum values are reserved,
        # except the haptic IDs that deliberately live in the custom range.
        self.assertEqual(CanId.HAPTIC_TELEMETRY, 0x300)
        self.assertEqual(CanId.HAPTIC_PING, 0x301)
        self.assertEqual(CanId.HAPTIC_DIAG, 0x302)
        for cid in CanId:
            if cid in (CanId.HAPTIC_TELEMETRY, CanId.HAPTIC_PING, CanId.HAPTIC_DIAG):
                continue
            self.assertFalse(
                is_reserved_can_id(cid.value),
                f"CanId {cid.name} (0x{cid.value:X}) collides with reserved range!"
            )

    def test_03_frame_roundtrip(self):
        """Serialize and deserialize a Frame and check byte exactness."""
        f = Frame(
            frame_type=FrameType.REQ,
            can_id=CanId.SYS_PING,
            payload=b"\x12\x34\x56\x78",
            seq=42
        )
        raw = f.to_bytes()

        self.assertEqual(raw[0], 0xAA)
        self.assertEqual(raw[1], 0x55)
        self.assertEqual(raw[-2], 0x55)
        self.assertEqual(raw[-1], 0xAA)
        self.assertEqual(len(raw), 12 + 4)  # 16 bytes

        parsed = Frame.from_bytes(raw)
        self.assertEqual(parsed.frame_type, FrameType.REQ)
        self.assertEqual(parsed.can_id, CanId.SYS_PING)
        self.assertEqual(parsed.payload, b"\x12\x34\x56\x78")
        self.assertEqual(parsed.seq, 42)
        self.assertTrue(parsed.is_crc_valid())

    def test_04_stream_parser_fuzz_and_resync(self):
        """Test stream parser recovery over noisy, fragmented, or concatenated bytes."""
        parser = FrameParser()

        f1 = Frame(FrameType.REQ, CanId.SYS_INFO, b"", seq=1)
        f2 = Frame(FrameType.REQ, CanId.STEP_GET_POS_XY, b"", seq=2)

        garbage = b"\x00\xFF\xAA\xAA\x12\x34\x55\xAA\x01\x02\x03"
        stream = garbage + f1.to_bytes() + b"\xDE\xAD" + f2.to_bytes() + b"\xBE\xEF"

        # Feed 1 byte at a time to test fragmentation
        frames = []
        for b in stream:
            res = parser.feed_byte(b)
            if res is not None:
                frames.append(res)

        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].can_id, CanId.SYS_INFO)
        self.assertEqual(frames[0].seq, 1)
        self.assertEqual(frames[1].can_id, CanId.STEP_GET_POS_XY)
        self.assertEqual(frames[1].seq, 2)

    def test_05_payload_encoders_decoders(self):
        """Verify SYS, STEP, and SERVO payload packing/unpacking."""
        # 1. System
        p_ping = Encoders.sys_ping(0xCAFEBABE)
        self.assertEqual(Decoders.sys_ping(p_ping), 0xCAFEBABE)

        raw_info = struct.pack("<BBBB I", 1, 2, 3, 4, 0x12345678)
        info = Decoders.sys_info(raw_info)
        self.assertEqual(info.hw_id, 1)
        self.assertEqual(info.hw_rev, 2)
        self.assertEqual(info.fw_major, 3)
        self.assertEqual(info.fw_minor, 4)
        self.assertEqual(info.build_flags, 0x12345678)

        # 2. Stepper
        p_xy = Encoders.step_move_xy(12.345, -67.890)
        x, y = Decoders.step_pos_xy(p_xy)
        self.assertAlmostEqual(x, 12.345, places=3)
        self.assertAlmostEqual(y, -67.890, places=3)

        p_z = Encoders.step_move_z(45.678, feed_mm_s=25, accel_mm_s2=250)
        z = Decoders.step_pos_z(p_z)
        self.assertAlmostEqual(z, 45.678, places=3)

        # 3. Servo
        p_spos = Encoders.servo_set_pos(1, pos=2048, speed=500, accel=30)
        self.assertEqual(len(p_spos), 6)
        s_id_req, accel_req, pos_req, spd_req = struct.unpack("<BBhh", p_spos)
        self.assertEqual(s_id_req, 1)
        self.assertEqual(accel_req, 30)
        self.assertEqual(pos_req, 2048)
        self.assertEqual(spd_req, 500)

        # Test response decoder
        resp_spos = struct.pack("<Bh", 1, 2048)
        s_id, pos = Decoders.servo_pos(resp_spos)
        self.assertEqual(s_id, 1)
        self.assertEqual(pos, 2048)

        # Dual telemetry frames
        raw_dyn = struct.pack("<BhhhB", 1, 2048, 120, -350, 1)
        dyn = Decoders.servo_fb_dynamics(raw_dyn)
        self.assertEqual(dyn.servo_id, 1)
        self.assertEqual(dyn.position, 2048)
        self.assertEqual(dyn.speed, 120)
        self.assertEqual(dyn.load_raw, -350)
        self.assertAlmostEqual(dyn.load_percent, -35.0, places=1)
        self.assertTrue(dyn.is_moving)

        raw_hlt = struct.pack("<BBbBhH", 1, 118, 38, 0x01, 250, 0)
        hlt = Decoders.servo_fb_health(raw_hlt)
        self.assertEqual(hlt.servo_id, 1)
        self.assertAlmostEqual(hlt.voltage_v, 11.8, places=1)
        self.assertEqual(hlt.temperature_c, 38)
        self.assertEqual(hlt.current_ma, 250)
        self.assertIn("VOLTAGE", hlt.alarms)

    def test_06_mock_client_transactions(self):
        """Verify TueUsbClient high-level API in mock mode."""
        with TueUsbClient(mock=True) as client:
            self.assertTrue(client.is_connected())

            # System
            echo_tok = client.ping(0xAABBCCDD)
            self.assertEqual(echo_tok, 0xAABBCCDD)

            info = client.get_info()
            self.assertEqual(info.hw_id, 1)

            uid = client.get_uid()
            self.assertEqual(uid.hex_string, "11223344-55667788-99AABBCC")

            led_state = client.set_led(0, LedState.ON)
            self.assertEqual(led_state, 1)

            # Stepper
            cfg = client.step_get_config()
            self.assertEqual(cfg.xy_steps_per_mm, 80)

            client.step_setpos_xy(10.0, 20.0)
            x, y = client.step_get_pos_xy()
            self.assertAlmostEqual(x, 10.0, places=3)
            self.assertAlmostEqual(y, 20.0, places=3)

            client.step_move_rel_xy(5.0, -2.0)
            x, y = client.step_get_pos_xy()
            self.assertAlmostEqual(x, 15.0, places=3)
            self.assertAlmostEqual(y, 18.0, places=3)

            # Servo
            client.servo_set_pos(1, 1500)
            self.assertEqual(client.servo_get_pos(1), 1500)

            fb = client.servo_get_feedback(1)
            self.assertEqual(fb.dynamics.servo_id, 1)
            self.assertEqual(fb.dynamics.position, 1500)
            self.assertAlmostEqual(fb.health.voltage_v, 12.0, places=1)

    def test_07_haptic_telemetry_roundtrip(self):
        """Verify the 0x300 haptic payload encoding and fire-and-forget frame."""
        original = HapticPayload(
            fz_left_n=12.3,
            fz_right_n=4.5,
            f_shear_avg_n=1.0,
            weight_tip=0.5,
            weight_mid=1.0,
            weight_base=0.0,
            left_contact=True,
            slip_alert=True,
            frame_counter=77,
        )
        raw = Encoders.haptic_telemetry(original)
        self.assertEqual(len(raw), 8)
        self.assertEqual(raw[0], 123)
        self.assertEqual(raw[3], 128)  # 0.5 * 255 = 127.5 -> 128
        self.assertEqual(raw[6], 0x05) # left contact + slip alert

        decoded = Decoders.haptic_telemetry(raw)
        self.assertAlmostEqual(decoded.fz_left_n, 12.3, places=3)
        self.assertAlmostEqual(decoded.fz_right_n, 4.5, places=3)
        self.assertTrue(decoded.left_contact)
        self.assertTrue(decoded.slip_alert)
        self.assertFalse(decoded.tare_active)
        self.assertEqual(decoded.frame_counter, 77)

        with TueUsbClient(mock=True) as client:
            frame = client.send_haptic_telemetry(original)
            self.assertEqual(frame.can_id, CanId.HAPTIC_TELEMETRY)
            self.assertEqual(frame.frame_type, FrameType.REQ)
            self.assertTrue(frame.is_crc_valid())
            self.assertEqual(len(frame.to_bytes()), 20)


    def test_08_haptic_ping_roundtrip(self):
        """CAN 0x301 UART-slave ping payload and mock transaction."""
        raw = Encoders.haptic_ping(0xA5)
        self.assertEqual(len(raw), 8)
        self.assertEqual(raw[0], 0xA5)

        info = Decoders.haptic_ping(bytes([0xA5, 1, 1, 0, 0, 9, 1, 3]))
        self.assertEqual(info.token, 0xA5)
        self.assertEqual(info.state_name, "IDLE")
        self.assertEqual(info.frame_counter, 9)
        self.assertEqual(info.errors, 1)
        self.assertEqual(info.uptime_s, 3)

        with TueUsbClient(mock=True) as client:
            pong = client.haptic_ping(0xA5)
            self.assertEqual(pong.token, 0xA5)
            self.assertEqual(pong.state_name, "RUNNING")


    def test_09_haptic_uart_diag(self):
        """CAN 0x302 diagnostics payload and mock transaction."""
        diag = Decoders.haptic_uart_diag(bytes([0x0B, 2, 0x34, 0x12, 20, 1, 2, 3]))
        self.assertTrue(diag.initialized)
        self.assertTrue(diag.rx_armed)
        self.assertTrue(diag.last_tx_ok)
        self.assertEqual(diag.rx_errors, 2)
        self.assertEqual(diag.rx_bytes, 0x1234)
        self.assertEqual(diag.last_ping_rx_bytes, 20)
        self.assertEqual(diag.tx_fail, 1)
        self.assertEqual(diag.timeouts, 2)
        self.assertEqual(diag.replies, 3)
        self.assertIn("STM32 UART", str(diag))

        with TueUsbClient(mock=True) as client:
            mock_diag = client.haptic_uart_diag()
            self.assertTrue(mock_diag.rx_armed)
            self.assertTrue(mock_diag.last_tx_ok)


    def test_10_dc_motor_protocol(self):
        """DC Motor CAN protocol (0x600 - 0x607) encoding and decoding."""
        # Validate CAN ID allocations
        self.assertEqual(CanId.DC_GET_INFO, 0x600)
        self.assertEqual(CanId.DC_SET_SPEED, 0x601)
        self.assertEqual(CanId.DC_GET_STATUS, 0x602)
        self.assertEqual(CanId.DC_SET_SPEED_ALL, 0x603)
        self.assertEqual(CanId.DC_BRAKE, 0x604)
        self.assertEqual(CanId.DC_COAST, 0x605)
        self.assertEqual(CanId.DC_GET_FAULTS, 0x606)
        self.assertEqual(CanId.DC_CONFIG_FREQ, 0x607)

        # 1. DC Get Info
        enc_info = Encoders.dc_get_info()
        self.assertEqual(len(enc_info), 0)
        resp_info_bytes = struct.pack("<BBHHH", 4, 20, 1000, 4199, 0x8872)
        info = Decoders.dc_info(resp_info_bytes)
        self.assertEqual(info.motor_count, 4)
        self.assertEqual(info.pwm_freq_khz, 20)
        self.assertEqual(info.max_duty, 1000)
        self.assertEqual(info.timer_arr, 4199)
        self.assertEqual(info.driver_type, 0x8872)
        self.assertEqual(info.driver_name, "DRV8872")

        # 2. DC Set Speed
        enc_spd = Encoders.dc_set_speed(1, 750)
        self.assertEqual(len(enc_spd), 3)
        m_id, spd, fault = Decoders.dc_set_speed(struct.pack("<BhB", 1, 750, 0))
        self.assertEqual(m_id, 1)
        self.assertEqual(spd, 750)
        self.assertFalse(fault)

        # 3. DC Get Status
        enc_st = Encoders.dc_get_status(2)
        self.assertEqual(enc_st, b"\x02")
        status = Decoders.dc_channel_status(struct.pack("<BBhBB", 2, DcMode.FORWARD, 500, 0, 0))
        self.assertEqual(status.motor_id, 2)
        self.assertEqual(status.mode, DcMode.FORWARD)
        self.assertEqual(status.speed, 500)
        self.assertEqual(status.duty_percent, 50.0)
        self.assertFalse(status.fault_active)
        self.assertFalse(status.fault_latched)

        # 4. DC Set Speed All
        enc_all = Encoders.dc_set_speed_all(500, -300, 1000, 0)
        self.assertEqual(len(enc_all), 8)
        mask, m1_m, m2_m, m3_m, m4_m = Decoders.dc_set_speed_all(
            struct.pack("<BBBBB", 0x00, DcMode.FORWARD, DcMode.REVERSE, DcMode.FORWARD, DcMode.COAST)
        )
        self.assertEqual(mask, 0)
        self.assertEqual(m1_m, DcMode.FORWARD)
        self.assertEqual(m2_m, DcMode.REVERSE)
        self.assertEqual(m3_m, DcMode.FORWARD)
        self.assertEqual(m4_m, DcMode.COAST)

        # 5. Brake & Coast
        self.assertEqual(Encoders.dc_brake(1), b"\x01")
        self.assertEqual(Encoders.dc_coast(1), b"\x01")
        m, mode = Decoders.dc_brake(struct.pack("<BB", 1, DcMode.BRAKE))
        self.assertEqual(m, 1)
        self.assertEqual(mode, DcMode.BRAKE)

        # 6. Faults
        faults = Decoders.dc_faults(struct.pack("<BBBBB", 0x05, 1, 0, 1, 0))
        self.assertEqual(faults.mask, 0x05)
        self.assertTrue(faults.m1_fault)
        self.assertFalse(faults.m2_fault)
        self.assertTrue(faults.m3_fault)
        self.assertFalse(faults.m4_fault)
        self.assertTrue(faults.has_any_fault())

        # 7. Config Freq
        self.assertEqual(Encoders.dc_config_freq(25), struct.pack("<H", 25))
        self.assertEqual(Decoders.dc_config_freq(struct.pack("<I", 25000)), 25000)


    def test_11_mock_client_dc_operations(self):
        """DC Motor client methods using simulated mock engine."""
        with TueUsbClient(mock=True) as client:
            info = client.dc_get_info()
            self.assertEqual(info.motor_count, 4)
            self.assertEqual(info.driver_name, "DRV8872")

            # Set speed
            m_id, spd, fault = client.dc_set_speed(1, 600)
            self.assertEqual(m_id, 1)
            self.assertEqual(spd, 600)
            self.assertFalse(fault)

            # Query status
            st = client.dc_get_status(1)
            self.assertEqual(st.motor_id, 1)
            self.assertEqual(st.speed, 600)
            self.assertEqual(st.mode, DcMode.FORWARD)

            # Reverse
            client.dc_set_speed(2, -400)
            st2 = client.dc_get_status(2)
            self.assertEqual(st2.speed, -400)
            self.assertEqual(st2.mode, DcMode.REVERSE)

            # Set all
            mask, m1, m2, m3, m4 = client.dc_set_speed_all(100, 200, 300, 400)
            self.assertEqual(mask, 0)
            self.assertEqual(m1, DcMode.FORWARD)

            # Brake
            m, b_mode = client.dc_brake(1)
            self.assertEqual(m, 1)
            self.assertEqual(b_mode, DcMode.BRAKE)

            # Coast
            m, c_mode = client.dc_coast(1)
            self.assertEqual(m, 1)
            self.assertEqual(c_mode, DcMode.COAST)

            # Faults
            faults = client.dc_get_faults()
            self.assertEqual(faults.mask, 0)

            # Frequency config
            actual_hz = client.dc_set_frequency(20)
            self.assertEqual(actual_hz, 20000)


if __name__ == "__main__":
    unittest.main()
