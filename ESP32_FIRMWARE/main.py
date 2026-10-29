from machine import Pin, PWM, I2C, UART
import time
import sys

try:
    import uselect as select
except ImportError:  # newer MicroPython exposes the module as `select`
    import select


# ==============================================================================
# HAPTIC SLAVE CONTROLLER - ESP32 WROOM FIRMWARE WITH ROBUST FAILSAFE
# ==============================================================================
#
# Hardware:
#   - ESP32-WROOM-32
#   - PCA9548A 8-Channel I2C Multiplexer (Addr: 0x77)
#   - 5x MCP4725 12-bit DACs controlling 5 pneumatic valves:
#       Ch 0 (A): Bottom - Distal Tip (Ngọn ngón tay)
#       Ch 1 (B): Bottom - Medial Center (Giữa đệm ngón)
#       Ch 2 (C): Bottom - Proximal Base (Chuôi đệm ngón)
#       Ch 3 (D): Lateral Left (Hông trái ngón tay)
#       Ch 4 (E): Lateral Right (Hông phải ngón tay)
#
# Communication:
#   - UART2 @ 115,200 baud (8-N-1, normal polling, no IRQ/DMA): Hardware link from STM32 Master
#       STM32 PD5 (USART2_TX) -> ESP32 GPIO16 (UART2_RX)
#       STM32 PD6 (USART2_RX) -> ESP32 GPIO17 (UART2_TX)
#       STM32 GND             -> ESP32 GND
#   - USB Serial (sys.stdin / REPL): Diagnostics & manual ASCII commands (>G<, >R<, >S<, >P<, >U<, >T<, >B<)
#
# Robust Failsafe (Chống lỗi toàn diện - Tự động xuất 0.0V):
#   1. MẤT TÍN HIỆU UART (Watchdog): Nếu không có gói hợp lệ > 150 ms -> Tự động xả 0V toàn bộ 5 van.
#   2. SAI PARSE / NHIỄU CRC (Consecutive Error Limiter): Nếu bị lỗi CRC hoặc sai cấu trúc >= 5 lần
#      liên tiếp -> Tự động ngắt khẩn cấp, xuất 0V và chuyển sang STATE_ERROR. Hệ thống chỉ
#      tự phục hồi sau RECOVERY_FRAMES frame hợp lệ liên tiếp (tránh bơm/xả liên tục khi nhiễu).
#   3. TRÀN BỘ ĐỆM NHIỄU (Noise Overflow): Nếu trong một lần parse có > 100 byte rác -> Xóa rác,
#      xuất 0V. Frame hợp lệ luôn được xử lý trước khi áp dụng failsafe này.
#   4. LỖI PHẦN CỨNG I2C (Bus Exception): Nếu mất kết nối I2C -> Reset PCA9548A, xuất 0V, báo đèn đỏ.
#   5. GIỚI HẠN ÁP AN TOÀN: Tuyệt đối không bao giờ xuất quá V_MAX = 2.20V (chặn đứng mức 3.3V).
# ==============================================================================


# ==============================================================================
# CONFIGURATION
# ==============================================================================

# ------------------------------------------------------------------------------
# LED Indicators (Active LOW)
# ------------------------------------------------------------------------------
LED_GREEN_PIN = 4
LED_RED_PIN   = 13
LED_BLUE_PIN  = 14

LED_ON  = 0
LED_OFF = 1

# ------------------------------------------------------------------------------
# I2C Bus Configuration
# ------------------------------------------------------------------------------
I2C_ID   = 0
SDA_PIN  = 21
SCL_PIN  = 22
I2C_FREQ = 400000

# ------------------------------------------------------------------------------
# PCA9548A Multiplexer
# ------------------------------------------------------------------------------
PCA_ADDR      = 0x77
PCA_RESET_PIN = 18
PCA_A2_PIN    = 25
PCA_A1_PIN    = 26
PCA_A0_PIN    = 27

# ------------------------------------------------------------------------------
# MCP4725 DACs (5 Channels: A, B, C, D, E)
# ------------------------------------------------------------------------------
MCP4725_ADDR = [
    0x60,  # Channel 0: Valve A (Bottom - Tip)
    0x60,  # Channel 1: Valve B (Bottom - Mid)
    0x60,  # Channel 2: Valve C (Bottom - Base)
    0x61,  # Channel 3: Valve D (Lateral Left)
    0x61   # Channel 4: Valve E (Lateral Right)
]

MCP4725_WRITE_DAC = 0x40
VDD               = 3.30
DAC_MAX           = 4095
DAC_COUNT         = 5

# ------------------------------------------------------------------------------
# UART2 Configuration (From STM32 Master - Normal UART @ 115200 8-N-1)
# ------------------------------------------------------------------------------
UART_ID       = 2
UART_BAUDRATE = 115200
UART_TX_PIN   = 17
UART_RX_PIN   = 16
UART_RX_BUF   = 512

# Maximum idle gap before discarding an incomplete (< 20 B) frame fragment
STALE_RX_TIMEOUT_MS = 25

# ------------------------------------------------------------------------------
# Firmware identity / binary ping protocol (CAN 0x301 over the STM32 UART link)
# ------------------------------------------------------------------------------
FW_VERSION_MAJOR      = 1
FW_VERSION_MINOR      = 0
PING_PROTOCOL_VERSION = 1

CAN_ID_HAPTIC_TELEMETRY = 0x300
CAN_ID_HAPTIC_PING      = 0x301
BEACON_CAN_ID           = 0x303
BEACON_PERIOD_MS        = 1000

# ------------------------------------------------------------------------------
# Haptic Safety & Failsafe Parameters
# ------------------------------------------------------------------------------
V_MIN       = 0.70    # Minimum opening voltage / mechanical deadband (Volts)
V_MAX       = 2.20    # ABSOLUTE SAFETY LIMIT (Volts) - Strictly prevents 3.3V!
F_THRESH    = 0.20    # Noise threshold for contact force (Newtons)
F_RATED     = 20.00   # Rated full-scale physical force (Newtons) mapping to V_MAX

WATCHDOG_TIMEOUT_MS    = 150   # Mất tín hiệu > 150 ms (15 chu kỳ @ 100Hz) -> Xuất 0.0V
CONSECUTIVE_ERRORS_MAX = 5     # 5 lần liên tiếp lỗi CRC hoặc sai frame -> Xuất 0.0V
MAX_RX_BUFFER_SIZE     = 100   # Ngưỡng byte rác tối đa trong bộ đệm UART
RECOVERY_FRAMES        = 3     # Số frame hợp lệ liên tiếp trước khi thoát STATE_ERROR


# ==============================================================================
# SYSTEM STATES
# ==============================================================================
STATE_IDLE    = 0
STATE_RUNNING = 1
STATE_ERROR   = 2

system_state        = STATE_IDLE
last_packet_ms      = time.ticks_ms()
last_rx_byte_ms     = time.ticks_ms()
consecutive_errors  = 0
recovery_count      = 0
last_haptic_counter = 0
uart_frames         = 0
rx_bytes_total      = 0
ping_rx             = 0
ping_tx             = 0
# Keep unsolicited 1Hz beacon disabled by default in pure polling mode so STM32
# never hits USART_SR_ORE overrun when idle. Can still be sent with >T< or >B<.
beacon_enabled      = False
beacon_seq          = 0
beacon_tx           = 0
last_beacon_ms      = time.ticks_ms()

# Active output voltages [A, B, C, D, E]
dac_voltage = [0.0, 0.0, 0.0, 0.0, 0.0]


# ==============================================================================
# HARDWARE INITIALIZATION
# ==============================================================================

# LEDs
led_green = Pin(LED_GREEN_PIN, Pin.OUT, value=LED_OFF)
led_red   = Pin(LED_RED_PIN, Pin.OUT, value=LED_OFF)
try:
    blue_pwm = PWM(Pin(LED_BLUE_PIN), freq=1000, duty_u16=0)
    _PWM_U16 = True
except (TypeError, ValueError):
    # Older MicroPython builds only expose PWM.duty() (0..1023).
    blue_pwm = PWM(Pin(LED_BLUE_PIN), freq=1000, duty=0)
    _PWM_U16 = False


def blue_brightness(percent):
    """Set the blue LED duty on both the duty_u16 and legacy duty APIs."""
    percent = max(0, min(100, int(percent)))
    if _PWM_U16:
        blue_pwm.duty_u16(int(65535 * percent / 100))
    else:
        blue_pwm.duty(int(1023 * percent / 100))


# PCA9548A Control Pins
pca_reset = Pin(PCA_RESET_PIN, Pin.OUT, value=1)
pca_a2    = Pin(PCA_A2_PIN, Pin.OUT, value=1)
pca_a1    = Pin(PCA_A1_PIN, Pin.OUT, value=1)
pca_a0    = Pin(PCA_A0_PIN, Pin.OUT, value=1)

# I2C Master
i2c = I2C(I2C_ID, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=I2C_FREQ)


def init_uart2():
    """Initialize UART2 at 115200 8-N-1 (normal polling, no IRQ) across ESP32 MicroPython builds."""
    try:
        Pin(UART_RX_PIN, Pin.IN, Pin.PULL_UP)
    except Exception:
        pass
    try:
        u = UART(
            UART_ID,
            baudrate=UART_BAUDRATE,
            bits=8,
            parity=None,
            stop=1,
            tx=Pin(UART_TX_PIN),
            rx=Pin(UART_RX_PIN),
            rxbuf=UART_RX_BUF,
            timeout=0,
        )
    except Exception:
        try:
            u = UART(
                UART_ID,
                baudrate=UART_BAUDRATE,
                bits=8,
                parity=None,
                stop=1,
                tx=Pin(UART_TX_PIN),
                rx=Pin(UART_RX_PIN),
                rxbuf=UART_RX_BUF,
            )
        except Exception:
            try:
                u = UART(
                    UART_ID,
                    baudrate=UART_BAUDRATE,
                    bits=8,
                    parity=None,
                    stop=1,
                    tx=UART_TX_PIN,
                    rx=UART_RX_PIN,
                )
            except Exception:
                u = UART(UART_ID, UART_BAUDRATE)
                u.init(UART_BAUDRATE, bits=8, parity=None, stop=1, tx=UART_TX_PIN, rx=UART_RX_PIN)
    # Drain any boot glitch bytes
    try:
        while u.any():
            u.read()
    except Exception:
        pass
    return u


# UART2 (Master Receiver/Responder @ 115200 8-N-1)
uart2 = init_uart2()

# USB Serial Poll (Diagnostic / REPL). Optional: a console/API mismatch must
# never prevent the UART2 loop from running.
try:
    serial_poll = select.poll()
    serial_poll.register(sys.stdin, select.POLLIN)
except Exception:
    serial_poll = None


# ==============================================================================
# SAE J1850 CRC-8 LOOKUP TABLE & CHECKSUM
# ==============================================================================
# Poly: 0x1D, Init: 0xFF, XorOut: 0xFF, MSB First
# Verification: crc8(b"123456789") == 0x4B

CRC8_TABLE = [
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
    0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
    0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
    0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
    0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
    0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
    0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
    0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
    0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
    0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
    0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
    0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
]


def crc8_sae_j1850(data):
    """Calculates SAE J1850 CRC-8 over byte sequence."""
    crc = 0xFF
    for b in data:
        crc = CRC8_TABLE[crc ^ b]
    return crc ^ 0xFF


def build_tue_frame(frame_type, can_id, seq, payload8):
    """Build a 20-byte TUE binary frame as immutable `bytes` (avoids bytearray slice assignment)."""
    body = bytes([
        frame_type & 0xFF,
        can_id & 0xFF,
        (can_id >> 8) & 0xFF,
        (can_id >> 16) & 0xFF,
        (can_id >> 24) & 0xFF,
        8,
        seq & 0xFF,
    ]) + bytes(payload8[:8])
    crc = crc8_sae_j1850(body)
    return b"\xAA\x55" + body + bytes([crc, 0x55, 0xAA])


# ==============================================================================
# PCA9548A & MCP4725 LOW-LEVEL DRIVERS WITH FAILSAFE
# ==============================================================================

def pca_reset_device():
    try:
        pca_reset.value(0)
        time.sleep_us(2)
        pca_reset.value(1)
        time.sleep_ms(2)
        return True
    except Exception:
        return False


def pca_select(channel):
    if channel < 0 or channel > 7:
        return False
    try:
        i2c.writeto(PCA_ADDR, bytes([1 << channel]))
        return True
    except OSError:
        return False


def pca_disable_all():
    try:
        i2c.writeto(PCA_ADDR, bytes([0]))
        return True
    except OSError:
        return False


def voltage_to_dac(voltage):
    val = int((voltage / VDD) * DAC_MAX)
    if val < 0:
        return 0
    if val > DAC_MAX:
        return DAC_MAX
    return val


def write_dac(channel, voltage):
    """Writes analog voltage (0.0 .. VDD) to MCP4725 on selected PCA channel."""
    if channel < 0 or channel >= DAC_COUNT:
        return False

    val = voltage_to_dac(voltage)
    if not pca_select(channel):
        return False

    addr = MCP4725_ADDR[channel]
    data = bytes([
        MCP4725_WRITE_DAC,
        (val >> 4) & 0xFF,
        (val & 0x0F) << 4
    ])
    try:
        i2c.writeto(addr, data)
        return True
    except OSError:
        return False


def set_voltage(channel, voltage):
    """Updates physical DAC output and caches state. Enforces safety clamp."""
    voltage = max(0.0, min(voltage, V_MAX))
    if not write_dac(channel, voltage):
        return False
    dac_voltage[channel] = voltage
    return True


def vent_all_chambers():
    """
    FAILSAFE SHUTDOWN: Sets all 5 pneumatic chambers to 0.0V (exhaust)
    and disconnects all PCA channels to eliminate any trapped pressure.
    Aborts immediately if PCA9548A is not responding on I2C so UART2 polling is never stalled.
    """
    global dac_voltage
    for ch in range(DAC_COUNT):
        dac_voltage[ch] = 0.0
    if not pca_select(0):
        return
    for ch in range(DAC_COUNT):
        try:
            if not write_dac(ch, 0.0):
                break
        except Exception:
            break
    pca_disable_all()


# ==============================================================================
# HAPTIC TRANSFER FUNCTION & SAFETY VOLTAGE LIMITER
# ==============================================================================

def compute_valve_voltage(force_n, v_min=V_MIN, v_max=V_MAX):
    """
    Transforms physical contact force (Newtons) into safe valve voltage (0.0 .. V_MAX).
    Enforces mechanical deadband compensation and hard clamps output to strictly prevent 3.3V.
    """
    if force_n <= F_THRESH:
        return 0.0

    ratio = (force_n - F_THRESH) / (F_RATED - F_THRESH)
    if ratio > 1.0:
        ratio = 1.0
    elif ratio < 0.0:
        ratio = 0.0

    v_out = v_min + (v_max - v_min) * ratio
    return min(v_out, v_max)


def handle_haptic_packet(payload):
    """
    Processes the 8-byte unified payload received from CAN ID 0x300:
      Byte 0: Fz_Left       (0.1 N / LSB) -> Van D (Hông trái)
      Byte 1: Fz_Right      (0.1 N / LSB) -> Van E (Hông phải)
      Byte 2: F_shear_avg   (0.1 N / LSB) -> Lực trượt / trọng lượng
      Byte 3: Weight_Tip    (0..255)      -> w_A (Ngọn ngón tay)
      Byte 4: Weight_Mid    (0..255)      -> w_B (Giữa đệm ngón)
      Byte 5: Weight_Base   (0..255)      -> w_C (Chuôi đệm ngón)
      Byte 6: Status_Flags  (Bitmask)
      Byte 7: Frame_Counter (0..255)
    """
    global last_packet_ms, recovery_count, last_haptic_counter
    try:
        last_packet_ms = time.ticks_ms()
        last_haptic_counter = payload[7]

        # After an error trip, require a few consecutive good frames before
        # re-pressurising the chambers; stay vented while recovering.
        if system_state == STATE_ERROR:
            recovery_count += 1
            if recovery_count < RECOVERY_FRAMES:
                return

        # 1. Trích xuất lực kẹp pháp tuyến má trái & má phải (0.1 N / LSB)
        f_left_z  = payload[0] * 0.1
        f_right_z = payload[1] * 0.1

        # 2. Trích xuất lực trượt / trọng lượng và trọng số 3 vùng
        f_shear = payload[2] * 0.1
        w_A     = payload[3] / 255.0
        w_B     = payload[4] / 255.0
        w_C     = payload[5] / 255.0

        # 3. Tính toán lực thực tế cho từng vị trí
        f_A = f_shear * w_A   # Van A: Mặt dưới - Ngọn (Distal Tip)
        f_B = f_shear * w_B   # Van B: Mặt dưới - Giữa (Medial Mid)
        f_C = f_shear * w_C   # Van C: Mặt dưới - Chuôi (Proximal Base)
        f_D = f_left_z        # Van D: Hông trái (Lateral Left)
        f_E = f_right_z       # Van E: Hông phải (Lateral Right)

        # 4. Tính toán điện áp an toàn qua hàm giới hạn (0.0 .. V_MAX)
        v_A = compute_valve_voltage(f_A)
        v_B = compute_valve_voltage(f_B)
        v_C = compute_valve_voltage(f_C)
        v_D = compute_valve_voltage(f_D)
        v_E = compute_valve_voltage(f_E)

        # 5. Cập nhật 5 DAC qua PCA9548A (chỉ ghi nếu có thay đổi để tối ưu bus I2C)
        target_volts = (v_A, v_B, v_C, v_D, v_E)
        for ch in range(DAC_COUNT):
            if abs(target_volts[ch] - dac_voltage[ch]) >= 0.015:
                if not set_voltage(ch, target_volts[ch]):
                    # Nếu lỗi ghi I2C phần cứng -> chuyển sang chế độ lỗi khẩn cấp
                    set_state(STATE_ERROR)
                    return

        set_state(STATE_RUNNING)

    except Exception:
        # Nếu có bất kỳ lỗi logic nào -> ngắt van an toàn về 0V
        vent_all_chambers()
        set_state(STATE_ERROR)


def build_ping_reply(seq, token):
    """Build 20-byte binary RESP_OK (0x02) frame for CAN ID 0x301 as immutable `bytes`."""
    uptime_s = (time.ticks_ms() // 1000) & 0xFF
    errors = consecutive_errors if consecutive_errors < 255 else 255
    payload = bytes([
        token & 0xFF,
        PING_PROTOCOL_VERSION,
        FW_VERSION_MAJOR,
        FW_VERSION_MINOR,
        system_state & 0xFF,
        last_haptic_counter & 0xFF,
        errors,
        uptime_s,
    ])
    return build_tue_frame(0x02, CAN_ID_HAPTIC_PING, seq, payload)


def handle_ping_frame(frame):
    """Answer a CAN 0x301 binary ping from the STM32 master over the 115200 UART link.

    Request payload byte 0 is an 8-bit token echoed by the reply. The reply is
    a normal 20-byte TUE frame (RESP_OK = 0x02) with this status payload:

        0 token, 1 protocol version, 2 fw major, 3 fw minor,
        4 system state, 5 last haptic frame counter,
        6 consecutive errors, 7 uptime seconds (mod 256)
    """
    global ping_rx, ping_tx
    if frame[2] != 0x01:              # only FRAME_TYPE_REQ is answered
        return

    token = frame[9] if frame[7] >= 1 else 0
    seq = frame[8]
    reply = build_ping_reply(seq, token)
    written = uart2.write(reply)
    ping_rx += 1
    ping_tx += 1
    # Print only AFTER the reply is on the wire so stdout can never delay it.
    send("PING rx seq={} token=0x{:02X} -> tx {}B".format(
        seq, token, written if written else len(reply)))


def send_beacon(announce=False):
    """Send one 20-byte ASYNC heartbeat (CAN 0x303) on UART2."""
    global beacon_seq, beacon_tx, last_beacon_ms
    payload = bytes([
        0xBE,
        0xAC,
        PING_PROTOCOL_VERSION,
        FW_VERSION_MAJOR,
        FW_VERSION_MINOR,
        system_state & 0xFF,
        last_haptic_counter & 0xFF,
        (time.ticks_ms() // 1000) & 0xFF,
    ])
    seq_sent = beacon_seq
    frame = build_tue_frame(0x04, BEACON_CAN_ID, seq_sent, payload)
    uart2.write(frame)
    beacon_seq = (beacon_seq + 1) & 0xFF
    beacon_tx += 1
    last_beacon_ms = time.ticks_ms()
    if announce:
        send("BEACON tx seq={} count={} pins=GPIO{}->GPIO{}".format(
            seq_sent, beacon_tx, UART_TX_PIN, UART_RX_PIN))


# ==============================================================================
# UART2 PROTOCOL FRAME PARSER (115,200 baud) WITH ANTI-ERROR PROTECTION
# ==============================================================================
# Use immutable `bytes` buffer so slicing (`uart_rx_buf = uart_rx_buf[1:]` and
# `uart_rx_buf = uart_rx_buf[20:]`) works on every MicroPython build without
# triggering `TypeError: 'bytearray' object doesn't support item deletion`.
uart_rx_buf = b""


def process_uart2():
    """
    Sliding-window stream parser for 20-byte TUE Binary Frames with full error rejection:
      - Tự động phát hiện lỗi Preamble, Postamble, CRC-8, CAN ID.
      - Đếm số lần lỗi liên tiếp: nếu >= CONSECUTIVE_ERRORS_MAX -> Xả van về 0V ngay lập tức!
      - Chống ngập rác bộ đệm: rác được đếm trong lúc parse và chỉ xả phần còn lại
        sau khi mọi frame hợp lệ trong bộ đệm đã được xử lý.
    """
    global uart_rx_buf, consecutive_errors, uart_frames, rx_bytes_total, last_rx_byte_ms

    try:
        now = time.ticks_ms()
        # Đọc toàn bộ dữ liệu có sẵn trên UART2
        n = uart2.any()
        if n:
            chunk = uart2.read(n)
            if chunk:
                rx_bytes_total += len(chunk)
                last_rx_byte_ms = now
                uart_rx_buf = uart_rx_buf + bytes(chunk)
        elif len(uart_rx_buf) > 0 and time.ticks_diff(now, last_rx_byte_ms) > STALE_RX_TIMEOUT_MS:
            # Flush stale incomplete fragment (< 20 B) after idle gap
            uart_rx_buf = b""
            return

        discarded = 0

        # Xử lý các khung gói tin có trong bộ đệm. Frame hợp lệ luôn được parse
        # trước khi áp dụng failsafe tràn rác, nên không bao giờ bị bỏ oan.
        while len(uart_rx_buf) >= 20:
            # Kiểm tra Preamble 0xAA 0x55
            if uart_rx_buf[0] != 0xAA or uart_rx_buf[1] != 0x55:
                uart_rx_buf = uart_rx_buf[1:]
                discarded += 1
                continue

            # Kiểm tra Postamble 0x55 0xAA ở cuối khung 20 byte
            if uart_rx_buf[18] != 0x55 or uart_rx_buf[19] != 0xAA:
                consecutive_errors += 1
                uart_rx_buf = uart_rx_buf[1:]
                discarded += 1
                if consecutive_errors >= CONSECUTIVE_ERRORS_MAX:
                    vent_all_chambers()
                    set_state(STATE_ERROR)
                continue

            # Kiểm tra toàn vẹn CRC-8 (SAE J1850) trên Bytes 2..16
            calc_crc = crc8_sae_j1850(uart_rx_buf[2:17])
            if calc_crc != uart_rx_buf[17]:
                # Gói tin bị hỏng / nhiễu bit trên đường truyền
                consecutive_errors += 1
                uart_rx_buf = uart_rx_buf[1:]
                discarded += 1
                if consecutive_errors >= CONSECUTIVE_ERRORS_MAX:
                    vent_all_chambers()
                    set_state(STATE_ERROR)
                continue

            frame = uart_rx_buf[:20]
            uart_rx_buf = uart_rx_buf[20:]
            uart_frames += 1

            # Kiểm tra CAN ID (Little-Endian) và DLC
            can_id = (
                frame[3] |
                (frame[4] << 8) |
                (frame[5] << 16) |
                (frame[6] << 24)
            )
            dlc = frame[7]

            if can_id == CAN_ID_HAPTIC_TELEMETRY and dlc == 8:
                payload = frame[9:17]
                # Gói tin hoàn toàn hợp lệ -> Reset cờ đếm lỗi
                consecutive_errors = 0
                handle_haptic_packet(payload)
            elif can_id == CAN_ID_HAPTIC_PING:
                # Ping/status request from the STM32 master; do NOT refresh
                # the haptic watchdog and do not touch the valve outputs.
                handle_ping_frame(frame)
            # Frame đúng cấu trúc nhưng khác ID: bỏ qua, không tính là lỗi
            # (watchdog vẫn bảo vệ nếu không có frame 0x300 nào tới).

        # Failsafe tràn rác: chỉ áp dụng sau khi đã parse hết frame hợp lệ.
        if discarded > MAX_RX_BUFFER_SIZE:
            uart_rx_buf = b""
            consecutive_errors += 1
            if consecutive_errors >= CONSECUTIVE_ERRORS_MAX:
                vent_all_chambers()
                set_state(STATE_ERROR)

    except Exception as exc:
        # Bắt mọi ngoại lệ hệ thống -> đưa về trạng thái an toàn 0V
        consecutive_errors += 1
        uart_rx_buf = b""
        vent_all_chambers()
        set_state(STATE_ERROR)
        send("UART2_ERR: {}".format(exc))


# ==============================================================================
# WATCHDOG FAILSAFE & STATE ANIMATION
# ==============================================================================
BREATH_PERIOD_MS = 1600
breath_start     = time.ticks_ms()
error_timer      = time.ticks_ms()
error_phase      = 0


def set_state(state):
    """Chuyển đổi trạng thái hệ thống. Tự động xả van nếu rời trạng thái RUNNING."""
    global system_state, breath_start, error_timer, error_phase, recovery_count

    # Bất cứ khi nào rời khỏi STATE_RUNNING (hoặc vào STATE_ERROR / STATE_IDLE), luôn xả sạch van về 0V
    if state != STATE_RUNNING and system_state == STATE_RUNNING:
        vent_all_chambers()

    system_state = state
    recovery_count = 0
    now = time.ticks_ms()

    if state == STATE_IDLE:
        led_green.value(LED_ON)
        led_red.value(LED_OFF)
        blue_brightness(0)
    elif state == STATE_RUNNING:
        led_green.value(LED_OFF)
        led_red.value(LED_OFF)
        breath_start = now
    elif state == STATE_ERROR:
        # Bắt buộc xả van về 0V khi gặp lỗi
        vent_all_chambers()
        led_green.value(LED_OFF)
        blue_brightness(0)
        error_timer = now
        error_phase = 0
        led_red.value(LED_ON)


def update_led():
    global error_timer, error_phase
    now = time.ticks_ms()
    if system_state == STATE_IDLE:
        led_green.value(LED_ON)
        led_red.value(LED_OFF)
        blue_brightness(0)
    elif system_state == STATE_RUNNING:
        led_green.value(LED_OFF)
        led_red.value(LED_OFF)
        elapsed = time.ticks_diff(now, breath_start)
        phase   = elapsed % BREATH_PERIOD_MS
        half    = BREATH_PERIOD_MS // 2
        brightness = (phase * 100) // half if phase < half else ((BREATH_PERIOD_MS - phase) * 100) // half
        blue_brightness(max(2, brightness))
    elif system_state == STATE_ERROR:
        led_green.value(LED_OFF)
        blue_brightness(0)
        duration = 100 if error_phase < 2 else 70
        if time.ticks_diff(now, error_timer) >= duration:
            error_timer = now
            error_phase = (error_phase + 1) % 4
        led_red.value(LED_ON if error_phase < 2 else LED_OFF)


def check_watchdog():
    """
    FAILSAFE WATCHDOG:
    - Nếu mất tín hiệu UART > WATCHDOG_TIMEOUT_MS: Tự động ngắt toàn bộ van về 0.0V.
    - Đảm bảo trong bất kỳ trạng thái nào ngoài STATE_RUNNING, tất cả van luôn là 0.0V.
    """
    global system_state
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, last_packet_ms)

    # 1. Phát hiện mất tín hiệu UART khi đang chạy
    if system_state == STATE_RUNNING:
        if elapsed > WATCHDOG_TIMEOUT_MS:
            vent_all_chambers()
            set_state(STATE_IDLE)

    # 2. Failsafe chủ động: Nếu không ở RUNNING mà có van nào còn điện áp > 0V -> ép về 0V ngay lập tức
    elif system_state != STATE_RUNNING:
        for ch in range(DAC_COUNT):
            if dac_voltage[ch] > 0.0:
                set_voltage(ch, 0.0)


# ==============================================================================
# USB SERIAL REPL DIAGNOSTICS (Optional ASCII commands: >G<, >R<, >S<, >P<, >U<, >T<, >B<)
# ==============================================================================
def serial_available():
    if serial_poll is None:
        return False
    try:
        return len(serial_poll.poll(0)) > 0
    except Exception:
        return False


def send(text):
    try:
        sys.stdout.write(str(text) + "\r\n")
    except Exception:
        pass


def handle_usb_command(cmd):
    global beacon_enabled
    if cmd == "G":
        for ch in range(DAC_COUNT):
            send("{:.3f}".format(dac_voltage[ch]))
    elif cmd == "R":
        vent_all_chambers()
        send("RESET_OK")
        set_state(STATE_IDLE)
    elif cmd == "S":
        # I2C Scan
        for ch in range(8):
            if pca_select(ch):
                time.sleep_ms(2)
                devs = [addr for addr in i2c.scan() if addr != PCA_ADDR]
                res = ",".join("{:02X}".format(a) for a in devs) if devs else "-"
                send("{}:{}".format(ch, res))
        pca_disable_all()
    elif cmd == "P":
        # Ping/UART link diagnostics
        send("PING_STATS uart_frames={} rx_bytes={} ping_rx={} ping_tx={} state={} errors={} beacon={} beacon_tx={}".format(
            uart_frames, rx_bytes_total, ping_rx, ping_tx, system_state, consecutive_errors,
            1 if beacon_enabled else 0, beacon_tx))
    elif cmd == "U":
        send("UART2 tx={} rx={} baud={} rxbuf={} frames={} rx_bytes={} ping_rx={} ping_tx={} beacon={} beacon_tx={}".format(
            UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE, UART_RX_BUF,
            uart_frames, rx_bytes_total, ping_rx, ping_tx, 1 if beacon_enabled else 0, beacon_tx))
    elif cmd == "T":
        # Send one immediate heartbeat to prove the ESP32 -> STM32 wire.
        send_beacon(announce=True)
    elif cmd == "B":
        # Toggle the 1 Hz heartbeat.
        beacon_enabled = not beacon_enabled
        send("BEACON {}".format("ON" if beacon_enabled else "OFF"))


usb_line = ""


def process_usb_serial():
    """Console reader that never blocks the 100 Hz UART path.
    Accepts commands either framed as `>P<` (immediately on `<`) or terminated by newline.
    """
    global usb_line
    while serial_available():
        ch = sys.stdin.read(1)
        if not ch:
            break
        if ch == "<" and usb_line.lstrip().startswith(">"):
            cmd = usb_line.lstrip()[1:].strip().upper()
            usb_line = ""
            if cmd:
                handle_usb_command(cmd)
        elif ch == "\n" or ch == "\r":
            line = usb_line.strip().upper()
            usb_line = ""
            if not line:
                continue
            if line.startswith(">") and line.endswith("<"):
                line = line[1:-1].strip()
            elif line.startswith(">"):
                line = line[1:].strip()
            if line:
                handle_usb_command(line)
        else:
            usb_line += ch
            if len(usb_line) > 64:
                usb_line = ""


# ==============================================================================
# BOOT SELF-TEST & BOOTSTRAP
# ==============================================================================
def run_boot_selftest():
    """Verify CRC8, 0x301 binary ping frame construction, and parser slicing at boot."""
    assert crc8_sae_j1850(b"123456789") == 0x4B, "CRC8 selftest failed"
    req = build_tue_frame(0x01, CAN_ID_HAPTIC_PING, 5, b"\x5A\x00\x00\x00\x00\x00\x00\x00")
    assert len(req) == 20 and crc8_sae_j1850(req[2:17]) == req[17], "0x301 req build failed"
    rep = build_ping_reply(5, 0x5A)
    assert len(rep) == 20 and rep[2] == 0x02 and rep[8] == 5 and rep[9] == 0x5A, "0x301 rep build failed"
    assert crc8_sae_j1850(rep[2:17]) == rep[17], "0x301 rep CRC failed"


pca_a2.value(1)
pca_a1.value(1)
pca_a0.value(1)
pca_reset_device()

run_boot_selftest()
set_state(STATE_IDLE)
vent_all_chambers()
send("HAPTIC_SLAVE_READY (UART2 @ 115200 baud, Binary 20B 0x300/0x301, Anti-Error Failsafe Active)")
send("UART2 tx=GPIO{} rx=GPIO{} baud={} | console: >G< >R< >S< >P< >U< >B< >T<".format(
    UART_TX_PIN, UART_RX_PIN, UART_BAUDRATE))


# ==============================================================================
# MAIN EXECUTION LOOP
# ==============================================================================
while True:
    try:
        # 1. Xử lý gói tin nhị phân từ UART2 với bộ lọc chống nhiễu & CRC
        process_uart2()

        # 1b. Optional heartbeat on UART2 (CAN 0x303 ASYNC, off by default in polling mode)
        if (beacon_enabled and
                time.ticks_diff(time.ticks_ms(), last_beacon_ms) >= BEACON_PERIOD_MS):
            send_beacon()

        # 2. Giám sát Watchdog (Tự động xả khí về 0V nếu mất tín hiệu > 150ms)
        check_watchdog()

        # 3. Cập nhật đèn LED trạng thái (Xanh = Idle, Xanh dương thở = Đang nhận tín hiệu, Đỏ = Lỗi)
        update_led()

        # 4. Hỗ trợ lệnh chẩn đoán USB Console
        process_usb_serial()
    except Exception as exc:
        # Never let one bad iteration kill the UART2 link.
        try:
            vent_all_chambers()
        except Exception:
            pass
        try:
            set_state(STATE_ERROR)
        except Exception:
            pass
        try:
            send("LOOP ERROR: {}".format(exc))
        except Exception:
            pass
        time.sleep_ms(50)
