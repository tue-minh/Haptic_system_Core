#ifndef HAPTIC_CONFIG_H
#define HAPTIC_CONFIG_H

// ==============================================================================
// HAPTIC SLAVE CONTROLLER - ESP32 WROOM (C++ port, reference: src/main.py)
// ==============================================================================
// Hardware:
//   - ESP32-WROOM-32
//   - PCA9548A/TCA9548A 8-Channel I2C Multiplexer (Addr: 0x77)
//   - 5x MCP4725 12-bit DACs controlling 5 pneumatic valves:
//       Ch 0 (A): Bottom - Distal Tip
//       Ch 1 (B): Bottom - Medial Center
//       Ch 2 (C): Bottom - Proximal Base
//       Ch 3 (D): Lateral Left
//       Ch 4 (E): Lateral Right
// Communication:
//   - UART2 @ 115200 8-N-1 from STM32 Master (STM32 PD5 -> GPIO16 RX, PD6 <- GPIO17 TX)
//   - USB Serial: debug-only ASCII console (>G< >R< >S< >P< >U< >B< >T<)
//
// Safety rule: all valve outputs are forced to 0.0V at power-up and stay at
// 0.0V until a fully valid 0x300 haptic frame arrives over UART2.
// ==============================================================================

// LED Indicators (Active LOW)
#define LED_GREEN_PIN  4
#define LED_RED_PIN    13
#define LED_BLUE_PIN   14
#define LED_ON         LOW
#define LED_OFF        HIGH

// Blue LED PWM (LEDC)
#define BLUE_LEDC_CHANNEL   0
#define BLUE_LEDC_FREQ_HZ   1000
#define BLUE_LEDC_RES_BITS  16

// I2C Bus
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 400000

// PCA9548A / TCA9548A Multiplexer
#define PCA_ADDR      0x77
#define PCA_RESET_PIN 18
#define PCA_A2_PIN    25
#define PCA_A1_PIN    26
#define PCA_A0_PIN    27

// MCP4725 DACs (5 channels: A, B, C, D, E)
#define MCP4725_WRITE_DAC 0x40
#define VDD               3.30f
#define DAC_MAX           4095
#define DAC_COUNT         5
static constexpr uint8_t MCP4725_ADDR[DAC_COUNT] = {
    0x60, 0x60, 0x60, 0x61, 0x61
};

// UART2 (From STM32 Master - Normal UART @ 115200 8-N-1)
#define UART_BAUDRATE 115200
#define UART_TX_PIN   17
#define UART_RX_PIN   16
#define UART_RX_BUF   512

// Maximum idle gap before discarding an incomplete (< 20 B) frame fragment
#define STALE_RX_TIMEOUT_MS 25

// Firmware identity / binary ping protocol (CAN 0x301 over the STM32 UART link)
#define FW_VERSION_MAJOR      1
#define FW_VERSION_MINOR      0
#define PING_PROTOCOL_VERSION 1

// CAN IDs
#define CAN_ID_HAPTIC_TELEMETRY 0x300UL
#define CAN_ID_HAPTIC_PING      0x301UL
#define BEACON_CAN_ID           0x303UL
#define BEACON_PERIOD_MS        1000UL

// TUE binary frame constants (20 bytes)
#define TUE_FRAME_LEN   20
#define TUE_PAYLOAD_LEN 8
#define TUE_SOF1        0xAA
#define TUE_SOF2        0x55
#define TUE_EOF1        0x55
#define TUE_EOF2        0xAA
#define FRAME_TYPE_REQ     0x01
#define FRAME_TYPE_RESP_OK 0x02
#define FRAME_TYPE_ASYNC   0x04

// Haptic Safety & Failsafe Parameters
#define V_MIN          0.70f   // Minimum opening voltage / mechanical deadband
#define V_MAX          2.20f   // ABSOLUTE SAFETY LIMIT - never above this
#define F_THRESH       0.20f   // Noise threshold for contact force (Newtons)
#define F_RATED        20.00f  // Rated full-scale physical force (Newtons)
#define V_WRITE_DELTA  0.015f   // Min voltage change before rewriting the DAC

#define WATCHDOG_TIMEOUT_MS    150UL // Lost UART signal > 150 ms -> vent to 0.0V
#define CONSECUTIVE_ERRORS_MAX 5     // 5 consecutive CRC/frame errors -> vent to 0.0V
#define MAX_RX_BUFFER_SIZE     100   // Garbage byte threshold per parse pass
#define RECOVERY_FRAMES        3     // Good frames required to leave STATE_ERROR

// LED breathing animation
#define BREATH_PERIOD_MS 1600UL

#endif // HAPTIC_CONFIG_H
