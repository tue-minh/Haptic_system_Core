/**
  ******************************************************************************
  * @file    tue_haptic_bridge.h
  * @brief   USB CDC -> UART haptic telemetry forwarding bridge (polling mode).
  ******************************************************************************
  * @attention
  *
  * The host PC streams the unified 20-byte tactile frame (CAN ID 0x300,
  * DLC = 8) to the STM32 over USB CDC at 100 Hz. The firmware:
  *
  *   1. ingests the frame in the tue_usb protocol task,
  *   2. caches the 8-byte payload in local telemetry state, and
  *   3. forwards the exact 20 raw bytes to the ESP32 slave over USART2
  *      (PD5 = TX, PD6 = RX) at 115200 baud 8-N-1 using polling TX.
  *
  * CAN ID 0x301 is the binary slave ping: the host sends a request over USB,
  * the STM32 sends a 20-byte TUE binary ping frame (CAN ID 0x301, TYPE = 0x01)
  * on USART2, polls for the 20-byte RESP_OK (TYPE = 0x02, CAN ID = 0x301)
  * reply from the ESP32 within TUE_HAPTIC_PING_TIMEOUT_MS, and relays the
  * 8-byte slave status payload back to the host in the USB RESP_OK frame.
  *
  * Wire frame (identical on USB and UART, all multi-byte fields LE):
  *
  *   +------+------+------+-----------+-----+-----+---------+------+------+------+
  *   | SOF1 | SOF2 | TYPE | CAN_ID 4B | DLC | SEQ | PAYLOAD | CRC8 | EOF1 | EOF2 |
  *   | 0xAA | 0x55 | u8   | u32 LE    | 0x08| u8  | 8 bytes | J1850| 0x55 | 0xAA |
  *   +------+------+------+-----------+-----+-----+---------+------+------+------+
  *
  * USART2 uses normal register polling only (no DMA, no UART interrupts).
  *
  ******************************************************************************
  */

#ifndef TUE_HAPTIC_BRIDGE_H
#define TUE_HAPTIC_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "usart.h"

/* ==========================================================================
 * Protocol constants
 * ========================================================================== */

/** CAN identifier of the unified tactile telemetry frame. */
#define TUE_HAPTIC_CAN_ID                0x00000300U

/** CAN identifier of the binary slave ping request / response frame. */
#define TUE_HAPTIC_CAN_PING              0x00000301U

/** CAN identifier of the STM32-local USART2 link diagnostics frame. */
#define TUE_HAPTIC_CAN_DIAG              0x00000302U

/** Fixed wire frame size: 12 bytes framing + 8 bytes payload. */
#define TUE_HAPTIC_FRAME_LEN             20U
#define TUE_HAPTIC_PAYLOAD_LEN           8U
#define TUE_HAPTIC_HEADER_LEN            7U
#define TUE_HAPTIC_SOF1                  0xAAU
#define TUE_HAPTIC_SOF2                  0x55U
#define TUE_HAPTIC_EOF1                  0x55U
#define TUE_HAPTIC_EOF2                  0xAAU
#define TUE_HAPTIC_FRAME_TYPE_REQ        0x01U
#define TUE_HAPTIC_FRAME_TYPE_RESP_OK    0x02U

/** Ping protocol version carried in payload[1] of 0x301 RESP_OK. */
#define TUE_HAPTIC_PING_PROTOCOL_VERSION 0x01U

/** Status_Flags bitmask (payload byte 6). */
#define TUE_HAPTIC_STATUS_LEFT_CONTACT   0x01U
#define TUE_HAPTIC_STATUS_RIGHT_CONTACT  0x02U
#define TUE_HAPTIC_STATUS_SLIP_ALERT     0x04U
#define TUE_HAPTIC_STATUS_TARE_ACTIVE    0x08U

/** Slave link parameters (normal UART at 115200 8-N-1, no IRQ, no DMA). */
#define TUE_HAPTIC_UART_BAUD             115200U

/** Binary slave ping reply timeout per attempt (ms) and max attempts. */
#define TUE_HAPTIC_PING_TIMEOUT_MS       90U
#define TUE_HAPTIC_PING_ATTEMPTS         2U

/** USART2 is the dedicated ESP32 slave link (polling mode, no DMA/IRQ). */
#define TUE_HAPTIC_UART                  huart2

/* ==========================================================================
 * Wire structures
 * ========================================================================== */

#pragma pack(push, 1)

/** Unified haptic telemetry payload (8 bytes, matches the host contract). */
typedef struct {
    uint8_t fz_left;        /**< 0.1 N / LSB, 0..25.5 N -> chamber D            */
    uint8_t fz_right;       /**< 0.1 N / LSB, 0..25.5 N -> chamber E            */
    uint8_t f_shear_avg;    /**< 0.1 N / LSB, 0..25.5 N -> slip / weight       */
    uint8_t weight_tip;     /**< 0..255 -> 0.0..1.0, zone A (tip)              */
    uint8_t weight_mid;     /**< 0..255 -> 0.0..1.0, zone B (mid)              */
    uint8_t weight_base;    /**< 0..255 -> 0.0..1.0, zone C (base)             */
    uint8_t status_flags;   /**< TUE_HAPTIC_STATUS_* bitmask                  */
    uint8_t frame_counter;  /**< 0..255 rolling counter                       */
} tue_haptic_payload_t;

/** Complete 20-byte frame as it appears on the USB and UART wires. */
typedef struct {
    uint8_t sof1;           /**< 0xAA                                          */
    uint8_t sof2;           /**< 0x55                                          */
    uint8_t frame_type;     /**< 0x01 = FRAME_TYPE_REQ, 0x02 = RESP_OK         */
    uint32_t can_id;        /**< 0x00000300 / 0x00000301, little-endian        */
    uint8_t dlc;            /**< 0x08                                          */
    uint8_t seq;            /**< rolling sequence counter                      */
    tue_haptic_payload_t payload;
    uint8_t crc8;           /**< SAE J1850 CRC-8 over bytes 2..16               */
    uint8_t eof1;           /**< 0x55                                          */
    uint8_t eof2;           /**< 0xAA                                          */
} tue_haptic_wire_frame_t;

#pragma pack(pop)

_Static_assert(sizeof(tue_haptic_payload_t) == TUE_HAPTIC_PAYLOAD_LEN,
               "haptic payload must be exactly 8 bytes");
_Static_assert(sizeof(tue_haptic_wire_frame_t) == TUE_HAPTIC_FRAME_LEN,
               "haptic wire frame must be exactly 20 bytes");

/* ==========================================================================
 * Local telemetry cache (exposed for reflex control / logging)
 * ========================================================================== */

/** Last received payload. */
extern volatile tue_haptic_payload_t g_tue_haptic_payload;

/** HAL_GetTick() timestamp of the last accepted frame. */
extern volatile uint32_t g_tue_haptic_timestamp_ms;

/** Number of frames ingested from USB. */
extern volatile uint32_t g_tue_haptic_frame_count;

/** Number of frames dropped because UART TX timed out. */
extern volatile uint32_t g_tue_haptic_tx_drops;

/** Number of SLIP_ALERT events observed. */
extern volatile uint32_t g_tue_haptic_slip_alerts;

/** Number of bytes received from the ESP32 slave. */
extern volatile uint32_t g_tue_haptic_rx_bytes;

/** Number of UART error events observed on the slave link. */
extern volatile uint32_t g_tue_haptic_rx_errors;

/** Ping diagnostics (CAN 0x302). */
extern volatile uint32_t g_tue_haptic_ping_tx_fail;
extern volatile uint32_t g_tue_haptic_ping_timeouts;
extern volatile uint32_t g_tue_haptic_ping_replies;
extern volatile uint32_t g_tue_haptic_last_ping_rx_bytes;
extern volatile uint8_t  g_tue_haptic_last_tx_ok;

/* ==========================================================================
 * API
 * ========================================================================== */

/**
  * @brief  Initialise the polling slave link (disables the USART2 IRQ).
  * @retval true always.
  * @note   USART2 itself is configured by MX_USART2_UART_Init() during boot.
  */
bool tue_haptic_init(void);

/**
  * @brief  Validate and ingest one raw 20-byte frame, then forward it to the
  *         slave over UART TX (blocking).
  * @param  raw  Pointer to the exact 20 bytes received from USB.
  * @param  len  Must be TUE_HAPTIC_FRAME_LEN.
  * @retval true when the frame was transmitted.
  */
bool tue_haptic_ingest(const uint8_t *raw, uint16_t len);

/**
  * @brief  Forward a raw frame without touching the telemetry cache.
  * @retval true when the frame was transmitted.
  */
bool tue_haptic_forward(const uint8_t *frame, uint16_t len);

/**
  * @brief  Ping the ESP32 slave over USART2 using a 20-byte binary frame
  *         (CAN ID 0x301, TYPE = 0x01) and poll for the 20-byte RESP_OK
  *         (CAN ID 0x301, TYPE = 0x02) reply.
  * @param  seq    Sequence number of the request.
  * @param  token  Caller-chosen byte echoed in payload[0] of the reply.
  * @param  out    Receives the 8-byte slave status payload on success.
  * @retval true when a valid 20-byte RESP_OK arrived within timeout.
  * @note   Blocks the USB task for at most the ping timeout.
  */
bool tue_haptic_ping(uint8_t seq, uint8_t token, uint8_t *out);

/**
  * @brief  Fill 8 bytes of UART link diagnostics (CAN 0x302).
  *         [0] flags: bit0 init, bit1 RX ready, bit3 last TX ok,
  *             bit4 last ping saw bytes
  *         [1] rx error count      [2..3] total RX bytes (LE16)
  *         [4] last ping RX bytes  [5] ping TX failures
  *         [6] ping timeouts       [7] ping replies
  */
void tue_haptic_get_diag(uint8_t *out);

/**
  * @brief  Reset the link counters.
  */
void tue_haptic_reset_stats(void);

/**
  * @brief  Copy the cached 8-byte payload atomically.
  * @param  out  Destination, filled with a consistent snapshot.
  * @note   Safe to call from any task; masks interrupts for the 8-byte copy.
  */
void tue_haptic_get_payload(tue_haptic_payload_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TUE_HAPTIC_BRIDGE_H */
