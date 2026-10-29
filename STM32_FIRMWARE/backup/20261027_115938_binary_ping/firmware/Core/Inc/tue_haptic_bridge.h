/**
  ******************************************************************************
  * @file    tue_haptic_bridge.h
  * @brief   USB CDC -> UART haptic telemetry forwarding bridge.
  ******************************************************************************
  * @attention
  *
  * The host PC streams the unified 20-byte tactile frame (CAN ID 0x300,
  * DLC = 8) to the STM32 over USB CDC at 100 Hz. The firmware:
  *
  *   1. ingests the frame in the tue_usb protocol task,
  *   2. caches the 8-byte payload in local telemetry state, and
  *   3. forwards the exact 20 raw bytes to the ESP32 slave over USART2
  *      (PD5 = TX, PD6 = RX) at 250 kbaud 8-N-1 using TX DMA.
  *
  * CAN ID 0x301 is the slave ping: the host sends a request over USB, the STM32
  * forwards it to the ESP32 on USART2, waits up to TUE_HAPTIC_PING_TIMEOUT_MS
  * for the RESP_OK reply, and relays the 8-byte status payload back to the host.
  * Reply layout: token, protocol, fw major, fw minor, state, last haptic frame
  * counter, consecutive errors, uptime seconds (mod 256).
  *
  * Wire frame (identical on USB and UART, all multi-byte fields LE):
  *
  *   +------+------+------+-----------+-----+-----+---------+------+------+------+
  *   | SOF1 | SOF2 | TYPE | CAN_ID 4B | DLC | SEQ | PAYLOAD | CRC8 | EOF1 | EOF2 |
  *   | 0xAA | 0x55 | 0x01 | 0x00000300| 0x08| u8  | 8 bytes | J1850| 0x55 | 0xAA |
  *   +------+------+------+-----------+-----+-----+---------+------+------+------+
  *
  * Zero-blocking contract: the USB task never waits on the UART. Frames are
  * copied into a 2-slot ping-pong buffer and dispatched through HAL TX DMA.
  * A frame that arrives while the previous 800 us DMA transfer is still in
  * flight replaces the pending slot (newest wins); nothing ever blocks.
  *
  * USART2 is the dedicated slave link. Its RX path is armed once with a
  * circular DMA buffer (128 bytes) terminated by the IDLE line event.
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

/** CAN identifier of the UART slave ping request/response. */
#define TUE_HAPTIC_CAN_PING              0x00000301U

/** CAN identifier of the STM32-side UART link diagnostics. */
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

/** Slave ping: protocol version and reply timeout. */
#define TUE_HAPTIC_PING_PROTOCOL_VERSION 0x01U
#define TUE_HAPTIC_PING_TIMEOUT_MS       50U

/** Status_Flags bitmask (payload byte 6). */
#define TUE_HAPTIC_STATUS_LEFT_CONTACT   0x01U
#define TUE_HAPTIC_STATUS_RIGHT_CONTACT  0x02U
#define TUE_HAPTIC_STATUS_SLIP_ALERT     0x04U
#define TUE_HAPTIC_STATUS_TARE_ACTIVE    0x08U

/** Slave link parameters. */
#define TUE_HAPTIC_UART_BAUD             250000U
#define TUE_HAPTIC_RX_RING_SIZE          128U
#define TUE_HAPTIC_TX_SLOTS              2U

/** Maximum time an in-flight TX DMA may take before it is treated as wedged.
 *  A 20-byte frame needs 800 us at 250 kbaud, so 5 ms is a 6x margin. */
#define TUE_HAPTIC_TX_TIMEOUT_MS         5U

/** USART2 is the dedicated ESP32 slave link. */
#define TUE_HAPTIC_UART                  huart2
#define TUE_HAPTIC_UART_TX_DMA           hdma_usart2_tx
#define TUE_HAPTIC_UART_RX_DMA           hdma_usart2_rx

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
    uint8_t frame_type;     /**< 0x01 = FRAME_TYPE_REQ                         */
    uint32_t can_id;        /**< 0x00000300, little-endian                     */
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

/** Number of frames dropped because both TX slots were occupied. */
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
  * @brief  Initialise the slave link: arm circular RX DMA with IDLE-line
  *         termination and disable the half-transfer interrupt.
  * @retval true always; TX forwarding is enabled even if the RX arm fails.
  * @note   A failed RX arm increments g_tue_haptic_rx_errors but does not
  *         block haptic forwarding. USART2 itself is configured by
  *         MX_USART2_UART_Init() during boot.
  */
bool tue_haptic_init(void);

/**
  * @brief  Validate and ingest one raw 20-byte frame, then forward it to the
  *         slave over UART TX DMA (non-blocking).
  * @param  raw  Pointer to the exact 20 bytes received from USB.
  * @param  len  Must be TUE_HAPTIC_FRAME_LEN.
  * @retval true when the frame was accepted (forwarded or queued).
  */
bool tue_haptic_ingest(const uint8_t *raw, uint16_t len);

/**
  * @brief  Forward a raw frame without touching the telemetry cache.
  * @retval true when the frame was accepted (forwarded or queued).
  */
bool tue_haptic_forward(const uint8_t *frame, uint16_t len);

/**
  * @brief  Ping the ESP32 slave over USART2 and wait for its reply.
  *         Sends a CAN 0x301 request frame and parses the RESP_OK reply.
  * @param  seq    Frame sequence echoed by the slave and matched here.
  * @param  token  8-bit token echoed back in reply byte 0.
  * @param  out    Receives the 8-byte reply payload (must not be NULL).
  * @retval true when a valid reply arrived within TUE_HAPTIC_PING_TIMEOUT_MS.
  * @note   Blocks the calling task for at most the ping timeout. Haptic frames
  *         that arrive meanwhile are queued in the ping-pong TX slots.
  */
bool tue_haptic_ping(uint8_t seq, uint8_t token, uint8_t *out);

/**
  * @brief  Fill 8 bytes of UART link diagnostics (CAN 0x302).
  *         [0] flags: bit0 init, bit1 rx_armed, bit2 tx_busy,
  *             bit3 last TX ok, bit4 last ping saw bytes
  *         [1] rx error count      [2..3] total RX bytes (LE16)
  *         [4] last ping RX bytes  [5] ping TX failures
  *         [6] ping timeouts       [7] ping replies
  */
void tue_haptic_get_diag(uint8_t *out);

/**
  * @brief  Reset the RX byte ring and counters (keeps DMA armed).
  */
void tue_haptic_reset_stats(void);

/**
  * @brief  Copy the cached 8-byte payload atomically.
  * @param  out  Destination, filled with a consistent snapshot.
  * @note   Safe to call from any task; masks interrupts for the 8-byte copy.
  */
void tue_haptic_get_payload(tue_haptic_payload_t *out);

/**
  * @brief  Number of bytes currently buffered from the slave.
  */
uint16_t tue_haptic_rx_available(void);

/**
  * @brief  Pop one byte received from the slave.
  * @retval false when the ring is empty.
  */
bool tue_haptic_rx_pop(uint8_t *byte);

/* IRQ-context entry points, called from the shared HAL weak callbacks. */
void tue_haptic_uart_tx_cplt_cb(UART_HandleTypeDef *huart);
void tue_haptic_uart_rx_event_cb(UART_HandleTypeDef *huart, uint16_t size);
void tue_haptic_uart_error_cb(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* TUE_HAPTIC_BRIDGE_H */
