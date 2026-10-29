/**
  ******************************************************************************
  * @file    tue_haptic_bridge.c
  * @brief   USB CDC -> UART haptic telemetry forwarding engine (polling mode).
  ******************************************************************************
  * @attention
  *
  * Transmission: blocking HAL_UART_Transmit() at 115200 baud. The USB protocol
  * task blocks for about 1.7 ms per 20-byte frame.
  *
  * Reception: blocking HAL_UART_Receive() only while a ping transaction is
  * active. No DMA and no USART2 interrupts are used.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_haptic_bridge.h"

#include <string.h>

#include "tue_usb_binary.h"   /* tue_usb_crc8() */

/* Private variables ---------------------------------------------------------*/
volatile tue_haptic_payload_t g_tue_haptic_payload;
volatile uint32_t g_tue_haptic_timestamp_ms = 0U;
volatile uint32_t g_tue_haptic_frame_count  = 0U;
volatile uint32_t g_tue_haptic_tx_drops     = 0U;
volatile uint32_t g_tue_haptic_slip_alerts  = 0U;
volatile uint32_t g_tue_haptic_rx_bytes     = 0U;
volatile uint32_t g_tue_haptic_rx_errors    = 0U;
volatile uint32_t g_tue_haptic_ping_tx_fail = 0U;
volatile uint32_t g_tue_haptic_ping_timeouts = 0U;
volatile uint32_t g_tue_haptic_ping_replies = 0U;
volatile uint32_t g_tue_haptic_last_ping_rx_bytes = 0U;
volatile uint8_t  g_tue_haptic_last_tx_ok   = 0U;

static bool s_initialized = false;

/* Private helpers -----------------------------------------------------------*/

static void uart_clear_error_flags(void)
{
    __HAL_UART_CLEAR_OREFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_NEFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_FEFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_PEFLAG(&TUE_HAPTIC_UART);
}

static void uart_drain_rx(void)
{
    while (__HAL_UART_GET_FLAG(&TUE_HAPTIC_UART, UART_FLAG_RXNE)) {
        (void)TUE_HAPTIC_UART.Instance->DR;
    }
    uart_clear_error_flags();
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

bool tue_haptic_init(void)
{
    if (s_initialized) {
        return true;
    }

    /* Polling mode: make sure the USART2 interrupt cannot fire unexpectedly. */
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    uart_clear_error_flags();
    uart_drain_rx();

    s_initialized = true;
    return true;
}

bool tue_haptic_forward(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len != TUE_HAPTIC_FRAME_LEN) || !s_initialized) {
        return false;
    }

    if (HAL_UART_Transmit(&TUE_HAPTIC_UART, (uint8_t *)frame, len, 20U) != HAL_OK) {
        g_tue_haptic_tx_drops++;
        return false;
    }
    return true;
}

/* USART2 ASCII ping: send "PING\n" and wait for a "PONG ...\n" reply.
 * The reply text (up to 8 bytes) is returned in out for the USB response. */
bool tue_haptic_ping(uint8_t seq, uint8_t token, uint8_t *out)
{
    static const uint8_t ping_msg[] = "PING\n";
    uint8_t line[24];
    uint8_t line_len = 0U;
    uint16_t rx_seen = 0U;

    (void)seq;
    (void)token;

    g_tue_haptic_last_ping_rx_bytes = 0U;

    if ((out == NULL) || !s_initialized) {
        g_tue_haptic_ping_timeouts++;
        return false;
    }

    /* Discard stale bytes so a late reply to a previous ping cannot match. */
    uart_drain_rx();

    HAL_StatusTypeDef status = HAL_UART_Transmit(&TUE_HAPTIC_UART,
                                                 (uint8_t *)ping_msg,
                                                 (uint16_t)(sizeof(ping_msg) - 1U),
                                                 20U);
    g_tue_haptic_last_tx_ok = (status == HAL_OK) ? 1U : 0U;
    if (status != HAL_OK) {
        g_tue_haptic_ping_tx_fail++;
        return false;
    }

    /* Polling receive: one byte at a time until the line ends or the overall
     * timeout expires. HAL_UART_Receive() spins on RXNE, so no USART2 IRQ is
     * involved and consecutive bytes cannot be missed. */
    uint32_t deadline = HAL_GetTick() + TUE_HAPTIC_PING_TIMEOUT_MS;
    while ((int32_t)(deadline - HAL_GetTick()) > 0) {
        uint8_t b = 0U;

        if (HAL_UART_Receive(&TUE_HAPTIC_UART, &b, 1U, 2U) == HAL_OK) {
            rx_seen++;
            g_tue_haptic_rx_bytes++;
            g_tue_haptic_last_ping_rx_bytes = (rx_seen > 255U) ? 255U : rx_seen;

            if ((b == (uint8_t)'\n') || (b == (uint8_t)'\r')) {
                if ((line_len >= 4U) && (memcmp(line, "PONG", 4U) == 0)) {
                    memset(out, 0, TUE_HAPTIC_PAYLOAD_LEN);
                    uint8_t n = (line_len > TUE_HAPTIC_PAYLOAD_LEN)
                                    ? TUE_HAPTIC_PAYLOAD_LEN : line_len;
                    memcpy(out, line, n);
                    g_tue_haptic_ping_replies++;
                    return true;
                }
                line_len = 0U;   /* not a PONG line */
            } else if (line_len < sizeof(line)) {
                line[line_len++] = b;
            } else {
                line_len = 0U;
            }
        } else if (TUE_HAPTIC_UART.ErrorCode != HAL_UART_ERROR_NONE) {
            g_tue_haptic_rx_errors++;
            uart_clear_error_flags();
        }
    }

    g_tue_haptic_ping_timeouts++;
    return false;
}

bool tue_haptic_ingest(const uint8_t *raw, uint16_t len)
{
    tue_haptic_wire_frame_t f;

    if ((raw == NULL) || (len != TUE_HAPTIC_FRAME_LEN)) {
        return false;
    }

    memcpy(&f, raw, sizeof(f));

    if ((f.sof1 != TUE_HAPTIC_SOF1) || (f.sof2 != TUE_HAPTIC_SOF2) ||
        (f.eof1 != TUE_HAPTIC_EOF1) || (f.eof2 != TUE_HAPTIC_EOF2) ||
        (f.frame_type != TUE_HAPTIC_FRAME_TYPE_REQ) ||
        (f.can_id != TUE_HAPTIC_CAN_ID) ||
        (f.dlc != TUE_HAPTIC_PAYLOAD_LEN)) {
        return false;
    }

    if (tue_usb_crc8(&raw[2], (size_t)(TUE_HAPTIC_HEADER_LEN + TUE_HAPTIC_PAYLOAD_LEN))
        != f.crc8) {
        return false;
    }

    /* Local telemetry cache for reflex control / logging: build off-stack and
     * publish atomically so a concurrent reader never sees half a frame. */
    tue_haptic_payload_t cache;
    cache.fz_left       = f.payload.fz_left;
    cache.fz_right      = f.payload.fz_right;
    cache.f_shear_avg   = f.payload.f_shear_avg;
    cache.weight_tip    = f.payload.weight_tip;
    cache.weight_mid    = f.payload.weight_mid;
    cache.weight_base   = f.payload.weight_base;
    cache.status_flags  = f.payload.status_flags;
    cache.frame_counter = f.payload.frame_counter;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_tue_haptic_payload      = cache;
    g_tue_haptic_timestamp_ms = HAL_GetTick();
    __set_PRIMASK(primask);

    g_tue_haptic_frame_count++;

    if ((f.payload.status_flags & TUE_HAPTIC_STATUS_SLIP_ALERT) != 0U) {
        g_tue_haptic_slip_alerts++;
    }

    /* Transparent pass-through of the exact 20 raw bytes. */
    return tue_haptic_forward(raw, len);
}

void tue_haptic_reset_stats(void)
{
    g_tue_haptic_timestamp_ms = 0U;
    g_tue_haptic_frame_count  = 0U;
    g_tue_haptic_tx_drops     = 0U;
    g_tue_haptic_slip_alerts  = 0U;
    g_tue_haptic_rx_bytes     = 0U;
    g_tue_haptic_rx_errors    = 0U;
    g_tue_haptic_ping_tx_fail = 0U;
    g_tue_haptic_ping_timeouts = 0U;
    g_tue_haptic_ping_replies = 0U;
    g_tue_haptic_last_ping_rx_bytes = 0U;
    g_tue_haptic_last_tx_ok   = 0U;

    tue_haptic_payload_t zero = {0};
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_tue_haptic_payload = zero;
    __set_PRIMASK(primask);
}

void tue_haptic_get_payload(tue_haptic_payload_t *out)
{
    if (out == NULL) {
        return;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = g_tue_haptic_payload;
    __set_PRIMASK(primask);
}

void tue_haptic_get_diag(uint8_t *out)
{
    if (out == NULL) {
        return;
    }
    /* bit0 init, bit1 RX ready (polling), bit3 last TX ok, bit4 ping saw bytes */
    out[0] = (uint8_t)((s_initialized ? 0x01U : 0U) |
                       (s_initialized ? 0x02U : 0U) |
                       (g_tue_haptic_last_tx_ok ? 0x08U : 0U) |
                       (g_tue_haptic_last_ping_rx_bytes ? 0x10U : 0U));
    out[1] = (uint8_t)(g_tue_haptic_rx_errors & 0xFFU);
    out[2] = (uint8_t)(g_tue_haptic_rx_bytes & 0xFFU);
    out[3] = (uint8_t)((g_tue_haptic_rx_bytes >> 8) & 0xFFU);
    out[4] = (uint8_t)(g_tue_haptic_last_ping_rx_bytes & 0xFFU);
    out[5] = (uint8_t)(g_tue_haptic_ping_tx_fail & 0xFFU);
    out[6] = (uint8_t)(g_tue_haptic_ping_timeouts & 0xFFU);
    out[7] = (uint8_t)(g_tue_haptic_ping_replies & 0xFFU);
}
