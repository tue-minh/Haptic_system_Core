/**
  ******************************************************************************
  * @file    tue_haptic_bridge.c
  * @brief   USB CDC -> UART haptic telemetry forwarding engine (polling mode).
  ******************************************************************************
  * @attention
  *
  * Pure normal/blocking register polling on USART2 (PD5 = TX, PD6 = RX) at
  * 115200 8-N-1. No DMA and no USART2 interrupts are used at any time.
  *
  * Both 0x300 (haptic telemetry) and 0x301 (slave ping request/response) use
  * the 20-byte TUE binary wire frame protected by SAE J1850 CRC-8.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_haptic_bridge.h"

#include <string.h>

#include "tue_usb_binary.h"   /* tue_usb_crc8() */

/* Private variables ---------------------------------------------------------*/
volatile tue_haptic_payload_t g_tue_haptic_payload;
volatile uint32_t g_tue_haptic_timestamp_ms       = 0U;
volatile uint32_t g_tue_haptic_frame_count        = 0U;
volatile uint32_t g_tue_haptic_tx_drops           = 0U;
volatile uint32_t g_tue_haptic_slip_alerts        = 0U;
volatile uint32_t g_tue_haptic_rx_bytes           = 0U;
volatile uint32_t g_tue_haptic_rx_errors          = 0U;
volatile uint32_t g_tue_haptic_ping_tx_fail       = 0U;
volatile uint32_t g_tue_haptic_ping_timeouts      = 0U;
volatile uint32_t g_tue_haptic_ping_replies       = 0U;
volatile uint32_t g_tue_haptic_last_ping_rx_bytes = 0U;
volatile uint8_t  g_tue_haptic_last_tx_ok         = 0U;

static bool s_initialized = false;

/* Private helpers -----------------------------------------------------------*/

/**
  * @brief  Poll USART2->SR / DR once. On STM32F4, reading SR followed by DR
  *         both retrieves the incoming byte and clears ORE/NE/FE/PE flags.
  * @param  out_byte  Receives the byte when RXNE was set.
  * @retval true if a valid byte was read from DR.
  */
static bool uart_poll_rx_byte(uint8_t *out_byte)
{
    USART_TypeDef *u = TUE_HAPTIC_UART.Instance;
    uint32_t sr = u->SR;
    const uint32_t err_mask = (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);

    if ((sr & (USART_SR_RXNE | err_mask)) == 0U) {
        return false;
    }

    /* Always read DR after SR to clear any error flags and fetch the data byte. */
    uint8_t b = (uint8_t)(u->DR & 0xFFU);

    if ((sr & err_mask) != 0U) {
        g_tue_haptic_rx_errors++;
    }

    if ((sr & USART_SR_RXNE) != 0U) {
        g_tue_haptic_rx_bytes++;
        if (out_byte != NULL) {
            *out_byte = b;
        }
        return true;
    }

    return false;
}

/**
  * @brief  Drain any pending bytes and error flags from USART2 while still
  *         counting them in diagnostic counters.
  */
static void uart_drain_rx(void)
{
    uint8_t dummy = 0U;
    for (uint16_t i = 0U; i < 256U; i++) {
        if (!uart_poll_rx_byte(&dummy)) {
            break;
        }
    }
}

/**
  * @brief  Blocking register-level UART transmit at 115200 8-N-1.
  *         Does not rely on HAL gState, so it can never get stuck in HAL_BUSY.
  */
static bool uart_write_bytes(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if ((data == NULL) || (len == 0U)) {
        return false;
    }

    USART_TypeDef *u = TUE_HAPTIC_UART.Instance;
    uint32_t start = HAL_GetTick();

    /* Ensure transmitter and receiver are enabled in CR1. */
    if ((u->CR1 & (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) !=
        (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE)) {
        u->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
    }

    for (uint16_t i = 0U; i < len; i++) {
        while ((u->SR & USART_SR_TXE) == 0U) {
            if ((HAL_GetTick() - start) >= timeout_ms) {
                g_tue_haptic_last_tx_ok = 0U;
                return false;
            }
        }
        u->DR = (uint8_t)data[i];
    }

    while ((u->SR & USART_SR_TC) == 0U) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            g_tue_haptic_last_tx_ok = 0U;
            return false;
        }
    }

    g_tue_haptic_last_tx_ok = 1U;
    return true;
}

/**
  * @brief  Build a 20-byte TUE binary wire frame with an 8-byte payload.
  */
static void build_wire_frame8(uint8_t *dst, uint8_t frame_type, uint32_t can_id,
                              uint8_t seq, const uint8_t *payload8)
{
    dst[0] = TUE_HAPTIC_SOF1;
    dst[1] = TUE_HAPTIC_SOF2;
    dst[2] = frame_type;
    dst[3] = (uint8_t)(can_id & 0xFFU);
    dst[4] = (uint8_t)((can_id >> 8) & 0xFFU);
    dst[5] = (uint8_t)((can_id >> 16) & 0xFFU);
    dst[6] = (uint8_t)((can_id >> 24) & 0xFFU);
    dst[7] = TUE_HAPTIC_PAYLOAD_LEN;
    dst[8] = seq;
    memcpy(&dst[9], payload8, TUE_HAPTIC_PAYLOAD_LEN);
    dst[17] = tue_usb_crc8(&dst[2],
                           (size_t)(TUE_HAPTIC_HEADER_LEN + TUE_HAPTIC_PAYLOAD_LEN));
    dst[18] = TUE_HAPTIC_EOF1;
    dst[19] = TUE_HAPTIC_EOF2;
}

/**
  * @brief  Scan a sliding byte buffer for a valid 20-byte 0x301 RESP_OK frame
  *         matching seq and token.
  */
static bool find_ping_reply_frame(const uint8_t *buf, uint8_t len,
                                  uint8_t seq, uint8_t token, uint8_t *out)
{
    if ((buf == NULL) || (len < TUE_HAPTIC_FRAME_LEN)) {
        return false;
    }

    uint8_t limit = (uint8_t)(len - TUE_HAPTIC_FRAME_LEN);
    for (uint8_t i = 0U; i <= limit; i++) {
        const uint8_t *p = &buf[i];
        if ((p[0] != TUE_HAPTIC_SOF1) || (p[1] != TUE_HAPTIC_SOF2) ||
            (p[18] != TUE_HAPTIC_EOF1) || (p[19] != TUE_HAPTIC_EOF2)) {
            continue;
        }
        if (p[2] != TUE_HAPTIC_FRAME_TYPE_RESP_OK) {
            continue;
        }
        uint32_t can_id = (uint32_t)p[3] |
                          ((uint32_t)p[4] << 8) |
                          ((uint32_t)p[5] << 16) |
                          ((uint32_t)p[6] << 24);
        if ((can_id != TUE_HAPTIC_CAN_PING) ||
            (p[7] != TUE_HAPTIC_PAYLOAD_LEN) ||
            (p[8] != seq) ||
            (p[9] != token)) {
            continue;
        }
        uint8_t crc = tue_usb_crc8(
            &p[2], (size_t)(TUE_HAPTIC_HEADER_LEN + TUE_HAPTIC_PAYLOAD_LEN));
        if (crc != p[17]) {
            continue;
        }

        memcpy(out, &p[9], TUE_HAPTIC_PAYLOAD_LEN);
        return true;
    }

    return false;
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

bool tue_haptic_init(void)
{
    USART_TypeDef *u = TUE_HAPTIC_UART.Instance;

    /* Pure polling mode: disable all USART2 interrupts and DMA requests. */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);

    u->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TCIE | USART_CR1_TXEIE |
                USART_CR1_PEIE   | USART_CR1_IDLEIE);
    u->CR3 &= ~(USART_CR3_EIE | USART_CR3_DMAT | USART_CR3_DMAR);
    u->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

    TUE_HAPTIC_UART.gState  = HAL_UART_STATE_READY;
    TUE_HAPTIC_UART.RxState = HAL_UART_STATE_READY;

    /* Flush any boot glitch bytes without counting them as traffic. */
    (void)u->SR;
    (void)u->DR;

    s_initialized = true;
    return true;
}

bool tue_haptic_forward(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len != TUE_HAPTIC_FRAME_LEN) || !s_initialized) {
        return false;
    }

    if (!uart_write_bytes(frame, len, 20U)) {
        g_tue_haptic_tx_drops++;
        return false;
    }
    return true;
}

bool tue_haptic_ping(uint8_t seq, uint8_t token, uint8_t *out)
{
    uint8_t req_payload[TUE_HAPTIC_PAYLOAD_LEN];
    uint8_t req_frame[TUE_HAPTIC_FRAME_LEN];
    uint8_t rx_window[40];
    uint16_t rx_seen = 0U;

    g_tue_haptic_last_ping_rx_bytes = 0U;

    if ((out == NULL) || !s_initialized) {
        g_tue_haptic_ping_timeouts++;
        return false;
    }

    memset(req_payload, 0, sizeof(req_payload));
    req_payload[0] = token;
    build_wire_frame8(req_frame, TUE_HAPTIC_FRAME_TYPE_REQ,
                      TUE_HAPTIC_CAN_PING, seq, req_payload);

    /* Harvest any stale bytes before starting the ping transaction. */
    uart_drain_rx();

    for (uint8_t attempt = 0U; attempt < TUE_HAPTIC_PING_ATTEMPTS; attempt++) {
        uint8_t win_len = 0U;

        if (!uart_write_bytes(req_frame, TUE_HAPTIC_FRAME_LEN, 20U)) {
            g_tue_haptic_ping_tx_fail++;
            return false;
        }

        uint32_t deadline = HAL_GetTick() + TUE_HAPTIC_PING_TIMEOUT_MS;
        while ((int32_t)(deadline - HAL_GetTick()) > 0) {
            uint8_t b = 0U;
            if (!uart_poll_rx_byte(&b)) {
                continue;
            }

            rx_seen++;
            g_tue_haptic_last_ping_rx_bytes = (rx_seen > 255U) ? 255U : rx_seen;

            if (win_len < (uint8_t)sizeof(rx_window)) {
                rx_window[win_len++] = b;
            } else {
                memmove(&rx_window[0], &rx_window[1], sizeof(rx_window) - 1U);
                rx_window[sizeof(rx_window) - 1U] = b;
            }

            if (find_ping_reply_frame(rx_window, win_len, seq, token, out)) {
                g_tue_haptic_ping_replies++;
                return true;
            }
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
    g_tue_haptic_timestamp_ms       = 0U;
    g_tue_haptic_frame_count        = 0U;
    g_tue_haptic_tx_drops           = 0U;
    g_tue_haptic_slip_alerts        = 0U;
    g_tue_haptic_rx_bytes           = 0U;
    g_tue_haptic_rx_errors          = 0U;
    g_tue_haptic_ping_tx_fail       = 0U;
    g_tue_haptic_ping_timeouts      = 0U;
    g_tue_haptic_ping_replies       = 0U;
    g_tue_haptic_last_ping_rx_bytes = 0U;
    g_tue_haptic_last_tx_ok         = 0U;

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

    /* Harvest any byte sitting in USART2->DR before reporting diagnostics. */
    if (s_initialized) {
        uart_drain_rx();
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
