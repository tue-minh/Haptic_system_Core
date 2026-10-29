/**
  ******************************************************************************
  * @file    tue_haptic_bridge.c
  * @brief   USB CDC -> UART haptic telemetry forwarding engine.
  ******************************************************************************
  * @attention
  *
  * Transmission: 2-slot ping-pong buffer + HAL_UART_Transmit_DMA(). Never
  * blocks and never hands a stack pointer to the DMA engine.
  *
  * Reception: one circular DMA buffer armed with
  * HAL_UARTEx_ReceiveToIdle_DMA(). The IDLE event reports the current DMA
  * write position, so the callback copies the delta into a byte ring that
  * application code can drain with tue_haptic_rx_pop().
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tue_haptic_bridge.h"

#include <string.h>

#include "cmsis_os2.h"
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

/* Transmit ping-pong buffers. Both live in .bss for the DMA lifetime. */
static uint8_t s_tx_slots[TUE_HAPTIC_TX_SLOTS][TUE_HAPTIC_FRAME_LEN];
static volatile uint8_t  s_tx_active   = 0U;
static volatile bool     s_tx_busy     = false;
static volatile bool     s_tx_pending  = false;
static volatile uint32_t s_tx_start_ms = 0U;

/* Receive path. */
static uint8_t s_rx_dma_buf[TUE_HAPTIC_RX_RING_SIZE];
static uint8_t s_rx_ring[TUE_HAPTIC_RX_RING_SIZE];
static volatile uint16_t s_rx_head     = 0U;
static volatile uint16_t s_rx_tail     = 0U;
static volatile uint16_t s_rx_last_pos = 0U;
static volatile bool     s_rx_armed    = false;

/* Private helpers -----------------------------------------------------------*/

static void rx_ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) & (TUE_HAPTIC_RX_RING_SIZE - 1U));
    if (next == s_rx_tail) {
        return; /* ring full: drop newest byte */
    }
    s_rx_ring[s_rx_head] = byte;
    s_rx_head = next;
    g_tue_haptic_rx_bytes++;
}

static void rx_arm(void)
{
    s_rx_last_pos = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&TUE_HAPTIC_UART, s_rx_dma_buf,
                                     (uint16_t)TUE_HAPTIC_RX_RING_SIZE) == HAL_OK) {
        __HAL_DMA_DISABLE_IT(TUE_HAPTIC_UART.hdmarx, DMA_IT_HT);
        s_rx_armed = true;
    } else {
        s_rx_armed = false;
        g_tue_haptic_rx_errors++;
    }
}

/* Start one queued TX slot. Caller must have IRQs masked or run in the DMA
 * ISR so the slot cannot be rewritten while the DMA engine reads it. */
static bool tx_start_slot(uint8_t slot)
{
    if (HAL_UART_Transmit_DMA(&TUE_HAPTIC_UART, s_tx_slots[slot],
                              TUE_HAPTIC_FRAME_LEN) != HAL_OK) {
        g_tue_haptic_tx_drops++;
        return false;
    }
    s_tx_active   = slot;
    s_tx_busy     = true;
    s_tx_start_ms = HAL_GetTick();
    return true;
}

static void rx_clear_flags(void)
{
    __HAL_UART_CLEAR_OREFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_NEFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_FEFLAG(&TUE_HAPTIC_UART);
    __HAL_UART_CLEAR_PEFLAG(&TUE_HAPTIC_UART);
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

bool tue_haptic_init(void)
{
    if (s_initialized) {
        return true;
    }

    s_tx_active   = 0U;
    s_tx_busy     = false;
    s_tx_pending  = false;
    s_tx_start_ms = 0U;

    s_rx_head     = 0U;
    s_rx_tail     = 0U;
    s_rx_last_pos = 0U;
    memset(s_rx_ring, 0, sizeof(s_rx_ring));

    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    rx_clear_flags();

    /* TX forwarding must work even when the (diagnostic) slave RX path cannot
     * be armed, so mark the bridge ready before attempting the DMA arm. */
    s_initialized = true;
    rx_arm();

    return true;
}

bool tue_haptic_forward(const uint8_t *frame, uint16_t len)
{
    if ((frame == NULL) || (len != TUE_HAPTIC_FRAME_LEN) || !s_initialized) {
        return false;
    }

    /* A transfer whose completion never arrives would otherwise leave
     * s_tx_busy set forever and silently stop all forwarding. Abort it and
     * drop back to idle; the next frame re-arms the engine. Done outside the
     * critical section because HAL_UART_AbortTransmit() may poll the DMA. */
    if (s_tx_busy && ((HAL_GetTick() - s_tx_start_ms) > TUE_HAPTIC_TX_TIMEOUT_MS)) {
        (void)HAL_UART_AbortTransmit(&TUE_HAPTIC_UART);
        uint32_t guard = __get_PRIMASK();
        __disable_irq();
        s_tx_busy    = false;
        s_tx_pending = false;
        g_tue_haptic_tx_drops++;
        __set_PRIMASK(guard);
    }

    /* This function runs in the USB protocol task, while the TX-complete
     * callback runs in the DMA ISR. Mask interrupts for the short
     * check/copy/start sequence so a completion cannot pick up a slot that is
     * still being filled. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool accepted;
    uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
    memcpy(s_tx_slots[slot], frame, TUE_HAPTIC_FRAME_LEN);

    if (s_tx_busy) {
        /* In-flight transfer owns s_tx_active; the other slot is free. */
        s_tx_pending = true;
        accepted = true;
    } else {
        accepted = tx_start_slot(slot);
    }

    __set_PRIMASK(primask);
    return accepted;
}

/* Build one fixed 20-byte wire frame with an 8-byte payload (the only frame
 * shape the UART slave parses). */
static void build_wire_frame8(uint8_t *out, uint8_t frame_type, uint32_t can_id,
                              uint8_t seq, const uint8_t *payload)
{
    out[0] = TUE_HAPTIC_SOF1;
    out[1] = TUE_HAPTIC_SOF2;
    out[2] = frame_type;
    out[3] = (uint8_t)(can_id & 0xFFU);
    out[4] = (uint8_t)((can_id >> 8) & 0xFFU);
    out[5] = (uint8_t)((can_id >> 16) & 0xFFU);
    out[6] = (uint8_t)((can_id >> 24) & 0xFFU);
    out[7] = TUE_HAPTIC_PAYLOAD_LEN;
    out[8] = seq;
    memcpy(&out[9], payload, TUE_HAPTIC_PAYLOAD_LEN);
    out[17] = tue_usb_crc8(&out[2],
                           (size_t)(TUE_HAPTIC_HEADER_LEN + TUE_HAPTIC_PAYLOAD_LEN));
    out[18] = TUE_HAPTIC_EOF1;
    out[19] = TUE_HAPTIC_EOF2;
}

bool tue_haptic_ping(uint8_t seq, uint8_t token, uint8_t *out)
{
    g_tue_haptic_last_ping_rx_bytes = 0U;

    /* A failed initial RX arm must not disable pinging forever: retry once. */
    if (!s_rx_armed) {
        rx_arm();
    }

    if ((out == NULL) || !s_initialized || !s_rx_armed) {
        g_tue_haptic_ping_timeouts++;
        return false;
    }

    uint8_t payload[TUE_HAPTIC_PAYLOAD_LEN] = {0};
    payload[0] = token;

    uint8_t frame[TUE_HAPTIC_FRAME_LEN];
    build_wire_frame8(frame, TUE_HAPTIC_FRAME_TYPE_REQ, TUE_HAPTIC_CAN_PING,
                      seq, payload);

    /* Discard stale bytes so a late reply to a previous ping cannot match. */
    uint8_t junk;
    while (tue_haptic_rx_pop(&junk)) {
        /* drain */
    }

    /* Wait for any in-flight haptic DMA transfer to finish. */
    uint32_t wait_start = HAL_GetTick();
    while (s_tx_busy) {
        if ((HAL_GetTick() - wait_start) > (TUE_HAPTIC_TX_TIMEOUT_MS * 2U)) {
            g_tue_haptic_ping_tx_fail++;
            return false;
        }
        osDelay(1U);
    }

    /* Reserve the transmitter; haptic frames queue in the idle slot meanwhile. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_tx_busy     = true;
    s_tx_start_ms = HAL_GetTick();
    __set_PRIMASK(primask);

    HAL_StatusTypeDef status = HAL_UART_Transmit(&TUE_HAPTIC_UART, frame,
                                                 TUE_HAPTIC_FRAME_LEN, 20U);

    g_tue_haptic_last_tx_ok = (status == HAL_OK) ? 1U : 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    s_tx_busy = false;
    if (s_tx_pending) {
        uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
        s_tx_pending = false;
        (void)tx_start_slot(slot);
    }
    __set_PRIMASK(primask);

    if (status != HAL_OK) {
        g_tue_haptic_ping_tx_fail++;
        return false;
    }

    /* Collect the reply. One 20-byte frame is expected, but the window scan
     * tolerates leading noise and resynchronises on the SOF markers. */
    uint8_t window[40];
    uint8_t wlen = 0U;
    uint16_t rx_seen = 0U;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < TUE_HAPTIC_PING_TIMEOUT_MS) {
        bool got_byte = false;
        uint8_t b;

        while (tue_haptic_rx_pop(&b)) {
            got_byte = true;
            rx_seen++;
            g_tue_haptic_last_ping_rx_bytes = (rx_seen > 255U) ? 255U : rx_seen;
            if (wlen >= sizeof(window)) {
                memmove(window, &window[1], sizeof(window) - 1U);
                wlen = (uint8_t)(sizeof(window) - 1U);
            }
            window[wlen++] = b;

            uint8_t i = 0U;
            while (((uint16_t)i + TUE_HAPTIC_FRAME_LEN) <= (uint16_t)wlen) {
                const uint8_t *p = &window[i];
                uint32_t can_id = (uint32_t)p[3] | ((uint32_t)p[4] << 8) |
                                  ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 24);

                if ((p[0] == TUE_HAPTIC_SOF1) && (p[1] == TUE_HAPTIC_SOF2) &&
                    (p[18] == TUE_HAPTIC_EOF1) && (p[19] == TUE_HAPTIC_EOF2) &&
                    (p[2] == TUE_HAPTIC_FRAME_TYPE_RESP_OK) &&
                    (can_id == TUE_HAPTIC_CAN_PING) &&
                    (p[7] == TUE_HAPTIC_PAYLOAD_LEN) && (p[8] == seq) &&
                    (p[9] == token) &&
                    (tue_usb_crc8(&p[2],
                                  (size_t)(TUE_HAPTIC_HEADER_LEN +
                                           TUE_HAPTIC_PAYLOAD_LEN)) == p[17])) {
                    memcpy(out, &p[9], TUE_HAPTIC_PAYLOAD_LEN);
                    g_tue_haptic_ping_replies++;
                    return true;
                }
                i++;
            }
        }

        if (!got_byte) {
            osDelay(1U);
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

    s_rx_tail     = s_rx_head;
    s_rx_last_pos = 0U;
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
    out[0] = (uint8_t)((s_initialized ? 0x01U : 0U) |
                       (s_rx_armed ? 0x02U : 0U) |
                       (s_tx_busy ? 0x04U : 0U) |
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

uint16_t tue_haptic_rx_available(void)
{
    return (uint16_t)((s_rx_head - s_rx_tail) & (TUE_HAPTIC_RX_RING_SIZE - 1U));
}

bool tue_haptic_rx_pop(uint8_t *byte)
{
    if ((byte == NULL) || (s_rx_head == s_rx_tail)) {
        return false;
    }
    *byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (TUE_HAPTIC_RX_RING_SIZE - 1U));
    return true;
}

/* ========================================================================== */
/* IRQ-context entry points                                                   */
/* ========================================================================== */

void tue_haptic_uart_tx_cplt_cb(UART_HandleTypeDef *huart)
{
    if (huart != &TUE_HAPTIC_UART) {
        return;
    }

    s_tx_busy = false;

    if (s_tx_pending) {
        uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
        s_tx_pending = false;
        (void)tx_start_slot(slot);
    }
}

void tue_haptic_uart_rx_event_cb(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &TUE_HAPTIC_UART) {
        return;
    }

    /* Circular DMA reports the current write position, not the delta.
     * HAL notifies a full buffer twice: once as a real DMA transfer-complete
     * (RxEventType == TC) and again as an IDLE event at position 0. Only the
     * IDLE report is a duplicate; a genuine second wrap arrives as TC and
     * must not be discarded. */
    uint16_t end = (size >= TUE_HAPTIC_RX_RING_SIZE) ? TUE_HAPTIC_RX_RING_SIZE : size;

    if ((huart->RxEventType == HAL_UART_RXEVENT_IDLE) &&
        (end == TUE_HAPTIC_RX_RING_SIZE)) {
        return;
    }

    uint16_t pos = s_rx_last_pos;

    while (pos != end) {
        rx_ring_push(s_rx_dma_buf[pos]);
        pos++;
        if (pos >= TUE_HAPTIC_RX_RING_SIZE) {
            pos = 0U;
        }
    }

    s_rx_last_pos = (end == TUE_HAPTIC_RX_RING_SIZE) ? 0U : end;
}

void tue_haptic_uart_error_cb(UART_HandleTypeDef *huart)
{
    if (huart != &TUE_HAPTIC_UART) {
        return;
    }

    g_tue_haptic_rx_errors++;

    /* A DMA/UART TX error aborts the transfer without invoking the TX-complete
     * callback, which would leave s_tx_busy stuck and stop forwarding. When
     * HAL no longer considers a transfer active, restart the pending frame. */
    if (huart->gState != HAL_UART_STATE_BUSY_TX) {
        s_tx_busy = false;
        if (s_tx_pending) {
            uint8_t slot = (uint8_t)(s_tx_active ^ 1U);
            s_tx_pending = false;
            (void)tx_start_slot(slot);
        }
    }

    rx_clear_flags();

    /* HAL aborts reception on overrun/framing errors: re-arm if idle. */
    if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
        rx_arm();
    }
}
