#include <Arduino.h>
#include <string.h>
#include "tue_protocol.h"
#include "haptic_core.h"

// ==============================================================================
// SAE J1850 CRC-8 LOOKUP TABLE & CHECKSUM
// ==============================================================================
// Poly: 0x1D, Init: 0xFF, XorOut: 0xFF, MSB First
// Verification: crc8("123456789") == 0x4B

static const uint8_t CRC8_TABLE[256] = {
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
};


uint8_t crc8_sae_j1850(const uint8_t* data, unsigned int len)
{
    uint8_t crc = 0xFF;
    for (unsigned int i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc ^ 0xFF;
}


// ==============================================================================
// FRAME BUILDERS
// ==============================================================================

bool build_tue_frame(uint8_t frame_type, uint32_t can_id, uint8_t seq,
                     const uint8_t* payload8, uint8_t* out)
{
    if (payload8 == 0 || out == 0) {
        return false;
    }
    out[0] = TUE_SOF1;
    out[1] = TUE_SOF2;
    out[2] = frame_type;
    out[3] = (uint8_t)(can_id & 0xFF);
    out[4] = (uint8_t)((can_id >> 8) & 0xFF);
    out[5] = (uint8_t)((can_id >> 16) & 0xFF);
    out[6] = (uint8_t)((can_id >> 24) & 0xFF);
    out[7] = TUE_PAYLOAD_LEN;  // DLC
    out[8] = seq;
    for (int i = 0; i < TUE_PAYLOAD_LEN; i++) {
        out[9 + i] = payload8[i];
    }
    out[17] = crc8_sae_j1850(out + 2, 15);  // CRC over bytes 2..16
    out[18] = TUE_EOF1;
    out[19] = TUE_EOF2;
    return true;
}


bool build_ping_reply(uint8_t seq, uint8_t token, uint8_t* out)
{
    // Reply payload for CAN 0x301 RESP_OK:
    //   0 token, 1 protocol version, 2 fw major, 3 fw minor,
    //   4 system state, 5 last haptic frame counter,
    //   6 consecutive errors (saturated), 7 uptime seconds (mod 256)
    uint16_t errors = get_consecutive_errors();
    if (errors > 255) {
        errors = 255;
    }
    uint8_t payload[8];
    payload[0] = token;
    payload[1] = PING_PROTOCOL_VERSION;
    payload[2] = FW_VERSION_MAJOR;
    payload[3] = FW_VERSION_MINOR;
    payload[4] = get_system_state();
    payload[5] = get_last_haptic_counter();
    payload[6] = (uint8_t)errors;
    payload[7] = (uint8_t)((millis() / 1000) & 0xFF);
    return build_tue_frame(FRAME_TYPE_RESP_OK, CAN_ID_HAPTIC_PING, seq, payload, out);
}


// ==============================================================================
// UART2 LINK STATE
// ==============================================================================

static uint8_t  uart_rx_buf[UART_RX_BUF];
static size_t   uart_rx_len     = 0;
static uint32_t last_rx_byte_ms = 0;

static uint32_t uart_frames   = 0;
static uint32_t rx_bytes_total = 0;
static uint32_t ping_rx       = 0;
static uint32_t ping_tx       = 0;

// Keep the unsolicited 1 Hz beacon disabled by default in pure polling mode so
// the STM32 never hits USART_SR_ORE overrun when idle (>T< / >B< can enable it).
static bool     beacon_enabled = false;
static uint8_t  beacon_seq     = 0;
static uint32_t beacon_tx      = 0;
static uint32_t last_beacon_ms = 0;


// ==============================================================================
// PING & BEACON
// ==============================================================================

static void handle_ping_frame(const uint8_t* frame)
{
    if (frame[2] != FRAME_TYPE_REQ) {  // only FRAME_TYPE_REQ is answered
        return;
    }

    uint8_t token = (frame[7] >= 1) ? frame[9] : 0;  // payload[0] = token echo
    uint8_t seq   = frame[8];
    uint8_t reply[TUE_FRAME_LEN];
    build_ping_reply(seq, token, reply);
    size_t written = Serial2.write(reply, TUE_FRAME_LEN);
    ping_rx++;
    ping_tx++;
    // Log only AFTER the reply is on the wire so the console can never delay it.
    Serial.printf("PING rx seq=%u token=0x%02X -> tx %uB\r\n",
                  (unsigned)seq, (unsigned)token,
                  (unsigned)(written ? written : TUE_FRAME_LEN));
}


void send_beacon(bool announce)
{
    uint8_t payload[8];
    payload[0] = 0xBE;
    payload[1] = 0xAC;
    payload[2] = PING_PROTOCOL_VERSION;
    payload[3] = FW_VERSION_MAJOR;
    payload[4] = FW_VERSION_MINOR;
    payload[5] = get_system_state();
    payload[6] = get_last_haptic_counter();
    payload[7] = (uint8_t)((millis() / 1000) & 0xFF);

    uint8_t seq_sent = beacon_seq;
    uint8_t frame[TUE_FRAME_LEN];
    build_tue_frame(FRAME_TYPE_ASYNC, BEACON_CAN_ID, seq_sent, payload, frame);
    Serial2.write(frame, TUE_FRAME_LEN);
    beacon_seq = (uint8_t)((beacon_seq + 1) & 0xFF);
    beacon_tx++;
    last_beacon_ms = millis();
    if (announce) {
        Serial.printf("BEACON tx seq=%u count=%lu pins=GPIO%u->GPIO%u\r\n",
                      (unsigned)seq_sent, (unsigned long)beacon_tx,
                      (unsigned)UART_TX_PIN, (unsigned)UART_RX_PIN);
    }
}


bool tue_beacon_enabled()
{
    return beacon_enabled;
}


void tue_beacon_toggle()
{
    beacon_enabled = !beacon_enabled;
}


// ==============================================================================
// UART2 PROTOCOL FRAME PARSER WITH ANTI-ERROR PROTECTION
// ==============================================================================

void process_uart2()
{
    uint32_t now = millis();

    // Read everything currently available on UART2
    int n = Serial2.available();
    if (n > 0) {
        int space = (int)(UART_RX_BUF - uart_rx_len);
        if (n > space) {
            n = space;  // the rest stays in the driver buffer for the next pass
        }
        if (n > 0) {
            int r = (int)Serial2.read(uart_rx_buf + uart_rx_len, (size_t)n);
            if (r > 0) {
                rx_bytes_total += (uint32_t)r;
                last_rx_byte_ms = now;
                uart_rx_len += (size_t)r;
            }
        }
    } else if (uart_rx_len > 0 && (uint32_t)(now - last_rx_byte_ms) > STALE_RX_TIMEOUT_MS) {
        // Flush a stale incomplete fragment (< 20 B) after an idle gap
        uart_rx_len = 0;
        return;
    }

    int discarded = 0;

    // Sliding-window parse of every complete 20-byte frame. Valid frames are
    // always processed before the noise-overflow failsafe is applied.
    while (uart_rx_len >= TUE_FRAME_LEN) {
        // Preamble 0xAA 0x55
        if (uart_rx_buf[0] != TUE_SOF1 || uart_rx_buf[1] != TUE_SOF2) {
            uart_rx_len--;
            memmove(uart_rx_buf, uart_rx_buf + 1, uart_rx_len);
            discarded++;
            continue;
        }

        // Postamble 0x55 0xAA at the end of the 20-byte frame
        if (uart_rx_buf[18] != TUE_EOF1 || uart_rx_buf[19] != TUE_EOF2) {
            hc_notify_uart_error();
            uart_rx_len--;
            memmove(uart_rx_buf, uart_rx_buf + 1, uart_rx_len);
            discarded++;
            continue;
        }

        // CRC-8 (SAE J1850) over bytes 2..16
        if (crc8_sae_j1850(uart_rx_buf + 2, 15) != uart_rx_buf[17]) {
            hc_notify_uart_error();
            uart_rx_len--;
            memmove(uart_rx_buf, uart_rx_buf + 1, uart_rx_len);
            discarded++;
            continue;
        }

        uint8_t frame[TUE_FRAME_LEN];
        memcpy(frame, uart_rx_buf, TUE_FRAME_LEN);
        memmove(uart_rx_buf, uart_rx_buf + TUE_FRAME_LEN, uart_rx_len - TUE_FRAME_LEN);
        uart_rx_len -= TUE_FRAME_LEN;
        uart_frames++;

        uint32_t can_id = (uint32_t)frame[3]
                        | ((uint32_t)frame[4] << 8)
                        | ((uint32_t)frame[5] << 16)
                        | ((uint32_t)frame[6] << 24);
        uint8_t dlc = frame[7];

        if (can_id == CAN_ID_HAPTIC_TELEMETRY && dlc == TUE_PAYLOAD_LEN) {
            // Fully valid haptic frame -> reset the consecutive error counter
            hc_reset_consecutive_errors();
            handle_haptic_packet(frame + 9);
        } else if (can_id == CAN_ID_HAPTIC_PING) {
            // Ping/status request from the STM32 master; does NOT refresh the
            // haptic watchdog and does not touch the valve outputs.
            handle_ping_frame(frame);
        }
        // Structurally valid frame with another CAN ID: ignored, not an error
        // (the watchdog still protects if no 0x300 frame ever arrives).
    }

    // Noise-overflow failsafe: applied only after every valid frame was parsed
    if (discarded > MAX_RX_BUFFER_SIZE) {
        uart_rx_len = 0;
        hc_notify_uart_error();
    }
}


void tue_beacon_poll()
{
    if (beacon_enabled && (uint32_t)(millis() - last_beacon_ms) >= BEACON_PERIOD_MS) {
        send_beacon(false);
    }
}


// ==============================================================================
// INIT & STATISTICS
// ==============================================================================

void tue_protocol_init()
{
    pinMode(UART_RX_PIN, INPUT_PULLUP);  // idle-high RX line
    Serial2.setRxBufferSize(UART_RX_BUF);
    Serial2.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // Drain any boot glitch bytes
    while (Serial2.available()) {
        Serial2.read();
    }

    last_rx_byte_ms = millis();
    last_beacon_ms  = millis();
}


uint32_t tue_uart_frames()    { return uart_frames;    }
uint32_t tue_rx_bytes_total() { return rx_bytes_total; }
uint32_t tue_ping_rx()        { return ping_rx;        }
uint32_t tue_ping_tx()        { return ping_tx;        }
uint32_t tue_beacon_tx()      { return beacon_tx;      }
