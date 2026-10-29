#ifndef TUE_PROTOCOL_H
#define TUE_PROTOCOL_H

#include <stdint.h>
#include "config.h"

// ==============================================================================
// TUE PROTOCOL - CRC-8, 20-byte binary frames, UART2 link to the STM32 master
// ==============================================================================
// Wire frame: 0xAA 0x55 | type | can_id LE32 | dlc | seq | payload(8) | crc8 | 0x55 0xAA
//   - CAN 0x300: haptic telemetry (100 Hz, one-way, drives the valves)
//   - CAN 0x301: binary ping from the STM32 (answered with RESP_OK, token echo)
//   - CAN 0x303: optional 1 Hz beacon (ASYNC, default OFF)
// CRC-8 SAE J1850: poly 0x1D, init 0xFF, xorout 0xFF, MSB-first; crc8("123456789") == 0x4B

uint8_t crc8_sae_j1850(const uint8_t* data, unsigned int len);

bool build_tue_frame(uint8_t frame_type, uint32_t can_id, uint8_t seq,
                     const uint8_t* payload8, uint8_t* out20);

// RESP_OK (0x02) reply for CAN 0x301: token echo + fw/state/counters payload
bool build_ping_reply(uint8_t seq, uint8_t token, uint8_t* out20);

// UART2 link
void tue_protocol_init();   // Serial2 @ 115200 8-N-1 on GPIO16(RX)/GPIO17(TX)
void process_uart2();       // sliding-window parser with anti-error protection
void tue_beacon_poll();     // sends the 1 Hz beacon while enabled
void send_beacon(bool announce);

// Beacon control (console commands >T< / >B<)
bool tue_beacon_enabled();
void tue_beacon_toggle();

// Link statistics (console diagnostics >P< / >U<)
uint32_t tue_uart_frames();
uint32_t tue_rx_bytes_total();
uint32_t tue_ping_rx();
uint32_t tue_ping_tx();
uint32_t tue_beacon_tx();

#endif // TUE_PROTOCOL_H
