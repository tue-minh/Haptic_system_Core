#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "tue_protocol.h"
#include "haptic_core.h"

// ==============================================================================
// HAPTIC SLAVE CONTROLLER - ESP32 WROOM (main)
// ==============================================================================
// 1. process UART2 binary frames from the STM32 master (CRC + anti-error failsafe)
// 2. optional 1 Hz beacon (default off)
// 3. watchdog failsafe (auto-vent to 0V after 150 ms without valid data)
// 4. status LEDs (green = idle, breathing blue = running, blinking red = error)
// 5. USB serial debug console (optional, strictly non-blocking, no effect on
//    the UART2/valve path; commands >G< >R< >S< >P< >U< >B< >T<)
// ==============================================================================

static void send_line(const char* text)
{
    Serial.println(text);
}


// ==============================================================================
// BOOT SELF-TEST (CRC8, 0x301 ping frame construction)
// ==============================================================================

static void selftest_fail(const char* msg)
{
    vent_all_chambers();
    digitalWrite(LED_RED_PIN, LED_ON);
    Serial.print("SELFTEST_FAILED: ");
    Serial.println(msg);
    while (true) {
        vent_all_chambers();
        delay(500);
    }
}


static void run_boot_selftest()
{
    const uint8_t vec[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    if (crc8_sae_j1850(vec, 9) != 0x4B) {
        selftest_fail("CRC8 selftest failed");
    }

    uint8_t req[TUE_FRAME_LEN];
    const uint8_t token_payload[8] = {0x5A, 0, 0, 0, 0, 0, 0, 0};
    build_tue_frame(FRAME_TYPE_REQ, CAN_ID_HAPTIC_PING, 5, token_payload, req);
    if (crc8_sae_j1850(req + 2, 15) != req[17]) {
        selftest_fail("0x301 req build failed");
    }

    uint8_t rep[TUE_FRAME_LEN];
    build_ping_reply(5, 0x5A, rep);
    if (!(rep[2] == FRAME_TYPE_RESP_OK && rep[8] == 5 && rep[9] == 0x5A)) {
        selftest_fail("0x301 rep build failed");
    }
    if (crc8_sae_j1850(rep + 2, 15) != rep[17]) {
        selftest_fail("0x301 rep CRC failed");
    }
}


// ==============================================================================
// USB SERIAL DEBUG CONSOLE (optional diagnostics - never blocks the 100 Hz path)
// ==============================================================================

static char    usb_line[80];
static uint8_t usb_line_len = 0;

static bool is_ws(char c)
{
    return (c == ' ' || c == '\t' || c == '\r' || c == '\n');
}


static bool line_lstrip_starts_with(char c)
{
    for (uint8_t i = 0; i < usb_line_len; i++) {
        if (is_ws(usb_line[i])) {
            continue;
        }
        return (usb_line[i] == c);
    }
    return false;
}


static void handle_usb_command(const char* cmd)
{
    if (strcmp(cmd, "G") == 0) {
        for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
            char buf[16];
            snprintf(buf, sizeof(buf), "%.3f", get_dac_voltage(ch));
            send_line(buf);
        }
    } else if (strcmp(cmd, "R") == 0) {
        vent_all_chambers();
        send_line("RESET_OK");
        set_state(STATE_IDLE);
    } else if (strcmp(cmd, "S") == 0) {
        for (uint8_t ch = 0; ch < 8; ch++) {
            if (!pca_select(ch)) {
                continue;
            }
            delay(2);
            uint8_t addrs[16];
            int n = hc_i2c_scan(addrs, 16);
            char line[64];
            if (n > 0) {
                if (n > 16) n = 16;
                int off = snprintf(line, sizeof(line), "%u:", (unsigned)ch);
                for (int i = 0; i < n; i++) {
                    off += snprintf(line + off, sizeof(line) - off, "%s%02X",
                                    (i > 0) ? "," : "", (unsigned)addrs[i]);
                }
            } else {
                snprintf(line, sizeof(line), "%u:-", (unsigned)ch);
            }
            send_line(line);
        }
        pca_disable_all();
    } else if (strcmp(cmd, "P") == 0) {
        Serial.printf("PING_STATS uart_frames=%lu rx_bytes=%lu ping_rx=%lu ping_tx=%lu "
                      "state=%u errors=%u beacon=%u beacon_tx=%lu\r\n",
                      (unsigned long)tue_uart_frames(),
                      (unsigned long)tue_rx_bytes_total(),
                      (unsigned long)tue_ping_rx(),
                      (unsigned long)tue_ping_tx(),
                      (unsigned)get_system_state(),
                      (unsigned)get_consecutive_errors(),
                      (unsigned)(tue_beacon_enabled() ? 1 : 0),
                      (unsigned long)tue_beacon_tx());
    } else if (strcmp(cmd, "U") == 0) {
        Serial.printf("UART2 tx=%u rx=%u baud=%u rxbuf=%u frames=%lu rx_bytes=%lu "
                      "ping_rx=%lu ping_tx=%lu beacon=%u beacon_tx=%lu\r\n",
                      (unsigned)UART_TX_PIN, (unsigned)UART_RX_PIN,
                      (unsigned)UART_BAUDRATE, (unsigned)UART_RX_BUF,
                      (unsigned long)tue_uart_frames(),
                      (unsigned long)tue_rx_bytes_total(),
                      (unsigned long)tue_ping_rx(),
                      (unsigned long)tue_ping_tx(),
                      (unsigned)(tue_beacon_enabled() ? 1 : 0),
                      (unsigned long)tue_beacon_tx());
    } else if (strcmp(cmd, "T") == 0) {
        send_beacon(true);
    } else if (strcmp(cmd, "B") == 0) {
        tue_beacon_toggle();
        send_line(tue_beacon_enabled() ? "BEACON ON" : "BEACON OFF");
    }
}


static void process_usb_serial()
{
    // Bounded so a flood of console bytes can never stall the UART2/valve path;
    // remaining bytes are picked up on the next (microseconds later) loop pass.
    int budget = 32;
    while (budget-- > 0) {
        if (!Serial.available()) {
            break;
        }
        char ch = (char)Serial.read();

        if (ch == '<' && line_lstrip_starts_with('>')) {
            // Immediate framing: ">CMD<" fires on '<'
            int s = 0;
            while (s < (int)usb_line_len && is_ws(usb_line[s])) s++;  // lstrip
            s++;  // skip '>'
            int e = (int)usb_line_len;
            while (e > s && is_ws(usb_line[e - 1])) e--;              // strip
            char cmd[80];
            int k = 0;
            for (; s < e; s++) {
                if (k < (int)sizeof(cmd) - 1) {
                    cmd[k++] = (char)toupper((unsigned char)usb_line[s]);
                }
            }
            cmd[k] = 0;
            usb_line_len = 0;
            if (k > 0) {
                handle_usb_command(cmd);
            }
        } else if (ch == '\n' || ch == '\r') {
            int s = 0;
            int e = (int)usb_line_len;
            while (s < e && is_ws(usb_line[s])) s++;                  // strip
            while (e > s && is_ws(usb_line[e - 1])) e--;
            char t[80];
            int k = 0;
            for (; s < e; s++) {
                if (k < (int)sizeof(t) - 1) {
                    t[k++] = (char)toupper((unsigned char)usb_line[s]);
                }
            }
            t[k] = 0;
            usb_line_len = 0;
            if (k == 0) {
                continue;
            }
            char cmd[80];
            int m = 0;
            if (t[0] == '>' && t[k - 1] == '<') {
                int i = 1;
                int end = k - 1;
                while (i < end && is_ws(t[i])) i++;
                while (end > i && is_ws(t[end - 1])) end--;
                for (; i < end; i++) cmd[m++] = t[i];
            } else if (t[0] == '>') {
                int i = 1;
                int end = k;
                while (i < end && is_ws(t[i])) i++;
                while (end > i && is_ws(t[end - 1])) end--;
                for (; i < end; i++) cmd[m++] = t[i];
            } else {
                for (int i = 0; i < k; i++) cmd[m++] = t[i];
            }
            cmd[m] = 0;
            if (m > 0) {
                handle_usb_command(cmd);
            }
        } else {
            if (usb_line_len < sizeof(usb_line) - 1) {
                usb_line[usb_line_len++] = ch;
            }
            if (usb_line_len > 64) {
                usb_line_len = 0;
            }
        }
    }
}


// ==============================================================================
// BOOTSTRAP
// ==============================================================================

void setup()
{
    // SAFETY FIRST: reset the PCA9548A mux and force all 5 MCP4725 DACs to
    // 0.0V through the mux before anything else runs. Valves stay at 0V until
    // a fully valid 0x300 UART frame arrives.
    haptic_core_init();

    Serial.begin(115200);  // debug console (optional)
    tue_protocol_init();   // UART2 @ 115200 from STM32 master

    run_boot_selftest();
    set_state(STATE_IDLE);
    vent_all_chambers();

    send_line("HAPTIC_SLAVE_READY (UART2 @ 115200 baud, Binary 20B 0x300/0x301, Anti-Error Failsafe Active)");
    Serial.printf("UART2 tx=GPIO%u rx=GPIO%u baud=%u | console: >G< >R< >S< >P< >U< >B< >T<\r\n",
                  (unsigned)UART_TX_PIN, (unsigned)UART_RX_PIN, (unsigned)UART_BAUDRATE);
}


// ==============================================================================
// MAIN EXECUTION LOOP (no delay - free running)
// ==============================================================================

void loop()
{
    process_uart2();      // 1. binary frames from UART2 with anti-noise failsafe
    tue_beacon_poll();    // 1b. optional 1 Hz heartbeat (off by default)
    check_watchdog();    // 2. vent to 0V if UART data is lost for > 150 ms
    update_led();        // 3. status LEDs
    process_usb_serial(); // 4. optional debug console (never blocks)
}
