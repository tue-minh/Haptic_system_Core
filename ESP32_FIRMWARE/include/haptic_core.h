#ifndef HAPTIC_CORE_H
#define HAPTIC_CORE_H

#include <stdint.h>
#include "config.h"

// ==============================================================================
// HAPTIC CORE - valve hardware control, safety state machine, watchdog
// ==============================================================================
// Owns: system state, error counters, active DAC voltages, LED indicators.
// Safety-first: haptic_core_init() drives all 5 MCP4725 outputs to 0.0V through
// the PCA9548A/TCA9548A mux before anything else runs. Outputs only leave 0V
// when fully valid haptic data arrives (via tue_protocol -> handle_haptic_packet).

enum HapticSystemState : uint8_t {
    STATE_IDLE    = 0,
    STATE_RUNNING = 1,
    STATE_ERROR   = 2
};

// Boot / hardware
void haptic_core_init();                    // PCA + Wire + safety boot vent + LEDs + state
void vent_all_chambers();                    // FAILSAFE: all 5 chambers to 0.0V
bool set_voltage(uint8_t channel, float voltage);  // clamped to V_MAX, updates cache
bool pca_select(uint8_t channel);           // select one mux channel (0..7)
bool pca_disable_all();                     // disconnect all mux channels
bool pca_reset_device();                    // hardware reset pulse
int  hc_i2c_scan(uint8_t* addr_out, int max_addrs); // scan currently selected channel

// Haptic data path (called by tue_protocol on a valid 0x300 frame)
void handle_haptic_packet(const uint8_t* payload8);

// State machine / safety
void  set_state(uint8_t state);
void  update_led();
void  check_watchdog();
float compute_valve_voltage(float force_n);
void  hc_notify_uart_error();     // ++consecutive_errors; trips vent + STATE_ERROR at max
void  hc_reset_consecutive_errors();

// Introspection (tue_protocol ping reply / console diagnostics)
uint8_t get_system_state();
uint16_t get_consecutive_errors();
uint8_t get_last_haptic_counter();
float   get_dac_voltage(uint8_t ch);

#endif // HAPTIC_CORE_H
