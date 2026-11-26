#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "haptic_core.h"

// ==============================================================================
// HAPTIC CORE - system state, valve control, failsafe watchdog, LED animation
// ==============================================================================

// ---------------- System state ----------------
static uint8_t  system_state        = STATE_IDLE;
static uint32_t last_packet_ms      = 0;
static uint16_t consecutive_errors = 0;
static uint8_t  recovery_count      = 0;
static uint8_t  last_haptic_counter = 0;

// Active force->voltage mapping parameters (boot defaults; runtime + NVS via 0x304/0x305)
static uint16_t g_vmin_mv     = (uint16_t)(V_MIN * 1000.0f + 0.5f);
static uint16_t g_vmax_mv     = (uint16_t)(V_MAX * 1000.0f + 0.5f);
static uint16_t g_fthresh_cn  = (uint16_t)(F_THRESH * 100.0f + 0.5f);
static uint16_t g_frated_cn   = (uint16_t)(F_RATED * 100.0f + 0.5f);
static float    g_v_min       = V_MIN;
static float    g_v_max       = V_MAX;
static float    g_f_thresh    = F_THRESH;
static float    g_f_rated     = F_RATED;
static bool     g_saved_nvs   = false;
static Preferences g_prefs;

static constexpr uint8_t NVS_MAGIC = 0xA5;

// Active output voltages [A, B, C, D, E]
static float dac_voltage[DAC_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// ---------------- LED animation state ----------------
static uint32_t breath_start = 0;
static uint32_t error_timer  = 0;
static uint8_t  error_phase  = 0;


static void blue_brightness(int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;
    ledcWrite(BLUE_LEDC_CHANNEL, (uint32_t)(65535UL * (uint32_t)percent / 100UL));
}


// ==============================================================================
// PCA9548A & MCP4725 LOW-LEVEL DRIVERS WITH FAILSAFE
// ==============================================================================

bool pca_select(uint8_t channel)
{
    if (channel > 7) {
        return false;
    }
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)(1u << channel));
    return (Wire.endTransmission() == 0);
}


bool pca_disable_all()
{
    Wire.beginTransmission(PCA_ADDR);
    Wire.write((uint8_t)0);
    return (Wire.endTransmission() == 0);
}


bool pca_reset_device()
{
    digitalWrite(PCA_RESET_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(PCA_RESET_PIN, HIGH);
    delay(2);
    return true;
}


static uint16_t voltage_to_dac(float voltage)
{
    int val = (int)((voltage / VDD) * (float)DAC_MAX);
    if (val < 0) {
        return 0;
    }
    if (val > DAC_MAX) {
        return (uint16_t)DAC_MAX;
    }
    return (uint16_t)val;
}


static bool write_dac(uint8_t channel, float voltage)
{
    if (channel >= DAC_COUNT) {
        return false;
    }

    uint16_t val = voltage_to_dac(voltage);
    if (!pca_select(channel)) {
        return false;
    }

    Wire.beginTransmission(MCP4725_ADDR[channel]);
    Wire.write((uint8_t)MCP4725_WRITE_DAC);
    Wire.write((uint8_t)((val >> 4) & 0xFF));
    Wire.write((uint8_t)((val & 0x0F) << 4));
    return (Wire.endTransmission() == 0);
}


bool set_voltage(uint8_t channel, float voltage)
{
    // Safety clamp: never output above configured V_MAX or hardware V_HW_CEILING
    if (voltage < 0.0f) voltage = 0.0f;
    float ceil_v = (g_v_max < V_HW_CEILING) ? g_v_max : V_HW_CEILING;
    if (voltage > ceil_v) voltage = ceil_v;
    if (!write_dac(channel, voltage)) {
        return false;
    }
    dac_voltage[channel] = voltage;
    return true;
}


void vent_all_chambers()
{
    // FAILSAFE SHUTDOWN: all 5 pneumatic chambers to 0.0V, then disconnect all
    // PCA channels. Aborts early if the mux is not responding on I2C so the
    // UART2 polling loop is never stalled.
    for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
        dac_voltage[ch] = 0.0f;
    }
    if (!pca_select(0)) {
        return;
    }
    for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
        if (!write_dac(ch, 0.0f)) {
            break;
        }
    }
    pca_disable_all();
}


// SAFETY-FIRST BOOT: write 0.0V to every MCP4725 through the mux before
// anything else runs. Unlike vent_all_chambers(), every channel is attempted
// even if an earlier one fails, so no valve is left pressurized at power-up.
static void safe_boot_vent()
{
    for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
        dac_voltage[ch] = 0.0f;
    }
    for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
        if (!pca_select(ch)) {
            continue;
        }
        Wire.beginTransmission(MCP4725_ADDR[ch]);
        Wire.write((uint8_t)MCP4725_WRITE_DAC);
        Wire.write((uint8_t)0x00);  // DAC register = 0 -> 0.0V output
        Wire.write((uint8_t)0x00);
        Wire.endTransmission();
    }
    pca_disable_all();
}


int hc_i2c_scan(uint8_t* addr_out, int max_addrs)
{
    int n = 0;
    for (uint16_t addr = 0x08; addr <= 0x77; addr++) {
        if (addr == PCA_ADDR) {
            continue;
        }
        Wire.beginTransmission((uint8_t)addr);
        if (Wire.endTransmission() == 0) {
            if (n < max_addrs) {
                addr_out[n] = (uint8_t)addr;
            }
            n++;
        }
    }
    return n;
}


// ==============================================================================
// HAPTIC TRANSFER FUNCTION & SAFETY VOLTAGE LIMITER
// ==============================================================================

static void apply_mapping_sanitized(uint16_t v_min_mv, uint16_t v_max_mv,
                                    uint16_t f_thresh_cn, uint16_t f_rated_cn)
{
    uint16_t hw_ceil_mv = (uint16_t)(V_HW_CEILING * 1000.0f + 0.5f);
    if (v_min_mv > hw_ceil_mv) v_min_mv = hw_ceil_mv;
    if (v_max_mv > hw_ceil_mv) v_max_mv = hw_ceil_mv;
    if (v_max_mv < v_min_mv)   v_max_mv = v_min_mv;

    if (f_thresh_cn > 2500U) f_thresh_cn = 2500U;
    if (f_rated_cn > 2550U)  f_rated_cn = 2550U;
    if (f_rated_cn <= (uint16_t)(f_thresh_cn + 10U)) {
        uint16_t min_rated = (uint16_t)(f_thresh_cn + 10U);
        f_rated_cn = (min_rated <= 2550U) ? min_rated : 2550U;
    }

    g_vmin_mv    = v_min_mv;
    g_vmax_mv    = v_max_mv;
    g_fthresh_cn = f_thresh_cn;
    g_frated_cn  = f_rated_cn;

    g_v_min    = (float)v_min_mv / 1000.0f;
    g_v_max    = (float)v_max_mv / 1000.0f;
    g_f_thresh = (float)f_thresh_cn / 100.0f;
    g_f_rated  = (float)f_rated_cn / 100.0f;
}

void hc_set_mapping(uint16_t v_min_mv, uint16_t v_max_mv,
                    uint16_t f_thresh_cn, uint16_t f_rated_cn, bool save_nvs)
{
    apply_mapping_sanitized(v_min_mv, v_max_mv, f_thresh_cn, f_rated_cn);

    if (save_nvs) {
        if (g_prefs.begin("haptic_cfg", false)) {
            g_prefs.putUChar("valid", NVS_MAGIC);
            g_prefs.putUShort("vmin", g_vmin_mv);
            g_prefs.putUShort("vmax", g_vmax_mv);
            g_prefs.putUShort("fth",  g_fthresh_cn);
            g_prefs.putUShort("frat", g_frated_cn);
            g_prefs.end();
            g_saved_nvs = true;
        }
    }

    // Clamp any active DAC output that exceeds the newly lowered ceiling
    for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
        if (dac_voltage[ch] > g_v_max) {
            set_voltage(ch, g_v_max);
        }
    }
}

void hc_get_mapping(uint16_t* v_min_mv, uint16_t* v_max_mv,
                    uint16_t* f_thresh_cn, uint16_t* f_rated_cn, bool* saved_in_nvs)
{
    if (v_min_mv)     *v_min_mv = g_vmin_mv;
    if (v_max_mv)     *v_max_mv = g_vmax_mv;
    if (f_thresh_cn)  *f_thresh_cn = g_fthresh_cn;
    if (f_rated_cn)   *f_rated_cn = g_frated_cn;
    if (saved_in_nvs) *saved_in_nvs = g_saved_nvs;
}

float compute_valve_voltage(float force_n)
{
    if (force_n <= g_f_thresh) {
        return 0.0f;
    }

    float denom = g_f_rated - g_f_thresh;
    float ratio = (denom > 0.01f) ? ((force_n - g_f_thresh) / denom) : 1.0f;
    if (ratio > 1.0f) {
        ratio = 1.0f;
    } else if (ratio < 0.0f) {
        ratio = 0.0f;
    }

    float v_out = g_v_min + (g_v_max - g_v_min) * ratio;
    return (v_out < g_v_max) ? v_out : g_v_max;
}


void handle_haptic_packet(const uint8_t* payload)
{
    // 8-byte unified payload received from CAN ID 0x300:
    //   [0] F_Valve_A (0.1N/LSB) -> Valve A: +Y avg (Bottom - Tip)
    //   [1] F_Valve_B (0.1N/LSB) -> Valve B: Fz avg (Bottom - Mid)
    //   [2] F_Valve_C (0.1N/LSB) -> Valve C: -Y avg (Bottom - Base)
    //   [3] F_Valve_D (0.1N/LSB) -> Valve D: +X avg (Lateral Left)
    //   [4] F_Valve_E (0.1N/LSB) -> Valve E: -X avg (Lateral Right)
    //   [5] Reserved (0x00)
    //   [6] Status_Flags         [7] Frame_Counter
    last_packet_ms      = millis();
    last_haptic_counter = payload[7];

    // After an error trip, require a few consecutive good frames before
    // re-pressurising the chambers; stay vented while recovering.
    if (system_state == STATE_ERROR) {
        recovery_count++;
        if (recovery_count < RECOVERY_FRAMES) {
            return;
        }
    }

    float f_A = payload[0] * 0.1f;  // Valve A: +Y avg (Bottom - Tip)
    float f_B = payload[1] * 0.1f;  // Valve B: Fz avg (Bottom - Mid)
    float f_C = payload[2] * 0.1f;  // Valve C: -Y avg (Bottom - Base)
    float f_D = payload[3] * 0.1f;  // Valve D: +X avg (Lateral Left)
    float f_E = payload[4] * 0.1f;  // Valve E: -X avg (Lateral Right)

    float target[DAC_COUNT];
    target[0] = compute_valve_voltage(f_A);
    target[1] = compute_valve_voltage(f_B);
    target[2] = compute_valve_voltage(f_C);
    target[3] = compute_valve_voltage(f_D);
    target[4] = compute_valve_voltage(f_E);

    // Write the 5 DACs through the mux (only on change, to keep the I2C bus light)
    for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
        if (fabsf(target[ch] - dac_voltage[ch]) >= V_WRITE_DELTA) {
            if (!set_voltage(ch, target[ch])) {
                // I2C hardware write failure -> emergency error state
                set_state(STATE_ERROR);
                return;
            }
        }
    }

    set_state(STATE_RUNNING);
}


// ==============================================================================
// STATE MACHINE, WATCHDOG & STATE ANIMATION
// ==============================================================================

void set_state(uint8_t state)
{
    // Leaving STATE_RUNNING (or entering IDLE/ERROR) always vents the valves to 0V
    if (state != STATE_RUNNING && system_state == STATE_RUNNING) {
        vent_all_chambers();
    }

    system_state  = state;
    recovery_count = 0;
    uint32_t now   = millis();

    if (state == STATE_IDLE) {
        digitalWrite(LED_GREEN_PIN, LED_ON);
        digitalWrite(LED_RED_PIN, LED_OFF);
        blue_brightness(0);
    } else if (state == STATE_RUNNING) {
        digitalWrite(LED_GREEN_PIN, LED_OFF);
        digitalWrite(LED_RED_PIN, LED_OFF);
        breath_start = now;
    } else if (state == STATE_ERROR) {
        vent_all_chambers();
        digitalWrite(LED_GREEN_PIN, LED_OFF);
        blue_brightness(0);
        error_timer = now;
        error_phase = 0;
        digitalWrite(LED_RED_PIN, LED_ON);
    }
}


void update_led()
{
    uint32_t now = millis();

    if (system_state == STATE_IDLE) {
        digitalWrite(LED_GREEN_PIN, LED_ON);
        digitalWrite(LED_RED_PIN, LED_OFF);
        blue_brightness(0);
    } else if (system_state == STATE_RUNNING) {
        digitalWrite(LED_GREEN_PIN, LED_OFF);
        digitalWrite(LED_RED_PIN, LED_OFF);
        uint32_t elapsed = (uint32_t)(now - breath_start);
        uint32_t phase   = elapsed % BREATH_PERIOD_MS;
        uint32_t half    = BREATH_PERIOD_MS / 2;
        int brightness = (phase < half)
            ? (int)((phase * 100UL) / half)
            : (int)(((BREATH_PERIOD_MS - phase) * 100UL) / half);
        if (brightness < 2) {
            brightness = 2;
        }
        blue_brightness(brightness);
    } else if (system_state == STATE_ERROR) {
        digitalWrite(LED_GREEN_PIN, LED_OFF);
        blue_brightness(0);
        uint32_t duration = (error_phase < 2) ? 100 : 70;
        if ((uint32_t)(now - error_timer) >= duration) {
            error_timer = now;
            error_phase = (uint8_t)((error_phase + 1) % 4);
        }
        digitalWrite(LED_RED_PIN, (error_phase < 2) ? LED_ON : LED_OFF);
    }
}


void check_watchdog()
{
    uint32_t now     = millis();
    uint32_t elapsed = (uint32_t)(now - last_packet_ms);

    if (system_state == STATE_RUNNING) {
        // Lost UART signal for > WATCHDOG_TIMEOUT_MS -> vent all valves to 0.0V
        if (elapsed > WATCHDOG_TIMEOUT_MS) {
            vent_all_chambers();
            set_state(STATE_IDLE);
        }
    } else {
        // Active failsafe: any valve still above 0V outside RUNNING -> force to 0V
        for (uint8_t ch = 0; ch < DAC_COUNT; ch++) {
            if (dac_voltage[ch] > 0.0f) {
                set_voltage(ch, 0.0f);
            }
        }
    }
}


void hc_notify_uart_error()
{
    consecutive_errors++;
    if (consecutive_errors >= CONSECUTIVE_ERRORS_MAX) {
        vent_all_chambers();
        set_state(STATE_ERROR);
    }
}


void hc_reset_consecutive_errors()
{
    consecutive_errors = 0;
}


// ==============================================================================
// BOOT
// ==============================================================================

void haptic_core_init()
{
    // SAFETY FIRST: reset the mux, then force every MCP4725 output to 0.0V
    // before anything else runs.
    pinMode(PCA_A2_PIN, OUTPUT);
    digitalWrite(PCA_A2_PIN, HIGH);
    pinMode(PCA_A1_PIN, OUTPUT);
    digitalWrite(PCA_A1_PIN, HIGH);
    pinMode(PCA_A0_PIN, OUTPUT);
    digitalWrite(PCA_A0_PIN, HIGH);
    pinMode(PCA_RESET_PIN, OUTPUT);
    digitalWrite(PCA_RESET_PIN, HIGH);
    pca_reset_device();

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    safe_boot_vent();

    pinMode(LED_GREEN_PIN, OUTPUT);
    digitalWrite(LED_GREEN_PIN, LED_OFF);
    pinMode(LED_RED_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LED_OFF);
    ledcSetup(BLUE_LEDC_CHANNEL, BLUE_LEDC_FREQ_HZ, BLUE_LEDC_RES_BITS);
    ledcAttachPin(LED_BLUE_PIN, BLUE_LEDC_CHANNEL);
    ledcWrite(BLUE_LEDC_CHANNEL, 0);

    system_state        = STATE_IDLE;
    last_packet_ms      = millis();
    consecutive_errors  = 0;
    recovery_count      = 0;
    last_haptic_counter = 0;
    breath_start        = millis();
    error_timer         = millis();
    error_phase         = 0;

    // Load saved mapping parameters from NVS EEPROM if present
    g_saved_nvs = false;
    apply_mapping_sanitized((uint16_t)(V_MIN * 1000.0f + 0.5f),
                            (uint16_t)(V_MAX * 1000.0f + 0.5f),
                            (uint16_t)(F_THRESH * 100.0f + 0.5f),
                            (uint16_t)(F_RATED * 100.0f + 0.5f));
    if (g_prefs.begin("haptic_cfg", true)) {
        if (g_prefs.getUChar("valid", 0) == NVS_MAGIC) {
            uint16_t vmin = g_prefs.getUShort("vmin", g_vmin_mv);
            uint16_t vmax = g_prefs.getUShort("vmax", g_vmax_mv);
            uint16_t fth  = g_prefs.getUShort("fth",  g_fthresh_cn);
            uint16_t frat = g_prefs.getUShort("frat", g_frated_cn);
            apply_mapping_sanitized(vmin, vmax, fth, frat);
            g_saved_nvs = true;
        }
        g_prefs.end();
    }
}


// ==============================================================================
// INTROSPECTION
// ==============================================================================

uint8_t get_system_state()
{
    return system_state;
}

uint16_t get_consecutive_errors()
{
    return consecutive_errors;
}

uint8_t get_last_haptic_counter()
{
    return last_haptic_counter;
}

float get_dac_voltage(uint8_t ch)
{
    if (ch >= DAC_COUNT) {
        return 0.0f;
    }
    return dac_voltage[ch];
}
