#include "physical_controls.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "driver/pulse_cnt.h"
#include "physical_controls.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "PHYS_CTRL";
static int8_t last_encoder_state = 0;
static adc_oneshot_unit_handle_t adc1_handle;
static pcnt_unit_handle_t pcnt_unit = NULL;

/**
 * Initializes ADC1 for Joystick and Battery Sensing 
 * based on PIN definitions in board_config.h
 */
esp_err_t adc_inputs_init(void) {
    // 1. Initialize ADC Unit 1
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // 2. Configure Channels
    // ADC_ATTEN_DB_12 allows the full range (0V to ~3.1V)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, 
    };

    // Register Joystick X using board_config.h address
    // GPIO 4 maps to ADC_CHANNEL_3
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
    
    // Register Joystick Y using board_config.h address
    // GPIO 5 maps to ADC_CHANNEL_4
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));
    
    // Register Battery Sense using board_config.h address
    // GPIO 6 maps to ADC_CHANNEL_5
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));

    ESP_LOGI(TAG, "ADC initialized for PIN_JOYSTICK_X, PIN_JOYSTICK_Y, and PIN_BATTERY_SENSE");
    return ESP_OK;
}

/**
 * Reads Analog values and returns raw integers and calculated voltage
 */
esp_err_t physical_controls_read_analog(int *joy_x, int *joy_y, float *v_batt) {
    int raw_x, raw_y, raw_batt;

    // Read values using the mapped channels
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw_x));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw_y));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &raw_batt));

    *joy_x = raw_x;
    *joy_y = raw_y;

    // Battery Voltage Calculation:
    // With your 100k/100k divider, the voltage at the pin is exactly half.
    // 3.1V is the typical full-scale reference for 12dB attenuation on S3.
    *v_batt = ((float)raw_batt / 4095.0f) * 3.1f * 2.0f;

    return ESP_OK;
}

esp_err_t z_axis_encoder_init(void) {
    // 1. Configure the PCNT unit
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // 2. Configure the PCNT channel
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = PIN_E6B2_PHA,
        .level_gpio_num = PIN_E6B2_PHB,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // 3. Set actions on edges and levels (Corrected constants for v5.x)
    // On rising edge of PHA: Increase count
    // On falling edge of PHA: Do nothing (to avoid double counting unless needed)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, 
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, // When PHA goes Low -> High
        PCNT_CHANNEL_EDGE_ACTION_HOLD));   // When PHA goes High -> Low

    // Define direction based on Level (PHB)
    // If PHB is Low: Direction is Normal
    // If PHB is High: Direction is Inverted
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, 
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,    // PHB Low
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE)); // PHB High

    // 4. Enable a filter to ignore noise (Glitch filter)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000, 
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // 5. Start the counter
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    return ESP_OK;
}

esp_err_t z_axis_get_value(int *value) {
    return pcnt_unit_get_count(pcnt_unit, value);
}




/**
 * Helper to write a single byte to an MCP register
 */
static esp_err_t mcp_write_byte(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * Reads both Port A and Port B simultaneously (16-bit read)
 */
static esp_err_t mcp_read_ports(uint8_t *porta, uint8_t *portb) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MCP_GPIOA, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP23017_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, porta, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, portb, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t physical_controls_init(void) {
    // Initializing I2C is assumed to be done in main or a central i2c_manager
    // Setting all pins to Inputs
    mcp_write_byte(MCP_IODIRA, 0xFF);
    mcp_write_byte(MCP_IODIRB, 0xFF);
    
    // Enabling Internal Pull-ups (0V when pressed)
    mcp_write_byte(MCP_GPPUA, 0xFF);
    mcp_write_byte(MCP_GPPUB, 0xFF);

    ESP_LOGI(TAG, "MCP23017 expansion I/O initialized successfully");

    // Initialize the direct Joystick Switch pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_JOYSTICK_SW),
        .pull_down_en = 0,
        .pull_up_en = 1, // Enable internal pull-up
    };
    gpio_config(&io_conf);

    return ESP_OK;
}

esp_err_t physical_controls_read_all(remote_controls_state_t *state) {
    uint8_t regA = 0, regB = 0;
    esp_err_t ret = mcp_read_ports(&regA, &regB);
    if (ret != ESP_OK) return ret;

    // Logic: Bitwise AND with pin mask, then invert (!) because pull-up = HIGH when idle
    state->pairing_pressed   = !(regA & (1 << MCP_PIN_PAIRING));
    state->shutter_pressed   = !(regA & (1 << MCP_PIN_SHUTTER));
    state->lock_panel_active = !(regA & (1 << MCP_PIN_LOCK_PANEL_TOGGLE)); //EA: Moved to ESP32-S3
    state->fine_toggle_active = !(regA & (1 << MCP_PIN_FINE_TOGGLE)); 
    state->reset_pos_pressed = !(regA & (1 << MCP_PIN_RESET_POS));
    state->objective_click   = !(regA & (1 << MCP_PIN_EC11_SW));
    
    // Read from Port B
    state->save_pos_pressed  = !(regB & (1 << MCP_PIN_SAVE_POS));

// Direct ESP32-S3 Pin for Joystick Switch (GPIO 3)
    // No I2C overhead, instant response
    state->lock_panel_active = (gpio_get_level(MCP_PIN_LOCK_PANEL_TOGGLE) == 0);

    /* --- EC11 Objective Encoder Logic --- */
    uint8_t current_enc = (regA & 0x03); // PHA and PHB are bits 0 and 1
    if (current_enc != last_encoder_state) {
        // Clockwise: 00 -> 01 or 11 -> 10
        if ((last_encoder_state == 0 && current_enc == 1) || (last_encoder_state == 3 && current_enc == 2)) {
            state->objective_encoder_diff = 1;
        } 
        // Counter-Clockwise: 00 -> 10 or 11 -> 01
        else if ((last_encoder_state == 0 && current_enc == 2) || (last_encoder_state == 3 && current_enc == 1)) {
            state->objective_encoder_diff = -1;
        } else {
            state->objective_encoder_diff = 0;
        }
        last_encoder_state = current_enc;
    } else {
        state->objective_encoder_diff = 0;
    }

    return ESP_OK;
}


static int center_x = 2048; // Default theoretical mid-point
static int center_y = 2048;
static const int DEADZONE_THRESHOLD = 150; // Adjusted for hardware play

/**
 * Samples the joystick multiple times to find the average center point.
 * Call this at startup while the joystick is at rest.
 */
esp_err_t joystick_calibrate(void) {
    long sum_x = 0;
    long sum_y = 0;
    const int samples = 50;

    ESP_LOGI("JOY_CALIB", "Calibrating joystick... do not touch!");

    for (int i = 0; i < samples; i++) {
        int raw_x, raw_y;
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw_x); // PIN_JOYSTICK_X
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw_y); // PIN_JOYSTICK_Y
        sum_x += raw_x;
        sum_y += raw_y;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    center_x = sum_x / samples;
    center_y = sum_y / samples;

    ESP_LOGI("JOY_CALIB", "Calibration Done. Centers -> X: %d, Y: %d", center_x, center_y);
    return ESP_OK;
}


/**
 * Processes raw ADC values into a clean -100 to +100 range
 */
void get_mapped_joystick(int *out_x, int *out_y) {
    int raw_x, raw_y;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &raw_x);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &raw_y);

    // Calculate displacement from calibrated center
    int dx = raw_x - center_x;
    int dy = raw_y - center_y;

    // Apply Deadzone logic
    if (abs(dx) < DEADZONE_THRESHOLD) {
        dx = 0;
    }
    if (abs(dy) < DEADZONE_THRESHOLD) {
        dy = 0;
    }

    // Map to percentage (-100 to 100)
    // Note: Half range is roughly 2048
    *out_x = (dx * 100) / 2048;
    *out_y = (dy * 100) / 2048;

    // Constrain X to bounds
    if (*out_x > 100) {
        *out_x = 100;
    } else if (*out_x < -100) {
        *out_x = -100;
    }

    // Constrain Y to bounds
    if (*out_y > 100) {
        *out_y = 100;
    } else if (*out_y < -100) {
        *out_y = -100;
    }
}

static bool last_shutter_physical_state = false;

bool shutter_triggered(remote_controls_state_t *state) {
    bool current_state = state->shutter_pressed;
    bool triggered = false;

    if (current_state && !last_shutter_physical_state) {
        triggered = true; // Button was just pressed
        ESP_LOGI("SHUTTER", "X-T3 Capture Triggered!");
    }

    last_shutter_physical_state = current_state;
    return triggered;
}