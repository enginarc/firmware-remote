#ifndef PHYSICAL_CONTROLS_H
#define PHYSICAL_CONTROLS_H

#include "board_config.h"
#include "esp_err.h"

// Struct to hold the state of all expanded inputs
typedef struct {
    bool pairing_pressed;
    bool shutter_pressed;
    bool lock_panel_active;
    bool fine_toggle_active;
    bool reset_pos_pressed;
    bool save_pos_pressed;
    int8_t objective_encoder_diff; // +1, -1, or 0
    bool objective_click;
} remote_controls_state_t;

esp_err_t physical_controls_init(void);
esp_err_t physical_controls_read_all(remote_controls_state_t *state);
esp_err_t physical_controls_read_analog(int *joy_x, int *joy_y, float *v_batt);
esp_err_t z_axis_encoder_init(void); // New for high-speed E6B2
esp_err_t z_axis_get_value(int *value);
esp_err_t joystick_calibrate(void);
void get_mapped_joystick(int *out_x, int *out_y);
esp_err_t adc_inputs_init(void);
bool shutter_triggered(remote_controls_state_t *state);

#endif