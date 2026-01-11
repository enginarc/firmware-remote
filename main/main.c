#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "cJSON.h"

#include "board_config.h"      
#include "physical_controls.h" 
#include "wireless_manager.h"

static const char *TAG = "REMOTE_MAIN";

/**
 * Setup WiFi in Station mode (required for ESP-NOW)
 */
static void wifi_setup(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi Station initialized for ESP-NOW.");
}

/**
 * Creates a JSON payload representing the current state of the remote
 */
char* create_payload(remote_controls_state_t *state, int z_val, int joy_x, int joy_y) {
    cJSON *root = cJSON_CreateObject();
    
    // Movement Data
    cJSON_AddNumberToObject(root, "x", joy_x);
    cJSON_AddNumberToObject(root, "y", joy_y);
    cJSON_AddNumberToObject(root, "z", z_val);
    
    // Status Flags
    cJSON_AddBoolToObject(root, "fine", state->fine_toggle_active);
    cJSON_AddBoolToObject(root, "shutter", state->shutter_pressed);
    cJSON_AddNumberToObject(root, "obj_diff", state->objective_encoder_diff);
    cJSON_AddBoolToObject(root, "save", state->save_pos_pressed);
    cJSON_AddBoolToObject(root, "reset", state->reset_pos_pressed);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

void app_main(void) {
    // 1. Initialize NVS (Used for storing Peer MAC addresses)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 2. Wireless Setup
    wifi_setup();
    wireless_manager_init();

    // 3. Physical Controls Hardware Initialization
    // Note: I2C is initialized inside physical_controls_init based on board_config.h
    ESP_ERROR_CHECK(physical_controls_init());
    ESP_ERROR_CHECK(z_axis_encoder_init()); // Start High-Speed PCNT for E6B2

    remote_controls_state_t controls;
    int z_encoder_val = 0;
    int pair_btn_cooldown = 0;

    ESP_LOGI(TAG, "Remote Panel Started. Entering Main Loop.");

    int x, y;
float batt;
if (physical_controls_read_analog(&x, &y, &batt) == ESP_OK) {
    // Logic to include x, y, and batt in your JSON payload
}

    adc_inputs_init();      // Initialize ADC Channels
    joystick_calibrate();   // Find center points while at rest

    int mapped_x, mapped_y;

        // Instead of raw ADC values, use the mapped calibrated values
        get_mapped_joystick(&mapped_x, &mapped_y);

        if (wireless_is_paired()) {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "jx", mapped_x); // Sends -100 to 100
            cJSON_AddNumberToObject(root, "jy", mapped_y); // Sends -100 to 100
            // ... add other button states ...
            
            char *payload = cJSON_PrintUnformatted(root);
            wireless_send_json(payload);
            free(payload);
        }

    // 4. Main Control Loop
    while (1) {
        // Read MCP23017 Expander (Buttons + EC11)
        if (physical_controls_read_all(&controls) == ESP_OK) {
            
            // Read High-Speed Z-Axis Encoder
            z_axis_get_value(&z_encoder_val);

            // Read Analog Joystick directly from ESP32-S3 ADC
            // Note: Replace with actual ADC read function once your ADC component is ready
            int joy_x = 0; // Placeholder for adc_read(PIN_JOYSTICK_X)
            int joy_y = 0; // Placeholder for adc_read(PIN_JOYSTICK_Y)

            // --- PAIRING LOGIC ---
            if (controls.pairing_pressed && pair_btn_cooldown == 0) {
                ESP_LOGI(TAG, "Pairing button pressed. Searching for Main Unit...");
                wireless_start_pairing();
                pair_btn_cooldown = 40; // 2 second debounce (40 * 50ms)
            }
            if (pair_btn_cooldown > 0) pair_btn_cooldown--;

            // --- COMMUNICATION LOGIC ---
            if (wireless_is_in_pairing_mode()) {
                 // Use the RGB LED on GPIO 48 if available to show blue blinking
            } 
            else if (wireless_is_paired()) {
                // Send control data only if paired
                char *payload = create_payload(&controls, z_encoder_val, joy_x, joy_y);
                if (payload) {
                    wireless_send_json(payload);
                    free(payload);
                }
            }
        }

        if (shutter_triggered(&controls)) {
        // Create a dedicated "Action" payload
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "cmd", "capture");
        cJSON_AddNumberToObject(root, "val", 1); // 1 for shutter fire
        
        char *payload = cJSON_PrintUnformatted(root);
        wireless_send_json(payload);
        free(payload);
        
        // Visual feedback for the user (optional)
        // ui_show_capture_animation(); 
        }

        // Loop timing (20Hz update rate)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}