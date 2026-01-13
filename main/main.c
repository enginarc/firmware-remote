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
#include "display.h"
#include "store.h"
#include "lvgl.h"
#include "esp_sleep.h"

static const char *TAG = "REMOTE_MAIN";

void lvgl_task(void *pvParameters) {
    while (1) {
        lv_timer_handler(); // Updates animations and widgets
        vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz refresh check
    }
}

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
    cJSON_AddNumberToObject(root, "obj_diff", state->encoder_diff);
    cJSON_AddBoolToObject(root, "save", state->save_pos_pressed);
    cJSON_AddBoolToObject(root, "reset", state->reset_pos_pressed);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return out;
}

void app_main(void) {

// 1. Initialize Storage
    store_init();
    
    // 2. Load the last used objective from memory
    int current_obj_idx = store_get_last_obj_index();
    
    // 3. Update Display with the loaded name
    display_init();
    display_update_objective(store_get_obj_name(current_obj_idx));
    display_init_ui(); // Creates the labels we defined earlier

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


    // Start the UI processing task on Core 1 
    // to keep Core 0 free for WiFi and Encoders
    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 4096, NULL, 5, NULL, 1);

    // 3. Physical Controls Hardware Initialization
    // Note: I2C is initialized inside physical_controls_init based on board_config.h
    ESP_ERROR_CHECK(physical_controls_init());
    ESP_ERROR_CHECK(z_axis_encoder_init()); // Start High-Speed PCNT for E6B2
    ESP_ERROR_CHECK(adc_inputs_init());      // Initialize ADC Channels
    ESP_ERROR_CHECK(joystick_calibrate());   // Find center points while at rest

    remote_controls_state_t controls;
    int z_val = 0;
    int m_x, m_y;
    float batt_v;
    int pair_cooldown = 0;

    // 4. Main Control Loop
    while (1) {
        // Read MCP23017 Expander (Buttons + EC11)
        if (physical_controls_read_all(&controls) == ESP_OK) {
             // Read High-Speed Z-Axis Encoder
            z_axis_get_value(&z_val);
            get_mapped_joystick(&m_x, &m_y); // Now updates every loop
            physical_controls_read_analog(NULL, NULL, &batt_v); // Get latest battery

            if (wireless_is_paired() && !wireless_is_in_pairing_mode()) {
                char *payload = create_payload(&controls, z_val, m_x, m_y);
                if (payload) {
                    wireless_send_json(payload);
                    free(payload);
                }
            }

            // --- PAIRING LOGIC ---
            if (controls.pairing_pressed && pair_cooldown == 0) {
                ESP_LOGI(TAG, "Pairing button pressed. Searching for Main Unit...");
                wireless_start_pairing();
                pair_cooldown = 40; // 2 second debounce (40 * 50ms)
            }
            if (pair_cooldown > 0) pair_cooldown--;

            // --- COMMUNICATION LOGIC ---
            if (wireless_is_in_pairing_mode()) {
                 // Use the RGB LED on GPIO 48 if available to show blue blinking
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

            // --- DEEP SLEEP LOGIC ---
            if (controls.sleep_button_pressed) {
                ESP_LOGI(TAG, "Sleep button pressed. Entering Deep Sleep...");
                
                // Show a brief message before shutting down
                display_show_message("POWER", "Shutting down...", MSG_TYPE_INFO);
                vTaskDelay(pdMS_TO_TICKS(1000));

                // Configure GPIO 0 to wake the device when pressed (pulled Low)
                esp_sleep_enable_ext0_wakeup(PIN_SLEEP_WAKE_BTN, 0); //
                esp_deep_sleep_start();
            }

            // --- SAVE POSITION LOGIC (TTP223) ---
            if (controls.save_pos_pressed) {
                display_show_message("SAVE", "Position Stored!", MSG_TYPE_INFO);
                // Send Save Command via ESP-NOW
                cJSON *root = cJSON_CreateObject();
                cJSON_AddStringToObject(root, "cmd", "save_pos");
                char *payload = cJSON_PrintUnformatted(root);
                wireless_send_json(payload);
                free(payload);
                cJSON_Delete(root);
            }

            // Logic: If any button is pressed or encoder moves, clear message
            if (controls.any_input_detected) { 
                display_clear_message();
            }

            // Update normal UI
            display_update_coords(m_x, m_y, z_val);
            
            // Example Warning logic
            if (abs(m_x) > 90) {
                display_show_message("LIMIT", "X-Axis near boundary!", MSG_TYPE_WARNING);
            }

            // Logic: EC11 Encoder rotation changes objectives
            // We assume physical_controls provides 'encoder_diff' (+1 or -1)
            if (controls.encoder_diff != 0) {
                current_obj_idx += controls.encoder_diff;
                
                // Wrap around logic (e.g., 100x -> 4x)
                if (current_obj_idx >= store_get_obj_count()) current_obj_idx = 0;
                if (current_obj_idx < 0) current_obj_idx = store_get_obj_count() - 1;

                // 4. Update the UI and Save to NVS
                display_update_objective(store_get_obj_name(current_obj_idx));
                store_set_last_obj_index(current_obj_idx);
            }
            
        }

        
        // Loop timing (20Hz update rate)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}