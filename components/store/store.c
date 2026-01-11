#include "store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "STORE";
static const char *NVS_NAME = "mscope_pref";
static const char *KEY_OBJ_IDX = "obj_idx";

// Default list of objectives
static const char objectives[MAX_OBJECTIVES][MAX_OBJ_NAME_LEN] = {
    "4x", "10x", "20x", "40x", "60x", "100x"
};

esp_err_t store_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

int store_get_obj_count(void) {
    return MAX_OBJECTIVES;
}

const char* store_get_obj_name(int index) {
    if (index < 0 || index >= MAX_OBJECTIVES) return "??";
    return objectives[index];
}

int store_get_last_obj_index(void) {
    nvs_handle_t handle;
    int32_t index = 0; // Default to first objective

    if (nvs_open(NVS_NAME, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_i32(handle, KEY_OBJ_IDX, &index);
        nvs_close(handle);
    }
    
    // Safety check for corruption
    if (index < 0 || index >= MAX_OBJECTIVES) index = 0;
    
    return (int)index;
}

esp_err_t store_set_last_obj_index(int index) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAME, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_i32(handle, KEY_OBJ_IDX, (int32_t)index);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    
    nvs_close(handle);
    ESP_LOGI(TAG, "Saved objective index: %d (%s)", index, objectives[index]);
    return err;
}