#include "wireless_manager.h"
#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "WIRELESS_MGR";
static uint8_t peer_mac[ESP_NOW_ETH_ALEN];
static bool is_paired = false;

// NEW: Pairing State Flag
static bool is_pairing_mode = false;

#define NVS_NAMESPACE "storage"
#define NVS_KEY_MAC   "scope_mac"

// --- INTERNAL HELPERS ---
static void save_peer_mac_to_nvs(const uint8_t *mac) {
    nvs_handle_t my_handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_blob(my_handle, NVS_KEY_MAC, mac, ESP_NOW_ETH_ALEN);
        nvs_commit(my_handle);
        nvs_close(my_handle);
        ESP_LOGI(TAG, "New Microscope MAC saved.");
    }
}

static bool load_peer_mac_from_nvs(uint8_t *mac_out) {
    nvs_handle_t my_handle;
    size_t required_size = ESP_NOW_ETH_ALEN;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) == ESP_OK) {
        esp_err_t err = nvs_get_blob(my_handle, NVS_KEY_MAC, mac_out, &required_size);
        nvs_close(my_handle);
        return (err == ESP_OK);
    }
    return false;
}

static void register_peer(const uint8_t *mac) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peerInfo.channel = 0; 
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(mac)) {
        esp_now_del_peer(mac); // Remove old reference if exists
    }
    esp_now_add_peer(&peerInfo);
    
    memcpy(peer_mac, mac, ESP_NOW_ETH_ALEN);
    is_paired = true;
}

// --- ESP-NOW CALLBACK ---
void on_data_recv(const esp_now_recv_info_t * info, const uint8_t *data, int len) {
    // 1. CRITICAL CHECK: Ignore broadcasts unless we are in Pairing Mode
    if (!is_pairing_mode) return;

    cJSON *root = cJSON_ParseWithLength((const char *)data, len);
    if (root == NULL) return;

    // 2. Look for explicit Pairing Signal
    // Expected JSON: {"PeerType": "Microscope", "Action": "Pair"}
    cJSON *type = cJSON_GetObjectItem(root, "PeerType");
    cJSON *action = cJSON_GetObjectItem(root, "Action");

    if (cJSON_IsString(type) && strcmp(type->valuestring, "Microscope") == 0 &&
        cJSON_IsString(action) && strcmp(action->valuestring, "Pair") == 0) {
        
        ESP_LOGW(TAG, "Pairing Signal Received! Saving MAC...");
        
        save_peer_mac_to_nvs(info->src_addr);
        register_peer(info->src_addr);
        
        // 3. Auto-Exit Pairing Mode on success
        is_pairing_mode = false; 
    }

    cJSON_Delete(root);
}

// --- PUBLIC API ---
void wireless_manager_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    uint8_t stored_mac[ESP_NOW_ETH_ALEN];
    if (load_peer_mac_from_nvs(stored_mac)) {
        ESP_LOGI(TAG, "Loaded Peer from NVS.");
        register_peer(stored_mac);
    } else {
        ESP_LOGW(TAG, "No Peer in NVS. Press PAIR button to pair.");
        is_paired = false;
    }
}

void wireless_start_pairing(void) {
    ESP_LOGW(TAG, "Entering PAIRING MODE for 10 seconds...");
    is_pairing_mode = true;
    
    // Optional: You could use a FreeRTOS timer here to auto-disable 
    // pairing_mode after 10-30 seconds if no signal is found.
}

bool wireless_is_in_pairing_mode(void) {
    return is_pairing_mode;
}

bool wireless_is_paired(void) {
    return is_paired;
}

esp_err_t wireless_send_json(const char *json_string) {
    if (!is_paired) return ESP_ERR_INVALID_STATE;
    return esp_now_send(peer_mac, (uint8_t *)json_string, strlen(json_string));
}

void on_receive_turret_data(cJSON *json) {
    cJSON *labels = cJSON_GetObjectItem(json, "labels");
    int active = cJSON_GetObjectItem(json, "current")->valueint;

    if (cJSON_IsArray(labels)) {
        for (int i = 0; i < cJSON_GetArraySize(labels); i++) {
            cJSON *item = cJSON_GetArrayItem(labels, i);
            // Update your TFT UI elements here
           // update_ui_label(i, item->valuestring, (i == active));
        }
    }
}