#ifndef WIRELESS_MANAGER_H
#define WIRELESS_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

void wireless_manager_init(void);
bool wireless_is_paired(void);
esp_err_t wireless_send_json(const char *json_string);

// NEW: Pairing Controls
void wireless_start_pairing(void);
bool wireless_is_in_pairing_mode(void);

#endif