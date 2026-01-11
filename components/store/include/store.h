#ifndef STORE_H
#define STORE_H

#include "esp_err.h"

// Objective list configuration
#define MAX_OBJECTIVES 6
#define MAX_OBJ_NAME_LEN 10

esp_err_t store_init(void);

// Objective Management
int store_get_obj_count(void);
const char* store_get_obj_name(int index);

// State Persistence
int  store_get_last_obj_index(void);
esp_err_t store_set_last_obj_index(int index);

#endif