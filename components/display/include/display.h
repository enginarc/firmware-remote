#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_err.h"

typedef enum {
    MSG_TYPE_INFO,
    MSG_TYPE_WARNING,
    MSG_TYPE_ERROR
} message_type_t;

// Initialization
esp_err_t display_init(void);
void display_init_ui(void);

// Data Updates
void display_update_coords(int x, int y, int z);
void display_update_objective(const char* objective);
void display_update_battery(int level);

// Overlay Management
void display_show_message(const char* title, const char* body, message_type_t type);
void display_clear_message(void);

#endif