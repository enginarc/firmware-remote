#include "display.h"
#include "board_config.h"
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"


static lv_obj_t *lbl_x, *lbl_y, *lbl_z;
static lv_obj_t *lbl_obj, *lbl_batt;
static lv_obj_t *overlay_cont = NULL;
static lv_timer_t *msg_timer = NULL;

// Helper: Handle message auto-dismissal
static void msg_timer_cb(lv_timer_t * timer) {
    display_clear_message();
}

void display_init_ui(void) {
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

    // Left Side: X, Y, Z Labels
    lbl_x = lv_label_create(scr);
    lv_obj_align(lbl_x, LV_ALIGN_TOP_LEFT, 10, 20);
    lv_obj_set_style_text_color(lbl_x, lv_color_hex(0xFFFFFF), 0);

    lbl_y = lv_label_create(scr);
    lv_obj_align(lbl_y, LV_ALIGN_TOP_LEFT, 10, 60);
    lv_obj_set_style_text_color(lbl_y, lv_color_hex(0xFFFFFF), 0);

    lbl_z = lv_label_create(scr);
    lv_obj_align(lbl_z, LV_ALIGN_TOP_LEFT, 10, 100);
    lv_obj_set_style_text_color(lbl_z, lv_color_hex(0xFFFFFF), 0);

    // Right Side: Objective and Battery
    lbl_obj = lv_label_create(scr);
    lv_obj_align(lbl_obj, LV_ALIGN_TOP_RIGHT, -10, 20);
    lv_obj_set_style_text_color(lbl_obj, lv_color_hex(0x00FF00), 0);

    lbl_batt = lv_label_create(scr);
    lv_obj_align(lbl_batt, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_text_color(lbl_batt, lv_color_hex(0xAAAAAA), 0);

    display_update_coords(0, 0, 0);
    display_update_objective("4x");
    display_update_battery(100);
}

void display_update_coords(int x, int y, int z) {
    lv_label_set_text_fmt(lbl_x, "X: %d", x);
    lv_label_set_text_fmt(lbl_y, "Y: %d", y);
    lv_label_set_text_fmt(lbl_z, "Z: %d", z);
}

void display_update_objective(const char* objective) {
    lv_label_set_text_fmt(lbl_obj, "Obj: %s", objective);
}

void display_update_battery(int level) {
    lv_label_set_text_fmt(lbl_batt, "Batt: %d%%", level);
}

void display_show_message(const char* title, const char* body, message_type_t type) {
    display_clear_message(); // Remove existing if any

    overlay_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(overlay_cont, 280, 120);
    lv_obj_center(overlay_cont);
    
    // Background color based on type
    uint32_t color = 0x222222;
    if (type == MSG_TYPE_ERROR) color = 0x880000;
    else if (type == MSG_TYPE_WARNING) color = 0x888800;
    
    lv_obj_set_style_bg_color(overlay_cont, lv_color_hex(color), 0);
    lv_obj_set_style_border_width(overlay_cont, 2, 0);
    lv_obj_set_style_border_color(overlay_cont, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t * t = lv_label_create(overlay_cont);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t * b = lv_label_create(overlay_cont);
    lv_label_set_text(b, body);
    lv_label_set_long_mode(b, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(b, 260);
    lv_obj_align(b, LV_ALIGN_CENTER, 0, 10);

    // Start 10-second timer
    msg_timer = lv_timer_create(msg_timer_cb, 10000, NULL);
    lv_timer_set_repeat_count(msg_timer, 1);
}

void display_clear_message(void) {
    if (overlay_cont) {
        lv_obj_del(overlay_cont);
        overlay_cont = NULL;
    }
    if (msg_timer) {
        lv_timer_del(msg_timer);
        msg_timer = NULL;
    }
}

static const char *TAG = "LCD_INIT";

// Resolution constants for Landscape mode
#define LCD_H_RES              320
#define LCD_V_RES              240
#define LCD_DRAW_BUFF_SIZE     (LCD_H_RES * 50) // 50 lines of buffer

static esp_lcd_panel_handle_t panel_handle = NULL;

// Callback: When LVGL finishes rendering a "chunk", send it to the LCD hardware
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
}

// Callback: Signal LVGL that the hardware transfer is complete
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

// Heartbeat for LVGL (must be called every 1-10ms)
static void lv_tick_inc_cb(void *arg) {
    lv_tick_inc(1);
}

esp_err_t display_init(void) {
    // 1. Setup Backlight (If connected to GPIO 21 per board_config.h)
    gpio_config_t bk_conf = {
        .pin_bit_mask = (1ULL << PIN_LCD_BL),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&bk_conf);
    gpio_set_level(PIN_LCD_BL, 1); // Turn on backlight

    // 2. Initialize SPI Bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_LCD_SCLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1, // Not used for LCD
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_DRAW_BUFF_SIZE * sizeof(lv_color_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 3. Configure LCD Panel IO (Interface between SPI and LCD)
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000, // 40MHz for smooth updates
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = NULL, // Will update after disp_drv is created
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    // 4. Configure ST7789 Panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    
    // Set to Landscape orientation
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // 5. Initialize LVGL
    lv_init();
    
    // Allocate drawing buffer
    lv_color_t *buf1 = heap_caps_malloc(LCD_DRAW_BUFF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_DRAW_BUFF_SIZE);

    // Register Display Driver
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    // Update the IO callback with the driver pointer
    io_config.user_ctx = &disp_drv;

    // Start 1ms tick timer for LVGL
    const esp_timer_create_args_t tick_args = { .callback = &lv_tick_inc_cb, .name = "lvgl_tick" };
    esp_timer_handle_t tick_timer = NULL;
    esp_timer_create(&tick_args, &tick_timer);
    esp_timer_start_periodic(tick_timer, 1000); // 1ms

    ESP_LOGI(TAG, "Display Initialized: ST7789 320x240 Landscape");
    return ESP_OK;
}