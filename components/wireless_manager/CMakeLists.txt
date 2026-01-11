idf_component_register(SRCS "wireless_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_wifi nvs_flash json esp_common board_config)