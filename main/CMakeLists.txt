idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES physical_controls wireless_manager board_config display store
                             driver esp_wifi nvs_flash json)