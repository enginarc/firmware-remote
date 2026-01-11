idf_component_register(
    SRCS "physical_controls.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc board_config
)