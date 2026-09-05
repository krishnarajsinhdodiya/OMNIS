idf_component_register(
    SRCS
        "main.c"
        "tick.c"
        "omnis_params.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_gptimer
        esp_timer
)
