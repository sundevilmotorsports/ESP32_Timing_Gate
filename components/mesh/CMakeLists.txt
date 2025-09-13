idf_component_register(SRCS "mesh.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_wifi esp_driver_gpio esp_driver_ledc nvs_flash esp_timer)