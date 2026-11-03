idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash bt bluetooth_low_energy_manager xbox_controller)
