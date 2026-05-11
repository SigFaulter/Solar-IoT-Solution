set(srcs
    "main.c"
    "ble_gatt.c"
    "mppt_ble.c"
    "mppt_publish.c"
    "space_parser.c"
    "eeprom_parser.c"
    "proto_builder.c"
    "settings_parser.c"
    "mppt.pb.c"
)

idf_component_register(
    SRCS            ${srcs}
    INCLUDE_DIRS    "."
    PRIV_REQUIRES   nvs_flash bt nanopb esp_hw_support esp_timer esp_pm freertos
)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-error=missing-field-initializers)
