set(srcs
    "main.cpp"
    "ble_gatt.c"
    "mppt_ble.c"
    "mppt_publish.cpp"
    "space_parser.cpp"
    "proto_builder.cpp"
    "mppt.pb.c"
)

idf_component_register(
    SRCS            ${srcs}
    INCLUDE_DIRS    "."
    PRIV_REQUIRES   nvs_flash bt nanopb esp_hw_support esp_timer esp_pm freertos
)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-error=missing-field-initializers)
# All C++ files use C++17
set_source_files_properties(
    main.cpp space_parser.cpp proto_builder.cpp mppt_publish.cpp
    PROPERTIES COMPILE_FLAGS "-std=gnu++17"
)
