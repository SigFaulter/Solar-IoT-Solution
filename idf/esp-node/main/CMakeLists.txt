set(srcs "main.cpp"
         "ble_transport.cpp"
         "proto_builder.cpp"
         "space_parser.cpp"
         "mppt.pb.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash bt nanopb esp_hw_support esp_timer esp_pm)
