idf_component_register(
    SRCS 
        "src/driver_BMP390L.c"
        "examples/bmp_test.c"
        "interface/interface_bmp390l.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "interface"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."

    REQUIRES driver i2c esp_timer indicator
) 