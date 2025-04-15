idf_component_register(
    SRCS 
        "src/driver_w25qxx.c"
        "src/driver_w25qxx_interface.c"
        "src/driver_w25qxx_advance.c"
        "examples/w25qxx_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    REQUIRES driver spi
) 