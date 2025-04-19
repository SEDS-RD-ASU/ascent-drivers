idf_component_register(
    SRCS 
        "src/driver_w25qxx.c"
        "examples/w25qxx_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    REQUIRES driver spi
) 