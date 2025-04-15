idf_component_register(
    SRCS 
        "src/driver_buzzer.c"
        "src/interface_buzzer.c"
        "examples/buzzer_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/../"
    REQUIRES driver adc
) 