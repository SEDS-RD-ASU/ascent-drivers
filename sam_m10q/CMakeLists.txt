idf_component_register(
    SRCS 
        "src/driver_SAM_M10Q.c"
        "examples/sam_m10q_test.c"
        "interface/interface_sam_m10q.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "interface"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    REQUIRES driver
) 