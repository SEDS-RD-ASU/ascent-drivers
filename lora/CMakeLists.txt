set(component_srcs "lora.c")

idf_component_register(
    SRCS 
        "${component_srcs}"
        "examples/lora-send.c"
        "examples/lora-receive.c"
    INCLUDE_DIRS 
        "include"
        "examples"
        "../bno055/src"
    REQUIRES driver
)
