idf_component_register(
    SRCS 
        "wifi/wifi_hotspot.c"
        "wifi/wifi_websocket.c"
        "wifi/wifi_server.c"
    INCLUDE_DIRS 
        "wifi"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
        "${CMAKE_CURRENT_SOURCE_DIR}/../hardware_io/pyro/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../patch/adc/src"
        "${CMAKE_CURRENT_SOURCE_DIR}/../.."
        "${CMAKE_CURRENT_SOURCE_DIR}/../../src/"
    REQUIRES driver esp_http_server esp_wifi nvs_flash esp_eth esp_timer
) 