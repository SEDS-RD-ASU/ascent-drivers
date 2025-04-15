// void app_main() {
//     ESP_ERROR_CHECK(sam_m10q_init(sda_pin, scl_pin, i2c_port, i2c_freq));

//     ESP_ERROR_CHECK(sam_m10q_set_10hz());
//     vTaskDelay(1000 / portTICK_PERIOD_MS);  // let it cook for a lil lol

//     while (1) {
//         ESP_ERROR_CHECK(sam_m10q_read_nmea());
//         vTaskDelay(333 / portTICK_PERIOD_MS);  // read at 3hz
//     }
// }


// fix this later abdul!