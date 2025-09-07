#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include <inttypes.h>
#include <string.h>
#include "esp_timer.h"

#include "driver_SAM_M10Q.h"

#include "ascent_r2_hardware_definition.h"
#include "i2c_manager.h"

#define GPS_MAX_PACKET_SIZE 256

static uint8_t gps_packet_buf[GPS_MAX_PACKET_SIZE];


// ----------- GPS SPECIFIC I2C ------------------ //

static esp_err_t read_gps_stream(i2c_port_t port, uint8_t device_addr, uint8_t *data, uint16_t buf_length, uint16_t *real_length) {
    uint8_t length_buf[2];
    uint16_t length;

    // start of cmd1
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // tell it i want 0xfd
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xFD, true);

    // start reading data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true); 

    // i2c_master_read(cmd, length_buf, 2, false);
    i2c_master_read(cmd,&length_buf[0],1,true);
    i2c_master_read(cmd,&length_buf[1],1,true);;

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // end of cmd1
    length = (length_buf[0] << 8) | (length_buf[1]);
    length --;
    printf("length_buf: 0x%02X, 0x%02X\n", length_buf[0], length_buf[1]);
    *real_length = length;

    if(length > buf_length) {
        return ESP_FAIL;
    }
    
    ret = i2c_manager_read_yeet(port, device_addr, data, length);
    
    return ret;
}



// ------ actual driver follows ----  //

// This is the third full overhaul of this driver. I am really bad at writing drivers. This GPS is a pain in my ass. - Abdul

esp_err_t sendGPSBytes(uint8_t *buf, uint16_t num_bytes) {
    return i2c_manager_write_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, buf, num_bytes);
}


esp_err_t readGPSBytes(uint8_t *buf, uint16_t num_bytes) {
    return i2c_manager_read_yeet(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, buf, num_bytes);
}


esp_err_t readNextGPSPacket(void) {
    uint16_t packet_length = 0;

    esp_err_t ret = read_gps_stream(I2C_MASTER_PORT, SAM_M10Q_I2C_ADDR, gps_packet_buf, GPS_MAX_PACKET_SIZE, &packet_length);
    if (ret != ESP_OK){
        return ret;
    }

    sam_m10q_msginfo_t msginfo = sam_m10q_get_msginfo(gps_packet_buf, packet_length);

    printf("msginfo: class: 0x%02X, id: 0x%02X, length: %u\n", msginfo.class, msginfo.id, msginfo.length);

    printf("packet length: %d\n", packet_length);

    for(int i = 0; i < packet_length; i++){
        printf("0x%02X ", gps_packet_buf[i]);
    }

    printf("\n");

    return ESP_OK;
}


esp_err_t requestUARTBaudrate(void) {
    uint8_t payload[] = {0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x2D, 0x80};
    return sendGPSBytes(payload, sizeof(payload));
}


esp_err_t disableNMEAoutprot(void) {
    uint8_t payload[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x72, 0x10, 0x00, 0x1e, 0xb1};
    return sendGPSBytes(payload, sizeof(payload));
}


esp_err_t disableI2Ctimeout(void) {
    uint8_t payload[] = {0xb5, 0x62, 0x06, 0x8a, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x02, 0x00, 0x51, 0x10, 0x01, 0xfe, 0x4f};
    return sendGPSBytes(payload, sizeof(payload));
}


sam_m10q_msginfo_t sam_m10q_get_msginfo(uint8_t *buf, uint16_t bufsize) {
    sam_m10q_msginfo_t msginfo;
    msginfo.class = buf[2];
    msginfo.id = buf[3];
    msginfo.length = (buf[4] << 8) | (buf[5]);
    msginfo.valid_checksum = false;

    // Validate checksum

    uint8_t ck_a;
    uint8_t ck_b;

    ck_a = buf[bufsize - 2];
    ck_b = buf[bufsize - 1];

    return msginfo;
}
