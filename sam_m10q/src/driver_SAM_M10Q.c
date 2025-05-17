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

#include "driver_SAM_M10Q.h"

#include "ascent_r2_hardware_definition.h"

// Calculate the UBX Checksum for a given buffer (3.4)
void ubxChecksum(uint8_t *buffer, int packet_length, U2 *checksum) {
    uint8_t N = packet_length;

    uint8_t i = 0;
    U1 CK_A = 0;
    U1 CK_B = 0;

    for (i = 0; i < N; i++) {
        CK_A = CK_A + buffer[i];
        CK_B = CK_B + CK_A;

        CK_A &= 0xFF;
        CK_B &= 0xFF;
    }

    *checksum = (CK_B << 8) | CK_A;
}

// Create a UBX-CFG-VALSET Frame (3.10.5.1)
void ubxCfgValset(ubxFrame *frame, U1 layer, ubxCfgData data, ubxCfgData data2, uint8_t repeat_times) {
    frame->sync_chars = SYNC_CHARS;
    frame->message_class = 0x06;
    frame->message_id = 0x8a;

    uint16_t payload_size = repeat_times * (4 + data.value_size);
    if (data2.value_size > 0) {
        payload_size += repeat_times * (4 + data2.value_size);
    }
    
    Un payload = (uint8_t *)malloc(payload_size);
    payload[0] = 0x01; // silly since we're not using transactions. freeballin it.
    payload[1] = (uint8_t)layer;
    payload[2] = 0x0; // no transaction
    payload[3] = 0x0; // reserved

    uint16_t payload_index = 4;
    for (uint8_t i = 0; i < repeat_times; i++) {
        // copy over the key id
        payload[payload_index++] = (data.key_id >> 0) & 0xFF;
        payload[payload_index++] = (data.key_id >> 8) & 0xFF;
        payload[payload_index++] = (data.key_id >> 16) & 0xFF;
        payload[payload_index++] = (data.key_id >> 24) & 0xFF;

        // copy the value
        for (uint8_t balls = 0; balls < data.value_size; balls++) {
            payload[payload_index++] = data.value[balls];
        }
    }

    if (data2.value_size > 0) {
        for (uint8_t i = 0; i < repeat_times; i++) {
            // copy over the key id for data2
            payload[payload_index++] = (data2.key_id >> 0) & 0xFF;
            payload[payload_index++] = (data2.key_id >> 8) & 0xFF;
            payload[payload_index++] = (data2.key_id >> 16) & 0xFF;
            payload[payload_index++] = (data2.key_id >> 24) & 0xFF;

            // copy the value for data2
            for (uint8_t balls = 0; balls < data2.value_size; balls++) {
                payload[payload_index++] = data2.value[balls];
            }
        }
    }

    frame->length = payload_index; // end index = length of payload
    frame->payload = payload;

    // make buffer for checksum
    uint8_t buffer[4 + frame->length];
    buffer[0] = frame->message_class;
    buffer[1] = frame->message_id;
    buffer[2] = (frame->length >> 0) & 0xFF;
    buffer[3] = (frame->length >> 8) & 0xFF;
    memcpy(buffer + 4, frame->payload, frame->length);

    ubxChecksum(buffer, frame->length + 4, &(frame->checksum));
}

// Create a UBX-CFG-RST Frame (3.10.2)
void ubxCfgRstStartGNSS(ubxFrame *frame) {
    frame->sync_chars = SYNC_CHARS;
    frame->message_class = 0x06;
    frame->message_id = 0x04;
    frame->length = 4;

    // Allocate payload buffer of 4 bytes
    uint8_t *payload = (uint8_t *)malloc(4);
    payload[0] = 0x00;
    payload[1] = 0x00;
    payload[2] = 0x09; // resetMode
    payload[3] = 0x00;

    frame->payload = payload;

    printf("Payload as bytes: ");
    for (uint16_t i = 0; i < frame->length; i++) {
        printf("%02X ", ((uint8_t *)frame->payload)[i]);
    }
    printf("\n");

    // make buffer for checksum
    uint8_t buffer[4 + frame->length];
    buffer[0] = frame->message_class;
    buffer[1] = frame->message_id;
    buffer[2] = (frame->length >> 0) & 0xFF;
    buffer[3] = (frame->length >> 8) & 0xFF;
    memcpy(buffer + 4, frame->payload, frame->length);

    ubxChecksum(buffer, frame->length + 4, &(frame->checksum));
}