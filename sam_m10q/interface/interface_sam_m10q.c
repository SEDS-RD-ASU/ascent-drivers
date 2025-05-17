#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include <inttypes.h>

#include "ascent_r2_hardware_definition.h"

#include "driver_SAM_M10Q.h"
#include "i2c_manager.h"

static const char *TAG = "SAM_M10Q";

static int i2c_port = I2C_MASTER_PORT;

esp_err_t gps_init() {
    // Store only the port number
    i2c_port_t port = i2c_port;
    
    // Check if I2C is already initialized
    if (!i2c_manager_is_initialized(port)) {
        ESP_LOGE(TAG, "I2C port %d not initialized", port);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGE(TAG, "GPS on I2C port %d initialized!!!", port);

    return ESP_OK;
}

bool readUbxPort(ubxFrame *frame) {
    U1 sync_check_buffer = 0x00;

    int retry_count = 0;
    const int max_retries = 100000;

    while (sync_check_buffer != 0xb5 && retry_count < max_retries) {
        i2c_manager_read_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, &sync_check_buffer, 1);
        retry_count++;
    }

    if (retry_count >= max_retries) {
        ESP_LOGE(TAG, "Failed to find first sync character (0xB5) after %d attempts", max_retries);
        return false;
    }
    
    i2c_manager_read_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, &sync_check_buffer, 1);

    if (sync_check_buffer == 0x62) {
        U4 framedetailbuffer;

        frame -> sync_chars = SYNC_CHARS;

        i2c_manager_read_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, (uint8_t *)&framedetailbuffer, 4);

        frame->message_class = ((uint8_t *)&framedetailbuffer)[0];
        frame->message_id = ((uint8_t *)&framedetailbuffer)[1];
        frame->length = ((uint8_t *)&framedetailbuffer)[2] | (((uint8_t *)&framedetailbuffer)[3] << 8);

        uint8_t payload_and_checksum[frame->length + 2];

        i2c_manager_read_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, payload_and_checksum, frame->length + 2);

        frame->payload = malloc(frame->length);
        memcpy(frame->payload, payload_and_checksum, frame->length);

        frame->checksum = payload_and_checksum[frame->length] | (payload_and_checksum[frame->length + 1] << 8);

        U2 idealchecksum;

        uint8_t buffer[4 + frame->length];

        buffer[0] = frame->message_class;
        buffer[1] = frame->message_id;
        buffer[2] = (frame->length >> 0) & 0xFF;
        buffer[3] = (frame->length >> 8) & 0xFF;
        memcpy(buffer + 4, frame->payload, frame->length);

        ubxChecksum(buffer, frame->length + 4, &idealchecksum);

        if (idealchecksum == frame->checksum) {
            return true;
        } else {
            return false;
        }

    } else {
        return false;
    }
}

void ubx10hz() {
    uint8_t msg[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x10, 0x00,
        0x01, 0x03, 0x00, 0x00, 0x01, 0x00,
        0x21, 0x30, 0x64, 0x00, 0x02, 0x00,
        0x21, 0x30, 0x01, 0x00, 0xAE, 0x5B
    };

    i2c_manager_write_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, msg, sizeof(msg));

    ubxFrame rollingFrame;

    vTaskDelay(10 / portTICK_PERIOD_MS);

    readUbxPort(&rollingFrame);
}

void freeUbxFrame(ubxFrame *frame) {
    if (frame->payload != NULL) {
        free(frame->payload);
        frame->payload = NULL;
    }
}

void sendUbxFrame(ubxFrame frame) {
    i2c_manager_write_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, (uint8_t *)&frame, sizeof(frame));
}

void NMEAstfu() {
    // Predefined frame to disable NMEA messages
    uint8_t msg[] = {
        0xB5, 0x62,
        0x0A, 0x06,
        0x00, 0x00,
        0x10, 0x3A
    };

    uint8_t msg2[] = {
        0xB5, 0x62,
        0x06, 0x04,
        0x04, 0x00,
        0x00, 0x00,
        0x09, 0x00,
        0x17, 0x76
    };

    ubxFrame rollingFrame1;

    readUbxPort(&rollingFrame1);

    i2c_manager_write_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, msg, sizeof(msg));

    vTaskDelay(10 / portTICK_PERIOD_MS);

    readUbxPort(&rollingFrame1);

    i2c_manager_write_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, msg2, sizeof(msg));

    vTaskDelay(10 / portTICK_PERIOD_MS);

    i2c_manager_write_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, msg, sizeof(msg));

    vTaskDelay(10 / portTICK_PERIOD_MS);

    readUbxPort(&rollingFrame1);

}

// ubx lib functions

bool uTimeIsLeapYear(int32_t year)
{
    bool isLeapYear = false;

    if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) {
        isLeapYear = true;
    }

    return isLeapYear;
}

bool uUbxProtocolIsLittleEndian()
{
    int32_t x = 1;

    return (*((char *) (&x)) == 1);
}

// Return a uint16_t from a pointer to a little-endian uint16_t.
uint16_t uUbxProtocolUint16Decode(const char *pByte)
{
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    const uint8_t *pInput = (const uint8_t *) pByte;
    uint16_t retValue;

    retValue  = *pInput;
    // Cast twice to keep Lint happy
    retValue += (uint16_t) (((uint16_t) *(pInput + 1)) << 8); // *NOPAD*

    return  retValue;
}

// Return a uint32_t from a pointer to a little-endian uint32_t.
uint32_t uUbxProtocolUint32Decode(const char *pByte)
{
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    const uint8_t *pInput = (const uint8_t *) pByte;
    uint32_t retValue;

    retValue  = *pInput;
    // Cast twice to keep Lint happy
    retValue += ((uint32_t) *(pInput + 1)) << 8;  // *NOPAD*
    retValue += ((uint32_t) *(pInput + 2)) << 16; // *NOPAD*
    retValue += ((uint32_t) *(pInput + 3)) << 24; // *NOPAD*

    return retValue;
}

// Return a uint64_t from a pointer to a little-endian uint64_t.
uint64_t uUbxProtocolUint64Decode(const char *pByte)
{
    // Use a uint8_t pointer for maths, more certain of its behaviour than char
    const uint8_t *pInput = (const uint8_t *) pByte;
    uint64_t retValue;

    retValue  = *pInput;
    // Cast twice to keep Lint happy
    retValue += ((uint64_t) *(pInput + 1)) << 8;  // *NOPAD*
    retValue += ((uint64_t) *(pInput + 2)) << 16; // *NOPAD*
    retValue += ((uint64_t) *(pInput + 3)) << 24; // *NOPAD*
    retValue += ((uint64_t) *(pInput + 4)) << 32; // *NOPAD*
    retValue += ((uint64_t) *(pInput + 5)) << 40; // *NOPAD*
    retValue += ((uint64_t) *(pInput + 6)) << 48; // *NOPAD*
    retValue += ((uint64_t) *(pInput + 7)) << 56; // *NOPAD*

    return retValue;
}

// Return a little-endian uint16_t from the given uint16_t.
uint16_t uUbxProtocolUint16Encode(uint16_t uint16)
{
    uint16_t retValue = uint16;

    if (!uUbxProtocolIsLittleEndian()) {
        retValue  = (uint16 & 0xFF00) >> 8;
        retValue += (uint16 & 0x00FF) << 8;
    }

    return retValue;
}

// Return a little-endian uint32_t from the given uint32_t.
uint32_t uUbxProtocolUint32Encode(uint32_t uint32)
{
    uint32_t retValue = uint32;

    if (!uUbxProtocolIsLittleEndian()) {
        retValue  = (uint32 & 0xFF000000) >> 24;
        retValue += (uint32 & 0x00FF0000) >> 8;
        retValue += (uint32 & 0x0000FF00) << 8;
        retValue += (uint32 & 0x000000FF) << 24;
    }

    return  retValue;
}

// Return a little-endian uint64_t from the given uint64_t.
uint64_t uUbxProtocolUint64Encode(uint64_t uint64)
{
    uint64_t retValue = uint64;

    if (!uUbxProtocolIsLittleEndian()) {
        retValue  = (uint64 & 0xFF00000000000000) >> 56;
        retValue += (uint64 & 0x00FF000000000000) >> 40;
        retValue += (uint64 & 0x0000FF0000000000) >> 24;
        retValue += (uint64 & 0x000000FF00000000) >> 8;
        retValue += (uint64 & 0x00000000FF000000) << 8;
        retValue += (uint64 & 0x0000000000FF0000) << 24;
        retValue += (uint64 & 0x000000000000FF00) << 40;
        retValue += (uint64 & 0x00000000000000FF) << 56;
    }

    return  retValue;
}

static const char gDaysInMonth[] = {31, 28, 31, 30, 31, 30, 31,
                                    31, 30, 31, 30, 31
                                   };
static const char gDaysInMonthLeapYear[] = {31, 29, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31
                                           };

int64_t uTimeMonthsToSecondsUtc(int32_t monthsUtc)
{
    int64_t secondsUtc = 0;

    for (int32_t x = 0; x < monthsUtc; x++) {
        if (uTimeIsLeapYear((x / 12) + 1970)) {
            secondsUtc += gDaysInMonthLeapYear[x % 12] * 3600 * 24;
        } else {
            secondsUtc += gDaysInMonth[x % 12] * 3600 * 24;
        }
    }

    return secondsUtc;
}

int32_t posDecode(char *pMessage,
                  int32_t *pLatitudeX1e7, int32_t *pLongitudeX1e7,
                  int32_t *pAltitudeMillimetres,
                  int32_t *pRadiusMillimetres,
                  int32_t *pAltitudeUncertaintyMillimetres,
                  int32_t *pSpeedMillimetresPerSecond,
                  int32_t *pSvs, int64_t *pTimeUtc, bool printIt) {
    int32_t errorCode = -1;
    int64_t t = -1;

    if ((pMessage[11] & 0x03) == 0x03) {
        int32_t year = (uUbxProtocolUint16Decode(pMessage + 4) - 1999) + 29;
        int32_t months = pMessage[6] - 1 + year * 12;

        t = uTimeMonthsToSecondsUtc(months);
        t += (pMessage[7] - 1) * 86400;
        t += pMessage[8] * 3600;
        t += pMessage[9] * 60;
        t += pMessage[10];

        if (printIt) printf("UTC time = %" PRId64 "\n", t);
    }

    if (pTimeUtc) *pTimeUtc = t;

    if ((t >= 0) && (pMessage[21] & 0x01)) {
        if (printIt) printf("%dD fix achieved\n", pMessage[20]);
        if (pSvs) *pSvs = pMessage[23];
        if (printIt) printf("satellite(s) = %d\n", pMessage[23]);

        if (pLongitudeX1e7) *pLongitudeX1e7 = uUbxProtocolUint32Decode(pMessage + 24);
        if (printIt) printf("longitude = %ld (degrees * 10^7)\n", *pLongitudeX1e7);

        if (pLatitudeX1e7) *pLatitudeX1e7 = uUbxProtocolUint32Decode(pMessage + 28);
        if (printIt) printf("latitude = %ld (degrees * 10^7)\n", *pLatitudeX1e7);

        int32_t altitude = INT_MIN;
        if (pMessage[20] == 0x03) altitude = uUbxProtocolUint32Decode(pMessage + 36);
        if (pAltitudeMillimetres) *pAltitudeMillimetres = altitude;
        if (printIt && altitude != INT_MIN) printf("altitude = %ld (mm)\n", altitude);

        if (pRadiusMillimetres) *pRadiusMillimetres = uUbxProtocolUint32Decode(pMessage + 40);
        if (printIt) printf("radius = %ld (mm)\n", *pRadiusMillimetres);

        if (pAltitudeUncertaintyMillimetres) *pAltitudeUncertaintyMillimetres = uUbxProtocolUint32Decode(pMessage + 44);
        if (printIt) printf("altitude uncertainty = %ld (mm)\n", *pAltitudeUncertaintyMillimetres);

        if (pSpeedMillimetresPerSecond) *pSpeedMillimetresPerSecond = uUbxProtocolUint32Decode(pMessage + 60);
        if (printIt) printf("speed = %ld (mm/s)\n", *pSpeedMillimetresPerSecond);

        errorCode = 0;
    }

    return errorCode;
}

// grab the position

void ubxPollLocation() {
    uint8_t pollLocationMsg[] = {
        0xB5, 0x62,
        0x01, 0x07,
        0x00, 0x00,
        0x08, 0x19
    };

    i2c_manager_write_register(i2c_port, SAM_M10Q_I2C_ADDR, 0xFF, pollLocationMsg, sizeof(pollLocationMsg));

    ubxFrame rollingFrame;

    vTaskDelay(100 / portTICK_PERIOD_MS);

    readUbxPort(&rollingFrame);

    int32_t latitude, longitude, altitude, radius, altitudeUncertainty, speed, svs;
    int64_t timeUtc;
    posDecode((char *)rollingFrame.payload,
              &latitude, &longitude,
              &altitude, &radius,
              &altitudeUncertainty, &speed,
              &svs, &timeUtc, true);
}