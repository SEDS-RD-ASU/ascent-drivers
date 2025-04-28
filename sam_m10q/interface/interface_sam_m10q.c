#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "driver_SAM_M10Q.h"

int32_t readUBX(int32_t *pLatitudeX1e7, int32_t *pLongitudeX1e7,
                      int32_t *pAltitudeMillimetres,
                      int32_t *pRadiusMillimetres,
                      int32_t *pAltitudeUncertaintyMillimetres,
                      int32_t *pSpeedMillimetresPerSecond,
                      int32_t *pSvs, int64_t *pTimeUtc, bool printIt)
{
    int32_t errorCode = (int32_t) -9;
    char *messageBody = NULL;
    uGnssPrivateUbxReceiveMessage_t response = {
        .cls = 0,
        .id = 0,
        .ppBody = &messageBody,
        .bodySize = 0
    };

    // Request UBX-NAV-PVT message (class 0x01, id 0x07)
    errorCode = sendReceiveUbxMessage_I2C(0x01, 0x07, NULL, 0, &response);
    if (errorCode != 92) {
        printf("Error code: %ld\n", errorCode);
    }
    // Expected payload length for UBX-NAV-PVT is 92 bytes
    if (errorCode == 92) {
        // Decode the position data
        errorCode = posDecode(messageBody,
                              pLatitudeX1e7, pLongitudeX1e7,
                              pAltitudeMillimetres, pRadiusMillimetres,
                              pAltitudeUncertaintyMillimetres,
                              pSpeedMillimetresPerSecond,
                              pSvs, pTimeUtc, printIt);
    } else {
        // If we received a valid frame header but wrong length, signal error
        if (errorCode >= 0) {
            errorCode = (int32_t) -10;
        }
    }

    // Free the received buffer exactly once (ownership stays here)
    if (messageBody != NULL) {
        free(messageBody);
        messageBody = NULL;
    }

    return errorCode;
}

int32_t setRateUbxCfgRate(int32_t measurementPeriodMs,
                          int32_t navigationCount)
{
    int32_t errorCode = (int32_t) -9;
    printf("Initializing error code to -9\n");

    // Message buffer for the 6-byte UBX-CFG-RATE message
    char message[6] = {0};
    printf("Initializing message buffer for UBX-CFG-RATE\n");

    if (1) {
        int32_t timeSystem = 0;
        printf("Setting timeSystem to 0\n");

        // First two bytes are the measurement rate in milliseconds
        *((uint16_t *) & (message[0])) = uUbxProtocolUint16Encode((uint16_t) measurementPeriodMs); // *NOPAD*;
        printf("Encoding measurement rate in milliseconds\n");

        // Next two bytes are the navigation count
        *((uint16_t *) &(message[2])) = uUbxProtocolUint16Encode((uint16_t) navigationCount); // *NOPAD*;
        printf("Encoding navigation count\n");

        // Last two bytes are the time system
        *((uint16_t *) &(message[4])) = uUbxProtocolUint16Encode((uint16_t) timeSystem); // *NOPAD*;
        printf("Encoding time system\n");

        // Send UBX-CFG-RATE

        char ackBody[2] = {0};
        char *pBody = &(ackBody[0]);

        uGnssPrivateUbxReceiveMessage_t response = {0};

        response.cls = 0x05;
        response.id = -1;
        response.ppBody = &pBody;
        response.bodySize = sizeof(ackBody);


        printf("Sending UBX-CFG-RATE message\n");

        errorCode = sendReceiveUbxMessage_I2C(0x06, 0x08, message, sizeof(message), &response);
        printf("Received response for UBX-CFG-RATE\n");

        switch (response.id) {
            case 0x00:
                errorCode = (int32_t) -329; //nack
                printf("Response ID indicates NACK\n");
                break;
            case 0x01:
                errorCode = (int32_t) -330;  //success
                printf("Response ID indicates success\n");
                break;
            default:
                printf("Default case for response ID\n");
                break;
        }
    }

    printf("\nError code: %ld\n", errorCode);

    return errorCode;
}


void gps_init() {
    setRateUbxCfgRate(100,1);
}