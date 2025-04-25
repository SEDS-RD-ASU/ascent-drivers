#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "driver_SAM_M10Q.h"

void gps_init() {
    printf("goober!");
}

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
    printf("Error code: %ld\n", errorCode);
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